"""Read-only Stage32R search across files, archive members, Git objects and live processes."""
import argparse
import fnmatch
import gzip
import hashlib
import io
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import time
import zipfile
from pathlib import Path

EXPECTED={
    'Tracking.cc':'3a3654bcc675f859cce1f2fb28ab32a129e8d734d0ac3d55417ff8b47a013264',
    'Frame.cc':'67e0c446d9da80db11aceee4ff2664487cfe1f2502bc50d843ff7471917eb42e',
    'ORBextractor.cc':'c63ba40006b94d23cf93fd832fd7a48af835ddf087fb5dcc7caf9692b397f576',
    'Tracking.h':'9b20af086b2365edd60721fdab10683e69808717e5cabac21274faabf7989ec3',
    'System.cc':'11aa821b22ba90cadcc1e7290dda3c5eadcfa3cd83f36e30fb9f5c0c4378027c',
    'System.h':'7d2eb3734a7a81110274683ea36dc0ba1ce14204d3d5f081bf950601ff13fab3',
    'libORB_SLAM3.so':'ddb134dca4a630c702af5cf4d5acc0cd4d6159433e78a09884d27163605e8252',
    'rgbd_full':'4833b5c64656d62549f26c3494ccd5e96bf724e6acfb173ea0450525b9c1728f',
    'ORBvoc.txt':'f8dd027f7a6cb88129821341194d7f2c75b77b3394257ddd0d2229863d1a3570'}
KNOWN_SIZES={8521,58308,5245240}
BUILD_NAMES={'makefile','cmakelists.txt','cmakecache.txt','build.ninja','compile_commands.json','link.txt','flags.make'}
ARCHIVE_SUFFIXES=('.tar','.tar.gz','.tgz','.tar.xz','.tar.bz2','.zip','.7z','.rar')
SIGNATURE=re.compile(rb'ORB.?SLAM3|rgbd_full|fyp_mcd_stereo_inertial|System\.(?:cc|h)')
LIMIT=32*1024**2


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024**2),b''):
            digest.update(block)
    return digest.hexdigest()


def kind(name):
    lower=Path(name.replace('\\','/')).name.lower().lstrip('.')
    for key in EXPECTED:
        k=key.lower()
        if lower==k or lower.startswith(k+'.') or lower.startswith(k+'~'):
            return key
    if lower.startswith('system') and ('.cc' in lower or '.h' in lower):
        return 'System.h' if '.h' in lower else 'System.cc'
    if lower.startswith('tracking') and ('.cc' in lower or '.h' in lower):
        return 'Tracking.h' if '.h' in lower else 'Tracking.cc'
    if lower.startswith('rgbd_full'):
        return 'rgbd_full'
    if lower.startswith('sequence.txt'):
        return 'sequence.txt'
    if lower=='camera.yaml':
        return 'camera.yaml'
    if lower in BUILD_NAMES:
        return 'build_file'
    if lower.endswith(('.py','.sh')) and re.search(r'orb|fyp|stage|sdk|rgbd|patch|build',lower):
        return 'source_script'
    if lower.endswith(('.patch','.diff')):
        return 'patch'
    return None


def command(args,timeout=45,**kwargs):
    try:
        result=subprocess.run(args,capture_output=True,timeout=timeout,**kwargs)
        return dict(command=list(args),returncode=result.returncode,
                    stdout=result.stdout.decode('utf-8','replace'),stderr=result.stderr.decode('utf-8','replace'))
    except (OSError,subprocess.TimeoutExpired) as error:
        return dict(command=list(args),error=str(error))


class Search:
    def __init__(self,out,work):
        self.out=out
        self.work=work
        self.rows=[]
        self.errors=[]
        self.archives=[]
        self.repos=set()
        self.archive_results=[]
        self.git_results=[]
        self.counts=dict(directories=0,files=0,archives=0)
        self.last=time.monotonic()
        self.skipped_junctions=[]
        self.bundles=[]
        self.library_sizes=[]
        self.elf_candidates=[]

    def row(self,dependency,location,data=None,digest=None,size=None,origin='file'):
        expected=EXPECTED.get(dependency)
        if data is not None:
            digest=sha(data)
            size=len(data)
        if dependency in ('build_file','patch','source_script'):
            # Only inspect bounded source/config/patch content for project references.
            if data is None or not SIGNATURE.search(data):
                return
        classification=('RECOVERED_AND_HASH_VERIFIED' if expected and digest==expected else
                        'RECOVERED_BUT_HASH_MISMATCH' if expected else 'RECONSTRUCTED_NOT_ORIGINAL')
        row=dict(dependency=dependency,location=location,origin=origin,bytes=size,sha256=digest,
                 expected_sha256=expected,classification=classification)
        if dependency=='sequence.txt':
            row['note']='No historical sequence SHA receipt; file content and original provenance still require validation.'
        if dependency in ('System.cc','System.h','Tracking.cc','Tracking.h','Frame.cc','ORBextractor.cc') and data is not None:
            canonical=data.replace(b'\r\n',b'\n')
            row['lf_sha256']=sha(canonical)
            if expected and sha(canonical)==expected and digest!=expected:
                row['normalization_can_match_historical']=True
                row['normalization']='CRLF to LF in a separate recovered copy only'
        if classification=='RECOVERED_AND_HASH_VERIFIED':
            if data is not None:
                destination=self.work/'recovered'/dependency
                destination.parent.mkdir(parents=True,exist_ok=True)
                if not destination.exists():
                    destination.write_bytes(data)
                assert file_sha(destination)==expected
                row['recovered_copy']=str(destination)
        self.rows.append(row)

    def file(self,path):
        try:
            info=path.stat()
            self.counts['files']+=1
            name=path.name.lower()
            dependency=kind(path.name)
            if dependency:
                if dependency in ('build_file','patch','source_script') and info.st_size>LIMIT:
                    return
                if info.st_size<=LIMIT:
                    data=path.read_bytes()
                    self.row(dependency,str(path),data=data,size=info.st_size)
                else:
                    self.row(dependency,str(path),digest=file_sha(path),size=info.st_size)
            elif info.st_size in KNOWN_SIZES:
                data=path.read_bytes()
                for target,digest in EXPECTED.items():
                    if sha(data)==digest:
                        self.row(target,str(path),data=data,origin='size_and_hash_discovery')
            # ELF executables under user/project roots can survive under different names.
            elif os.name!='nt' and 16_000<=info.st_size<=1_000_000 and info.st_mode & stat.S_IXUSR:
                with path.open('rb') as stream:
                    elf=stream.read(4)==b'\x7fELF'
                if elf:
                    self.elf_candidates.append(path)
            if name.endswith('.bundle'):
                with path.open('rb') as stream:
                    if stream.read(32).startswith((b'# v2 git bundle',b'# v3 git bundle')):
                        self.bundles.append(path)
            if name.endswith(ARCHIVE_SUFFIXES):
                self.archives.append(path)
            if path.name=='.git' and path.is_file():
                self.repos.add(path.parent)
        except (OSError,ValueError) as error:
            self.errors.append(dict(path=str(path),error=str(error)))

    def walk(self,roots):
        virtual={'/proc','/sys','/dev','/run','/snap'}
        out=self.out.resolve()
        work=self.work.resolve()
        excluded_git=False
        for root in roots:
            root=Path(root)
            if not root.exists():
                self.errors.append(dict(path=str(root),error='ROOT_NOT_PRESENT'))
                continue
            for folder,dirs,files in os.walk(root,onerror=lambda e:self.errors.append(dict(path=e.filename,error=str(e))),followlinks=False):
                p=Path(folder)
                if os.name=='nt' and hasattr(os.path,'isjunction'):
                    links=[name for name in dirs if os.path.isjunction(p/name)]
                    self.skipped_junctions.extend(str(p/name) for name in links)
                    dirs[:]=[name for name in dirs if name not in links]
                self.counts['directories']+=1
                if p.resolve() in (out,work):
                    dirs[:]=[]
                    continue
                if os.name!='nt':
                    dirs[:]=[d for d in dirs if str(p/d) not in virtual]
                if 'HEAD' in files and 'objects' in dirs and 'config' in files:
                    self.repos.add(p)
                    dirs.remove('objects')
                if '.git' in dirs:
                    self.repos.add(p)
                    dirs.remove('.git')
                for name in files:
                    self.file(p/name)
                if time.monotonic()-self.last>20:
                    print('DISCOVERY',self.counts,'candidate files',len(self.rows),flush=True)
                    self.last=time.monotonic()
        for path in self.elf_candidates:
            try:
                if file_sha(path)==EXPECTED['rgbd_full']:
                    self.row('rgbd_full',str(path),data=path.read_bytes(),origin='renamed_ELF_discovery')
            except OSError as error:
                self.errors.append(dict(path=str(path),error=str(error)))

    def archive(self,path):
        result=dict(path=str(path),bytes=path.stat().st_size,candidate_members=0,matching_members=[])
        try:
            if path.name.lower().endswith('.zip'):
                with zipfile.ZipFile(path) as archive:
                    for member in archive.infolist():
                        dependency=kind(member.filename)
                        if not dependency or member.is_dir():
                            continue
                        result['candidate_members']+=1
                        if member.file_size>LIMIT and dependency not in ('libORB_SLAM3.so','rgbd_full','ORBvoc.txt'):
                            result.setdefault('skipped_oversize_members',[]).append(member.filename)
                            continue
                        with archive.open(member) as stream:
                            data=stream.read()
                        self.row(dependency,str(path)+'!'+member.filename,data=data,origin='zip_member_crc_checked')
                        if sha(data)==EXPECTED.get(dependency):
                            result['matching_members'].append(member.filename)
            elif path.name.lower().endswith(('.7z','.rar')):
                result['status']='UNSUPPORTED_ARCHIVE_FORMAT'
            else:
                with tarfile.open(path,'r:*') as archive:
                    for member in archive:
                        dependency=kind(member.name)
                        if not dependency or not member.isfile():
                            continue
                        result['candidate_members']+=1
                        if member.size>LIMIT and dependency not in ('libORB_SLAM3.so','rgbd_full','ORBvoc.txt'):
                            result.setdefault('skipped_oversize_members',[]).append(member.name)
                            continue
                        with archive.extractfile(member) as stream:
                            data=stream.read()
                        self.row(dependency,str(path)+'!'+member.name,data=data,origin='tar_member')
                        if sha(data)==EXPECTED.get(dependency):
                            result['matching_members'].append(member.name)
            result.setdefault('status','INSPECTED')
        except (OSError,tarfile.TarError,zipfile.BadZipFile,RuntimeError,EOFError) as error:
            result['status']='UNREADABLE'
            result['error']=str(error)
        self.archive_results.append(result)

    def git(self,repo):
        result=dict(repository=str(repo))
        base=['git','-c','safe.directory='+str(repo),'-C',str(repo)]
        refs=command(base+['rev-parse','HEAD'])
        result['head']=refs.get('stdout','').strip()
        result['worktrees']=command(base+['worktree','list','--porcelain'])
        all_paths=command(base+['rev-list','--all','--reflog','--objects'],timeout=120)
        if all_paths.get('returncode')!=0:
            result['object_discovery']=all_paths
            self.git_results.append(result)
            return
        candidates={}
        for line in all_paths['stdout'].splitlines():
            object_id,_,path=line.partition(' ')
            dependency=kind(Path(path).name) if path else None
            if dependency:
                candidates[object_id]=(dependency,path)
        result['named_historical_objects']=len(candidates)
        # Include unreachable and renamed blobs with the exact recorded source/library sizes.
        objects=command(base+['cat-file','--batch-all-objects','--batch-check=%(objectname) %(objecttype) %(objectsize)'],timeout=120)
        unnamed=[]
        for line in objects.get('stdout','').splitlines():
            parts=line.split()
            if len(parts)==3 and parts[1]=='blob' and int(parts[2]) in KNOWN_SIZES and parts[0] not in candidates:
                unnamed.append(parts[0])
        result['all_blob_size_candidates']=len(unnamed)
        for oid in unnamed:
            candidate=subprocess.run(base+['cat-file','blob',oid],capture_output=True)
            if candidate.returncode:
                continue
            for target,digest in EXPECTED.items():
                if sha(candidate.stdout)==digest:
                    self.row(target,str(repo)+'@'+oid,data=candidate.stdout,origin='git_size_matched_blob')
        for oid,(dependency,path) in candidates.items():
            info=command(base+['cat-file','-s',oid])
            size=int(info['stdout'].strip()) if info.get('returncode')==0 else LIMIT+1
            if size>LIMIT and dependency not in ('libORB_SLAM3.so','rgbd_full','ORBvoc.txt'):
                continue
            candidate=subprocess.run(base+['cat-file','blob',oid],capture_output=True)
            if candidate.returncode==0:
                self.row(dependency,str(repo)+'@'+oid+':'+path,data=candidate.stdout,origin='git_historical_blob')
        result['status']='INSPECTED'
        self.git_results.append(result)

    def bundle(self,path):
        destination=self.work/'git_bundle_copies'/hashlib.sha256(str(path).encode()).hexdigest()[:16]
        destination.parent.mkdir(parents=True,exist_ok=True)
        if not destination.exists():
            copied=command(['git','clone','--mirror','--quiet',str(path),str(destination)],timeout=120)
            if copied.get('returncode')!=0:
                self.git_results.append(dict(bundle=str(path),copy_result=copied))
                return
        self.git(destination)
        self.git_results[-1]['original_bundle']=str(path)

    def processes(self):
        result=[]
        if os.name=='nt':
            return result
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                executable=entry/'exe'
                target=os.readlink(executable)
                relevant='rgbd' in target.lower() or 'orb_slam' in target.lower()
                if relevant:
                    data=executable.read_bytes()
                    matched=sha(data)==EXPECTED['rgbd_full']
                    result.append(dict(pid=entry.name,target=target,sha256=sha(data),historical_match=matched))
                    if matched:
                        self.row('rgbd_full',str(executable),data=data,origin='live_process_executable')
                maps=entry/'maps'
                for line in maps.read_text().splitlines():
                    if 'libORB_SLAM3' in line:
                        result.append(dict(pid=entry.name,mapping=line))
                        interval=line.split()[0]
                        mapped=entry/'map_files'/interval
                        try:
                            with mapped.open('rb') as stream:
                                data=stream.read()
                            self.row('libORB_SLAM3.so',str(mapped),data=data,origin='live_mapped_library')
                        except OSError as error:
                            result.append(dict(pid=entry.name,mapping_error=str(error)))
            except (OSError,ValueError):
                continue
        return result

    def save(self,roots,processes):
        self.out.mkdir(parents=True,exist_ok=True)
        self.counts['archives']=len(self.archive_results)
        for name,data in (('search_candidates.json',self.rows),('archive_search.json',self.archive_results),
                          ('git_search.json',self.git_results),('live_process_search.json',processes)):
            (self.out/name).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
        (self.out/'search_coverage.json.gz').write_bytes(gzip.compress((json.dumps(dict(roots=roots,counts=self.counts,
            discovered_repositories=[str(p) for p in sorted(self.repos)],archives_discovered=len(self.archives),
            filesystem_errors=self.errors,skipped_windows_junctions=self.skipped_junctions,virtual_filesystems_excluded=['/proc','/sys','/dev','/run','/snap'],
            snapshot_and_process_search_separate=True,source_outputs_excluded_from_self_search=str(self.out)),indent=2)+'\n').encode('utf-8'),mtime=0))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--roots',nargs='+',required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--extra-repo',action='append',default=[])
    args=parser.parse_args()
    search=Search(args.out,args.work)
    search.walk(args.roots)
    search.repos.update(Path(p) for p in args.extra_repo)
    for path in sorted(set(search.archives)):
        search.archive(path)
        if time.monotonic()-search.last>20:
            print('ARCHIVE SEARCH',len(search.archive_results),'/',len(search.archives),flush=True)
            search.last=time.monotonic()
    for repo in sorted(search.repos):
        search.git(repo)
        print('GIT SEARCH',str(repo),flush=True)
    for bundle in sorted(set(search.bundles)):
        search.bundle(bundle)
        print('BUNDLE SEARCH',str(bundle),flush=True)
    processes=search.processes()
    search.save(args.roots,processes)
    print('SEARCH_COMPLETE',search.counts,'candidate records',len(search.rows),flush=True)


if __name__=='__main__':
    main()
