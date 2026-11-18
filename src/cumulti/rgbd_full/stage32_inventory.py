"""Record native dependency and project-backup discovery without extracting archives."""
import argparse
import json
import subprocess
import tarfile
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    names=['Tracking.cc','System.cc','System.h','libORB_SLAM3.so','ORBvoc.txt','rgbd_full','camera.yaml','sequence.txt']
    command=['find','/home/cas','/tmp','/dev/shm','/mnt','/opt','/media','-type','f','(']
    for i,name in enumerate(names):
        if i:
            command.append('-o')
        command.extend(['-name',name])
    command.append(')')
    found=subprocess.run(command,capture_output=True,text=True)
    archives=[]
    for path in sorted(Path('/tmp').glob('fyp*.tar*')):
        try:
            with tarfile.open(path) as archive:
                items=[member.name for member in archive if
                       Path(member.name).name in names or member.name.endswith('/Optimizer.cc')]
            archives.append(dict(path=str(path),matching_members=items))
        except tarfile.TarError as error:
            archives.append(dict(path=str(path),error=str(error)))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(search_command=command,search_returncode=found.returncode,
        matching_files=found.stdout.splitlines(),search_error_lines=found.stderr.splitlines(),
        backup_archive_inventory=archives,source_reference_is_new_recovery=True),indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
