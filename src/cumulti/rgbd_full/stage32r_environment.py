"""Capture Stage32R native build and snapshot accessibility evidence without installation."""
import argparse
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path


def run(args,input_text=None,timeout=20):
    try:
        p=subprocess.run(args,input=input_text,text=True,capture_output=True,timeout=timeout)
        return dict(command=args,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr)
    except (OSError,subprocess.TimeoutExpired) as error:
        return dict(command=args,error=str(error))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result=dict(platform=platform.platform(),uid=os.getuid() if hasattr(os,'getuid') else None,
                packages_installed=[],tracking_parameters_changed=[],instrumented_build_started=False)
    commands={
        'compiler_version':['g++','--version'],
        'compiler_target':['g++','-dumpmachine'],
        'cmake_version':['cmake','--version'],
        'native_opencv_version':['pkg-config','--modversion','opencv4'],
        'native_opencv_link_flags':['pkg-config','--libs','opencv4'],
        'native_opencv_compile_flags':['pkg-config','--cflags','opencv4'],
        'pangolin_pkg_config':['pkg-config','--modversion','pangolin'],
        'glibc_version':['getconf','GNU_LIBC_VERSION'],
        'loader_version':['ldd','--version'],
        'mounts':['findmnt','-rn','-o','TARGET,SOURCE,FSTYPE'],
        'docker_access':['docker','ps','-a','--format','{{.ID}} {{.Names}} {{.Image}}'],
        'lxd_access':['lxc','list','--format','csv'],
        'snap_saved':['snap','saved'],
        'open_deleted_runtime':['lsof','-nP','+L1']}
    result['commands']={key:run(cmd) for key,cmd in commands.items()}
    # Do not expose unrelated process records in the project artifact.
    deleted=result['commands']['open_deleted_runtime']
    deleted['stdout']='\n'.join(line for line in deleted.get('stdout','').splitlines()
                                if any(token in line for token in ('ORB_SLAM3','rgbd_full','fyp_mcd')))
    ld=run(['ldconfig','-p'])
    result['relevant_shared_libraries']=[line.strip() for line in ld.get('stdout','').splitlines()
                                        if any(k in line.lower() for k in ('orb_slam','pangolin','opencv_core',
                                                                         'stdc++','boost_serialization','g2o','dbow'))]
    macros=run(['g++','-dM','-E','-x','c++','-'],input_text='#include <string>\n')
    result['current_compiler_abi_macros']=[line for line in macros.get('stdout','').splitlines()
                                         if any(k in line for k in ('_GLIBCXX_USE_CXX11_ABI ','__GLIBCXX__ ','__GXX_ABI_VERSION '))]
    result['historical_compiler_abi']='UNVERIFIED_WITHOUT_MATCHED_ORIGINAL_ELF_OR_BUILD_RECEIPT'
    locations=['/root','/var/lib/docker','/var/run/docker.sock','/var/backups',
               '/.snapshots','/timeshift/snapshots','/var/lib/snapd/snapshots','/mnt','/media']
    result['snapshot_and_backup_access']=[dict(path=path,exists=Path(path).exists(),
                                              readable=os.access(path,os.R_OK),
                                              traversable=os.access(path,os.X_OK)) for path in locations]
    result['guest_snapshot_scope']='Mounted and accessible guest paths only; provider-side snapshots were not available through this account.'
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
