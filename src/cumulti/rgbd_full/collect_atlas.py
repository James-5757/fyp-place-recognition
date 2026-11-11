"""Collect only the Atlas browser assets and validate the real HTTP replay."""
import json
import subprocess
import sys
import tarfile
import tempfile
import time
from pathlib import Path

from validate_browser import run as validate

ROOT = Path(__file__).resolve().parents[3]
REMOTE = '/home/cas/fyp_place_recognition'
OUT = ROOT/'outputs/cumulti_v1/24_rgbd_atlas_v1'
SITE = 'demo/stage24_full_rgbd/atlas'


def retry(args, *, deadline):
    while time.monotonic() < deadline:
        try:
            if subprocess.run(args, timeout=180).returncode == 0:
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(10)
    raise TimeoutError(args)


def main():
    stage24d = '--stage24d' in sys.argv
    site = 'demo/stage24_full_rgbd/stage24d' if stage24d else SITE
    out = ROOT/'outputs/cumulti_v1/24d_rgbd_pose_graph' if stage24d else OUT
    remote_out = out.relative_to(ROOT).as_posix()
    deadline = time.monotonic() + 7200
    ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','fyp-server-codex']
    while time.monotonic() < deadline:
        try:
            if subprocess.run(ssh + [f'test -f {REMOTE}/{site}/manifest.json'], timeout=20).returncode == 0:
                break
        except subprocess.TimeoutExpired:
            pass
        time.sleep(30)
    else:
        raise TimeoutError('Atlas assets did not become available')
    bundle_name = f'fyp_stage24_{Path(site).name}_browser.tar'
    remote_bundle = f'/tmp/{bundle_name}'
    retry(ssh + [f'cd {REMOTE} && tar -cf {remote_bundle} {site}'], deadline=deadline)
    bundle = Path(tempfile.gettempdir())/bundle_name
    retry(['scp','fyp-server-codex:'+remote_bundle,str(bundle)],deadline=deadline)
    with tarfile.open(bundle) as archive:
        assert all(Path(member.name).parts[:3] == Path(site).parts
                   for member in archive.getmembers())
        archive.extractall(ROOT,filter='data')
    url = 'http://127.0.0.1:8026/stage24_full_rgbd/' + ('' if stage24d else '?atlas=1')
    report = validate(url, out/'browser_validation')
    assert report['status'] == 'PASS'
    (out/'browser_complete.json').write_text(json.dumps({'status':'PASS','seek_cases':report['seek_cases']})+'\n',
                                             encoding='utf-8',newline='\n')
    retry(['scp','-r',str(out/'browser_validation'),
           f'fyp-server-codex:{REMOTE}/{remote_out}/'],deadline=deadline)


if __name__ == '__main__':
    main()
