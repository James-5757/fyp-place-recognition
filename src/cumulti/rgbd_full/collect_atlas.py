"""Collect only the Atlas browser assets and validate the real HTTP replay."""
import json
import subprocess
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
        result = subprocess.run(args, timeout=180)
        if result.returncode == 0:
            return
        time.sleep(10)
    raise TimeoutError(args)


def main():
    deadline = time.monotonic() + 7200
    ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','fyp-server-codex']
    while time.monotonic() < deadline:
        result = subprocess.run(ssh + [f'test -f {REMOTE}/{SITE}/manifest.json'], timeout=20)
        if result.returncode == 0:
            break
        time.sleep(30)
    else:
        raise TimeoutError('Atlas assets did not become available')
    remote_bundle = '/tmp/fyp_stage24_atlas_browser.tar'
    retry(ssh + [f'cd {REMOTE} && tar -cf {remote_bundle} {SITE}'], deadline=deadline)
    bundle = Path(tempfile.gettempdir())/'fyp_stage24_atlas_browser.tar'
    retry(['scp','fyp-server-codex:'+remote_bundle,str(bundle)],deadline=deadline)
    with tarfile.open(bundle) as archive:
        assert all(Path(member.name).parts[:3] == ('demo','stage24_full_rgbd','atlas')
                   for member in archive.getmembers())
        archive.extractall(ROOT,filter='data')
    report = validate('http://127.0.0.1:8026/stage24_full_rgbd/?atlas=1', OUT/'browser_validation')
    assert report['status'] == 'PASS'
    (OUT/'browser_complete.json').write_text(json.dumps({'status':'PASS','seek_cases':report['seek_cases']})+'\n',
                                             encoding='utf-8',newline='\n')
    retry(['scp','-r',str(OUT/'browser_validation'),
           f'fyp-server-codex:{REMOTE}/outputs/cumulti_v1/24_rgbd_atlas_v1/'],deadline=deadline)


if __name__ == '__main__':
    main()
