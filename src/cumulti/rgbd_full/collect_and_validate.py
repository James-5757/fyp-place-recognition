"""Windows-side collection and actual browser acceptance after the server finishes."""
import json,subprocess,sys,tarfile,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
REMOTE='/home/cas/fyp_place_recognition'
OUT=ROOT/'outputs/cumulti_v1/24_rgbd_full_v1'

def ssh(command):
 return subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','fyp-server-codex',command],capture_output=True,text=True,timeout=120)

def run():
 OUT.mkdir(parents=True,exist_ok=True);start=time.monotonic()
 while True:
  result=ssh(f'test -f {REMOTE}/outputs/cumulti_v1/24_rgbd_full_v1/validation.json')
  if result.returncode==0:break
  if time.monotonic()-start>10800:raise TimeoutError('Server full experiment did not finish within three hours')
  print('WAITING_FOR_COMPLETE_FULL_STREAM_RESULTS',round(time.monotonic()-start),flush=True);time.sleep(45)
 result=ssh(f'cd {REMOTE} && tar -cf /tmp/fyp_stage24_full_presentation.tar demo/stage24_full_rgbd')
 assert result.returncode==0,result.stderr
 bundle=Path(tempfile.gettempdir())/'fyp_stage24_full_presentation.tar'
 subprocess.run(['scp','fyp-server-codex:/tmp/fyp_stage24_full_presentation.tar',str(bundle)],check=True)
 with tarfile.open(bundle) as archive:
  assert all(Path(m.name).parts[:2]==('demo','stage24_full_rgbd') for m in archive.getmembers())
  archive.extractall(ROOT,filter='data')
 from validate_browser import run as validate
 result=validate('http://127.0.0.1:8026/stage24_full_rgbd/',OUT/'browser_validation')
 assert result['status']=='PASS',result
 subprocess.run(['scp','-r',str(OUT/'browser_validation'),f'fyp-server-codex:{REMOTE}/outputs/cumulti_v1/24_rgbd_full_v1/'],check=True)
 result=ssh(f'cd {REMOTE} && /home/cas/.venvs/fyp_stage24/bin/python src/cumulti/rgbd_full/report.py')
 assert result.returncode==0,result.stderr
 (OUT/'collection_complete.json').write_text(json.dumps(dict(browser='PASS',server_report=result.stdout,wall_s=time.monotonic()-start),indent=2)+'\n')
 print('FULL_DEMO_COLLECTED_AND_BROWSER_VALIDATED',result.stdout,flush=True)

if __name__=='__main__':run()
