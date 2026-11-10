"""Autonomously finish dense assets and bridge diagnostics after full tracking, not a pilot."""
import fcntl,subprocess,sys,time
from common import OUT

if __name__=='__main__':
 lock=(OUT/'finalization.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
 if (OUT/'validation.json').exists():sys.exit(0)
 start=time.monotonic()
 while not all((OUT/r/'tracking_metrics.json').exists() and (OUT/r/'depth_projection_complete.json').exists() for r in ['robot1','robot3']):
  if time.monotonic()-start>10800:raise TimeoutError('Full tracking did not complete; inspect native/projection logs')
  time.sleep(10)
 # Import/execute current committed stages only after the long tracking wait.
 for stage in ['audit','maps','fusion','report']:
  subprocess.run([sys.executable,str(__import__('pathlib').Path(__file__).with_name(stage+'.py'))],check=True)
