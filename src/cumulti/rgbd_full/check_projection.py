"""Check independent projection against already consumed sequential PNGs."""
import csv,json
import depth_parallel as parallel
from common import *

if __name__=='__main__':
 checks=[]
 for robot in ['robot1','robot3']:
  rows=list(csv.DictReader((OUT/robot/'sync_pairs.csv').open()));parallel.initialize(robot)
  test=OUT/'projection_parity'/robot;(test/'depth').mkdir(parents=True,exist_ok=True);parallel.OUT_ROBOT=test
  indices=[i for i,r in enumerate(rows) if r['paired']=='True' and (OUT/robot/'depth'/f'{i:06d}.png').exists()][::50][:3]
  assert len(indices)==3
  for i in indices:
   parallel.project((i,rows[i]));expected=OUT/robot/'depth'/f'{i:06d}.png';actual=test/'depth'/f'{i:06d}.png';checks.append(dict(robot=robot,index=i,sha256=sha(actual),matches=sha(actual)==sha(expected)))
 assert all(c['matches'] for c in checks)
 dump(OUT/'projection_parallel_parity.json',dict(status='PASS',checks=checks,tracking_restarted=False))
 print('PROJECTION_PARITY_PASS',len(checks),flush=True)
