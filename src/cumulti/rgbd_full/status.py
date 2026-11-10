"""Compact read-only progress for the independent full-duration experiment."""
import csv,json
from common import OUT

if __name__=='__main__':
 result={}
 for robot in ['robot1','robot3']:
  out=OUT/robot;data={}
  for name in ['RGB_progress','DEPTH_progress','depth_projection_progress','tracking_metrics']:
   p=out/(name+'.json')
   if p.exists():data[name]=json.loads(p.read_text())
  p=out/'frame_poses.csv'
  if p.exists():
   rows=list(csv.DictReader(p.open()))
   if rows and rows[-1].get('relative_ns'):
    data['native_frames_written']=len(rows);data['native_recorded_seconds']=int(rows[-1]['relative_ns'])/1e9
    a=out/'RGB_audit.json'
    if a.exists():data['remaining_recorded_s']=max(0,json.loads(a.read_text())['duration_s']-data['native_recorded_seconds'])
  result[robot]=data
 for key in ['audit_decision','dense_mapping_complete','validation']:
  p=OUT/(key+'.json')
  if p.exists():result[key]=json.loads(p.read_text()).get('status','COMPLETE')
 print(json.dumps(result,indent=2))
