"""Watch newly expanded camera DBs and audit every recorded calibration message."""
import argparse,json,sqlite3,time
from pathlib import Path
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
from common import OUT,dump

def run(robot,kind):
 out=OUT/robot;receipt=out/(kind+'_expansion_receipt.json')
 if (out/(kind+'_calibration_audit.json')).exists():return
 while not receipt.exists():time.sleep(2)
 r=json.loads(receipt.read_text());db=Path(r['db_work_path'])
 c=sqlite3.connect(f'file:{db}?mode=ro&immutable=1',uri=True)
 store=get_typestore(Stores.ROS2_HUMBLE)
 for typ,enc,definition in c.execute('select topic_type,encoding,encoded_message_definition from message_definitions'):
  if enc=='ros2msg':store.register(get_types_from_msg(definition,typ))
 results=[]
 for tid,name,typ in c.execute('select id,name,type from topics').fetchall():
  if not (name.endswith('camera_info') or name.endswith('depth_to_color')):continue
  variants={};count=0
  for timestamp,data in c.execute('select timestamp,data from messages where topic_id=? order by timestamp,id',(tid,)):
   m=store.deserialize_cdr(data,typ)
   if name.endswith('camera_info'):
    v=dict(K=m.k.tolist(),D=m.d.tolist(),R=m.r.tolist(),P=m.p.tolist(),width=int(m.width),height=int(m.height),distortion_model=m.distortion_model,frame_id=m.header.frame_id)
   else:v=dict(rotation=m.rotation.tolist(),translation=m.translation.tolist(),frame_id=m.header.frame_id)
   key=json.dumps(v,sort_keys=True)
   if key not in variants:variants[key]=dict(value=v,count=0,first_bag_ns=str(timestamp))
   variants[key]['count']+=1;variants[key]['last_bag_ns']=str(timestamp);count+=1
  results.append(dict(topic=name,messages=count,distinct_values=len(variants),variants=list(variants.values())))
 dump(out/(kind+'_calibration_audit.json'),dict(robot=robot,kind=kind,topics=results,constant=all(t['distinct_values']==1 for t in results)))
 print('FULL_CALIBRATION_AUDIT_COMPLETE',robot,kind,flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('robot');p.add_argument('kind');a=p.parse_args();run(a.robot,a.kind)
