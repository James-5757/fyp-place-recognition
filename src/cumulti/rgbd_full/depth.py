"""Project every associated actual depth cloud; publish PNGs atomically for tracking."""
import csv,fcntl,json,sqlite3,sys,time
import cv2,numpy as np
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
from common import *

def run(robot):
 sys.addaudithook(guard);cv2.setNumThreads(1);out=OUT/robot
 lock=(out/'depth_projection.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
 if (out/'depth_projection_complete.json').exists():return
 assert (OUT/'audit_decision.json').exists()
 rgb=json.loads((out/'RGB_audit.json').read_text());dep=json.loads((out/'DEPTH_audit.json').read_text())
 rows=list(csv.DictReader((out/'sync_pairs.csv').open()));K=np.array(rgb['calibration']['K']).reshape(3,3);K[:2]*=.5;D=np.array(rgb['calibration']['D'])
 ext=dep['extra']['T_color_from_depth'];R=np.array(ext['R']);t=np.array(ext['t_m'])
 c=sqlite3.connect('file:'+dep['archive']['db_work_path']+'?mode=ro&immutable=1',uri=True);store=get_typestore(Stores.ROS2_HUMBLE)
 for typ,enc,definition in c.execute('select topic_type,encoding,encoded_message_definition from message_definitions'):
  if enc=='ros2msg':store.register(get_types_from_msg(definition,typ))
 folder=out/'depth';folder.mkdir(exist_ok=True);start=time.monotonic();coverage=[]
 for i,row in enumerate(rows):
  path=folder/f'{i:06d}.png'
  if row['paired']=='True':
   m=store.deserialize_cdr(c.execute('select data from messages where id=?',(int(row['depth_message_id']),)).fetchone()[0],dep['type']);fields={f.name:f for f in m.fields}
   assert all(fields[n].datatype==7 and fields[n].count==1 for n in ['x','y','z'])
   endian='>' if m.is_bigendian else '<';dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,offsets=[fields[n].offset for n in ['x','y','z']],itemsize=int(m.point_step)))
   a=np.ndarray((int(m.height),int(m.width)),dtype=dtype,buffer=m.data,strides=(int(m.row_step),int(m.point_step)))
   xyz=np.stack([a[n].ravel() for n in ['x','y','z']],1);z=align_depth(xyz,R,t,K,D)
  else:z=np.zeros((400,640),np.float32)
  image=np.rint(z*1000).astype(np.uint16);tmp=path.with_name(path.stem+'.tmp.png');assert cv2.imwrite(str(tmp),image);tmp.replace(path)
  coverage.append(dict(index=i,valid_depth_fraction=float((image>0).mean()),paired=row['paired']))
  if i%100==0:dump(out/'depth_projection_progress.json',dict(frames=i+1,total=len(rows),elapsed_s=time.monotonic()-start))
 writecsv(out/'depth_coverage.csv',coverage);dump(out/'depth_projection_complete.json',dict(frames=len(rows),elapsed_s=time.monotonic()-start,coverage=stats([r['valid_depth_fraction'] for r in coverage])))

if __name__=='__main__':run(sys.argv[1])
