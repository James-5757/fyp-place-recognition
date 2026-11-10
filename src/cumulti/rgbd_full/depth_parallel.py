"""Parallelize only independent deterministic depth projection, never camera tracking."""
import csv,fcntl,json,os,resource,sqlite3,sys,time
from concurrent.futures import ProcessPoolExecutor
import cv2,numpy as np
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
from common import *

def initialize(robot):
 global OUT_ROBOT,CONNECTION,STORE,K,D,R,T,TYPE
 cv2.setNumThreads(1);sys.addaudithook(guard);OUT_ROBOT=OUT/robot
 rgb=json.loads((OUT_ROBOT/'RGB_audit.json').read_text());dep=json.loads((OUT_ROBOT/'DEPTH_audit.json').read_text());K=np.array(rgb['calibration']['K']).reshape(3,3);K[:2]*=.5;D=np.array(rgb['calibration']['D']);ext=dep['extra']['T_color_from_depth'];R=np.array(ext['R']);T=np.array(ext['t_m']);TYPE=dep['type']
 CONNECTION=sqlite3.connect('file:'+dep['archive']['db_work_path']+'?mode=ro&immutable=1',uri=True);STORE=get_typestore(Stores.ROS2_HUMBLE)
 for typ,enc,definition in CONNECTION.execute('select topic_type,encoding,encoded_message_definition from message_definitions'):
  if enc=='ros2msg':STORE.register(get_types_from_msg(definition,typ))

def project(job):
 i,row=job;path=OUT_ROBOT/'depth'/f'{i:06d}.png';reused=path.exists()
 if reused:
  image=cv2.imread(str(path),cv2.IMREAD_UNCHANGED);assert image is not None and image.shape==(400,640) and image.dtype==np.uint16
 else:
  if row['paired']=='True':
   m=STORE.deserialize_cdr(CONNECTION.execute('select data from messages where id=?',(int(row['depth_message_id']),)).fetchone()[0],TYPE);fields={f.name:f for f in m.fields};assert all(fields[n].datatype==7 and fields[n].count==1 for n in ['x','y','z'])
   endian='>' if m.is_bigendian else '<';dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,offsets=[fields[n].offset for n in ['x','y','z']],itemsize=int(m.point_step)));a=np.ndarray((int(m.height),int(m.width)),dtype=dtype,buffer=m.data,strides=(int(m.row_step),int(m.point_step)));xyz=np.stack([a[n].ravel() for n in ['x','y','z']],1)
   z=align_depth(xyz,R,T,K,D)
  else:z=np.zeros((400,640),np.float32)
  image=np.rint(z*1000).astype(np.uint16);tmp=path.with_name(path.stem+f'.{os.getpid()}.tmp.png');assert cv2.imwrite(str(tmp),image);tmp.replace(path)
 return dict(index=i,valid_depth_fraction=float((image>0).mean()),paired=row['paired'],reused_completed_png=reused,worker_peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)

def run(robot):
 out=OUT/robot;lock=(out/'depth_projection.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
 if (out/'depth_projection_complete.json').exists():return
 assert (OUT/'audit_decision.json').exists();(out/'depth').mkdir(exist_ok=True);rows=list(csv.DictReader((out/'sync_pairs.csv').open()));coverage=[];start=time.monotonic()
 with ProcessPoolExecutor(max_workers=4,initializer=initialize,initargs=(robot,)) as pool:
  for result in pool.map(project,enumerate(rows),chunksize=4):
   coverage.append(result)
   if len(coverage)%100==0:dump(out/'depth_projection_progress.json',dict(frames=len(coverage),total=len(rows),elapsed_s=time.monotonic()-start,projection_workers=4))
 writecsv(out/'depth_coverage.csv',coverage);dump(out/'depth_projection_complete.json',dict(frames=len(rows),elapsed_s=time.monotonic()-start,coverage=stats([r['valid_depth_fraction'] for r in coverage]),workers=4,reused_completed_png=sum(r['reused_completed_png'] for r in coverage),max_worker_peak_RSS_MiB=max(r['worker_peak_RSS_MiB'] for r in coverage),algorithm='same frozen projection, independent frames, atomic PNGs; no tracking process restart'))

if __name__=='__main__':run(sys.argv[1])
