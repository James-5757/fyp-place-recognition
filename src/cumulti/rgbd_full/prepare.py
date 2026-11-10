"""Phase1: complete ZIP/SQLite topic/header audits, with reusable camera input cache."""
import argparse,fcntl,json,os,shutil,sqlite3,struct,sys,time,zipfile
from pathlib import Path
import cv2,numpy as np,yaml
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
from common import *

def header(payload):
 assert bytes(payload[:4]) in [b'\x00\x01\x00\x00',b'\x00\x00\x00\x00']
 sec,nsec=struct.unpack_from('<iI' if payload[1]==1 else '>iI',payload,4);assert nsec<10**9
 return sec*10**9+nsec

def expand(robot,kind,out,archive):
 # One RAM-backed RGB work copy avoids exceeding the disk budget for four DBs.
 if robot=='robot3' and kind=='RGB':
  work=Path('/dev/shm/fyp_rgbd_full_v1');work.mkdir(mode=0o700,exist_ok=True)
  assert work.stat().st_uid==os.getuid();db=work/'robot3_RGB.db3'
 else:work=out/'work';work.mkdir(exist_ok=True);db=work/(kind+'.db3')
 receipt=out/(kind+'_expansion_receipt.json');before=archive.stat();start=time.monotonic()
 with zipfile.ZipFile(archive) as z:
  member=next(n for n in z.namelist() if n.endswith('.db3'));info=z.getinfo(member)
  if receipt.exists() and db.exists() and db.stat().st_size==info.file_size:
   r=json.loads(receipt.read_text());assert r['archive_bytes']==before.st_size and r['archive_mtime_ns']==before.st_mtime_ns
   return db,r
  reclaim=db.stat().st_size if db.exists() else 0
  assert shutil.disk_usage(work).free+reclaim>info.file_size+8*1024**3,('scratch space insufficient',str(work))
  count=0;last=time.monotonic()
  with z.open(member) as src,db.open('wb') as target:
   while True:
    chunk=src.read(32*1024**2)
    if not chunk:break
    target.write(chunk);count+=len(chunk)
    if time.monotonic()-last>10:
     dump(out/(kind+'_progress.json'),dict(phase='EXPAND',bytes=count,total=info.file_size,elapsed_s=time.monotonic()-start));last=time.monotonic()
  assert archive.stat().st_size==before.st_size and archive.stat().st_mtime_ns==before.st_mtime_ns
  r=dict(archive=str(archive),archive_bytes=before.st_size,archive_mtime_ns=before.st_mtime_ns,member=member,db_bytes=info.file_size,
    CRC32=f'{info.CRC:08x}',CRC_validated_at_EOF=True,db_work_path=str(db),temporary_storage='tmpfs' if str(db).startswith('/dev/shm/') else 'disk',elapsed_s=time.monotonic()-start)
  dump(receipt,r);return db,r

def run(robot,kind):
 sys.addaudithook(guard);cv2.setNumThreads(1);out=OUT/robot;out.mkdir(parents=True,exist_ok=True)
 lock=(out/(kind+'.lock')).open('w');fcntl.flock(lock,fcntl.LOCK_EX)
 result=out/(kind+'_audit.json')
 if result.exists():print('AUDIT_REUSED',robot,kind,flush=True);return
 archive=RAW/robot/f'{robot}_main_campus_camera_{"rgb" if kind=="RGB" else "depth"}.zip'
 db,receipt=expand(robot,kind,out,archive);store=get_typestore(Stores.ROS2_HUMBLE)
 c=sqlite3.connect(f'file:{db}?mode=ro&immutable=1',uri=True)
 definitions=c.execute('select topic_type,encoding,encoded_message_definition from message_definitions').fetchall()
 for typ,enc,definition in definitions:
  if enc=='ros2msg':store.register(get_types_from_msg(definition,typ))
 topics=c.execute('select id,name,type from topics').fetchall();suffix='/color/image_raw' if kind=='RGB' else '/depth/color/points'
 tid,topic,typ=next(t for t in topics if t[1].endswith(suffix))
 # Header is the actual first field; validate prefix parsing against full CDR.
 example=c.execute('select data from messages where topic_id=? order by id limit 1',(tid,)).fetchone()[0]
 m=store.deserialize_cdr(example,typ);assert header(example)==int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
 start=time.monotonic();index=c.execute('select id,timestamp,substr(data,1,12) from messages where topic_id=? order by timestamp,id',(tid,)).fetchall()
 rows=[dict(index=i,message_id=mid,bag_ns=str(t),header_ns=str(header(blob))) for i,(mid,t,blob) in enumerate(index)]
 infoid,infoname,infotype=next(t for t in topics if t[1].endswith('/color/camera_info' if kind=='RGB' else '/depth/camera_info'))
 ci=store.deserialize_cdr(c.execute('select data from messages where topic_id=? order by id limit 1',(infoid,)).fetchone()[0],infotype)
 calibration=dict(topic=infoname,K=ci.k.tolist(),D=ci.d.tolist(),width=int(ci.width),height=int(ci.height),distortion_model=ci.distortion_model,frame_id=ci.header.frame_id)
 extra={}
 if kind=='RGB':
  assert m.encoding in ['rgb8','bgr8'];extra['image_encoding']=m.encoding;folder=out/'rgb';folder.mkdir(exist_ok=True)
  for i,row in enumerate(rows):
   image=store.deserialize_cdr(c.execute('select data from messages where id=?',(row['message_id'],)).fetchone()[0],typ)
   assert header_ns(image)==int(row['header_ns'])
   a=np.frombuffer(image.data,np.uint8).reshape(int(image.height),int(image.step))[:,:int(image.width)*3].reshape(int(image.height),int(image.width),3)
   if image.encoding=='rgb8':a=a[:,:,::-1]
   a=cv2.resize(a,(640,400),interpolation=cv2.INTER_AREA);p=folder/f'{i:06d}.jpg';assert cv2.imwrite(str(p),a,[cv2.IMWRITE_JPEG_QUALITY,85])
   row['rgb']=str(p.relative_to(ROOT))
   if i%250==0:dump(out/'RGB_progress.json',dict(phase='EXPORT_REAL_RGB',frames=i+1,total=len(rows)))
 else:
  eid,ename,etype=next(t for t in topics if t[1].endswith('depth_to_color'))
  ext=store.deserialize_cdr(c.execute('select data from messages where topic_id=? order by id limit 1',(eid,)).fetchone()[0],etype)
  R=ext.rotation.reshape(3,3,order='F');assert abs(np.linalg.det(R)-1)<1e-6 and np.max(abs(R.T@R-np.eye(3)))<1e-6
  extra['T_color_from_depth']=dict(R=R.tolist(),t_m=ext.translation.tolist(),storage='column-major actual RealSense message')
  extra['point_fields']=[dict(name=f.name,offset=int(f.offset),datatype=int(f.datatype),count=int(f.count)) for f in m.fields]
  extra['point_frame']=m.header.frame_id;extra['native_depth_Image_topic']=any(t[2]=='sensor_msgs/msg/Image' for t in topics)
  assert m.header.frame_id==ci.header.frame_id
 with zipfile.ZipFile(archive) as z:
  meta=yaml.safe_load(z.read(next(n for n in z.namelist() if n.endswith('metadata.yaml'))))['rosbag2_bagfile_information']
 expected=next(t['message_count'] for t in meta['topics_with_message_count'] if t['topic_metadata']['name']==topic);assert len(rows)==expected
 values=np.array([int(r['bag_ns']) for r in rows],np.int64);heads=np.array([int(r['header_ns']) for r in rows],np.int64)
 writecsv(out/(kind+'_timestamps.csv'),rows)
 audit=dict(robot=robot,kind=kind,topic=topic,type=typ,topics=topics,frames=len(rows),metadata_frames=expected,archive=receipt,calibration=calibration,
   first_bag_ns=str(values[0]),last_bag_ns=str(values[-1]),first_header_ns=str(heads[0]),last_header_ns=str(heads[-1]),duration_s=float((values[-1]-values[0])/1e9),
   median_frequency_hz=float(1e9/np.median(np.diff(values))),overall_frequency_hz=float((len(rows)-1)*1e9/(values[-1]-values[0])),bag_gap_ms=stats(np.diff(values)/1e6),
   bag_gaps_over_250ms=int(np.sum(np.diff(values)>250000000)),header_nonincreasing=int(np.sum(np.diff(heads)<=0)),header_gap_ms=stats(np.diff(heads)/1e6),
   index_complete=True,header_parser_validated=True,rows=rows,extra=extra,GT=False,LiDAR=False,IMU=False)
 c.close();dump(result,audit)
 if kind=='RGB':db.unlink() # Exact new scratch file; original ZIP never written.
 dump(out/(kind+'_progress.json'),dict(phase='AUDITED',frames=len(rows),duration_s=audit['duration_s'],depth_DB_retained_for_phase2=kind=='DEPTH'))
 print('FULL_AUDIT_COMPLETE',robot,kind,len(rows),audit['duration_s'],flush=True)

def header_ns(m):return int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('robot',choices=['robot1','robot3']);p.add_argument('kind',choices=['RGB','DEPTH']);a=p.parse_args();run(a.robot,a.kind)
