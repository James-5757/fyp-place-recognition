"""Full native-rate camera-only tracking; never truncate to a pilot window."""
import csv,fcntl,json,os,re,subprocess,sys,time
import numpy as np
from common import *
ORB=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')

def run(robot):
 sys.addaudithook(guard);out=OUT/robot
 lock=(out/'tracking.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
 if (out/'tracking_metrics.json').exists():return
 assert json.loads((OUT/'audit_decision.json').read_text())['status']=='GO_COMPLETE_AVAILABLE_RECORDINGS'
 data=json.loads((out/'RGB_audit.json').read_text());K=np.array(data['calibration']['K']).reshape(3,3);K[:2]*=.5;D=data['calibration']['D']
 settings={'Camera1.fx':K[0,0],'Camera1.fy':K[1,1],'Camera1.cx':K[0,2],'Camera1.cy':K[1,2],
  'Camera1.k1':D[0],'Camera1.k2':D[1],'Camera1.p1':D[2],'Camera1.p2':D[3],'Camera1.k3':D[4],
  'Camera.width':640,'Camera.height':400,'Camera.fps':10,'Camera.RGB':0,'Stereo.b':.05,'Stereo.ThDepth':40.,'RGBD.DepthMapFactor':1000.,
  'ORBextractor.nFeatures':1000,'ORBextractor.scaleFactor':1.2,'ORBextractor.nLevels':8,'ORBextractor.iniThFAST':20,'ORBextractor.minThFAST':7,
  'Viewer.KeyFrameSize':.05,'Viewer.KeyFrameLineWidth':1.,'Viewer.GraphLineWidth':.9,'Viewer.PointSize':2.,'Viewer.CameraSize':.08,'Viewer.CameraLineWidth':3.,
  'Viewer.ViewpointX':0.,'Viewer.ViewpointY':-.7,'Viewer.ViewpointZ':-1.8,'Viewer.ViewpointF':500.}
 cfg=out/'camera.yaml';cfg.write_text('%YAML:1.0\nFile.version: "1.0"\nCamera.type: "PinHole"\n'+''.join(f'{k}: {v}\n' for k,v in settings.items()))
 rows=list(csv.DictReader((out/'sync_pairs.csv').open()));seq=out/'sequence.txt'
 seq.write_text(''.join(f"{r['relative_ns']} {ROOT/r['rgb']} {out/'depth'/f'{i:06d}.png'}\n" for i,r in enumerate(rows)))
 while not (out/'depth'/f'{min(99,len(rows)-1):06d}.png').exists():time.sleep(2)
 env=dict(os.environ,LC_ALL='C',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=':'.join(map(str,[ORB/'lib',ORB.parent/'pangolin-install/lib',ORB/'Thirdparty/DBoW2/lib',ORB/'Thirdparty/g2o/lib'])))
 command=['strace','-f','-e','trace=open,openat','-o',str(out/'native_access.log'),str(OUT/'bin/rgbd_full'),str(ORB/'Vocabulary/ORBvoc.txt'),str(cfg),str(seq),str(out/'frame_poses.csv'),str(out/'unused_corrected.tum')]
 dump(out/'tracking_started.json',dict(command=command,settings=settings,frames=len(rows),source_sha256=sha(ROOT/'src/cumulti/rgbd_full/track.cc'),binary_sha256=sha(OUT/'bin/rgbd_full'),vocabulary_sha256=sha(ORB/'Vocabulary/ORBvoc.txt'),library_sha256=sha(ORB/'lib/libORB_SLAM3.so'),sdk_revision=subprocess.check_output(['git','-C',str(ORB),'rev-parse','HEAD'],text=True).strip(),GT=False,LiDAR=False,IMU=False,GNSS=False))
 start=time.monotonic()
 with (out/'tracking_stdout.log').open('w') as log:r=subprocess.run(['/usr/bin/time','-v']+command,env=env,cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=data['duration_s']*3+1800)
 assert r.returncode==0,('Native tracking failed',r.returncode)
 denied=[line for line in (out/'native_access.log').read_text().splitlines() if any(s in line.lower() for s in ['_lidar.zip','gt_utm','gt_rel','imu_gps','/full_2hz/','/22b_','/12d_','clean_local_trajectory'])]
 assert not denied,('Forbidden native input',denied[:1])
 poses=list(csv.DictReader((out/'frame_poses.csv').open()));assert len(poses)==len(rows)
 epoch=-1;previous=None;valid=0;loss=[];begin=None;events=[]
 for i,p in enumerate(poses):
  good=p['valid_pose']=='1';p.update(rgb_bag_ns=rows[i]['rgb_bag_ns'],rgb_header_ns=rows[i]['rgb_header_ns'])
  if good:
   values=np.array([float(p[k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]);assert np.isfinite(values).all() and abs(np.linalg.norm(values[3:])-1)<1e-5
   fingerprint=(p['map_id'],p['map_init_kf'])
   if fingerprint!=previous:
    epoch+=1;events.append(dict(index=i,relative_ns=p['relative_ns'],epoch=epoch,map_id=p['map_id'],map_init_kf=p['map_init_kf']));previous=fingerprint
   valid+=1
   if begin is not None:loss.append(dict(start_index=begin,end_index=i-1,start_ns=rows[begin]['relative_ns'],end_ns=rows[i]['relative_ns']));begin=None
  elif begin is None:begin=i
  p['map_epoch']=epoch if good else -1
 if begin is not None:loss.append(dict(start_index=begin,end_index=len(rows)-1,start_ns=rows[begin]['relative_ns'],end_ns=rows[-1]['relative_ns']))
 writecsv(out/'camera_poses.csv',poses);writecsv(out/'map_epoch_events.csv',events,['index','relative_ns','epoch','map_id','map_init_kf']);writecsv(out/'tracking_loss_intervals.csv',loss,['start_index','end_index','start_ns','end_ns'])
 text=(out/'tracking_stdout.log').read_text();rss=re.search(r'Maximum resident set size \(kbytes\): (\d+)',text)
 dump(out/'tracking_metrics.json',dict(robot=robot,frames=len(rows),valid_frames=valid,valid_ratio=valid/len(rows),duration_s=data['duration_s'],map_epochs=epoch+1,map_changes=max(0,len(events)-1),map_id_changes=sum(a['map_id']!=b['map_id'] for a,b in zip(events,events[1:])),native_active_reset_log_count=text.count('LM: Active map reset recieved'),tracking_loss_intervals=len(loss),wall_s=time.monotonic()-start,peak_RSS_MiB=int(rss.group(1))/1024 if rss else None,track_ms=stats([float(p['track_ms']) for p in poses]),processing_complete=True,continuous_single_gauge=epoch==0 and valid==len(rows),pose_kind='recorded online camera Twc, not retrospectively corrected after BA',GT=False,LiDAR=False,IMU=False,GNSS=False))

if __name__=='__main__':run(sys.argv[1])
