"""Post-hoc diagnostics, never tracking/encoding/retrieval/optimization."""
import csv,hashlib,json,os,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];OLD=ROOT/'outputs/cumulti_v1/24_rgbd_long_v1';OUT=ROOT/'outputs/cumulti_v1/24_rgbd_loss_rootcause_v1'
SDK=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def run():
 OUT.mkdir(exist_ok=True);(OUT/'overlays').mkdir(exist_ok=True)
 if (OUT/'native_complete.json').exists():
  receipt=json.loads((OUT/'native_complete.json').read_text());assert hashlib.sha256((OUT/'native_feature_support.csv').read_bytes()).hexdigest()==receipt['sha256'];print('NATIVE_FEATURE_DIAGNOSTICS_REUSED');return
 if (OUT/'native_feature_support.csv').exists():raise RuntimeError('Unfinalized native CSV: validate completion before reuse; never overwrite it silently')
 windows=list(csv.DictReader((OLD/'loss_windows.csv').open()));selected={};marked={}
 for robot in ['robot1','robot3']:
  selected[robot]=set(range(0,1200,10));marked[robot]=set()
  for w in windows:
   if w['robot']!=robot:continue
   a,b=int(w['start_index']),int(w['end_index']);selected[robot].update(range(max(0,a-20),min(1200,b+21)));marked[robot].update([max(0,a-1),a,min(1199,b+1)])
 lines=[]
 for robot in ['robot1','robot3']:
  source=json.loads((OLD/robot/'export.json').read_text());frames=source['frames'];origin=int(frames[0]['rgb_bag_ns'])
  for capacity,config in [(1000,'baseline1000'),(2000,'fallback2000')]:
   for i in sorted(selected[robot]):
    f=frames[i];lines.append(f"{robot} {capacity} {i} {(int(f['rgb_bag_ns'])-origin)/1e9:.9f} {ROOT/f['rgb']} {ROOT/f['depth']} {OLD/robot/config/'camera.yaml'} {int(i in marked[robot])}\n")
 (OUT/'sequence.txt').write_text(''.join(lines))
 dump(OUT/'diagnostic_policy.json',dict(selection='all7 lost intervals ±20 frames, plus every10th frame control; both native feature capacities on identical images',
  sampling='failure-conditioned post-hoc diagnostics, not a new unbiased accuracy benchmark',selected_frames={r:len(v) for r,v in selected.items()},
  extractor='actual linked ORB-SLAM3 ORBextractor; saved YAML1000/2000,1.2,8,20/7',
  depth_lookup='truncate pixel coordinates exactly as Frame::ComputeStereoFromRGBD; explicit rounded lookup disagreement also counted',
  close_depth='saved Stereo.b * Stereo.ThDepth =2m, virtual native close/far threshold',
  stable_patch='3x3 all nonzero and depth span<=0.1m; diagnostic only, no filter applied',
  pair_matching='consecutive selected original frames only; mutual Hamming ratio0.75; undistorted fundamental RANSAC1.5px,.99,seed20261110',
  SLAM_rerun=False,new_poses=False,retrieval=False,GT=False,core_modified=False))
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=':'.join(map(str,[SDK/'lib',SDK.parent/'pangolin-install/lib',SDK/'Thirdparty/DBoW2/lib',SDK/'Thirdparty/g2o/lib'])))
 with (OUT/'native_stdout.log').open('w') as f:
  p=subprocess.run(['strace','-f','-e','trace=open,openat','-o',str(OUT/'native_access.log'),str(OUT/'bin/native_features'),str(OUT/'sequence.txt'),str(OUT/'native_feature_support.csv'),str(OUT/'overlays')],cwd=OUT,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=240)
 assert p.returncode==0,p.returncode
 trace=(OUT/'native_access.log').read_text();assert not any(s in trace.lower() for s in ['gt_utm','gt_rel','_lidar.zip','imu_gps','/full_2hz/'])
 assert len(list(csv.DictReader((OUT/'native_feature_support.csv').open())))==len(lines)
 dump(OUT/'native_complete.json',dict(rows=len(lines),sha256=hashlib.sha256((OUT/'native_feature_support.csv').read_bytes()).hexdigest(),native_returncode=0,SLAM_rerun=False))
 print('OFFLINE_NATIVE_FEATURE_AUDIT_COMPLETE',len(lines))
if __name__=='__main__':run()
