"""Bounded camera-only bridge study; rejected/unsupported epoch pairs stay separate."""
import csv,json,os,subprocess,sys
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from common import *
ORB=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')
POLICY=dict(query_interval_s=5,database_interval_s=1,proposals_per_query=3,proposal='frozen ORB vocabulary / DBoW2, native 1000 features',verification='OpenCV ORB 1000, mutual Hamming ratio, metric rigid RANSAC',ratio=.75,min_mutual=60,min_metric=40,ransac_iterations=512,seed=20261110,inlier_m=.15,min_inliers=30,min_fraction=.5,max_rmse_m=.1,max_bidir_median_px=3,min_spread_ratio=.05,min_support=3,max_support_translation_m=.5,max_support_rotation_deg=5,min_query_span_s=2,min_database_span_s=2,GT=False,LiDAR=False,IMU=False,GNSS=False,causal_online=False)

def pose(p):
 T=np.eye(4);T[:3,:3]=Rotation.from_quat([float(p[k]) for k in ['qx','qy','qz','qw']]).as_matrix();T[:3,3]=[float(p[k]) for k in ['tx','ty','tz']];return T

def rigid(a,b):
 # candidate a -> query b; row-vector convention a @ R.T + t.
 ac=a.mean(0);bc=b.mean(0);u,s,vt=np.linalg.svd((a-ac).T@(b-bc));v=vt.T;fix=np.eye(3);fix[2,2]=np.linalg.det(v@u.T);R=v@fix@u.T;return R,bc-R@ac

def run():
 sys.addaudithook(guard);cv2.setNumThreads(1);out=OUT/'fusion';out.mkdir(exist_ok=True)
 declared=out/'predeclared_policy.json'
 if declared.exists():assert json.loads(declared.read_text())==POLICY
 else:dump(declared,POLICY)
 rows={r:list(csv.DictReader((OUT/r/'camera_poses.csv').open())) for r in ['robot1','robot3']};selected={};cfg={}
 lines=[]
 for r,interval in [('robot1',5),('robot3',1)]:
  source=json.loads((OUT/r/'RGB_audit.json').read_text());K=np.array(source['calibration']['K']).reshape(3,3);K[:2]*=.5;cfg[r]=(K,np.array(source['calibration']['D']));next_t=0;selected[r]=[]
  pairs=list(csv.DictReader((OUT/r/'sync_pairs.csv').open()))
  for i,p in enumerate(rows[r]):
   t=int(p['relative_ns'])/1e9
   if p['valid_pose']=='1' and pairs[i]['paired']=='True' and t>=next_t:
    lines.append(f"{r} {i} {ROOT/pairs[i]['rgb']}\n");selected[r].append(i);next_t=t+interval
 seq=out/'rgb_proposal_sequence.txt';seq.write_text(''.join(lines));env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=':'.join(map(str,[ORB/'lib',ORB.parent/'pangolin-install/lib',ORB/'Thirdparty/DBoW2/lib',ORB/'Thirdparty/g2o/lib'])))
 with (out/'proposal_stdout.log').open('w') as log:
  result=subprocess.run([str(OUT/'bin/rgbd_propose'),str(ORB/'Vocabulary/ORBvoc.txt'),str(seq),str(out/'rgb_candidates.csv')],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
 assert result.returncode==0
 feature_cache={};extractor=cv2.ORB_create(nfeatures=1000,scaleFactor=1.2,nlevels=8,fastThreshold=20)
 def features(robot,i):
  key=(robot,i)
  if key in feature_cache:return feature_cache[key]
  image=cv2.imread(str(OUT/robot/'rgb'/f'{i:06d}.jpg'),0);depth=cv2.imread(str(OUT/robot/'depth'/f'{i:06d}.png'),cv2.IMREAD_UNCHANGED);keys,desc=extractor.detectAndCompute(image,None)
  uv=np.array([k.pt for k in keys],np.float32).reshape(-1,2);pix=np.floor(uv).astype(int);z=depth[pix[:,1],pix[:,0]]/1000.;K,D=cfg[robot];xy=cv2.undistortPoints(uv.reshape(-1,1,2),K,D).reshape(-1,2);xyz=np.column_stack([xy,np.ones(len(xy))])*z[:,None]
  value=(uv,desc,xyz,(z>=.3)&(z<=10));feature_cache[key]=value;return value
 matcher=cv2.BFMatcher(cv2.NORM_HAMMING);records=[];accepted=[]
 for proposal in csv.DictReader((out/'rgb_candidates.csv').open()):
  qi=int(proposal['query_index']);ci=int(proposal['candidate_index']);q,c=rows['robot1'][qi],rows['robot3'][ci];qu,qd,qx,qok=features('robot1',qi);cu,cd,cx,cok=features('robot3',ci)
  record=dict(**proposal,query_epoch=int(q['map_epoch']),candidate_epoch=int(c['map_epoch']),accepted=False,reason='INSUFFICIENT_MUTUAL_RGB_MATCHES')
  if qd is None or cd is None:records.append(record);continue
  forward=matcher.knnMatch(qd,cd,k=2);reverse=matcher.knnMatch(cd,qd,k=2);back={v[0].queryIdx:v[0].trainIdx for v in reverse if len(v)==2 and v[0].distance<.75*v[1].distance}
  pairs=[(v[0].queryIdx,v[0].trainIdx) for v in forward if len(v)==2 and v[0].distance<.75*v[1].distance and back.get(v[0].trainIdx)==v[0].queryIdx];record['mutual_matches']=len(pairs)
  if len(pairs)<60:records.append(record);continue
  pairs=np.array([(a,b) for a,b in pairs if qok[a] and cok[b]],int);record['metric_matches']=len(pairs);record['reason']='INSUFFICIENT_MEASURED_DEPTH_MATCHES'
  if len(pairs)<40:records.append(record);continue
  a=cx[pairs[:,1]];b=qx[pairs[:,0]];rng=np.random.default_rng(POLICY['seed']+qi*100000+ci);best=None
  for iteration in range(512):
   indices=rng.choice(len(a),3,replace=False);R,t=rigid(a[indices],b[indices]);error=np.linalg.norm(a@R.T+t-b,axis=1);mask=error<.15
   if best is None or mask.sum()>best.sum():best=mask
  record['reason']='METRIC_GEOMETRY_REJECTED'
  if best.sum()<30 or best.mean()<.5:records.append(record);continue
  R,t=rigid(a[best],b[best]);error=np.linalg.norm(a@R.T+t-b,axis=1);mask=error<.15;n=int(mask.sum());fraction=float(mask.mean());rmse=float(np.sqrt(np.mean(error[mask]**2)))
  spread=[np.linalg.svd(v[mask]-v[mask].mean(0),compute_uv=False) for v in [a,b]];spread_ratio=float(min(s[1]/max(s[0],1e-12) for s in spread))
  predq=cv2.projectPoints(a[mask]@R.T+t,np.zeros(3),np.zeros(3),*cfg['robot1'])[0].reshape(-1,2);predc=cv2.projectPoints((b[mask]-t)@R,np.zeros(3),np.zeros(3),*cfg['robot3'])[0].reshape(-1,2)
  reprojection=float(max(np.median(np.linalg.norm(predq-qu[pairs[mask,0]],axis=1)),np.median(np.linalg.norm(predc-cu[pairs[mask,1]],axis=1))))
  good=n>=30 and fraction>=.5 and rmse<=.1 and reprojection<=3 and spread_ratio>=.05
  record.update(accepted=good,reason='VERIFIED_SINGLE_RGBD_PAIR' if good else 'METRIC_GEOMETRY_REJECTED',inliers=n,inlier_fraction=fraction,rmse_m=rmse,bidir_median_px=reprojection,spread_ratio=spread_ratio)
  if good:
   Z=np.eye(4);Z[:3,:3]=R;Z[:3,3]=t;S=pose(q)@Z@np.linalg.inv(pose(c));accepted.append(dict(query_index=qi,candidate_index=ci,query_epoch=int(q['map_epoch']),candidate_epoch=int(c['map_epoch']),query_s=int(q['relative_ns'])/1e9,candidate_s=int(c['relative_ns'])/1e9,rmse_m=rmse,inliers=n,T_robot1_epoch_from_robot3_epoch=S.tolist(),T_query_from_candidate=Z.tolist()))
  records.append(record)
 fields=list(dict.fromkeys(k for r in records for k in r));writecsv(out/'rgbd_geometric_verification.csv',records,fields)
 groups=[]
 for pair in sorted({(e['query_epoch'],e['candidate_epoch']) for e in accepted}):
  edges=[e for e in accepted if (e['query_epoch'],e['candidate_epoch'])==pair]
  for anchor in sorted(edges,key=lambda e:(e['rmse_m'],-e['inliers'])):
   S=np.array(anchor['T_robot1_epoch_from_robot3_epoch']);support=[]
   for e in edges:
    delta=np.linalg.inv(S)@np.array(e['T_robot1_epoch_from_robot3_epoch']);angle=np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude())
    if np.linalg.norm(delta[:3,3])<=.5 and angle<=5:support.append(e)
   unique_q={e['query_index'] for e in support};unique_c={e['candidate_index'] for e in support}
   if len(unique_q)>=3 and len(unique_c)>=3 and np.ptp([e['query_s'] for e in support])>=2 and np.ptp([e['candidate_s'] for e in support])>=2:
    groups.append(dict(query_epoch=pair[0],candidate_epoch=pair[1],anchor=anchor,support=support,status='VERIFIED_EPOCH_PAIR_ONLY'));break
 merged=[]
 dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('r','u1'),('g','u1'),('b','u1')])
 for g in groups:
  pieces=[]
  for robot,epoch in [('robot1',g['query_epoch']),('robot3',g['candidate_epoch'])]:
   source=OUT/robot/'maps'/f'epoch_{epoch:04d}.ply'
   with source.open('rb') as f:
    while True:
     line=f.readline();assert line,('Invalid PLY header',str(source))
     if line.strip()==b'end_header':break
    cloud=np.frombuffer(f.read(),dtype=dtype).copy()
   if robot=='robot3':
    S=np.array(g['anchor']['T_robot1_epoch_from_robot3_epoch']);xyz=np.column_stack([cloud[k] for k in ['x','y','z']])@S[:3,:3].T+S[:3,3]
    for j,k in enumerate(['x','y','z']):cloud[k]=xyz[:,j]
   pieces.append(cloud)
  combined=np.concatenate(pieces);path=out/f"verified_R1e{g['query_epoch']}_R3e{g['candidate_epoch']}.ply"
  with path.open('wb') as f:f.write(('ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(combined))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode());f.write(combined.tobytes())
  merged.append(dict(path=str(path.relative_to(ROOT)),bytes=path.stat().st_size,sha256=sha(path),points=len(combined),query_epoch=g['query_epoch'],candidate_epoch=g['candidate_epoch'],scope='this verified epoch pair only; unrelated epochs remain separate',server_only=True))
 dump(out/'verified_epoch_map_manifest.json',merged)
 status='VERIFIED_EPOCH_PAIRS_ONLY' if groups else 'NO_RELIABLE_RGBD_BRIDGE_ESTABLISHED'
 decision=dict(status=status,verified_epoch_pairs=groups,proposal_queries=len(selected['robot1']),proposal_database=len(selected['robot3']),tested_pairs=len(records),single_pair_passes=len(accepted),scope='bounded uniform RGB-only offline search; absence is not proof no overlap exists',unrelated_epochs_glued=False,global_full_recording_fusion=False,GT=False,LiDAR=False)
 dump(out/'verified_rgbd_pairs.json',accepted);dump(out/'fusion_decision.json',decision)
 dump(SITE/'fusion_summary.json',decision)
 manifest=json.loads((SITE/'manifest.json').read_text());manifest.update(global_fusion=status,verified_epoch_pairs=len(groups));dump(SITE/'manifest.json',manifest)
 print(status,flush=True)

if __name__=='__main__':
 if '--declare-only' in sys.argv:
  from datetime import datetime,timezone
  dump(OUT/'fusion/predeclared_policy.json',POLICY)
  dump(OUT/'fusion/policy_declaration_receipt.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(Path(__file__)),tracking_complete_at_declaration={r:(OUT/r/'tracking_metrics.json').exists() for r in ['robot1','robot3']}))
 else:run()
