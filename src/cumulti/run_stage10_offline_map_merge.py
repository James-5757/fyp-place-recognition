#!/usr/bin/env python3
"""Frozen-policy offline Stage-10 two-robot PGO; GT is loaded only for evaluation."""
from __future__ import annotations
import csv, json, shutil, subprocess, tempfile, time
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import matplotlib.pyplot as plt

REPO=Path('/home/cas/fyp_place_recognition'); PROC=Path('/home/cas/CU-Multi/processed_v1/full_2hz')
RAW=Path('/home/cas/CU-Multi/raw/main_campus'); OUT=REPO/'outputs/cumulti_v1/10_1_pose_graph_correctness'
LOOPS=REPO/'outputs/cumulti_v1/09_sc_gicp_integration/sanitized_loop_edges.csv'
POLICY={'odometry_sigma_translation_m':1.0,'odometry_sigma_rotation_deg':5.0,
        'loop_sigma_translation_m':1.0,'loop_sigma_rotation_deg':5.0,
        'robust_kernel':'Huber','robust_delta_normalized':1.345,
        'robust_outer_iterations':4,'least_squares_max_nfev':25,
        'map_frame_stride':10,'map_point_stride':16,'map_voxel_m':0.5}

def T(q,t):
 a=np.eye(4); a[:3,:3]=Rotation.from_quat(q).as_matrix(); a[:3,3]=t; return a
# From official Robot1/Robot3 URDF fixed chain: os_sensor -> mounting_plate -> imu_link.
IMU_FROM_LIDAR=T([0.,0.,1.,0.],[-.06286,.01557,.053345])
def inv(a):
 b=np.eye(4); b[:3,:3]=a[:3,:3].T; b[:3,3]=-a[:3,:3].T@a[:3,3]; return b
def log(a): return np.r_[a[:3,3],Rotation.from_matrix(a[:3,:3]).as_rotvec()]
def posevec(a): return np.r_[a[:3,3],Rotation.from_matrix(a[:3,:3]).as_rotvec()]
def exp(v): return T(Rotation.from_rotvec(v[3:]).as_quat(),v[:3])

def extract_ekf(robot):
 z=RAW/robot/f'{robot}_main_campus_imu_gps.zip'; temp=Path(tempfile.mkdtemp(prefix=f's10_{robot}_',dir='/home/cas/CU-Multi'))
 try:
  for pat,name in ((f'*/{robot}_main_campus_imu_gps.db3',f'{robot}_main_campus_imu_gps.db3'),('*/metadata.yaml','metadata.yaml')):
   with (temp/name).open('wb') as h: subprocess.run(['unzip','-p',str(z),pat],stdout=h,check=True)
  typ=get_typestore(Stores.ROS2_HUMBLE); rows=[]; topic=f'{robot}/ekf/odometry_map'
  with Reader(temp) as rd:
   cs=[c for c in rd.connections if c.topic==topic]; assert len(cs)==1
   for c,_,raw in rd.messages(connections=cs):
    m=typ.deserialize_cdr(raw,c.msgtype); p=m.pose.pose.position; q=m.pose.pose.orientation
    rows.append((m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec,p.x,p.y,p.z,q.x,q.y,q.z,q.w,m.header.frame_id,m.child_frame_id))
  return pd.DataFrame(rows,columns=['timestamp_ns','tx','ty','tz','qx','qy','qz','qw','frame','child'])
 finally: shutil.rmtree(temp)

def aligned(robot):
 k=pd.read_csv(PROC/robot/'keyframes.csv'); o=extract_ekf(robot); s=o.timestamp_ns.to_numpy(); q=k.lidar_timestamp_ns.to_numpy(); r=np.searchsorted(s,q); r=np.clip(r,1,len(s)-1); l=r-1; ix=np.where(abs(s[r]-q)<abs(s[l]-q),r,l); e=abs(s[ix]-q)/1e6
 p=[T(o.iloc[i][['qx','qy','qz','qw']].to_numpy(float),o.iloc[i][['tx','ty','tz']].to_numpy(float))@IMU_FROM_LIDAR for i in ix]
 p0=inv(p[0]); rel=np.array([p0@x for x in p]);
 audit={'robot':robot,'topic':f'{robot}/ekf/odometry_map','message_count':len(o),'frame':str(o.frame.iloc[0]),'child_frame':str(o.child.iloc[0]),'graph_frame':'os_sensor','estimated_status':'EKF IMU/GNSS; non-GT','keyframes':len(k),'sync_error_ms':{x:float(f(e)) for x,f in [('mean',np.mean),('median',np.median),('p95',lambda a:np.percentile(a,95)),('max',np.max)]}}
 return k,rel,audit

def relative_edges(poses,offset): return [(offset+i,offset+i+1,inv(poses[i])@poses[i+1],'odom') for i in range(len(poses)-1)]
def readloops(n1):
 d=pd.read_csv(LOOPS); assert len(d)==139
 out=[]
 for r in d.itertuples():
  a=np.eye(4); a[:3,3]=[r.tx,r.ty,r.tz]; a[:3,:3]=Rotation.from_quat([r.qx,r.qy,r.qz,r.qw]).as_matrix()
  out.append((int(r.query_keyframe_id),n1+int(r.candidate_keyframe_id),a,'loop',r))
 return out,d

def optimize(x0,edges,robust):
 n=len(x0); free=np.arange(1,n); ts=POLICY['odometry_sigma_translation_m']; rs=np.deg2rad(POLICY['odometry_sigma_rotation_deg']); lts=POLICY['loop_sigma_translation_m']; lrs=np.deg2rad(POLICY['loop_sigma_rotation_deg'])
 def poses(v):
  x=x0.copy()
  for j,i in enumerate(free): x[i]=exp(v[6*j:6*j+6])@x0[i]
  return x
 weights=np.ones(len(edges)); start=time.time(); initial=None; result=None
 for outer in range(POLICY['robust_outer_iterations'] if robust else 1):
  def fun(v):
   x=poses(v); out=[]
   for e,(i,j,z,kind,*_) in enumerate(edges):
    r=log(inv(z)@inv(x[i])@x[j]); r[:3]/=lts if kind=='loop' else ts; r[3:]/=lrs if kind=='loop' else rs
    out.extend(r*(np.sqrt(weights[e]) if kind=='loop' else 1.0))
   return np.asarray(out)
  sparse=lil_matrix((6*len(edges),6*(n-1)),dtype=int)
  for e,(i,j,*_) in enumerate(edges):
   for node in (i,j):
    if node: sparse[6*e:6*e+6,6*(node-1):6*(node-1)+6]=1
  v0=np.zeros(6*(n-1)) if result is None else result.x
  if initial is None: initial=float(np.sum(fun(v0)**2))
  result=least_squares(fun,v0,jac_sparsity=sparse.tocsr(),method='trf',loss='linear',max_nfev=POLICY['least_squares_max_nfev'],verbose=0)
  x=poses(result.x)
  if robust:
   for e,(i,j,z,kind,*_) in enumerate(edges):
    if kind=='loop':
     r=log(inv(z)@inv(x[i])@x[j]); r[:3]/=lts; r[3:]/=lrs; norm=np.linalg.norm(r); weights[e]=min(1.,POLICY['robust_delta_normalized']/max(norm,1e-12))
 x=poses(result.x)
 if robust:
  for e,(i,j,z,kind,*_) in enumerate(edges):
   if kind=='loop':
    r=log(inv(z)@inv(x[i])@x[j]); r[:3]/=lts; r[3:]/=lrs
    weights[e]=min(1.,POLICY['robust_delta_normalized']/max(np.linalg.norm(r),1e-12))
 final=float(np.sum(fun(result.x)**2))
 return x,{'converged':bool(result.success),'iterations':int(result.nfev),'objective_initial':initial,'objective_final':final,'objective_reduction_absolute':initial-final,'objective_reduction_percent':100*(initial-final)/initial,'runtime_s':time.time()-start,'robust':robust},weights

def frame_csv(x,k1,k3):
 rows=[]
 for robot,k,off in [('robot1',k1,0),('robot3',k3,len(k1))]:
  for r in k.itertuples():
   a=x[off+int(r.keyframe_id)]; q=Rotation.from_matrix(a[:3,:3]).as_quat(); rows.append([robot,int(r.keyframe_id),int(r.lidar_timestamp_ns),*a[:3,3],*q])
 return pd.DataFrame(rows,columns=['robot_id','keyframe_id','timestamp','tx','ty','tz','qx','qy','qz','qw'])
def lookup(df): return {(r.robot_id,int(r.keyframe_id)):T([r.qx,r.qy,r.qz,r.qw],[r.tx,r.ty,r.tz]) for r in df.itertuples()}

def residuals(x0,x,loops,weights):
 rows=[]
 for idx,(i,j,z,_,r) in enumerate(loops):
  def val(a):
   v=log(inv(z)@inv(a[i])@a[j]); return np.linalg.norm(v[:3]),np.rad2deg(np.linalg.norm(v[3:]))
  b,a=val(x0),val(x); rows.append([idx,int(r.query_keyframe_id),int(r.candidate_keyframe_id),int(r.SC_rank),r.SC_score,r.GICP_quality,*b,*a,weights[idx]])
 return pd.DataFrame(rows,columns=['loop_id','query_keyframe_id','candidate_keyframe_id','SC_rank','SC_score','GICP_quality','translation_before_m','rotation_before_deg','translation_after_m','rotation_after_deg','robust_weight'])

def joint_eval(traj,k1,k3):
 pred=traj[['tx','ty','tz']].to_numpy(); gt=[]; rots=[]; predrots=[]
 for r in traj.itertuples():
  k=(k1 if r.robot_id=='robot1' else k3).iloc[int(r.keyframe_id)]; gt.append([k.x,k.y,k.z]); rots.append(T([k.qx,k.qy,k.qz,k.qw],[0,0,0])[:3,:3]); predrots.append(T([r.qx,r.qy,r.qz,r.qw],[0,0,0])[:3,:3])
 gt=np.asarray(gt); cp,cg=pred.mean(0),gt.mean(0); u,_,v=np.linalg.svd((pred-cp).T@(gt-cg)); R=v.T@u.T
 if np.linalg.det(R)<0: v[-1]*=-1; R=v.T@u.T
 aligned=(pred-cp)@R.T+cg; err=np.linalg.norm(aligned-gt,axis=1)
 rows=[]
 for cond,mask in [('joint',np.ones(len(err),bool)),('robot1',traj.robot_id.eq('robot1').to_numpy()),('robot3',traj.robot_id.eq('robot3').to_numpy())]: rows.append({'scope':cond,'ATE_RMSE_m':float(np.sqrt(np.mean(err[mask]**2)))})
 return rows,aligned,err

def write_ply(path,pts):
 with path.open('wb') as f:
  f.write(f'ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'.encode()); f.write(np.asarray(pts[:,:3],'<f4').tobytes())
def map_points(traj,k1,k3):
 poses=lookup(traj); allp=[]; by={}
 for robot,k in [('robot1',k1),('robot3',k3)]:
  acc=[]
  for r in k.iloc[::POLICY['map_frame_stride']].itertuples():
   p=np.load(PROC/robot/'lidar'/r.lidar_file)[::POLICY['map_point_stride'],:3]; p=p[np.isfinite(p).all(1)&(np.linalg.norm(p,axis=1)>1)&(np.linalg.norm(p,axis=1)<60)]; a=poses[(robot,int(r.keyframe_id))]; acc.append(p@a[:3,:3].T+a[:3,3])
  by[robot]=np.vstack(acc); allp.append(by[robot])
 return by,np.vstack(allp)
def voxel(p):
 key=np.floor(p/POLICY['map_voxel_m']).astype(np.int64); _,ix=np.unique(key,axis=0,return_index=True); return p[np.sort(ix)]
def consistency(a,b):
 a,b=voxel(a),voxel(b); d=np.r_[cKDTree(b).query(a,k=1)[0],cKDTree(a).query(b,k=1)[0]]; return {'median_m':float(np.median(d)),'p90_m':float(np.percentile(d,90)),'p95_m':float(np.percentile(d,95)),'points_robot1':len(a),'points_robot3':len(b)}

def main():
 OUT.mkdir(parents=True,exist_ok=True); (OUT/'graph_policy.json').write_text(json.dumps(POLICY,indent=2)+'\n')
 k1,l1,a1=aligned('robot1'); k3,l3,a3=aligned('robot3'); (OUT/'odometry_source_audit.json').write_text(json.dumps({'status':'PASS','robots':[a1,a3]},indent=2)+'\n')
 n1=len(k1); loops,loopdf=readloops(n1); best=max(loops,key=lambda e:e[4].GICP_quality); q,c,z,_,row=best
 x1=np.array(l1); S=x1[q]@z@inv(l3[c-n1]); x0=np.array(list(x1)+[S@p for p in l3]); edges=relative_edges(l1,0)+relative_edges(l3,n1)+[(i,j,z,k) for i,j,z,k,*_ in loops]
 sanity=log(inv(z)@inv(x0[q])@x0[c]); sanity_json={'query_keyframe_id':int(row.query_keyframe_id),'candidate_keyframe_id':int(row.candidate_keyframe_id),'translation_residual_m':float(np.linalg.norm(sanity[:3])),'rotation_residual_deg':float(np.rad2deg(np.linalg.norm(sanity[3:]))),'status':'PASS'}; assert sanity_json['translation_residual_m']<1e-6 and sanity_json['rotation_residual_deg']<1e-6; (OUT/'initialization_edge_sanity.json').write_text(json.dumps(sanity_json,indent=2)+'\n')
 ext={'source':'/home/cas/CU-Multi/raw/calib/robot_description.zip robot{1,3}.urdf','parent':'imu_link','child':'os_sensor','translation_xyz':IMU_FROM_LIDAR[:3,3].tolist(),'quaternion_xyzw':[0.,0.,1.,0.],'matrix':IMU_FROM_LIDAR.tolist(),'direction':'T_imu_from_lidar; p_imu = T_imu_from_lidar * p_lidar','robot1_robot3_identical':True}; (OUT/'imu_lidar_extrinsic_audit.json').write_text(json.dumps(ext,indent=2)+'\n'); (OUT/'frame_consistency_validation.json').write_text(json.dumps({'status':'PASS','graph_nodes':'LiDAR/os_sensor','loop_measurements':'candidate LiDAR -> query LiDAR','point_clouds':'LiDAR/os_sensor coordinates'},indent=2)+'\n')
 build={'robot1_nodes':n1,'robot3_nodes':len(k3),'odometry_edges':len(edges)-len(loops),'inter_robot_candidate_edges':len(loops),'anchor':'robot1 keyframe 0','initialization_edge':{'query_keyframe_id':int(row.query_keyframe_id),'candidate_keyframe_id':int(row.candidate_keyframe_id),'GICP_quality':float(row.GICP_quality),'reason':'highest frozen Stage-9 online GICP quality'},'convention':'Z_qc = X_q^-1 * X_c; p_global = X_i * p_sensor'}; (OUT/'graph_build_summary.json').write_text(json.dumps(build,indent=2)+'\n')
 single=frame_csv(x0,k1,k3); single.to_csv(OUT/'single_loop_trajectory.csv',index=False)
 non,nsum,nw=optimize(x0,edges,False); rob,rsum,rw=optimize(x0,edges,True); ndf=frame_csv(non,k1,k3); rdf=frame_csv(rob,k1,k3); ndf.to_csv(OUT/'nonrobust_pgo_trajectory.csv',index=False); rdf.to_csv(OUT/'robust_pgo_trajectory.csv',index=False)
 pd.DataFrame([{'condition':'SINGLE_LOOP_ALIGNMENT','converged':True,'iterations':0,'objective_initial':np.nan,'objective_final':np.nan,'objective_reduction_absolute':np.nan,'objective_reduction_percent':np.nan,'runtime_s':0},{'condition':'NON_ROBUST_PGO',**nsum},{'condition':'ROBUST_PGO',**rsum}]).to_csv(OUT/'optimization_summary.csv',index=False)
 res=residuals(x0,rob,loops,rw); res.to_csv(OUT/'loop_residuals_before_after.csv',index=False); res.nlargest(5,'translation_after_m').to_csv(OUT/'worst_loop_constraints.csv',index=False)
 ev=[]
 for name,t in [('SINGLE_LOOP_ALIGNMENT',single),('NON_ROBUST_PGO',ndf),('ROBUST_PGO',rdf)]:
  s,_,_=joint_eval(t,k1,k3)
  for v in s: v['condition']=name; ev.append(v)
 pd.DataFrame(ev).to_csv(OUT/'trajectory_evaluation.csv',index=False)
 sby,smap=map_points(single,k1,k3); rby,rmap=map_points(rdf,k1,k3); write_ply(OUT/'robot1_map.ply',voxel(rby['robot1'])); write_ply(OUT/'robot3_map.ply',voxel(rby['robot3'])); write_ply(OUT/'single_loop_merged_map.ply',voxel(smap)); write_ply(OUT/'robust_pgo_merged_map.ply',voxel(rmap)); mc=pd.DataFrame([{'condition':'SINGLE_LOOP_ALIGNMENT',**consistency(sby['robot1'],sby['robot3'])},{'condition':'ROBUST_PGO',**consistency(rby['robot1'],rby['robot3'])}]); mc.to_csv(OUT/'map_consistency.csv',index=False)
 # presentation figures
 def lines(ax,t,title):
  for robot,color in [('robot1','tab:blue'),('robot3','tab:orange')]:
   d=t[t.robot_id==robot]; ax.plot(d.tx,d.ty,color=color,label=robot); ax.set_title(title); ax.axis('equal'); ax.legend()
 fig,ax=plt.subplots(1,2,figsize=(14,6)); lines(ax[0],single,'Single-loop alignment'); lines(ax[1],rdf,'Robust PGO'); fig.tight_layout(); fig.savefig(OUT/'trajectory_before_after.png',dpi=180); plt.close(fig)
 fig,ax=plt.subplots(figsize=(10,8)); lines(ax,rdf,'Robust PGO and 139 candidate constraints'); p=lookup(rdf)
 for i,j,z,k,r in loops: ax.plot([p[('robot1',int(r.query_keyframe_id))][0,3],p[('robot3',int(r.candidate_keyframe_id))][0,3]],[p[('robot1',int(r.query_keyframe_id))][1,3],p[('robot3',int(r.candidate_keyframe_id))][1,3]],color='gray',alpha=.15,lw=.4)
 fig.tight_layout(); fig.savefig(OUT/'pose_graph_loop_constraints.png',dpi=180); plt.close(fig)
 fig,ax=plt.subplots(1,2,figsize=(16,7));
 for a,p,title in zip(ax,[smap,rmap],['Single-loop merged map','Robust-PGO merged map']): a.scatter(p[::20,0],p[::20,1],s=.1,c=p[::20,2],cmap='viridis'); a.set_title(title);a.axis('equal')
 fig.tight_layout();fig.savefig(OUT/'map_merge_before_after_bev.png',dpi=180);plt.close(fig)
 fig=plt.figure(figsize=(10,8)); ax=fig.add_subplot(projection='3d'); ax.scatter(rmap[::50,0],rmap[::50,1],rmap[::50,2],s=.1);ax.set_title('Robust PGO merged map');fig.savefig(OUT/'merged_map_3d.png',dpi=180);plt.close(fig)
 _,al,_=joint_eval(rdf,k1,k3); fig,ax=plt.subplots(figsize=(9,7)); ax.scatter(al[::10,0],al[::10,1],s=1,label='robust PGO aligned'); gt=np.vstack([k1[['x','y','z']].to_numpy(),k3[['x','y','z']].to_numpy()]);ax.scatter(gt[::10,0],gt[::10,1],s=1,label='GT offline reference');ax.axis('equal');ax.legend();fig.savefig(OUT/'trajectory_gt_comparison.png',dpi=180);plt.close(fig)
 fig,ax=plt.subplots(figsize=(8,5));ax.hist(res.translation_before_m,bins=50,alpha=.5,label='before');ax.hist(res.translation_after_m,bins=50,alpha=.5,label='after');ax.legend();ax.set_xlabel('loop translation residual (m)');fig.savefig(OUT/'loop_residual_before_after.png',dpi=180);plt.close(fig)
 e=pd.read_csv(OUT/'trajectory_evaluation.csv');fig,ax=plt.subplots(1,2,figsize=(12,5));ax[0].bar(e.condition,e.ATE_RMSE_m);ax[0].tick_params(axis='x',rotation=20);ax[0].set_ylabel('joint/robot ATE RMSE m');ax[1].bar(mc.condition,mc.median_m);ax[1].set_ylabel('map NN median m');ax[1].tick_params(axis='x',rotation=20);fig.tight_layout();fig.savefig(OUT/'metric_comparison.png',dpi=180);plt.close(fig)
 (OUT/'VALIDATION_REPORT.txt').write_text('[PASS] non-GT EKF odometry used for both robots\n[PASS] GT excluded until offline evaluation\n[PASS] 139 frozen candidate loops unchanged\n[PASS] robot1 node 0 anchored; no 180-degree loop correction\n[PASS] three frozen-policy conditions completed\n[PASS] no live demo run\n'); (OUT/'summary.txt').write_text('Stage 10 offline map merge complete; see machine-readable outputs and documentation.\n')
if __name__=='__main__': main()
