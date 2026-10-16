#!/usr/bin/env python3
"""Presentation-only local maps + offline height diagnosis; never edits science."""
import hashlib, json, sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
SITE=ROOT/'demo/stage19_rgb_three_map'
OUT=ROOT/'outputs/cumulti_v1/19_rgb_three_map'
OLD=ROOT/'demo/progressive_multi_robot/data'
sys.path.insert(0,str(ROOT/'src/demo'))
from export_stage19_rgb import dump, digest

def main():
    SITE.mkdir(parents=True,exist_ok=True); OUT.mkdir(parents=True,exist_ok=True)
    replay=json.loads((OLD/'replay.json').read_text())
    print('local rows', {r:len(v) for r,v in replay['local_trajectory'].items()},flush=True)
    local=ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv'
    cols=['robot_id','keyframe_id','timestamp','tx','ty','tz','qx','qy','qz','qw']
    tr=pd.read_csv(local,usecols=cols)
    chunks=[]; sources=[dict(path=str(local),sha256=digest(local),role='frozen independent local Pose3')]
    k=pd.read_csv('/home/cas/CU-Multi/processed_v1/full_2hz/robot1/keyframes.csv',usecols=['keyframe_id','lidar_file']).set_index('keyframe_id')
    rt=tr[tr.robot_id.eq('robot1')].set_index('keyframe_id')
    for kid in range(150,2000,10):
        path=Path(k.loc[kid].lidar_file)
        if not path.is_absolute(): path=Path('/home/cas/CU-Multi/processed_v1/full_2hz/robot1/lidar')/path
        sources.append(dict(path=str(path),sha256=digest(path),role='read-only local LiDAR display subset'))
        p=np.asarray(np.load(path,mmap_mode='r')[::16,:3],float)
        p=p[np.isfinite(p).all(axis=1)&(np.linalg.norm(p,axis=1)>1)&(np.linalg.norm(p,axis=1)<60)]
        pose=rt.loc[kid]; q=Rotation.from_quat(pose[['qx','qy','qz','qw']].to_numpy(float)).as_matrix()
        p=p@q.T+pose[['tx','ty','tz']].to_numpy(float)
        _,ix=np.unique(np.floor(p/.5).astype(np.int64),axis=0,return_index=True); p=p[np.sort(ix)]
        p=np.round(p[::max(1,int(np.ceil(len(p)/180)))],3)
        chunks.append(dict(kf=kid,points=p.tolist()))
    pre=json.loads((OLD/'PRE_FIRST_LOOP.json').read_text())
    # Robot3 map is the saved independent-local PRE_FIRST_LOOP map, not optimized global.
    local_bounds={}
    for r in ['robot1','robot3']:
        p=np.array([v[2:5] for v in replay['local_trajectory'][r]])
        local_bounds[r]=[*p.min(axis=0).tolist(),*p.max(axis=0).tolist()]
    quaternions={r:tr[tr.robot_id.eq(r)].sort_values('keyframe_id')[['qx','qy','qz','qw']].to_numpy().tolist() for r in ['robot1','robot3']}
    dump(SITE/'local_maps.json',dict(robot1_chunks=chunks,robot3_points=pre['maps']['robot3'],trajectory=replay['local_trajectory'],quaternions=quaternions,bounds=local_bounds,local_pose_source_sha256=digest(local),GT_used=False,geometry_units='metres; XYZ uniformly scaled; no vertical exaggeration',map_protocol='frame10/point16/range1-60/voxel0.5, deterministic cap180/chunk (display only)'))
    final=json.loads((OLD/'FINAL_STREAM_END.json').read_text())
    stats={};offline_sources=[];fig,axes=plt.subplots(2,2,figsize=(11,6),dpi=140)
    for robot in ['robot1','robot3']:
        a=tr[tr.robot_id.eq(robot)].sort_values('keyframe_id'); xyz=a[['tx','ty','tz']].to_numpy()
        f=np.asarray([r[2:5] for r in final['trajectory'][robot]])
        z=xyz[:,2]; dz=np.diff(z); xy=np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1)
        # Offline reference only: read z to test the terrain hypothesis, never feed demo.
        gp=Path(f'/home/cas/CU-Multi/raw/main_campus/{robot}/{robot}_main_campus_gt_utm_poses.csv')
        offline_sources.append(dict(path=str(gp),sha256=digest(gp),role='offline height diagnosis only; never browser assets'))
        gt=pd.read_csv(gp,comment='#',header=None,names=['t','x','y','z','qx','qy','qz','qw'])
        gt_t=gt.t.to_numpy()*1e9; query=a.timestamp.to_numpy(np.int64)
        right=np.clip(np.searchsorted(gt_t,query),0,len(gt)-1);left=np.maximum(0,right-1)
        ix=np.where(abs(gt_t[left]-query)<=abs(gt_t[right]-query),left,right)
        gx=gt[['x','y','z']].to_numpy()[ix];gz=gx[:,2]
        # Rigid per-run alignment is diagnostic only; no transformed points enter presentation.
        xc=xyz-xyz.mean(axis=0);gc=gx-gx.mean(axis=0);u,s,v=np.linalg.svd(xc.T@gc)
        rot=u@np.diag([1,1,np.linalg.det(u@v)])@v; az=(xc@rot)[:,2]
        sec=(query-query[0])/1e9;i=0 if robot=='robot1' else 1
        axes[i,0].plot(sec,z-z[0],label='local gauge Z');axes[i,0].set_title(robot+' frozen gauge (not altitude)')
        axes[i,1].plot(sec,az,label='local after diagnostic rigid alignment');axes[i,1].plot(sec,gz-gz.mean(),label='offline reference height');axes[i,1].set_title('OFFLINE ONLY — not fed into demo')
        for ax in axes[i]:ax.set_xlabel('recorded seconds');ax.set_ylabel('metres');ax.legend(fontsize=7);ax.grid(alpha=.2)
        stats[robot]=dict(local_z_min=float(z.min()),local_z_max=float(z.max()),local_z_span_m=float(np.ptp(z)),final_z_span_m=float(np.ptp(f[:,2])),local_max_abs_adjacent_dz_m=float(abs(dz).max()),local_adjacent_dz_p95_m=float(np.percentile(abs(dz),95)),local_max_xy_step_m=float(xy.max()),offline_reference_z_span_m=float(np.ptp(gz)),offline_reference_max_abs_adjacent_dz_m=float(abs(np.diff(gz)).max()),offline_rigid_alignment_rotation=rot.tolist(),offline_gauge_z_vs_reference_up_deg=float(np.degrees(np.arccos(np.clip(rot[2,2],-1,1)))),offline_rigid_aligned_z_rmse_m=float(np.sqrt(np.mean((az-(gz-gz.mean()))**2))),offline_rigid_aligned_z_correlation=float(np.corrcoef(az,gz)[0,1]),max_gt_sync_ms=float(abs(gt_t[ix]-query).max()/1e6),GT_used_only_for='offline height hypothesis analysis; never in display/selection/optimization')
    dump(OUT/'trajectory_height_diagnosis.json',dict(robots=stats,display='true XYZ / uniform metric scale; perspective can accentuate slope, no Z exaggeration',coordinate_source='Stage12D: inverse(first EKF pose @ IMU_FROM_LIDAR) @ (EKF pose @ IMU_FROM_LIDAR); KF0-relative LiDAR axes, not gravity-aligned ENU',conclusion='Dominant apparent hundreds-of-metres elevation is coordinate-gauge orientation, not terrain: rigid diagnostic alignment reduces vertical discrepancy to sub-metre. Real reference elevation also varies. Residual drift/noise/optimization deformation are not ruled out. No flattening, GT display alignment or scientific pose edits.',GT_access='only offline diagnosis, not RGB/local map export'))
    fig.tight_layout();fig.savefig(OUT/'trajectory_height_diagnosis.png');plt.close(fig)
    sources += [dict(path=str(p),sha256=digest(p),role='accepted Stage18.1 immutable asset') for p in sorted(OLD.glob('*.json'))]
    for path in [ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv',ROOT/'outputs/cumulti_v1/13b_incremental_isam2/isam2_parameter_policy.json']:
        sources.append(dict(path=str(path),sha256=digest(path),role='frozen loop/policy lineage, read-only'))
    dump(OUT/'data_provenance_manifest.json',dict(baseline='bf30e1a23ee7b7ccb58d095949d443ab0ee2a7dd',sources=sources,offline_only_sources=offline_sources,raw_modified=False,solver_runs=0,retrieval_runs=0,GICP_runs=0,clean_loops=75,excluded=45,graph_nodes=5796,GT_not_in_demo=True))
    print(json.dumps(stats),flush=True)

if __name__=='__main__': main()
