#!/usr/bin/env python3
"""Stage18: read-only frozen checkpoint packaging; no solver/GT/frontend calls."""
import argparse, hashlib, json, math, time
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
D12 = ROOT/'outputs/cumulti_v1/12d_clean_backend'
B13 = ROOT/'outputs/cumulti_v1/13b_incremental_isam2'
OUT = ROOT/'outputs/cumulti_v1/18_progressive_demo'
DATA = ROOT/'demo/progressive_multi_robot/data'
ACCEPTED = 'ad649678e6dcc1021724cae97b291b03d6a9ad59'
POSE = ['robot_id','keyframe_id','timestamp','tx','ty','tz','qx','qy','qz','qw']
SOURCES = {}

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()

def source(path, role):
    p=Path(path); SOURCES[str(p)]={'path':str(p),'role':role,'sha256':sha(p),'size_bytes':p.stat().st_size}
    return p

def dump(path,obj):
    Path(path).write_text(json.dumps(obj,separators=(',',':'),allow_nan=False)+'\n')

def matrix(row):
    T=np.eye(4); T[:3,:3]=Rotation.from_quat([row.qx,row.qy,row.qz,row.qw]).as_matrix()
    T[:3,3]=[row.tx,row.ty,row.tz]; return T

def sample(points,n=10000):
    p=np.asarray(points); assert np.isfinite(p).all()
    return np.round(p[::max(1,math.ceil(len(p)/n))],3).tolist()

def ply(path):
    source(path,'frozen per-robot pose-reprojected LiDAR map')
    with Path(path).open('rb') as f:
        header=[]
        while True:
            line=f.readline().decode('ascii').strip(); header.append(line)
            if line=='end_header': break
        assert 'format binary_little_endian 1.0' in header
        assert [x for x in header if x.startswith('property')]==['property float x','property float y','property float z']
        count=int(next(x for x in header if x.startswith('element vertex')).split()[-1])
        p=np.frombuffer(f.read(),dtype='<f4').reshape(-1,3); assert len(p)==count
    return sample(p)

def trajectory(path):
    tr=pd.read_csv(source(path,'frozen Pose3 trajectory'),usecols=POSE)
    assert not tr.duplicated(['robot_id','keyframe_id']).any()
    rows={}
    for robot,start,end in [('robot1',150,1999),('robot3',234,4179)]:
        r=tr[tr.robot_id.eq(robot)].sort_values('keyframe_id')
        assert r.keyframe_id.tolist()==list(range(start,int(r.keyframe_id.max())+1))
        assert r.timestamp.is_monotonic_increasing and r.keyframe_id.max()<=end
        assert np.isfinite(r[['tx','ty','tz','qx','qy','qz','qw']]).all().all()
        assert np.max(abs(np.linalg.norm(r[['qx','qy','qz','qw']],axis=1)-1))<1e-6
        rows[robot]=[[int(x.keyframe_id),str(x.timestamp),x.tx,x.ty,x.tz] for x in r.itertuples()]
    return tr,rows

def cloud(path):
    source(path,'non-GT cached LiDAR XYZI scan'); p=np.load(path,mmap_mode='r')
    p=np.asarray(p[::16,:3],dtype=float); radius=np.linalg.norm(p,axis=1)
    return p[np.isfinite(p).all(axis=1)&(radius>1)&(radius<60)]

def common_initial_map():
    """Exact frozen first-loop pose reprojection with global per-source voxelization."""
    tr=pd.read_csv(source(D12/'clean_first_loop_trajectory.csv','frozen common initialization'),usecols=POSE)
    tr=tr[tr.robot_id.eq('robot3')|tr.keyframe_id.le(283)]
    maps={}; rows={}
    for robot,start in [('robot1',150),('robot3',234)]:
        proc=Path('/home/cas/CU-Multi/processed_v1/full_2hz')/robot
        k=pd.read_csv(source(proc/'keyframes.csv','LiDAR IDs/paths only; no GT columns'),usecols=['keyframe_id','lidar_file'])
        end=283 if robot=='robot1' else 4179
        poses=tr[tr.robot_id.eq(robot)].set_index('keyframe_id'); acc=[]
        for f in k[k.keyframe_id.between(start,end)].iloc[::10].itertuples():
            path=Path(f.lidar_file)
            if not path.is_absolute():path=proc/'lidar'/path
            p=cloud(path); T=matrix(poses.loc[f.keyframe_id]); acc.append(p@T[:3,:3].T+T[:3,3])
        p=np.vstack(acc); _,ix=np.unique(np.floor(p/.5).astype(np.int64),axis=0,return_index=True)
        maps[robot]=sample(p[np.sort(ix)])
        rows[robot]=[[int(x.keyframe_id),str(x.timestamp),x.tx,x.ty,x.tz] for x in tr[tr.robot_id.eq(robot)].itertuples()]
    dump(DATA/'COMMON_INITIAL.json',dict(name='COMMON_FRAME_INITIALIZATION',kf=283,K=1,nodes=4080,frame='ROBOT1_KF150_GAUGE',trajectory=rows,maps=maps,optimizer_runs=0))

def main():
    if DATA.exists() and (DATA/'replay.json').exists(): raise RuntimeError('Refuse Stage18 data overwrite')
    DATA.mkdir(parents=True,exist_ok=True); OUT.mkdir(parents=True,exist_ok=True)
    history={str(p):sha(p) for p in (ROOT/'demo/final_replay').rglob('*') if p.is_file()}
    frozen_decisions=[ROOT/'outputs/cumulti_v1/15_informative_loop_selection/summary.json',ROOT/'outputs/cumulti_v1/17_relative_measurement_geometry/stage17_geometry_decision.json']
    decision_hashes={str(p):sha(p) for p in frozen_decisions}
    loops=pd.read_csv(source(D12/'ordered_rank1_clean_loops.csv','clean75 arrival order'))
    assert len(loops)==75 and loops.arrival_index.tolist()==list(range(1,76))
    assert loops.query_keyframe_id.is_monotonic_increasing
    assert (int(loops.iloc[0].frozen_loop_id),int(loops.iloc[0].query_keyframe_id),int(loops.iloc[0].candidate_keyframe_id))==(45,283,252)
    frontcols=['query_keyframe_id','candidate_keyframe_id','SC_rank','query_pointcloud_path','candidate_pointcloud_path','registration_seconds']
    front=pd.read_csv(source(ROOT/'outputs/cumulti_v1/10_2_pgo_policy_selection/rank1_sanitized_edges.csv','frozen frontend fields ONLY; offline GT columns explicitly not decoded'),usecols=frontcols)
    assert len(front)==120
    events=[]
    for r in loops.itertuples():
        f=front[(front.query_keyframe_id==r.query_keyframe_id)&(front.candidate_keyframe_id==r.candidate_keyframe_id)]
        assert len(f)==1; f=f.iloc[0]
        q=cloud(f.query_pointcloud_path); c=cloud(f.candidate_pointcloud_path)
        q=q[::max(1,math.ceil(len(q)/300))]; c=c[::max(1,math.ceil(len(c)/300))]
        T=matrix(r); aligned=c@T[:3,:3].T+T[:3,3]
        events.append(dict(index=int(r.arrival_index),frozen_id=int(r.frozen_loop_id),query=int(r.query_keyframe_id),candidate=int(r.candidate_keyframe_id),query_timestamp=str(r.query_timestamp),candidate_timestamp=str(r.candidate_timestamp),score=float(r.SC_score),rank=int(f.SC_rank),shift=int(r.best_shift),yaw=float(r.SC_yaw_initialization_deg),quality=float(r.GICP_quality),gicp_ms=float(f.registration_seconds)*1000,transform=T.tolist(),direction=r.transform_direction,query_cloud=q.tolist(),candidate_cloud=c.tolist(),aligned_cloud=aligned.tolist()))
    _,localrows=trajectory(D12/'clean_local_trajectory.csv')
    trace=pd.read_csv(source(B13/'incremental_event_trace.csv','actual per-query causal graph counts and backend timings'))
    trace=trace[trace.event_type.ne('FINAL_STATE')]
    assert trace.robot1_keyframe.tolist()==list(range(150,2000))
    tracecols=['robot1_keyframe','total_graph_nodes','cumulative_global_factors','cumulative_loop_factors','isam2_update_ms','estimate_extraction_ms','map_publication_ms']
    tracevalues=trace[tracecols].to_numpy().tolist()
    cps=pd.read_csv(source(B13/'checkpoint_manifest.csv','saved historical persistent iSAM2 checkpoint identities'))
    snapshots=[]
    for c in cps.itertuples():
        tr,rows=trajectory(c.trajectory_path); assert sha(c.trajectory_path)==c.trajectory_sha256
        assert int(tr[tr.robot_id.eq('robot1')].keyframe_id.max())==int(c.robot1_keyframe)
        manifest=json.loads(source(c.map_manifest,'exact checkpoint map protocol and source hashes').read_text())
        assert manifest['protocol']['frame_stride']==10 and manifest['protocol']['point_stride']==16 and manifest['protocol']['voxel_m']==.5 and manifest['protocol']['range_m']==[1,60]
        maps={}
        for a in manifest['artifacts']:
            if a.get('robot_source') in ['robot1','robot3']:
                assert sha(a['path'])==a['sha256']; maps[a['robot_source']]=ply(a['path'])
        assert set(maps)=={'robot1','robot3'}
        s=dict(name=c.checkpoint,kf=int(c.robot1_keyframe),K=int(c.loop_factors),nodes=int(c.graph_nodes),frame=c.frame,trajectory=rows,maps=maps)
        dump(DATA/(c.checkpoint+'.json'),s)
        snapshots.append({k:s[k] for k in ['name','kf','K','nodes','frame']}|{'file':c.checkpoint+'.json','sha256':sha(DATA/(c.checkpoint+'.json'))})
    # Pre-loop Robot1 map grows from arrived local scans, never from the future PRE_FIRST_LOOP snapshot.
    proc=Path('/home/cas/CU-Multi/processed_v1/full_2hz/robot1')
    keyframes=pd.read_csv(source(proc/'keyframes.csv','LiDAR paths only; GT columns excluded'),usecols=['keyframe_id','lidar_file'])
    localtr=pd.read_csv(D12/'clean_local_trajectory.csv',usecols=POSE).query("robot_id=='robot1'").set_index('keyframe_id')
    chunks=[]
    for k in range(150,284,10):
        fn=Path(keyframes.set_index('keyframe_id').loc[k].lidar_file)
        if not fn.is_absolute(): fn=Path(front.query_pointcloud_path.iloc[0]).parent/fn
        p=cloud(fn); T=matrix(localtr.loc[k]); p=p@T[:3,:3].T+T[:3,3]
        _,ix=np.unique(np.floor(p/.5).astype(np.int64),axis=0,return_index=True)
        chunks.append(dict(kf=k,points=sample(p[np.sort(ix)],900)))
    # Explicit first-loop common-frame initialization, BEFORE nonlinear iSAM2 correction.
    common=pd.read_csv(source(D12/'clean_first_loop_trajectory.csv','frozen S=Xq Z inverse(Lc), no optimization'),usecols=POSE)
    common=common[common.robot_id.eq('robot3')|common.keyframe_id.le(283)]
    commonrows={r:[[int(x.keyframe_id),str(x.timestamp),x.tx,x.ty,x.tz] for x in common[common.robot_id.eq(r)].itertuples()] for r in ['robot1','robot3']}
    init=matrix(loops.iloc[0]); lq=matrix(localtr.loc[283]); lc=pd.read_csv(D12/'clean_local_trajectory.csv',usecols=POSE).query("robot_id=='robot3' and keyframe_id==252").iloc[0]
    S=lq@init@np.linalg.inv(matrix(lc)); assert np.linalg.norm(np.linalg.inv(init)@np.linalg.inv(lq)@S@matrix(lc)-np.eye(4))<1e-9
    # Only the saved Stage12D Full75 batch solution is offered as an explicitly separate FINAL reference.
    batchpath=D12/'clean_all75_gtsam_trajectory.csv'; _,batchrows=trajectory(batchpath)
    # Reuse exact corresponding batch-final Stage13B maps, never sparse K40 maps.
    bm=json.loads(source(B13/'snapshots/FINAL_STREAM_END_BATCH_map_manifest.json','identical final Full75 batch maps').read_text())
    batchmaps={a['robot_source']:ply(a['path']) for a in bm['artifacts'] if a.get('robot_source') in ['robot1','robot3']}
    # Verify final checkpoint batch coordinates reproduce the Stage12D reference (no alignment).
    bref=pd.read_csv(source(B13/'snapshots/FINAL_STREAM_END_batch_trajectory.csv','final identical-graph batch reproduction'),usecols=POSE)
    b12=pd.read_csv(batchpath,usecols=POSE)
    assert np.max(abs(bref[['tx','ty','tz']].to_numpy()-b12[['tx','ty','tz']].to_numpy()))<1e-6
    dump(DATA/'BATCH_FINAL.json',dict(name='FULL75_BATCH_REFERENCE',kf=1999,K=75,nodes=5796,frame='ROBOT1_KF150_GAUGE',trajectory=batchrows,maps=batchmaps))
    parameters=json.loads(source(B13/'isam2_parameter_policy.json','frozen Stage13B solver policy').read_text())
    assert parameters['parameters']['relinearizeSkip']==10 and parameters['parameters']['wildfireThreshold']==.001
    dump(DATA/'replay.json',dict(accepted_checkpoint=ACCEPTED,events=events,local_trajectory=localrows,local_robot1_chunks=chunks,snapshots=snapshots,common_initial_trajectory=commonrows,reference_initialization=S.tolist(),trace_columns=tracecols,trace=tracevalues,parameters=parameters['parameters'],final_counts=dict(nodes=5796,odometry=5794,loops=75,prior=1,total=5870),GT_used=False,numerical_diagnostic_line_closed=True))
    assert all(sha(p)==h for p,h in history.items()) and all(sha(p)==h for p,h in decision_hashes.items())
    dump(OUT/'demo_data_provenance.json',dict(accepted_checkpoint=ACCEPTED,sources=list(SOURCES.values()),historical_demo_hashes=history,frozen_decision_hashes=decision_hashes,frontend_columns_loaded=frontcols,keyframe_columns_loaded=['keyframe_id','lidar_file'],GT_loaded=False,optimizer_runs=0,point_xyz_retained=True,render_subset='deterministic stride; coordinates rounded to millimetres for map display only',map_protocol=manifest['protocol'],estimate_policy='step-held latest real saved Stage13B checkpoint; no morph; streaming local panel separate',batch_reference='Stage12D Full75; independently reproduced identical final batch map; no alignment',all_clean75_used=True,excluded45_reintroduced=False))
    append_common_map()
    print('STAGE18 ASSETS READY',len(events),'events',len(snapshots),'real checkpoints; optimizer runs 0',flush=True)

def append_common_map():
    p=json.loads((OUT/'demo_data_provenance.json').read_text()); SOURCES.update({x['path']:x for x in p['sources']})
    common_initial_map(); p['sources']=list(SOURCES.values());p['common_initial_map']='exact first-loop restricted prefix; raw frame10/point16/range(1,60) reprojection then per-source global voxel0.5m; no solve'
    dump(OUT/'demo_data_provenance.json',p)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--add-common-frame-map',action='store_true');args=parser.parse_args()
    if args.add_common_frame_map:
        assert not (DATA/'COMMON_INITIAL.json').exists(),'Refuse common-map overwrite'
        append_common_map();print('COMMON FRAME MAP READY; no solver rerun')
    else: main()
