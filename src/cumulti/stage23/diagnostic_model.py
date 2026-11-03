"""Stage23B frozen graph reconstruction; no GT, registration or historical writes."""
import hashlib,json,os,resource,sys,time
from pathlib import Path
import numpy as np,pandas as pd,gtsam
from scipy.spatial.transform import Rotation
ROOT=Path('/home/cas/fyp_place_recognition');SRC=ROOT/'src/cumulti';sys.path.insert(0,str(SRC))
OUT=ROOT/'outputs/cumulti_v1/23_information_aware';A=ROOT/'outputs/cumulti_v1/22a_online_frontend';B=ROOT/'outputs/cumulti_v1/22b_budget_aware'
P0='P0_FULL_TOP3';P5='P5_BUDGET_AWARE_C75_B80';SIG=np.array([np.deg2rad(5)]*3+[1.]*3)
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for x in iter(lambda:f.read(4*1024**2),b''):h.update(x)
    return h.hexdigest()
def dump(n,x):
    p=OUT/n;p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n');os.replace(tmp,p)
def read(p):return json.loads(Path(p).read_text())
def csv(n,x):
    (x if isinstance(x,pd.DataFrame) else pd.DataFrame(x)).to_csv(OUT/n,index=False,lineterminator='\n')
def key(n):return gtsam.symbol('a' if n[0]=='robot1' else 'b',int(n[1]))
def inv(T):
    V=np.eye(4);V[:3,:3]=T[:3,:3].T;V[:3,3]=-V[:3,:3]@T[:3,3];return V
def matrix(r):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([r.qx,r.qy,r.qz,r.qw]).as_matrix();T[:3,3]=[r.tx,r.ty,r.tz];return T
def identity(r):return int(r.canonical_robot1_kf),int(r.canonical_robot3_kf)
def noise():
    base=gtsam.noiseModel.Diagonal.Sigmas(SIG);return base,gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),base)
def load():
    loops={n:pd.read_csv(B/'policies'/n/'online_verified_loops.csv') for n in [P0,P5]}
    frame=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv');nodes=[(r.robot_id,int(r.keyframe_id)) for r in frame.itertuples()]
    local={n:matrix(r) for n,r in zip(nodes,frame.itertuples())};arc={}
    for robot,g in frame.groupby('robot_id'):
        s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(g[['tx','ty','tz']].to_numpy(),axis=0),axis=1))];arc.update({(robot,int(k)):float(v) for k,v in zip(g.keyframe_id,s)})
    events=pd.read_csv(A/'dual_stream_event_schedule.csv').to_dict('records');params=gtsam.LevenbergMarquardtParams();policy=read(B/'policies'/P0/'online_backend_policy.json')
    assert all(getattr(params,k)()==v for k,v in policy['LM'].items());assert np.array_equal(SIG,policy['noise_sigmas']) and policy['Huber']==1.345 and policy['prior_sigma']==1e-6
    return loops,frame,nodes,local,arc,events,params
def build(name,loops,nodes,local,events):
    base,robust=noise();rows=loops[name];first=rows.iloc[0];initial_Z=np.asarray(json.loads(first.canonical_transform));q,c=identity(first)
    S=local['robot1',q]@initial_Z@inv(local['robot3',c]);initial={n:local[n] if n[0]=='robot1' else S@local[n] for n in nodes}
    groups={int(i):g for i,g in rows.groupby('discovery_event')};start=int(first.discovery_event);arrived=set();graph=gtsam.NonlinearFactorGraph();index={}
    def odom(n):return gtsam.BetweenFactorPose3(key((n[0],n[1]-1)),key(n),gtsam.Pose3(inv(local[n[0],n[1]-1])@local[n]),base)
    for e in events:
        n=(e['robot_id'],e['keyframe_id']);arrived.add(n);i=e['event_index']
        if i<start:continue
        if i==start:
            graph.add(gtsam.PriorFactorPose3(key(('robot1',150)),gtsam.Pose3(local['robot1',150]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
            for x in sorted(arrived):
                if x[1]>(150 if x[0]=='robot1' else 234):graph.add(odom(x))
        else:graph.add(odom(n))
        for row in groups.get(i,pd.DataFrame()).itertuples():
            qc=identity(row);assert qc not in index;index[qc]=graph.size();graph.add(gtsam.BetweenFactorPose3(key(('robot1',qc[0])),key(('robot3',qc[1])),gtsam.Pose3(np.asarray(json.loads(row.canonical_transform))),robust))
    values=gtsam.Values()
    for n in nodes:values.insert(key(n),gtsam.Pose3(initial[n]))
    assert graph.size()==5795+len(rows) and values.size()==5796
    return graph,values,index
def matrices(v,nodes):return np.array([v.atPose3(key(n)).matrix() for n in nodes])
def to_values(a,nodes):
    v=gtsam.Values()
    for n,T in zip(nodes,a):v.insert(key(n),gtsam.Pose3(T))
    return v
def guard():
    def hook(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            p=os.fsdecode(args[0]).lower()
            if any(s in p for s in ['gt_utm','gt_rel','offline_','stage23_preflight_report','stage22b_missing_loop_audit','dataset_route_inventory','bidirectional_overlap_coverage','keyframes.csv']):raise PermissionError('GT-free diagnostic process forbids '+p)
    sys.addaudithook(hook)
