"""Stage22A isolated policy/provenance and causal primitives (no GT reads)."""
import contextlib, hashlib, json, os, subprocess, sys, time
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from stage20_schedule import schedule
import run_stage8_efficient_sc_retrieval as s8
import run_stage2_sc_baseline as s2

ROOT=Path('/home/cas/fyp_place_recognition')
OUT=Path(os.environ.get('CUMULTI_STAGE22A_OUTPUT_ROOT',ROOT/'outputs/cumulti_v1/22a_online_frontend'))
PROC=Path('/home/cas/CU-Multi/processed_v1/full_2hz')
STARTS={'robot1':150,'robot3':234}
TEAM=Path('/home/cas/fyp_robot13_geometry_20260915/code/robot13_gicp.py')
BASE='70512f89a44ec887512624a717a6df312a79d9b9'

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()

def dump(name,value):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp')
    with tmp.open('w') as f:
        json.dump(value,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(tmp,p)

def read(name):return json.loads((OUT/name).read_text())
def csv(name,rows):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    (rows if isinstance(rows,pd.DataFrame) else pd.DataFrame(rows)).to_csv(p,index=False,lineterminator='\n')
def timed(fn):
    t=time.perf_counter_ns();v=fn();return v,(time.perf_counter_ns()-t)/1e6
def frames():
    return {r:pd.read_csv(PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).query('keyframe_id>=@start').set_index('keyframe_id',drop=False) for r,start in STARTS.items()}
def events_for(f):return schedule({r:list(zip(x.keyframe_id.map(int),x.lidar_timestamp_ns.map(int))) for r,x in f.items()},[])[0]
def inv(T):
    v=np.eye(4);v[:3,:3]=T[:3,:3].T;v[:3,3]=-v[:3,:3]@T[:3,3];return v
def canonical(qrobot,q,c,T):return (q,c,T) if qrobot=='robot1' else (c,q,inv(T))
def duplicate(q,c,accepted):return any(abs(q-a)<=3 and abs(c-b)<=5 for a,b in accepted)

class Database:
    """Only append actually computed arrived descriptors; rebuild KD-tree per query."""
    def __init__(self,capacity=4180):
        self.ids=[];self.keys=np.empty((capacity,20),np.float32)
        self.u=np.empty((capacity,20,60),np.float32);self.valid=np.empty((capacity,60),bool)
    def insert(self,k,d):
        assert k not in self.ids
        i=len(self.ids);self.ids.append(k);self.keys[i]=s8.ring_key(d[None])[0]
        u,v=s8.s5.normalize_columns(d[None]);self.u[i]=u[0];self.valid[i]=v[0]
    def query(self,d):
        n=len(self.ids)
        if not n:return [],dict(database_build_or_update_ms=0.,Ring_Key_search_ms=0.,exact_SC_scoring_ms=0.,total_retrieval_ms=0.)
        t=time.perf_counter_ns();tree,build=timed(lambda:cKDTree(self.keys[:n]))
        idx,search=timed(lambda:np.atleast_1d(tree.query(s8.ring_key(d[None])[0],k=min(1000,n),workers=1)[1]).astype(int))
        assert len(idx)==min(n,1000) and np.all((idx>=0)&(idx<n))
        (scores,shifts),sc=timed(lambda:s8.score(d,self.u[idx],self.valid[idx]))
        order=np.argsort(-scores,kind='stable')[:min(3,n)]
        result=[dict(candidate_keyframe_id=int(self.ids[idx[j]]),candidate_rank=i+1,SC_score=float(scores[j]),best_shift=int(shifts[j]),yaw_initialization_deg=float((int(shifts[j]) if shifts[j]<=30 else int(shifts[j])-60)*6)) for i,j in enumerate(order)]
        return result,dict(database_build_or_update_ms=build,Ring_Key_search_ms=search,exact_SC_scoring_ms=sc,total_retrieval_ms=(time.perf_counter_ns()-t)/1e6)

def online_guard():
    """Process-lifetime audit hook: forbid historical loop/result/GT file access."""
    forbidden=['gt_utm','gt_rel','ordered_rank1_clean_loops','candidate_interface','rank1_sanitized_edges','09_sc_gicp_integration/','10_2_pgo_policy_selection/','21_cost_baseline/']
    def hook(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            p=os.fsdecode(args[0]).lower()
            if any(s in p for s in forbidden):raise PermissionError('Stage22A online oracle access forbidden: '+p)
    sys.addaudithook(hook)

def verify_freeze():
    lock=read('policy_lock.json')
    for n,h in lock['files'].items():assert sha(OUT/n)==h,('frozen policy changed',n)
    for p,h in read('stage22a_input_manifest.json')['inputs'].items():assert sha(p)==h,('input changed',p)

def prepare():
    assert not OUT.exists(),'Fresh Stage22A output directory required'
    subprocess.run(['git','merge-base','--is-ancestor',BASE,'HEAD'],cwd=ROOT,check=True)
    OUT.mkdir(parents=True);f=frames();events=events_for(f);assert len(events)==5796
    source=['run_stage2_sc_baseline.py','run_stage5_visual_viewpoint.py','run_stage8_efficient_sc_retrieval.py','run_stage9_sc_gicp_integration.py','stage20_schedule.py','run_stage20_dual_stream.py','stage21_wire.py']
    inputs=[ROOT/'src/cumulti'/n for n in source]+[TEAM]
    inputs += [PROC/r/n for r in STARTS for n in ['keyframes.csv','scan_context_descriptors.npy']]
    inputs += [ROOT/'outputs/cumulti_v1/12d_clean_backend'/n for n in ['clean_local_trajectory.csv','clean_graph_policy.json','clean_gtsam_solver_policy.json']]
    inputs += [ROOT/'outputs/cumulti_v1/12c_dataset_stability/stable_segment_selection.json',ROOT/'outputs/cumulti_v1/13b_incremental_isam2/isam2_parameter_policy.json']
    for p in inputs:
        if p.is_relative_to(ROOT):
            blob=subprocess.check_output(['git','show',BASE+':'+str(p.relative_to(ROOT))],cwd=ROOT)
            assert hashlib.sha256(blob).hexdigest()==sha(p),('frozen baseline input mismatch',p)
    protected=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().strip('\0').split('\0')
    hashes={p:sha(ROOT/p) for p in protected if p.startswith(('src/','scripts/','outputs/','demo/')) and (ROOT/p).is_file()}
    scans=[]
    for r,x in f.items():
        for row in x.itertuples():
            p=PROC/r/'lidar'/row.lidar_file;assert p.is_file()
            scans.append(dict(robot=r,keyframe=int(row.keyframe_id),path=str(p),bytes=p.stat().st_size,sha256=sha(p)))
    csv('lidar_input_manifest.csv',scans)
    dump('stage22a_input_manifest.json',dict(baseline=BASE,inputs={str(p):sha(p) for p in inputs},lidar_manifest_sha256=sha(OUT/'lidar_input_manifest.csv'),protected_hashes=hashes,counts={r:len(x) for r,x in f.items()},timestamp_columns_read=['keyframe_id','lidar_timestamp_ns','lidar_file'],node_frame='LiDAR/os_sensor; frozen KF0-relative non-GT EKF+URDF convention; local poses not rebased',GT_used=False))
    pilot=np.linspace(0,5795,150,dtype=int).tolist()
    policy=dict(M=1000,TopK=3,event_order=['normalized_ns','robot_id','keyframe_id'],clock='original timestamp minus own first retained timestamp; separate recordings, hypothetical simultaneous start',online_order='load/compute/query opposite/publish/append own',KD_maintenance='rebuild current opposite database each query; measured',tie_break='frozen cKDTree deterministic input order and stable descending SC score sort',descriptor_integrity='bitwise equals frozen descriptor for every arriving scan; never prepopulate',retrieval_repeat='two full independent real-scan causal passes; compare scientific candidate fields, not timings',GICP=dict(voxel=.75,min_range=2,max_range=50,min_height=-3,max_height=5,coarse=3,fine=1,iterations=30,quality=.6091,cache_size=64,method='unchanged actual teammate.run one-pair calls; rank-ordered early stop'),duplicate='first qualifying canonical R1/R3 pair; suppress iff |delta R1 KF|<=3 AND |delta R3 KF|<=5 versus an already admitted representative; no future replacement',graph_admission='geometric acceptance + finite proper SE3 (rotation orthogonality/determinant tolerance 1e-6) + retained endpoints + causal first-neighborhood representative; no GT/extra score threshold',graph_safety='frozen12D catastrophic jump screen each persistent update; stop backend and mark NOT_READY on failure; never substitute archived loops',graph_noise='frozen12D odom 5deg/1m diagonal, loops same + Huber1.345; R1KF150 prior 1e-6; default frozen13B ISAM2; no extra updates',pilot_event_indices=pilot,pilot_reuse='new Stage22A pilot registrations reused at their causal logical event in full chronological admission; hash checked; pilot never admits factors; each event/rank actual call exactly once',resume='write durable intent before actual call; atomic result+timing after; unresolved intent blocks rather than silently rerunning; policy/input hashes verified',communication='baseline B1 full-SC-first central descriptor cache, no descriptor request required; on-demand exact XYZI cloud cache unbounded; no cache/selector tuning; actual encoded buffers, simulated sends, network NOT_MEASURED',threads={k:os.environ.get(k) for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']},GT_online=False,archive_online=False,optimization_after_GT=False)
    dump('stage22a_policy.json',policy)
    dump('stage22a_execution_budget.json',dict(full_GICP_execution_cap_seconds=7200,cap_includes_actual_pilot_and_full_attempt_wall_seconds=True,pilot_queries=150,pilot_selection='linspace(0,5795,150,dtype=int) before outcomes',pilot_projection_multiplier=5796/150,cap_change_allowed=False,RSS_limit_GiB=8,decision='run full only if pilot projected actual harness wall <=7200s',available_disk=subprocess.check_output(['df','-h',str(PROC)]).decode()))
    csv('dual_stream_event_schedule.csv',events)
    dump('policy_lock.json',dict(files={n:sha(OUT/n) for n in ['stage22a_input_manifest.json','stage22a_policy.json','stage22a_execution_budget.json','lidar_input_manifest.csv','dual_stream_event_schedule.csv']}))
    print('STAGE22A_POLICY_FROZEN',flush=True)

if __name__=='__main__':prepare()
