"""Stage23D isolated paths and immutable input ledger; no scientific retuning."""
import hashlib,json,os,sys,time,resource,subprocess,gzip
from pathlib import Path
import numpy as np
import pandas as pd
ROOT=Path('/home/cas/fyp_place_recognition')
BASE=ROOT/'outputs/cumulti_v1'
A=BASE/'22a_online_frontend'; B=BASE/'22b_budget_aware'; OLD=BASE/'23_information_aware'
MAP=BASE/'23d_policy_map_compare'; OUT=BASE/'23d_budget_scheduler'
DEMO=ROOT/'demo/stage23d_policy_map_compare'
PROC=Path('/home/cas/CU-Multi/processed_v1/full_2hz')
P0='P0_FULL_TOP3';P5='P5_BUDGET_AWARE_C75_B80'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(4*1024**2),b''):h.update(b)
    return h.hexdigest()
def dump(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n');os.replace(tmp,p)
def csv(p,x,columns=None):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    (x if isinstance(x,pd.DataFrame) else pd.DataFrame(x,columns=columns)).to_csv(p,index=False,lineterminator='\n')
def protect_verify():
    frozen=read(MAP/'comparison_input_manifest.json')
    for name,v in frozen['inputs'].items():assert sha(name)==v['sha256'],name
    archive=frozen['historical_integrity_inventory']
    assert sha(archive['path'])==archive['sha256']
    for name,v in json.loads(gzip.decompress(Path(archive['path']).read_bytes())).items():assert sha(name)==v['sha256'],name
    assert sha(OUT/'scheduler_predeclared_policy.json')==frozen['scheduler_policy_sha256']
def guard():
    def hook(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            s=os.fsdecode(args[0]).lower()
            if any(t in s for t in ['gt_utm','gt_rel','offline_','/23_information_aware/','/23c_information_validity/']):
                raise PermissionError('Causal scheduler forbidden input: '+s)
    sys.addaudithook(hook)
def freeze():
    assert not MAP.exists() and not OUT.exists(),'Stage23D is fresh-only: preserve existing work'
    MAP.mkdir(parents=True);OUT.mkdir(parents=True);DEMO.mkdir(parents=True,exist_ok=True)
    protected=[]
    for d in BASE.iterdir():
        if d.is_dir() and d.name.startswith(('18_','18_1','19_','20_','21_','22a_','22b_','23_','23c_')):
            protected.extend(p for p in d.rglob('*') if p.is_file() and '__pycache__' not in str(p))
    tracked=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
    protected += [ROOT/p for p in tracked if p.startswith(('demo/','src/cumulti/stage22','src/cumulti/stage23/','src/cumulti/stage23c/','src/demo/')) or p in ['PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md','AGENTS.md']]
    protected += [ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv']
    scans={}
    for robot,start,end in [('robot1',150,2000),('robot3',234,4180)]:
        files={int(p.name.split('_')[0]):p for p in (PROC/robot/'lidar').glob('*.npy')}
        for k in range(start,end,10):scans[robot+':'+str(k)]=str(files[k]);protected.append(files[k])
    protected += [ROOT/'demo/final_replay/app.js',BASE/'10_3_solver_audit/nontrivial_small_graph_sanity.json']
    policy=dict(name='P6_DEFERRED_LEXICOGRAPHIC',frozen_reference='5ed035d891053c93a22321fb6a809f849ceb20d1',max_queue_queries=256,max_age_events=100,window_events=100,compute_fraction=.75,communication_fraction=.8,cache_capacity='unbounded SAME as frozen P5',cloud_request_response_bytes=1048681,loop_bytes=249,coverage='own admitted canonical endpoint pair bins from frozen non-GT local odometry arc length /50m',coverage_bin_m=50,priority_classes=['no admitted bridge yet','previously uncovered endpoint-bin pair','cached or lower transfer cost','remaining rank/confidence'],tie_break='class, optional direction attempts/arrivals, transfer, original rank, descending confidence, discovery event, canonical IDs',eligibility='finite original SC candidates, both endpoints arrived; sequential original ranks only; no GT/quality gate',ablations=['P5_FROZEN','P6_D0_IMMEDIATE_CONTROL','P6_D1_DEFERRED_ONLY','P6_D2_DEFERRED_COVERAGE','P6_D3_DEFERRED_COVERAGE_DIRECTION'],D0_D1='same non-weighted eligibility and FIFO order; D0 drops at event end, D1 queues',D2='same as D1 plus explicit bridge/coverage/cached lexicographic priority',D3='same as D2 plus least-served direction attempts/arrivals within a priority class; not a weighted score',P5_comparison_caveat='D1 versus P5 changes weighted eligibility too; D0 isolates queue benefit within P6 family',duplicate_rule='drop a new query whose current canonical identity is already pending; after resolution allow fresh original receipt attempts, suppress graph neighborhoods exactly delta3/delta5',rank_dependency='only rank1 initially; after measured failure unlock next original rank; any geometric success closes query even if graph-duplicate; expiration/invalid/drop closes remaining ranks',unknown_rule='commit selection before receipt access; missing receipt halts that policy at first unresolved decision, never continue with guessed state',budget='same frozen cumulative calls and application bytes, last100 call/byte caps, fixed full-stream publication reserved;249 bytes reserved pre-attempt; released if not graph-admitted',CPU_cap='historical hard compute unit is calls; additional predicted-ms caps = call caps * max frozen predicted per-call ms, identical for all P6 variants and redundant, NOT a new physical CPU-time guarantee',queue_parameters='engineering hypotheses frozen before running, not GT tuned',new_GICP_calls=0,new_backend_solves=0,no_GT_policy_selection=True)
    dump(OUT/'scheduler_predeclared_policy.json',policy)
    manifest=dict(reference_commit=policy['frozen_reference'],inputs={str(p):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(set(protected))},selected_scans=scans,scheduler_policy_sha256=sha(OUT/'scheduler_predeclared_policy.json'),existing_dirty_worktree=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),GT_presentation_transform=False,map_protocol=dict(frame_stride=10,point_stride=16,voxel_m=.5,range_m=[1,60],display_cap_per_robot=25000),progressive='PROGRESSIVE_STATE_EXPORT_REQUIRED')
    dump(MAP/'comparison_input_manifest.json',manifest)
    compact_manifest()
    print('FROZEN',len(manifest['inputs']),'files',len(scans),'raw scans',flush=True)
def compact_manifest():
    # Preserve every pre-evaluation digest server-side; version only the compact
    # essential-input manifest and digest of this comprehensive inventory.
    path=MAP/'comparison_input_manifest.json';m=read(path);all_inputs=m['inputs'];archive=MAP/'historical_integrity_inventory.json.gz'
    archive.write_bytes(gzip.compress(json.dumps(all_inputs,sort_keys=True).encode(),mtime=0))
    keep={str(p) for p in [A/'dual_stream_event_schedule.csv',A/'online_candidate_trace.csv',A/'online_gicp_attempts.csv',A/'online_protocol_message_trace.csv',B/'stage22b_experiment_policy.json',B/'stage22b_budget_policy.json',B/'finalist_lock.json',B/'offline_map_accuracy_by_policy.csv',B/'policy_comparison_full.csv',B/'policy_results_by_budget.csv',OLD/'stage23b_policy.json',OLD/'stage23b_GT_free_loop_metadata.csv',BASE/'12d_clean_backend/clean_local_trajectory.csv']}
    keep.update(m['selected_scans'].values())
    for policy in [P0,P5]:keep.update(str(p) for p in (B/'policies'/policy).glob('*') if p.is_file());keep.update(str(p) for p in (B/'policies'/policy/'maps').glob('*.ply'))
    m['inputs']={p:v for p,v in all_inputs.items() if p in keep}
    m['historical_integrity_inventory']=dict(path=str(archive),sha256=sha(archive),bytes=archive.stat().st_size,protected_file_count=len(all_inputs),scope='all pre-evaluation hashes; server-only comprehensive inventory, never omitted from verification')
    dump(path,m)
if __name__=='__main__':freeze()
