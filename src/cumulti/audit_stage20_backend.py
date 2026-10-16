"""Independent saved-trace/availability audit; no solver, GT or frontend writes."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np,pandas as pd
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/cumulti_v1/20_dual_stream'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def availability(out, frames=None):
    """Deterministic sensor availability annotations, not new sensor observations."""
    p=out/'local_graph_progress_by_robot.csv';x=pd.read_csv(p,dtype={'original_timestamp_ns':'int64'})
    manifest=ROOT/'demo/stage19_rgb_three_map/rgb_manifest.json';m=json.loads(manifest.read_text())
    x['rgb_available']=False;x['rgb_original_timestamp_ns']='';x['rgb_frame_file']='';x['rgb_age_ms']=np.nan
    for robot in ['robot1','robot3']:
        rows=m['frames'][robot];ts=np.array([int(r['rgb_timestamp_ns']) for r in rows],np.int64)
        for i,r in x[x.robot_id.eq(robot)].iterrows():
            j=int(np.searchsorted(ts,int(r.original_timestamp_ns),side='right'))-1
            if j>=0:
                x.loc[i,'rgb_available']=True;x.loc[i,'rgb_original_timestamp_ns']=str(int(ts[j]));x.loc[i,'rgb_frame_file']=rows[j]['file'];x.loc[i,'rgb_age_ms']=(int(r.original_timestamp_ns)-int(ts[j]))/1e6
    x['rgb_policy']='original own-run bag clock causal floor; Stage19 Robot3 coverage is SPARSE'
    x.to_csv(p,index=False)
    metadata=dict(stage19_manifest_sha256=sha(manifest),read_columns=['keyframe_id','lidar_timestamp_ns','lidar_file'],keyframe_tables=[],GT_columns_decoded=False)
    frozen=json.loads((ROOT/'outputs/cumulti_v1/18_progressive_demo/demo_data_provenance.json').read_text())
    for robot in ['robot1','robot3']:
        kp=Path(f'/home/cas/CU-Multi/processed_v1/full_2hz/{robot}/keyframes.csv');h=sha(kp)
        expected=[s['sha256'] for s in frozen['sources'] if s['path']==str(kp)]
        assert expected and all(v==h for v in expected)
        metadata['keyframe_tables'].append(dict(path=str(kp),sha256=h,Stage18_hash_match=True))
    (out/'sensor_availability_provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')

def audit(out):
    checks={};decision=json.loads((out/'stage20a_backend_decision.json').read_text())
    assert decision['status']=='DUAL_STREAM_BACKEND_READY',decision
    t=pd.read_csv(out/'central_incremental_event_trace.csv').fillna({'new_loop_ids':''})
    sched=pd.read_csv(out/'dual_stream_event_schedule.csv');lo=pd.read_csv(out/'loop_eligibility_schedule.csv')
    local=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv',usecols=['robot_id','keyframe_id','timestamp'])
    expected=[];positions={}
    for robot,start,end in [('robot1',150,1999),('robot3',234,4179)]:
        a=local[local.robot_id.eq(robot)].sort_values('keyframe_id');assert a.keyframe_id.tolist()==list(range(start,end+1))
        origin=int(a.timestamp.iloc[0]);checks[robot+'_retained_count']=len(a)==end-start+1
        for row in a.itertuples():expected.append((int(row.timestamp)-origin,robot,int(row.keyframe_id),int(row.timestamp)))
    expected.sort();checks['exact_event_schedule']=list(zip(sched.normalized_ns,sched.robot_id,sched.keyframe_id,sched.timestamp_ns))==expected
    for i,(_,r,k,_) in enumerate(expected):positions[(r,k)]=i
    raw=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv')
    expected_loops=sorted([(max(positions[('robot1',int(r.query_keyframe_id))],positions[('robot3',int(r.candidate_keyframe_id))]),int(r.frozen_loop_id)) for r in raw.itertuples()])
    checks['exact_loop_eligibility_order']=list(zip(lo.release_event_index,lo.frozen_loop_id))==expected_loops
    arrived={'robot1':0,'robot3':0};last={'robot1':149,'robot3':233};seen=set();first=None
    for i,row in enumerate(t.itertuples()):
        r=row.robot_id;arrived[r]+=1;last[r]+=1
        assert row.keyframe_id==last[r] and row.robot1_nodes==arrived['robot1'] and row.robot3_nodes==arrived['robot3']
        eligible={fid for event,fid in expected_loops if event<=i};new={int(v) for v in str(row.new_loop_ids).split(';') if v}
        assert new==eligible-seen and not seen&new;seen|=new
        assert row.cumulative_loops==len(seen) and row.current_robot1_kf==last['robot1'] and row.current_robot3_kf==last['robot3']
        if seen:
            if first is None:first=i
            assert row.central_nodes==sum(arrived.values()) and row.central_odometry==sum(arrived.values())-2
            assert row.central_factors==sum(arrived.values())-1+len(seen) and row.components==1
        else:assert row.central_nodes==0 and row.central_factors==0 and row.components==sum(v>0 for v in arrived.values())
    checks['all_event_prefixes_no_future_exact_once']=True
    f=json.loads((out/'first_bridge_fusion.json').read_text());expected_first_nodes=int(t.iloc[first].robot1_nodes+t.iloc[first].robot3_nodes);checks['first_bridge_not_prebuilt']=f['event_index']==first and f['joint_nodes']==expected_first_nodes and f['joint_nodes']!=4080 and f['components_before']==2 and f['components_after']==1
    checks['final_graph_exact']=json.loads((out/'final_graph_equivalence.json').read_text())['PASS']
    manifest=json.loads((out/'stage20_input_manifest.json').read_text());checks['historical_hashes_unchanged']=all(sha(p)==h for p,h in manifest['protected_hashes'].items())
    for p in manifest['inputs']:assert sha(p['path'])==p['sha256']
    integ=pd.read_csv(out/'trajectory_integrity_report.csv');checks['no_catastrophic_jump']=int(integ.new_catastrophic_steps.sum())==0
    checks['synthetic_tests']=json.loads((out/'scheduler_unit_tests.json').read_text())['status']=='PASS'
    av=pd.read_csv(out/'local_graph_progress_by_robot.csv',dtype={'original_timestamp_ns':'int64','rgb_original_timestamp_ns':'Int64'})
    checks['local_graph_progress']=bool((av.local_nodes==av.local_odometry+1).all() and (av.local_graph_factors==av.local_odometry+1).all())
    checks['RGB_no_future']=bool((av[av.rgb_available].rgb_original_timestamp_ns<=av[av.rgb_available].original_timestamp_ns).all())
    states=json.loads((out/'backend_state_manifest.json').read_text())
    for state in states:
        assert sha(state['trajectory'])==state['sha256'];tr=pd.read_csv(state['trajectory'])
        assert len(tr)==state['robot1_kf']-149+state['robot3_kf']-233
        assert np.isfinite(tr[['tx','ty','tz','qx','qy','qz','qw']]).all().all()
        for robot,end in [('robot1',state['robot1_kf']),('robot3',state['robot3_kf'])]:assert tr[tr.robot_id.eq(robot)].keyframe_id.max()==end
        for art in state['maps']:assert sha(art['path'])==art['sha256']
    checks['snapshots_finite_prefix_only']=True
    result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,events=len(t),loops=len(seen),first10_loop_ids=lo.frozen_loop_id.head(10).tolist())
    (out/'stage20a_independent_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    assert result['status']=='PASS'

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=OUT);p.add_argument('--annotate',action='store_true');a=p.parse_args()
    if a.annotate:availability(a.output)
    audit(a.output)
