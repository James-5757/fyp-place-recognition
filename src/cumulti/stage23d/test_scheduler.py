"""Synthetic scheduler tests only; fixtures are NOT scientific GICP outcomes."""
import tempfile
import scheduler as s
from common import *

def main():
    tests={};real_A=s.A
    with tempfile.TemporaryDirectory(prefix='stage23d_synthetic_') as temp:
        s.A=Path(temp);(s.A/'attempts').mkdir()
        e=s.Engine.__new__(s.Engine);e.fixed=np.zeros(5796,dtype=np.int64);e.cap=dict(calls=20,bytes=20000000,roll_calls=1,roll_bytes=5000000,max_cpu=1)
        e.plan=dict(max_age_events=2,max_queue_queries=2);e.p=dict(predicted_cpu_ms={r+':'+str(k):1 for r in ['robot1','robot3'] for k in [1,2,3]})
        e.events=[dict(event_index=i,robot_id='robot1' if i%2==0 else 'robot3',keyframe_id=(150 if i%2==0 else 234)+i//2) for i in range(8)]
        e.bins={(x['robot_id'],x['keyframe_id']):0 for x in e.events}
        def row(event,rank=1):
            v=e.events[event];return dict(query_robot=v['robot_id'],query_keyframe_id=v['keyframe_id'],candidate_robot='robot3' if v['robot_id']=='robot1' else 'robot1',candidate_keyframe_id=234 if v['robot_id']=='robot1' else 150,candidate_rank=rank,SC_score=.6)
        def receipt(event,rank,accept):
            path=s.A/'attempts'/f'e{event:04d}_r{rank}.json';raw=path.with_suffix('.csv');raw.write_text('SYNTHETIC UNIT FIXTURE, NOT GICP\n');dump(path,dict(measurement=dict(row(event,rank),geometrically_accepted=accept,canonical_transform=np.eye(4).tolist(),result_csv_sha256=sha(raw),call_cpu_ms=1)))
        e.groups={1:[row(1),row(1,2)],2:[row(2)],3:[row(3)]};receipt(1,1,True)
        fast=e.run('SYNTHETIC_FAST',3);slow=e.run('SYNTHETIC_SLOW',3,cache_contexts=False)
        strip=lambda tr:[{k:v for k,v in x.items() if k!='policy'} for x in tr]
        tests['cached_contexts_exactly_match_original_scan_semantics']=strip(fast[1])==strip(slow[1])
        tests['accepted_rank1_never_reads_unknown_rank2']=fast[0]['completed_receipt_decisions']==1 and fast[0]['unknown_outcomes']==0
        tests['rolling_cap_blocks_then_queue_expires']=fast[0]['queue_expirations']>0 and any(x['reason']=='ROLLING_CALL_CAP' for x in fast[1] if x['action']=='DEFERRED')
        (s.A/'attempts/e0001_r1.json').unlink();e.groups={1:[row(1)]};unknown=e.run('SYNTHETIC_UNKNOWN',1)
        tests['unknown_selected_outcome_halts_without_imputation']=unknown[0]['status']=='PARTIAL' and unknown[0]['completed_receipt_decisions']==0 and len(unknown[2])==1
        e.groups={1:[dict(row(1),candidate_keyframe_id=999)]}
        try:e.run('SYNTHETIC_FUTURE',1);tests['future_endpoint_rejected']=False
        except AssertionError:tests['future_endpoint_rejected']=True
        e.groups={1:[row(1)]};e.cap['bytes']=1;blocked=e.run('SYNTHETIC_BYTES',1)
        tests['hard_global_byte_cap_never_overridden']=blocked[0]['completed_receipt_decisions']==0 and blocked[0]['application_bytes']==0
    s.A=real_A;dump(OUT/'scheduler_synthetic_tests.json',dict(status='PASS' if all(tests.values()) else 'FAIL',tests=tests,scope='synthetic control-flow fixtures only; no executed GICP or scientific factor'))
    assert all(tests.values()),tests
    print('SYNTHETIC_SCHEDULER_TESTS_PASS',flush=True)
if __name__=='__main__':main()
