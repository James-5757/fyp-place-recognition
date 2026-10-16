"""Standalone synthetic scheduler tests (not substitutes with real CU-Multi)."""
import json
from stage20_schedule import schedule,replay_registry

def run():
    s={'robot1':[(10,1000),(11,1010),(12,1020),(13,1030)],'robot3':[(20,9000),(21,9010),(22,9020),(23,9050)]}
    def loop(i,q,c):return dict(frozen_loop_id=i,query_robot='robot1',query_keyframe_id=q,candidate_robot='robot3',candidate_keyframe_id=c)
    ls=[loop(1,11,22),loop(2,12,20),loop(3,12,22),loop(4,13,23),loop(5,10,23)]
    e,l=schedule(s,ls);trace=replay_registry(e,l);checks={}
    checks['different_absolute_starts_normalize_to_zero']=e[0]['normalized_ns']==e[1]['normalized_ns']==0
    checks['same_time_robot_tie_break']=[r['robot_id'] for r in e[:2]]==['robot1','robot3']
    checks['query_arrives_first']=next(x for x in l if x['frozen_loop_id']==1)['release_event_index']==5
    checks['candidate_arrives_first']=next(x for x in l if x['frozen_loop_id']==2)['release_event_index']==4
    checks['same_timestamp_group_waits_for_both']=next(x for x in l if x['frozen_loop_id']==3)['release_event_index']==5
    checks['multiple_loops_one_event']=trace[-1]['released']==[4,5]
    checks['robot1_finishes_first']=e[-1]['robot_id']=='robot3' and e[-2]['robot_id']=='robot1'
    checks['each_local_chain_grows']=all(x['nodes']==i+1 for i,x in enumerate(trace))
    checks['exact_once_no_future']=sum(len(x['released']) for x in trace)==5
    checks['components_2_to_1']=trace[3]['components']==2 and trace[4]['components']==1
    checks['deterministic_input_map_order']=schedule(dict(reversed(list(s.items()))),list(reversed(ls)))==(e,l)
    checks['final_semantic_factor_identity']=trace[-1]['nodes']==8 and trace[-1]['odometry']==6 and trace[-1]['loops']==5
    assert all(checks.values()),checks
    return dict(status='PASS',synthetic=True,checks=checks,events=e,loops=l,trace=trace)

if __name__=='__main__':print(json.dumps(run(),indent=2))
