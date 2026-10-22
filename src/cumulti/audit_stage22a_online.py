"""Independent saved-artifact audit; never reruns SC/GICP/optimization."""
import json
from stage22a_common import *

def main():
    verify_freeze();checks=[]
    def check(name,ok,detail=None):
        checks.append(dict(name=name,PASS=bool(ok),detail=detail));assert ok,(name,detail)
    f=frames();events=events_for(f);seen={r:set() for r in STARTS};position={(e['robot_id'],e['keyframe_id']):e['event_index'] for e in events}
    a1=read('stage22a1_decision.json');trace=pd.read_csv(OUT/'online_candidate_trace.csv');groups={int(i):g for i,g in trace.groupby('event_index')};growth=pd.read_csv(OUT/'database_growth.csv').set_index('event_index')
    check('A1_complete',a1['status']=='CAUSAL_RETRIEVAL_READY' and len(events)==5796)
    summary=read('bidirectional_retrieval_summary.json');check('real_recompute_equivalence',summary['descriptor_bitwise_checks']==11592);check('two_pass_determinism',len(set(summary['two_pass_candidate_sha256']))==1)
    for e in events:
        i=e['event_index'];r=e['robot_id'];other='robot3' if r=='robot1' else 'robot1';g=groups.get(i,pd.DataFrame());expected=min(3,len(seen[other]));check('topK_'+str(i),len(g)==expected)
        for row in g.itertuples():
            check('candidate_causal_'+str(i)+'_'+str(row.candidate_rank),row.candidate_keyframe_id in seen[other] and position[(other,row.candidate_keyframe_id)]<i and row.current_R1_database_size==len(seen['robot1']) and row.current_R3_database_size==len(seen['robot3']) and int(row.candidate_original_timestamp_ns)==int(f[other].loc[row.candidate_keyframe_id,'lidar_timestamp_ns']))
        seen[r].add(e['keyframe_id']);check('growth_'+str(i),int(growth.loc[i,'robot1_size'])==len(seen['robot1']) and int(growth.loc[i,'robot3_size'])==len(seen['robot3']))
    a2=read('stage22a2_decision.json');coverage=read('frontend_execution_coverage.json');complete=a2['status']=='ONLINE_GICP_VERIFICATION_READY'
    check('coverage_status_honest',coverage['queries_verified']==5796 if complete else a2['status']=='FRONTEND_VERIFICATION_INCOMPLETE' and coverage['queries_verified']<5796)
    pilot_queries=pd.read_csv(OUT/'gicp_pilot_queries.csv');check('predeclared_pilot_event_IDs',pilot_queries.event_index.to_list()==read('stage22a_policy.json')['pilot_event_indices'])
    check('unchanged_execution_cap',read('stage22a_execution_budget.json')['full_GICP_execution_cap_seconds']==7200)
    attempts=pd.read_csv(OUT/'online_gicp_attempts.csv');check('attempt_identity_unique',not attempts.duplicated(['event_index','candidate_rank']).any())
    check('RSS_within_frozen_resource_budget',float(attempts.peak_RSS_mib.max())<=8192)
    check('actual_call_budget_between_attempts',float(attempts.call_wall_ms.sum())/1000<=7200+float(attempts.call_wall_ms.max())/1000)
    candidates={(int(r.event_index),int(r.candidate_rank)):r for r in trace.itertuples()};ledger_count=0
    for r in attempts.itertuples():
        i,rank=int(r.event_index),int(r.candidate_rank);c=candidates[(i,rank)];check('GICP_candidate_'+str(i)+'_'+str(rank),r.query_robot==c.query_robot and r.query_keyframe_id==c.query_keyframe_id and r.candidate_keyframe_id==c.candidate_keyframe_id and r.best_shift==c.best_shift)
        p=OUT/'attempts'/f'e{i:04d}_r{rank}.json';receipt=json.loads(p.read_text());check('actual_receipt_'+str(i)+'_'+str(rank),sha(p.with_suffix('.csv'))==receipt['measurement']['result_csv_sha256']);ledger_count+=1
        T=np.asarray(json.loads(r.verified_transform));C=np.asarray(json.loads(r.canonical_transform));expected=T if r.query_robot=='robot1' else np.linalg.inv(T)
        check('transform_'+str(i)+'_'+str(rank),np.max(abs(C-expected))<1e-8)
        check('quality_'+str(i)+'_'+str(rank),bool(r.geometrically_accepted)==(r.gicp_quality>=.6091) and abs(r.gicp_quality-r.gicp_fitness/(1+r.gicp_RMSE))<1e-10)
    for i,g in attempts.groupby('event_index'):
        ranks=g.candidate_rank.to_list();check('rank_early_stop_'+str(i),ranks==list(range(1,len(ranks)+1)) and not g.iloc[:-1].geometrically_accepted.any())
    physical_count=len(list((OUT/'attempts').glob('e*_r?.json')))
    check('exact_once_applied_and_physical_calls',ledger_count==coverage['actual_calls'] and (physical_count==ledger_count if complete else physical_count>=ledger_count))
    # A capped chronological prefix may leave pre-executed tail pilot receipts.
    # They remain measured pilot work, not claimed full-prefix verification.
    for p in (OUT/'attempts').glob('e*_r?.json'):
        receipt=json.loads(p.read_text());check('physical_receipt:'+p.stem,sha(p.with_suffix('.csv'))==receipt['measurement']['result_csv_sha256'])
    discovery=pd.read_csv(OUT/'online_loop_discovery_trace.csv');representatives=[]
    for r in discovery.itertuples():
        pair=(int(r.canonical_robot1_kf),int(r.canonical_robot3_kf));dup=any(abs(pair[0]-q)<=3 and abs(pair[1]-c)<=5 for q,c in representatives)
        check('causal_duplicate_'+str(r.discovery_event),dup==(r.duplication_status=='SUPPRESSED_CAUSAL_DUPLICATE'))
        if not dup:representatives.append(pair)
    check('new_loop_count',len(representatives)==coverage['independent_loops'])
    for rel,h in read('stage22a_input_manifest.json')['protected_hashes'].items():check('immutable:'+rel,sha(ROOT/rel)==h)
    scan_manifest=pd.read_csv(OUT/'lidar_input_manifest.csv');point_counts=[]
    for row in scan_manifest.itertuples():
        p=Path(row.path);cloud=np.load(p,mmap_mode='r',allow_pickle=False)
        check('raw_cloud_unchanged:'+row.robot+':'+str(row.keyframe),sha(p)==row.sha256 and p.stat().st_size==row.bytes)
        check('cloud_schema:'+row.robot+':'+str(row.keyframe),cloud.dtype==np.float32 and cloud.ndim==2 and cloud.shape[1]==4 and np.isfinite(cloud[:,:3]).all())
        point_counts.append(len(cloud))
    import run_stage10_offline_map_merge as s10
    expected=np.array([[-1.,0,0,-.06286],[0,-1.,0,.01557],[0,0,1.,.053345],[0,0,0,1.]])
    check('frozen_IMU_from_LiDAR_extrinsic',np.array_equal(s10.IMU_FROM_LIDAR,expected))
    dump('descriptor_input_audit.json',dict(status='PASS',scans=len(scan_manifest),fields=['x','y','z','intensity'],dtype='float32',points_min=min(point_counts),points_max=max(point_counts),all_lidar_SHA256_match=True,node_frame='LiDAR/os_sensor',IMU_from_LiDAR=expected.tolist(),GT_used=False))
    integrity=read('online_graph_integrity.json');a3=read('stage22a3_decision.json')
    if integrity['status']=='PASS':
        check('exact_new_final_graph',integrity['nodes']==5796 and integrity['odometry']==5794 and integrity['loops']==len(representatives) and integrity['factors']==5795+len(representatives))
        for name in ['online_incremental_trajectory.csv','online_batch_trajectory.csv']:
            t=pd.read_csv(OUT/name);check('trajectory_schema:'+name,len(t)==5796 and not t.duplicated(['robot_id','keyframe_id']).any() and np.isfinite(t[['tx','ty','tz','qx','qy','qz','qw']]).all().all() and np.max(abs(np.linalg.norm(t[['qx','qy','qz','qw']],axis=1)-1))<1e-8)
        diff=pd.read_csv(OUT/'online_incremental_vs_batch.csv').iloc[0];expected=diff.translation_median_m<=.1 and diff.translation_p95_m<=.5 and diff.rotation_median_deg<=.5 and diff.rotation_p95_deg<=2
        check('honest_numerical_decision',(a3['status']=='CAUSAL_ONLINE_GRAPH_READY')==bool(expected))
    else:check('honest_graph_failure_or_not_run',a3['status']==('CAUSAL_ONLINE_GRAPH_NOT_READY' if complete else 'NOT_RUN'))
    costs=pd.read_csv(OUT/'online_communication_cost.csv');messages=pd.read_csv(OUT/'online_protocol_message_trace.csv');check('protocol_sum',int(costs.serialized_application_bytes.sum())==int(messages.serialized_total_bytes.sum()))
    check('protocol_calls_include_all_actual_pilot_work',int(costs.actual_GICP_calls.sum())==physical_count);check('new_loop_messages',int(costs.new_loop_factors.sum())==len(representatives))
    offline=read('offline_evaluation_entry.json')
    for name,h in offline['decision_hashes'].items():check('no_postGT_retuning:'+name,sha(OUT/name)==h)
    check('unit_tests',read('synthetic_unit_tests.json')['status']=='PASS')
    csv('independent_audit_checks.csv',checks)
    dump('independent_validation.json',dict(status='PASS',checks=len(checks),scientific_graph_ready=a3['status']=='CAUSAL_ONLINE_GRAPH_READY',audit_PASS_is_not_backend_readiness=True,offline_decisions_unchanged=True))
    print('INDEPENDENT_AUDIT_PASS',len(checks),flush=True)
if __name__=='__main__':main()
