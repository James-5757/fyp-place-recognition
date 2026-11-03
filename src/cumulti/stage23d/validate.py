"""Independent trace/accounting audit. No policy selection or GT labels."""
from collections import deque
from common import *

def main():
    protect_verify();lock=read(OUT/'scheduler_trace_freeze.json')
    for n,h in lock['sha256'].items():assert sha(OUT/n)==h,n
    p=read(B/'stage22b_budget_policy.json');caps=dict(calls=int(.75*p['reference_calls']),bytes=int(.8*p['reference_application_bytes']),roll_calls=225,roll_bytes=int(100*p['reference_application_bytes']/5796*.8))
    messages=pd.read_csv(A/'online_protocol_message_trace.csv',usecols=['event_index','message_type','serialized_total_bytes']);fixed=messages[~messages.message_type.isin(['POINT_CLOUD_REQUEST','POINT_CLOUD_RESPONSE','LOOP_CONSTRAINT'])].groupby('event_index').serialized_total_bytes.sum().reindex(range(5796),fill_value=0).to_numpy(np.int64)
    events=pd.read_csv(A/'dual_stream_event_schedule.csv');arrived={(r.robot_id,int(r.keyframe_id)):int(r.event_index) for r in events.itertuples()};candidate=pd.read_csv(A/'online_candidate_trace.csv');canonical={}
    for r in candidate.itertuples():canonical[int(r.event_index),int(r.candidate_rank)]=(r.query_robot,int(r.query_keyframe_id),r.candidate_robot,int(r.candidate_keyframe_id))
    tr=pd.read_csv(OUT/'p6_queue_event_trace.csv');factors=pd.read_csv(OUT/'p6_verified_factors.csv');summaries=pd.read_csv(OUT/'p6_ablation_comparison.csv');native_policy=read(B/'stage22b_experiment_policy.json');cpu=native_policy['predicted_cpu_ms'];maxcpu=max(cpu.values());checks={};details=[]
    for policy,rows in tr.groupby('policy',sort=False):
        verified=rows[rows.action=='VERIFIED'];selected=rows[rows.action=='SELECTED'];assert len(verified)==len(selected),'Unknown actions must halt; this complete trace has none'
        cache=set();pairs=[];ledger=deque();variable=0;calls=0;history={};direction_factors={};maxrollbytes=0;maxrollcalls=0
        for r in verified.itertuples():
            i=int(r.event_index);e=int(r.discovery_event);rank=int(r.candidate_rank);qr,q,cr,c=canonical[e,rank];ks=[(qr,q),(cr,c)]
            assert e<=i and i-e<=100 and max(arrived[k] for k in ks)<=e
            previous=history.setdefault(e,[]);assert rank==len(previous)+1 and not any(previous),'Higher rank unlocked after success or rank skipped'
            receipt=A/'attempts'/f'e{e:04d}_r{rank}.json';m=read(receipt)['measurement'];assert sha(receipt.with_suffix('.csv'))==m['result_csv_sha256'];accept=m['geometrically_accepted'];assert bool(r.geometrically_accepted)==accept
            pair=(int(r.robot1_kf),int(r.robot3_kf));admit=accept and not any(abs(pair[0]-a)<=3 and abs(pair[1]-b)<=5 for a,b in pairs)
            transfer=1048681*sum(k not in cache for k in ks);extra=transfer+249*admit
            assert transfer==int(r.transfer_bytes) and extra==int(r.variable_bytes) and bool(r.graph_admitted)==admit
            cache.update(ks);previous.append(accept);calls+=1;variable+=extra
            if admit:
                pairs.append(pair);direction_factors[qr]=direction_factors.get(qr,0)+1
                Z=np.array(m['canonical_transform']);assert Z.shape==(4,4) and np.isfinite(Z).all() and np.max(abs(Z[3]-[0,0,0,1]))<1e-8 and abs(np.linalg.det(Z[:3,:3])-1)<1e-6 and np.max(abs(Z[:3,:3].T@Z[:3,:3]-np.eye(3)))<1e-6
                saved=factors[(factors.policy==policy)&(factors.discovery_event==e)&(factors['rank']==rank)];assert len(saved)==1;assert saved.iloc[0].receipt_sha256==sha(receipt);np.testing.assert_array_equal(np.array(json.loads(saved.iloc[0].canonical_transform)),Z)
            while ledger and ledger[0][0]<=i-100:ledger.popleft()
            ledger.append((i,extra,cpu[qr+':'+str(rank)]));rollbytes=int(fixed[max(0,i-99):i+1].sum())+sum(x[1] for x in ledger);rollcalls=len(ledger)
            assert rollcalls<=caps['roll_calls'] and rollbytes<=caps['roll_bytes'] and sum(x[2] for x in ledger)<=225*maxcpu+1e-7
            assert calls<=caps['calls'] and variable+fixed.sum()<=caps['bytes']
            maxrollbytes=max(maxrollbytes,rollbytes);maxrollcalls=max(maxrollcalls,rollcalls)
        summary=summaries[summaries.policy==policy].iloc[0];assert calls==summary.completed_receipt_decisions and variable+fixed.sum()==summary.application_bytes and len(pairs)==summary.admitted_factors
        assert np.all(np.diff(rows.event_index)>=0) and summary.peak_queue_queries<=256
        details.append(dict(policy=policy,status='PASS',verified_calls=calls,exact_application_bytes=int(variable+fixed.sum()),admitted_pairs=len(pairs),admitted_directions=direction_factors,max_rolling_calls=maxrollcalls,max_rolling_bytes=maxrollbytes,rank_dependencies_preserved=True,cache_reconstructed=True,no_future_endpoints=True))
    checks['all_four_P6_traces_independently_reconstructed']=len(details)==4
    checks['synthetic_control_flow_and_scan_equivalence']=read(OUT/'scheduler_synthetic_tests.json')['status']=='PASS'
    checks['frozen_P0_P5_unchanged_and_same_map_gauge']=read(MAP/'map_comparison_validation.json')['status']=='PASS'
    checks['selected_unknowns_never_imputed']=pd.read_csv(OUT/'unknown_gicp_extension_plan.csv').empty and lock['new_GICP_calls']==0
    recovery=read(OUT/'engineering_recovery.json');d0=details[0];checks['optimized_scan_preserves_completed_pre_recovery_D0']=d0['verified_calls']==recovery['pre_change_completed_D0']['calls'] and d0['exact_application_bytes']==recovery['pre_change_completed_D0']['application_bytes']
    validation=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,details=details,hard_limits=caps,cache='same unbounded central logical cache as frozen P5',rolling_CPU='predicted CPU ceiling, redundant with225-call cap; actual physical CPU latency not guaranteed',unknown_source_combinations_unchanged=2112,unknown_plan_empty_reason='sequential original rank dependencies stop after the same measured geometric success; no need to infer historically unmeasured higher ranks',GT_used=False,scope='causal scheduler correctness/accounting, NOT P6 map accuracy or independent validation')
    dump(OUT/'scheduler_causal_validation.json',validation);assert all(checks.values())
    # Known-vs-unknown receipt availability is descriptive, AFTER trace freeze.
    rej=pd.read_csv(OUT/'p5_rejection_analysis.csv');rej['known_receipt_available']=[(A/'attempts'/f'e{int(r.event_index):04d}_r{int(r.candidate_rank)}.json').exists() for r in rej.itertuples()]
    csv(OUT/'p5_rejection_receipt_availability.csv',rej)
    denied=rej[rej.reason!='STOP_AT_GEOMETRIC_ACCEPTANCE'];first=denied.sort_values(['event_index','candidate_rank']).groupby('event_index').head(1);busy=first[first.blocked_during_busy_window]
    timeline=pd.read_csv(OUT/'p5_rolling_budget_timeline.csv');local=pd.read_csv(BASE/'12d_clean_backend/clean_local_trajectory.csv');bins={}
    for robot,frame in local.sort_values('keyframe_id').groupby('robot_id'):
        x=frame[['tx','ty','tz']].to_numpy();bins.update({(robot,int(k)):int(a//50) for k,a in zip(frame.keyframe_id,np.r_[0,np.linalg.norm(np.diff(x,axis=0),axis=1).cumsum()])})
    loops=pd.read_csv(B/'policies'/P5/'online_verified_loops.csv');p5bins={(bins['robot1',int(r.canonical_robot1_kf)],bins['robot3',int(r.canonical_robot3_kf)]) for r in loops.itertuples()}
    post=dict(meaning='descriptive after trace freeze, not scheduler inputs',nonexecuted_candidate_rows=len(rej),early_stop_rows_not_budget_rejections=int((rej.reason=='STOP_AT_GEOMETRIC_ACCEPTANCE').sum()),actually_denied_query_count=len(first),first_denial_reasons=first.reason.value_counts().astype(int).to_dict(),first_denials_during_busy_windows=len(busy),busy_first_denials_with_later_100event_capacity=int(busy.first_later_frozen_capacity_event.notna().sum()),known_nonexecuted_rows=int(rej.known_receipt_available.sum()),unknown_nonexecuted_rows=int((~rej.known_receipt_available).sum()),known_denied_rows=int(denied.known_receipt_available.sum()),unknown_denied_rows=int((~denied.known_receipt_available).sum()),events_with_two_uncached_cloud_headroom=int(((timeline.rolling_byte_capacity_remaining>=2097362+249)&(timeline.rolling_call_capacity_remaining>=1)).sum()),P5_first_bridge_event=int(loops.iloc[0].discovery_event),P5_covered_odometry_bin_pairs=len(p5bins))
    dump(OUT/'p5_postfreeze_rejection_summary.json',post)
    browser=read(MAP/'browser_validation.json') if (MAP/'browser_validation.json').exists() else dict(status='NOT_RUN',reason='no browser evidence imported')
    decision=dict(P0_P5_FINAL_DEMO='PASS' if browser['status']=='PASS' else 'PARTIAL',P5_UTILITY_AUDIT='COMPLETE',P6_SCHEDULER_PROTOTYPE='PASS' if validation['status']=='PASS' else 'BLOCKED',P6_INDEPENDENT_ACCURACY='NOT_RUN',STAGE23_FINAL_INDEPENDENT_VALIDATION='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE',progressive='PROGRESSIVE_STATE_EXPORT_REQUIRED',promotion='NOT_AUTHORIZED; greater resource utilization or admitted count is not measured map improvement',new_GICP_executions=0,new_backend_solves=0,selected_unknown_results=len(pd.read_csv(OUT/'unknown_gicp_extension_plan.csv')),mapping_comparison='P0 versus P5 frozen actual maps only; P6 mapping accuracy NOT_RUN',next_action='independently reconstruct P6 graphs only under separate authorization; preserve all frozen scientific settings and use suitable independent bidirectional recordings for promotion')
    dump(OUT/'stage23d_decision.json',decision);print(json.dumps(decision),flush=True)
if __name__=='__main__':main()
