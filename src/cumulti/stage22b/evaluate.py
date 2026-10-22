"""Gate F and post-graph offline GT evaluation. Never feeds a policy or optimizer."""
import time
from common import *


def evaluate_queries():
    lock=read(OUT/'pre_gt_decision_lock.json');verify_lock()
    write_json(OUT/'offline_evaluation_entry.json',dict(decisions_frozen_before_GT=True,unix_ns=time.time_ns(),
        pre_gt_decision_sha256=sha(OUT/'pre_gt_decision_lock.json'),finalist_sha256=sha(OUT/'finalist_lock.json')))
    reference=pd.read_csv(A/'offline_causal_query_evaluation.csv').set_index('event_index')
    positive=pd.read_csv(A/'offline_geometrically_accepted_including_duplicates_labels.csv').set_index('discovery_event')
    allrows=[];scope_coverage=[];failures=[]
    for name,decision in lock['decisions'].items():
        directory=OUT/'policies'/name
        assert sha(directory/'candidate_decision_trace.csv')==decision['decisions_sha256']
        assert sha(directory/'online_verified_loops.csv')==decision['loops_sha256']
        discoveries=pd.read_csv(directory/'online_loop_discovery_trace.csv')
        loops=pd.read_csv(directory/'online_verified_loops.csv')
        accepted=set(discoveries.discovery_event.astype(int));admitted=set(loops.discovery_event.astype(int))
        lost=set(reference[reference.geometric_true_positive].index)-accepted
        trace=pd.read_csv(directory/'candidate_decision_trace.csv')
        denied=trace[(~trace.execute)&(trace.reason!='STOP_AT_GEOMETRIC_ACCEPTANCE')].drop_duplicates('event_index').set_index('event_index')
        for event in sorted(lost):
            failures.append(dict(policy=name,event_index=int(event),query_robot=reference.loc[event,'query_robot'],
                scope='dev' if event<1000 else 'embargo' if event<1100 else 'heldout',
                lost_baseline_geometric_true_positive=True,
                first_denial_reason=denied.loc[event,'reason'] if event in denied.index else 'NOT_CLASSIFIED',
                attribution='observed first stopping rule, not an isolated causal component ablation',evidence_type='OFFLINE_GT_EVALUATION'))
        for scope,indices in [('dev',range(1000)),('heldout',range(1100,5796)),('full',range(5796))]:
            frame=reference.loc[list(indices)]
            for direction in ['ALL','robot1','robot3']:
                subset=frame if direction=='ALL' else frame[frame.query_robot==direction]
                valid=set(subset[subset.valid_overlap].index)
                geo=accepted&set(subset.index);adm=admitted&set(subset.index)
                tp=sum(bool(positive.loc[i,'positive_d_xy_lt5']) for i in geo)
                atp=sum(bool(positive.loc[i,'positive_d_xy_lt5']) for i in adm)
                res=dict(policy=name,scope=scope,direction=direction,queries=len(subset),positive_queries=len(valid),
                    geometric_accepted=len(geo),geometric_true=tp,false_geometric=len(geo)-tp,
                    precision=tp/len(geo) if geo else None,recall=tp/len(valid) if valid else None,
                    admitted=len(adm),admitted_true=atp,false_admitted=len(adm)-atp,
                    admitted_precision=atp/len(adm) if adm else None,admission_recall=atp/len(valid) if valid else None,
                    evidence_type='OFFLINE_GT_EVALUATION')
                allrows.append(res)
    result=pd.DataFrame(allrows)
    csv(OUT/'policy_comparison_dev.csv',result[result.scope=='dev'])
    csv(OUT/'policy_comparison_holdout.csv',result[result.scope=='heldout'])
    csv(OUT/'policy_comparison_full.csv',result[result.scope=='full'])
    csv(OUT/'offline_failure_cases.csv',failures)
    csv(OUT/'offline_failure_causes.csv',pd.DataFrame(failures).groupby(['policy','scope','first_denial_reason']).size().reset_index(name='lost_baseline_true_queries'))
    events=pd.read_csv(A/'dual_stream_event_schedule.csv');end_R1=int(events[events.robot_id=='robot1'].event_index.max())
    for scope,indices in [('dev',range(1000)),('heldout',range(1100,5796))]:
        for period,ids in [('both_progressing',[i for i in indices if i<=end_R1]),('after_R1_finishes',[i for i in indices if i>end_R1])]:
            f=reference.loc[ids]
            scope_coverage.append(dict(scope=scope,period=period,queries=len(f),robot1_queries=int((f.query_robot=='robot1').sum()),
                robot3_queries=int((f.query_robot=='robot3').sum()),positive_overlap_queries=int(f.valid_overlap.sum()),
                baseline_geometric_positive_queries=int(f.geometric_true_positive.sum())))
    csv(OUT/'offline_split_coverage.csv',scope_coverage)
    coverage_ok=all(any(x['scope']=='heldout' and x['period']==period and x['positive_overlap_queries']>0 for x in scope_coverage)
                    for period in ['both_progressing','after_R1_finishes'])
    write_json(OUT/'gate_F.json',dict(status='PASS' if coverage_ok else 'COMPLETED_WITH_COVERAGE_LIMITATION',
        representative_positive_coverage=coverage_ok,heldout_generalization_gate='PASS' if coverage_ok else 'FAIL_POSITIVE_DIRECTION_AND_PHASE_COVERAGE',
        independent_policy_selection=True,retuning=False,
        caveat='single chronological holdout on same route; very early genuine opportunities reside in development, not heldout; not independent geography',
        evidence_type='OFFLINE_GT_EVALUATION',frozen_decision_sha256=sha(OUT/'pre_gt_decision_lock.json')))


def graph_accuracy():
    import run_stage10_offline_map_merge as s10
    from scipy.spatial.transform import Rotation
    gt={r:pd.read_csv(PROC/r/'keyframes.csv').set_index('keyframe_id',drop=False) for r in ['robot1','robot3']}
    rows=[];integrity=[];agreement=[];consistency=[]
    for name in read(OUT/'pre_gt_decision_lock.json')['decisions']:
        directory=OUT/'policies'/name
        if not (directory/'online_graph_integrity.json').exists():continue
        rec=read(directory/'online_graph_integrity.json')
        bridge=read(directory/'first_online_bridge.json') if (directory/'first_online_bridge.json').exists() else None
        integrity.append(dict(policy=name,**rec,evidence_type='ACTUAL_EXECUTED',
            first_bridge_discovery_event=None if bridge is None else bridge['discovery_event'],
            first_bridge_logical_seconds=None if bridge is None else bridge['normalized_time_ns']/1e9,
            bridge_status='NO_FIRST_BRIDGE_FOUND' if bridge is None else 'FIRST_BRIDGE_FOUND',
            final_components=1 if rec['status']=='PASS' and rec['loops']>0 else None,
            time_scope='hypothetical normalized event attribution, not measured service completion'))
        agreement.extend(pd.read_csv(directory/'online_incremental_vs_batch.csv').assign(policy=name,evidence_type='ACTUAL_EXECUTED').to_dict('records'))
        consistency.extend(pd.read_csv(directory/'online_map_consistency.csv').assign(policy=name,evidence_type='ACTUAL_EXECUTED').to_dict('records'))
        if rec['status']!='PASS':continue
        for condition in ['incremental','batch']:
            tr=pd.read_csv(directory/('online_'+condition+'_trajectory.csv'))
            ate,_,_=s10.joint_eval(tr,gt['robot1'],gt['robot3'])
            rows.extend(dict(policy=name,condition=condition,metric='ATE_joint_rigid',evidence_type='OFFLINE_GT_EVALUATION',**row) for row in ate)
            lookup=s10.lookup(tr);allt=[];allr=[]
            for robot in ['robot1','robot3']:
                ids=tr[tr.robot_id==robot].keyframe_id.astype(int).tolist();ts=[];rs=[]
                for k in ids[:-10]:
                    a,z=gt[robot].loc[k],gt[robot].loc[k+10]
                    G0=s10.T([a.qx,a.qy,a.qz,a.qw],[a.x,a.y,a.z]);G1=s10.T([z.qx,z.qy,z.qz,z.qw],[z.x,z.y,z.z])
                    E=inv(inv(G0)@G1)@inv(lookup[robot,k])@lookup[robot,k+10]
                    ts.append(float(np.linalg.norm(E[:3,3])));rs.append(float(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude())))
                allt.extend(ts);allr.extend(rs)
                rows.append(dict(policy=name,condition=condition,metric='RPE_interval10',scope=robot,samples=len(ts),
                    translation_RMSE_m=float(np.sqrt(np.mean(np.square(ts)))),rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(rs)))),evidence_type='OFFLINE_GT_EVALUATION'))
            rows.append(dict(policy=name,condition=condition,metric='RPE_interval10',scope='combined',samples=len(allt),
                translation_RMSE_m=float(np.sqrt(np.mean(np.square(allt)))),rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(allr)))),evidence_type='OFFLINE_GT_EVALUATION'))
    csv(OUT/'graph_integrity_by_policy.csv',integrity)
    csv(OUT/'incremental_vs_batch_by_policy.csv',agreement)
    csv(OUT/'gt_free_map_consistency_by_policy.csv',consistency)
    csv(OUT/'offline_map_accuracy_by_policy.csv',rows)

if __name__=='__main__':
    evaluate_queries()
    graph_accuracy()
