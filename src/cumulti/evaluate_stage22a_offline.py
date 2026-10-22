"""Read GT/archive ONLY after persisted new frontend/backend decisions.

No retrieval/registration/optimization entrypoint is called by this evaluator.
Recall is query based, not archival75 overlap. Strict spatial positives d_xy<5m.
"""
import time
from scipy.spatial.transform import Rotation
from stage22a_common import *

def main():
    assert (OUT/'stage22a2_decision.json').exists() and (OUT/'stage22a3_decision.json').exists()
    complete=read('stage22a2_decision.json')['status']=='ONLINE_GICP_VERIFICATION_READY'
    frozen={n:sha(OUT/n) for n in ['stage22a1_decision.json','stage22a2_decision.json','stage22a3_decision.json','online_verified_loops.csv','online_loop_discovery_trace.csv']}
    dump('offline_evaluation_entry.json',dict(status='PRECONDITION_PASSED',decision_hashes=frozen,GT_read_after_decisions=True,entry_unix_ns=time.time_ns()))
    f=frames();events=events_for(f);pos={(e['robot_id'],e['keyframe_id']):e['event_index'] for e in events}
    gt={r:pd.read_csv(PROC/r/'keyframes.csv').set_index('keyframe_id',drop=False) for r in STARTS}
    trace=pd.read_csv(OUT/'online_candidate_trace.csv');ranked={int(i):x.sort_values('candidate_rank').to_dict('records') for i,x in trace.groupby('event_index')}
    discoveries=pd.read_csv(OUT/'online_loop_discovery_trace.csv');reps=pd.read_csv(OUT/'online_verified_loops.csv');found={int(m['discovery_event']):m for m in reps.to_dict('records')};geometric={int(m['discovery_event']):m for m in discoveries.to_dict('records')}
    arrays={r:gt[r].loc[f[r].keyframe_id,['x','y']].to_numpy(float) for r in STARTS};ids={r:list(f[r].keyframe_id.map(int)) for r in STARTS};seen={r:0 for r in STARTS};evalrows=[]
    def distance(qr,q,cr,c):return float(np.linalg.norm(gt[qr].loc[q,['x','y']].to_numpy(float)-gt[cr].loc[c,['x','y']].to_numpy(float)))
    for e in events:
        r=e['robot_id'];other='robot3' if r=='robot1' else 'robot1';k=e['keyframe_id'];n=seen[other]
        xy=gt[r].loc[k,['x','y']].to_numpy(float);dist=np.linalg.norm(arrays[other][:n]-xy,axis=1);positive_ids={ids[other][j] for j in np.flatnonzero(dist<5)}
        candidates=ranked.get(e['event_index'],[]);hits=[x['candidate_keyframe_id'] in positive_ids for x in candidates]
        accepted=found.get(e['event_index']);true_edge=accepted is not None and distance('robot1',accepted['canonical_robot1_kf'],'robot3',accepted['canonical_robot3_kf'])<5
        geometric_edge=geometric.get(e['event_index']);geometric_true=geometric_edge is not None and distance('robot1',geometric_edge['canonical_robot1_kf'],'robot3',geometric_edge['canonical_robot3_kf'])<5
        evalrows.append(dict(event_index=e['event_index'],query_robot=r,query_keyframe_id=k,available_database=n,positive_count=len(positive_ids),valid_overlap=bool(positive_ids),top1_positive=bool(hits and hits[0]),top3_positive=any(hits),geometrically_accepted=geometric_edge is not None,geometric_true_positive=bool(geometric_true),admitted_loop=accepted is not None,admitted_true_positive=bool(true_edge),missed_query=bool(positive_ids) and not geometric_true))
        seen[r]+=1
    csv('offline_causal_query_evaluation.csv',evalrows)
    metrics=[]
    for direction in ['ALL','robot1','robot3']:
        rows=[x for x in evalrows if direction=='ALL' or x['query_robot']==direction];valid=[x for x in rows if x['valid_overlap']];admit=[x for x in rows if x['admitted_loop']]
        geometric_rows=[x for x in rows if x['geometrically_accepted']]
        metrics.append(dict(direction=direction,queries=len(rows),valid_overlap_queries=len(valid),candidate_R1=sum(x['top1_positive'] for x in valid)/len(valid) if valid else None,candidate_R3=sum(x['top3_positive'] for x in valid)/len(valid) if valid else None,geometrically_accepted_queries=len(geometric_rows),false_geometric_acceptances=sum(not x['geometric_true_positive'] for x in geometric_rows),admitted_loops=len(admit),true_admitted_loops=sum(x['admitted_true_positive'] for x in admit),loop_precision=sum(x['geometric_true_positive'] for x in geometric_rows)/len(geometric_rows) if geometric_rows else None,query_based_loop_recall=sum(x['geometric_true_positive'] for x in valid)/len(valid) if valid else None,admitted_loop_precision=sum(x['admitted_true_positive'] for x in admit)/len(admit) if admit else None,query_based_admission_recall=sum(x['admitted_true_positive'] for x in valid)/len(valid) if valid else None,missed_true_query_events=sum(x['missed_query'] for x in rows),note='primary loop precision/recall use geometric acceptance BEFORE duplicate suppression; separate admission metrics AFTER suppression; recall denominator is causal queries with >=1 already-arrived GT positive, not all-pair recall'))
    csv('offline_frontend_accuracy.csv',metrics)
    if not complete:
        for m in metrics:
            m['loop_precision']=None;m['query_based_loop_recall']=None;m['missed_true_query_events']=None
            m['note']='Full SC candidate recall valid; loop recall/missed-count NOT_RUN because complete chronological GICP verification was not completed'
        csv('offline_frontend_accuracy.csv',metrics)
    for name,data in [('admitted',reps),('geometrically_accepted_including_duplicates',discoveries)]:
        labeled=[]
        for row in data.to_dict('records'):
            q,c=int(row['canonical_robot1_kf']),int(row['canonical_robot3_kf']);d=distance('robot1',q,'robot3',c)
            qe,ce=pos[('robot1',q)],pos[('robot3',c)];earliest=min(qe,ce);latest=max(qe,ce);disc=int(row['discovery_event'])
            labeled.append(dict(loop_id=row.get('loop_id'),discovery_event=disc,robot1_kf=q,robot3_kf=c,d_xy_m=d,positive_d_xy_lt5=d<5,first_query_normalized_ns=events[earliest]['normalized_ns'],candidate_availability_normalized_ns=events[latest]['normalized_ns'],discovery_normalized_ns=events[disc]['normalized_ns'],delay_from_first_observation_s=(events[disc]['normalized_ns']-events[earliest]['normalized_ns'])/1e9,delay_after_both_available_s=(events[disc]['normalized_ns']-events[latest]['normalized_ns'])/1e9))
        csv('offline_'+name+'_labels.csv',labeled)
    # First historical75 decode occurs here, never during candidate/graph construction.
    archive=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv')
    new_pairs={(int(r.canonical_robot1_kf),int(r.canonical_robot3_kf)) for r in reps.itertuples()};ever={(int(r.query_keyframe_id),int(r.candidate_keyframe_id)) if r.query_robot=='robot1' else (int(r.candidate_keyframe_id),int(r.query_keyframe_id)) for r in trace.itertuples()}
    attempts=pd.read_csv(OUT/'online_gicp_attempts.csv');attempt_pairs={(int(r.canonical_robot1_kf),int(r.canonical_robot3_kf)) for r in attempts.itertuples()}
    attempt_events={(int(r.event_index),int(r.candidate_rank)) for r in attempts.itertuples()};availability=[]
    for row in trace.itertuples():
        a=pos[(row.query_robot,int(row.query_keyframe_id))];b=pos[(row.candidate_robot,int(row.candidate_keyframe_id))]
        availability.append(dict(event_index=int(row.event_index),candidate_rank=int(row.candidate_rank),query_robot=row.query_robot,query_keyframe_id=int(row.query_keyframe_id),candidate_robot=row.candidate_robot,candidate_keyframe_id=int(row.candidate_keyframe_id),first_observation_time_ns=events[min(a,b)]['normalized_ns'],candidate_available_time_ns=events[b]['normalized_ns'],both_endpoints_available_time_ns=events[max(a,b)]['normalized_ns'],shortlisted_time_ns=events[a]['normalized_ns'],delay_from_first_observation_s=(events[a]['normalized_ns']-events[min(a,b)]['normalized_ns'])/1e9,actually_GICP_verified=(int(row.event_index),int(row.candidate_rank)) in attempt_events,ever_shortlisted=True))
    csv('online_pair_availability_and_discovery.csv',availability)
    comparisons=[]
    for r in archive.itertuples():
        pair=(int(r.query_keyframe_id),int(r.candidate_keyframe_id));a=pos[('robot1',pair[0])];b=pos[('robot3',pair[1])]
        comparisons.append(dict(frozen_loop_id=int(r.frozen_loop_id),robot1_kf=pair[0],robot3_kf=pair[1],ever_shortlisted=pair in ever,actually_GICP_verified=pair in attempt_pairs,exact_pair_admitted=pair in new_pairs,first_query_ns=events[min(a,b)]['normalized_ns'],candidate_available_ns=events[max(a,b)]['normalized_ns'],archived_overlap_is_GT_recall=False))
    csv('online_vs_archived_loop_comparison.csv',comparisons)
    dump('offline_archive_comparison_summary.json',dict(archive_count=75,exact_overlap=sum(r['exact_pair_admitted'] for r in comparisons),new_exact_pairs_not_in_archive=len(new_pairs-{(int(r.query_keyframe_id),int(r.candidate_keyframe_id)) for r in archive.itertuples()}),archive_ever_shortlisted=sum(r['ever_shortlisted'] for r in comparisons),archive_actually_verified=sum(r['actually_GICP_verified'] for r in comparisons),archive_overlap_is_ground_truth_recall=False))
    # Valid finite trajectories permit supported offline evaluation even if batch
    # agreement fails. One joint rigid alignment, no per-robot alignment/tuning.
    integrity=read('online_graph_integrity.json');pose_metrics=[]
    if integrity['status']=='PASS':
        import run_stage10_offline_map_merge as s10
        for condition in ['online_incremental','online_batch']:
            tr=pd.read_csv(OUT/(condition+'_trajectory.csv'));ate,_,_=s10.joint_eval(tr,gt['robot1'],gt['robot3']);lookup=s10.lookup(tr)
            pose_metrics.extend(dict(condition=condition,metric='ATE_joint_rigid',use='OFFLINE_ONLY',**row) for row in ate)
            allt=[];allr=[]
            for robot in STARTS:
                ts=[];rs=[]
                for k in ids[robot][:-10]:
                    a,z=gt[robot].loc[k],gt[robot].loc[k+10];G0=s10.T([a.qx,a.qy,a.qz,a.qw],[a.x,a.y,a.z]);G1=s10.T([z.qx,z.qy,z.qz,z.qw],[z.x,z.y,z.z]);E=inv(inv(G0)@G1)@inv(lookup[(robot,k)])@lookup[(robot,k+10)]
                    ts.append(float(np.linalg.norm(E[:3,3])));rs.append(float(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude())))
                allt.extend(ts);allr.extend(rs);pose_metrics.append(dict(condition=condition,metric='RPE_interval10',scope=robot,translation_RMSE_m=float(np.sqrt(np.mean(np.square(ts)))),rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(rs)))),samples=len(ts),use='OFFLINE_ONLY'))
            pose_metrics.append(dict(condition=condition,metric='RPE_interval10',scope='combined',translation_RMSE_m=float(np.sqrt(np.mean(np.square(allt)))),rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(allr)))),samples=len(allt),use='OFFLINE_ONLY'))
    else:pose_metrics=[dict(status='NOT_RUN',reason='invalid/incomplete new graph; no fabricated GT trajectory scores')]
    csv('offline_graph_accuracy.csv',pose_metrics)
    assert all(sha(OUT/n)==h for n,h in frozen.items())
    dump('offline_evaluation_audit.json',dict(status='PASS',new_results_and_decisions_unchanged=True,GT_after_freeze=True,frontend_rerun_after_GT=False))
if __name__=='__main__':main()
