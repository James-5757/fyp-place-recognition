#!/usr/bin/env python3
"""Independent Stage13B provenance/scheduling/aggregation audit; no solves/GT reads.

Distinguish successful experiment execution from numerical agreement: a truthful
NOT_READY result is auditable even when default ISAM2 differs from batch.
"""
import json
from pathlib import Path
import gtsam
import numpy as np
import pandas as pd
import run_stage13b_incremental_isam2 as b

def main():
    out=b.OUT
    j=lambda n:json.loads((out/n).read_text())
    checks={}
    trace=pd.read_csv(out/'incremental_event_trace.csv')
    stream=trace[trace.event_type.ne('FINAL_STATE')]
    loops=pd.read_csv(b.D12/'ordered_rank1_clean_loops.csv')
    inputs=j('frozen_input_manifest.json')['inputs']
    checks['historical_hashes_unchanged']=all(b.d.sha256(x['path'])==x['sha256'] for x in inputs)
    checks['1850_original_events_plus_final']=len(trace)==1851 and stream.robot1_keyframe.to_list()==list(range(150,2000))
    original=pd.read_csv(b.d.s10.PROC/'robot1/keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns'])
    checks['original_timestamps_preserved']=stream.robot1_timestamp_ns.to_list()==original.iloc[150:].lidar_timestamp_ns.to_list()
    checks['loop_query_timestamps_match_stream']=all(int(row.query_timestamp)==int(original.iloc[int(row.query_keyframe_id)].lidar_timestamp_ns) for row in loops.itertuples())
    pre=stream[stream.robot1_keyframe.lt(283)];post=stream[stream.robot1_keyframe.ge(283)]
    checks['no_joint_solver_before_bridge']=pre.total_graph_nodes.eq(0).all() and pre.isam2_update_ms.eq(0).all() and pre.connected_components.eq(2).all()
    checks['only_arrived_robot1_nodes']=post.total_graph_nodes.to_list()==(post.robot1_keyframe-150+1+3946).to_list()
    checks['first_graph_counts']=j('first_fusion_manifest.json')['total_nodes']==4080 and j('first_fusion_manifest.json')['odometry_factors']==4078
    checks['single_persistent_ISAM2']=post.instance_id.nunique()==1 and len(post)==1717
    checks['odom_inserted_exactly_once']=int(post.new_odometry_factors.sum())==5794 and post.iloc[1:].new_odometry_factors.eq(1).all()
    checks['loops_inserted_exactly_once']=int(post.new_loop_factors.sum())==75 and post[post.new_loop_factors.gt(0)].robot1_keyframe.to_list()==loops.query_keyframe_id.to_list()
    checks['final_counts']=int(post.iloc[-1].total_graph_nodes)==5796 and int(post.iloc[-1].cumulative_global_factors)==5870 and int(post.iloc[-1].cumulative_loop_factors)==75
    checks['final_structural_equivalence']=j('final_graph_equivalence.json')['PASS']
    checkpoints=pd.read_csv(out/'checkpoint_manifest.csv')
    expected=['PRE_FIRST_LOOP','FIRST_LOOP','K2','K5','K10','K20','K40','K75','FINAL_STREAM_END']
    checks['frozen_checkpoint_schedule']=checkpoints.checkpoint.to_list()==expected
    for row in checkpoints.itertuples():
        tr=pd.read_csv(row.trajectory_path)
        r1=tr[tr.robot_id.eq('robot1')];r3=tr[tr.robot_id.eq('robot3')]
        checks[row.checkpoint+'_available_nodes']=r1.keyframe_id.to_list()==list(range(150,int(row.robot1_keyframe)+1)) and r3.keyframe_id.to_list()==list(range(234,4180))
        checks[row.checkpoint+'_schema']=np.isfinite(tr[b.d.POSE_COLUMNS[3:]].to_numpy()).all() and np.allclose(np.linalg.norm(tr[['qx','qy','qz','qw']],axis=1),1,atol=1e-8) and not tr.duplicated(['robot_id','keyframe_id']).any()
        checks[row.checkpoint+'_trajectory_hash']=b.d.sha256(row.trajectory_path)==row.trajectory_sha256
        manifest=json.loads(Path(row.map_manifest).read_text())
        checks[row.checkpoint+'_map_frames']=manifest['source_keyframes']['robot1']==list(range(150,int(row.robot1_keyframe)+1,10)) and manifest['source_keyframes']['robot3']==list(range(234,4180,10))
        checks[row.checkpoint+'_map_artifacts']=all(b.d.sha256(x['path'])==x['sha256'] and Path(x['path']).stat().st_size==x['size_bytes'] for x in manifest['artifacts'])
        if row.checkpoint=='PRE_FIRST_LOOP': checks['no_fused_prebridge_map']=not manifest['common_frame_metric_available'] and all(x['robot_source']!='merged' for x in manifest['artifacts'])
    comp=pd.read_csv(out/'incremental_vs_batch_checkpoints.csv');limits=j('incremental_batch_agreement_policy.json')
    forensic=[]
    local=b.d.s10.lookup(pd.read_csv(b.D12/'clean_local_trajectory.csv'))
    for row in comp.itertuples():
        tr=pd.read_csv(out/f'snapshots/{row.checkpoint}_trajectory.csv')
        pose=b.d.s10.lookup(tr)[('robot1',150)]
        E=b.d.s10.inv(local[('robot1',150)])@pose
        ev=stream[stream.robot1_keyframe.eq(row.robot1_keyframe)].iloc[0]
        forensic.append(dict(checkpoint=row.checkpoint,robot1_keyframe=row.robot1_keyframe,anchor_translation_m=float(np.linalg.norm(E[:3,3])),anchor_rotation_deg=float(np.rad2deg(b.Rotation.from_matrix(E[:3,:3]).magnitude())),variables_relinearized_at_checkpoint=int(ev.nodes_relinearized),variables_reeliminated_at_checkpoint=int(ev.variables_reeliminated),agreement=row.agreement_status,objective_gap=row.graph_error_difference))
    b.csv('incremental_anchor_relinearization_audit.csv',forensic)
    checks['all_checkpoint_anchors_fixed']=all(x['anchor_translation_m']<1e-7 and x['anchor_rotation_deg']<1e-6 for x in forensic)
    b.dump('numerical_discrepancy_forensics.json',dict(structural_factor_identity_verified=j('final_graph_equivalence.json')['PASS'],anchor_gauge_verified=checks['all_checkpoint_anchors_fixed'],batch_parameters_frozen=True,ISAM2_parameters_changed=False,first_fusion_initialization_near_zero=j('first_loop_incremental_fusion_result.json')['initial_alignment'],checkpoint_evidence=forensic,findings=['K2/K10/K40/K75/final show zero relinearized variables in the checkpoint arrival update; prior updates may already have relinearized other variables','FIRST_LOOP/K5/K20 pass while other immediate checkpoint estimates fail; agreement is not monotonic','Final objectives almost agree while same-gauge poses exceed translation limits; objective equality alone is insufficient'],remaining_uncertainty='Factor insertion order, different causal guess/optimization paths, robust linearization caching, wildfire stopping and weakly constrained directions are plausible, not independently established causes. No new optimization or parameter trials authorized/performed.'))
    passed=[]
    for row in comp.itertuples():
        fits=all(getattr(row,x)<=limits[x] for x in ['translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg']);passed.append(fits)
        checks[row.checkpoint+'_agreement_truthful']=row.agreement_status==('INCREMENTAL_BATCH_AGREEMENT_PASS' if fits else 'ISAM2_BATCH_DIVERGENCE')
        checks[row.checkpoint+'_identical_batch_counts']=row.nodes==row.robot1_keyframe-150+1+3946 and row.odometry_factors==row.nodes-2 and row.total_factors==row.odometry_factors+row.K+1
    checks['no_catastrophic_checkpoint_steps']=pd.read_csv(out/'incremental_trajectory_integrity.csv').new_catastrophic_steps.eq(0).all()
    final=pd.read_csv(out/'incremental_final_trajectory.csv');checks['final_full_schema']=b.d.schema_audit(final)['PASS']
    events=j('execution_events.json');phase={x['phase']:x for x in events};policy=j('incremental_policy.json')
    checks['policy_before_ISAM2_and_GT']=policy['UTC']<phase['POLICY_FROZEN']['UTC']<min(x['UTC'] for x in events if x['phase']=='CHECKPOINT')<phase['PRE_GT_DECISION']['UTC']<phase['FIRST_GT_ACCESS']['UTC']
    decision=j('pre_gt_incremental_decision.json')
    checks['readiness_truthful_no_GT']=decision['decision']==('INCREMENTAL_BACKEND_READY' if all(passed) else 'INCREMENTAL_BACKEND_NOT_READY') and decision['GT_used'] is False
    t= (out/'pre_gt_incremental_decision.json').stat().st_mtime_ns
    checks['comparisons_saved_before_GT_decision']=all((out/n).stat().st_mtime_ns<t for n in ['incremental_vs_batch_checkpoints.csv','incremental_map_consistency.csv','incremental_vs_batch_map_consistency.csv','incremental_latency_summary.csv','incremental_final_trajectory.csv','final_graph_equivalence.json'])
    checks['no_updates_or_LM_after_GT']=not any(x['phase'] in ['CHECKPOINT','STREAM_PROGRESS'] and x['UTC']>phase['FIRST_GT_ACCESS']['UTC'] for x in events)
    param=j('isam2_parameter_policy.json');checks['installed_defaults_unmodified']=param['parameter_text']==str(gtsam.ISAM2Params()) and param['extra_empty_updates']==0
    gt=pd.read_csv(out/'incremental_offline_gt_evaluation.csv')
    checks['offline_GT_after_decision']=(out/'incremental_offline_gt_evaluation.csv').stat().st_mtime_ns>t and len(gt)==6 and gt['use'].eq('OFFLINE GT DIAGNOSTIC ONLY').all()
    r=gt[gt.metric_type.eq('RPE')].set_index('scope')
    for col in ['translation_RMSE_m','rotation_RMSE_deg']:
        rmse=np.sqrt(sum(r.loc[x,col]**2*r.loc[x,'sample_count'] for x in ['robot1','robot3'])/sum(r.loc[x,'sample_count'] for x in ['robot1','robot3']))
        checks['combined_RPE_'+col]=np.isclose(rmse,r.loc['combined',col],rtol=1e-12)
    checks['9_figures']=len(list(out.glob('0*.png')))==9
    failed=out/'_failed_attempts/api_keyset'
    if failed.exists():
        old=json.loads((failed/'isam2_parameter_policy.json').read_text())
        no_GT=not (failed/'pre_gt_incremental_decision.json').exists() and 'FIRST_GT_ACCESS' not in (failed/'run.log').read_text()
        checks['API_failure_rerun_not_retuning']=old['parameters']==param['parameters'] and no_GT
        b.dump('development_preflight_note.json',dict(reason='Installed KeySet is not iterable; graph.keyVector() used after actual API inspection',archive_path=str(failed),parameters_identical=old['parameters']==param['parameters'],GT_not_accessed=no_GT,failure_log_sha256=b.d.sha256(failed/'run.log'),historical_outputs_removed=False))
    checks={k:bool(v) for k,v in checks.items()}
    b.dump('independent_final_audit.json',dict(execution_and_provenance_PASS=all(checks.values()),numerical_agreement_PASS=all(passed),checks=checks,optimization_rerun=False,GT_evaluation_rerun=False))
    print(json.dumps(dict(audit_PASS=all(checks.values()),numerical_agreement_PASS=all(passed),failed_checks=[k for k,v in checks.items() if not v]),indent=2));assert all(checks.values()),checks

if __name__=='__main__':main()
