#!/usr/bin/env python3
"""Independent saved-artifact/provenance audit; never optimizes or accesses GT."""
import json
from decimal import Decimal
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import run_stage14_final_consistency as s

def main():
    out=s.OUT;j=lambda name:json.loads((out/name).read_text());checks={}
    manifest=j('stage14_input_manifest.json');policy=j('stage14_experiment_policy.json')
    checks['inputs_unchanged']=all(s.d.sha256(x['path'])==x['sha256'] for x in manifest['inputs']+manifest['inherited_inputs'])
    checks['history_immutable']=all(s.d.sha256(p)==h for p,h in manifest['historical_hashes_before'].items())
    checks['gates_unchanged']=policy['gates']==s.LIMITS and policy['checkpoints']==s.CP
    checks['no_parameter_search']=policy['A']['skip']==1 and policy['A']['threshold']==.01 and policy['A']['wildfire']==.001 and policy['A']['extra_calls']==5
    checks['fixed_support_and_conditioning']=policy['support_windows_kf']==[50,100,250] and len(policy['conditioning']['poses'])==9 and policy['conditioning']['additional_address_space_bytes']==1024**3
    provenance=j('loop_provenance_audit.json')
    old=pd.read_csv(s.d.LOOP_SOURCE,usecols=s.d.LOOP_COLUMNS);old.insert(0,'frozen_loop_id',np.arange(len(old)))
    kept=old[old.query_keyframe_id.ge(150)&old.candidate_keyframe_id.ge(234)].sort_values(['query_timestamp','query_keyframe_id','candidate_keyframe_id'],kind='stable').reset_index(drop=True)
    clean=pd.read_csv(s.b.D12/'ordered_rank1_clean_loops.csv',usecols=['arrival_index','frozen_loop_id']+s.d.LOOP_COLUMNS)
    pd.testing.assert_frame_equal(kept[['frozen_loop_id']+s.d.LOOP_COLUMNS],clean[['frozen_loop_id']+s.d.LOOP_COLUMNS],check_exact=False,atol=1e-12,rtol=1e-12)
    checks['loop120_75_45']=len(old)==120 and len(clean)==75 and len(provenance['excluded_frozen_loop_ids'])==45 and provenance['selected_frozen_loop_ids']==clean.frozen_loop_id.astype(int).to_list()
    original=s.allowed_frames();trace=pd.read_csv(out/'fresh_rebuild_microtrace.csv');numbers=pd.read_csv(out/'fresh_rebuild_summary.csv');lm=pd.read_csv(out/'lm_basin_probe.csv')
    checks['nine_fresh54states']=len(trace)==54 and all(z.step.to_list()==list(range(6)) for _,z in trace.groupby(['checkpoint','solver_state'],sort=False))
    checks['six_LM_probes']=len(lm)==6 and set(lm.solver_state)=={'B0_LM_CANONICAL','B1_LM_D0','B2_LM_D2'}
    counter_valid=True;newpose_files=[]
    for cp,kf in s.CP.items():
        ref=pd.read_csv(s.B/f'snapshots/{cp}_batch_trajectory.csv',usecols=s.d.POSE_COLUMNS);ref=ref.set_index(['robot_id','keyframe_id'])
        for row in pd.concat([trace,numbers[~numbers.solver_state.str.startswith('A')],lm],ignore_index=True).query('checkpoint == @cp').itertuples():
            path=out/f'estimates/{cp}_{row.solver_state}_step{int(row.step)}.csv';tr=pd.read_csv(path);newpose_files.append(path)
            frame={'robot1':original['robot1'][original['robot1'].keyframe_id.le(kf)],'robot3':original['robot3']}
            checks[f'{cp}_{row.solver_state}_{int(row.step)}_schema']=s.b.schema(tr,frame)
            keyed=tr.set_index(['robot_id','keyframe_id']);r=ref.loc[keyed.index]
            translation=np.linalg.norm(keyed[['tx','ty','tz']].to_numpy()-r[['tx','ty','tz']].to_numpy(),axis=1)
            q=['qx','qy','qz','qw'];rotation=np.rad2deg((Rotation.from_quat(r[q].to_numpy()).inv()*Rotation.from_quat(keyed[q].to_numpy())).magnitude())
            metrics=dict(translation_median_m=np.median(translation),translation_p95_m=np.percentile(translation,95),rotation_median_deg=np.median(rotation),rotation_p95_deg=np.percentile(rotation,95))
            checks[f'{cp}_{row.solver_state}_{int(row.step)}_metrics']=all(np.isclose(getattr(row,k),v,atol=1e-9,rtol=1e-8) for k,v in metrics.items())
            checks[f'{cp}_{row.solver_state}_{int(row.step)}_gate']=bool(row.PASS)==all(metrics[k]<=v for k,v in s.LIMITS.items())
            if row.solver_state.startswith('A'):
                for short in ['relinearized','reeliminated']:
                    raw=Decimal(str(getattr(row,short+'_raw')));value=getattr(row,short+'_valid_count');status=getattr(row,short+'_diagnostic')
                    if status=='INVALID_COUNTER':counter_valid &= pd.isna(value) and not (0<=raw<=len(tr))
                    else:counter_valid &= 0<=raw<=len(tr) and raw==Decimal(str(value))
    checks['invalid_counters_never_zeroed_clipped']=counter_valid
    eq=j('final_factor_equivalence.json')
    for cp,kf in s.CP.items():
        e=eq[cp];K=2 if cp=='K2' else 75
        checks[cp+'_actual_factor_keys']=e['template_indices']==s.indices(kf,K) and len(set(e['template_indices']))==e['factors'] and e['nodes']==3946+kf-149 and e['odometry']==e['nodes']-2 and e['selected_clean_loop_ids']==clean.head(K).frozen_loop_id.astype(int).to_list()
    checks['final5796_5870']=eq['FINAL_STREAM_END']['nodes']==5796 and eq['FINAL_STREAM_END']['factors']==5870 and eq['FINAL_STREAM_END']['loops']==75
    checks['no_new_catastrophes']=pd.read_csv(out/'trajectory_integrity_audit.csv').new_catastrophic_steps.eq(0).all()
    for file in (out/'nodewise').glob('*.csv'):checks[file.stem+'_finite_Logmap']=pd.read_csv(file).log_finite.all()
    support=pd.read_csv(out/'loop_support_diagnostic.csv');ends=clean.candidate_keyframe_id.to_numpy(int)
    for name,z in support.groupby('solver_state'):
        ids=z.keyframe_id.to_numpy(int);distance=abs(ids[:,None]-ends[None,:])
        checks[name+'_support_counts']=np.array_equal(distance.min(1),z.nearest_loop_endpoint_kf_distance) and all(np.array_equal((distance<=w).sum(1),z[f'endpoint_count_w{w}']) for w in [50,100,250])
    cond=j('conditioning_method.json');checks['conditioning_budget_and_no_dense']=cond['dense_global_Hessian'] is False and cond['policy']==policy['conditioning']
    if cond['status']=='PASS':
        w=pd.read_csv(out/'weak_direction_diagnostic.csv');checks['18_valid_6x6_marginals']=len(w)==18 and w.status.eq('VALID').all() and all(len(json.loads(x))==6 for x in w.eigenvalues_json)
        expected={(x['robot_id'],x['keyframe_id'],x['group']) for x in policy['conditioning']['poses']}
        checks['marginal_pose_set_frozen']=all(set(zip(z.robot_id,z.keyframe_id,z.group))==expected for _,z in w.groupby('linearization'))
    else:checks['conditioning_honest_NOT_TESTED']=cond['status']=='NOT_TESTED' and not (out/'09_weak_direction_diagnostic.png').exists()
    checks['required_figures']=len(list(out.glob('0*.png')))==(9 if cond['status']=='PASS' else 8)
    timeline=j('execution_events.json');freeze=next(x['UTC'] for x in timeline if x['phase']=='POLICY_FROZEN')
    checks['policy_before_trials']=policy['UTC']<freeze and all(freeze<x['UTC'] for x in timeline if x['phase'] in ['REBUILD_COMPLETE','LM_COMPLETE'])
    checks['no_GT_maps_demo']=j('summary.json')['GT_used'] is False and not list(out.rglob('*.ply')) and j('stage14_numerical_decision.json')['production_ready'] is False
    pair=pd.read_csv(out/'pairwise_solver_agreement.csv');basin=[];fresh=[];rebuild=[]
    for cp in ['K75','FINAL_STREAM_END']:
        for historical,new in [('D0_PERSISTENT','A2_FRESH_D0'),('D2_PERSISTENT','A3_FRESH_D2')]:
            old=numbers[numbers.checkpoint.eq(cp)&numbers.solver_state.eq(historical)].iloc[0];after=numbers[numbers.checkpoint.eq(cp)&numbers.solver_state.eq(new)].iloc[0]
            if (not old.PASS and after.PASS) or old.translation_p95_m-after.translation_p95_m>=max(.01,.2*old.translation_p95_m):rebuild.append(cp)
        if len(pair[pair.checkpoint.eq(cp)&pair.left.str.startswith('B')&pair.near_objective&~pair.original_gate_consistent]):basin.append(cp)
        if len(pair[pair.checkpoint.eq(cp)&pair.left.str.startswith('A')&~pair.original_gate_consistent]):fresh.append(cp)
    both=all(cp in rebuild for cp in ['K75','FINAL_STREAM_END']);bs=len(basin)==2;fs=len(fresh)==2
    expected_status='MULTIPLE_NUMERICAL_MECHANISMS_SUPPORTED' if both and bs else 'INITIALIZATION_OR_BASIN_SENSITIVITY' if bs and fs else 'FRESH_REBUILD_EXPLAINS_RESIDUAL' if both else 'RESIDUAL_NUMERICAL_DISCREPANCY_UNRESOLVED'
    checks['frozen_decision_rule']=j('stage14_numerical_decision.json')['decision']==expected_status
    recovery=out/'_failed_attempts/geometry_reporting'
    comparisons=[];archived=[]
    if recovery.exists():
        checks['recovery_exact_original_policy']=s.d.sha256(recovery/'stage14_experiment_policy.json')==s.d.sha256(out/'stage14_experiment_policy.json')
        for path in (recovery/'estimates').glob('*.csv'):
            newer=out/'estimates'/path.name;a=pd.read_csv(path);bb=pd.read_csv(newer)
            checks[path.stem+'_recovery_keys']=a[['robot_id','keyframe_id','timestamp']].equals(bb[['robot_id','keyframe_id','timestamp']])
            delta=np.max(abs(a[['tx','ty','tz','qx','qy','qz','qw']].to_numpy()-bb[['tx','ty','tz','qx','qy','qz','qw']].to_numpy()))
            comparisons.append(dict(file=path.name,max_coordinate_delta=float(delta),PASS=bool(delta<1e-7),old_sha256=s.d.sha256(path),new_sha256=s.d.sha256(newer)))
            archived.append(dict(path=str(path),sha256=s.d.sha256(path),size=path.stat().st_size,server_only=True))
        checks['recovery_numeric_reproduction']=len(comparisons)==66 and all(x['PASS'] for x in comparisons)
        s.csv('recovery_reproduction_audit.csv',comparisons);s.dump('failed_attempt_artifact_manifest.json',archived)
    checks={k:bool(v) for k,v in checks.items()};result=dict(PASS=all(checks.values()),checks=checks,failed=[k for k,v in checks.items() if not v],optimizer_invoked=False,GT_used=False)
    s.dump('independent_final_audit.json',result);print(json.dumps(dict(PASS=result['PASS'],failed=result['failed']),indent=2));assert result['PASS'],result['failed']

if __name__=='__main__':main()
