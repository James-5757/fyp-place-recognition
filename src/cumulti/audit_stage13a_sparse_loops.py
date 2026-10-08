#!/usr/bin/env python3
"""Read-only numerical/provenance audit; writes only a new Stage13A audit JSON.

No optimization, LiDAR extraction or GT evaluation is performed here.
"""
import json
from pathlib import Path
import numpy as np
import pandas as pd
import run_stage13a_sparse_loops as a

def main():
    out=a.OUT
    j=lambda name:json.loads((out/name).read_text())
    checks={}
    manifest=j('stage13a_input_manifest.json')
    checks['input_hashes_unchanged']=all(a.d.sha256(x['path'])==x['sha256'] for x in manifest['inputs'])
    immutable=j('frozen_input_immutability_audit.json')
    checks['historical_files_unchanged']=all(a.d.sha256(p)==h for p,h in immutable['hashes_before'].items())
    decision=j('pre_gt_sparse_k_decision.json')
    solver=pd.read_csv(out/'gtsam_results_by_K.csv')
    checks['seven_independent_graphs']=solver.K.to_list()==a.KS and solver.nodes.eq(5796).all() and solver.odometry_factors.eq(5794).all() and solver.loop_factors.to_list()==a.KS and solver.prior_factors.eq(1).all()
    checks['identical_initialization']=solver.initialization_sha256.nunique()==1
    checks['finite_normal_solves']=solver.finite_poses.all() and solver.termination.eq('NORMAL_RETURN').all() and not solver.iteration_limit_reached.any()
    checks['anchor_fixed']=solver.anchor_translation_m.lt(1e-7).all() and solver.anchor_rotation_deg.lt(1e-6).all()
    reps=pd.concat([pd.read_csv(out/f'solve_repetitions_K{k}.csv') for k in a.KS])
    checks['28_independent_solves']=len(reps)==28 and all(reps[reps.K.eq(k)].repetition.to_list()==[0,1,2,3] for k in a.KS)
    schemas={str(k):a.d.schema_audit(pd.read_csv(out/f'trajectories/K{k}_optimized.csv')) for k in a.KS}
    checks['trajectory_schemas']=all(x['PASS'] for x in schemas.values())
    for filename in ['trajectory_artifact_manifest.json','map_artifact_manifest.json']:
        artifacts=j(filename)
        if isinstance(artifacts,dict): artifacts=artifacts['artifacts']
        checks[filename]=all(Path(x['path']).stat().st_size==x['size_bytes'] and a.d.sha256(x['path'])==x['sha256'] for x in artifacts)
    loops=pd.read_csv(out/'loop_residuals_by_K.csv'); summary=pd.read_csv(out/'loop_residual_summary_by_K.csv')
    checks['525_frozen_measurement_diagnostics']=len(loops)==525 and all(len(loops[loops.K.eq(k)])==75 for k in a.KS)
    empty=summary[summary.K.eq(75)&summary.group.eq('NOT_YET_INSERTED_OFFLINE')]
    checks['empty_future_set_NA']=len(empty)==1 and empty['count'].iloc[0]==0 and pd.isna(empty.translation_median_m.iloc[0])
    checks['no_new_catastrophes']=pd.read_csv(out/'trajectory_integrity_by_K.csv').new_catastrophic_steps.eq(0).all()
    checks['K1_K75_reproduction']=j('stage12d_reproduction_audit.json')['K1_PASS'] and j('stage12d_reproduction_audit.json')['K75_PASS']
    maps=pd.read_csv(out/'map_consistency_by_K.csv')
    eligible=maps[maps.K.lt(75)&maps.median_m.le(1.1*maps.iloc[-1].median_m)&maps.p95_m.le(1.1*maps.iloc[-1].p95_m)]
    expected=int(eligible.K.iloc[0]) if len(eligible) else None
    checks['frozen_rule_recomputed']=decision['selected_K']==expected and decision['GT_used'] is False
    # Old current-process run predates event persistence; import its actual JSON
    # progress log, not invented timestamps. Future runner runs persist directly.
    if (out/'execution_events.json').is_file(): events=j('execution_events.json')
    else:
        events=[json.loads(line) for line in Path('/tmp/fyp_stage13a_run.log').read_text().splitlines() if line.startswith('{')]
        a.dump('execution_events.json',events)
    phases={x['phase']:x for x in events}
    completed=[x for x in events if x['phase']=='K_COMPLETE_GT_FREE']
    checks['chronological_GT_isolation']=len(completed)==7 and phases['POLICIES_FROZEN']['UTC']<max(x['UTC'] for x in completed)<phases['PRE_GT_DECISION_PERSISTED']['UTC']<phases['FIRST_GT_ACCESS']['UTC']
    tdecision=(out/'pre_gt_sparse_k_decision.json').stat().st_mtime_ns
    before=['stage13a_policy.json','fixed_overlap_roi_policy.json','sparse_k_selection_policy.json','runtime_by_K.csv','map_consistency_by_K.csv','overlap_map_consistency_by_K.csv','loop_residuals_by_K.csv','loop_residual_summary_by_K.csv']+[f'solve_repetitions_K{k}.csv' for k in a.KS]
    checks['metrics_and_repetitions_precede_decision']=all((out/name).stat().st_mtime_ns<tdecision for name in before)
    checks['GT_output_after_decision']=(out/'offline_gt_evaluation_by_K.csv').stat().st_mtime_ns>tdecision
    gt=pd.read_csv(out/'offline_gt_evaluation_by_K.csv')
    checks['all42_GT_rows']=len(gt)==42 and gt['use'].eq('OFFLINE GT DIAGNOSTIC ONLY').all()
    for k in a.KS:
        r=gt[gt.K.eq(k)&gt.metric_type.eq('RPE')].set_index('scope')
        for metric in ['translation_RMSE_m','rotation_RMSE_deg']:
            expected=np.sqrt(sum(r.loc[robot,metric]**2*r.loc[robot,'sample_count'] for robot in ['robot1','robot3'])/sum(r.loc[robot,'sample_count'] for robot in ['robot1','robot3']))
            checks[f'K{k}_RPE_union_{metric}']=bool(np.isclose(expected,r.loc['combined',metric],rtol=1e-12,atol=1e-12))
    checks['9_figures']=len(list(out.glob('0*.png')))==9
    checks={k:bool(v) for k,v in checks.items()}
    a.dump('independent_final_audit.json',dict(PASS=all(checks.values()),checks=checks,optimization_rerun=False,GT_evaluation_rerun=False))
    print(json.dumps(checks,indent=2)); assert all(checks.values()),checks

if __name__=='__main__': main()
