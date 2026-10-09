#!/usr/bin/env python3
"""Independent artifact/schedule/policy audit; no optimizer, batch solve or GT."""
import json
from pathlib import Path
import numpy as np
import pandas as pd
import run_stage13c_isam2_convergence as c

def main():
    out=c.OUT;j=lambda n:json.loads((out/n).read_text());checks={}
    manifest=j('stage13c_input_manifest.json')
    checks['accepted_inputs_unchanged']=all(c.d.sha256(x['path'])==x['sha256'] for x in manifest['inputs'])
    checks['historical_files_unchanged']=all(c.d.sha256(p)==h for p,h in manifest['historical_hashes_before'].items())
    policy=j('stage13c_experiment_policy.json');params=j('variant_parameter_manifest.json');limits=policy['agreement_limits']
    checks['declared_variants_unchanged']=policy['variants']==c.VARIANTS and policy['checkpoints']==c.CP
    checks['original_thresholds']=limits==dict(translation_median_m=.1,translation_p95_m=.5,rotation_median_deg=2./4,rotation_p95_deg=2.)
    events=pd.read_csv(out/'per_event_solver_diagnostics.csv');numbers=pd.read_csv(out/'numerical_agreement_by_variant.csv');objectives=pd.read_csv(out/'objective_agreement_by_variant.csv')
    originals=pd.read_csv(c.d.s10.PROC/'robot1/keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns']).iloc[150:]
    loops=pd.read_csv(c.b.D12/'ordered_rank1_clean_loops.csv')
    expected_snapshots=0
    for v in c.VARIANTS:
        name=v['name'];short=name[:2];ev=events[events.variant.eq(name)];post=ev[ev.update_kind.ne('PRE_FUSION_LOCAL_ONLY')]
        checks[short+'_all1850events']=ev.robot1_keyframe.to_list()==list(range(150,2000))
        checks[short+'_timestamps']=ev.timestamp_ns.to_list()==originals.lidar_timestamp_ns.to_list()
        checks[short+'_1717insertion_updates']=len(post)==1717 and post.iloc[0].new_values==4080 and post.iloc[1:].new_values.eq(1).all()
        checks[short+'_only_arrived_nodes']=post.graph_nodes.to_list()==(post.robot1_keyframe-150+1+3946).to_list()
        checks[short+'_loop_sequence']=post[post.loop_inserted].robot1_keyframe.to_list()==loops.query_keyframe_id.to_list() and post.cumulative_loops.iloc[-1]==75
        checks[short+'_parameter_manifest']=next(x for x in params if x['name']==name)['skip']==v['skip'] and ev.relinearizeSkip.eq(v['skip']).all() and ev.relinearizationThreshold.eq(v['threshold']).all()
        actual,text=c.configure(v);checks[short+'_printed_API_settings']=text==next(x for x in params if x['name']==name)['parameter_text']
        for cp,kf in c.CP.items():
            tr=pd.read_csv(out/f'checkpoints/{short}/{cp}.csv')
            frame={'robot1':originals[originals.keyframe_id.le(kf)],'robot3':pd.DataFrame({'keyframe_id':range(234,4180)})}
            checks[short+'_'+cp+'_schema']=c.b.schema(tr,frame);expected_snapshots+=1
            row=numbers[numbers.variant.eq(name)&numbers.checkpoint.eq(cp)].iloc[0]
            checks[short+'_'+cp+'_gates_truthful']=bool(row.PASS)==all(row[x]<=lim for x,lim in limits.items())
            vals=pd.read_csv(out/f'checkpoints/{short}/{cp}_pose_differences.csv')
            checks[short+'_'+cp+'_quantiles']=np.isclose(vals.translation_m.quantile(.95),row.translation_p95_m,atol=1e-12) and np.isclose(vals.rotation_deg.quantile(.95),row.rotation_p95_deg,atol=1e-12)
        checks[short+'_finalschema']=c.d.schema_audit(pd.read_csv(out/f'trajectories/{short}/final_trajectory.csv'))['PASS']
    checks['40checkpoints']=expected_snapshots==40 and len(numbers)==40
    checks['all_equivalent']=all(x['PASS'] for x in j('final_graph_equivalence_by_variant.json').values()) and len(j('final_graph_equivalence_by_variant.json'))==5
    checks['no_new_catastrophes']=pd.read_csv(out/'trajectory_integrity_by_variant.csv').new_catastrophic_steps.eq(0).all()
    checks['V0_reproduced']=pd.read_csv(out/'baseline_reproduction_audit.csv').PASS.all()
    extra=pd.read_csv(out/'extra_update_diagnostic.csv')
    checks['V4_exact225emptycalls']=len(extra)==300 and len(extra[extra.extra_step.gt(0)])==225 and all(x.extra_step.to_list()==[0,1,2,3] for _,x in extra.groupby('arrival_index'))
    checks['empty_calls_no_new_factors']=extra[extra.extra_step.gt(0)].new_factors.eq(0).all() and extra[extra.extra_step.gt(0)].new_values.eq(0).all()
    artifacts=j('extra_estimate_manifest.json')
    checks['75estimate_archives']=len(artifacts)==75 and all(c.d.sha256(x['path'])==x['sha256'] and Path(x['path']).stat().st_size==x['size_bytes'] for x in artifacts)
    timeline=j('execution_events.json');freeze=next(x['UTC'] for x in timeline if x['phase']=='POLICY_FROZEN')
    checks['policy_before_every_replay']=policy['UTC']<freeze and all(freeze<x['UTC'] for x in timeline if x['phase']=='VARIANT_START')
    decision=j('stage13c_numerical_decision.json');checks['GT_free_selection']=decision['GT_used_for_selection'] is False and decision['selected_variant']!=c.VARIANTS[-1]['name']
    qualifies=[x for x in decision['candidate_results'] if x['eligible']]
    selected=None
    if qualifies:
        best=min(x['post_update_median_ms'] for x in qualifies);ties=[x for x in qualifies if x['post_update_median_ms']<=best*1.01]
        pbest=min(x['post_update_p95_ms'] for x in ties);ties=[x for x in ties if x['post_update_p95_ms']<=pbest*1.01];selected=min(ties,key=lambda x:int(x['variant'][1]))['variant']
    checks['selection_rule_recomputed']=selected==decision['selected_variant']
    checks['selection_qualification_truthful']=all(x['eligible']==(not x['variant'].startswith('V4') and numbers[numbers.variant.eq(x['variant'])].PASS.all()) for x in decision['candidate_results'])
    checks['no_maps_or_GT_outputs']=not list(out.rglob('*.ply')) and not list(out.glob('*offline_gt*')) and j('summary.json')['batch_solves']==0
    checks['9figures']=len(list(out.glob('0*.png')))==9
    checks={k:bool(v) for k,v in checks.items()};c.dump('independent_final_audit.json',dict(PASS=all(checks.values()),checks=checks,optimizer_rerun=False,GT_loaded=False))
    print(json.dumps(dict(PASS=all(checks.values()),failed=[k for k,v in checks.items() if not v]),indent=2));assert all(checks.values()),checks

if __name__=='__main__':main()
