#!/usr/bin/env python3
"""Independent saved-artifact audit: no optimizer, GT or report rewriting."""
import hashlib
import json
from decimal import Decimal
import numpy as np
import pandas as pd
import run_stage13d_solver_diagnostics as s

def main():
    out=s.OUT;j=lambda n:json.loads((out/n).read_text());checks={}
    manifest=j('stage13d_input_manifest.json');policy=j('stage13d_experiment_policy.json')
    checks['accepted_inputs_match']=all(s.d.sha256(x['path'])==x['sha256'] for x in manifest['inputs']+manifest['inherited_frozen_inputs'])
    checks['history_immutable']=all(s.d.sha256(p)==h for p,h in manifest['historical_hashes_before'].items())
    checks['original_gates']=policy['limits']==dict(translation_median_m=.1,translation_p95_m=.5,rotation_median_deg=.5,rotation_p95_deg=2.)
    checks['fixed_conditions']=policy['checkpoints']==s.CP and [(x['name'],x['skip'],x['threshold'],x['extra']) for x in policy['variants']]==[('D0',10,.1,0),('D1',10,.1,5),('D2',1,.01,5)]
    timeline=j('execution_events.json');freeze=next(x['UTC'] for x in timeline if x['phase']=='POLICY_FROZEN')
    checks['policy_before_api_and_runs']=policy['UTC']<freeze and all(freeze<x['UTC'] for x in timeline if x['phase'] in ['COUNTER_API_AUDITED','VARIANT_START'])
    trace=pd.read_csv(out/'checkpoint_microtrace.csv');timing=pd.read_csv(out/'diagnostic_update_timing.csv');numbers=pd.read_csv(out/'numerical_convergence_summary.csv')
    original=pd.read_csv(s.d.s10.PROC/'robot1/keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns']).iloc[150:]
    loops=pd.read_csv(s.b.D12/'ordered_rank1_clean_loops.csv')
    for variant in ['D0','D1','D2']:
        ev=timing[timing.variant.eq(variant)]
        checks[variant+'_arrived_order_and_timestamps']=ev.kf.to_list()==list(range(283,2000)) and ev.timestamp_ns.to_list()==original[original.keyframe_id.ge(283)].lidar_timestamp_ns.to_list()
        checks[variant+'_actual_nodes']=ev.nodes.to_list()==(ev.kf+3797).to_list()
        checks[variant+'_only_frozen_loop_arrivals']=ev[ev.kind.isin(['FIRST_FUSION','LOOP'])].kf.to_list()==loops.query_keyframe_id.to_list()
        checks[variant+'_single_insertion_per_event']=ev.iloc[0].new_values==4080 and ev.iloc[1:].new_values.eq(1).all()
        for cp,kf in s.CP.items():
            sub=trace[trace.variant.eq(variant)&trace.checkpoint.eq(cp)]
            after=sub[sub.extra_step.ge(0)].sort_values('extra_step');steps=[0] if variant=='D0' else list(range(6))
            checks[variant+'_'+cp+'_schedule']=after.extra_step.to_list()==steps and len(sub[sub.stage.eq('BEFORE_EVENT')])==1
            checks[variant+'_'+cp+'_no_extra_graph_mutation']=after.graph_nodes.nunique()==1 and after.graph_factors.nunique()==1 and after.loop_factors.nunique()==1
            if kf>283:
                before=sub[sub.stage.eq('BEFORE_EVENT')].iloc[0]
                checks[variant+'_'+cp+'_before_own_graph']=before.current_kf==kf-1 and before.graph_nodes==kf+3796 and before.loop_factors==len(loops[loops.query_keyframe_id.lt(kf)])
            else:checks[variant+'_disconnected_before']=sub.iloc[0].comparison_status=='NOT_APPLICABLE_DISCONNECTED_LOCAL_FRAMES' and sub.iloc[0].graph_nodes==0
            for row in sub[sub.graph_nodes.gt(0)].itertuples():
                tr=pd.read_csv(out/f'checkpoints/{variant}_{cp}_{row.stage}_{int(row.extra_step)}.csv')
                frame={'robot1':original[original.keyframe_id.le(int(row.current_kf))],'robot3':pd.DataFrame({'keyframe_id':range(234,4180)})}
                checks[f'{variant}_{cp}_{row.stage}{int(row.extra_step)}_schema']=s.b.schema(tr,frame)
                vals=pd.read_csv(out/f'pose_differences/{variant}_{cp}_{row.stage}_{int(row.extra_step)}.csv')
                checks[f'{variant}_{cp}_{row.stage}{int(row.extra_step)}_quantiles']=all(np.isclose(vals[col].quantile(q),getattr(row,metric),atol=1e-12,rtol=1e-10) for col,q,metric in [('translation_m',.5,'translation_median_m'),('translation_m',.95,'translation_p95_m'),('rotation_deg',.5,'rotation_median_deg'),('rotation_deg',.95,'rotation_p95_deg')])
                checks[f'{variant}_{cp}_{row.stage}{int(row.extra_step)}_gates']=bool(row.PASS)==all(getattr(row,k)<=limit for k,limit in policy['limits'].items())
        checks[variant+'_extra_only_selected']=ev[ev.extra_update_ms.gt(0)].kf.to_list()==list(s.CP.values()) if variant!='D0' else ev.extra_update_ms.eq(0).all()
    checks['D0_reproduced']=pd.read_csv(out/'D0_reproduction_audit.csv').PASS.all()
    checks['all_final_graphs']=len(j('final_factor_equivalence.json'))==3 and all(x['PASS'] and x['nodes']==5796 and x['factors']==5870 and x['loops']==75 for x in j('final_factor_equivalence.json').values())
    refs=pd.read_csv(out/'batch_reference_provenance.csv');checks['only_five_missing_before_refs_solved']=len(refs)==11 and refs.reused.sum()==6 and (~refs.reused).sum()==5
    checks['reference_hashes_match']=all(s.d.sha256(out/f'batch_references/{r.label}.csv')==r.saved_sha256 for r in refs[~refs.reused].itertuples())
    counters=pd.read_csv(out/'counter_validity_audit.csv',dtype=str,keep_default_na=False);valid=True
    for r in counters.itertuples():
        if r.diagnostic_status=='INVALID_COUNTER':valid=valid and r.diagnostic_value==''
        elif r.diagnostic_status=='IN_RANGE_UNVERIFIED_SEMANTICS':
            value=Decimal(r.raw_value);valid=valid and value==int(value) and 0<=value<=int(r.graph_nodes) and Decimal(r.diagnostic_value)==value
    checks['raw_invalid_never_clipped_zeroed']=valid
    residual=pd.read_csv(out/'solver_loop_residual_comparison.csv')
    checks['no_future_loop_residuals']=all(r.query_kf<=s.CP[r.checkpoint]-(1 if r.stage=='BEFORE_EVENT' else 0) for r in residual.itertuples())
    checks['no_catastrophes']=pd.read_csv(out/'trajectory_integrity_audit.csv').new_catastrophic_steps.eq(0).all()
    decision=j('stage13d_numerical_decision.json');qualifiers=[v for v,sub in numbers.groupby('variant') if len(sub)==6 and sub.final_PASS.all()]
    checks['decision_truthful']=decision['qualifying_diagnostic_schedules']==qualifiers and decision['GT_used'] is False and decision['production_ready'] is False
    checks['no_GT_maps']=not list(out.rglob('*.ply')) and not list(out.glob('*gt_evaluation*'))
    checks['8_figures']=len(list(out.glob('0*.png')))==8
    checks={k:bool(v) for k,v in checks.items()};result=dict(PASS=all(checks.values()),checks=checks,failed=[k for k,v in checks.items() if not v],optimizer_invoked=False,GT_used=False)
    s.dump('independent_final_audit.json',result);print(json.dumps(dict(PASS=result['PASS'],failed=result['failed']),indent=2));assert result['PASS'],result['failed']

if __name__=='__main__':main()
