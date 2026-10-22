"""Gate G: genuinely re-execute finalized policies on the frozen120-query slice."""
import argparse
import contextlib
import importlib.util
import resource
import time
from common import *


def main():
    import open3d  # Preflight before recording any actual-call intent.
    verify_lock();assert (OUT/'pre_gt_decision_lock.json').exists()
    guard()
    import run_stage22a_verification as original
    from stage22a_common import TEAM
    spec=importlib.util.spec_from_file_location('stage22b_frozen_teammate',TEAM)
    team=importlib.util.module_from_spec(spec);spec.loader.exec_module(team)
    policy=read(OUT/'stage22b_experiment_policy.json')
    candidate_columns=pd.read_csv(A/'online_candidate_trace.csv',nrows=0).columns.tolist()
    selected=set(policy['actual_reexecution_query_ids']);plans={};coverage={};rows=[]
    for name in policy['actual_reexecution_policies']:
        frame=pd.read_csv(OUT/'policies'/name/'selected_attempts.csv')
        plans[name]=frame[frame.event_index.isin(selected)].to_dict('records')
    plan=dict(query_ids=sorted(selected),pairs={name:[[int(r['event_index']),int(r['candidate_rank'])] for r in records] for name,records in plans.items()},
        input_lock_sha256=sha(OUT/'policy_lock.json'),pre_gt_decision_sha256=sha(OUT/'pre_gt_decision_lock.json'),
        evidence_type='ACTUAL_EXECUTED',frozen_unix_ns=time.time_ns())
    planpath=OUT/'actual_execution_plan.json'
    if planpath.exists():assert read(planpath)['pairs']==plan['pairs']
    else:write_json(planpath,plan)
    beginning=time.monotonic()
    for name,records in plans.items():
        directory=OUT/'actual'/name;directory.mkdir(parents=True,exist_ok=True)
        completed=0
        for row in records:
            ident=f"e{int(row['event_index']):04d}_r{int(row['candidate_rank'])}"
            receipt=directory/(ident+'.json');raw=directory/(ident+'.csv');intent=directory/(ident+'.intent.json')
            if receipt.exists():
                saved=read(receipt);assert sha(raw)==saved['result_csv_sha256'];rows.append(saved);completed+=1;continue
            assert not intent.exists() and not raw.exists(),'Uncertain actual execution: do not silently repeat'
            if time.monotonic()-beginning>=policy['benchmark_limit_seconds']:
                coverage[name]=dict(planned=len(records),completed=completed,status='TIME_CAP_INCOMPLETE');break
            write_json(intent,dict(policy=name,event=int(row['event_index']),rank=int(row['candidate_rank']),plan_sha256=sha(planpath)))
            start=time.perf_counter();cpu=time.process_time()
            with (directory/'teammate_stdout.log').open('a') as output,contextlib.redirect_stdout(output),contextlib.redirect_stderr(output):
                result=original.actual_call(team,{key:row[key] for key in candidate_columns},raw)
            transform_error=float(np.max(abs(np.array(result['verified_transform'])-np.array(json.loads(row['verified_transform'])))))
            quality_error=abs(result['gicp_quality']-row['gicp_quality'])
            saved={k:result[k] for k in ['event_index','candidate_rank','call_wall_ms','call_cpu_ms','cloud_load_decode_ms','cloud_preprocess_ms','coarse_ms','fine_ms','other_harness_ms','peak_RSS_mib','result_csv_sha256']}
            saved.update(policy=name,evidence_type='ACTUAL_EXECUTED',process_total_wall_ms=(time.perf_counter()-start)*1000,
                process_total_cpu_ms=(time.process_time()-cpu)*1000,transform_max_abs_error=transform_error,quality_abs_error=quality_error,
                acceptance_equal=result['geometrically_accepted']==row['geometrically_accepted'],
                original_receipt_evidence=REPLAY,threads=1,cloud_index_optimization_applied=False)
            write_json(receipt,saved);rows.append(saved);completed+=1
            csv(OUT/'gicp_actual_replay_timings.csv',rows)
            if (not saved['acceptance_equal'] or transform_error>policy['benchmark_transform_tolerance'] or quality_error>policy['benchmark_quality_tolerance']):
                write_json(OUT/'gate_G.json',dict(status='FAIL_EQUIVALENCE',completed=len(rows),last=saved));raise RuntimeError('GICP equivalence gate failed')
            if completed%25==0:print('ACTUAL',name,completed,'/',len(records),flush=True)
        else:coverage[name]=dict(planned=len(records),completed=completed,status='COMPLETE')
    csv(OUT/'gicp_actual_replay_timings.csv',rows)
    extra_calls=int((OUT/'actual_failed_reporting/P0_FULL_TOP3/e0048_r1.csv').exists())
    write_json(OUT/'gate_G.json',dict(status='PASS' if all(v['status']=='COMPLETE' for v in coverage.values()) and len(coverage)==len(plans) else 'INCOMPLETE',
        evidence_type='ACTUAL_EXECUTED',full_stream_reexecuted=False,queries_per_policy=120,coverage=coverage,
        actual_calls=len(rows),additional_confirmed_reporting_failure_calls=extra_calls,
        total_physical_GICP_calls=len(rows)+extra_calls,
        wall_seconds=time.monotonic()-beginning,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        actual_network_bytes='NOT_MEASURED'))

if __name__=='__main__':main()
