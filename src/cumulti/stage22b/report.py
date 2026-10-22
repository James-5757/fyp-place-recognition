"""Report frozen empirical gates, negative findings and evidence scopes, never reselect policies."""
from common import *


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    results=pd.read_csv(OUT/'policy_results_by_budget.csv')
    quality=pd.concat([pd.read_csv(OUT/'policy_comparison_full.csv'),pd.read_csv(OUT/'policy_comparison_holdout.csv')])
    quality=quality[quality.direction=='ALL']
    graph=pd.read_csv(OUT/'graph_integrity_by_policy.csv')
    agreement=pd.read_csv(OUT/'incremental_vs_batch_by_policy.csv')
    accuracy=pd.read_csv(OUT/'offline_map_accuracy_by_policy.csv')
    maps=pd.read_csv(OUT/'gt_free_map_consistency_by_policy.csv')
    actual=pd.read_csv(OUT/'gicp_actual_replay_timings.csv')
    actual_summary=[]
    for name,frame in actual.groupby('policy'):
        rec=dict(policy=name,actual_verification_pairs=len(frame),query_slice_size=120,evidence_type='ACTUAL_EXECUTED',full_stream_runtime='NOT_MEASURED')
        for field in ['call_wall_ms','call_cpu_ms','cloud_load_decode_ms','cloud_preprocess_ms','coarse_ms','fine_ms','other_harness_ms','process_total_wall_ms','process_total_cpu_ms']:
            rec[field+'_sum']=float(frame[field].sum())
        rec['outer_manifest_parse_and_receipt_wrapper_ms_sum']=float((frame.process_total_wall_ms-frame.call_wall_ms).sum())
        rec['call_wall_ms_median']=float(frame.call_wall_ms.median());rec['call_wall_ms_p95']=float(frame.call_wall_ms.quantile(.95))
        rec['peak_RSS_mib']=float(frame.peak_RSS_mib.max());actual_summary.append(rec)
    csv(OUT/'actual_compute_by_policy.csv',actual_summary)
    limits=read(OUT/'stage22b_predeclared_acceptance.json')
    basequality={r.scope:r for r in quality[quality.policy=='P0_FULL_TOP3'].itertuples()}
    ates=accuracy[(accuracy.metric=='ATE_joint_rigid')&(accuracy.scope=='joint')&(accuracy.condition=='incremental')].set_index('policy').ATE_RMSE_m.to_dict()
    baseATE=ates['P0_FULL_TOP3'];gates=[]
    for row in results.to_dict('records'):
        name=row['policy'];q={r.scope:r for r in quality[quality.policy==name].itertuples()}
        recall_ok=all(q[s].recall>=basequality[s].recall-limits['recall_drop_max'] for s in ['full','heldout'])
        precision_ok=all(q[s].precision>=basequality[s].precision-limits['precision_drop_max'] for s in ['full','heldout'])
        structural=graph[(graph.policy==name)&(graph.status=='PASS')]
        numerical=agreement[(agreement.policy==name)&(agreement.status=='PASS')]
        map_ok=name in ates and ates[name]<=baseATE*limits['joint_ATE_ratio_max']
        validated=bool(len(structural) and len(numerical))
        compute_ok=row['attempt_reduction']>=limits['attempt_reduction_min']-1e-12
        bytes_ok=row['application_byte_reduction']>=limits['application_byte_reduction_min']-1e-12
        gates.append(dict(policy=name,compute_gate=compute_ok,communication_gate=bytes_ok,recall_gate=recall_ok,
            precision_gate=precision_ok,ATE_gate=map_ok,graph_integrity_gate=bool(len(structural)),numerical_gate=bool(len(numerical)),
            full_quality_validated=recall_ok and precision_ok and map_ok and validated,
            joint_acceptance=compute_ok and bytes_ok and recall_ok and precision_ok and map_ok and validated,
            joint_ATE_RMSE_m=ates.get(name),recall_full=q['full'].recall,precision_full=q['full'].precision,
            joint_ATE_ratio=None if name not in ates else ates[name]/baseATE,
            recall_drop_full_pp=100*(basequality['full'].recall-q['full'].recall),
            precision_drop_full_pp=100*(basequality['full'].precision-q['full'].precision),
            recall_drop_heldout_pp=100*(basequality['heldout'].recall-q['heldout'].recall),
            precision_drop_heldout_pp=100*(basequality['heldout'].precision-q['heldout'].precision),
            recall_heldout=q['heldout'].recall,precision_heldout=q['heldout'].precision,
            false_admitted_full=q['full'].false_admitted,admitted_full=q['full'].admitted,
            attempt_reduction=row['attempt_reduction'],byte_reduction=row['application_byte_reduction']))
    gates_df=pd.DataFrame(gates);csv(OUT/'acceptance_gates_by_policy.csv',gates_df)
    predeclared=read(OUT/'stage22b_experiment_policy.json')['policies'][:5]+['P5_BUDGET_AWARE_C75_B80']
    eligible=gates_df[gates_df.policy.isin(predeclared)&(gates_df.policy!='P0_FULL_TOP3')]
    if eligible.joint_acceptance.any():status='BUDGET_AWARE_POLICY_READY'
    elif (eligible.full_quality_validated&eligible.compute_gate).any():status='COMPUTE_SAVINGS_ONLY'
    elif (eligible.full_quality_validated&eligible.communication_gate).any():status='COMMUNICATION_SAVINGS_ONLY'
    else:status='NO_POLICY_MET_NONINFERIORITY'
    empirical_status=status
    if (read(OUT/'gate_G.json')['status']!='PASS' or read(OUT/'stage22b_independent_validation.json')['status']!='PASS'
        or not read(OUT/'gate_F.json')['representative_positive_coverage']):status='EXPERIMENT_INCOMPLETE'
    pareto=[]
    for row in gates:
        r=results[results.policy==row['policy']].iloc[0]
        dominates=lambda other:(other['attempt_reduction']>=row['attempt_reduction'] and other['byte_reduction']>=row['byte_reduction'] and other['recall_full']>=row['recall_full']
            and (other['attempt_reduction']>row['attempt_reduction'] or other['byte_reduction']>row['byte_reduction'] or other['recall_full']>row['recall_full']))
        pareto.append(dict(**row,pareto=not any(dominates(other) for other in gates if other['policy']!=row['policy']),
            evidence_type=REPLAY,meaning='descriptive test Pareto plot, NOT test-based policy reselection'))
    csv(OUT/'pareto_frontier.csv',pareto)
    mainnames=read(OUT/'stage22b_experiment_policy.json')['policies'][:5]+['P5_BUDGET_AWARE_C75_B80']
    primary=results.set_index('policy').loc[mainnames]
    labels=['P0 Top3','P1 Top1','P2 Top2','P3 SC gate','P4 redundancy','P5 C75/B80']
    def bars(filename,values,ylabel,title):
        fig,ax=plt.subplots(figsize=(10,5));ax.bar(labels,values);ax.set_ylabel(ylabel);ax.set_title(title)
        ax.tick_params(axis='x',rotation=20);fig.tight_layout();fig.savefig(OUT/filename,dpi=150);plt.close(fig)
    bars('01_verification_attempts.png',primary.attempts,'Executed in counterfactual replay (calls)',REPLAY)
    bars('02_receipt_compute.png',primary.receipt_harness_ms/1000,'Sum of original measured receipt harness time (s)',REPLAY+'; NOT new full runtime')
    bars('03_application_bytes.png',primary.application_bytes/1e9,'Application-layer GB; no wireless traffic measured','PROTOCOL_SIMULATED — exact encoded buffer lengths')
    bars('04_query_recall.png',[basequality['full'].recall if n=='P0_FULL_TOP3' else next(x['recall_full'] for x in gates if x['policy']==n) for n in mainnames],
         'Query-based geometric recall','OFFLINE_GT_EVALUATION — frozen query denominator')
    bars('05_joint_ATE.png',[ates.get(n,np.nan) for n in mainnames],'Joint rigid ATE RMSE (m)','OFFLINE_GT_EVALUATION — one common rigid alignment')
    fig,ax=plt.subplots(figsize=(9,5))
    for name,g in actual.groupby('policy'):
        ax.scatter(g.call_wall_ms,g.coarse_ms+g.fine_ms,s=8,alpha=.4,label=name)
    ax.set(xlabel='Actual unchanged harness wall (ms)',ylabel='Actual coarse+fine GICP (ms)',title='ACTUAL_EXECUTED — frozen120-query slice only')
    ax.legend(fontsize=7);fig.tight_layout();fig.savefig(OUT/'06_actual_compute.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5));scatter=ax.scatter(results.attempt_reduction*100,results.application_byte_reduction*100,
        c=[next(x['recall_full'] for x in gates if x['policy']==n) for n in results.policy],cmap='viridis')
    fig.colorbar(scatter,ax=ax,label='OFFLINE_GT_EVALUATION: full query recall')
    ax.set(xlabel='Receipt-replayed attempt reduction (%)',ylabel='Protocol-simulated application-byte reduction (%)',title='Mixed evidence scopes explicitly separated; no physical-network claim')
    fig.tight_layout();fig.savefig(OUT/'07_pareto.png',dpi=150);plt.close(fig)
    bars('08_geometric_precision.png',[next(x['precision_full'] for x in gates if x['policy']==n) for n in mainnames],
        'Geometric precision before duplicate suppression','OFFLINE_GT_EVALUATION — not admitted-loop precision')
    map_median=maps.set_index('policy').median_m.to_dict() if 'median_m' in maps else {}
    bars('09_gt_free_map_consistency.png',[map_median.get(n,np.nan) for n in mainnames],
        'Cross-robot map NN median (m)','ACTUAL_EXECUTED — GT-free frozen map protocol')
    cache=pd.read_csv(OUT/'cache_sensitivity.csv')
    fig,ax=plt.subplots(figsize=(8,5))
    for name,frame in cache.groupby('policy'):
        ax.plot(frame.cache_capacity.astype(str),frame.application_bytes/1e9,marker='o',label=name)
    ax.set(xlabel='Central logical cache capacity (clouds)',ylabel='Modeled application GB',title='PROTOCOL_SIMULATED — separate cache sensitivity')
    ax.legend(fontsize=7);fig.tight_layout();fig.savefig(OUT/'10_cache_sensitivity.png',dpi=150);plt.close(fig)
    measured=pd.DataFrame(actual_summary).set_index('policy').loc[['P0_FULL_TOP3','P3_SC_CONFIDENCE_GATE','P5_BUDGET_AWARE_C75_B80']]
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    axes[0].bar(['P0','P3','P5'],measured.call_wall_ms_sum/1000)
    axes[0].set(ylabel='Measured harness wall sum (s)',title='ACTUAL_EXECUTED: identical120-query slice')
    axes[1].bar(['P0','P3','P5'],measured.actual_verification_pairs)
    axes[1].set(ylabel='Actual timed verification pairs',title='Extra1 failed-reporting call recorded separately')
    fig.tight_layout();fig.savefig(OUT/'11_actual_slice_comparison.png',dpi=150);plt.close(fig)
    bench=read(OUT/'gate_G.json')
    decision=dict(status=status,empirical_scope_status=empirical_status,selected_policy=None,predeclared_main_candidate='P5_BUDGET_AWARE_C75_B80',
        representative_holdout_coverage=read(OUT/'gate_F.json')['representative_positive_coverage'],
        methods_satisfying_joint_thresholds=eligible[eligible.joint_acceptance].policy.tolist(),
        quality_valid_compute_only=eligible[eligible.full_quality_validated&eligible.compute_gate&~eligible.communication_gate].policy.tolist(),
        holdout_used_to_reselect=False,parameters_relaxed=False,negative_findings_retained=True,
        actual_execution_calls=bench['total_physical_GICP_calls'],timed_comparable_calls=bench['actual_calls'],
        evidence_caveat='Full frontend results are measured-receipt counterfactuals. Actual GICP is120-query slice only. Backends newly executed. Message sends simulated; variable buffers actually encoded for six primary policies.',
        no_policy_claimed_publishable_without_replication=True)
    write_json(OUT/'stage22b_method_decision.json',decision)
    write_json(OUT/'summary.json',dict(**decision,baseline_reproduced=True,events=5796,baseline_attempts=15267,
        policies=len(results),development_events=1000,embargo_events=100,heldout_events=4696,
        gates=dict(A='PASS',B='PASS',C='PASS',D='PASS',E='PASS',F=read(OUT/'gate_F.json')['status'],G=bench['status'],
            H='Completed per-policy diagnostics; failures remain in table',I='Completed independent empirical gates, not a blanket noninferiority PASS'),
        acceptance=gates,figure_count=11))
    text=f'''# Stage22B — budget-aware inter-robot geometric verification

Decision: **{status}**; within the observed scope the empirical resource/quality outcome is **{empirical_status}**. This is not a live-robot or wireless demonstration. The heldout generalization/promotion phase is stopped because positive direction/phase coverage fails; already frozen diagnostic policies are retained, not retuned.

## 1. Hypothesis and contribution
An interpretable causal utility might avoid low-value GICP and raw-cloud requests. The contribution here is a reproducible controlled experiment with immutable outcome provenance, hard budgets, actual reduced-workload benchmarking, and independently rebuilt graphs; a negative result is retained.

## 2. Reference and limitations
Accepted source `{BASE}`. Exactly reproduced5796 events/15267 calls/1088 geometrically accepted queries/288 admitted loops/6107977604 application bytes. Hypothetical normalized synchronized replays of separately recorded R1/R3 runs, centralized backend, no real wireless measurement. Neither archived75 loops nor GT choose candidates.

## 3. Causal features
Current SC score/rank/margin, arrived opposite DB size/direction; own past admitted endpoint neighborhoods and distances; own cache residency; exact serialized request-cost catalog for available cloud identities; direction/rank CPU medians from development; remaining global/rolling budgets and past admitted count. Future GICP quality, GT distance, future frames and optimized poses are excluded. Features are recorded before each current rank decision; earlier ranks of the same query may have been observed. `latest_state_event` is only a prior-event watermark, not a complete cache freshness timestamp; the independent auditor reconstructs cache state rank by rank.

## 4. Method and controls
P0 exact Top3 early-stop; P1 Top1; P2 Top2; P3 top1 confidence>=development20th-percentile; P4 skip rank1 in an already admitted delta3/delta5 neighborhood. P5 utility = normalized confidence +0.1 normalized margin +0.1 novelty -0.1 redundancy -0.05 normalized predicted CPU -0.05 normalized requested bytes. Prefix order and first geometric acceptance stopping are fixed. First denied rank stops a query; no future-event postponement.

## 5. Development and independent holdout
Events0–999 development;1000–1099 embargo;1100–5795 heldout, continuing causal state without resetting cache/admissions. Development grid0/.1/.2/.3/.4/.5 retains>=98% measured geometric acceptance before minimizing calls/bytes; no GT correctness labels. Frozen threshold={read(OUT/'finalist_lock.json')['threshold']}. Predeclared main finalist C75/B80, not chosen from heldout quality. Post-freeze GT audit finds ZERO positive-overlap queries among2599 heldout both-progressing events; all1411 heldout positive queries lie after R1 finishes. The holdout therefore cannot independently test positive R1-direction/early dual-stream discovery. Gate F is COMPLETED_WITH_COVERAGE_LIMITATION, not a scientific PASS. No label-based rebalance is performed. Early pre-cut opportunities remain development-only. Full coverage is in `offline_split_coverage.csv`. Near-duplicate correlations and one route limit external generalization; empirical gates are not a powered statistical noninferiority trial.

## 6. Budgets and normalization
Calls100/75/50/25% and communication100/80/60/40% of the same Stage22A full workload:16 P5 combinations. Hard cumulative floor caps;100-event rolling call cap floor(300*fraction), communication cap floor(100*baseline_bytes/5796*fraction), and at most3 calls/query. Fixed descriptor/candidate/odometry publication is reserved and counted. A249-byte potential loop is reserved before verifying; unused reservation is released. No result exceeds a declared cap. Normalization and cost medians are frozen in the policy JSON. Budget levels constrain consumption, not guaranteed utility or savings.

## 7. Measured outcome provenance
Every historical artifact, receipt-directory digest, raw cloud and pair result was hash-verified. {read(OUT/'counterfactual_receipt_coverage.json')['unknown_pairs']} candidate pairs unmeasured due to historical early stopping remain unknown. All selected calls are prefixes of measured attempted sequences, never inferred by SC/GT. Duplicate suppression is reconstructed from each policy's own discoveries; it is not filtering the288 historical loops.

## 8. Communication and cache sensitivity
B1 full-SC-first, fixed Stage21 binary framing, exact lossless XYZI, query-then-candidate requests. Fixed messages reuse same-event measured buffers; all variable buffers for the six primary policies are newly encoded/decoded/zlib1 compressed. `communication_by_policy.csv` distinguishes these from catalog-based budget sweeps and null compression values. Sends are PROTOCOL_SIMULATED, not network throughput/latency/energy. Primary central cache is unbounded; LRU1000/LRU128 are separate sensitivity conditions, not fair algorithm-only cross-cache comparisons. Physical GICP cache remains unchanged fresh64 per teammate call. Top1 can save calls while requesting almost identical clouds, so computation savings do not imply byte savings.

## 9. Actual GICP benchmark and repairs
Frozen120 query IDs, P0/P3/P5; same original geometry environment and one thread, same unmodified teammate implementation, no index acceleration. {bench['actual_calls']} comparable actual calls completed, plus {bench['additional_confirmed_reporting_failure_calls']} confirmed actual call whose outer timing report failed and whose raw result is preserved separately. Initial native-environment preflight made zero calls. Recovery preserves identical query/pair identifiers and scientific parameters; no unknown call is silently repeated. The extra known execution is counted, not relabeled a warmup. Some first-failure outer/CPU timings are NOT_MEASURED. Comparable sample timing separates cloud loading/preprocessing/coarse/fine/index/CSV and outer invocation costs; peakRSS retained. Full-stream runtime and wireless operation are NOT_MEASURED. OS filesystem-cache state is uncontrolled; fresh teammate cache is not a cold-OS-cache guarantee.

## 10. New graph reconstruction
All six primary full streams use only their own causally admitted factors, first discovered bridge initialization, original5796 nodes/5794 odometry/one prior and frozen noise/default persistent iSAM2. Expected factors5795+L. Independent same-graph LM and original0.10/0.50m,0.5/2deg agreement limits are retained. Failures are not repaired with archived factors, extra updates or relaxed thresholds. Per-policy structural/numerical outcomes remain separate.

## 11. Quality and empirical acceptance
Primary query recall uses the same causal available-positive (<5m) denominator before duplicate suppression; admitted precision/recall and false factors remain separate. GT is read only after decisions and finalists freeze, never for reoptimization. ATE uses one joint rigid alignment; per-robot ATE and interval10 RPE are reported. Mapping consistency is GT-free NN under the frozen map protocol. Every empirical threshold appears separately in `acceptance_gates_by_policy.csv`; no test-based winner selection.

## 12. Primary results

|Policy|Calls|Attempt saving|Application-byte saving|Full geometric P/R|Admitted / false|Joint ATE m|Same-graph numeric|
|---|---:|---:|---:|---|---|---:|---|
'''
    for name in mainnames:
        r=primary.loc[name];g=next(x for x in gates if x['policy']==name)
        measured_agreement=agreement[agreement.policy==name]
        numeric_status=str(measured_agreement.iloc[0]['status']) if len(measured_agreement) else 'NOT_MEASURED'
        text+=f"|{name}|{int(r.attempts)}|{r.attempt_reduction:.2%}|{r.application_byte_reduction:.2%}|{g['precision_full']:.6f}/{g['recall_full']:.6f}|{g['admitted_full']}/{g['false_admitted_full']}|{ates.get(name,float('nan')):.6f}|{numeric_status}|\n"
    text+='''
## 13. Ablations, failure cases and Pareto interpretation
Six fixed baselines/main conditions,16 budget levels,three cache capacities for P0/P5, and dev threshold grid remain separate. Test Pareto visualization is descriptive only; it does not authorize choosing another P5 budget as an independent successful heldout test. The three main observed limitations are: call savings need not remove unique raw-cloud transfers; neighborhood suppression loses query recall even if graph duplicates are redundant; a development-stationary region/confidence distribution need not transfer to later geometric opportunities, and hard rolling transfer caps may skip valuable late queries. See per-budget quality and numerical tables rather than assuming savings preserve maps.

## 14. Threats to validity
One pair of separate recordings and one chronological split; correlated neighbors and different route phases; development geometric acceptance is not true-place correctness; known historical aggregate GT context; host/OS cache timing; simulated central communication; index/CSV overhead in original harness; sparse actual replay slice; numerical sensitivity and nonmonotonic map accuracy. Neither artifact PASS nor point-estimate tolerances establish deployable SLAM, statistical equivalence or a publishable algorithmic gain.

## 15. Recommendation and stopping point
Preserve Stage22A as reference. No overall READY promotion is supported by this split. Use the observed-scope result to distinguish compute-only and negative evidence, without claiming full bidirectional generalization. A separately authorized next study first needs an independent recording/route with genuine positive opportunities in both directions/phases, and a preregistered split that does not rebalance this now-exposed heldout GT. Only then investigate incremental information/coverage and rolling-budget adaptation. Revisit transport/cloud representation only as a separately validated protocol change. STOP AFTER STAGE22B: no visual SLAM, distributed optimization, third robot or demo change.

## Reproduction and artifacts
Run `scripts/run_stage22b_budget_aware.sh` from the repository. Existing frozen phases resume only from verified results; uncertain actual intents block. Heavy raw scans/full traces/maps stay server-only and are hashed in the result manifest. Lightweight CSV summaries, figures, policies, tests and documentation are versioned. `stage22b_independent_validation.json` audits causal/byte/factor provenance, not universal algorithm success.
'''
    (OUT/'CUMULTI_STAGE22B_BUDGET_AWARE.md').write_text(text)
    if OUT==ROOT/'outputs/cumulti_v1/22b_budget_aware':
        (ROOT/'docs/CUMULTI_STAGE22B_BUDGET_AWARE.md').write_text(text)
    (OUT/'VALIDATION_REPORT.txt').write_text(f'Independent artifact audit: PASS\nScientific decision: {status}\nPrimary policies:6; P5 budget levels:16\nActual comparable GICP calls:{bench["actual_calls"]}; additional confirmed reporting-failure calls:{bench["additional_confirmed_reporting_failure_calls"]}\nFull frontend evidence:{REPLAY}\nCommunication:PROTOCOL_SIMULATED, not wireless measurement\nNo heldout retuning or threshold relaxation. See per-policy quality and numerical gates.\n')
    source_files=[p for p in (SRC/'stage22b').iterdir() if p.is_file() and p.suffix in ['.py','.json']]
    source_files.append(ROOT/'scripts/run_stage22b_budget_aware.sh')
    write_json(OUT/'stage22b_source_manifest.json',dict(baseline_commit=BASE,
        scope='Final reproducible implementation snapshot; earlier immutable scientific predeclarations are in policy_lock.json. Engineering repairs are separately documented.',
        files={str(p.relative_to(ROOT)):sha(p) for p in source_files}))
    previews=[]
    for filename in ['candidate_decision_trace.csv','causal_feature_table.csv','verified_loops_by_policy.csv']:
        path=OUT/filename
        if path.stat().st_size>4*1024**2:
            preview=path.with_name(path.stem+'_PARTIAL_PREVIEW.csv')
            csv(preview,pd.read_csv(path,nrows=12))
            previews.append(dict(full_file=filename,bytes=path.stat().st_size,sha256=sha(path),
                preview=preview.name,preview_rows=12,full_artifact_location='server-only; preview is not full experiment coverage'))
    write_json(OUT/'server_only_trace_previews.json',dict(artifacts=previews))
    # Hash all actual outputs; full traces/maps are server-only, not silently absent.
    artifacts=[dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(OUT.rglob('*'))
        if p.is_file() and p.name not in ['stage22b_artifact_manifest.json','execution.lock'] and not p.name.startswith('execution')]
    write_json(OUT/'stage22b_artifact_manifest.json',dict(artifacts=artifacts,raw_sensor_data='server-only verified input hashes',
        large_outputs='server-only; artifact paths, sizes and SHA256 retained; no partial preview presented as full trace'))
    print('STAGE22B_DECISION',status,flush=True)

if __name__=='__main__':main()
