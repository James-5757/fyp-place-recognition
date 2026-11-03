"""Generate Stage23D documentation from frozen outputs; no GT-based selection."""
from common import *
import gzip

def table(df,cols):
    def fmt(x):
        if isinstance(x,(list,dict)):return json.dumps(x)
        if pd.isna(x):return 'not measured'
        return f'{x:.6g}' if isinstance(x,(float,np.floating)) else str(x)
    return '| '+' | '.join(cols)+' |\n| '+' | '.join('---' for _ in cols)+' |\n'+'\n'.join('| '+' | '.join(fmt(r[c]) for c in cols)+' |' for r in df.to_dict('records'))+'\n'
def main():
    decision=read(OUT/'stage23d_decision.json');audit=read(OUT/'p5_utility_audit.json');post=read(OUT/'p5_postfreeze_rejection_summary.json');validation=read(OUT/'scheduler_causal_validation.json');cost=pd.read_csv(OUT/'p6_ablation_comparison.csv');paired=pd.read_csv(MAP/'paired_trajectory_difference.csv');regions=pd.read_csv(MAP/'regional_comparison.csv');browser=read(MAP/'browser_validation.json') if (MAP/'browser_validation.json').exists() else dict(status='NOT_RUN')
    stats=[]
    for mode,f in paired.groupby('condition'):
        for r,g in [('joint',f)]+list(f.groupby('robot_id')):
            row=dict(condition=mode,robot=r)
            for metric in ['translation_difference_m','rotation_difference_deg']:
                for label,value in [('median',g[metric].median()),('p95',g[metric].quantile(.95)),('max',g[metric].max())]:row[metric+'_'+label]=float(value)
            stats.append(row)
    csv(MAP/'paired_trajectory_summary.csv',stats)
    # Availability/coverage comparison remains GT-free, not Recall/ATE.
    p5=cost[cost.policy=='P5_FROZEN'].iloc[0];cost.loc[cost.policy=='P5_FROZEN','first_bridge_event']=post['P5_first_bridge_event'];cost.loc[cost.policy=='P5_FROZEN','covered_odometry_bin_pairs']=post['P5_covered_odometry_bin_pairs']
    calls_cap=validation['hard_limits']['calls'];bytes_cap=validation['hard_limits']['bytes'];cost['call_utilization']=cost.completed_receipt_decisions/calls_cap;cost['byte_utilization']=cost.application_bytes/bytes_cap;cost['bytes_saved_vs_frozen_P5']=float(p5.application_bytes)-cost.application_bytes
    for field in ['delayed_verifications','queue_expirations','queue_drops','unknown_outcomes','peak_queue_queries']:cost.loc[cost.policy=='P5_FROZEN',field]=0
    csv(OUT/'resource_comparison_summary.csv',cost)
    tr=pd.read_csv(OUT/'p6_queue_event_trace.csv');v=tr[tr.action=='VERIFIED'];cpu_summary=[]
    for p,f in v.groupby('policy'):
        cpu_summary.append(dict(policy=p,predicted_CPU_ms=float(f.predicted_CPU_ms.sum()),sum_existing_receipt_CPU_ms=float(f.actual_receipt_CPU_ms.sum()),meaning='historical receipt timing projection, NOT new GICP execution or scheduler runtime'))
    csv(OUT/'receipt_compute_summary.csv',cpu_summary)
    notes='''# Stage23D — budget scheduling and frozen mapping comparison

## Separate decisions
'''+table(pd.DataFrame([decision]),['P0_P5_FINAL_DEMO','P5_UTILITY_AUDIT','P6_SCHEDULER_PROTOTYPE','P6_INDEPENDENT_ACCURACY'])+'''
Independent scientific validation remains **NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE**. PASS here validates presentation and scheduler implementation, not P6 mapping quality. No policy is promoted from this GT-exposed recording.

## Experiment record and protected scope
Frozen reference: Stage23C `5ed035d891053c93a22321fb6a809f849ceb20d1`. All changes are new Stage23D paths. The compact input manifest retains essential input/scan hashes and a SHA256 reference to the comprehensive server-only gzip inventory of historical files. Both are checked; no protected file was reset, stashed or rewritten. Existing unrelated dirty demo and Stage10.3 files remain untouched. This document records the new experiment without rewriting frozen context/history files.

## Track A — real final-map demo
`demo/stage23d_policy_map_compare/` reuses pinned Three.js r170 and OrbitControls. Real frozen P0/P5 incremental and Batch trajectories each contain5796 valid Pose3 nodes,5794 original odometry factors and the same Robot1 KF150 prior. Source counts are P0=288, P5=231, common147, P0-only141, P5-only84. Original timestamps/keyframe IDs match. Gauge-anchor matrices are checked against the frozen local pose; no best-fit or GT presentation transform is applied.

580 identical original scans (frame stride10), point stride16, finite range1–60m and per-source voxel0.5m reconstruct both incremental PLYs within the explicit float32 comparison tolerance2e-5m. Original source-file SHA256 matches are required. Batch maps use the same raw scans and their actual existing Batch poses; no solver is run. Full voxel geometry is used for diagnostics; browser geometry is a deterministic25000-point cap per source, labeled accordingly. XYZ is never flattened or rescaled.

Side-by-side panels keep cameras synchronized for rotation, zoom and pan. Trajectory overlay preserves identical original KF identities; the opposite policy is dashed. Loop identity modes separate common/P0-only/P5-only. Regional navigation distinguishes the seven frozen temporal missing-loop groups (which may span spatial regions) from four50m local-odometry bin pairs. Offline metrics are separate from GT-free display geometry. Point overlap is not proof of physical accuracy, and arbitrary global NN p95 is not literal alignment error.

Exact per-event/per-loop P0/P5 optimized states are not available; historical final trajectories and Stage20's different clean75 snapshots cannot substitute for them. **PROGRESSIVE_STATE_EXPORT_REQUIRED**. This demo deliberately remains a final-state inspector. A later separately authorized exact state replay must use the original frozen backend parameters.

Paired pose differences, with no alignment between policies (these are NOT GT errors):
'''+table(pd.DataFrame(stats),['condition','robot','translation_difference_m_median','translation_difference_m_p95','translation_difference_m_max','rotation_difference_deg_median','rotation_difference_deg_p95','rotation_difference_deg_max'])+'''
`regional_comparison.csv` reports per-region paired poses, displacement of identical raw points, and within-source P0-vs-P5 NN. `source_matched_map_consistency.json` separately reports R1-vs-R3 symmetric NN for each real state and region, including the fraction/summary restricted to distances<5m. This cutoff is a diagnostic truncation, not a tuned acceptance gate or a GT-positive definition. Regional empty subsets are explicitly unavailable. Global metrics retain non-overlap effects and are not calibrated map accuracy. Original ATE/RPE are imported from Stage22B, not recalculated or used to choose a policy. The P5 historical precision/recall failure remains unchanged even if its joint ATE happens to be smaller here.

## Track B — exact heuristic audit
Utility = confidence +0.1 normalized margin +0.1 novelty −0.1 redundancy −0.05 normalized predicted CPU −0.05 normalized transfer. The exact historical normalizations, weights, threshold0.5 and reconstruction error are stored in `p5_utility_audit.json`. These weights were predeclared heuristics, not derived optimal mapping utility or calibrated probability of map improvement. Only the threshold was development-tested on events0–999 using measured geometric acceptance; heldout GT was not used to fit it.

Frozen-state sensitivity (remove one contribution; count threshold flips). This is descriptive only: changing a term in a real replay would also change later cache/admission state, so these counts are not outcomes of new policies.
'''+table(pd.DataFrame([dict(term=k,median_absolute_contribution=audit['absolute_contribution_medians'][k],fixed_state_threshold_flips=audit['drop_one_term_threshold_flip_counts_fixed_state'][k]) for k in audit['absolute_contribution_medians']]),['term','median_absolute_contribution','fixed_state_threshold_flips'])+'''
Budget underutilization/rejection evidence:

```json
'''+json.dumps(dict(remaining_global_bytes=audit['remaining_global_bytes'],**post),indent=2)+'''
```

Nonexecuted higher ranks after a geometric success are early-stop rows, not budget denials. First-denial query counts are therefore reported separately from inherited per-rank reasons. Rolling timeline includes fixed publication plus actual variable bytes, calls and predicted CPU. `p5_rejection_analysis.csv` includes SC rank/confidence, busy status and the first later frozen-trace capacity event within100events. This is a descriptive reopening at the OLD trace's after-action state, not a claim the candidate would execute or be useful in a rescheduled stream. Known/unknown receipt availability is added only after the complete scheduler trace freezes; no GT annotation is needed.

## Track C — predeclared queue prototype
`P6_DEFERRED_LEXICOGRAPHIC` is represented by D3. All variants share the original candidate discovery, unbounded logical central cache, exact serialized cloud request+response1048681bytes and admitted loop249bytes, canonical identities, geometric receipts and duplicate rule (deltaR1<=3 AND deltaR3<=5). No source graph, Scan Context, GICP or backend noise/solver is changed.

Queue maximum256queries, age100events; reconsider only arrived endpoints. Global calls/bytes and rolling100-event calls/bytes use frozen C75/B80 limits; reserve fixed full-stream publication globally and a potential249byte loop before verification. The historical compute budget is a CALL cap, not physical CPU time. Additional predicted-CPU ceilings are call caps times the largest frozen predicted cost, identical and redundant across variants. Existing receipt timings are projections; actual new GICP calls remainzero.

D0 is a no-queue control with the same non-weighted eligibility as D1. D1 uses FIFO deferral. D2 adds priority classes: first bridge opportunity, uncovered own-admission endpoint-bin pair, cached/low-transfer candidate, then rank/confidence with deterministic original-event/identity ties. D3 additionally orders the least-served robot direction by attempts/arrivals within a class. These classes and queue limits are frozen engineering hypotheses, not theoretical optimality. D1 vs frozen P5 also changes weighted eligibility; only D0 vs D1 isolates queueing inside the new family.

Only the next original query rank is eligible. Measured failure unlocks the next rank; any geometric success closes the query, even if the resulting factor is a suppressed graph duplicate. Expiration/invalid/query-drop closes higher ranks too. A later query whose active canonical pair is already pending is dropped explicitly. Current GICP quality/acceptance and GT are never inspected before selection.

Selection is logged before testing whether its receipt exists. Missing results yield UNKNOWN_REQUIRES_ACTUAL_GICP and halt that policy at the unresolved branch; neither acceptance nor rejection is fabricated. The selected experimental traces required no unknown outcomes: original sequential early-stop dependencies avoided the unmeasured higher ranks. The2112 original unknown candidate combinations remain unmeasured. Synthetic tests explicitly verify that a genuinely missing selected outcome halts with PARTIAL.

Observed resource and admission results:
'''+table(cost,['policy','completed_receipt_decisions','delayed_verifications','queue_expirations','application_bytes','bytes_saved_vs_frozen_P5','byte_utilization','first_bridge_event','admitted_factors','covered_odometry_bin_pairs','unknown_outcomes','runtime_s'])+'''
Negative saved bytes mean MORE traffic than P5. More utilization/attempts/coverage does not establish better maps; factor counts can decrease. Direction coverage, exact caps, reconstructed cache/rolling usage and rank dependencies are in `scheduler_causal_validation.json`. Runtime includes scheduler work, frozen receipt I/O and receipt-hash validation, not new GICP or graph optimization. Peak RSS is process high-water mark and therefore cumulative across sequential variants, not isolated per-policy memory.

No new P6 pose graph or GT evaluation was run. Admitted factors are candidate inputs for a later independently reconstructed graph, not a completed mapping comparison or proof of accuracy. Complete Recall/ATE and P6 independent accuracy remain NOT_RUN. The experiment does not establish that smarter scheduling uses limited resources more effectively for mapping; the causal implementation is working and the mixed/negative resource/admission observations are retained.

## Validation, browser evidence and reproduction
The saved-artifact auditor independently reconstructs received cloud identities, canonical pair admissions, cumulative/rolling budgets, source receipt hashes and early-stop rank dependencies. Synthetic tests cover future access, unknown outcomes, expiration, byte caps, rank-success stopping and equivalence of cached versus original candidate-context scanning. A performance repair caches unchanged contexts across budget denials and invalidates after each executed outcome; interrupted CPU time and unchanged completed D0 results are retained in `engineering_recovery.json`. It changes no priority, scientific threshold or budget.

The browser test uses local Chromium with software SwiftShader, not a hardware-accelerated foreground rehearsal or a SLAM benchmark. It checks file loading, identities, synchronized cameras, overlays, all11regions, Batch switching, cached buffers, toggles and disposal. Screenshots preserve incremental, regional-overlay and Batch gauge-XY views. Startup and test timing are recorded, without claiming60FPS, network throughput or live operation.

Browser report:

```json
'''+json.dumps(browser,indent=2)+'''
```

Commands from repository root:

```bash
bash scripts/run_stage23d_demo.sh
bash scripts/run_stage23d_scheduler.sh
bash scripts/run_stage23d_demo.sh --serve
```

Open http://127.0.0.1:8024/stage23d_policy_map_compare/ after serving the parent demo directory. A local Windows server can use the absolute path shown in the new README. The comprehensive historical gzip inventory and full plain-text traces remain server-side with SHA256; essential inputs, summary/required outputs and deterministic presentation assets are versioned. The full queue and event-budget traces additionally have versioned lossless `.csv.gz` copies, with original/archive SHA256 and a checked decompression roundtrip in `trace_archive_manifest.json`; decompress to regenerate their plain CSVs. No raw LiDAR, ROS bag, secret or model is committed. Browser reproduction needs optional existing Playwright/Chromium; without imported browser evidence the demo status is explicitly PARTIAL/NOT_RUN.

## Limitations and stopping point
Separate recordings on a normalized hypothetical dual-stream clock, centralized backend, protocol-simulated application bytes, original tilted gauge XYZ and exposed historical GT. Queue eligibility/classification are engineering choices, not an accepted accuracy-preserving method. No distributed SLAM, physical network claims, independent final testing or large GICP extension occurred. Stop after Stage23D. A separately authorized follow-up could reconstruct actual P6 graphs under unchanged backend settings and prospectively evaluate independent recordings with positive bidirectional overlap; no current result justifies P6 promotion.
'''
    (ROOT/'docs/CUMULTI_STAGE23D.md').write_text(notes)
    for name in ['p6_queue_event_trace.csv','p6_budget_event_trace.csv']:
        raw=(OUT/name).read_bytes();packed=OUT/(name+'.gz');packed.write_bytes(gzip.compress(raw,mtime=0));assert gzip.decompress(packed.read_bytes())==raw
    dump(OUT/'trace_archive_manifest.json',{n:dict(original_sha256=sha(OUT/n),archive=n+'.gz',archive_sha256=sha(OUT/(n+'.gz')),original_bytes=(OUT/n).stat().st_size,archive_bytes=(OUT/(n+'.gz')).stat().st_size,lossless_roundtrip=True) for n in ['p6_queue_event_trace.csv','p6_budget_event_trace.csv']})
    dump(MAP/'demo_asset_manifest.json',{str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in DEMO.rglob('*') if p.is_file()})
    dump(OUT/'stage23d_artifact_manifest.json',{str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p),server_only=p.name in ['historical_integrity_inventory.json.gz','p6_queue_event_trace.csv','p6_budget_event_trace.csv','p6_verified_factors.csv','p5_utility_components.csv','p5_rejection_receipt_availability.csv']) for folder in [MAP,OUT] for p in folder.iterdir() if p.is_file() and p.name!='stage23d_artifact_manifest.json'})
    print('STAGE23D_DOCUMENTATION_COMPLETE',flush=True)
if __name__=='__main__':main()
