# Stage23D — budget scheduling and frozen mapping comparison

## Separate decisions
| P0_P5_FINAL_DEMO | P5_UTILITY_AUDIT | P6_SCHEDULER_PROTOTYPE | P6_INDEPENDENT_ACCURACY |
| --- | --- | --- | --- |
| PASS | COMPLETE | PASS | NOT_RUN |

Independent scientific validation remains **NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE**. PASS here validates presentation and scheduler implementation, not P6 mapping quality. No policy is promoted from this GT-exposed recording.

## Experiment record and protected scope
Frozen reference: Stage23C `5ed035d891053c93a22321fb6a809f849ceb20d1`. All changes are new Stage23D paths. The compact input manifest retains essential input/scan hashes and a SHA256 reference to the comprehensive server-only gzip inventory of historical files. Both are checked; no protected file was reset, stashed or rewritten. Existing unrelated dirty demo and Stage10.3 files remain untouched. This document records the new experiment without rewriting frozen context/history files.

## Track A — real final-map demo
`demo/stage23d_policy_map_compare/` reuses pinned Three.js r170 and OrbitControls. Real frozen P0/P5 incremental and Batch trajectories each contain5796 valid Pose3 nodes,5794 original odometry factors and the same Robot1 KF150 prior. Source counts are P0=288, P5=231, common147, P0-only141, P5-only84. Original timestamps/keyframe IDs match. Gauge-anchor matrices are checked against the frozen local pose; no best-fit or GT presentation transform is applied.

580 identical original scans (frame stride10), point stride16, finite range1–60m and per-source voxel0.5m reconstruct both incremental PLYs within the explicit float32 comparison tolerance2e-5m. Original source-file SHA256 matches are required. Batch maps use the same raw scans and their actual existing Batch poses; no solver is run. Full voxel geometry is used for diagnostics; browser geometry is a deterministic25000-point cap per source, labeled accordingly. XYZ is never flattened or rescaled.

Side-by-side panels keep cameras synchronized for rotation, zoom and pan. Trajectory overlay preserves identical original KF identities; the opposite policy is dashed. Loop identity modes separate common/P0-only/P5-only. Regional navigation distinguishes the seven frozen temporal missing-loop groups (which may span spatial regions) from four50m local-odometry bin pairs. Offline metrics are separate from GT-free display geometry. Point overlap is not proof of physical accuracy, and arbitrary global NN p95 is not literal alignment error.

Exact per-event/per-loop P0/P5 optimized states are not available; historical final trajectories and Stage20's different clean75 snapshots cannot substitute for them. **PROGRESSIVE_STATE_EXPORT_REQUIRED**. This demo deliberately remains a final-state inspector. A later separately authorized exact state replay must use the original frozen backend parameters.

Paired pose differences, with no alignment between policies (these are NOT GT errors):
| condition | robot | translation_difference_m_median | translation_difference_m_p95 | translation_difference_m_max | rotation_difference_deg_median | rotation_difference_deg_p95 | rotation_difference_deg_max |
| --- | --- | --- | --- | --- | --- | --- | --- |
| batch | joint | 1.25844 | 4.10858 | 5.42842 | 1.12574 | 4.11226 | 10.5105 |
| batch | robot1 | 1.44583 | 4.33089 | 5.42842 | 1.50916 | 4.40938 | 10.5105 |
| batch | robot3 | 1.24411 | 3.71779 | 4.7571 | 1.07798 | 3.9457 | 7.23948 |
| incremental | joint | 1.26497 | 4.06583 | 5.37643 | 1.12538 | 4.09586 | 10.4891 |
| incremental | robot1 | 1.43249 | 4.29014 | 5.37643 | 1.50089 | 4.38921 | 10.4891 |
| incremental | robot3 | 1.25257 | 3.68036 | 4.70873 | 1.07562 | 3.92584 | 7.2261 |

`regional_comparison.csv` reports per-region paired poses, displacement of identical raw points, and within-source P0-vs-P5 NN. `source_matched_map_consistency.json` separately reports R1-vs-R3 symmetric NN for each real state and region, including the fraction/summary restricted to distances<5m. This cutoff is a diagnostic truncation, not a tuned acceptance gate or a GT-positive definition. Regional empty subsets are explicitly unavailable. Global metrics retain non-overlap effects and are not calibrated map accuracy. Original ATE/RPE are imported from Stage22B, not recalculated or used to choose a policy. The P5 historical precision/recall failure remains unchanged even if its joint ATE happens to be smaller here.

## Track B — exact heuristic audit
Utility = confidence +0.1 normalized margin +0.1 novelty −0.1 redundancy −0.05 normalized predicted CPU −0.05 normalized transfer. The exact historical normalizations, weights, threshold0.5 and reconstruction error are stored in `p5_utility_audit.json`. These weights were predeclared heuristics, not derived optimal mapping utility or calibrated probability of map improvement. Only the threshold was development-tested on events0–999 using measured geometric acceptance; heldout GT was not used to fit it.

Frozen-state sensitivity (remove one contribution; count threshold flips). This is descriptive only: changing a term in a real replay would also change later cache/admission state, so these counts are not outcomes of new policies.
| term | median_absolute_contribution | fixed_state_threshold_flips |
| --- | --- | --- |
| confidence | 0.635427 | 13326 |
| margin | 0.00553358 | 103 |
| novelty | 0.1 | 2193 |
| redundancy | 0 | 10 |
| CPU | 0.0499141 | 729 |
| transfer | 0.025 | 330 |

Budget underutilization/rejection evidence:

```json
{
  "remaining_global_bytes": 706943031,
  "meaning": "descriptive after trace freeze, not scheduler inputs",
  "nonexecuted_candidate_rows": 8732,
  "early_stop_rows_not_budget_rejections": 1472,
  "actually_denied_query_count": 2552,
  "first_denial_reasons": {
    "UTILITY_BELOW_THRESHOLD": 1379,
    "COMMUNICATION_BUDGET": 1153,
    "COMPUTE_BUDGET": 20
  },
  "first_denials_during_busy_windows": 1224,
  "busy_first_denials_with_later_100event_capacity": 1147,
  "known_nonexecuted_rows": 6620,
  "unknown_nonexecuted_rows": 2112,
  "known_denied_rows": 6620,
  "unknown_denied_rows": 640,
  "events_with_two_uncached_cloud_headroom": 2821,
  "P5_first_bridge_event": 200,
  "P5_covered_odometry_bin_pairs": 49
}
```

Nonexecuted higher ranks after a geometric success are early-stop rows, not budget denials. First-denial query counts are therefore reported separately from inherited per-rank reasons. Rolling timeline includes fixed publication plus actual variable bytes, calls and predicted CPU. `p5_rejection_analysis.csv` includes SC rank/confidence, busy status and the first later frozen-trace capacity event within100events. This is a descriptive reopening at the OLD trace's after-action state, not a claim the candidate would execute or be useful in a rescheduled stream. Known/unknown receipt availability is added only after the complete scheduler trace freezes; no GT annotation is needed.

## Track C — predeclared queue prototype
`P6_DEFERRED_LEXICOGRAPHIC` is represented by D3. All variants share the original candidate discovery, unbounded logical central cache, exact serialized cloud request+response1048681bytes and admitted loop249bytes, canonical identities, geometric receipts and duplicate rule (deltaR1<=3 AND deltaR3<=5). No source graph, Scan Context, GICP or backend noise/solver is changed.

Queue maximum256queries, age100events; reconsider only arrived endpoints. Global calls/bytes and rolling100-event calls/bytes use frozen C75/B80 limits; reserve fixed full-stream publication globally and a potential249byte loop before verification. The historical compute budget is a CALL cap, not physical CPU time. Additional predicted-CPU ceilings are call caps times the largest frozen predicted cost, identical and redundant across variants. Existing receipt timings are projections; actual new GICP calls remainzero.

D0 is a no-queue control with the same non-weighted eligibility as D1. D1 uses FIFO deferral. D2 adds priority classes: first bridge opportunity, uncovered own-admission endpoint-bin pair, cached/low-transfer candidate, then rank/confidence with deterministic original-event/identity ties. D3 additionally orders the least-served robot direction by attempts/arrivals within a class. These classes and queue limits are frozen engineering hypotheses, not theoretical optimality. D1 vs frozen P5 also changes weighted eligibility; only D0 vs D1 isolates queueing inside the new family.

Only the next original query rank is eligible. Measured failure unlocks the next rank; any geometric success closes the query, even if the resulting factor is a suppressed graph duplicate. Expiration/invalid/query-drop closes higher ranks too. A later query whose active canonical pair is already pending is dropped explicitly. Current GICP quality/acceptance and GT are never inspected before selection.

Selection is logged before testing whether its receipt exists. Missing results yield UNKNOWN_REQUIRES_ACTUAL_GICP and halt that policy at the unresolved branch; neither acceptance nor rejection is fabricated. The selected experimental traces required no unknown outcomes: original sequential early-stop dependencies avoided the unmeasured higher ranks. The2112 original unknown candidate combinations remain unmeasured. Synthetic tests explicitly verify that a genuinely missing selected outcome halts with PARTIAL.

Observed resource and admission results:
| policy | completed_receipt_decisions | delayed_verifications | queue_expirations | application_bytes | bytes_saved_vs_frozen_P5 | byte_utilization | first_bridge_event | admitted_factors | covered_odometry_bin_pairs | unknown_outcomes | runtime_s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P5_FROZEN | 8647 | 0 | 0 | 4179439052 | 0 | 0.855324 | 200 | 231 | 49 | 0 | not measured |
| P6_D0_IMMEDIATE_CONTROL | 10386 | 0 | 0 | 4823332921 | -6.43894e+08 | 0.987097 | 203 | 246 | 51 | 0 | 1.32548 |
| P6_D1_DEFERRED_ONLY | 10911 | 10686 | 1720 | 4822278513 | -6.42839e+08 | 0.986881 | 250 | 223 | 49 | 0 | 31.8291 |
| P6_D2_DEFERRED_COVERAGE | 10752 | 6799 | 2082 | 4827519926 | -6.48081e+08 | 0.987954 | 204 | 215 | 53 | 0 | 16.2372 |
| P6_D3_DEFERRED_COVERAGE_DIRECTION | 11102 | 8222 | 1860 | 4826470249 | -6.47031e+08 | 0.987739 | 204 | 211 | 53 | 0 | 16.7167 |

Negative saved bytes mean MORE traffic than P5. More utilization/attempts/coverage does not establish better maps; factor counts can decrease. Direction coverage, exact caps, reconstructed cache/rolling usage and rank dependencies are in `scheduler_causal_validation.json`. Runtime includes scheduler work, frozen receipt I/O and receipt-hash validation, not new GICP or graph optimization. Peak RSS is process high-water mark and therefore cumulative across sequential variants, not isolated per-policy memory.

No new P6 pose graph or GT evaluation was run. Admitted factors are candidate inputs for a later independently reconstructed graph, not a completed mapping comparison or proof of accuracy. Complete Recall/ATE and P6 independent accuracy remain NOT_RUN. The experiment does not establish that smarter scheduling uses limited resources more effectively for mapping; the causal implementation is working and the mixed/negative resource/admission observations are retained.

## Validation, browser evidence and reproduction
The saved-artifact auditor independently reconstructs received cloud identities, canonical pair admissions, cumulative/rolling budgets, source receipt hashes and early-stop rank dependencies. Synthetic tests cover future access, unknown outcomes, expiration, byte caps, rank-success stopping and equivalence of cached versus original candidate-context scanning. A performance repair caches unchanged contexts across budget denials and invalidates after each executed outcome; interrupted CPU time and unchanged completed D0 results are retained in `engineering_recovery.json`. It changes no priority, scientific threshold or budget.

The browser test uses local Chromium with software SwiftShader, not a hardware-accelerated foreground rehearsal or a SLAM benchmark. It checks file loading, identities, synchronized cameras, overlays, all11regions, Batch switching, cached buffers, toggles and disposal. Screenshots preserve incremental, regional-overlay and Batch gauge-XY views. Startup and test timing are recorded, without claiming60FPS, network throughput or live operation.

Browser report:

```json
{
  "status": "PASS",
  "checks": {
    "actual_file_loading": true,
    "side_by_side_identity": true,
    "camera_rotation_synchronization": true,
    "camera_zoom_synchronization": true,
    "camera_pan_synchronization": true,
    "overlay_toggle": true,
    "loop_identity_toggle": true,
    "seven_temporal_four_route_regions": true,
    "batch_real_trajectory_toggle": true,
    "buffers_cached_across_switches": true,
    "map_visibility_toggle": true,
    "explicit_final_only_not_live": true,
    "resource_cleanup": true,
    "no_browser_errors": true
  },
  "errors": [],
  "failed_requests": [],
  "browser_version": "147.0.7727.15",
  "url": "http://127.0.0.1:8024/stage23d_policy_map_compare/",
  "headless": true,
  "renderer": "software ANGLE SwiftShader; not foreground/GPU smoothness validation",
  "presentation_metrics": {
    "loadedAssets": 8,
    "bufferBuilds": 8,
    "frames": 183,
    "errors": [],
    "disposed": false,
    "startup_ms": 483.09999999403954
  },
  "total_browser_test_s": 6.675935000006575,
  "scientific_runtime_measured": false,
  "screenshots": {
    "batch_gauge_xy.png": {
      "bytes": 281239,
      "sha256": "903e2156b6d14f9f543ae3b422f2f22ca4a60bd97f731ba0d2daba9fb70ae804"
    },
    "final_incremental.png": {
      "bytes": 260336,
      "sha256": "ce558a6ae0ae55559034a355aeb0f5bcb483b61c179df425d07d4c7f5a809e49"
    },
    "regional_overlay.png": {
      "bytes": 546100,
      "sha256": "6dd11732ece0e6f7d49d3cd98082437c772b3f99cc12c835dfdb8e77bfe0f3dd"
    }
  }
}
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
