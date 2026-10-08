# Stage13B — Incremental multi-robot map fusion with ISAM2

## 1. Motivation

Test first-loop connectivity and repeated genuine incremental optimization against identical-graph batch LM. Accepted checkpoint `636d255d18202d2dae7aa4f2ea23827cbf08701c`; independent new outputs only.

Current method status: **FAIL_NUMERICAL_AGREEMENT**. Execution/provenance/integrity succeed, but readiness is **INCREMENTAL_BACKEND_NOT_READY**; do not conflate successful execution with numerical readiness.

## 2. Stage13A limitations

Stage13A full-graph loop budgets include future Robot1 poses and are not causal stream checkpoints. It found NO_SPARSE_SAVING and non-monotonic map/GT outcomes. Nothing here rewrites those results.

## 3. Robot3 reference-map protocol

Robot3 KF234–4179 is a previously recorded complete 3946-pose local reference map, not a simultaneously running robot.

## 4. Robot1 streaming scheduler

Robot1 KF150–1999 arrives in original timestamp/keyframe order (nominal 2Hz), fast offline replay without sleep. Before bridge, local relative odometry is composed; after bridge initial guess = estimated previous pose × new odometry. No future Robot1 node, factor or scan is inserted.

## 5. Frozen loop arrival semantics

PRECOMPUTED FROZEN LOOP FACTORS: all75 frozen query-order factors inserted exactly once at query arrival. Candidate can come from any stored Robot3 frame. Offline sanitation may use later information; this tests causal insertion, NOT causal online discovery by SC/GICP.

## 6. Independent local frames

No joint unanchored solver before bridge, no fused map/NN before bridge, no pair of permanent hard priors. Pre-loop snapshot preserves two independent coordinate systems.

## 7. First-loop common-frame initialization

R1#283 ↔ R3#252, exact Stage12D LiDAR/os_sensor gauge and URDF extrinsic. `S=Xq Zqc inverse(Lc)`; numerical-zero first-loop residual recorded. One correct inter-robot loop establishes initial common-frame connectivity; it does not guarantee globally correct reconstruction.

## 8. First ISAM2 graph construction

4080 nodes (134 arrived R1 +3946 stored R3),4078 odometry,1 loop,1 prior on R1 KF150. Actual union-find verifies 2→1. ISAM2 initialized only on this connected graph. Factor objects from Stage12D construction reused exactly.

## 9. Sequential odometry updates

Same live ISAM2 instance thereafter. New Values contains exactly one arrived R1 pose; newFactors contains only its new odometry plus contemporaneous archived loops. Registries assert no repeats/future keys. Every estimate checked finite and every physical adjacent translation checked against frozen Stage12D catastrophe rule.

## 10. Inter-robot loop updates

Installed default GaussNewton ISAM2, one update per event, no hidden LM solve used to generate incremental poses, no extra empty updates. Relinearize skip10, threshold0.1, wildfire0.001, CHOLESKY, cache enabled; actual binding print/API in policy/audit. Parameters never changed after comparison. Batch runs are separate diagnostics. An initial development attempt stopped at checkpoint key validation because the installed `KeySet` is not iterable; after inspecting the API, validation switched to `keyVector()`. That attempt is preserved under `_failed_attempts/api_keyset/`, with identical parameters and no GT access; it is not pooled into primary latency measurements.

## 11. First-fusion cost

| first_graph_construction_ms | factor_template_preparation_ms | first_isam2_update_ms | first_estimate_extraction_ms | first_global_map_publication_ms | first_fusion_total_ms | one_sample_only | excludes_batch_reference_and_diagnostics |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 46.763276 | 61.797343 | 26.786819 | 3.813258 | 2374.703719 | 2459.272580 | True | True |

First-fusion graph construction/update/extraction/map publication measured separately, one sample. Prebuilding immutable factor template separately reported. Reference-map loading/publication and thousands of initial poses are not steady-state update cost.

## 12. Steady-state update latency

| event_type | count | mean | median | p90 | p95 | p99 | max | update_missed_deadlines | total_processing_missed_deadlines |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| LOCAL_ODOMETRY_ONLY | 133 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0 | 1 |
| FIRST_INTER_ROBOT_FUSION | 1 | 26.786819 | 26.786819 | 26.786819 | 26.786819 | 26.786819 | 26.786819 | 0 | 1 |
| ODOMETRY_UPDATE | 1642 | 1.827058 | 0.048966 | 0.154217 | 14.949600 | 38.182569 | 51.080648 | 0 | 0 |
| INTER_ROBOT_LOOP_UPDATE | 74 | 2.832547 | 0.484134 | 3.953031 | 20.222451 | 34.387762 | 35.027600 | 0 | 6 |

Actual next-keyframe intervals used for deadlines. Timing excludes frontend, network, batch diagnostics and integrity/full-matrix auditing; snapshot map publication is separately included in event processing and can miss deadlines. Small loop/first-fusion sample counts limit tails. No whole-system real-time claim. `total_graph_nodes` means instantiated joint-global graph nodes (zero before bridge), while both independent local maps have available pose counts separately recorded. Robot3 reference loading and the immutable full factor-template preparation are startup work, not hidden steady-state updates. Publication includes scan loading, transforms, voxelization, PLY writing and hashing; NN evaluation is separate.

## 13. Incremental vs batch comparison

Checkpoints FIRST_LOOP/K2/K5/K10/K20/K40/K75/FINAL_STREAM_END use identical arrived nodes/odom/loops/prior/noise, not full Stage13A K1. Batch x0 is the frozen first-loop frame restricted to available nodes; same Stage12D LM defaults. Same R1 KF150 gauge, NO alignment. Fixed limits 0.10/0.50m translation median/p95;0.5/2° orientation median/p95.

| checkpoint | nodes | K | translation_median_m | translation_p95_m | rotation_median_deg | rotation_p95_deg | graph_error_difference | agreement_status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| FIRST_LOOP | 4080 | 1 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | -0.000000 | INCREMENTAL_BATCH_AGREEMENT_PASS |
| K2 | 4175 | 2 | 3.382671 | 12.286618 | 1.342069 | 2.216089 | 0.689949 | ISAM2_BATCH_DIVERGENCE |
| K5 | 4309 | 5 | 0.039307 | 0.116133 | 0.023190 | 0.025991 | 0.000002 | INCREMENTAL_BATCH_AGREEMENT_PASS |
| K10 | 4435 | 10 | 1.005827 | 2.456849 | 0.601675 | 2.549446 | 4.574689 | ISAM2_BATCH_DIVERGENCE |
| K20 | 4461 | 20 | 0.074729 | 0.374318 | 0.035544 | 0.156282 | 0.000113 | INCREMENTAL_BATCH_AGREEMENT_PASS |
| K40 | 4811 | 40 | 0.211279 | 0.611506 | 0.160136 | 0.319450 | 0.038249 | ISAM2_BATCH_DIVERGENCE |
| K75 | 5764 | 75 | 0.123186 | 0.744521 | 0.054755 | 0.745961 | 1.220269 | ISAM2_BATCH_DIVERGENCE |
| FINAL_STREAM_END | 5796 | 75 | 0.124431 | 0.563985 | 0.055319 | 0.301537 | 0.000034 | ISAM2_BATCH_DIVERGENCE |

Diagnosis: `ISAM2_BATCH_DIVERGENCE`. Factors/anchor/noise/order provenance is independently recorded. One nonlinear ISAM2 update with skip10/wildfire and robust-factor linearization differs from converged LM and its initialization path; these are plausible mechanisms, not experimentally proven causes. No loosened thresholds or retuning/reruns.

Independent follow-up file `incremental_anchor_relinearization_audit.csv` verifies the same fixed anchor and records exposed relinearized/reeliminated counts at checkpoint arrival. `numerical_discrepancy_forensics.json` separates verified structural/gauge facts from unproven numerical mechanisms. Near-equal final objective does not guarantee the frozen pose-difference thresholds pass.

## 14. Final graph equivalence

5796 nodes,5794 odometry,75 loops,1 prior. All original template factors exactly once; GTSAM factor equality (1e-12) verifies measurements/noise despite insertion order. Final graph is structurally identical to Stage13A K75; final pose difference saved separately.

## 15. Map-consistency evolution

| checkpoint | robot1_keyframe | robot1_nodes | median_m | p95_m | points_robot1 | points_robot3 | metric_status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| PRE_FIRST_LOOP | 282 | 133 | N/A | N/A | 16928 | 655943 | NOT_APPLICABLE_SEPARATE_FRAMES |
| FIRST_LOOP | 283 | 134 | 106.740594 | 244.281603 | 16928 | 655410 | OK |
| K2 | 378 | 229 | 97.786531 | 241.046461 | 29781 | 660555 | OK |
| K5 | 512 | 363 | 75.697172 | 199.349421 | 52489 | 660883 | OK |
| K10 | 638 | 489 | 54.513985 | 174.025114 | 73402 | 665527 | OK |
| K20 | 664 | 515 | 53.593765 | 173.317510 | 78541 | 664160 | OK |
| K40 | 1014 | 865 | 30.543199 | 130.748740 | 130448 | 663075 | OK |
| K75 | 1967 | 1818 | 3.026428 | 119.044076 | 279249 | 664896 | OK |
| FINAL_STREAM_END | 1999 | 1850 | 3.031268 | 118.993911 | 280895 | 665059 | OK |

Deterministic Stage12D frame10/point16/voxel0.5/range1–60, only arrived R1 scans. Source-specific maps and manifests exported. Different checkpoint coverage prevents treating this as the Stage13A full-trajectory sweep. Identical-scan batch metrics isolate solver effects at each checkpoint; between checkpoints acquisition and new factors both change, so their separate causal effects are NOT identifiable from raw NN curves alone. No common-frame metric pre-bridge.

## 16. Pre-GT decision

`INCREMENTAL_BACKEND_NOT_READY`, persisted 2026-10-08T12:22:56.053740+00:00 after all incremental/batch runs and GT-free comparisons. Numerical readiness requires all fixed agreement gates; GT unused.

## 17. Offline GT diagnostic

One joint rigid alignment, per-robot/joint ATE and per-robot/combined10-KF RPE, only after saved decision. OFFLINE GT DIAGNOSTIC ONLY.

| metric_type | use | scope | ATE_RMSE_m | interval_keyframes | translation_RMSE_m | rotation_RMSE_deg | sample_count |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ATE_joint_rigid_alignment | OFFLINE GT DIAGNOSTIC ONLY | joint | 19.631962 | N/A | N/A | N/A | N/A |
| ATE_joint_rigid_alignment | OFFLINE GT DIAGNOSTIC ONLY | robot1 | 12.122472 | N/A | N/A | N/A | N/A |
| ATE_joint_rigid_alignment | OFFLINE GT DIAGNOSTIC ONLY | robot3 | 22.298223 | N/A | N/A | N/A | N/A |
| RPE | OFFLINE GT DIAGNOSTIC ONLY | robot1 | N/A | 10.000000 | 9.487451 | 5.996723 | 1840.000000 |
| RPE | OFFLINE GT DIAGNOSTIC ONLY | robot3 | N/A | 10.000000 | 10.998900 | 8.137287 | 3936.000000 |
| RPE | OFFLINE GT DIAGNOSTIC ONLY | combined | N/A | 10.000000 | 10.540963 | 7.521802 | 5776.000000 |

GT never changes readiness or parameters; no solve after GT.

## 18. Limitations

Reference Robot3 is stored; Robot1 alone streams; loops precomputed offline. No causal online SC/GICP, simultaneous two-robot acquisition, live input, decentralized optimization, network/transport/bandwidth/consensus benchmark. First-fusion cost != steady incremental update; batch latency != incremental latency. NN map agreement != GT trajectory accuracy. Single replay and default policy only; significant discrepancy cannot be silently tuned away.

## 19. Prepared demo assets

`demo_event_manifest.json` contains arrival/fusion/loop events, counts/connectivity/timings and snapshot paths; PLY source/frame/hash manifests are server-only. Historical `demo/final_replay/` untouched. No new demo started.

## 20. Next stage

Stop after Stage13B. If numerical disagreement persists, separately declare a GT-free relinearization/optimizer methodology audit before claiming backend readiness; do not change this frozen default run. Any Stage15 visualization is separately authorized. Do not implement distributed SLAM or select new loops.

Reproduce with `bash scripts/run_stage13b_incremental_isam2.sh` from root and an empty new-stage output directory; runner refuses overwrite. All historical stages immutable.
