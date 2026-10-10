# Experiment log

## Latest entry — 28. Stage 13D (2026-10-09)

Objective: isolate targeted iSAM2 convergence behavior and audit suspicious
Stage13C counters without changing historical results. Independent source:
`src/cumulti/run_stage13d_solver_diagnostics.py`; shell launcher and independent
saved-artifact auditor accompany it. Fixed D0(default), D1(default+five empty
calls at six checkpoints), D2(skip1/threshold0.01+same five calls), wildfire0.001.
Before/after graph states use separate identical-graph references. Six saved
Stage13B references reused; five missing before-event references use frozen12D
LM once each, for evaluation only and never ISAM2 initialization.

Outputs: `outputs/cumulti_v1/13d_solver_diagnostics/`, raw/clean counter audit,
tiny Pose3 reproduction, microtraces, nodewise/per-robot/worst20 errors, inserted
loop residuals, timings/overheads, exact factor identity, immutable decision,
eight figures, source-evidence provenance and independent audit PASS.

Official4.2 result/update code explains a no-work path with an uninitialized
re-elimination field, consistent with tiny/real invalid values. Preserve raw
returns and label invalid/null; do not infer counts from positive raw values.
Marked-set relinearization bookkeeping is not a count of changed coordinates.
D0 replicates the original six checkpoint pose statistics/objectives. D1 K2
calls102–106 do not trigger skip10 and do nothing. D2 K2 extra2 reaches all
four original numerical gates (p95 from12.286618 to0.106397 m); extra5 gives
0.055109 m. D2 K40 also reaches agreement, but K75/final still fail. Decision:
**NUMERICAL_DISCREPANCY_PARTIALLY_EXPLAINED**. All final factors unchanged;
no new catastrophic jumps or GT/scan/map/frontend/demo operation. No Hessian
nullspace claim or production readiness. Stop here; any further study needs a
separately declared protocol. See `docs/CUMULTI_STAGE13D_SOLVER_DIAGNOSTICS.md`.

## Pre-13D history (unchanged)

Reconstructed from source names, output directories and README_zh.md. Historical KITTI and CU-Multi work are deliberately separate.

## Latest entry — 27. Stage 13C (2026-10-09)

Objective: diagnose Stage13B iSAM2/batch disagreement without GT or changing
the frozen graph. Independent source `src/cumulti/run_stage13c_isam2_convergence.py`
and audit `src/cumulti/audit_stage13c_isam2_convergence.py` replay the same
Robot3-reference/Robot1-stream scheduler in five fresh persistent instances.
Inputs are hash-checked frozen odometry, 75 sanitized loops and eight saved
Stage13B batch references; no new LM solve. Variants: skip10/threshold0.1,
skip1/0.1, skip1/0.01, skip1/0.001, and default plus exactly three diagnostic
empty updates after each loop. Wildfire0.001, noise/Huber/anchor stay frozen.

Outputs: `outputs/cumulti_v1/13c_isam2_convergence/`, per-event counters/timings,
40 checkpoint comparisons, per-robot/worst20 localization, objective gaps,
extra-update effects, graph/schema/provenance audits, frozen decision and nine
figures. Large per-loop estimate archives remain server-only with SHA manifests.
V0 reproduces all eight original checkpoint pose statistics/objectives within
1e-7. All variants finish with exactly 5,796 nodes and 5,870 factors, finite
poses and no new catastrophic step. Independent execution audit PASS.

Numerical gates: V0 passes 3/8 checkpoints; V1–V4 pass 4/8 each. Skip1 repairs
K10 p95 (2.456849→0.135119 m) but leaves K2 at 12.286618 m. V4's 225 empty
calls change poses materially in 25 calls and reduce K2 p95 to 1.679341 m,
still above 0.50 m. Final translation p95 remains 0.563985/0.566468/0.574742/
0.571735/0.564448 m. Decision: **NO_STANDARD_CONFIG_MET_GATES**; no selected
standard configuration, V4 ineligible. Lower thresholds increase update cost
without monotonic final agreement. All backend-only next-interval checks pass,
but no map publication/frontend/network or end-to-end real-time test occurred.
No GT decoding/evaluation, new batch optimization, maps, demo changes or
parameter-grid expansion. See `docs/CUMULTI_STAGE13C_ISAM2_CONVERGENCE.md`.

## Historical entries 1–26

1. Formal Scan Context baseline: `scan_context.py` and `retrieval_baseline*.py`; `outputs/baseline_*` and `formal_split*`; establishes the geometry anchor.
2. Candidate recall/case analysis: `analyze_sc_candidate_recall.py`, `analyze_retrieval_cases.py` and visualization scripts; documents candidate ceilings and misses.
3. BEV/yaw validation: `generate_bev.py`, `validate_sc_yaw.py` and `full_bev_reranking.py`; BEV-only reranking regressed against Scan Context.
4. RGB and temporal mean pooling: `full_rgb_reranking.py` and `full_temporal_rgb_reranking.py`; mean pooling was worse than single-frame RGB.
5. Temporal Cross-Max/filtering: `full_temporal_cross_frame_reranking.py` and `top20_crossmax_visual_filtering.py`; Cross-Max outperformed mean pooling and enabled selective filtering.
6. Canonical-v2/VLM: `src/canonical_v2` and selective-VLM scripts; VLM override caused net regressions.
7. CU-Multi acquisition: downloaded Main Campus robot1/robot2 raw archives and calibration to `/home/cas/CU-Multi/raw/`, kept outside Git and unchanged.
8. CU-Multi Stage 1 adapter validation: `src/cumulti/prepare_stage1_validation.py`; input was the ROS2 SQLite bags and UTM GT CSV. It temporarily expanded one `.db3` at a time, produced exactly 100 float32 XYZI clouds and 100 nearest RGB PNGs per robot, and wrote timestamps, poses and sync indices to `/home/cas/CU-Multi/processed_v1/`. Mean absolute RGB/GT synchronization errors were 24.884/10.928 ms (robot1) and 25.532/10.831 ms (robot2). Artifacts and caveats: `docs/CUMULTI_STAGE1_VALIDATION.md`. No retrieval algorithm was run and GT was used only offline.
9. CU-Multi Stage 2 frozen Scan Context baseline: fixed 2 Hz LiDAR grids (robot1 2,000; robot2 2,230), offline `d_xy < 5 m` labels, and Robot1→Robot2 R@1 0.994756 over 1,907 valid queries. The baseline remains frozen; no visual model or GT-based candidate selection was used.
10. CU-Multi Stage 2.5 diagnostic: `src/cumulti/run_stage25_diagnostics.py`; read existing rankings only to diagnose the high baseline. Common-start stationarity contributes but does not explain the near-saturation; moving-only R@1 remains 0.993835. All ten Rank-1 failures were analysed by offline GT distance/yaw and visual panels. See `docs/CUMULTI_STAGE2_5_DIAGNOSTICS.md`.
11. CU-Multi Stage 3 pair selection: `src/cumulti/run_stage3_pair_selection.py`; downloaded only the two robot3/4 UTM GT CSVs and evaluated all 12 directions across six unordered pairs using timestamp-based 2 Hz samples and offline `d_xy < 5 m` labels. Recommended robot1–robot3 as hard-but-usable; no robot3/4 sensor archive or retrieval algorithm was used. See `docs/CUMULTI_STAGE3_PAIR_SELECTION.md`.
12. CU-Multi Stage 4 Robot1-to-Robot3 LiDAR-only baseline: `src/cumulti/run_stage4_r1_r3_sc_baseline.py`; read Robot3 LiDAR/RGB ROS2 archives as temporary SQLite expansions, sampled only actual LiDAR timestamps at 2 Hz (4,180 keyframes), reused the exact frozen Robot1 Stage 2 cache (2,000 queries), and ranked the complete Robot3 database with unchanged Scan Context. RGB was synchronized but never retrieved; GT was applied only after ranking for `<5 m` labels, headings and evaluation. Primary R@1/R@5/R@10/R@20 was 0.993453/0.996181/0.997272/0.998363 across 1,833 valid queries; reverse Robot3-to-Robot1 R@1 was 0.998814 across 1,686 valid queries. See `docs/CUMULTI_STAGE4_R1_R3_SC_BASELINE.md`.
13. CU-Multi Stage 5 visual viewpoint analysis (corrected): `src/cumulti/run_stage5_visual_viewpoint.py`; recovered historical OpenCLIP `ViT-B-32-quickgelu` / `laion400m_e32` checkpoint, encoded RGB for all frozen Robot1 (2,000) and Robot3 (4,180) 2 Hz keyframes, reconstructed frozen SC Top-20 candidates, and reranked with Single RGB, RGB5 Mean and Cross-Max. Candidate-conditioned R@1: SC=0.995, Single=0.703, Mean5=0.684, CrossMax=0.716. End-to-end R@1: SC=0.993, Single=0.702, Mean5=0.682, CrossMax=0.715. Corrected rescues: Single=4, Mean5=2, CrossMax=1 (candidate-generation failures excluded). Regressions: Single=538, Mean5=572, CrossMax=512. Bug fixes: rank -1 never counted as success; end-to-end <= candidate-conditioned asserted; sync-clean checks both robots. See `docs/CUMULTI_STAGE5_VISUAL_VIEWPOINT_ANALYSIS.md`.
14. CU-Multi Stage 6 selective visual verification: `src/cumulti/run_stage6_selective_visual_verification.py`; reused frozen Stage-5 embeddings without re-encoding, deterministically reconstructed and Rank-1 verified frozen SC Top-20 scores, calibrated best-shift only offline, and swept interpretable GT-free confidence/viewpoint gates. The Single RGB q5%/q50%/delta 0.05 point rescues one query with zero regressions at 4.31% visual invocation; the required fixed viewpoint ablation shows no added gain. See `docs/CUMULTI_STAGE6_SELECTIVE_VISUAL_VERIFICATION.md`.
15. CU-Multi Stage 7 held-out generalization validation: `src/cumulti/run_stage7_generalization_validation.py`; froze Stage-6 decisions, built and audited a one-time Robot2 OpenCLIP cache, reconstructed whole-database Robot2-to-Robot3 SC rankings, and applied the absolute Stage-6 Single-RGB gate with no GT policy feature. SC is R@1 0.997263 on 2,192 valid queries; strict transfer is neutral (115 invocations, 14 overrides, 0 rescues, 0 regressions). GT-only heading analysis and best-shift calibration are separated, as is Robot3-to-Robot2 SC sanity. Cross-Max was skipped because no exact predeclared Robot2 temporal transfer configuration existed. See `docs/CUMULTI_STAGE7_GENERALIZATION_VALIDATION.md`.
16. CU-Multi Stage 7.5 supervisor validation: `src/cumulti/run_stage75_supervisor_validation.py`; reused the frozen Robot1-to-Robot3 SC Top-20 protocol and cached Stage-5 embeddings without re-encoding, stratified only after ranking by predeclared GT heading/distance, and audited 20 uniformly distributed processed XYZI frames per Robot1/2/3. Same-view <=10 degrees contains 482 queries: SC R@1=1.0, Single=0.906639, Mean5=0.883817, Cross-Max=0.890041. All 60 audited frames pass the declared 34/36-bin, <=20-degree-gap full-azimuth condition, including 0--20/40/80 m slices. See `docs/CUMULTI_STAGE7_5_SUPERVISOR_VALIDATION.md`.
17. CU-Multi Stage 8 efficient hierarchical retrieval: `src/cumulti/run_stage8_efficient_sc_retrieval.py`; read frozen SC caches, built canonical 20-D Ring Keys and an exact Euclidean cKDTree, asserted exact shared-candidate score/shift agreement, and timed exhaustive/fixed-M retrieval in memory. Predeclared selection chooses M1000 (6.12x primary speedup) and transfers without metric loss to Robot2-to-Robot3 (5.34x). Adaptive progressive q50--q95 margin12 sweep has no valid rule under the predeclared M<=500 and <=0.1pp constraint; it was not transferred. See `docs/CUMULTI_STAGE8_EFFICIENT_SC_RETRIEVAL.md`.
18. CU-Multi Stage 9 fast SC--GICP integration: `src/cumulti/run_stage9_frame_audit.py`, `src/cumulti/run_stage9_sc_gicp_integration.py`, and `src/cumulti/run_stage9_integrated_latency.py`; first resolved the Robot1/Robot3 `/tf` LiDAR-frame convention, then reused frozen Stage-8 M1000 candidates and the teammate `robot13_gicp.py` backend (0.75 m voxel, 3/1 m correspondence, 30 iterations, threshold 0.6091). GT is attached only after registration for offline labels/error analysis. SC yaw initialization gives 61.65% acceptance versus 21.60% identity across 2,000 rank-1 pairs. Rank-ordered Top-3 early stop has TP/FP/FN/TN=1374/29/459/138 and exports 139 temporal-cluster representatives. Actual sequential 100-query integration-harness latency is 387.85 ms mean Rank-1 and 672.20 ms Top-3. PGO was not run. See `docs/CUMULTI_STAGE9_SC_GICP_INTEGRATION.md`.
19. CU-Multi Stage 10.1 pose-graph correctness replay: `src/cumulti/run_stage10_offline_map_merge.py`; replayed the fixed Stage-10 graph in a new output directory after correcting the initialization composition, using the official URDF IMU-to-LiDAR transform, and adding strict initialization/frame/objective checks. It keeps the same 139 Stage-9.1 loops, noise, Huber rule, outer iterations, solver cap, and map sampling. Both solvers reach the frozen cap; output is therefore correctness-pass but mixed/non-converged, not a live-system result. See `docs/CUMULTI_STAGE10_1_POSE_GRAPH_CORRECTNESS.md`.
20. CU-Multi Stage 10.2 PGO convergence and loop-policy selection: `src/cumulti/run_stage102_pgo_policy_selection.py`; compared immutable 139-edge Top-3 sanitation with a GT-free, identically sanitized 120-edge Rank-1 policy from 1,233 frozen registrations. Robust PGO uses only the fixed 25/50/100/200 budget schedule. Neither policy converged by 200, so the pre-GT decision is NO_POLICY_READY; offline GT evaluation and non-robust diagnostics are supporting analysis only. See `docs/CUMULTI_STAGE10_2_PGO_POLICY_SELECTION.md`.
21. CU-Multi Stage 12A trajectory integrity audit: `src/cumulti/run_stage12a_trajectory_integrity.py`; compared exact frozen EKF local odometry, single-loop common-frame trajectory, Rank1 GTSAM trajectory, and all three demo trajectory representations without GT. Robot1/Robot3 maximum local adjacent XY steps are 291.454/333.575 m at normal ~0.5 s intervals; the same anomalies occur in raw `robot{1,3}/ekf/odometry_map` messages and propagate through pre/post-PGO. No new catastrophic top-20 post-PGO translation arises from a previously unflagged pre-PGO transition. Browser stride-10 chords accentuate the shape of the upstream jump. Outputs are isolated under `outputs/cumulti_v1/12a_trajectory_integrity/`; see `docs/CUMULTI_STAGE12A_TRAJECTORY_INTEGRITY.md`. No historical data or demo behavior changed.
22. CU-Multi Stage 12B upstream EKF forensic audit: `src/cumulti/run_stage12b_ekf_forensics.py`; used frozen Stage 12A top-ten local steps per Robot1/3 to inspect raw ROS2 IMU/GNSS archives without GT. Primary raw EKF map jumps are 298.101/344.384 m XYZ within ~0.5 s, mirrored in EKF earth odometry, while independent dual-antenna GNSS stays nearly stationary. Large position covariance, stable frame IDs, and multi-second excursion/recovery suggest early estimator initialization/convergence instability, but no TF topics or decodable recording-time vendor status prove an internal reset. Conservative classification is `EKF_STATE_RESET_OR_REINITIALIZATION` at MEDIUM confidence, with `MORE_SOURCE_INVESTIGATION_REQUIRED` and sparse-loop-on-current-odometry `NO`. See `docs/CUMULTI_STAGE12B_EKF_FORENSICS.md`; no repair or PGO rerun was performed.
23. CU-Multi Stage 12C dataset/stability audit: `src/cumulti/run_stage12c_dataset_stability.py`; compared the official CU-Multi layout with local raw/processed files, verified that R1/R3 `/tf` lives in separate GT relative-pose bags rather than IMU/GPS, and validated current-pipeline ZIP/SQLite inputs. A GT-free, predeclared 10-s look-back plus 30-s persistence rule declares conservative starts at Robot1 KF150 (~75.0 s) and Robot3 KF234 (~117.0 s), retaining 1,850/3,946 keyframes and 75 frozen Rank-1 sanitized loops. No later large pose-step recurrence occurs, but one R1 and three R3 covariance-only samples exceed the frozen major threshold, so delayed-start backend reconstruction remains insufficient under this rule. GT UTM displacement at the original jumps was checked offline only after selection. No graph, retrieval, demo, or odometry repair was run. See `docs/CUMULTI_STAGE12C_DATASET_STABILITY.md`.
24. CU-Multi Stage 12D clean backend: `src/cumulti/run_stage12d_clean_backend.py`; reused frozen R1 KF150/R3 KF234 starts and all 75 retained Rank1 sanitized loops with unchanged Stage10.3 extrinsic/noise/Huber settings. The 5,796-node / 5,794-odometry-factor graph solves in 0.351 s (7 iterations), reducing graph error 3843.986→23.840 with no new catastrophic step (post-PGO max XY 1.141/1.411 m). First-arrival and highest-quality rules both select R1 KF283↔R3 KF252, so identical final solutions give a sensitivity WARN rather than evidence about distinct seeds. GT-free map NN median/p95 is 3.031/119.278 m versus first-loop 10.865/144.833 m; pre-GT decision is CLEAN_BACKEND_READY. Offline-only joint ATE worsens from 16.966 to 19.608 m, and no solver rerun follows GT. Stage13 K=1,2,5,10,20,40,75 is frozen but not executed; no iSAM2 or demo change. See `docs/CUMULTI_STAGE12D_CLEAN_BACKEND.md`.
25. CU-Multi Stage 13A sparse-loop batch evaluation: `src/cumulti/run_stage13a_sparse_loops.py`; reused the actual Stage12D factor template and frozen first-loop x0 for independent K=1,2,5,10,20,40,75 graphs (5,796 nodes, 5,794 odometry factors, one prior). First loop connects two chains (2→1), K1/K75 reproduce Stage12D, and all 28 warmup/measured solves precede the persisted GT-free decision. Frozen 10% median/p95 rule finds **NO_SPARSE_SAVING**; global NN median is 10.865/8.103/4.288/4.229/4.100/6.189/3.031 m, explicitly non-monotonic. Mean batch solve latency is 0.115–0.393 s (three measured samples per K); fixed 20m loop-endpoint ROI is an offline GT-free diagnostic, not a GT overlap mask. Offline-only joint ATE is 16.966/21.182/17.771/22.383/21.900/27.598/19.608 m, also non-monotonic. Nine figures, source-attributed server-only maps with SHA256 manifests, and independent provenance/schema/RPE-union audit pass. Readiness is READY_FOR_INCREMENTAL_METHOD_STUDY, requiring a separately declared causal scheduler; no iSAM2, frontend rerun, frozen-result change or demo modification. See `docs/CUMULTI_STAGE13A_SPARSE_LOOPS.md`.
26. CU-Multi Stage 13B incremental iSAM2 study: `src/cumulti/run_stage13b_incremental_isam2.py`; prebuilt Robot3 KF234–4179 reference plus original-order Robot1 KF150–1999, inserting all 75 precomputed frozen loops at query arrival. First bridge R1#283↔R3#252 connects 4080 nodes with one prior (2→1), then 1716 subsequent events reuse the same ISAM2 instance. Defaults frozen before execution (GaussNewton, threshold0.1, skip10, wildfire0.001); initial KeySet-binding validation error was archived and fixed to keyVector before GT, without parameter changes. Final factors equal Stage13A K75 exactly (5796 nodes/5794 odometry/75 loops/1 prior). First update is 26.787 ms, first fusion including map publication 2459.273 ms; odometry-only/loop update means are 1.827/2.833 ms, p95 14.950/20.222 ms. All poses finite, no new catastrophic steps, but only FIRST_LOOP/K5/K20 meet fixed batch-agreement limits; final translation median/p95 difference is 0.124431/0.563985 m and rotation 0.055319/0.301537°. Pre-GT decision: **INCREMENTAL_BACKEND_NOT_READY**; method validation **FAIL_NUMERICAL_AGREEMENT**, independent provenance/execution audit PASS. Final NN median/p95 3.031268/118.993911 m, offline-only joint/R1/R3 ATE 19.631962/12.122472/22.298223 m, combined10-KF RPE 10.540963 m/7.521802°. Nine figures and archived-event demo assets prepared; no historical demo change, causal online frontend, simultaneous robots, retuning or solve after GT. See `docs/CUMULTI_STAGE13B_INCREMENTAL_ISAM2.md`.


## Entry 29 — Stage 14 (2026-10-10)

Objective: distinguish final incremental state/rebuild sensitivity, initialization
sensitivity and descriptive structural/conditioning evidence without GT. New
`run_stage14_final_consistency.py`, independent auditor and launcher use exact
frozen75loop/odom/prior factors at K2/K75/FINAL. A1canonical/A2D0/A3D2 fresh
instances keep skip1/threshold0.01/wildfire0.001 and five empty calls; six LM
probes use frozen12D defaults. Original gates, gauge and inputs are fixed.

Outputs: `outputs/cumulti_v1/14_final_consistency/`;54 fresh states, historical
comparisons, six LM probes, nodewise physical/Logmap differences, PCA/smoothness,
fixed50/100/250KF loop-support proxies,18 bounded6x6 marginals, separate costs,
frozen decision, nine figures and independent audit PASS. Historical120-to75
provenance and excluded45 IDs verified. A prefix-schema preflight and later
reporting failure are archived; mandatory timing recovery uses exactly the
original policy and reproduces all66 state files with maxcoordinate delta0.
No parameter search or best-time selection.

K2 fresh graphs pass; K75/final fresh p95 remains0.56–0.57m, all FAIL. LM from
canonical x0 reproduces reference, but persistent x0 probes keep different
coordinates at near-equal objectives. Robot3 error is smooth/distributed;
structural support associations and finite local marginal information do not
establish one causal mechanism or a Hessian nullspace. Frozen decision:
**RESIDUAL_NUMERICAL_DISCREPANCY_UNRESOLVED**. No GT, scans/maps, frontend or
demo changes; stop pending a separately declared methodology protocol.
