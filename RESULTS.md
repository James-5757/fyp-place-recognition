# Results

## Stage 13D — targeted convergence and counter-validity audit (verified)

Independent D0/D1/D2 runners preserve the frozen graph, noise and gauge. D0
reproduces all six selected Stage13B checkpoints within 1e-7. All three final
graphs retain 5,796 nodes / 5,794 odometry / 75 loops / one prior; independent
artifact/provenance audit PASS. No GT, scans/maps, SC/GICP or demo changes.

The tiny Pose3 test finds 57 invalid re-elimination returns among 60 empty
calls; the real D1/D2 empty calls contain 37 invalid returns among 60. Raw
integers are retained, invalid diagnostic values are null/INVALID_COUNTER,
never zeroed or clipped. Official GTSAM4.2 code has an uninitialized
re-elimination counter on no-work paths, consistent with the reproduction;
exact wheel/source build identity remains uninstrumented. Historical13C's
positive raw returns do not prove actual re-elimination counts. This corrects
interpretation only, not its frozen numerical results.

At K2, D1's five empty calls leave translation p95 at 12.286618 m; its call
ordinals102–106 do not reach the skip10 boundary. D2 (skip1/threshold0.01)
changes p95 over steps0–5 to 12.286618/1.679833/0.106397/0.045502/0.055087/
0.055109 m; all four original gates first pass at extra2. The first extra
reduces objective0.768361→0.078936. K2 discrepancy principally belongs to
Robot3: its p95 changes12.339156→0.056441 m, versus Robot1
0.034756→0.000229 m. Further changes are not strictly monotonic.

D1 passes FIRST_LOOP/K40; D2 passes FIRST_LOOP/K2/K10/K40. Both fail K75 and
FINAL; D2 final p95 is0.574742 m despite near-equal objective. No diagnostic
schedule passes all six selected checkpoints. Persisted decision:
**NUMERICAL_DISCREPANCY_PARTIALLY_EXPLAINED**, not production readiness.
Five missing BEFORE_EVENT batch references were solved evaluation-only with
frozen12D LM parameters; six after-references were reused. Hessian conditioning
NOT_TESTED. See `docs/CUMULTI_STAGE13D_SOLVER_DIAGNOSTICS.md`.

## Earlier verified results (unchanged)

Verified values below are recorded in README_zh.md and existing named historical outputs.

| Experiment | Result |
|---|---|
| Scan Context | R@1 151/158 = 0.955696; MRR 0.963608 |
| Scan Context recall | R@5 154/158; R@10 156/158; R@20 158/158 |
| Visual-only | BEV R@1 0.917722; RGB 0.892405; RGB-3 mean 0.867089; RGB-5 mean 0.854430 |
| Temporal Cross-Max | R@1 0.930380; MRR 0.948312; 1 correction, 5 regressions |
| SC + Cross-Max filtering | 0.6*SC + 0.4*Cross-Max; Top-20 to Top-5: 157/158 retention, 3 rescues, 0 losses |
| SC + Cross-Max ranking | SC weight 0.7: R@1 153/158 = 0.968354; MRR 0.977321; 2 corrections, 0 regressions |
| VLM v2 override | 2 corrections, 3 regressions; R@1 150/158 = 0.949367 |

Initial Top-5 misses are 115, 380, 955 and 1565; their first-positive ranks are 15, 10, 12 and 7. Filtering rescues 115, 955 and 1565; 380 remains a miss.

Historical notes requiring direct output-file recovery: the requested SC + Cross-Max R@1 about 0.9620; canonical Top-20 about 156/158 (README instead records Top-10 156/158 and Top-20 158/158); and BEV yaw mean error about 1.67 degrees. RGB viewpoint invariance is not established conclusively.

## CU-Multi Stage 3 pair-selection result (verified GT-only analysis)

This is not a retrieval metric. Four-robot Main Campus trajectory analysis at 2 Hz with offline `d_xy < 5 m` positives recommends robot1–robot2 as an easy pair (weaker-direction valid queries: 1,907; mean directed overlap: 93.5%) and robot1–robot3 as the hard-but-usable pair (weaker direction: 1,686 valid queries; mean nearest-positive heading difference above 90°: 64.9%; above 150°: 60.9%). Robot1–robot4 is an optional asymmetric stress test (weaker direction: 1,432 valid queries; mean overlap: 57.3%). See `docs/CUMULTI_STAGE3_PAIR_SELECTION.md` and `outputs/cumulti_v1/03_pair_selection/`.

## CU-Multi Stage 4 Robot1-to-Robot3 Scan Context baseline (verified)

This is the first LiDAR-only retrieval result for the Stage 3 hard-but-usable pair. Robot1 uses the frozen Stage 2 2 Hz cache (2,000 queries); Robot3 uses 4,180 actual-LiDAR-timestamp 2 Hz keyframes. With the frozen Scan Context configuration and whole Robot3 database ranking, there are 1,833 valid-overlap queries and 167 no-overlap queries under offline `d_xy < 5 m` positives. Robot1-to-Robot3 Recall@1/@5/@10/@20 is **0.993453 / 0.996181 / 0.997272 / 0.998363**, MRR is **0.994682**, and the median/worst first-positive ranks are **1 / 467**. The reverse Robot3-to-Robot1 sanity direction has 1,686 valid queries and R@1 0.998814. GT is never used for descriptor construction, database filtering or ranking; RGB is synchronized only for audit. See `docs/CUMULTI_STAGE4_R1_R3_SC_BASELINE.md` and `outputs/cumulti_v1/04_robot1_robot3_sc/`.

## CU-Multi Stage 5 Visual Viewpoint Analysis (verified, corrected)

Stage 5 measures visual evidence reliability on the frozen Robot1-to-Robot3 SC Top-20 protocol. OpenCLIP ViT-B-32-quickgelu (laion400m_e32) encodes RGB for all frozen 2 Hz keyframes. Three visual methods rerank only the frozen SC Top-20 candidates.

Bug fixes applied: (1) rank -1 (no GT-positive in Top-20) is never counted as Recall success; (2) rescue/regression computed only within candidate-available queries, excluding candidate-generation failures; (3) end-to-end R@K <= candidate-conditioned R@K asserted; (4) sync-clean checks both Robot1 and Robot3 observations.

Candidate-conditioned (CANDIDATE_AVAILABLE queries only, 1,830 of 1,833 valid):
- Frozen SC: R@1=0.995082, R@5=0.997814, MRR=0.996281
- Single RGB: R@1=0.703279, R@5=0.940984, MRR=0.803501
- RGB5 Mean: R@1=0.683607, R@5=0.914208, MRR=0.777015
- Cross-Max: R@1=0.715847, R@5=0.902732, MRR=0.793528

End-to-end (ALL 1,833 valid queries, candidate-miss = unavoidable miss):
- Frozen SC: R@1=0.993453, R@5=0.996181
- Single RGB: R@1=0.702128, R@5=0.939444
- RGB5 Mean: R@1=0.682488, R@5=0.912711
- Cross-Max: R@1=0.714675, R@5=0.901255

Rescues (corrected): Single=4, Mean5=2, CrossMax=1
Regressions: Single=538, Mean5=572, CrossMax=512
Candidate-generation failures: 3 (zero visual rescues)

12 Stage-4 SC failures: 9 recoverable, 3 candidate-generation failures.

Sync-quality sensitivity: Single RGB 1833/1833 clean (identical); Mean5/CrossMax 1831/1833 clean (slight difference). Sync-clean checks both query AND candidate RGB observations.

See `docs/CUMULTI_STAGE5_VISUAL_VIEWPOINT_ANALYSIS.md` and `outputs/cumulti_v1/05_visual_viewpoint_analysis/`.

## CU-Multi Stage 6 selective visual verification (verified)

Stage 6 keeps Stages 4/5 frozen and uses cached Stage-5 OpenCLIP embeddings with GT-free confidence/viewpoint gates over the frozen SC Top-20. SC stays at **0.993453** R@1 (1,821/1,833); the Top-20 ceiling is **0.998363** (1,830/1,833), with 1847/1848/1849 impossible to rescue. The best-shift proxy is geometrically calibrated offline (MAE 8.604°, median 1.689°, p90 11.047°, Pearson/Spearman 0.923/0.886), but the fixed ablation shows no viewpoint-gating advantage beyond confidence.

Single RGB with SC-margin q5%, visual-margin q50% and compatibility delta 0.05 gives the only zero-regression rescue point: R@1/R@5 **0.993999/0.996727**, 4.31% valid-query invocation, 9 overrides, 1 rescue, 0 regressions, and 1.741 ms/query Model-A added compute. Cross-Max has no zero-regression rescue point. Global fusion at alpha_SC=0.50 is harmful (Single 1 rescue/7 regressions; Cross-Max 0/41). See `docs/CUMULTI_STAGE6_SELECTIVE_VISUAL_VERIFICATION.md` and `outputs/cumulti_v1/06_selective_visual_verification/`.

## CU-Multi Stage 7 held-out generalization validation (verified)

Stage 7 transfers the predeclared Stage-6 Single-RGB strict gate to independent
Robot2-to-Robot3, using frozen 2-Hz grids, unchanged Scan Context, and a one-time
Robot2 OpenCLIP cache. On 2,192 valid-overlap queries, SC achieves **R@1 0.997263,
R@5 1.000000, MRR 0.998223**. Strict transfer invokes visual verification on 115
queries (4.973%), makes 14 overrides, and has **0 rescues / 0 regressions**, leaving
R@1 unchanged. This is neutral held-out transfer, not evidence of a general visual
gain. Robot3-to-Robot2 SC-only sanity has R@1 0.996807 over 1,879 valid queries. See
`docs/CUMULTI_STAGE7_GENERALIZATION_VALIDATION.md` and
`outputs/cumulti_v1/07_generalization_validation/`.

## CU-Multi Stage 7.5 same-view and LiDAR azimuth control (verified)

On the predeclared Robot1-to-Robot3 same-view <=10-degree subset (482 valid
queries), frozen SC is **R@1/R@5 1.000000/1.000000**. Single RGB is
**0.906639/0.981328**, RGB5 Mean **0.883817/0.970954**, and Cross-Max
**0.890041/0.962656**. The <=10-degree plus distance <2 m control (437 queries)
remains SC/Single/Cross-Max R@1 **1.000000/0.913043/0.897025**. Actual XYZI
azimuth audits on 20 uniformly distributed frames per Robot1/2/3 all pass the
predeclared >=34/36-bin and <=20-degree-gap full-azimuth criterion; each has mean
36/36 occupied bins, zero largest observed gap, including 0--20/40/80 m slices.
This supports separate FoV, geometry-stability, and generic-OpenCLIP limitations
interpretations rather than a universal modality claim. See
`docs/CUMULTI_STAGE7_5_SUPERVISOR_VALIDATION.md`.

## CU-Multi Stage 8 efficient Scan Context retrieval (verified)

Stage 8A is a standard Ring-Key plus KD-tree baseline. Stage-8 exhaustive timing
reproduces the frozen Robot1-to-Robot3 SC metrics at **R@1/R@5/R@20
0.993453/0.996181/0.998363** with 106.473 ms mean latency. The predeclared rule
selects fixed **M=1000**: R@1 0.992908, CandidateRecall 0.999454, 17.404 ms mean,
and 6.12x speedup. Frozen M1000 transfers to Robot2-to-Robot3 with unchanged R@1
0.997263 and 5.34x speedup. Stage 8B finds no acceptable q50--q95 margin12 policy
under its fixed M<=500 and <=0.1pp loss constraint, so adaptive held-out evaluation
is correctly unavailable. See `docs/CUMULTI_STAGE8_EFFICIENT_SC_RETRIEVAL.md`.

## CU-Multi Stage 9 fast SC--GICP integration (verified)

Stage 9 keeps Stage-8 fixed M=1000 Scan Context and the teammate GICP backend
unchanged, after a direct Robot1/Robot3 `/tf` frame audit. On 2,000 Robot1
queries, Scan Context yaw initialization raises GICP acceptance from **21.60%**
(identity) to **61.65%**, while median registration latency drops from
**137.01 ms** to **21.60 ms**. Rank-ordered Top-3 early stop yields TP/FP/FN/TN
of **1374/29/459/138** under offline `d_xy < 5 m` analysis (precision
**97.93%**, recall **74.96%**) and produces 139 temporally sanitized candidate
loop constraints. Their registration transformations passed the frozen Stage-9
acceptance/sanitation pipeline but are not guaranteed-correct loop closures;
their robustness remains a Stage-10 pose-graph question. A 100-query sequential
harness measures
M=1000 retrieval plus actual teammate GICP at **387.85 ms** mean for Rank-1 and
**672.20 ms** mean for Top-3 early stop. Rank-1 is compatible with the mean
500-ms budget on the tested host, but its p95 is 538.40 ms; Top-3 does not meet
the mean sequential 2-Hz budget. These are host-specific pipeline measurements,
not a PGO result or a fully real-time claim. See
`docs/CUMULTI_STAGE9_SC_GICP_INTEGRATION.md`.

## CU-Multi Stage 10.1 pose-graph correctness replay (verified, mixed system result)

Stage 10.1 preserves the frozen Stage-10 policy and historical outputs, but corrects cross-robot initialization, applies the audited URDF IMU-to-LiDAR extrinsic, and records the objective before optimization. The initialization edge sanity check passes at 7.32e-15 m / 2.51e-15 degrees. At the frozen `max_nfev=25` cap neither solver converges: non-robust/robust objective reductions are 84.42%/99.67%. Joint ATE is 28.60 m (single loop), 20.66 m (non-robust), and 23.65 m (robust); robust map nearest-neighbour median/p95 is 2.920/127.071 m versus 9.556/142.640 m for single loop. Thus the correctness layer passes but the system result is mixed and is not a converged or live-ready PGO claim. See `docs/CUMULTI_STAGE10_1_POSE_GRAPH_CORRECTNESS.md`.

## CU-Multi Stage 10.2 PGO policy selection (verified, no policy ready)

The exact 139-edge Stage-9.1 Top-3 policy and a 120-edge Rank-1 policy from 1,233 frozen accepted Rank-1 GICP registrations were tested only at the predeclared 25/50/100/200 robust-PGO budgets. Neither formally converged by 200, so the GT-free demo decision is **NO_POLICY_READY**. Map NN median/p95 is 9.556/142.640 m (single), 2.962/115.283 m (Top-3), and 2.875/121.945 m (Rank-1). Offline-only joint ATE is 28.595, 27.609, and 29.029 m, respectively; it did not choose the policy. See `docs/CUMULTI_STAGE10_2_PGO_POLICY_SELECTION.md`.

## CU-Multi Stage 12A trajectory integrity audit (verified, GT-free)

The frozen non-GT local EKF trajectory has maximum adjacent XY steps of
**291.454 m** for Robot1 and **333.575 m** for Robot3 at normal approximately
0.5-second intervals. The same intervals are present in the raw EKF topic and
the pre-PGO common-frame trajectory; post-PGO maxima are 287.045 and 336.712 m.
None of the 20 largest post-PGO steps for either robot is a newly introduced
transition without a pre-PGO robust flag. Demo stride-10 lines make some of
these existing jumps look straighter, especially Robot3 keyframes 40–50 and
50–60. This is a source-trajectory integrity finding, not a new retrieval or
registration result. See `docs/CUMULTI_STAGE12A_TRAJECTORY_INTEGRITY.md`.

## CU-Multi Stage 13A sparse-loop batch evaluation (verified)

Using Stage12D's fixed stable segments, same first-loop initialization and actual
factor construction, all seven K=1/2/5/10/20/40/75 graphs pass finite-pose and
trajectory-integrity checks. The first loop establishes connectivity (2→1), not
proven global accuracy. K1 and K75 reproduce the frozen Stage12D references.

Global map NN medians are **10.865/8.103/4.288/4.229/4.100/6.189/3.031 m**;
p95 values are **144.833/129.738/135.557/115.805/117.668/115.251/119.278 m**.
The predeclared ≤1.10×K75 median AND p95 rule selects **NO_SPARSE_SAVING**.
No GT participates in this decision. Batch LM means are 0.115–0.393 s (one
warmup, three measured repetitions; not incremental/real-time latency).

Offline-only joint ATE is **16.966/21.182/17.771/22.383/21.900/27.598/19.608 m**.
Neither map agreement nor trajectory accuracy improves monotonically. Full
per-robot/combined RPE, fixed loop-region NN, common75 hindsight residuals and
timings are in `outputs/cumulti_v1/13a_sparse_loops/`. Stage13B readiness is
**READY_FOR_INCREMENTAL_METHOD_STUDY**, not authorization or evidence for live
fusion. See `docs/CUMULTI_STAGE13A_SPARSE_LOOPS.md`.

## CU-Multi Stage 13B genuine incremental iSAM2 (verified, not ready)

Robot3 is a prebuilt reference; Robot1 streams original-order keyframes. The
75 loop factors are precomputed archives, not causal online SC/GICP results.
At KF283 the connected graph has 4080 nodes/4078 odometry/1 loop/1 prior;
the final persistent ISAM2 graph exactly matches Stage13A K75 (5796/5794/75/1).

First ISAM2 update: **26.787 ms**; first fusion including map publication:
**2459.273 ms**. Odometry-only update mean/median/p95/max:
**1.827/0.049/14.950/51.081 ms**; loop-event:
**2.833/0.484/20.222/35.028 ms**. Solver updates meet measured next-keyframe
intervals, but first fusion and six loop-checkpoint map publications do not;
this is not end-to-end real-time performance.

Fixed default iSAM2 fails numerical batch agreement at K2/K10/K40/K75/final.
Final same-gauge translation median/p95 difference is **0.124431/0.563985 m**
(limits 0.10/0.50 m); rotation **0.055319/0.301537°**. Factor equality, finite
poses and integrity pass, but pre-GT readiness is **INCREMENTAL_BACKEND_NOT_READY**.
No threshold change or retuning was performed. Nearly equal final objectives
(23.840418 vs 23.840384) do not prove pose agreement in this graph.

Final source-specific map NN median/p95: **3.031268/118.993911 m**. Earlier
snapshot NN uses less Robot1 coverage and is not comparable with Stage13A's
full-trajectory sweep. Offline-only joint/R1/R3 ATE:
**19.631962/12.122472/22.298223 m**; combined RPE10:
**10.540963 m / 7.521802°**. No GT participates in readiness. See
`docs/CUMULTI_STAGE13B_INCREMENTAL_ISAM2.md`.

## Stage 13C — fixed-grid iSAM2 convergence ablation (verified)

Independent runner: `scripts/run_stage13c_isam2_convergence.sh`. V0 reproduces
all eight Stage13B checkpoints (pose statistics and nonlinear objective) within
the predeclared 1e-7 replication tolerance. All five variants preserve the final
5,796-node / 5,794-odometry / 75-loop / one-prior graph, pass finite-pose and
integrity checks, and use no GT, map generation, or new batch optimization.

The original four numerical gates must pass at every checkpoint. V0 passes
3/8; V1/V2/V3/V4 each pass 4/8. Final translation median/p95 differences from
the same-gauge frozen batch reference (m) are respectively:
0.124431/0.563985, 0.124359/0.566468, 0.108179/0.574742,
0.111753/0.571735, and 0.123634/0.564448. Decision:
**NO_STANDARD_CONFIG_MET_GATES**, selected configuration = none; V4 is an
offline-only diagnostic and is ineligible for standard selection.

Changing only skip10 to skip1 reduces K10 translation p95 from 2.456849 to
0.135119 m, but K2 remains 12.286618 m for V0–V3. Lower thresholds increase
ordinary mean update latency from V1's 3.041299 to V2's 5.183803 and V3's
8.254052 ms without monotonic final agreement. V4's 225 empty calls materially
change poses in 25 calls; K2 p95 improves to 1.679341 m but still fails. Empty
calls also advance the relinearization skip counter, so they do not isolate
nonlinear iteration count alone. Backend-only interval checks pass, not an
end-to-end real-time claim. Independent execution/provenance audit PASS;
numerical readiness remains negative. See
`docs/CUMULTI_STAGE13C_ISAM2_CONVERGENCE.md` and the new Stage13C artifacts.


## Stage 14 — final-state consistency / path dependence (verified, GT-free)

Historical120 Rank1 sanitized loops are filtered only by frozen stable endpoint
ranges to the identical75-loop clean set; excluded45 are not restored. Independent
runner evaluates nine freshD2 graphs (three initializations at K2/K75/FINAL),
five empty calls each, plus six frozen-parameter LM basin probes. All final
5796nodes/5794odom/75loops/one-prior factors remain unchanged.

All K2 fresh conditions pass, p95 approximately0.054–0.055m. K75 A1/A2/A3
p95 is0.560519/0.568438/0.568640m; FINAL is0.563622/0.570785/0.570995m,
all FAIL unchanged original gates. Rebuilding from persistent final coordinates
moves poses measurably (A2/D0 p95 movement0.030435m; A3/D2 0.020879m), but
is insufficient to repair the final residual. Fresh initialization differences
are measurable yet mutually within original gates.

Canonical LM reproduces the saved reference; persistent-initialized LM does
not converge to essentially the same coordinates. Final B1/B2 p95 difference
is0.564032/0.574739m with objective23.840367/23.840377 versus canonical
23.840384. This shows initialization/termination sensitivity, not multiple
proven global minima or a physically correct batch trajectory.

Persistent Robot3 discrepancy is LONG_CHAIN_SMOOTH_DEFORMATION: adjacent-delta
p95 is0.003740/0.003814m forD0/D2; PCA first component explains83.52%/81.07%.
Nearest loop-endpoint KF-distance Pearson/Spearman associations are
0.888/0.729 and0.900/0.723, respectively; associations are not observability
proof. All18 selected6x6 marginals are finite/PSD-valid in a bounded sparse
worker (~317MiB peakRSS); frozen-weight local information is weak/nonuniform,
including large lateRobot1 uncertainty. No global denseHessian/nullspace claim.

Decision: **RESIDUAL_NUMERICAL_DISCREPANCY_UNRESOLVED** under the frozen
conservative rule. Independent audit PASS; nine figures complete. One necessary
exact-policy recovery after a reporting failure reproduces all66 saved state
coordinates exactly (maxdelta0); only recovery runtime is primary. No GT,
frontend/maps/demo modification, gate relaxation or production-ready claim.
See `docs/CUMULTI_STAGE14_FINAL_CONSISTENCY.md`.
