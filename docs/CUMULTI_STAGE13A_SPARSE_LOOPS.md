# CU-Multi Stage 13A — Sparse frozen-loop batch evaluation

## 1. Research questions

Test first-loop connectivity, loop-budget/map agreement, sparse-K sufficiency relative to K75, batch cost, and agreement with offline trajectory accuracy. These are distinct outcomes.

## 2. Accepted Stage12D checkpoint

`8946e29eb74658bfb0a5bf85635ff1aeac05a417`. Frozen inputs/hashes: `stage13a_input_manifest.json`; input immutability audit accompanies outputs.

## 3. Fixed graph and loop policy

Robot1 KF150–1999 (1850), Robot3 KF234–4179 (3946): 5796 nodes, 5794 odometry factors, one Robot1 KF150 prior. Exact Stage12D LiDAR/os_sensor gauge/extrinsic and factors are reused by constructing its 75-loop template and copying the prior/all odometry/first K factor prefix. Rotation 5°, translation 1 m, loop Huber 1.345, no robust odometry. No covariance alert removed or covariance-based weight introduced.

## 4. Offline replay arrival-order limitation

First K Robot1 query-arrival rows with a pre-existing Robot3 database. Separate runs are NOT a physically synchronized causal two-stream schedule; future Robot3 frames may already be in the offline database. No live/decentralized/real-time claim.

## 5. First-loop connectivity

Actual union-find checks two odometry chains: 2 components -> 1. Frozen R1#283 ↔ R3#252 establishes `S=Xq Zqc inverse(Lc)`; initialization residual 3.97e-15 m / 2.69e-15°. K1 maximum pose difference from frozen first-loop trajectory: 4.74e-13 m / 4.25e-15 rad.

One loop is sufficient for initial graph connectivity, but does not guarantee globally accurate map reconstruction.

## 6. K=1,2,5,10,20,40,75 setup

Only loop prefix length changes; all original IDs, timestamps, nodes, odometry and prior stay fixed. Schedule/hash verified directly against Stage12D.

## 7. Identical initialization protocol

Same frozen first-loop common-frame x0 for every K, every warmup, every measured repetition. Never initialize from smaller-K solutions. Initial matrix SHA256 recorded per condition.

## 8. GTSAM results

GTSAM 4.2, isolated `/home/cas/.venvs/fyp_gtsam`; same Stage12D default LM getters verified exactly, no tuning. Normal return does not expose a specific tolerance-stop reason. Cross-K raw objectives have differing factors and must not rank quality.

Reproduction entrypoint: `bash scripts/run_stage13a_sparse_loops.sh` from repository root, with an empty Stage13A output directory. The runner refuses to overwrite existing outputs or rerun after a saved GT decision. `--report-only` regenerates documentation from saved tables without optimization/GT evaluation; `python src/cumulti/audit_stage13a_sparse_loops.py` validates schemas, hashes, decision chronology and RPE aggregation without a solve. K1's objective is already numerical zero (~1e-24); its relative reduction has no meaningful interpretation as an optimization gain.

| K | initial_error | final_error | iterations | termination | finite_poses |
| --- | --- | --- | --- | --- | --- |
| 1 | 0.000000 | 0.000000 | 1 | NORMAL_RETURN | True |
| 2 | 27.321859 | 0.078412 | 7 | NORMAL_RETURN | True |
| 5 | 80.604714 | 0.284931 | 7 | NORMAL_RETURN | True |
| 10 | 322.727296 | 1.723336 | 8 | NORMAL_RETURN | True |
| 20 | 1039.577935 | 2.944522 | 7 | NORMAL_RETURN | True |
| 40 | 2064.920985 | 15.770975 | 7 | NORMAL_RETURN | True |
| 75 | 3843.986006 | 23.840384 | 7 | NORMAL_RETURN | True |

## 9. Trajectory integrity

All K reuse Stage12D lineage catastrophe rule (XYZ >5× frozen Stage12C XY limit newly, or newly ≥100m). Every trajectory has 5796 finite/unit-quaternion/monotonic-ID rows. See `trajectory_integrity_by_K.csv` for per-robot XY/XYZ/rotation/speed maxima and clean baseline. No new catastrophic discontinuity.

## 10. Inserted vs uninserted residuals

Exact `E=inverse(Z) inverse(Xq) Xc`: physical translation norm and rotation angle. Inserted K, remaining 75−K, and common full75 measured separately. Remaining/full75 are OFFLINE HINDSIGHT diagnostics, not metrics causally available during replay. K75 remaining set is empty N/A, not zero. Reconstructed Huber weight is diagnostic, not internal Pose3 Logmap weight.

| K | translation_median_m | translation_p95_m | rotation_median_deg | rotation_p95_deg |
| --- | --- | --- | --- | --- |
| 1 | 43.993923 | 56.775505 | 29.258950 | 68.530607 |
| 2 | 40.072930 | 48.524264 | 50.162487 | 90.568232 |
| 5 | 2.904412 | 28.887825 | 17.667030 | 51.280914 |
| 10 | 7.967033 | 20.588324 | 22.985033 | 77.137184 |
| 20 | 4.799500 | 18.027137 | 13.818365 | 73.172941 |
| 40 | 0.203237 | 48.751255 | 1.615119 | 49.098126 |
| 75 | 0.052696 | 0.196202 | 0.415948 | 1.383935 |

## 11. Global map NN results

Frozen scans/frame stride10 from retained starts, point stride16, voxel0.5m, range1–60m. Each robot independently voxelized with preserved source attribution; concatenate both directed 3D NN arrays for statistics. Global NN is affected by nonoverlap coverage and is not GT pose accuracy.

| K | median_m | p90_m | p95_m | points_robot1 | points_robot3 | solve_mean_s | median_m_ROI | p95_m_ROI |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 10.865166 | 120.651606 | 144.832800 | 279845 | 655410 | 0.115073 | 3.238937 | 21.839732 |
| 2 | 8.102668 | 105.697233 | 129.738145 | 279855 | 660432 | 0.349497 | 3.066004 | 21.594200 |
| 5 | 4.288245 | 110.131079 | 135.556565 | 280046 | 660778 | 0.356874 | 1.781834 | 17.551371 |
| 10 | 4.228913 | 89.744532 | 115.804903 | 282098 | 665201 | 0.393496 | 1.875424 | 18.754738 |
| 20 | 4.099509 | 91.034713 | 117.668069 | 281442 | 664484 | 0.347927 | 1.844217 | 19.650516 |
| 40 | 6.188868 | 88.458555 | 115.251448 | 282826 | 663221 | 0.343200 | 1.742609 | 19.332942 |
| 75 | 3.030506 | 92.692846 | 119.277542 | 280707 | 665213 | 0.360765 | 1.339788 | 15.463829 |

## 12. Fixed overlap-region map results

GT-FREE LOOP-REGION MAP DIAGNOSTIC, not a GT overlap mask. Union of 20m XY cylinders around all150 frozen loop endpoints in the frozen first-loop common frame, saved before solving. Same fixed centers at every K; ≥100 points per robot required, otherwise explicit N/A. See table ROI columns and `overlap_map_consistency_by_K.csv`.

## 13. Runtime tradeoff

One warmup + three measured solves per K from same x0. Mean/median/empirical p95 recorded; three samples do not estimate stable tail latency. Solve excludes map/decompression; template construction separate, per-K graph construction is factor-prefix copying. Map construction/export, consistency and canonical total reported separately; actual wall includes extra benchmark runs/diagnostics. Hardware in `runtime_environment.json`. Batch solve, not incremental update or 2Hz deployment evidence.

## 14. Pre-GT sparse-K decision

`NO_SPARSE_SAVING`, selected K=None. Smallest K<75 with BOTH global median and p95 ≤1.10×K75, normal finite solver, no new catastrophic step. K75 is consistency reference, not truth. Secondary ROI qualification is reported but does not select K. Decision persisted at 2026-10-08T06:58:53.913957+00:00 before any new GT access. GT used for selection: NO.

## 15. Offline GT ATE/RPE

OFFLINE GT DIAGNOSTIC ONLY. One joint rigid alignment across both robots (not independent robot alignment). RPE interval10, per robot plus union of relative samples.

| K | joint | robot1 | robot3 |
| --- | --- | --- | --- |
| 1 | 16.965798 | 24.771673 | 11.623035 |
| 2 | 21.181950 | 14.331587 | 23.721974 |
| 5 | 17.771334 | 16.761708 | 18.225429 |
| 10 | 22.382595 | 14.175470 | 25.330742 |
| 20 | 21.899635 | 14.800222 | 24.530518 |
| 40 | 27.597901 | 33.238967 | 24.510183 |
| 75 | 19.607957 | 12.108315 | 22.270788 |

| K | scope | translation_RMSE_m | rotation_RMSE_deg | sample_count |
| --- | --- | --- | --- | --- |
| 1 | robot1 | 9.451816 | 5.890492 | 1840.000000 |
| 1 | robot3 | 11.024930 | 8.435852 | 3936.000000 |
| 1 | combined | 10.549292 | 7.716677 | 5776.000000 |
| 2 | robot1 | 9.451965 | 5.891715 | 1840.000000 |
| 2 | robot3 | 11.022620 | 8.421906 | 3936.000000 |
| 2 | combined | 10.547690 | 7.706587 | 5776.000000 |
| 5 | robot1 | 9.448997 | 5.933764 | 1840.000000 |
| 5 | robot3 | 11.023989 | 8.396337 | 3936.000000 |
| 5 | combined | 10.547817 | 7.697847 | 5776.000000 |
| 10 | robot1 | 9.436405 | 6.028832 | 1840.000000 |
| 10 | robot3 | 11.031042 | 8.416476 | 3936.000000 |
| 10 | combined | 10.549251 | 7.736270 | 5776.000000 |
| 20 | robot1 | 9.430758 | 6.044676 | 1840.000000 |
| 20 | robot3 | 11.031517 | 8.371277 | 3936.000000 |
| 20 | combined | 10.547980 | 7.706733 | 5776.000000 |
| 40 | robot1 | 9.496524 | 6.022123 | 1840.000000 |
| 40 | robot3 | 10.991843 | 8.350458 | 3936.000000 |
| 40 | combined | 10.538549 | 7.685689 | 5776.000000 |
| 75 | robot1 | 9.487356 | 5.996891 | 1840.000000 |
| 75 | robot3 | 10.998888 | 8.137056 | 3936.000000 |
| 75 | combined | 10.540927 | 7.521674 | 5776.000000 |

More loop constraints do not necessarily imply monotonic improvement in offline trajectory accuracy.

Largest absolute global-median step occurs at K5. Global median worsens on insertion to K=[40]; global p95 worsens to K=[5, 20, 75]; joint ATE worsens to K=[2, 10, 40]. Thus neither agreement nor accuracy is assumed monotonic. K1→K75 global median changes 10.865166→3.030506 m while joint ATE changes 16.965798→19.607957 m. No monotonic saturation claim is supported if intermediate budgets regress.

Frozen loop-quality and endpoint coverage diagnostics (same first-loop frame, no extra selection):

| K | GICP_quality_median | GICP_quality_p95 | robot1_endpoint_path_span_m | robot3_endpoint_path_span_m | endpoint_bbox_x_m | endpoint_bbox_y_m |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0.749343 | 0.749343 | 0.000000 | 0.000000 | 0.683978 | 0.261823 |
| 2 | 0.711856 | 0.745595 | 27.856423 | 1808.948788 | 22.514237 | 16.246260 |
| 5 | 0.694030 | 0.741800 | 134.643815 | 1916.042952 | 22.514237 | 32.674460 |
| 10 | 0.687402 | 0.741591 | 213.929457 | 2326.715046 | 68.213919 | 47.276899 |
| 20 | 0.678106 | 0.732976 | 233.217159 | 2326.715046 | 86.397704 | 58.672827 |
| 40 | 0.659350 | 0.717303 | 461.810932 | 2492.740420 | 99.719326 | 79.987574 |
| 75 | 0.642445 | 0.712606 | 1079.436305 | 2759.185287 | 117.608678 | 118.266230 |

Larger endpoint coverage can plausibly redistribute drift along both odometry chains; qualities and spatial spread are observational correlates only, not tested causes. Map agreement, trajectory accuracy, residuals and runtime remain distinct. No parameter/selection changes or solves follow GT.

## 16. K1/K75 reproduction

K1 PASS=True; K75 PASS=True. Exact reference values read from committed Stage12D files, not written as new measurements. Tolerances frozen before solves: {"map_absolute_m": 1e-05, "graph_absolute": 1e-07, "residual_absolute_m": 1e-07, "pose_translation_m": 1e-07, "pose_rotation_rad": 1e-08, "reason": "same deterministic factors/solver/scans; float64 CSV round-trip tolerance fixed before new runs"}. See `stage12d_reproduction_audit.json` actual differences. Historical outputs unchanged.

## 17. Limitations

One fixed robot pair, seven prefix budgets, single frozen seed, three timing samples; no guarantee of globally correct loop measurements/odometry, map NN confounded by route coverage. Fixed all75-endpoint ROI and uninserted residuals have hindsight information. No alternate loop ordering or causal online scheduler tested. Late covariance metadata retained, factor noise unchanged. No live demo or iSAM2 started. Server-only PLY hashes/sizes tracked in manifest; nine measured-data figures produced.

## 18. Recommended Stage13B design

`READY_FOR_INCREMENTAL_METHOD_STUDY`. Separate method study must declare a causal two-stream scheduler, candidate-database availability, first-fusion and arrival events, and batch-reference agreement before iSAM2. This study's query-order prefixes are insufficient for simultaneous multi-robot fusion claims. STOP after Stage13A; Stage13B not started.
