# CU-Multi Stage 12D — stable-segment clean backend graph

## 1. Motivation

Stage 10.3 demonstrated that native GTSAM could solve the supplied graph numerically. Stages 12A–C then showed that the graph inherited early EKF excursions of hundreds of metres. Stage 12D tests the same frozen Rank1/GICP/Pose3 backend on the independently selected Stage 12C delayed-start segments. This is a clean batch baseline; no K sweep, iSAM2, new retrieval, new registration, or demo is executed.

Source: `src/cumulti/run_stage12d_clean_backend.py`; entrypoint: `scripts/run_stage12d_clean_backend.sh`; outputs: `outputs/cumulti_v1/12d_clean_backend/`. Historical inputs and outputs are read-only. Two PLY maps remain server-only, with committed size/SHA256 manifests.

## 2. Frozen stable-segment policy

`clean_graph_policy.json` and `initialization_equivalence_policy.json` were persisted before graph construction/solving. Robot1 keeps original IDs **150–1999** (1,850 nodes); Robot3 keeps **234–4179** (3,946 nodes). These are exactly the starts in the frozen Stage 12C selection. The graph has **5,796 nodes**, **5,794 sequential odometry factors**, **75 inter-robot loop factors**, and one prior on Robot1 KF150. No semantic keyframe IDs are reassigned.

The local pose matrices retain their exact Stage 10.3 KF0-relative coordinate gauge; only the retained ranges change. Robot1 KF150 is anchored at its frozen local pose. No trajectory is smoothed, repaired, interpolated, or rebased. Figures subtract constant display origins for readability, which does not change any graph pose or factor.

## 3. Covariance-only alerts retained

All retained nodes, including Stage 12C's late covariance-only health alerts, remain in the graph. `clean_covariance_health_metadata.csv` retains these covariance values and flags. Absolute EKF pose covariance is not directly equivalent to the covariance of independent relative-pose factors: the required propagation and cross-correlations are unavailable. `covariance_policy_note.json` records that factor weights did not change. Both odometry and loops retain **1 m translation / 5° rotation** diagonal noise. Odometry is not robustified; loops use **Huber 1.345**. This Stage 12D policy is explicitly authorized independently of the stricter compound health assessment in Stage 12C; it does not rewrite that historical assessment.

## 4. Clean graph construction and frames

The runner reconstructs the frozen non-GT EKF local trajectory with the same Stage 10.3 nearest-timestamp selection and IMU-to-LiDAR convention. Before the pre-GT decision, `keyframes.csv` is read using only `keyframe_id`, `lidar_timestamp_ns`, and `lidar_file`; GT pose columns are not loaded. Raw `ekf/odometry_map` poses are composed with the audited `T_imu_from_lidar`, quaternion `[0,0,1,0]` and translation `[-0.06286,0.01557,0.053345]` m. Graph nodes and cloud coordinates are LiDAR/`os_sensor`.

Every retained odometry factor is `Z_i,i+1 = X_i^-1 X_i+1`, using continuous original IDs. GTSAM Pose3 tangent noise ordering is `[rx,ry,rz,tx,ty,tz]`. The tight prior uses the same Stage 10.3 six-dimensional `1e-6` sigmas. `clean_graph_manifest.json`, `odometry_source_audit.json`, and `frozen_input_manifest.json` document actual counts, frames, files, and hashes. Each retained local step is independently checked against the matching frozen Stage 12A row before GTSAM runs.

## 5. Frozen loop filtering

The original 120-row `rank1_sanitized_edges.csv` hash is verified against the Stage 10.3 frozen manifest:

`65fb62d7895e68b817d6ede8ff9acfd4b6a9d8e2219c1194b7b7ff6226e8c7d3`.

Filtering only Robot1 query ID ≥150 and Robot3 candidate ID ≥234 retains exactly **75** rows, matching Stage 12C. Measurements, SC scores/shifts, GICP quality, and transform direction are preserved. GT labels in the original CSV are not loaded. `clean_loop_provenance_audit.json` verifies the retained rows and unchanged transform values. Graph-building aborts on a count, continuity, hash, or endpoint mismatch.

## 6. First-arrival loop definition

`ordered_rank1_clean_loops.csv` orders by Robot1 query timestamp/keyframe, with candidate keyframe as tie-break. `arrival_index` is one-based; `frozen_loop_id` is the original zero-based row ID in the 120-loop source. The order represents **query arrival in a separate-run R1→R3 replay**, not physically simultaneous robot clocks. The first row is frozen loop **45**, **Robot1 KF283 → Robot3 KF252**, SC score **0.8145303130**, GICP quality **0.7493432841**. It is selected from arrival order without GT.

## 7. First-loop common frame and connectivity

With `Z_qc = T_query_from_candidate`, the frozen composition is `S = X_q @ Z_qc @ inverse(L_c)` and `X3_global(j) = S @ L3(j)`. Robot1 retains its local frame. The first-edge measurement check gives **2.512e-15 m / 1.591e-15°** residual. `first_loop_initialization.json` stores the full measurement and common-frame matrix.

Union-find over the actual retained odometry chains yields **two connected components before the first loop and one after it**, recorded in `first_loop_connectivity.json`. One inter-robot loop provides graph connectivity and initial map alignment. It does not guarantee globally accurate trajectories.

## 8. Initialization sensitivity — identical-seed limitation

The predeclared numerical limits are translation median ≤0.10 m / p95 ≤0.50 m and orientation median ≤0.5° / p95 ≤2°. Both required initialization rules were run on the same complete 75-factor graph with the same anchor and LM parameters.

For this frozen subset, **FIRST_ARRIVAL_INIT and HIGHEST_GICP_QUALITY_INIT select the same edge**, KF283→KF252. Both runs therefore use identical initial values; resulting per-node translation and rotation differences are zero. `initialization_sensitivity.csv` and its summary explicitly mark `same_initialization_edge=true`, `distinct_seeds_tested=false`, and **EQUIVALENT_IDENTICAL_SEEDS_WARN**. The comparison demonstrates rule compatibility and repeatability, not robustness to genuinely different initial seeds. No substitute edge or artificial perturbation was introduced. FIRST_ARRIVAL_INIT remains the default for future replay semantics.

## 9. Clean full-loop GTSAM result

The isolated `/home/cas/.venvs/fyp_gtsam` environment uses GTSAM 4.2 / Python 3.10.20. The runner records actual default LM getters in `clean_gtsam_solver_policy.json`: maximum 100 iterations, relative/absolute error tolerance `1e-5`, initial lambda `1e-5`, lambda factor 10, upper lambda bound 100000, and multifrontal Cholesky/COLAMD. No parameter is tuned from GT.

| Initialization | Initial graph error | Final graph error | Reduction | Runtime | Iterations |
|---|---:|---:|---:|---:|---:|
| First arrival | 3843.986006 | 23.840384 | 99.3798% | 0.350595 s | 7 |
| Highest GICP quality (same edge) | 3843.986006 | 23.840384 | 99.3798% | 0.347814 s | 7 |

Both returned normally, below the iteration cap, with finite poses. The Robot1 anchor translation error is zero and orientation error ~2.05e-16°. The canonical full-75 result reuses the first-arrival solve from this comparison; it is not rerun after GT evaluation. Normal return and iteration count are recorded; the binding does not expose a specific tolerance-stop reason. Original semantic IDs and timestamps are retained in both exported optimized trajectories, with finite/unit-quaternion/continuity checks.

## 10. Clean loop residuals

For every loop, `E = Z^-1 X_query^-1 X_candidate`. Physical translation norm and relative rotation angle follow the Stage 10.3 diagnostic convention. Translation median/p90/p95/max is **0.052696 / 0.138310 / 0.196202 / 0.342854 m**. Rotation median/p90/p95/max is **0.415948 / 1.103571 / 1.383935 / 2.525095°**.

The reconstructed diagnostic whitened norm uses matrix translation /1 m and rotation vector /5°. Every diagnostic Huber weight is 1; counts below 0.1/0.25/0.5 are all zero. This is labeled `diagnostic_reconstructed_huber_weight`, not the internal GTSAM weight: native Pose3 factors use the Lie Logmap translation tangent, which is not identical to physical matrix translation. No factor weight is replaced with this diagnostic.

## 11. Retained input and post-PGO trajectory integrity

`clean_local_trajectory_integrity.csv` and summary report median/p95/p99/max for XY/XYZ translation, relative rotation, and implied speed. Maximum local adjacent XY is **1.127652 m** for Robot1 and **1.470416 m** for Robot3; the early 300-m excursions are absent from the retained graph. The frozen pre-solve catastrophic gate is 100-m XYZ, with an additional post-PGO lineage test using five times the Stage 12C step limit. No thresholds are fitted to the optimized result.

`clean_post_pgo_step_audit.csv` records local, first-loop common-frame, and optimized values on identical adjacent IDs. Optimized maximum XY steps are **1.141472 / 1.410912 m**; maximum XYZ steps **1.165282 / 1.707105 m**. No new catastrophic discontinuity appears. Neither robot's post-PGO top-20 XY steps contains a new robust flag without a pre-common-frame flag. Sharp-turn rotation statistics are retained as diagnostics rather than automatically classified as catastrophic translation.

## 12. GT-free clean maps

Both maps use the exact frozen sampling: frame stride **10**, point stride **16**, **0.5 m** voxel, finite points with 1–60 m range. The frame-stride origin is the first retained keyframe of each robot. The symmetric nearest-neighbor metric uses the union of R1→R3 and R3→R1 distances between independently voxelized robot clouds.

| Condition | NN median | NN p90 | NN p95 |
|---|---:|---:|---:|
| First-loop-only common frame | 10.865166 m | 120.651606 m | 144.832800 m |
| All 75 loops, clean GTSAM | 3.030506 m | 92.692846 m | 119.277542 m |

The maps contain finite points; server-only PLYs have reproducible hash/size records in `clean_map_manifest.json`. These large full-map distances include unmatched spatial coverage and residual odometry distortion. A finite map and improved cross-map NN distribution do not establish a globally accurate reconstructed scene.

## 13. Pre-GT clean backend decision

`pre_gt_clean_backend_decision.json` was persisted before GT pose loading. The result is **CLEAN_BACKEND_READY** for the defined batch-backend sanity conditions: valid frozen ranges/factors, valid first-loop alignment, normal finite solve, no new catastrophic step, finite loop residuals and map. Initialization is compatible under the frozen criterion, with the identical-seed caveat above. `execution_phase_audit.json` verifies solver result → persisted pre-GT decision → offline GT result chronology. The original decision was not rewritten during reporting recovery. No GT improvement is required or used for this readiness choice.

## 14. Offline GT diagnostic

Only after the decision exists does the evaluator load GT fields. ATE applies **one joint global rigid alignment to both robots**, never separate robot alignments. RPE uses exact 10-keyframe intervals separately per robot; combined RMSE is computed from the union of 1,840 Robot1 and 3,936 Robot3 relative-error samples.

| Condition | Joint ATE | Robot1 ATE | Robot3 ATE |
|---|---:|---:|---:|
| First-loop only | 16.965798 m | 24.771673 m | 11.623035 m |
| All 75 loops | 19.607957 m | 12.108315 m | 22.270788 m |

| Condition | Robot1 RPE m / deg | Robot3 RPE m / deg | Combined RPE m / deg |
|---|---:|---:|---:|
| First-loop only | 9.451816 / 5.890492 | 11.024930 / 8.435852 | 10.549292 / 7.716677 |
| All 75 loops | 9.487356 / 5.996891 | 10.998888 / 8.137056 | 10.540927 / 7.521674 |

These are **OFFLINE GT ONLY** values under the frozen historical evaluation convention. Joint ATE worsens with all 75 loops compared with the clean first-loop baseline; Robot1 improves while Robot3 worsens. The optimizer was not rerun, retuned, or selected using these values. Numerical backend readiness must not be described as improved global trajectory accuracy.

## 15. Historical versus clean comparison

`historical_vs_clean_backend.csv` reads frozen Stage 10.3 metrics without rerunning that experiment. Historical Rank1 has 6,180 nodes, 6,178 odometry factors and 120 loops, runtime **7.553377 s**, loop translation p95 **1.408717 m**, map NN median/p95 **2.766472 / 138.380483 m**, and maximum adjacent XY **287.045 / 336.712 m**. Its offline joint ATE is **30.638930 m**. Clean Stage 12D uses fewer nodes/loops and different retained time ranges, with the values above. This is a data-quality/backend sanity comparison, not a fair algorithm benchmark. Raw graph objective values across the different factor sets are not compared as equivalent scores.

## 16. Frozen Stage 13 schedule

`stage13_loop_schedule.json` freezes **K = 1, 2, 5, 10, 20, 40, 75**, always the first K rows of the query-arrival order. Each entry lists original frozen loop IDs, first/last query IDs, quality statistics, endpoint bounds in each local frame, path-distance spans, and bounds in the first-loop common frame. All proxies are non-GT. No K is selected as best, and no sparse-loop optimization has run.

## 17. Interpretation

The first valid frozen loop connects the two odometry chains and permits initial map fusion. Subsequent loop factors refine graph and inter-map consistency in this batch result, but they do not guarantee improved offline GT accuracy. Removing the frozen early initialization ranges resolves the catastrophic-step issue in the graph, while residual odometry/map distortion and identical-seed sensitivity coverage remain limitations. This supports a clean **offline** baseline for the next declared experiment, not real-time or decentralized deployment.

## 18. Reproducibility and next stage

From repository root on the CU-Multi server: `bash scripts/run_stage12d_clean_backend.sh`. The committed runner performs input verification, both full-75 solves, maps, pre-GT decision, offline evaluation, six figures, and validation. `--finalize-only` recovers reporting/figures from saved solver outputs without optimization or GT re-evaluation; `--validate-only` checks provenance/chronology and report consistency. A reporting recovery was used after the original figure generation encountered an empty separate-panel series; solver outputs, pre-GT decision, maps, and offline metrics were preserved. No graph solve was repeated after GT was seen.

`VALIDATION_REPORT.txt` contains all requested gates and an explicit initialization-sensitivity WARN. The clean baseline is ready for a separately authorized Stage 13 first-loop/sparse-loop/incremental study. Stage 12D stops with the schedule prepared; the K sweep, iSAM2 and existing demo remain untouched.
