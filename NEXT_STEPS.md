# Next steps

CU-Multi Main Campus robot1/robot2 Stage 1 is complete: the read-only raw archives were inspected and `/home/cas/CU-Multi/processed_v1/` now contains a validated 100-keyframe-per-robot subset. See `docs/CUMULTI_STAGE1_VALIDATION.md` for actual topics, synchronization quality, calibration assumptions, and unresolved coordinate-frame questions.

Do not expand this subset to the full dataset or run retrieval yet. Before Stage 2, confirm the upstream UTM zone/datum and the GT measurement-frame origin; export/inspect CameraInfo intrinsics only if camera geometry becomes necessary. Keep GT restricted to synchronization, overlap/positive definition, analysis and evaluation—never candidate selection or reranking.

After those questions are resolved, design the CU-Multi retrieval protocol separately from historical KITTI: define robot1/robot2 database-query split, temporal sampling, overlap labels and held-out evaluation. Preserve SC + Cross-Max as the historical KITTI retrieval baseline; do not overwrite `outputs/canonical_v2` or treat the Stage 1 subset as a retrieval result.

For visual experiments, measure OpenCLIP viewpoint sensitivity explicitly and compare single view, mean pooling, Cross-Max and controlled multi-view aggregation under a fixed candidate protocol. Record dataset version, selected frames, timestamps, thresholds, pose frame, calibration and command in the experiment log for every new experiment.

## Current Stage 9 stop point

Stage 9 has completed the frame-gated SC--GICP integration without changing the
Stage-8 frontend or GICP algorithm. Fixed M1000 remains the retrieval baseline;
Rank-1 is mean-budget compatible at 500 ms on the tested host, but its p95
exceeds that budget and Top-3 early stop exceeds it on mean latency. The exported
139 sanitized candidate loop constraints are registration outputs only, not
guaranteed-correct loop closures or a graph-optimization result. Their robustness
must be evaluated in a separately frozen Stage-10 pose graph.

Stop here. Do not start PGO, map merging, pose-graph tuning, CVTNet, or fusion
tuning without a separately declared graph protocol: robust-kernel/edge-weight
rule, odometry source, loop evaluation split, failure containment, and offline
GT-only trajectory evaluation. Keep GT offline and preserve Stage 4/5/6/7/7.5/8/9
outputs.

## Stage 10A blocking condition

The Stage-10 odometry audit found an admissible Robot1 EKF IMU/GNSS odometry
candidate, but no admissible Robot3 non-GT local trajectory. The locally present
Robot3 relative-pose bag is GT data and must not be substituted. Transfer only
`robot3_main_campus_imu_gps.zip`, then repeat the audit before any PGO or map
merge work. See `docs/CUMULTI_STAGE10_ODOMETRY_AUDIT.md`.

## Stage 10.1 stop point

The corrected Stage-10.1 replay passes its initialization, extrinsic, frame, and objective-bookkeeping checks, but both solvers reach the deliberately frozen 25-evaluation cap. Its map and ATE outcomes are mixed. Do not describe it as a converged PGO system or use it for a live demo. Any follow-up must be declared as a separate policy experiment, with GT remaining offline-only and without rewriting the Stage-10/10.1 historical outputs.
