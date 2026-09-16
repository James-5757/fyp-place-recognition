# Next steps

CU-Multi Main Campus robot1/robot2 Stage 1 is complete: the read-only raw archives were inspected and `/home/cas/CU-Multi/processed_v1/` now contains a validated 100-keyframe-per-robot subset. See `docs/CUMULTI_STAGE1_VALIDATION.md` for actual topics, synchronization quality, calibration assumptions, and unresolved coordinate-frame questions.

Do not expand this subset to the full dataset or run retrieval yet. Before Stage 2, confirm the upstream UTM zone/datum and the GT measurement-frame origin; export/inspect CameraInfo intrinsics only if camera geometry becomes necessary. Keep GT restricted to synchronization, overlap/positive definition, analysis and evaluation—never candidate selection or reranking.

After those questions are resolved, design the CU-Multi retrieval protocol separately from historical KITTI: define robot1/robot2 database-query split, temporal sampling, overlap labels and held-out evaluation. Preserve SC + Cross-Max as the historical KITTI retrieval baseline; do not overwrite `outputs/canonical_v2` or treat the Stage 1 subset as a retrieval result.

For visual experiments, measure OpenCLIP viewpoint sensitivity explicitly and compare single view, mean pooling, Cross-Max and controlled multi-view aggregation under a fixed candidate protocol. Record dataset version, selected frames, timestamps, thresholds, pose frame, calibration and command in the experiment log for every new experiment.

## Current Stage 7.5 stop point

Stage 7.5 adds a read-only supervisor control to Stage 7. In same-view Robot1-to-
Robot3 comparisons, SC remains stronger than the cached generic visual methods even
at <=10-degree heading difference and <2 m nearest-positive distance. The actual
processed LiDAR frames are approximately full-azimuth under the predeclared 10-degree
occupancy test. These results separate FoV advantage, geometry robustness, and the
limits of the current generic OpenCLIP representation; they do not establish a
universal camera-versus-LiDAR claim.

Stop here. Do not start GICP, CVTNet, PGO, SC retrieval acceleration or fusion
tuning. A subsequent claim should use a newly predeclared, meaningfully hard
route/robot split or a revised independent hypothesis. Keep GT offline and preserve
Stage 4/5/6/7/7.5 outputs.
