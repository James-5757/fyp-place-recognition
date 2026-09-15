# Next steps

CU-Multi Main Campus robot1/robot2 Stage 1 is complete: the read-only raw archives were inspected and `/home/cas/CU-Multi/processed_v1/` now contains a validated 100-keyframe-per-robot subset. See `docs/CUMULTI_STAGE1_VALIDATION.md` for actual topics, synchronization quality, calibration assumptions, and unresolved coordinate-frame questions.

Do not expand this subset to the full dataset or run retrieval yet. Before Stage 2, confirm the upstream UTM zone/datum and the GT measurement-frame origin; export/inspect CameraInfo intrinsics only if camera geometry becomes necessary. Keep GT restricted to synchronization, overlap/positive definition, analysis and evaluation—never candidate selection or reranking.

After those questions are resolved, design the CU-Multi retrieval protocol separately from historical KITTI: define robot1/robot2 database-query split, temporal sampling, overlap labels and held-out evaluation. Preserve SC + Cross-Max as the historical KITTI retrieval baseline; do not overwrite `outputs/canonical_v2` or treat the Stage 1 subset as a retrieval result.

For visual experiments, measure OpenCLIP viewpoint sensitivity explicitly and compare single view, mean pooling, Cross-Max and controlled multi-view aggregation under a fixed candidate protocol. Record dataset version, selected frames, timestamps, thresholds, pose frame, calibration and command in the experiment log for every new experiment.

## Current Stage 4 stop point

Stage 4 has completed the Robot1-to-Robot3 LiDAR-only Scan Context baseline. It retains the frozen Robot1 Stage 2 cache, has a formal Robot3 LiDAR timestamp grid, and ranks all Robot3 descriptors without GT filtering. The primary R@1 is 0.993453 over 1,833 valid-overlap queries. Do not overwrite its outputs, its derived cache, or the frozen Robot1-to-Robot2 Stage 2 baseline.

The next authorized research step should be a deliberately isolated viewpoint experiment. Keep the Stage 4 Scan Context candidate protocol fixed, explicitly stratify offline evaluation by nearest-positive heading difference, and compare any visual representation only against the same candidates. Begin with an RGB data-integrity and timestamp audit, then test OpenCLIP single-view sensitivity before any mean pooling or Cross-Max aggregation. Do not allow GT heading, pose or overlap values to enter candidate selection or reranking. No VLM, CVTNet, GICP or PGO is part of this next step unless separately authorized.
