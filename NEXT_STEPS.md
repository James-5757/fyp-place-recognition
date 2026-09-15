# Next steps

CU-Multi Main Campus robot1/robot2 Stage 1 is complete: the read-only raw archives were inspected and `/home/cas/CU-Multi/processed_v1/` now contains a validated 100-keyframe-per-robot subset. See `docs/CUMULTI_STAGE1_VALIDATION.md` for actual topics, synchronization quality, calibration assumptions, and unresolved coordinate-frame questions.

Do not expand this subset to the full dataset or run retrieval yet. Before Stage 2, confirm the upstream UTM zone/datum and the GT measurement-frame origin; export/inspect CameraInfo intrinsics only if camera geometry becomes necessary. Keep GT restricted to synchronization, overlap/positive definition, analysis and evaluation—never candidate selection or reranking.

After those questions are resolved, design the CU-Multi retrieval protocol separately from historical KITTI: define robot1/robot2 database-query split, temporal sampling, overlap labels and held-out evaluation. Preserve SC + Cross-Max as the historical KITTI retrieval baseline; do not overwrite `outputs/canonical_v2` or treat the Stage 1 subset as a retrieval result.

For visual experiments, measure OpenCLIP viewpoint sensitivity explicitly and compare single view, mean pooling, Cross-Max and controlled multi-view aggregation under a fixed candidate protocol. Record dataset version, selected frames, timestamps, thresholds, pose frame, calibration and command in the experiment log for every new experiment.

## Current Stage 5 stop point

Stage 5 has completed the visual viewpoint analysis on the frozen Robot1-to-Robot3 protocol. Three visual methods (Single RGB, RGB5 Mean, Cross-Max) reranked the frozen SC Top-20 candidates using OpenCLIP ViT-B-32-quickgelu (laion400m_e32). The candidate-conditioned, end-to-end, heading-stratified, true-positive similarity, rescue/regression, sync-quality and Stage-4 failure analyses are all complete. No fusion, VLM, CVTNet, GICP or PGO was run.

The next authorized research step is viewpoint-aware fusion, which must be a separately authorized Stage 6. It should combine frozen SC candidates with visual evidence under a fixed protocol, explicitly stratified by heading. Do not implement it without explicit authorization.
