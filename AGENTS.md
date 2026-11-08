# FYP repository instructions

1. Read PROJECT_CONTEXT.md, RESULTS.md, EXPERIMENT_LOG.md, then NEXT_STEPS.md before changing the project.
2. Inspect existing code before changing architecture, metrics or data conventions.
3. Preserve reproducibility and never overwrite historical outputs; use a new named output directory.
4. Do not commit raw datasets, model checkpoints, credentials, API keys, caches or large generated files.
5. Ground truth is only for synchronization, overlap definition, analysis and evaluation. Never select or rerank candidates with it.
6. Do not change Scan Context, OpenCLIP, Cross-Max, VLM or GICP core algorithms without an explicit isolated experiment.
7. Update EXPERIMENT_LOG.md after meaningful experiments, RESULTS.md for verified metrics, and NEXT_STEPS.md for direction changes.

## Bounded camera RGB-D local-map demo (2026-11-08)

PASS_FUNCTIONAL_DEMO_TRACKING_PARTIAL. PARTIAL_PREDECLARED_GATE_NOT_MET. New independent camera-only maps, original depth PointCloud2 adaptation, timestamp-aligned RGB and native paced ORB-SLAM3 RGBD. No LiDAR/GT/IMU input or semantic map; reset epochs never merged. No cross-robot visual fusion or full-sequence run. See docs/CUMULTI_RGBD_CAMERA_MAPPING_DEMO.md and demo/rgbd_camera_mapping/README.md. Historical Stage24A/B blocked reports are preserved; Robot3 depth arrival resolves availability, not missing LiDAR-optical calibration.

Verified short-window tracking: robot1 44/60 valid frames, 2 separate tracking epoch(s), 76260 display points across its epochs; robot3 60/60 valid frames, 1 separate tracking epoch(s), 100000 display points across its epochs. No joint camera map has been established.
