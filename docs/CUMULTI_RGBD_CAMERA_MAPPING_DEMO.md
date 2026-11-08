# Camera-only RGB-D mapping demo — bounded prototype

Date: 2026-11-08. User authorized a separate RGB-D map demo; semantic mapping is deferred.

Functional/artifact validation: **PASS_FUNCTIONAL_DEMO_TRACKING_PARTIAL**. Full-window tracking: **PARTIAL_PREDECLARED_GATE_NOT_MET**.

No LiDAR-to-camera extrinsic is needed for this route: depth-optical XYZ is transformed with the actual depth_to_color calibration, then paired with original color images and new RGB-D camera poses. This does not unblock coloring the old LiDAR P5 map.

| Robot | Tracked / input | Continuous max | Tracking epochs | RGB map points across separate epochs | Median / p95 bag sync ms | Native wall s | Track median / p95 ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| robot1 | 44 / 60 | 41 | 2 | 76260 | 1.511121 / 3.742267 | 18.357414 | 17.197031 / 21.952997 |
| robot3 | 60 / 60 | 60 | 1 | 100000 | 1.340864 / 3.621123 | 20.271963 | 21.239178 / 26.609902 |

## Tracking integrity

The first unpaced Robot1 engineering run tracked 44/60 frames and reset its active map. Its report remains at robot1/tracking.json. Pacing did not eliminate the Robot1 loss; its paced report remains PARTIAL, with no threshold relaxation or GT-based tuning. Active-map ID 0 was reused across reset. The paced implementation preserves sensor frame intervals and explicitly records tracking epochs. Map ID alone is not evidence of a shared frame. Every post-gap epoch is conservatively isolated; the browser never connects or overlays reset maps. A functional demo is not a full-window SLAM success.

The fixed acceptance gate is >=45/60 valid poses, >=40 continuous frames, one map and tracking epoch, finite/unit poses. Parameter settings are recorded in each paced/tracking.json. There were no additional window searches or optimizer/feature retuning to force this gate.

## Adapter and geometry

The actual ZIP DBs are CRC-checked at EOF, opened read-only, and decoded using embedded ROS2 definitions. The input is unorganized camera PointCloud2, not native depth Image. Actual field offsets, strides, endian, depth optical frame, camera K/D and column-major depth_to_color are used. Real RGB is the accepted fixed camera-pilot JPEG export (640x400, 5Hz, camera elapsed 120–132s); no new camera/LiDAR sync grid is introduced.

robot1: depth valid coverage median 53.183%; maximum RGB/depth header delta 0.000000ms. This does not prove a LiDAR hardware clock calibration.

robot3: depth valid coverage median 49.265%; maximum RGB/depth header delta 0.000000ms. This does not prove a LiDAR hardware clock calibration.

Depth-optical and color Z are gated at 0.3–10m, saturation/NaN/rear/FoV failures discarded. Closest color Z wins per pixel; holes remain zero. PNG is uint16 mm with native DepthMapFactor=1000. RGB-D pseudo-stereo baseline 0.05m is an explicit virtual algorithm parameter, not an inferred IR baseline; close/far multiplier 40. Depth images are backprojected through undistorted color rays and the corresponding recorded TrackRGBD Twc. Dense-map policy: every second valid frame (plus first of a submap), pixel stride 4x4, 5cm voxels, retain first observation, cap100000 vertices per submap.

Synthetic direction/occlusion/saturation/NaN tests and actual image/depth/buffer/pose/timestamp/hash checks are independent of GT. Projection/backprojection roundtrip tests numerical self-consistency, not physical calibration accuracy. The alignment overlays are qualitative, not ground-truth accuracy measurements. Original archives are unchanged; temporary DB copies are removed after successful export.

## Demo, limitations and reproduction

Open `http://127.0.0.1:8021/rgbd_camera_mapping/` after serving the whole demo parent. Native pipeline: `bash scripts/run_rgbd_demo.sh`. Camera exports and native SDK are server prerequisites. Repeated default invocation reuses completed phases. Browser validation command and controls are in demo/rgbd_camera_mapping/README.md.

Browser checks: PASS; startup 1452.362ms, measured rendering 8.652fps on headless software WebGL. Play/pause/reset/seek/final, actual left orbit/right pan/wheel zoom, image decode, reset-epoch separation and cached buffers are checked. Software rendering remains a performance limitation, not a hardware frame-rate or real-time SLAM claim.

Both panels remain independent local optical gauges. Replay is offline; separate runs are not simultaneous physical robots. RGB is measured image color; preview pseudocolor represents measured depth, not semantic inference. No camera-only inter-robot match/factor/common frame is established. Do not label this a fused global camera map. Earlier dense samples are not reprojected after local BA; ghosting, depth holes and tracking loss remain. The 100000-point presentation budget is reached by Robot3: the UI marks DISPLAY CAP, later camera poses/images still advance but additional map vertices are omitted. No full-run robustness or GT accuracy evaluation, semantic inference, visual cross-robot fusion, or live deployment was performed.

Frozen LiDAR/SC/GICP/P0/P5/P6/Stage18–23E results and Stage24A/B accepted reports are untouched. The previous RGB/semantic LiDAR-projection blocker remains valid. Next step: diagnose RGB-D tracking loss and validate longer camera-only continuity before proposing camera-based multi-robot map fusion.
