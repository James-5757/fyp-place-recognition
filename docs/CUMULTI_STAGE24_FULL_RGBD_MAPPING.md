# Full-duration camera-only RGB-D mapping

Processing / segmented-demo status: **PASS_SEGMENTED_FULL_AVAILABLE_RECORDINGS**. Rendering acceptance and uninterrupted tracking are separate qualifications.

## Full source audit

| Robot | RGB frames | Depth clouds | RGB duration (s) | Paired RGB fraction |
|---|---:|---:|---:|---:|
| robot1 | 9997 | 8765 | 999.789160747 | 0.876763 |
| robot3 | 20977 | 19918 | 2098.463330423 | 0.949516 |

Full archive CRC and all CameraInfo/extrinsics messages were checked. Exact original bag/header timestamps, gaps >150 ms, frequencies and nearest-depth differences are in `full_data_audit.csv`, `timestamp_gaps.csv` and per-robot `sync_pairs.csv`. RGB/depth association uses bag time within 25 ms; this does not certify hardware exposure synchronization.

Robot1 does not contain 1972.5 seconds of camera recording. The presentation retains both complete available streams, ending at the longer camera duration; Robot1 explicitly freezes as RECORDING COMPLETE. The Stage20-time jump is a reference, not manufactured coverage.

## Tracking and resets

| Robot | Valid / total | Valid ratio | Map epochs | Native wall (s) | Peak RSS (MiB) |
|---|---:|---:|---:|---:|---:|
| robot1 | 9864/9997 | 0.986696 | 6 | 1004.565 | 3010.7734375 |
| robot3 | 20494/20977 | 0.976975 | 20 | 2110.655 | 6957.453125 |

Each robot ran independently from its actual recording start at the original RGB message rate, paced by original bag-time differences. The baseline is 1000 native ORB features, established 640×400 scaled color calibration, original distortion coefficients, actual factory depth-to-color extrinsics and the frozen .3–10 m projection range. Missing depth is zero/unknown, never completed.

Valid poses require native tracking state OK, finite SE(3), and a dominant non-bad MapPoint map vote of at least 0.8. Map ID plus Map.GetInitKFid identifies observed native gauge changes. Every observed transition starts a new conservative Map Epoch, including active-map resets that reuse a map ID. Invalid poses stay blank. See per-frame poses, reset events and loss intervals.

The native library/SDK were not modified. Source, binary, vocabulary and native library hashes are recorded. Native file-access traces are checked for forbidden sensor/GT paths. Native asynchronous mapping is not claimed bitwise deterministic.

## Dense map and presentation

RGB-colored points are back-projected with undistorted color-camera rays and measured aligned depth, transformed by their recorded online camera Twc. Mapping samples 1 Hz with pixel stride 8 and 0.1 m first-observation voxels; browser LOD uniformly spans every epoch. Points retain first-frame IDs, and the renderer reveals only points available at the selected original frame. Each epoch has its own binary PLY and hash. No cross-epoch pose glue is introduced.

The optical gauge is not gravity-aligned. Dense samples are not retrospectively reprojected after later bundle adjustment; this limits long-sequence map consistency. This is not a new optimized dense SLAM backend.

The demo preserves two independently clocked RGB-D local-map panels. It includes 10:1 compression, 0.5/1/2/4 speed, play/pause/reset/seek, RGB previews at 2 Hz, loss markers, epoch selection and 3D orbit/pan/zoom. RGB prefetch is bounded; map GPU resources use a two-epoch cache per robot and dispose evicted buffers. A shorter stream ends visibly.

## Conditional bridge study

Decision: **NO_RELIABLE_RGBD_BRIDGE_ESTABLISHED**. A bounded offline RGB-only BoW search tested 576 proposals; 0 passed single-pair metric RGB-D geometry.

The protocol was fixed in source before the full tracking results. A bridge requires multiple distinct RGB-D pairs with consistent metric transforms in the same two epochs, sufficient temporal span and bidirectional reprojection checks. Search uses no LiDAR, old Scan Context loops, GT, IMU or GNSS. The bounded search does not prove absence of a common place. Verified pairs, if any, apply only to those epochs and do not imply full-recording fusion.

## Reproduction and assets

From repository root: `bash scripts/run_rgbd_full.sh`; post-tracking: `python src/cumulti/rgbd_full/finalize.py`. Use `/home/cas/.venvs/fyp_stage24/bin/python`. Raw data and native SDK are server dependencies; no redownload is required. Serve the whole `demo/` directory and open `/stage24_full_rgbd/` over HTTP.

Source, audit, lightweight results and asset manifests belong in Git. Full preview image sequences, raw databases, native binaries and full PLY files remain server-only. `presentation_rgb_manifest.json` and `map_artifact_manifest.json` record sizes and hashes. Historical 12 s / 120 s experiments, Stage20, P5/P6 and the SLAM research baselines are preserved.

## Validation

- robot1_FULL_NATIVE_FRAME_COUNT: PASS
- robot1_NO_MISSING_OR_DUPLICATE_IDS: PASS
- robot1_EXACT_ORIGINAL_TIMESTAMPS: PASS
- robot1_ACTUAL_NATIVE_SOURCE_AND_BINARY_HASHES: PASS
- robot1_BASELINE_FEATURES_UNCHANGED: PASS
- robot1_MONOTONIC_ORIGINAL_BAG_TIMESTAMPS: PASS
- robot1_INVALID_POSES_NOT_FABRICATED: PASS
- robot1_SEPARATE_EPOCHS: PASS
- robot1_REAL_ORDERED_POINT_GROWTH: PASS
- robot1_RGB_PREVIEW_ORIGINAL_TIMES: PASS
- robot1_ASSET_HASHES: PASS
- robot3_FULL_NATIVE_FRAME_COUNT: PASS
- robot3_NO_MISSING_OR_DUPLICATE_IDS: PASS
- robot3_EXACT_ORIGINAL_TIMESTAMPS: PASS
- robot3_ACTUAL_NATIVE_SOURCE_AND_BINARY_HASHES: PASS
- robot3_BASELINE_FEATURES_UNCHANGED: PASS
- robot3_MONOTONIC_ORIGINAL_BAG_TIMESTAMPS: PASS
- robot3_INVALID_POSES_NOT_FABRICATED: PASS
- robot3_SEPARATE_EPOCHS: PASS
- robot3_REAL_ORDERED_POINT_GROWTH: PASS
- robot3_RGB_PREVIEW_ORIGINAL_TIMES: PASS
- robot3_ASSET_HASHES: PASS
- NO_INTER_EPOCH_GLUING: PASS
- CAMERA_ONLY_TRACKING: PASS
- HISTORICAL_RESULTS_NOT_REUSED_AS_FULL_RUN: PASS
- Browser: PASS

Measured browser loading: 0.976 s; software-WebGL idle rendering: 27.67 FPS. These measurements are machine-specific, not a real-time SLAM claim.

## Coverage and tracking are distinct

- robot1: time-weighted valid ratio 0.986689; 1205 native-valid poses on RGB frames without a paired depth cloud. Those frames add no measured dense points.
- robot3: time-weighted valid ratio 0.977228; 1012 native-valid poses on RGB frames without a paired depth cloud. Those frames add no measured dense points.

A native-valid pose does not imply that a new depth measurement exists at that frame. The saved per-frame state and the actual paired-depth flag are preserved separately; missing depth remains unknown. No missing depth or missing camera pose is fabricated.

## Adjacent-pose integrity diagnostics

Trajectory line rendering breaks at tracking losses, epoch boundaries and abrupt adjacent changes (>2 m, >10 m/s or >45°). This is a visualization safeguard, not a new pose estimate or an artificial map reset. All original recorded poses and map samples remain available.

- robot1: 1 flagged adjacent changes within valid same-epoch pose pairs.
- robot3: 4 flagged adjacent changes within valid same-epoch pose pairs.

Fingerprint transitions can include first-keyframe bookkeeping. Map Epochs are a conservative partition; not every observed transition proves a native origin reset. Native Map ID changes, observed fingerprint changes and active-reset log counts are reported separately.

Native scheduling follows original camera timestamps but waits for missing prepared depth files; startup/preprocessing waits are included in measured wall time. Input timestamp frequency must not be confused with achieved wall-clock throughput.

RGB labels distinguish the current tracking input from the actual displayed preview. Each preview retains its own original RGB bag/header timestamps, message ID, paired depth timestamps and synchronization error; the displayed preview age is explicit.
