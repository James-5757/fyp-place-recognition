# Native-rate RGB-D long-window tracking and reset-management audit

Run started: 2026-11-08; finalization: 2026-11-09 (Asia/Shanghai). Accepted short-demo parent: `88dfda9c5544b1e2e7f5d1e6c02619094f6f4ebe`.

Audit: **PASS_AUDIT**. Pre-fusion stability decision: **NO_GO_RGBD_FUSION**. Selected shared configuration: **None**.

## Fixed experiment and gate

Both independent camera timelines use elapsed [120,240)s, every original RGB frame at nominal10Hz, with original IDs/header/bag timestamps. This extends the accepted 12s pilot; it is a120s qualification, not a complete1000s/2099s run. Native timestamps pace replay. Missing associated depth is zero/unknown; RGB frames are retained, without an identity pose or depth fabrication.

Before native long tracking, policy.json fixed >=95%valid frames and time-weighted tracking, >=90s continuous tracking, no native coordinate-frame changes/resets, <=1s lost interval, finite/unit poses and no forbidden input reads. Final native corrected poses must cover >=95%input frames and are filtered to actual online state2 tracked frames. integrity_policy.json additionally rejects adjacent valid same-frame jumps >2m, >10m/s or >45degrees, using conservative engineering limits, not GT.

Baseline1000 uses the original1000 ORB features; if either robot fails, fallback2000 is evaluated on both, changing only feature capacity. Fixed optics, depth registration/range, scale pyramid1.2/8, FAST20/7, virtual RGB-D baseline0.05m and close/far40 are retained. Lowest shared passing configuration wins; no further searches or gate relaxation. Correct nominal Camera.fps10 accompanies the denser input, so the old5Hz comparison is not a controlled proof that rate alone causes differences.

## Measured native results

| Robot | Config | Tracked/input | Valid % | Time-weighted % | Longest s | Longest loss s | Frame changes / native resets | Corrected tracked poses | Median/p95 tracking ms | Gate |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| robot1 | baseline1000 | 1200/1200 | 100.000 | 100.000 | 119.904 | 0.000 | 0 / 0 | 1200 | 21.483/30.983 | PASS |
| robot1 | fallback2000 | 1136/1200 | 94.667 | 94.655 | 57.803 | 3.116 | 4 / 2 | 0 | 32.068/50.299 | FAIL |
| robot3 | baseline1000 | 1168/1200 | 97.333 | 97.407 | 91.205 | 3.110 | 1 / 0 | 0 | 22.860/32.913 | FAIL |
| robot3 | fallback2000 | 1137/1200 | 94.750 | 94.741 | 83.999 | 3.207 | 2 / 1 | 0 | 31.627/43.808 | FAIL |

Native wall time includes vocabulary loading, real-time pacing, traced file I/O and shutdown; do not treat it as unpaced compute throughput. No real-time deployment claim is made. Exact runtimes, peak memory and all failed checks are recorded in metrics.json.

robot1 baseline1000 failed checks: none. First12s: 121/121 valid, compared with the separately preserved5Hz short experiment.

robot1 fallback2000 failed checks: valid_fraction, time_weighted_fraction, longest_continuous, coordinate_frame_changes, max_untracked_gap, final_native_trajectory_coverage. First12s: 121/121 valid, compared with the separately preserved5Hz short experiment.

robot3 baseline1000 failed checks: coordinate_frame_changes, max_untracked_gap, final_native_trajectory_coverage. First12s: 121/121 valid, compared with the separately preserved5Hz short experiment.

robot3 fallback2000 failed checks: valid_fraction, time_weighted_fraction, longest_continuous, coordinate_frame_changes, max_untracked_gap, final_native_trajectory_coverage. First12s: 90/121 valid, compared with the separately preserved5Hz short experiment.

## Actual reset identification

Regression: **PASS_REAL_MAP_ID_REUSE_RESET_DETECTED**. Same immutable60 Robot1 inputs, 44 valid poses, 1 native reset log event. Observed fingerprints: [[0, 0], [0, 3]]. The old map ID was reused, while initial keyframe ID changed. Full events and time are in reset_regression/validation.json.

SDK inspection confirms Map::AddKeyFrame assigns a new initial-KF ID when the emptied map is repopulated; ResetActiveMap clears the map without resetting the global next-keyframe counter. The observer votes tracked nonbad MapPoints to obtain map ID and initial-KF fingerprint. Gaps with an unchanged fingerprint are tracking gaps, not automatically coordinate resets. A changed fingerprint forbids odometry/path/map stitching. Synthetic gap/reused-ID tests and the actual native log independently cross-check this distinction. Native core source was not patched.

Native final-BA export uses the explicit single-map SaveTrajectoryEuRoC overload only when exactly one frame was witnessed. Raw final exports can contain predicted recently-lost frames; corrected_camera_trajectory.csv deliberately retains only timestamps independently observed as online valid state2. Multiple/reset gauges are not exported as one canonical corrected trajectory. No missing poses are interpolated.

## Sensor adaptation

robot1: 1200 RGB, 1200 paired depth, 0 explicit missing depth; paired bag delta median/p95=1.330753/3.995225ms; maximum paired header delta=0.000000ms.

robot3: 1200 RGB, 1200 paired depth, 0 explicit missing depth; paired bag delta median/p95=1.124486/4.674309ms; maximum paired header delta=0.000000ms.

All large bag DB members were decoded after complete EOF/CRC validation, opened read-only and removed after this run’s exports. Actual embedded ROS2 definitions/field strides/endian, depth optical frame and column-major depth_to_color are used. Projection remains nearest color Z per pixel, no filling, 0.3–10m engineering range, uint16mm/DepthMapFactor1000 and actual scaled color K/D. No LiDAR optical mounting extrinsic is needed for independent RGB-D maps. This does not resolve the separate LiDAR RGB/semantic projection blocker. Header matching is not a proof of hardware exposure calibration. The overlapping old60 JPEGs are hash-compared in synchronization_report.csv; no GT/local-LiDAR trajectories define the new camera grid.

## Fusion disposition and limitations

Input/loss-window forensics: **NO_MISSING_RGBD_PAIRS_OR_OBSERVED_HEADER_MISMATCH_IN_THIS_COHORT; native visual tracking/map lifecycle remains limiting**. All2400 input RGB frames in this cohort have paired depth and zero RGB/depth header delta; no missing-frame explanation is supported here. The old overlapping60 RGB AND depth images per robot are byte-identical to this new export. Camera cadence, depth coverage, Laplacian/brightness and independent OpenCV ORB-count proxies are recorded for every input frame, with every lost interval and contact sheet.

robot1 baseline1000: 0 lost interval(s), 0 new-map creation(s), 0 active reset(s); fingerprint/log agreement=True. Maximum recorded RGB gap 137.485ms.

robot1 fallback2000: 4 lost interval(s), 2 new-map creation(s), 2 active reset(s); fingerprint/log agreement=True. Maximum recorded RGB gap 137.485ms.

robot3 baseline1000: 1 lost interval(s), 1 new-map creation(s), 0 active reset(s); fingerprint/log agreement=True. Maximum recorded RGB gap 145.739ms.

robot3 fallback2000: 2 lost interval(s), 1 new-map creation(s), 1 active reset(s); fingerprint/log agreement=True. Maximum recorded RGB gap 145.739ms.

These are descriptive diagnostics, not proof that blur, depth coverage or a particular frontend component caused the failures. Depth/registration errors, appearance and native association/mapping sensitivity are not ruled out. Increasing feature capacity did not cure this fixed cohort; do not infer universal superiority of1000features.

Qualitative Robot3 baseline loss-window inspection shows strong magenta/yellow casts, bright ground/shadow boundaries and viewpoint change. The independent image-only ORB proxy still detects about its configured capacity; raw keypoint availability alone does not explain native tracking failure. Per-frame ORB keypoint depth-support fractions are included to distinguish image texture from valid metric feature support, without claiming a causal diagnosis.

**Dual-robot fusion: NOT RUN.** Neither predeclared shared configuration passed both robots. No camera match, common-frame transform, cross-robot factor or fused global map is claimed. This is a tracking/continuity limitation, not a request to download more sensor data. The existing short demo remains functional and frozen.

Visual tracking state/finite poses alone do not prove absolute accuracy or drift bounds. Local optical z is not terrain elevation; diagnostic plots do not connect lost/reset intervals. No GT ATE, semantic inference, LiDAR input, IMU/GNSS, full-run robustness, universal solver claims, or live dual-robot operation. Frozen historical KITTI, SC/GICP, P0/P5/P6 and Stage18–24 short results are untouched.

## Reproduction

From repository root on the server: `bash scripts/run_rgbd_long.sh`. The existing camera Python environment, separate ORB-SLAM3 SDK/vocabulary, original archives and server-only processed camera frames are prerequisites. The regression and overlap hash checks also reference the accepted short-pilot processed frames. Completed phases are reused; there are no hidden required scripts. The final verification is a cached server resume/analysis check, not a second fresh raw-to-native experiment. A Git-only clone can read the lightweight results but cannot execute image diagnostics without those separately supplied/processed datasets and frames. A fresh native multithreaded solve may not be bitwise identical; hashes identify the accepted source, settings, inputs and captured results. Full raw/expanded bags, native executables, vocabulary, traces and frame sequences are intentionally server-only.
