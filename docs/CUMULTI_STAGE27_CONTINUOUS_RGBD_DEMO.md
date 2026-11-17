# Stage27 continuous full-recording RGB-D replay

The existing Stage24 RGB-D demo opens the Stage27 continuous view by default. The Verified Atlas and Raw Epoch links retain the original scientific views. Stage27 is an exploratory display in one persistent coordinate frame. It does not qualify global RGB-D SLAM accuracy, alter Stage24D transforms or admit the Stage26 proposals to the validated graph.

## Open the demo

With the generated assets installed, run this from the repository root.

```sh
python -m http.server 8031 --bind 127.0.0.1 --directory demo
```

Open http://127.0.0.1:8031/stage24_full_rgbd/. Left drag orbits, the wheel zooms and right drag pans. Top/3D reframe the visible trajectories, Follow moves the viewing camera smoothly towards the chosen robot, and End fits the full scene. Quarter density keeps every loaded chunk while rendering one point in four. Backward seek hides only future contributions. Forward playback and seeking share the same timestamp-indexed chunk buffers.

## Display frame and provenance

The first Robot1 camera defines the gauge, with Y displayed upwards relative to that camera. The two recordings use independent start times. Stage24D relative transforms take priority. For a disconnected component, Stage26 aggregate depth-passing candidates are considered as exploratory estimates, excluding the cycle-inconsistent candidate. The Stage24E independently held-out aggregate is a relaxed fallback. The exploratory estimate must be within 1.5 m and 20 degrees of the bounded motion prediction to be selected for visualization. These are display screening limits. No external pose stream is used because verified complete camera-to-LiDAR mounting calibration was unavailable.

Constant-motion extrapolation uses the last approximately 0.8 s of an Epoch when available, at most one second of extrapolation, and caps the increment at 0.8 m / 0.2 rad. Short Epochs can inherit the previous terminal pose. Local relative poses are retained by rigid composition. Estimated connectors are dashed and carry their provenance; no poses are synthesized within a tracking gap.

Rigid Stage24D placement would create a 38.474 m / 48.59 degree jump when Robot3 returned to its earlier verified component at Epoch 19. That component relationship is retained in the frozen graph. Stage27 explicitly omits it from display placement and uses a 0.171 m / 0.31 degree extrapolated transition. All 26 Epochs have finite display transforms. The 24 boundaries use 12 display-only extrapolations, 10 Stage26 exploratory registrations, one verified relative transition, and one re-entry through an existing Stage24D component. The largest displayed boundary translation and orientation increments are 0.800 m and 23.38 degrees.

## Map export

Recover the selected frames from the four CRC-checked Stage26 archive expansions. The exporter verifies the original topic, ROS message IDs, bag and header timestamps, optical frame and Stage24 per-frame depth coverage. Depth is aligned into the color frame by the same frozen projection. Sample the first valid RGB-D frame per recorded second per Epoch and include the last valid RGB-D frame of each Epoch. Back-project calibrated depth, color from the corresponding RGB image, and retain the first point in each 0.15 m voxel per robot. Chunk files hold up to 30 recorded seconds and store prefix counts for exact frame visibility. No future poses or points enter playback before their recorded time.

The run covers 2,098.463 s with 949 Robot1 and 2,056 Robot3 samples. The final map contains 429,303 points in 127 chunks. Finite native camera poses are exported at their original cadence. Tracking gaps can remain visible as dropped measurements and drift, and the two portions of the same physical structure may be geometrically misaligned. The combined PLY is exploratory.

## Reproduce and validate

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
/home/cas/.venvs/fyp_stage24/bin/python src/cumulti/rgbd_full/stage27_continuous.py --scratch /home/cas/fyp_stage26/work
python src/cumulti/rgbd_full/stage27_validate_browser.py 'http://127.0.0.1:8031/stage24_full_rgbd/continuous.html?debug=1' outputs/cumulti_v1/27_continuous_rgbd/screenshots
```

The exporter uses frozen Stage24 pose, synchronization, camera calibration and coverage tables, the Stage24D pose graph, Stage24E aggregate records and the Stage26 registration tables. SQLite copies must have been produced by a successful archive expansion. The test requires Playwright and Microsoft Edge (or pass another installed Chromium channel). It checks all 24 boundaries in the actual WebGL scene, retaining existing chunk UUIDs and point counts, then tests seek, rewind, controls, paired robot visibility and the final unbroken 4× run. Browser evidence and screenshots are in the output folder. The binary presentation assets and final PLY are generated artifacts, installed in the working demo and stored on the experiment server.
