# Stage28 interactive LiDAR and RGB-D mapping

Stage28 starts from Stage27 commit `1f7ed7cb5bceb65e47a2e6a2174102c016c4b234`. It uses the new entry point `demo/stage28_multisensor/`. Historical demo code and output trees are outside the Stage28 diff.

## Implementation inventory

- Stage12D supplied the frozen clean 75-loop global LiDAR trajectory and the IMU-to-LiDAR mount. Stage12C supplied EKF observations matched to each original LiDAR keyframe. The server's complete 2 Hz cache supplied both robots' original XYZ/intensity scans and timestamps. The Robot1 raw ZIP was absent; its complete cached keyframes were present and sufficed.
- Stage22A documents logical-time causal retrieval and replay over the clean LiDAR backend. At the Stage27 checkpoint there is no standalone tracked Stage22 demo directory. Stage28 rebuilds the LiDAR presentation from the input scans and pose evidence, without replacing an earlier viewer or labeling the new output as Stage22.
- Stage27 supplied the original sensor timestamps, frozen camera calibration, epoch transformations, trajectories and 1 Hz colored point prefixes. Stage28 reuses all 127 immutable RGB-D chunks and applies a rigid view transform. The original RGB/depth ROS2 data remain in the source archives and Stage26 SQLite caches.
- The renderer follows the Stage27 timestamp-prefix approach and uses the same vendored Three.js and OrbitControls. The new viewer supports six sensor/robot combinations, frame and Epoch jumps, follow, 3D orbit/pan/zoom, top-down XY, robot/source coloring and an optional 2 m ground grid with projected trajectories.

## LiDAR continuity

Both maps play from the first through last cached LiDAR keyframe. The Stage12D stable tail is identical to the frozen solved trajectory. This includes Robot1 KF150–1999 and Robot3 KF234–4179. Before the stable anchor, Stage28 composes original EKF relative increments backwards in the LiDAR gauge. The same measured mounting transform is used throughout. Within each provisional Epoch, local relative motion is preserved. Raw frame changes, timestamp gaps greater than 1 s, rotation increments above 35 degrees and translation increments exceeding min(2.5 m, 4 m/s × dt) open new presentation Epochs. The affected steps are replaced with a bounded local median motion estimate, or zero velocity when no nearby support exists. They are not loop closures.

The two largest raw boundary translations were 298.10 m for Robot1 and 321.48 m for Robot3. These are upstream EKF discontinuities. Connecting those raw poses directly in a viewer would draw a false long segment even with a persistent scene. Stage28 shows 17 Robot1 and 23 Robot3 transitions. The maximum displayed boundary translations are 0.499 m and 1.625 m, respectively. Robot1 KF1199 lacks a Stage12C header-window EKF association and is covered by the exact frozen Stage12D pose. The stable trajectories share the Stage12D gauge. The reconstructed prefixes remain exploratory.

Select approximately 1 Hz and retain every Epoch's initial samples and the final cached keyframe. The exporter uses a point stride of 32, 1–48 m range and a 0.34 m voxel grid per robot. The LiDAR result contains 855,407 Robot1 and 2,090,930 Robot3 points, for 2,946,337 total in 162 timestamped chunks. The local motion estimates and the frozen-tail handoff are recorded in the per-robot continuity tables. Frozen-tail PGO is a per-node pose set, so it has no single rigid transform relative to the raw EKF trajectory.

## RGB-D continuity

The independent views filter Robot1 or Robot3 from the same immutable Stage27 sample stream. The calibrated projection, optical frame, RGB colors, point prefix counts and original frame provenance are inherited. Robot1 has 6 Epochs and Robot3 has 20. The RGB-D overview contains 429,303 points from 3,005 sampled observations. The underlying recording clocks start independently; simultaneous playback does not imply simultaneous sensing. Stage26 exploratory and display-only placements remain flagged. A physical global camera map is still unverified.

## Gravity and physical scale

Points and poses use metric right-handed coordinates. LiDAR uses Stage12D Z up, and RGB-D is rotated from the Stage27 nominal Y-up display into nominal Z-up. No translation or scaling is added by the gravity step. In each 12 s window, RANSAC proposes a plane from near-sensor points 0.2–3.5 m below the camera or LiDAR trajectory. Only planes with sufficient support and spatial spread contribute. The reported normal is the median over usable windows. A rigid roll/pitch correction requires early/late normal disagreement ≤5°, p90 normal spread ≤9° and tilt ≤35°. The renderer applies the same rotation to clouds, markers and paths, preserving slopes and vertical structure.

| Sensor | Robot | Valid windows | Normal p90 spread | Early/late difference | Display orientation |
|---|---|---:|---:|---:|---|
| LiDAR | Robot1 | 10 | 3.11° | 0.55° | Ground-plane correction 1.50° |
| LiDAR | Robot3 | 21 | 6.32° | 3.12° | Ground-plane correction 1.40° |
| RGB-D | Robot1 | 60 | 14.50° | 8.98° | Nominal axis conversion |
| RGB-D | Robot3 | 104 | 15.45° | 11.35° | Nominal axis conversion |

The historical EKF is IMU-informed. The preserved IMU audit includes acceleration and angular-speed magnitudes, which cannot determine a gravity direction alone. Stage28 did not transfer IMU orientation to an optical frame without a calibrated mount. The combined LiDAR view uses one correction for both robots to preserve the existing common gauge. Combined RGB-D likewise uses one axis conversion for both. RGB-D top view is explicitly nominal.

## Camera–LiDAR comparison and fusion

**NO-GO for overlay or projection.** The URDF provides the LiDAR and camera-link mounting geometry. Recorded camera bags provide factory depth-to-color calibration. Together these do not establish the physical camera-link to optical-frame displacement. The per-robot audit retains the actual URDF joints, recorded optical transform and ROS topic inventory.

On original bag timestamps, stable LiDAR brackets cover 924.36 s / 1,972.43 s, with median bracket intervals of about 0.50 s. Translation is interpolated linearly, rotation by SLERP, and brackets wider than 0.65 s are excluded. The motion experiment fits `A X = X B` on original within-Epoch relative motions and evaluates distinct held-out time windows. At zero offset the holdout median translation/rotation errors are 3.70 m / 7.85° for Robot1 and 5.12 m / 5.29° for Robot3. A −100 to +100 ms clock-offset sensitivity check leaves metre-scale errors. The experiment uses visual and optimized LiDAR trajectories in their native gauges and cannot isolate mount error from pose drift or deformation. No candidate extrinsic was used for mapping.

## Open the demo and reproduce

From a repository with the generated assets installed:

```sh
python -m http.server 8031 --bind 127.0.0.1 --directory demo
```

Open `http://127.0.0.1:8031/stage28_multisensor/`. The six links select each map. Click **End** to see the accumulated map; **Top** is XY and **3D** restores perspective. Odd or uncertain graph transitions remain annotated in the selector and map. Drag to orbit, wheel to zoom, and right-drag to pan. Playback defaults to 10 recorded seconds per demo second. Quarter density reduces rendered points while preserving chunk visibility and timeline state.

On the server, run these from the checkout with Stage12C/D and Stage27 files present. The Stage27 RGB-D chunks are required in their original path.

```sh
export OPENBLAS_NUM_THREADS=1
py=/home/cas/.venvs/fyp_stage24/bin/python
"$py" src/cumulti/rgbd_full/stage28_lidar.py --cache /home/cas/CU-Multi/processed_v1/full_2hz --limit 12
"$py" src/cumulti/rgbd_full/stage28_lidar.py --cache /home/cas/CU-Multi/processed_v1/full_2hz
"$py" src/cumulti/rgbd_full/stage28_gravity.py
"$py" src/cumulti/rgbd_full/stage28_calibration.py --scratch /home/cas/fyp_stage26/work
"$py" src/cumulti/rgbd_full/stage28_validate_assets.py
python src/cumulti/rgbd_full/stage28_validate_browser.py http://127.0.0.1:8031/stage28_multisensor/ outputs/cumulti_v1/28_multisensor/screenshots
```

The browser report checks every boundary, counts retained chunk objects, exercises the camera and seek controls, verifies four complete independent replays and captures six final views, LiDAR BEVs and representative boundary screenshots. The asset audit verifies SHA256, finite data, rigid rotations, timestamps and all frozen LiDAR tail poses. The six generated combined/individual PLYs stay on the experiment server under `/home/cas/fyp_stage26/outputs/cumulti_v1/28_multisensor/`. Their hashes and sizes are in `gravity_alignment.json`. Runtime LiDAR and RGB-D chunks are installed in the working demo and excluded from Git.
