# Camera-only RGB-D local mapping prototype

Serve the **parent `demo/` directory** and open `/rgbd_camera_mapping/`.
For this Windows workspace:

```powershell
C:/Python314/python.exe -m http.server 8021 --bind 127.0.0.1 --directory E:/university/uic/FYP/demo
```

Then visit http://127.0.0.1:8021/rgbd_camera_mapping/ . Do not double-click
`index.html`: ES modules / fetch require HTTP. This uses the existing pinned
Three.js / OrbitControls distribution under `../progressive_multi_robot/vendor/`.

Both panels are independent **camera RGB-D local maps**. They use the original
Robot1 / Robot3 camera recordings, actual camera calibration, and new native
ORB-SLAM3 RGB-D tracking. No LiDAR, frozen LiDAR trajectory, IMU, GNSS, GT,
semantic predictions, or historical inter-robot factors supply the poses/map.
The first frame of each recording defines its own camera optical gauge.
There is no common world frame, visual inter-robot loop, or merged map here.
Side-by-side playback does not mean the robots were recorded simultaneously.

Controls: Play/Pause, Reset, Seek, speed, final local maps; independent left-drag
orbit, right-drag pan, wheel zoom, fit and point/trajectory visibility controls.
Color is measured image color, not robot provenance coloring or semantic labels.
Depth preview is an explicitly synthetic color scale of **measured depth**
(0–10m display range); black is missing depth, not measured free space.

## Scope and reconstruction

- Fixed accepted camera-pilot window: elapsed 120–132 seconds, 60 frames / 5 Hz
  per robot. These are independent camera timelines, not LiDAR keyframe IDs.
- Reuse original camera-derived 640×400 JPEGs from Stage24's camera export.
  No additional JPEG recompression for RGB.
- Decode original camera depth PointCloud2 using embedded ROS2 definitions.
  Read field offsets, row strides, endianness and header frame IDs.
- Transform depth-optical XYZ using recorded `depth_to_color` (column-major
  rotation, metres), project using color K/D at the actual resized resolution.
- Keep nearest color-camera Z per pixel. Missing pixels stay zero: no hole
  filling, learned depth, LiDAR completion or image interpolation.
- Engineering validity range 0.3–10m in both optical frames; 65.535m saturation
  and nonfinite / rear / out-of-FoV points are excluded. This range is not a
  sensor accuracy claim. Bag-time association cutoff 25ms; original bag and
  exposure-header timestamps plus both deltas are retained.
- Native ORB-SLAM3 `RGBD`, viewer off, no inertial mode. PNG depth is uint16 mm,
  `RGBD.DepthMapFactor=1000`. Actual camera K/D; frozen ORB example feature
  settings. RGB-D's virtual pseudo-stereo baseline is predeclared 0.05m and
  close/far multiplier 40, not a measured physical IR baseline. No GT tuning.
- Every second tracked frame contributes depth pixels at stride 4 in both
  image dimensions. Undistort rays, multiply by measured Z, apply that recorded
  `TrackRGBD` camera-to-local pose. 5cm voxels, first observation retained,
  at most 100,000 display vertices per local map.
- The browser caches immutable point buffers and reveals only the prefix
  through the selected frame. It does not interpolate poses or geometry.
  Camera path breaks at tracking failures. A separate map is retained after
  every tracking gap, conservatively even if relocalization might recover the
  previous frame. ORB's map ID alone is insufficient because active resets can
  reuse it. Only one epoch is visible per robot; reset maps are never glued.
- Native inputs are paced to original relative camera timestamps. The first
  unpaced Robot1 engineering run and the subsequent paced result are both kept.
  Pacing did not eliminate its reset. The predeclared full-window tracking
  acceptance gate remains unchanged; a functional partial demo is not a pass
  of that gate. Tracking loss / frozen maps / reset epochs are visibly labeled.
- Earlier dense observations are **not** retrospectively reprojected after
  local BA. `final_trajectory.tum` is diagnostic only; the demo uses recorded
  frame poses. Ghosting is therefore possible; do not claim globally optimized
  dense reconstruction or full-sequence accuracy.

## Reproduce on the server

From `/home/cas/fyp_place_recognition`, with the existing isolated semantic/
camera Python environment and separately installed ORB-SLAM3 SDK:

```bash
bash scripts/run_rgbd_demo.sh
```

The runner writes only `outputs/cumulti_v1/24_rgbd_demo_v1/` and this new demo.
Completed export/tracking reports are reused, not silently rerun. Existing
Stage24 camera exports are prerequisites, with provenance in their manifest.
Native ORB-SLAM3 is multithreaded; a fresh solve is not claimed to be bitwise
identical. Accepted poses, display geometry and images have explicit hashes.
Large temporary complete SQLite copies are required for indexed access to
the zipped bags, CRC-checked to EOF and removed after successful export.
Do not run two exports for the same robot concurrently.
Original archives, binary SDK, vocabulary, raw/expanded bags, depth sequences,
native binaries, tracing logs and caches stay server-only.

`data.json`, the output map manifest and the independent validation report
record actual results. A successful small-window demo does not demonstrate
full-run robustness, hardware exposure synchronization or inter-robot fusion.
