# Full available camera-only RGB-D replay

This is a new independent experiment and presentation. It does not replace Stage20, P5/P6, the 12-second RGB-D prototype or the 120-second tracking qualification.

## Recording duration

The entire available camera recording is used, starting at each camera's original first RGB message. Robot1 contains approximately 999.8 seconds; Robot3 approximately 2098.5 seconds. Their starts are independently normalized to zero for presentation; the demo does not establish simultaneous physical recording.

Robot1 cannot supply 1972.5 seconds of camera observations. It explicitly ends while Robot3 continues. The Stage20-time jump is only a comparison marker. Playback uses 10 recorded seconds per demo second, multiplied by 0.5× / 1× / 2× / 4×.

## Scientific scope

- Tracking uses every original RGB message (~10 Hz), measured camera depth, actual calibration and the established 1000-feature configuration.
- Unknown/unpaired depth remains zero. Missing or invalid camera poses remain absent.
- Each observed native map fingerprint change starts a separate Map Epoch. Select a previous epoch to inspect it alone; epochs are never overlaid or glued.
- Dense maps use actual RGB-D samples at 1 Hz, pixel stride 8 and 0.1 m voxels. Point-buffer first-frame IDs control true progressive growth. Uniform display LOD spans the entire epoch.
- The camera gauge is not gravity-aligned. Camera poses are recorded online outputs; older dense points are not retrospectively reprojected after later bundle adjustment.
- Trajectory lines break across tracking loss, epoch transitions and flagged abrupt adjacent pose changes (>2 m, >10 m/s or >45°). This only controls line rendering; original poses and map points are retained.
- RGB previews are compressed 320×200 JPEG at 2 Hz. This presentation rate is separate from native tracking and the original camera recording rate.
- RGB-only bridge proposals and metric geometric verification are offline diagnostics. A verified epoch-pair transform never authorizes gluing unrelated epochs or claiming a complete global map.
- No LiDAR, IMU, GNSS, GT, semantic reconstruction or simultaneous live-robot operation is used or implied.

## Run on the research server

From `/home/cas/fyp_place_recognition`:

```bash
bash scripts/run_rgbd_full.sh
/home/cas/.venvs/fyp_stage24/bin/python src/cumulti/rgbd_full/finalize.py
/home/cas/.venvs/fyp_stage24/bin/python src/cumulti/rgbd_full/status.py
```

The full runner audits all four original archives and all calibration messages before tracking. Tracking and depth projection run concurrently. Finalization builds segment-aware maps, runs the bounded RGB-D bridge study and validates artifacts. Existing completed stages are reused, not silently rerun.

Dependencies include the original raw camera archives, the isolated `fyp_stage24` Python environment, the existing native ORB-SLAM3 SDK/vocabulary, native OpenCV/Pangolin and `strace`. No SDK source is modified. Temporary DB storage is substantial; the audited expansion receipts identify exact new scratch files. Original archives are read-only.

## Open the demo

Serve the **whole parent `demo/` directory**, which also contains the pinned Three.js vendor files:

```bash
python -m http.server 8022 --bind 127.0.0.1 --directory demo
```

Then open `http://127.0.0.1:8022/stage24_full_rgbd/`. Do not double-click `index.html` through `file://`.

Local reproduction needs the generated `manifest.json`, per-robot `data.json`, epoch binary buffers and presentation RGB sequence. These are generated deterministically from the camera-only run and described by artifact manifests. Raw bags, native input images/depth PNGs, full preview sequences, PLY maps, native binaries and SDK/model files are intentionally not uploaded to GitHub. A source-only checkout must generate or copy the presentation bundle first; a missing bundle produces an explicit load error.

## Controls and resource limits

Play/Pause, Reset, Seek, speed and Stage20/final jumps operate on normalized recorded camera time. Left drag orbits, right drag pans and the wheel zooms each 3D panel. Fit affects the visible map only. Each panel retains at most two epoch geometries and twelve decoded RGB previews, with at most two image requests pending. Evicted GPU geometries/materials are disposed.

Full-stream processing completion, uninterrupted tracking quality and browser rendering acceptance are reported separately in `docs/CUMULTI_STAGE24_FULL_RGBD_MAPPING.md` and `outputs/cumulti_v1/24_rgbd_full_v1/VALIDATION_REPORT.txt`. A working demo must not be described as proof of continuous tracking or successful full-recording global fusion.
