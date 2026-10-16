# Stage 19 — persistent three-map / authentic RGB replay

Serve the **demo parent directory**, because the new presentation reuses frozen
Stage18.1 assets by relative URL. Do not serve this subdirectory alone.

Windows (any current working directory):

```powershell
python -m http.server 8021 --bind 127.0.0.1 --directory "E:\university\uic\FYP\demo"
```

Open http://127.0.0.1:8021/stage19_rgb_three_map/ . Keep the terminal open.
The preserved Stage18.1 is at `/progressive_multi_robot/` on the same server.

Linux, from repository root: `scripts/run_stage19_demo.sh` (port 8021).
No GTSAM/ROS/browser build is needed for playback; Three.js r170 stays bundled
in the accepted Stage18.1 directory. No runtime CDN dependency.

Three persistent metric-XYZ panels: independent R1 local, prebuilt R3 local,
actual frozen common-frame global (empty until first accepted loop). Global
is step-held at real persistent iSAM2 states, not interpolated toward FINAL.
Local panels never take global optimized poses. Independent orbit/pan/zoom,
source/trajectory/loop toggles, point-size, top/perspective/fit controls.

Authentic camera ZIPs are decoded by `src/demo/export_stage19_rgb.py`:
first `--mode prototype`, then `--mode full`, with a 120MiB presentation gate.
JPEG width384 / quality50. R1 retains intermediate camera observations at
approximately5Hz plus keyframe associations; original camera stream remains
approximately10Hz, distinct from the frozen2Hz LiDAR sampling. R3 includes
explicit historical candidate observations, **not live simultaneous video**.
`rgb_manifest.json` and per-robot CSVs preserve original nanoseconds as strings,
bag/header times, IDs, source archives, checksums, sizes, and association errors.
R1 displays floor(time), never a future nearest camera frame. Missing/failed
images are visibly labeled, not replaced with stock/generated imagery.

Play/Pause, Reset, Seek, demo speed .25/.5/1/2/4, PRE_FIRST_LOOP, FIRST_LOOP,
K2/K5/K10/K20/K40/K75/FINAL and candidate/GICP jumps are supported. Demo clock
is event-paced/accelerated; recorded R1 sensor nanoseconds are separate.
Between LiDAR KFs, map/node state is held while RGB advances strictly before
the next KF; query/SC/GICP/accept/backend phases freeze at event sensor time.
Only next-two images prefetch; two concurrent decodes,24 image objects LRU,
six waiting jobs maximum. Seeking invalidates old asynchronous display requests.
No all-RGB startup decode. Evicted Image objects are garbage-collected (no
object URLs to leak); replaced global GPU geometry is explicitly disposed.

**Z is not necessarily altitude.** Frozen trajectories use KF0-relative LiDAR
gauge axes, not gravity-aligned ENU; uniform XYZ display has no Z exaggeration.
The separate offline height diagnosis reports the frame-orientation evidence.
No GT-derived alignment/flattening enters this demo.

Validation and performance: `outputs/cumulti_v1/19_rgb_three_map/`.
Independent audit: `python src/demo/audit_stage19_rgb_demo.py` on the server.
Presentation performance is not SLAM timing. Stage13B numerical readiness
remains negative, Stage15 sparse policy remains negative, Stage17 diagnostic
line remains closed. No new retrieval/registration/optimization, dual stream,
distributed SLAM or live demo. Stop after Stage19.
