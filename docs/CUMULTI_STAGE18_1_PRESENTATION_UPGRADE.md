# Stage18.1 — RViz-style 3D presentation upgrade

This changes presentation only, from accepted Stage18 commit
`0447a62210d46a685285b8f6d649d49fd880960d`. No Stage19, GT load, new retrieval,
registration, loop selection, batch solve or solver retuning occurred.

## Scientific state versus animation

All accepted Stage18 map/trajectory JSONs are byte-preserved. Retained ranges
are R1KF150–1999 / R3KF234–4179; clean75 / excluded45; final graph is
5796 Pose3 nodes,5794 odometry,75 loops,one prior,5870 factors. The prior remains
R1KF150 and historical default persistent iSAM2 skip10/threshold0.1/wildfire0.001,
5-degree/1-m diagonal noise and Huber1.345 remain unchanged.

One exact historical Stage13B reproduction extracted only the current Robot1
Pose3 for each KF. It saved1850 timestamp-preserving pose rows, used1717 genuine
updates,75 loops,zero extra empty calls and reproduced all eight existing
checkpoints (maximum matrix-element difference2.923217223838037e-13; gate1e-7).
It does not save a new full graph at every frame or create an accuracy result.
The exporter refuses to overwrite its stream. The first local R1KF283 pose is
also exported explicitly, so pre-acceptance events cannot use the post-loop pose.

Robot-marker interpolation is presentation-only interpolation between consecutive
recorded causal poses. Only marker position/quaternion changes continuously;
map points, graph nodes and loop geometry remain at real recorded states.
Cross-fade is a visual opacity transition between two real recorded states and
is not an optimizer state. Different local frames are never vertex-morphed or
overlaid. An atomic state/context swap prevents a pending fetch from mixing a
future marker with an old geometry. Candidate endpoint highlights and SC/GICP
values remain hidden at QUERY ARRIVED until the candidate subphase; Reset also
hides historical candidate thumbnails rather than exposing a future event.
Robot3 remains a prebuilt reference; its
display icon is a reference observation, not a live robot.

Continuous new-scan global growth is optional and was not implemented. Between
real post-loop/checkpoint states the map remains explicitly step-held; marker
animation does not imply continuously optimized global geometry.

## Rendering and story

Default 3D MAP VIEW uses locally packaged Three.js0.170.0 (r170), official
`mrdoob/three.js` tagr170 build/OrbitControls, with retained MIT license. No CDN,
application framework or presentation-time build. True saved XYZ become GPU
BufferGeometry/Points. Robot1 cyan,Robot3 orange,loops/encounter green,prior white.
Axes/grid are labelled presentation guides, not additional measurements.

Before acceptance: two separate scissored cameras/scenes in independent local
frames. At R1KF283↔R3KF252 (historical ID45): candidate highlight → actual frozen
GICP XYZ BEFORE/AFTER inset → bright accepted edge → real COMMON_INITIAL → first
persistent joint estimate. This is a common observed place, not simultaneous
physical encounter. Subsequent states follow the frozen arrival order, with
K10/K20/K40/K75 and FINAL emphasis. Final auto-fits a bird's-eye view; perspective
reveals actual Z structure. Cameras, source/loop toggles and point sizes affect
display only. Standard OrbitControls rotate/pan/zoom remain available.

SYSTEM VIEW preserves local map panels,Robot Agents,SC score/rank/shift/yaw,
GICP thumbnails/quality,Pose3 graph,status,event replay and scientific limitations.
Map pan/zoom/fit and FINAL-only batch-reference selection remain available.
No peer-to-peer/distributed optimizer,semantic mesh,VIO or live transport exists.

## Measured causes and performance limitations

All measurements are **BROWSER PRESENTATION PERFORMANCE**, never SLAM timing.
The unchanged accepted baseline, instrumented without renderer changes, had
304 canvas dimension assignments /152 redraws /450 DOM mutation records in a
40-second global sample. Async render wall time mean67.539ms,p95286.700ms,
max501.200ms; the largest body load/parse288.2ms included12.8ms JSON parse.
Its local sample had540 dimension assignments /270 redraws. Repeated resizing,
uncached static rendering and fetch-in-render were demonstrated—not inferred
from the apparent motion alone. Repeated array lookups/bounds rebuilding were
also visible in the frozen source, but their isolated cost was not benchmarked.

New Canvas2D dimensions change only on actual element/DPR changes. Separate
marker overlays do not redraw static map/graph canvases; snapshot bounds and
event/trajectory/endpoint maps are preindexed. DOM values update only when a
KF/event/state/value changes. WebGL map buffers rebuild at a real state/chunk
change, not each requestAnimationFrame. The next three states prefetch; nine
essential resources must load before Play. Hovering a jump prefetches its target.

The System View40-second sample recorded5 canvas resizes,151 redraws,29 map
redraws,34 graph redraws and903 individual text writes; its replay advanced
through many more distinct real states than the old sample, so these totals
are not a matched-event speedup benchmark. At pausedK10,19 marker animation
callbacks caused zero map/graph/base canvas redraws or size assignments, and
only six debug-text writes. These counter checks verify caching directly.

New 3D and System measurements report synchronous RAF CPU cost, while baseline
reports asynchronous whole-render wall time including fetch waits. They MUST
NOT be divided to claim a speedup. After-metrics and loading/parse distributions
are recorded verbatim in the JSONs. Transport also differs: baseline used an
SSH tunnel; upgraded validation used local HTTP after repeated tunnel resets.
Startup times are therefore not a controlled network comparison.

All automated40-second samples show approximately1Hz scheduling, even with
document.visibilityState='visible'. This is consistent with automated in-app
browser throttling, but its exact cause is not established. A normal Edge
connector attempt was unavailable. Consequently 50–60FPS and the primary
"no obvious presentation stutter" criterion are **NOT VERIFIED**; do not declare
unconditional Stage18.1 presentation acceptance. Functional rendering, scene
controls,causal provenance and no normal-playback pending fetches are separately
audited. Rehearse in a normal foreground hardware-accelerated browser before
supervisor presentation. Exact60FPS itself is not an acceptance requirement.

## Reproduction and evidence

From repository root:

```bash
scripts/run_progressive_multi_robot_demo.sh
# HTTP localhost8018; PORT can select another port.
/home/cas/.venvs/fyp_gtsam/bin/python src/demo/audit_stage18_1_presentation.py
```

Any computer with Python can serve the committed distributable without GTSAM:

```bash
python -m http.server 8018 --bind 127.0.0.1 --directory demo/progressive_multi_robot
```

Open `http://127.0.0.1:8018/` (not file://). Assets and Three license are bundled.
Use `?debug=1` to measure40s and inspect RAF/frame cost/point count/state. Detailed
Generic WebGL renderer/version and Three revision are shown; unmasked device
fingerprinting is not requested.
Normal view has no FPS overlay. No frontend build or internet is required.

To reproduce the *baseline presentation measurement*, not a scientific experiment:
`python src/demo/prepare_stage18_1_baseline.py`, then open `/benchmark_before/`.
The accepted old app/style are reconstructed from git; only profiling hooks differ.
This scratch site is not part of the distributable or commit.

New display metadata can be regenerated with
`src/demo/prepare_stage18_1_3d_assets.py`. Pose extraction requires the frozen
server artifacts and isolated GTSAM environment; it has already run exactly
once and must not be rerun just for playback. Committed stream and extraction
policy/validation are sufficient for presentation. Official dependency URLs,
SHA256s and sizes are recorded in the asset loading summary.

Eleven requested browser screenshots, browser observations, before/after
measurements, manifests,independent audit and VALIDATION_REPORT are in
`outputs/cumulti_v1/18_1_presentation_upgrade/`. Optional MP4 not produced.

Stage13B remains INCREMENTAL_BACKEND_NOT_READY; Stage15 remains
NO_INFORMATIVE_SPARSE_POLICY; Stage17 numerical diagnostic line remains CLOSED.
No visualization proves physical map accuracy or distributed/live SLAM readiness.
Stop after18.1; no Stage19 automatically.
