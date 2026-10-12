# Clean75 progressive multi-robot mapping demo — Stage18 / 18.1

## Stage18.1 presentation upgrade

Default **3D MAP VIEW** uses locally bundled Three.js 0.170.0 / MIT. No CDN,
build step or network dependency. **SYSTEM VIEW** retains the explanatory
dashboard, numerical caveats, recorded measurements and technical controls.

Play presents about 70 seconds of event-paced replay at 1x (0.5/1/2/4x).
Before the first loop the two local coordinate frames remain separate; after
the accepted R1KF283 ↔ R3KF252 measurement, a real common-frame initialization
is shown, followed by real post-loop states. Robot3 is a reference run, not a
simultaneously live robot. GICP inset BEFORE/AFTER uses the saved exact SE(3).

TOP/PERSPECTIVE/FOLLOW R1/FOLLOW R3/FIT, OrbitControls drag/pan/wheel, point size,
source visibility and loop visibility are available. Final automatically uses
bird's-eye orientation. Follow R3 follows a reference observation marker.
Marker position/quaternion interpolation is presentation-only; map vertices,
graph nodes and loop endpoints are never interpolated. Optional opacity fades
between real states are not optimizer states. Continuous per-scan global map
growth was intentionally not added; the current real map remains step-held.

Use `?debug=1` only for rehearsal/performance evidence. The normal presentation
has no diagnostic tables over the 3D world. Local WebGL-capable browsers are
required; use HTTP, not file://. Versioned frontend URLs avoid mixed old/new
scripts after an upgrade. See `docs/CUMULTI_STAGE18_1_PRESENTATION_UPGRADE.md`.

Functional/data-integrity verification is separate from smoothness acceptance:
the automated in-app browser produced ~1 Hz RAF even on the unchanged baseline.
No 60-FPS claim is made; a normal foreground-browser rehearsal remains required.

This is an **offline deterministic replay of frozen CU-Multi experiments**.
It is not live SLAM or a distributed Kimera-Multi implementation. The modular
visualization is Kimera-Multi-inspired; our actual pipeline is LiDAR → Scan
Context → frozen GICP → centralized Pose3 factor graph / persistent GTSAM iSAM2
→ trajectory-based point-cloud reprojection. No semantic labels or meshes.

## Run / share

The committed lightweight browser data are sufficient; no raw dataset, GTSAM,
CDN or internet connection is needed to play the demo after cloning/downloading.
From the repository root, on any machine with Python:

```bash
python -m http.server 8018 --bind 127.0.0.1 --directory demo/progressive_multi_robot
```

Open http://127.0.0.1:8018/ in a normal browser (not `file://`). On the FYP server:

```bash
scripts/run_progressive_multi_robot_demo.sh
```

If connecting from another computer, leave an SSH tunnel open in its terminal:

```bash
ssh -N -L 8018:127.0.0.1:8018 -p 30122 cas@116.177.246.186
```

Then open http://127.0.0.1:8018/ on that computer. A tunnel is private to that
computer; it is not a public sharing link. Others can clone the GitHub repository
and run the first command themselves. Public hosting/security changes are not
part of Stage18. `BIND` and `PORT` environment variables configure the server
launcher; its safe default is localhost only. Stop a foreground server with Ctrl-C.

## What the supervisor should see

1. **Individual local maps**: blue Robot1 Local Map and orange Robot3 Local Map
   in two distinct panels labelled Independent Local Frames. Robot1's displayed
   scans/trajectory are limited to arrived KFs. Robot3 is REFERENCE READY.
2. **FIRST CROSS-ROBOT ENCOUNTER**: R1 KF283 ↔ R3 KF252, frozen ID45, first
   detected common place — *not a simultaneous physical meeting*. Highlighted
   local endpoints, SC candidate → GICP → accepted loop → common-frame initialization.
3. **Progressive Map Fusion**: real FIRST_LOOP/K2/K5/K10/K20/K40/K75/FINAL states,
   current loop count K/75, per-source points/trajectories, newest green constraint.
4. **FINAL GLOBAL BIRD'S-EYE VIEW**: complete source-coloured map in a dedicated
   wide panel. Top-down visualization of 3D Pose3 states; XYZ is retained internally.

Scroll to zoom, drag to pan, double-click or Fit map to reset. Map-source,
trajectory, loop and first-encounter switches are available. Play/Pause/Reset,
0.5/1/2/5x, a scrubber and checkpoint shortcuts work without a solver.
The playback speed and visual frontend substates are presentation timing, not
measured end-to-end latency. GICP quality is a frozen numerical value/progress
bar, not a probability of correctness. All75 events are accepted-clean records;
the demo does not fabricate rejected measurements.

## Causal semantics / data lineage

Historical `demo/final_replay/` is untouched and represents120 loops/6180 nodes.
The NEW demo uses retained R1 KF150–1999, R3 KF234–4179 and clean75 only.
Final graph:5796 nodes,5794 odometry,75 loop factors,one KF150 prior,5870 factors.
The45 excluded historical loops remain excluded.

Robot3 is a **prebuilt reference database**,3946 keyframes. Robot1 alone streams
in original timestamp/KF order. Query arrival gates every event; no future loop
is exposed. At first acceptance the exact frozen initialization is
`S = X_query Z_query_candidate inverse(L_candidate)`; no local-frame overlays
before that event. GICP thumbnails apply the actual `T_query_from_candidate`
to XYZ, never pixel shifting. The final map is sampled from the entire retained
map coverage, not a restricted encounter ROI; display subset is deterministic.

Stage13B saved9 real checkpoints:

| State | Robot1 KF | Arrived loops |
|---|---:|---:|
| PRE_FIRST_LOOP |282|0|
| FIRST_LOOP |283|1|
| K2 |378|2|
| K5 |512|5|
| K10 |638|10|
| K20 |664|20|
| K40 |1014|40|
| K75 |1967|75|
| FINAL |1999|75|

The original checkpoints are reused unchanged. To show **every newly accepted
constraint**, Stage18 additionally exports the missing68 post-loop states via
one exact historical Stage13B replay:1717 updates,one persistent instance,
default parameters,zero extra empty updates and no batch solves. All eight
original post-fusion checkpoints are reproduction gates at1e-7 maximum matrix
difference, never targets for tuning. The original75 measurements are unchanged.

There is now a real post-loop state for each K1–75, plus FINAL. Between loop
arrivals, global geometry is **step-held at the latest real estimate**, with its
KF/count explicitly displayed. No interpolation or future final-state morph.
The graph distinguishes the displayed estimate from recorded post-query counts
and timings. K40 is the first40 arrivals on the way to75, NOT CoverageK40 selection.

Frozen historical iSAM2: skip10,threshold0.1,wildfire0.001,one update per event,
no extra empty updates. Map: frame stride10 from retained start,point stride16,
voxel0.5m,range1–60m. Saved per-robot PLYs are repackaged as deterministic
browser subsets (≤10000 points/source/original checkpoint,≤2500/source/additional
loop state; millimetre-rounded map display coordinates only). Additional states
reproject the same raw sampled scans through their actual causal estimate,
voxelize per source, and use display-only stride5 trajectories while preserving
all arrived loop endpoints and graph counts. Actual scientific trajectories,
maps and factors are unmodified; backend nodes/edges are never decimated.
The pre-loop Robot1 map uses causally revealed locally transformed scan chunks.

## Numerical limitations — not a new accuracy result

Stage13B remains **INCREMENTAL_BACKEND_NOT_READY** under its original batch
agreement gates. Stage13–17 caveats are preserved. Incremental iSAM2 does not
exactly reproduce batch poses and neither is GT. The optional FINAL-only Full75
batch toggle uses Stage12D's complete batch trajectory and matching reproduced
final batch maps in the same gauge, with no rigid alignment. No sparse backend
is promoted; Stage15 remains NO_INFORMATIVE_SPARSE_POLICY. Stage17's numerical
diagnostic line remains CLOSED. This is presentation/integration completion,
not deployment readiness, real-time SLAM, perfect fusion or physical truth.

## Reproduce / audit assets on the research server

With frozen checkpoints and scan caches present:

```bash
OPENBLAS_NUM_THREADS=1 /home/cas/.venvs/fyp_gtsam/bin/python src/demo/prepare_progressive_multi_robot_assets.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/cas/.venvs/fyp_gtsam/bin/python src/demo/export_stage13b_progressive_states.py
/home/cas/.venvs/fyp_gtsam/bin/python src/demo/audit_progressive_multi_robot_demo.py
```

Generator refuses overwrite once replay.json exists. Use a fresh copied workspace
for regeneration, preserving committed assets/historical outputs. It performs
no SC/GICP/solver/GT call. The separate display-state exporter alone reconstructs
the unchanged historical persistent backend, with no new scientific experiment.
CSV files containing offline GT columns are read with an
explicit non-GT `usecols` allowlist; provenance hashes cover the source file,
not a decoding of excluded columns. Frozen trajectory/map/measurement hashes,
historical-demo hashes, policy hashes and generated checkpoint hashes are in
`outputs/cumulti_v1/18_progressive_demo/demo_data_provenance.json`.

During fusion the browser uniformly caps displayed maps at2500 points/source;
FINAL deliberately increases detail to10000/source. This display-only change
is labelled in the map hint and is not a change in scientific map coverage.

The auditor requires both data/provenance consistency and browser validation
evidence before declaring presentation PASS. Screenshots in the same output
directory cover local maps, SC, GICP, first fusion, multiple-loop graph,
K10/K20/K40/K75/final and the labelled batch-reference view. All browser code
is local, dependency-free Canvas2D. There is no claimed 3D orbit viewer; the
labelled XY projection is deliberately used for the presentation.
