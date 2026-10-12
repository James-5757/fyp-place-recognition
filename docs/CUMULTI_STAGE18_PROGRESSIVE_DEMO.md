# Stage18 — final progressive multi-robot mapping demo

Scientific input checkpoint: `ad649678e6dcc1021724cae97b291b03d6a9ad59`.
This stage is presentation/system integration, not another algorithm experiment.

## Four mandatory visual concepts

The new `demo/progressive_multi_robot/` shows blue Robot1 Local Map and orange
Robot3 Local Map in independent panels/frames, a dedicated FIRST CROSS-ROBOT
ENCOUNTER at R1KF283 ↔ R3KF252, progressive source-coloured global map fusion,
and a final wide FINAL GLOBAL BIRD'S-EYE VIEW. The encounter means a detected
common place in recorded runs, not simultaneous physical co-location.

Seven scenes connect independent local mapping → SC candidate → frozen GICP
verification → first common-frame initialization → centralized persistent Pose3
updates → progressive map reprojection → final complete collaborative map.
Pre-loop maps are never overlaid. SC/GICP are precomputed measurements and the
visual substate timings are presentation-only. Robot3 is a prebuilt3946-keyframe
reference database; Robot1 alone streams in original arrival order.

## Frozen lineage / numerical fidelity

Historical `demo/final_replay/` (120loops,6180nodes) is untouched, including its
pre-existing uncommitted changes. New lineage: R1KF150–1999,R3KF234–4179,
75clean/45excluded; final5796nodes,5794odometry,75loops,one KF150 prior,
5870factors. All measurements,transform direction,extrinsic,noise and gauge
are preserved. Maps are metric point clouds,not semantic meshes.

Reuse original Stage13B checkpoints PRE_FIRST_LOOP,FIRST_LOOP,K2,K5,K10,K20,
K40,K75,FINAL. Verified arrival identities: K10KF638,K20KF664,K40KF1014,
K75KF1967,FINALKF1999. K40 means an arrival checkpoint along the full75 path,
not the rejected CoverageK40 selector.

To make every newest constraint visible, a separate Stage18 display-state
exporter reconstructs the exact historical persistent backend with default
skip10/threshold0.1/wildfire0.001,one update per event and zero empty updates.
Its eight historical checkpoint comparisons are hard reproduction gates at
1e-7 maximum transform-matrix difference. No parameter is searched or changed.
The remaining68 post-loop states provide all K1–75; no synthetic future morph.
Between loop arrivals, the latest real state is held and labelled with its KF.
The current event's accepted constraint appears after its real backend-update
presentation state. Display trajectory decimation keeps every loop endpoint;
actual backend factors are not decimated.

Original per-source maps are reused. Additional per-loop maps use frozen
frame10/point16/range(1,60) raw scan sampling,actual Pose3 reprojection and
0.5m per-source voxelization,then deterministic browser subsets. First common
frame uses the exact `S=Xq Z_qc inverse(Lc)` restricted to arrived R1KF283.
GICP mini views preserve XYZ and apply frozen `T_query_from_candidate` exactly.
No pixel-shift alignment,frame interpolation,mesh deformation or post-alignment.

The optional FINAL-only Full75 batch numerical reference uses Stage12D poses
and matching final reproduced batch maps. It is not GT or the incremental result.

## Validation / artifacts

`outputs/cumulti_v1/18_progressive_demo/` contains source SHA256 provenance,
display replay protocol and reproduction evidence,independent data audit,
actual browser observations,mandatory presentation checks and screenshots.
The independent audit checks source/historical hashes,clean75 identities,
excluded45,non-GT column allowlists,SE(3) thumbnail transforms,trajectory ranges,
causal snapshot availability,all75 post-loop states and final graph counts.
Browser testing covers loading,Play/Pause/Reset,scene/checkpoint jumps,batch
switch and source/trajectory/loop controls. Screenshots cover local/SC/GICP,
common initialization/first fusion,multiple-loop graph,K10/K20/K40/K75/final
and the separate batch view. Browser rendering is dependency-free Canvas2D;
no CDN or raw dataset is required to play committed assets.

Final independent audit: **3301 checks PASS**, including all12 supervisor visual
acceptance items. Thirteen actual browser screenshots are archived. All eight
historical post-fusion checkpoints reproduce with maximum matrix difference
2.923217223838037e-13. The display-state reconstruction performed1717 updates,
zero extra empty calls,zero batch solves and zero new scientific experiments.
Play/Pause/Reset,first common-frame transition,K11 newest-loop map,canonical
K10/K20/K40/K75/FINAL and the separately labelled batch toggle were checked.
An early-load jump race was fixed and retested; asset audit method-name guards
were corrected to distinguish dictionary/hash updates from solver API calls.

The numerical diagnostic line remains CLOSED. Stage13B's
INCREMENTAL_BACKEND_NOT_READY and Stage15's NO_INFORMATIVE_SPARSE_POLICY are
unchanged; low update time or visual fusion is not real-time/accuracy proof.
No GT is decoded; no SC/GICP/batch experiment is rerun. One exact historical
incremental replay is solely for missing display states. There is no live sensor
input,distributed PGO,physical network/bandwidth measurement or Kimera-Multi
implementation. Top-down XY is explicitly a visualization of 3D Pose3 states.

Run/share/reproduce commands and limitations are in
`demo/progressive_multi_robot/README.md`. Stop after Stage18; do not automatically
start numerical diagnostics,distributed SLAM or live deployment.
