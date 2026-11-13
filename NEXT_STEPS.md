# Next steps

## Current presentation checkpoint — Stage 19 (2026-10-15)

Use `scripts/run_stage19_demo.sh` and `/stage19_rgb_three_map/` for the persistent
three-map/authentic RGB presentation; Stage18.1 remains separately available.
Serve the demo parent directory, not the new child alone. Review the Stage19
validation,performance and clock-offset limitations before supervisor rehearsal.
Gauge XY/Z is not automatically gravity-aligned ENU; do not explain the large
displayed Z span as terrain alone or silently flatten/GT-align scientific poses.

Stop after Stage19. Do not automatically start dual-stream scheduling,
distributed/live SLAM, visual pose estimation, new loop selection or numerical
retuning. Preserve all frozen factors/estimates and Stage13B/15/17 caveats.

## Current update — after Stage 13D (2026-10-09)

Decision: **NUMERICAL_DISCREPANCY_PARTIALLY_EXPLAINED**. No tested diagnostic
schedule meets the original gates at all six selected checkpoints. D2 fixes
K2/K40 agreement with extra updates but still fails K75/final; do not promote
it as a deployable policy or relabel Stage13B/C as PASS.

Empty calls advance the modulo relinearization schedule and may do no nonlinear
work. Re-elimination raw counts on no-work paths are unreliable, consistent
with the official4.2 uninitialized-field path. Use Stage13D cleaned validity
interpretations, not the historical huge counters; retain historical bytes.

A next, separately authorized GT-free methodology protocol may investigate
final-state stopping, linearization/cache behavior and bounded conditioning
diagnostics on identical factors. Current near-equal objective does not prove
pose agreement or a nullspace. Do not extend the update budget, force updates,
select new loops, tune from GT or change the demo automatically. Stop after13D.
See `docs/CUMULTI_STAGE13D_SOLVER_DIAGNOSTICS.md`.

## Pre-13D planning record (unchanged)

## Current update — after Stage 13C (2026-10-09)

Stage13C's independent fixed-grid convergence study is complete. V0 reproduces
Stage13B; all five variants preserve the same factors and pass integrity, but
none passes the original all-checkpoint pose-agreement gates. Persisted
decision: **NO_STANDARD_CONFIG_MET_GATES**. Do not promote a near-pass, choose
V4 as an online policy, relax the gates, or alter frozen historical outputs.

More frequent relinearization checks substantially improve K10, not K2 or all
final checkpoints. Lower thresholds raise latency without monotonic final
agreement. Empty updates sometimes change estimates, sometimes do not, and
also advance the skip counter; the cause is not established as relinearization
alone. A separately authorized, predeclared GT-free numerical-method audit may
study nonlinear stopping/update paths, cached linearization and weak directions
on identical factors. Do not expand the parameter search automatically.

No additional GT evaluation or maps are needed to interpret Stage13C. Backend
interval compliance is not full-system real-time readiness. Keep the demo and
SC/GICP unchanged, and stop here pending the next protocol. See
`docs/CUMULTI_STAGE13C_ISAM2_CONVERGENCE.md`.

## Historical planning notes (superseded where applicable)

CU-Multi Main Campus robot1/robot2 Stage 1 is complete: the read-only raw archives were inspected and `/home/cas/CU-Multi/processed_v1/` now contains a validated 100-keyframe-per-robot subset. See `docs/CUMULTI_STAGE1_VALIDATION.md` for actual topics, synchronization quality, calibration assumptions, and unresolved coordinate-frame questions.

Do not expand this subset to the full dataset or run retrieval yet. Before Stage 2, confirm the upstream UTM zone/datum and the GT measurement-frame origin; export/inspect CameraInfo intrinsics only if camera geometry becomes necessary. Keep GT restricted to synchronization, overlap/positive definition, analysis and evaluation—never candidate selection or reranking.

After those questions are resolved, design the CU-Multi retrieval protocol separately from historical KITTI: define robot1/robot2 database-query split, temporal sampling, overlap labels and held-out evaluation. Preserve SC + Cross-Max as the historical KITTI retrieval baseline; do not overwrite `outputs/canonical_v2` or treat the Stage 1 subset as a retrieval result.

For visual experiments, measure OpenCLIP viewpoint sensitivity explicitly and compare single view, mean pooling, Cross-Max and controlled multi-view aggregation under a fixed candidate protocol. Record dataset version, selected frames, timestamps, thresholds, pose frame, calibration and command in the experiment log for every new experiment.

## Current Stage 9 stop point

Stage 9 has completed the frame-gated SC--GICP integration without changing the
Stage-8 frontend or GICP algorithm. Fixed M1000 remains the retrieval baseline;
Rank-1 is mean-budget compatible at 500 ms on the tested host, but its p95
exceeds that budget and Top-3 early stop exceeds it on mean latency. The exported
139 sanitized candidate loop constraints are registration outputs only, not
guaranteed-correct loop closures or a graph-optimization result. Their robustness
must be evaluated in a separately frozen Stage-10 pose graph.

Stop here. Do not start PGO, map merging, pose-graph tuning, CVTNet, or fusion
tuning without a separately declared graph protocol: robust-kernel/edge-weight
rule, odometry source, loop evaluation split, failure containment, and offline
GT-only trajectory evaluation. Keep GT offline and preserve Stage 4/5/6/7/7.5/8/9
outputs.

## Stage 10A blocking condition

The Stage-10 odometry audit found an admissible Robot1 EKF IMU/GNSS odometry
candidate, but no admissible Robot3 non-GT local trajectory. The locally present
Robot3 relative-pose bag is GT data and must not be substituted. Transfer only
`robot3_main_campus_imu_gps.zip`, then repeat the audit before any PGO or map
merge work. See `docs/CUMULTI_STAGE10_ODOMETRY_AUDIT.md`.

## Stage 10.1 stop point

The corrected Stage-10.1 replay passes its initialization, extrinsic, frame, and objective-bookkeeping checks, but both solvers reach the deliberately frozen 25-evaluation cap. Its map and ATE outcomes are mixed. Do not describe it as a converged PGO system or use it for a live demo. Any follow-up must be declared as a separate policy experiment, with GT remaining offline-only and without rewriting the Stage-10/10.1 historical outputs.

## Stage 10.2 stop point

Neither frozen-loop policy formally converged within the predeclared 25/50/100/200 budget schedule. The GT-free conclusion is NO_POLICY_READY, so do not start a live robot demo. Any future graph work must be a separately declared Stage 10.3 methodological investigation, rather than a parameter tweak selected from these offline GT metrics. Preserve all Stage 10.2 outputs and keep GT offline-only.

## Stage 12A trajectory integrity checkpoint

The read-only GT-free audit traced the replay's long lines to implausible
displacements already present in Robot1/Robot3 raw EKF `odometry_map` messages.
The single-loop common-frame transform does not add a new 3-D jump, and the
Rank1 GTSAM solve does not create a new catastrophic top-20 translation, although
its odometry factors inherit those measurements. The demo's 10-keyframe stride
also draws visually misleading straight chords across parts of the anomalous
Robot3 path. Before a sparse first-loop / incremental-loop study or a new demo,
investigate the EKF source intervals documented in
`docs/CUMULTI_STAGE12A_TRAJECTORY_INTEGRITY.md`. Any trajectory or demo repair
must be a separate reviewed change; preserve frozen Stage 10.3 outputs.

## Current Stage 13A stop point (supersedes earlier stage-specific stop notes)

Stage12D's frozen stable-segment graph has now been independently evaluated at
K=1/2/5/10/20/40/75, preserving the first-arrival loop schedule, initial poses,
factor noise and historical outputs. Stage13A passes reproduction and integrity
audits, but the frozen map-median/p95 rule finds NO_SPARSE_SAVING. Do not choose
K from the subsequent offline GT results or claim monotonic accuracy gains.

Readiness is READY_FOR_INCREMENTAL_METHOD_STUDY. A separately authorized
Stage13B must predeclare a causal two-stream event scheduler and database
availability, then compare incremental updates against the frozen batch
reference. Current ordering is a separate-run query replay with an existing
Robot3 database, not simultaneous online robot operation. Do not start iSAM2,
live/replay demo changes, alternative loop selection, or covariance weighting
automatically. Stop after Stage13A; see `docs/CUMULTI_STAGE13A_SPARSE_LOOPS.md`.

## Current Stage 13B stop point

The reference-Robot3/streaming-Robot1 scheduler and persistent ISAM2 backend are
implemented, with exact final frozen-factor equivalence and no future nodes,
duplicate factors or catastrophic jumps. However, frozen default iSAM2 does
not meet all predeclared identical-graph batch pose-agreement limits: readiness
is INCREMENTAL_BACKEND_NOT_READY. Do not loosen thresholds, alter this run,
choose new loops, or use offline GT to tune parameters.

Recommend a separately authorized GT-free methodology audit of relinearization,
wildfire/partial updates, robust-factor linearization and stopping behavior,
using identical graphs and fixed comparison rules. Low update latency and
almost equal final objectives do not by themselves establish numerical or
end-to-end real-time readiness. Preserve the negative result. Stage15 demo
assets exist, but do not change the historical demo or start distributed SLAM
automatically. See `docs/CUMULTI_STAGE13B_INCREMENTAL_ISAM2.md`.


## Current update — after Stage 14 (2026-10-10; supersedes earlier stop notes)

Stage14 completed with **RESIDUAL_NUMERICAL_DISCREPANCY_UNRESOLVED**. Fresh
rebuilds repair K2 but not the remaining K75/FINAL gates. Rebuild from the same
persistent coordinates changes poses slightly, without resolving the final
residual; do not label internal Bayes-tree history as the sole cause.

LM probes demonstrate numerical initialization/termination sensitivity with
near-equal objectives, not proof of distinct global minima. Robot3 mismatch is
smooth along long chains and associated with structural loop support. Bounded
selected marginals support weak/nonuniform local information under fixed weights,
but do not prove a global nullspace or actual physical trajectory uncertainty.

Recommend a separately predeclared GT-free reference/stopping/linearization
methodology study before proposing loop-selection or deployment. Preserve
original gates, all historical120/retained75 identities, and excluded45 loops.
Do not start informative loop selection, change the demo, adjust factor noise,
restore excluded loops or use GT automatically. Stop after14. Read
`docs/CUMULTI_STAGE14_FINAL_CONSISTENCY.md` before further changes.

## Current update — after Stage 15 (2026-10-11; supersedes earlier stop notes)

Decision: **NO_INFORMATIVE_SPARSE_POLICY**. Informative nested selection from
the same75-loop pool improves some same-K map/structural diagnostics butdoes
not satisfy the fixed held-out measurement residual envelope. Do not advertise
K40's hypothetical46.67%factor/event reduction as an accepted saving or measured
network bandwidth. No default iSAM2 validation was triggered,no GTdecoded,and
no full/selected graph numerical reference is promoted to physical truth.

Keepfull75 as the current clean reference,excluded45 excluded,frozen noise,
canonical seed,originalcriteria andStage14 numerical caveats. Preserveten
supplementary rotation-screen FAIL conditions;do not relaxthresholds or call
all Stage15 trajectories integrity-PASS. The sparse negative decision remains
supported by held-out translation residuals independentofthese flags.

A separately authorized GT-free study could examine actual measurement
information/weak-direction support androbust residual geometry,witha policy
frozen before new metrics. Timeline maximin/quality ranks alone are insufficient
evidence of sparse sufficiency or observability. Do not expandselectors/budgets,
retune using GT,changeodometry/noise,restartSC/GICP,restoreexcluded45,orchange
the demo/startdistributedSLAM automatically. StopafterStage15;read
`docs/CUMULTI_STAGE15_INFORMATIVE_LOOP_SELECTION.md`.

## Current update — after Stage 16 (2026-10-11; supersedes earlier stop notes)

Stage16 decision: MEASUREMENT_INFORMATION_PARTIALLY_EXPLAINS_SPARSE_FAILURE.
Do not promote information rankings into an accepted sparse policy. Keep
Stage15 NO_INFORMATIVE_SPARSE_POLICY,75 clean/45 excluded,frozen factors/noise,
anchor,thresholds and all prior numerical/integrity caveats unchanged.

Missing-loop information is heterogeneous and high-DeltaI add-backs produce
larger median target residual corrections,but global tails/trajectory effects
are mixed. Largest pair-covariance modes are dominated by common-motion
uncertainty;full75 largest-eigenvalue shrinkage is small. Fixed Stage14 weak
windows lack clean-loop endpoints,so region-specific support is not proven.

A separately authorized GT-free protocol could study measurement-projected
relative uncertainty and robust nonlinear residual geometry before designing
an information-aware selector. Maintain exact joint cross-correlation,explicit
linearization points and predeclared predictive validation. No GT tuning,
selector/budget expansion,demo modification or distributed SLAM automatically.
Stop after16;see `docs/CUMULTI_STAGE16_MEASUREMENT_INFORMATION.md`.

## Stage 17 final checkpoint — supersedes the preceding diagnostic proposal

Measurement-projected relative uncertainty and nonlinear robust geometry audit
is complete: RELATIVE_UNCERTAINTY_AND_NONLINEARITY_PARTIALLY_EXPLAIN_STAGE16_GAP.
The numerical diagnostic line is CLOSED regardless of this partial conclusion.
Do not automatically start Stage18 diagnostics, new selector/budget experiments,
additional add-back runs or GT-based tuning. Stage15 NO_INFORMATIVE_SPARSE_POLICY
and the frozen historical backend results remain unchanged.

The next separately authorized phase is final progressive mapping demo/system
integration and thesis/report synthesis, using the existing frozen results and
explicit scientific limitations. No demo or new experiment starts automatically.
See `docs/CUMULTI_STAGE17_RELATIVE_MEASUREMENT_GEOMETRY.md`.

## Stage 18 — final presentation checkpoint

The new clean75 progressive mapping demo is the final presentation lineage;
historical120-loop demo stays unchanged. Use the new README and provenance,
not the older full6180 graph,for supervisor demonstration and dissertation
figures. Preserve all scientific limitations: default incremental estimate is
not physically calibrated truth or exact batch equality; sparse-policy negative
result and numerical diagnostic-line closure are unchanged.

Stop after Stage18. The next separately authorized activity may be presentation
rehearsal and thesis/report synthesis. Do not automatically start another
numerical diagnostic,selector experiment,distributed backend or live deployment.

## Stage18.1 — presentation rehearsal only

The 3D/system rendering upgrade is committed as a presentation change. Before unconditional supervisor-demo acceptance, rehearse normal1x in a foreground WebGL/hardware-accelerated browser and record whether there is obvious stutter; automated in-app~1Hz RAF cannot validate that criterion. Use normal URL without ?debug=1 for presentation. Preserve clean75,excluded45,Stage13B negative readiness,Stage15 negative policy and closed Stage17 line. No Stage19,GT,new loop policy,distributed/live backend or solver retuning automatically.

## Stage20 — current authorized stopping point

The new dual-stream replay is hypothetical normalized time over separate recordings, using availability-gated precomputed frozen loops and a persistent **centralized** iSAM2. Structural readiness does not resolve numerical batch disagreement or historical Stage13B limitations. Use the new Stage20 demo/README/validation evidence for rehearsal; retain Stage19 as a frozen reference. Local sensors progress independently; global visualization is explicitly step-held at actual saved states, never future-map morphing. Do not infer a terrain-only explanation from gauge Z.

Stop after Stage20. Any future distributed backend, communication study, online SC/GICP discovery, solver retuning, loop-selection change or live deployment needs separate authorization. Preserve the frozen75 measurements and all historical stage outcomes. No automatic next research stage.

## Stage21 — current completed cost baseline / stopping point

Stage21 COST_BASELINE_READY applies to measured buffer sizes, a fixed100-query prebuilt-frontend workload and a separate Stage20 oracle-update workload. It is not a deployed communication-efficient online SLAM result. Preserve the original missing-network, causal-availability and numerical-readiness caveats. Full-SC cache misses and geometric cloud requests, not Ring-Key bytes alone, dominate important parts of the communication accounting; original index/manifest overhead and map publication are significant compute costs.

A separately authorized Stage22 may test cache- and verification-cost-aware prioritization with fixed quality, an explicit causal frontend and realistic resource assumptions. Do not infer that the unbounded reference cache is an accepted optimal policy. Lossy/preprocessed cloud transport requires new validation before claiming no quality loss. No new selector, distributed optimizer, wireless throughput/energy claim, GT tuning or Stage22 run starts automatically. Stop after Stage21.

## Stage22A — authorized stopping point

Decision CAUSAL_ONLINE_GRAPH_READY. Preserve all frozen historical stages and the new causal
frontend results. Stage22B is not started or automatically authorized. Any
budget/selector/graph admission or solver policy change requires an independent
predeclared follow-up; never tune against the offline GT results. Distinguish
actual algorithm execution from hypothetical normalized dual-stream timing and
protocol-simulated sends. See Stage22A doc for coverage and readiness limitations.

## Stage22B — current stopping point (supersedes authorization notes above)

The separately authorized budget-aware experiment is implemented and measured.
Do not promote its P5 policy: cost reductions lose too much query quality.
Full accepted Stage22A remains the reference. P1/P2 numerical failures and
near-zero byte savings must not be hidden by call-count improvements.

The frozen chronological holdout has no positive dual-progress/R1-direction
queries. Overall generalization status remains EXPERIMENT_INCOMPLETE; do not
retroactively rebalance this now-GT-exposed route or loosen noninferiority
limits. A future separately authorized study needs another independent route
or recording with meaningful positive overlap in both directions/phases and
a prospectively frozen validation protocol before proposing a publishable
budget-aware algorithm. Transport/cloud representation changes require their
own verification rather than assumed geometric equivalence.

See `docs/CUMULTI_STAGE22B_BUDGET_AWARE.md` for complete evidence scopes,
negative findings,actual vs receipt-based runtime,cache sensitivity and mapping
diagnostics. STOP AFTER STAGE22B. No visual SLAM,third robot,distributed
optimization,new demo or automatic follow-up experiment.

## Stage24A/B — camera feasibility checkpoint (2026-11-06)

Stage24A RGB/semantic3D maps remain BLOCKED: archived URDF reaches camera_link
but not the color optical frame. Do not guess mounting/optical transforms or
color unseen surfaces. New calibration-gated P5 demo and12 real SegFormerB0
2D predictions exist; RGB/semantic map modes explicitly show100%unknown.
RGB-D is BLOCKED for R1/R3: R3 depth missing; R1/R2 depth archives contain
camera PointCloud2 rather than native depth images. Actual R1 cloud is
unorganized,376811x1,with848x480 camera calibration; raw depth scale and
registered-image adaptation require validation. R1 camera-cloud/RGB bag sync
median1.38209ms,p956.2070072ms is not hardware exposure calibration.

Independent camera-only timelines contain1000/2099 R1/R3 visual keyframes.
Six fixed12s ORB-SLAM3 monocular windows each use60real RGB frames: valid
counts R1=0/47/58,R3=0/49/41. Arbitrary per-map scale is preserved; no LiDAR,
SC,GICP,GNSS,IMU or GT supplies tracking/geometry/scale. New offline image-only
retrieval72queries/36opposite candidates yields1epipolar-plausible nonmetric
pair,not a verified metric loop or measured place-recognition precision.
Later pilot frames may be in the offline database; this is not causal online
retrieval. No full visual SLAM or GT evaluation was run.

Three-robot inspection finds existing numeric_key/symbol mapping aliases
Robot2 with Robot3; generic registry/typed endpoints/connectivity and per-pair
budget migration are needed before a new3-agent run. R2 archives exist; R4
has only GT locally. No sensor measurements may be invented.
Historical10098 tracked files match their pre-stage hashes. Independent
artifact audit831checks PASS; browser orbit/zoom/pan/mode checks PASS, with
software-WebGL performance only. No old scientific artifact or frozen
P0/P5/P6 policy was changed. Details and reproducible commands:
docs/CUMULTI_STAGE24A_RGB_SEMANTIC_MAP.md and
docs/CUMULTI_STAGE24B_VISION_FEASIBILITY.md.
Stop after24A/B; Stage24C is NOT_STARTED. Minimum next work is measured optical
mounting/exposure calibration and validated camera-cloud-to-depth adaptation,
then small fixed-cohort projection/visual-geometry tests,not automatic full runs.

## Bounded camera RGB-D local-map demo (2026-11-08)

PASS_FUNCTIONAL_DEMO_TRACKING_PARTIAL. PARTIAL_PREDECLARED_GATE_NOT_MET. New independent camera-only maps, original depth PointCloud2 adaptation, timestamp-aligned RGB and native paced ORB-SLAM3 RGBD. No LiDAR/GT/IMU input or semantic map; reset epochs never merged. No cross-robot visual fusion or full-sequence run. See docs/CUMULTI_RGBD_CAMERA_MAPPING_DEMO.md and demo/rgbd_camera_mapping/README.md. Historical Stage24A/B blocked reports are preserved; Robot3 depth arrival resolves availability, not missing LiDAR-optical calibration.

Verified short-window tracking: robot1 44/60 valid frames, 2 separate tracking epoch(s), 76260 display points across its epochs; robot3 60/60 valid frames, 1 separate tracking epoch(s), 100000 display points across its epochs. No joint camera map has been established.

## Native-rate RGB-D long-window stability (2026-11-08)

PASS_AUDIT. Frozen pre-fusion decision: NO_GO_RGBD_FUSION. Each robot uses120s and all original RGB frames. Real reset regression proves map ID alone is insufficient; use map ID plus initial-keyframe ID. No GT/LiDAR/IMU or semantic input. See docs/CUMULTI_RGBD_LONG_TRACKING_RESET_AUDIT.md and outputs/cumulti_v1/24_rgbd_long_v1/pre_fusion_decision.json.

robot1 baseline1000: 1200/1200 tracked, 0 frame change(s), gate PASS; robot1 fallback2000: 1136/1200 tracked, 4 frame change(s), gate FAIL; robot3 baseline1000: 1168/1200 tracked, 1 frame change(s), gate FAIL; robot3 fallback2000: 1137/1200 tracked, 2 frame change(s), gate FAIL.
Dual-robot fusion not run because stability gate failed. Diagnose tracking/depth/motion before a separately documented revised experiment; do not lower the frozen gate or join reset maps. No further download is required for this cohort.

## RGB-D loss-cause audit (2026-11-10)

Offline native feature/source audit confirms tracking loss, not missing data: roughly3s relocalization timeout then map restart/reset. All7 first-bad images retain many native keypoints and adjacent 2D geometric matches; native map/PnP inlier diagnostics are still missing, so the unique initial failure cause is unresolved. Robot3 baseline atRGB-D frame255: saved map associations117→22,1006 native features,269 with depth,237 independent adjacent F inliers. No SLAM/GT/fusion rerun or frozen result change. See docs/CUMULTI_RGBD_LOSS_ROOTCAUSE_AUDIT.md.


## After full available camera RGB-D replay (2026-11-10)

Stop at this checkpoint. Review full-sequence tracking fragmentation and unpaired-depth intervals independently of browser success. Inspect per-epoch maps rather than treating a reset-gauge collection as one consistent map. Any new bridge or graph-fusion method requires its own explicit validation; the bounded RGB-D bridge study does not establish complete global fusion. Do not change the frozen LiDAR/iSAM2 baselines or semantic-mounting-calibration conclusions.

Full available camera-only experiment and segmented replay are validated. RGB durations are 999.789160747 s / 2098.463330423 s, so Robot1 cannot supply the Stage20 1972.5-second duration. The new demo spans both complete available streams with a 10:1 clock and an explicit shorter-recording end. This is not proof of uninterrupted tracking or a complete global map.

- robot1: 9864/9997 valid camera poses (0.986696), 6 Map Epochs, 1004.565 s native wall time, 3010.7734375 MiB native peak RSS.
- robot3: 20494/20977 valid camera poses (0.976975), 20 Map Epochs, 2110.655 s native wall time, 6957.453125 MiB native peak RSS.

Bridge diagnostic: NO_RELIABLE_RGBD_BRIDGE_ESTABLISHED; 0 single-pair geometric passes, 0 supported epoch-pair bridges. No unrelated reset gauges are glued. See docs/CUMULTI_STAGE24_FULL_RGBD_MAPPING.md, demo/stage24_full_rgbd/README.md and outputs/cumulti_v1/24_rgbd_full_v1/VALIDATION_REPORT.txt. All scientific and presentation limitations remain explicit.

## Stage24E feasibility (2026-11-13)

The audit covers 24 within-recording epoch boundaries and two initializations. Of 1,301 sequential RGB-D frame pairs, 14 passed geometry and none formed a new bridge under the unchanged multi-frame gate. Full-route camera-only continuity remains NO-GO. Original camera timestamps overlap the Stage12D LiDAR trajectory, but the measured color-optical mounting transform is missing, so hybrid projection remains NO-GO. See docs/CUMULTI_STAGE24E_FEASIBILITY.md for the per-boundary and calibration evidence.
