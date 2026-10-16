# Stage20 — availability-gated dual-stream collaborative mapping

Baseline: `ee11e20925da35e369a562a598a5761bc68b5cf7` (Stage19).

## Scope and decision

Stage20A: **DUAL_STREAM_BACKEND_READY**, a structural/finite/integrity decision, not acceptance of numerical batch agreement or physical accuracy. Stage20B is a separately validated presentation of these states. No distributed optimizer, communication experiment, online SC/GICP, new loop policy or GT tuning was introduced. The historical Stage13B readiness limitation remains unchanged.

Robot1 and Robot3 were separately recorded. This experiment starts their retained streams at normalized zero, preserving original within-run nanosecond intervals. It is a hypothetical two-stream offline replay, not physical simultaneous operation. The 75 archived loop factors were computed offline. Endpoint gating is causal only with respect to simulated data availability; it does not establish an online retrospective search algorithm.

## Frozen inputs and scheduling

R1 KF150–1999 (1,850) and R3 KF234–4179 (3,946) use unchanged Stage12D local poses and sequential odometry. SHA256 input/protected-artifact manifests are written before execution; the four clean12D inputs are also compared against baseline Git blobs. Source timestamps retain integer ns, not floating epoch seconds. Scheduler order is `(timestamp − own retained start, robot ID, original KF)`; loops at one event tie-break by frozen ID. An edge releases after the later of its endpoint **events**, including deterministic handling of tied timestamp groups. Synthetic unit tests separately cover all required scheduling cases.

Independent locally anchored odometry graphs grow before and after fusion. Only the arriving robot is extended. Their local poses remain immutable and are never overwritten with optimized global coordinates. Before the first bridge no disconnected central iSAM2 is created. At fusion only the arrived prefixes enter one connected graph; only the R1 KF150 prior is retained centrally.

## First bridge and delayed availability

The first edge is independently verified as ID45, R1 KF283 ↔ R3 KF252. It releases at **66.501611950 s**, event267. Current prefixes are **R1 KF283 / R3 KF367**, 134 nodes each: 268 joint nodes, 266 odometry, one loop, one prior. Components change from two to one. Initialization uses the frozen convention `Z_qc = T_query_from_candidate`, `S = X_q Z_qc inverse(L_c)`. Residual is 1.432144669219779e-14 m / 1.5905806597954793e-15 degrees.

Primary first-fusion measurements: graph construction 6.252477876842022 ms, first iSAM2 update 3.2714339904487133 ms, extraction 0.42410707101225853 ms, map publication 132.4341930449009 ms, total measured service 142.48698530718684 ms. Presentation-extraction timings are not substituted for these primary values.

The first ten eligibility-ordered frozen IDs are **45, 89, 90, 94, 51, 53, 55, 57, 59, 61**. ID89 (historical query-order position45) is the second edge at 1123.502391955 s. Historical second edge ID46 must wait for R3 KF2930. R1 finishes around924.501 s, before any subsequent edge is available. The long first-loop plateau is genuine, not an animation bug. Late archived constraints can attach to the already completed R1 trajectory. Multiple edges may arrive in one event: checkpoint K10 actually contains11 edges and K40 contains41; labels show actual counts rather than inventing intermediate optimizer states.

## Persistent backend and identity

The default historical13B iSAM2 policy remains unchanged: relinearization threshold0.1, skip10, wildfire0.001; zero extra empty updates. One persistent central instance receives5,529 updates from the first bridge to the end. New nodes initialize from the preceding estimated pose composed with frozen odometry. Existing measurements, extrinsics, directions, robust kernels, prior and noise models are not recomputed.

Final graph: **5,796 nodes, 5,794 odometry, 75 loops, one prior, 5,870 factors**. Verification covers keys, timestamps, exact-once template indices and actual GTSAM factor equality including measurements/noise. All event estimates are finite; the frozen trajectory-integrity test finds zero newly catastrophic jumps. Independent audit reconstructs the schedule and every prefix without calling the scheduler implementation.

Primary odometry-event local processing medians are0.029049348086118698 ms (R1) and0.02891523763537407 ms (R3). Central update median/p95: R1 **9.421286405995488 / 23.412942653521895 ms**; R3 **0.057821162045001984 / 22.60635029524564 ms**. Loop-event update median/p95 is0.5204137414693832 / 38.174487114883924 ms, while map publication median/p95 is1570.5297298263758 / 1735.058965231292 ms. Map generation is therefore not hidden inside a fast optimizer claim. These are category-conditional CPU measurements, not proof of real-time SLAM.

Actual combined-event intervals, including near-ties, are used for deadline diagnostics. Serial offline backend queue length is explicitlyzero; a separate hypothetical real-time FIFO backlog model is labelled as such. There is no network transport, serialized message payload measurement or measured bandwidth. **NETWORK_COMMUNICATION_NOT_MEASURED**. Typed future message categories are policy metadata only.

`measured_service_ms` sums the instrumented local/construction/update/extraction/map stages; it deliberately excludes the separately logged finite/integrity auditing time and offline batch-diagnostic work. The `backend_deadline_met` column compares the iSAM2 update alone to the next actual combined-stream interval, not the total service/map or end-to-end wall time. The hypothetical FIFO model uses this instrumented service subtotal, not measured asynchronous queueing. These distinctions prohibit an end-to-end real-time interpretation.

## Numerical diagnostic — not silently accepted

Batch LM uses exactly the available nodes/factors at selected Stage20 checkpoints, frozen LM settings and the canonical first-bridge non-GT initialization. No partial graph is compared to a complete-trajectory13A graph. Each comparison stays in the same gauge; robots are never independently aligned.

At FINAL, incremental objective25.06800539143728 versus batch23.840383645478475 does **not** imply coordinate equality. All-robot translation median/p95 is0.12801838436717367 / 0.7617601789804498 m; rotation median/p95 is0.05498979368372768 / 1.0290840352584825 degrees. R1 passes the unchanged diagnostic; R3 and joint result fail. R3 translation median/p95 is0.1618892375173447 / 0.7782886107489599 m. Existing median/p95 thresholds0.1/0.5 m and0.5/2 degrees were not relaxed. The complete checkpoint table is authoritative. No GT evaluation or solver retuning followed these observations.

## Stage20B state and RGB provenance

The new `demo/stage20_dual_stream/` leaves Stage19 and earlier demos unchanged. R1 and R3 local map/trajectory prefixes remain visible and independently progress. Before fusion the global panel is UNALIGNED, not an overlay of unrelated local coordinates. Afterward it displays actual Stage20 central estimates only. Both sensor markers step through arrived original KFs; no scientific nodes, vertices or loop measurements are interpolated.

An exact same-policy display replay adds periodic60-second snapshots, all loop-event states and mandatory milestones:108 real saved states. All75 matching primary checkpoints reproduce with **zero** matrix translation and rotation difference, identical event trace and no extra updates. Display extraction does not run batch diagnostics; an empty comparison set is explicitly NOT_RUN, not a numerical PASS. Primary results remain canonical. Global state time and available prefixes are labelled, step-held between actual saved states while the local streams continue. Consequently global publication is checkpoint-based, not asserted to update visually at every sensor event.

Local scans use frame stride10, point stride16, range1–60m, voxel0.5m, deterministic cap180/chunk for display; R3 uses395 arrived-gated chunks rather than the old prebuilt full cloud. Saved global clouds are deterministically capped for browser display. Source provenance stays blue R1/orange R3/green loops. Exact endpoints are retained in trajectory display subsets. XYZ scale is uniform; gauge Z is not guaranteed gravity/altitude. Stage19's height diagnosis is not replaced with a terrain-only explanation, and no GT-based flattening is applied.

Robot1 reuses the authentic Stage19 forward JPEGs. Robot3's old71 historical references are not a video. Before any new extraction, count/byte/quality policy is fsynced: at most2,200 additional images /24 MiB,320px wide, JPEG45. A first10-second prototype at the actual recorded camera rate gates the full approximately1 Hz sequence, with causal loop-endpoint coverage. Raw ZIPs stay unchanged; one temporary SQLite expansion is CRC-validated and removed after read-only extraction. Bag/header timestamps, nearest LiDAR associations, errors, sizes and image hashes remain in the manifest. Images are selected separately on each robot's original clock, at/before target time; recording completion freezes that robot's last sensor time. Final measured RGB/export/browser statistics are generated into the validation artifacts rather than assumed here.

## Controls, validation and reproduction

Controls: Play/Pause/Reset/Seek;0.5x/1x/2x/4x with explicit base10 recorded seconds/demo second; both follow cameras, top/perspective, three-panel toggles, provenance/trajectory/loop toggles, and twelve new-schedule checkpoint jumps. FINAL uses the actual completed two-stream state, not old Stage19 coordinates.

Use `demo/stage20_dual_stream/README.md` for the parent-directory launch command; serving only the subfolder breaks sibling imports. Browser startup, RAF/playback, seek and bounded RGB cache measurements are separate from GTSAM CPU timings. No universal60FPS claim. RGB decode is demand-driven, at most2 active /6 queued /24 cached; global JSON cache holds3 states and replaced GPU state resources are disposed. Local chunk geometry is cached, not rebuilt every RAF.

Primary execution: `scripts/run_stage20_dual_stream.sh --output <new-directory>` (refuses an existing output directory). Display reproduction: same Python runner with `--display-extract --output <separate-directory>`. `prepare_stage20_assets.py` gates packaging on exact primary/display-state equality. `export_stage20_rgb.py` measures and enforces prototype budgets. `audit_stage20_backend.py` and `audit_stage20_demo.py` independently audit scientific/backend and packaged-asset invariants; browser screenshots and DOM audit snapshots validate controls and presentation.

No historical results are overwritten. Large full trajectory/PLY/reproduction artifacts remain server-only with hashes; lightweight machine-readable results and bounded presentation assets are versioned. Stage20 stops here: no distributed SLAM, new loop selection, live demo or GT tuning starts automatically.

## Final presentation measurements

Robot3 prototype:100 actual frames covering9.904999123s,714,729 bytes; projected full count2,011 and14,373,200.19 bytes. Actual export: **2,011 JPEGs /14,155,304 bytes**, within the predeclared2,200 /25,165,824-byte budget. Native recorded rate10.027196866332522Hz is not the approximately1Hz presentation rate. R1 reuses5,686 Stage19 files without copying them into the new site.

At LiDAR keyframe times, causal exported-image display age median/p95/max is **38.9026315 /98.73809415 /130.517312ms** for R1, and **127.737403 /596.9674145 /770.082234ms** for R3. These are low-rate presentation ages, not sensor synchronization calibration errors. Nearest-LiDAR association-error statistics for each exported image are separately retained in `stage20b_rgb_sync_statistics.json`; they are not conflated with causal playback age.

Foreground Edge validation exercised all12 checkpoint jumps, panel and loop toggles, cameras, speeds, reset/seek and play/pause. The45.425s early playback sample rendered1,736 frames (38.2168409466153 observed FPS); the92.664s late-fusion sample rendered2,957 frames (31.91099024432358 observed FPS, includes completion/idle tail). Both robot prefixes advanced early; only R3 continued after R1 completed. Maximum image cache24, active decodes2, decode errors0. No browser script errors were recorded. These samples are specific to this machine/browser/foreground run, not a general60FPS guarantee, full performance characterization or solver/runtime comparison.

Final status: **DUAL_STREAM_BACKEND_READY / THREE_MAP_DEMO_READY / AUTHENTIC_LOW_RATE_FORWARD_STREAM**. Independent backend and asset audits pass. Joint numerical batch agreement remains **FAIL** under unchanged historical thresholds. Three screenshot files show before fusion, the first bridge and the complete final three-map state.
