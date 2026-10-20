# Stage21 — communication and computation cost baseline

Accepted input baseline: `5e9316b4bdd9c7a6fc412ad0ba5515f5adf23cf2`.

Decision: **COST_BASELINE_READY**, for the explicitly declared finite workloads below. This is a reproducible resource-measurement framework, not a communication optimization, causal online detector, distributed system or energy-efficiency result. Full historical2000-query traffic reconstruction remains incomplete; physical network quantities are unmeasured.

## 1. Motivation and evidence boundaries

The study asks which buffers and operations dominate cost, rather than choosing a new loop policy. Two protocols are never combined into an end-to-end timing:

- **HISTORICAL_PREBUILT_FRONTEND:** Stage8/9 Robot1 queries use a complete Robot3 reference database. The common B0/B1/B2 workload is all4180 R3 database keyframes plus exactly the original100 Stage9 latency-query IDs over R1 KF0–1999.
- **DUAL_STREAM_ARCHIVED_FACTOR_REPLAY:** Stage20 independently streams R1 KF150–1999 and R3 KF234–4179, then inserts offline-precomputed constraints after both endpoints arrive. B3 uses this separate graph-update workload as an explicitly labelled oracle lower bound, excluding place recognition and registration.

All new CPU measurements run on the Linux server. A logical “robot-local” assignment is a server-measured proxy, not a measurement of physical onboard compute. No real robot network, radio, packet loss, transport latency, battery or power meter is used.

## 2. Source audit and frozen measurement policy

`stage21_input_inventory.json`, `stage21_source_provenance.json` and `stage21_input_hashes.json` list actual files and hashes, not documentation-only assumptions. The server has the Stage9 candidate interface and the teammate's **6036 exact-pair results** (6000 in misleadingly named `gicp_rank1.csv`, plus36 Stage9 new pairs). The original100-query harness and its exact query IDs are available. Some large source traces were intentionally never committed; they are now explicitly inventoried, not silently treated as missing.

The original full M1000 descriptor-access trace is **MISSING_SOURCE**. A new execution of unchanged Stage8 scoring records the actual100x1000 shortlist access sequence, so the declared100-query B2 study is complete. This does not reconstruct or invent the missing full2000-query historical request sequence.

Before timing, `stage21_measurement_policy.json` is written and fsynced. It fixes the source commit/hashes, exact100 query IDs,173 historical early-stop attempted pairs,20 uniform cloud samples per robot, M1000, all frozen GICP settings, cache scenarios, compression, repeats, warmups, resource limit and comparison scope. Runtime uses the original teammate environment: Python3.10.20, NumPy2.2.6, SciPy1.15.3, Open3D0.19.0 on Xeon Gold6530. OMP/OpenBLAS/MKL are fixed to one thread for the controlled new benchmark; this is not algorithm tuning or evidence of a speedup over historical timing conditions.

Ten retrieval-query warmups, three GICP-pair warmups and three descriptor-frame warmups per robot are excluded. Two measured frontend repetitions are required; no favorable-ID selection or post-hoc expansion occurs. A predeclared1800-second frontend workload cap stops incomplete execution instead of scaling it into a measured full-run claim.

## 3. Application message schema and accounting

`stage21_wire.py` implements a deterministic versioned little-endian binary format, not pickle. The48-byte application header contains magic/version/type, sender/receiver/owner IDs, original KF, logical event, original int64 nanosecond timestamp, payload length and CRC32. Data-owner ID is independent of sender; request metadata names the target frame. The frozen coordinate convention, calibration, SI units and schema are assumed shared out of band; their deployment/synchronization costs are not invented.

All12 requested categories are specified and schema-tested. Only architecture-required categories become simulated sends. ACKs, global pose corrections and map distribution messages are not automatically added without a declared send policy. Their absence is not a claim that real deployed systems need no such traffic. MAP_CHUNK buffers from actual first-bridge/final Stage20 maps are measured separately, never taken from capped browser geometry.

Byte accounting uses actual encoded buffer lengths. Array nbytes, NPY container size, Python-object memory, ROS bag ZIP size and network packet size remain distinct. Optional compression is lossless zlib level1 on the entire application message, with independent compression/decompression timing and exact round-trip checks. Offline provenance hashes are log metadata, not secretly transmitted fields omitted from the byte count.

Measured application buffers:

| Representation | Data/body | Complete message |
|---|---:|---:|
| Ring Key |20 float32 =80 B|128 B|
| Full SC |20x60 float32 =4800 B|4848 B|
| Full-SC request |0 B body, target in header|48 B|
| Candidate request |4 B (M/TopK/database)|52 B|
| Cloud request |1 B field-count instruction|49 B|
| XYZ cloud,65536 points |786432 B coordinates +8 B count/fields|786488 B|
| XYZI cloud,65536 points |1048576 B coordinates/intensity +8 B metadata|1048632 B|
| Frozen loop |201 B endpoint/matrix/noise/Huber body|249 B|
| Initial/relative odometry update |185 B kind/previous-ID/matrix/noise|233 B|

The factor matrix and noise arrays use float64. Pose3 tangent sigma ordering stays `[rx,ry,rz,tx,ty,tz]`; rotation sigma5degrees, translation1m, loop Huber1.345, and the R1 prior remains1e-6. All75 real loop messages round-trip without changing measurements or noise.

## 4. Point-cloud compatibility and payload measurement

All4280 B0 clouds are actually loaded, encoded, decoded and checked; total cloud cost is not extrapolated from the40 representative samples. Original native files are float32 XYZI,65536 points and1048704-byte NPY containers in these samples. All sampled raw point counts are identical: size diversity is **not available** and is not fabricated by cropping or browser point caps. Geometry still produces varying compressed sizes and registration work.

Frozen GICP consumes XYZ only but its storage loader enforces Nx4. Therefore XYZ transport requires an explicit receive adapter to Nx4 with a zero unused-intensity column. Forty predetermined original/transported clouds produce bitwise-identical consumed XYZ, cropped/voxelized point arrays and SC descriptors. An actual unchanged-backend original-versus-adapted pair gives zero matrix and quality difference. **XYZ transport does not preserve original intensity**; it is lossless for the consumed coordinate fields, not a claim of all-field XYZI losslessness. The separately measured XYZI representation preserves all four fields. No voxelized or visualization-capped cloud silently replaces scientific input.

Payload framing/CRC, envelope decode, compression and decompression timings are separate from float-array conversion/copy timings in `payload_preparation_timings.csv`. Envelope decode returns the validated payload; typed array views and the receiver's Nx4 adapter are separate operations, not hidden complete receiver-pipeline timings.

## 5. Fixed architectures and communication results

The protocol contract explicitly records operation locations, triggers, required data and cache boundaries. No map-correction publication cadence or unnecessary control messages is invented.

| Mode | Declared workload and cache | Messages | Uncompressed application bytes | zlib1 bytes |
|---|---|---:|---:|---:|
| B0_FULL_CLOUD_UPLOAD |4180 R3 prebuild clouds +100 R1 queries; central full-cloud retention|4280|3366168640|2569576316|
| B1_FULL_SC_DESCRIPTOR_FIRST |same workload; all full SC upfront; received-cloud cache|4898|224467723|157901598|
| B2_RING_KEY_HIERARCHICAL |same workload; Ring Keys upfront; unbounded full-SC response cache|13326|224897867|158460015|
| B3_LOOP_FACTOR_ONLY_ORACLE |different Stage20 workload, frontend excluded|5871|1369143|1000733|

These totals sum **measured application buffers over protocol-simulated send events**. They are not measured wireless traffic, total packet bytes, deployed end-to-end costs or evidence that archived75 can be discovered with this budget. B3 is never used as a percentage-savings comparator to B0–B2.

B0 uses the cloud-response encoding for unsolicited uploads; no fictitious request is added. B1/B2 upload queries and send explicit candidate requests; central GICP then requests clouds in the same frozen173-attempt order. Cloud caching gives259 unique responses (100 R1 +159 R3), representing203700392 complete response bytes in both B1 and B2. Repeated cached clouds are not counted twice.

B3 has5796 initial/relative local update messages plus75 loop messages. One initial R3 pose message initializes node metadata, not a second central prior. Thus5871 messages correctly coexist with the unchanged5870-factor graph:5794 odometry +75 loops +one prior. Loop messages alone total18675 B; odometry/initial messages total1350468 B. B3 excludes frontend costs and assumes an oracle producer already knows the loops.

`communication_summary_by_mode.csv` separates uplink/downlink, database synchronization/query phase, category bytes, maximum message, per-workload-keyframe cost and amortized per-factor cost. B0 pays3287519840 B at database synchronization and78648800 B in the sampled query phase. B1 pays20264640 B at synchronization and204203083 B during queries. Therefore the initial-sync comparison must not be confused with incremental query traffic. Recorded-time rates use the actual100-query span999.535027661s, a sparse sample, not an invented continuous2Hz query schedule. Separate full-grid publication-frequency modelling uses original sensor timestamps and is explicitly not measured network bandwidth.

## 6. Ring-Key hierarchy and cache sensitivity

The80-byte Ring Key cannot perform full-SC exact ranking. The actual100-query M1000 scoring trace requires100100 descriptor accesses including query descriptors.

| Descriptor cache | Hits | Misses/full-SC responses | Total protocol bytes |
|---|---:|---:|---:|
| Unbounded |95886|4214|224897867|
| LRU1000 |47377|52723|462397931|
| LRU128 |220|99880|693278603|
| None |0|100100|694355723|

Unbounded caching eventually fetches4114 of4180 R3 descriptors plus100 query descriptors: approximately the whole reference database. Cache cold-start, repeated requests, request framing and cumulative bytes therefore matter. B2 is slightly more expensive than B1 in this declared amortized workload, despite much smaller initial publication. No improved cache or candidate-selection policy is selected in Stage21.

The unbounded model's maximum descriptor-data residency is20227200 B; its Python ID-index shallow-size proxy is separately701976 B. The latter is the recorded `sys.getsizeof` sum, including repeatedly counted shared value references, not a deduplicated ownership/allocation measurement. These are modelled payload residency and an object-size proxy, not actual remote cache RSS or transmitted bytes. Cache dictionary checks are instrumented separately. Physical serializer buffers use an LRU16; that local implementation buffer is not confused with the simulated receiver cache.

## 7. Actual frontend timing and historical reference

New runs execute unchanged SC extraction, Ring-Key construction, cKDTree M1000, exact SC/yaw scoring, stable Top3 sorting and direct teammate GICP calls. All100 query Top3 IDs/shifts match frozen Stage9; descriptor extraction matches frozen caches bitwise. Historical rank-ordered early stopping fixes173 requests per repetition, not a new selection policy. Across200 measured queries /346 calls, quality-gate decisions agree with historical records.

| Operation / conditional scope | New mean wall ms | New median / p95 ms |
|---|---:|---:|
| SC extraction, R1 frame (n200)|1.624028|1.656805 /1.770325|
| SC extraction, R3 sampled frame (n40)|1.673763|1.675849 /1.788489|
| M1000 retrieval/query (n200)|20.435896|19.822379 /25.480969|
| GICP coarse+fine/attempt (n346)|86.836912|70.456951 /169.027607|
| manifest/index/CSV/other harness overhead per attempt|225.440720|217.732901 /282.377969|
| Rank1 sequential harness/query|331.262856|312.139642 /412.790203|
| Frozen Top3-trace sequential harness/query|570.253815|345.786393 /1177.452674|

The integrated harness values come from actual same-invocation clocks, never from adding unrelated benchmark averages. Nested instrumentation wraps the unchanged loader, per-cache-miss preprocessing and actual Open3D coarse/fine calls. Exclusive preprocessing subtracts only the load interval nested inside that **same call**; it does not subtract historical means. The harness overhead includes repeated index CSV reading/path checks and output handling and is not mislabeled GICP compute. Additional verification-decision arithmetic is a separately labelled microbenchmark.

Authoritative historical references remain Stage8 fixedM1000 mean17.403548ms and exhaustive106.473ms; Stage9 original100-query Rank1/Top3 integrated means387.851245 /672.204225ms. New timings do not overwrite these results or demonstrate a method speedup. Frozen GICP parameters remain voxel0.75m, crop XY range2–50m/Z[-3,5]m, coarse3m/fine1m,30 iterations, quality threshold0.6091.

Per-operation process CPU time, wall mean/median/p90/p95/max and sample counts remain distinct. Raw-field transport preserves algorithm inputs but does not prove a new online accuracy result. Existing historical offline labels are consulted only after sample freeze for coverage:92 true/8 false Rank1 pairs. The173 attempts cover ranks1/2/3 with100/40/33 calls and67 accepted/106 rejected outcomes. Rank3 successes are absent in this fixed sample; no post-hoc samples are added to improve coverage.

## 8. Backend attribution — Stage20 only

No iSAM2 rerun or graph reoptimization is needed. `backend_cost_summary.csv` imports the actual Stage20 event trace, separating local-chain processing, central updates, first fusion, estimate extraction, map publication, integrity auditing and offline diagnostic batch LM.

- Local-chain update median: R1/R3 approximately0.028869 /0.028871ms; these are not raw EKF estimation costs or physical robot measurements.
- Odometry-only iSAM2 median/p95: R1 9.421286 /23.412943ms; R3 0.057821 /22.606350ms.
- Loop-event iSAM2 median/p95:0.520414 /38.174487ms (70 release events,75 constraints).
- First fusion:6.252478ms graph construction,3.271434ms update,0.424107ms extraction,132.434193ms map publication; measured service subtotal142.486985ms.
- Nonzero map-publication events (n74): median1565.260211ms, p951741.766438ms, max3738.483418ms. These are conditional publication costs, not every-event latency.
- Estimate extraction: median2.563636ms over5529 initialized events. Full finite/integrity auditing has its own separately reported cost and is not hidden in an online solver claim.

Stage20's service subtotal excludes separately logged integrity analysis and batch diagnostics. Its deadline flag checks update-only latency against actual next combined-event intervals, not a fixed0.5s for nearly tied events or a complete pipeline deadline. Offline batch LM is deduplicated by checkpoint before counting; robot-specific diagnostic rows are not counted as extra solves. Historical Stage20 process CPU time was not recorded and remains NOT_MEASURED. Joint numerical batch disagreement and all historical readiness limitations remain unchanged.

## 9. Dual-stream availability audit

The6000 historical candidate endpoints are checked against each robot's own Stage20 retained start and deterministic event ordering. Results:203 available at query time,5016 future candidates,331 outside the retained candidate segment and450 outside the retained query segment. Among archived clean75, only1 candidate is already available at its query time;74 are future. No GT enters this timing audit.

This explains why the prebuilt-database frontend cannot be described as occurring causally inside Stage20. Endpoint-gated archived factor insertion does not implement online buffering, retrospective search or rediscovery. No such mechanism is added here.

## 10. Bottlenecks and interpretation

Within their separately measured scopes, the largest costs are conditional map publication (~1.6s), repeated harness/index/I/O handling (~225ms/attempt), and geometric registration (~87ms/attempt). SC retrieval (~20ms/query) and extraction (~1.6ms/frame) are smaller here. These per-event/per-attempt figures are not combined into a fabricated end-to-end ranking. Optional point-cloud compression adds its own measured CPU cost and may also be material.

Potential communication bottlenecks are raw cloud synchronization/verification responses, repeated full-SC fetches for M1000 scoring, and cache-thrashing request/response/framing overhead. Oracle factor messages are small but exclude the work required to discover them. Independent local maps do not make this architecture decentralized.

## 11. Validity, limitations and missing evidence

- Original cache/index files and historical algorithms/results/demos remain byte-identical to their pre-Stage21 contents, including preexisting unrelated working-tree edits. Context documents may receive append-only Stage21 notes.
- Serialized buffers are measured; send schedules and caches are protocol simulations; TCP/IP/link packet overhead, network bytes actually transmitted, wireless bandwidth, latency and packet loss are **NOT_MEASURED**.
- All new timing occurs on one server with controlled threads. OS page cache is not flushed; “first observed in this process” is not a physical cold-storage measurement. Actual code wrappers and harness output overhead are included where specified.
- RSS is process high-water, not a per-component allocation delta. The measured frontend peak is527.8984375MiB. Process CPU/wall ratios are utilization proxies, not energy. Physical robot compute, energy and sensor/EKF generation costs remain unavailable.
- Point encoding preserves consumed XYZ, not unused intensity; all-field XYZI is separately measured. Synthetic unit fixtures are not real send events.
- The original exact2000-query full descriptor-access trace is missing. Full-run historical traffic remains incomplete, although the new100-query access trace is measured and complete. No scaling of the sample is presented as measured full-run time.
- Map/correction send cadence and deployment calibration/configuration exchange are unspecified; no bytes or transmission rates are fabricated for them.
- One payload-report argument and a NumPy-boolean JSON conversion were corrected during implementation. The completed4280-cloud measurements were retained; map-report completion is recorded separately and original catalog wall duration is null, not reconstructed. No algorithm parameters, samples, numerical acceptance limits or measured cloud byte values were changed.

## 12. Outputs, reproduction and independent validation

Run `scripts/run_stage21_cost_baseline.sh` from repository root. Set `CUMULTI_STAGE21_OUTPUT_ROOT` to a **new** directory for an independent reproduction. Existing experiment outputs are refused. Required existing data and the original teammate virtualenv remain server prerequisites; no hidden uncommitted helper is required. Source/input hashes and frozen sample IDs make each stage inspectable.

All requested policies, schemas, metrics, architecture tables, cache sensitivity, availability audit, decision, validation report and10 figures are under `outputs/cumulti_v1/21_cost_baseline/`. The full548475-row application-message trace is approximately143MiB and remains server-only with SHA256 and byte size in `message_trace_manifest.json`; the first2000-row preview is explicitly named partial, never substituted for the full trace. Independent audit streams the full trace, reconciles every architecture/category/count/byte total, recomputes cache misses, verifies original input hashes, and independently rebuilds190 sampled encoded/compressed buffers. Raw LiDAR/ROS archives, large full trace and intermediate GICP pair files are not committed.

## 13. Stage22 hypothesis — not implemented

The best-supported hypothesis is that a future budget-aware objective must explicitly include **full-SC cache residency/misses and expected geometric-verification cloud cost**, rather than optimize Ring-Key upload bytes alone. A causal available-data protocol, fixed-quality constraints and physical resource assumptions must precede any new selector or communication-saving claim. Preprocessed/lossy cloud representations need independent accuracy validation and cannot be called zero-quality-loss by assumption.

`stage22_pareto_input_SEPARATE_SCOPES.csv` preserves cost units, workload labels, quality-equivalence scope and unavailable fields; it is not an accepted joint online Pareto frontier. No candidate policy, compression optimization, distributed optimizer or Stage22 experiment is selected or started. **STOP AFTER STAGE21.**
