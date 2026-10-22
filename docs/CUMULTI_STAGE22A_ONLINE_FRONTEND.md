# CU-Multi Stage22A — causal online inter-robot frontend

Final decision: **CAUSAL_ONLINE_GRAPH_READY**. Artifact-validation PASS does not imply backend
numerical readiness, physical correctness, or full verification if incomplete.

## Motivation and frozen scope
Stage21 found74/75 archived clean candidates unavailable at the historical R1
query time. This independent baseline uses no archived loops to select queries,
register pairs, initialize the graph or replace bad constraints. Stage8 exact
SC/RingKey/scoring, Stage9 teammate GICP, Stage12D odometry/URDF/noise,
Stage20 scheduler and Stage21 wire schema are unchanged. Historical results,
including prior incremental/batch discrepancies, remain frozen. No demo changed.

## Causal clocks and databases
5,796 arrivals: R1KF150–1999 (1,850), R3KF234–4179 (3,946). Each recording
subtracts its own first retained timestamp; ties use normalized ns/robot/KF.
These are separate recordings, NOT a simultaneous physical meeting. At every
event: actual LiDAR load → frozen SC extraction → query opposite arrived DB →
publish → append own descriptor. Current opposite KD-tree is rebuilt with its
maintenance cost measured. M=min(1000,N); TopK=min(3,M). Frozen KD ordering and
stable descending-score sort determine ties. Reverse new-frame queries naturally
permit retrospective discovery; no unbounded historical re-query was added.

Two independent real-scan passes made 11592 bitwise
descriptor checks. Scientific candidate hashes are identical:
`471483d0d7c3b2ad5def393d28bce8d961a3ed4f46e669369b55d7ba9b73f749`. Future candidates:0. There is one
empty-database query. Total first-pass retrieval=116.352113s;
descriptor extraction=9.836585s. Frozen descriptor
arrays are integrity references only, never causal database initialization.

## Actual GICP, pilot and coverage
The150-event pilot was uniformly selected by event index before outcomes.
Pilot full-cost projection=6347.878332s
(**MODELLED_PROJECTION**, not measured full time); frozen cap=7200s, including
new pilot attempts reused exactly once at their correct chronological event.
Coverage: 5796/5796 queries, actual calls=15267,
geometrically accepted candidates=1088, rejected=
14179. Measured total actual-call harness time=
6413.994504s. Every receipt contains actual quality,
fitness, RMSE, matrix, input IDs and load/preprocess/coarse/fine/other timings.
`call_wall_ms` measures unchanged teammate.run(), including manifest read,
full cloud-index validation, loading/preprocessing, registration and CSV write;
it excludes outer manifest creation, result parsing and receipt persistence.
It is NOT a full end-to-end pipeline latency. Full process wall is separately
measured in `gicp_full_process_observer.json` or
`gicp_full_invocation_wall.json`; the pilot projection is never substituted for it.
Frozen voxel.75m, XY2–50m, Z[-3,5]m, coarse3m/fine1m,30iterations and
quality fitness/(1+RMSE)>=.6091 are unchanged. Rank-ordered Top3 early stop.

The unchanged teammate cloud-index loader parses its existing mixed-metadata
keyframe CSV but consumes only KF/path fields; GT pose fields do not enter its
registration. New Stage22A indexing reads allowlisted ID/ns/path columns.
Runtime guards reject raw GT and historical loop/result paths. Opaque input
hashing is provenance, not oracle selection.

## Direction, causal duplicates and new discovery
GICP returns T_query_from_candidate. R3-query measurements are inverted as
complete SE(3) matrices to canonical T_R1_from_R3. Synthetic inverse,
composition and Pose3 factor tests pass. New admitted representatives=
288; causal duplicates=800. First qualifying
canonical pair wins within |deltaR1KF|<=3 AND |deltaR3KF|<=5; later qualities
never replace earlier representatives. Unlike historical whole-cluster max
quality sanitation, this policy is causal and predeclared. Geometric acceptance
is distinct from graph admission: finite proper SE3, retained endpoints and
causal duplicate screen. No additional GT-fitted gate.

First bridge (null means not run/found):
```json
{
  "discovery_event": 200,
  "normalized_time_ns": 50001923410,
  "query_robot": "robot1",
  "query_keyframe_id": 250,
  "candidate_robot": "robot3",
  "candidate_keyframe_id": 235,
  "canonical_robot1_kf": 250,
  "canonical_robot3_kf": 235,
  "translation_residual_m": 1.464821375527116e-14,
  "rotation_residual_rad": 2.802531543882101e-17,
  "common_frame_transform": [
    [
      0.7065163738039425,
      -0.7076934238603759,
      -0.0021520621985807314,
      7.34997606439574
    ],
    [
      0.7076964662352445,
      0.7065153282038994,
      0.0013426433197370365,
      -16.219132856942927
    ],
    [
      0.0005702850825774972,
      -0.0024716063026267165,
      0.9999967829634305,
      -1.4996215393483254
    ],
    [
      0.0,
      0.0,
      0.0,
      1.0
    ]
  ],
  "archived_id_forced": false
}
```
After decisions were frozen, exact overlap with archived75=
6; newly admitted exact pairs outside that archive=
282. Archived overlap is NOT GT recall.
Offline query-based candidate/loop accuracy and false accepted pairs are in
`offline_frontend_accuracy.csv` and labelled offline tables. Positive definition
is d_xy<5m among already available opposite frames. Query recall and precision
use explicit denominators; duplicate suppression affects per-query admission.

## Persistent backend and numerical status
Backend status: **CAUSAL_ONLINE_GRAPH_READY**. Structural status:
PASS; numerical status:
PASS. Reason: original numerical agreement gates passed.
First newly admitted bridge initializes currently available prefixes only via
S=X_R1q Z_R1fromR3 inverse(L_R3c). One R1KF150 prior; local independent maps
remain unmerged before that event. One default persistent GTSAM4.2 ISAM2 instance
inserts subsequent odometry/new loops exactly once, no extra empty updates.
Frozen rot-first5deg/translation1m sigmas, Huber1.345 loops and1e-6 prior remain.
Frozen catastrophic-jump rule stops an unsafe backend rather than replacing
new measurements with clean75. Numerical comparison, when supported, uses
the identical NEW graph and one gauge, default frozen LM; gates0.10/0.50m
translation median/p95 and0.5/2deg rotation median/p95 are not relaxed.
No GT-based solver tuning. Maps use frame10/point16/voxel0.5m/range1–60m.
Offline ATE uses one JOINT rigid alignment; interval10 RPE is separately computed
for R1/R3 and the union of samples. Unsupported scores are NOT_RUN, not zero.

## Communication, computation and resource scope
B1 full-SC-first is a baseline, NOT a selected efficient policy. Each newly
arrived descriptor is actually serialized once; full-descriptor requests are0
because descriptors were uploaded, not because exact scoring uses only80bytes.
Lossless raw XYZI is requested on demand; unbounded logical central cloud cache
is separately modelled from actual per-call fresh64-entry teammate cache.
Measured serialized application bytes=6107977604,
compressed=4211894087; cloud cache hits/misses=
24738/5796. Compression is measured
zlib1, not a wireless throughput measurement. Original online timings are kept
separate from post-hoc serialization replay, which re-encodes actual raw inputs.
Actual network bytes/latency, robot CPU/memory/energy: NOT_MEASURED.

## Validation and reproducibility
Independent saved-artifact checks:133474 PASS. Run
`scripts/run_stage22a_online_frontend.sh` from repository root. For an independent
fresh reproduction set `CUMULTI_STAGE22A_OUTPUT_ROOT` to a new output directory;
the default invocation resumes/skips completed gates, never overwrites results.
It freezes new
policy first, supports hash-checked exact-once verification resume and never
reruns a completed gate. Unresolved in-flight intents block rather than repeat
an uncertain measurement. Large event/message/attempt traces and raw scans are
server-only; sizes/SHA256 and directory digests are explicit in the artifact
manifest. No uncommitted hidden experiment script is required.

## Limitations and stopping point
This is offline replay with causal logical availability and real algorithm
execution, not physically simultaneous robots, real-time deployment,
decentralized optimization or measured networking. High GICP quality alone is
not proof of true place identity. Keep separate structural, numerical and
offline-accuracy outcomes. No claims of all-pair GT recall or full coverage from
a pilot. Normalized discovery timestamps are logical event attribution in this
staged replay, NOT measured wall-clock completion latency; service-time backlog
and real sensor deadlines are not modelled. A pilot measurement executed out of
order is released only at its corresponding logical event. Zero logical delay
after both endpoints arrive is not a real-time performance claim.

The initial SSH-bound verification was interrupted after
2400 queries. Its
7289 durable receipts were hash-verified before
detached resume, with no uncertain in-flight attempt and no repeated registration.
The7200s actual-call budget was not reset. `checkpoint_recovery_audit.json`
preserves evidence. Resumed subprocess wall does not invent an uninterrupted
total latency. The optional initial pidfd timing observer was unavailable in
that Python build; the utility now has a quantified proc-polling fallback.
These are execution/reporting repairs, not threshold/noise/candidate changes.

Stage22B: CAUSAL_BASELINE_AVAILABLE_FOR_SEPARATELY_AUTHORIZED_STUDY; not started. STOP AFTER22A.
