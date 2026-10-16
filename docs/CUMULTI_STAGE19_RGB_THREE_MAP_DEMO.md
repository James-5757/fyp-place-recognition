# Stage 19 — persistent three-map dashboard / authentic RGB

Accepted baseline: `bf30e1a23ee7b7ccb58d095949d443ab0ee2a7dd`.
Finalization: 2026-10-16. Functional/data validation PASS; unrestricted foreground
smoothness remains NOT VERIFIED.
New presentation lives in `demo/stage19_rgb_three_map/`. Stage18/18.1 frontend,
data, reports and numerical experiments are preserved byte-for-byte. No new
SC, GICP, optimizer, loop selector, Visual SLAM, distributed or dual-stream run.

## Data audit and export

Both original Main Campus R1/R3 RGB ZIPs exist under the requested read-only
raw paths. Stage12C had validated their ZIP CRC/SQLite/count integrity; the
Stage19 single DB-member expansion again reads through EOF (CRC checked).
No download was needed. ROS2 SQLite3, exact topics
`robot1/camera/color/image_raw`, `robot3/camera/color/image_raw`, type
`sensor_msgs/msg/Image`. CameraInfo topics are the corresponding
`robotN/camera/color/camera_info`, with `plumb_bob` distortion. Native images
are `rgb8`,1280×800,step3840, with robot-specific `camera_color_optical_frame`.
Intrinsic matrices/distortion and complete bag topic metadata are retained in
the JSON export report. They describe the native image, not the resized JPEG.
R1 calibration report confirms mounting_plate→camera_link fixed zero rotation;
Robot3's physical forward-axis mounting is not independently calibrated here.
No RGB ray projection or visual pose estimation is performed.

Measured bag-time median-interval rates: R1 10.0145Hz, R3 10.0272Hz—not2Hz.
Prototype: up to100 temporally distributed real images per robot (R3's smaller
candidate set is included), decoded and compressed before full export. The
prototype projected47,941,875bytes, below the declared120MiB gate. Prototype
costs175.50s R1 /378.09s R3 include mandatory temporary DB expansion, not just
JPEG encoding. Actual full counts/costs are recorded, not inferred, in
`rgb_full.json` and the independent validation totals.

Full export: R1 5,686 JPEGs /50,053,427bytes /225.121s; R3 71 JPEGs /
652,192bytes /361.352s (including expansion). Total50,705,619bytes,48.36MiB.
Full export adds causal predecessor frames to the prototype's nearest/base
selection, so its R1 count is higher than the prototype projection; the actual
size still passes the unchanged120MiB gate. All files decode and match SHA256.

Reusable exporter: `src/demo/export_stage19_rgb.py --mode prototype`, then
`--mode full`, in the isolated existing Python environment. It uses SQLite
message-index queries rather than holding raw image messages in RAM; only one
temporary DB per robot. Width384,aspect preserved,JPEG quality50,no synthetic
images,undistortion or image-plane registration. R1 exports a roughly5Hz base
plus exact nearest and causal keyframe associations; deduplication produces
a variable-rate retained stream. R3 exports explicit historical candidate
observations plus the initial reference observation, not a second live video.

## Clock and scientific causality

Frozen LiDAR keyframe timestamps and prior RGB synchronization use ROS2 bag
timestamps. Stage19 preserves this convention and the original RGB header
timestamps separately; all original ns values are decimal strings, avoiding
JavaScript's >2^53 integer rounding. Manifest rows carry original RGB index/ID,
bag/header ns, associated original LiDAR KF/ns,absolute delta,source archive,
dimensions,bytes and SHA256. Full CSVs and JSON retain exact associations.

Full nearest-bag error statistics are in `rgb_full.json`; they are bag-clock
association errors, NOT sensor hardware synchronization accuracy. The prototype
found header minus bag medians around−107ms/−109ms,with larger outliers. Thus
recording/transport delay is a real limitation; no undocumented clock correction
is applied. This stage cannot establish calibrated exposure-time alignment
against a LiDAR sensor header clock that the frozen grid did not preserve.

Frozen2Hz nearest-bag RGB median/p95/max errors(ms): R1 23.354/44.975/69.070;
R3 27.853/47.273/127.949. Actual R1 causal display age at every retained LiDAR
KF is median38.903,p9598.738,max130.517ms. The full exported R1 header−bag
median is−109.449ms,range−764.123 to−89.886ms; R3 median−109.319ms,range
−334.317 to−101.838ms. No exported header is later than its bag timestamp.

R1 playback chooses **floor(exported bag timestamp≤selected bag clock)**
(also rejecting any frame whose original header timestamp lies in the future),
not the potentially future nearest frame used by historical visual experiments.
This is an observational display choice, not a rerun/change to those methods.
Between two LiDAR keyframes, scientific map state stays at the earlier arrived
KF while RGB may advance strictly before the next KF. Event phases freeze the
clock at that query's exact timestamp. No scientific graph node, vertex or loop
measurement is interpolated. Async seek generations prevent older requests
from replacing the newly selected frame. Loading/missing/error states are
explicit. R3 is identified by historical candidate KF (or initial reference
KF234),with its own recorded timestamp; there is no claimed shared live clock.

## Dashboard and real states

One existing Three.js r170 renderer uses three scissored scenes/cameras, with
independent local orbit/pan/zoom surfaces and a global surface. All three labels
and maps persist after fusion. R1 local geometry uses frozen local Pose3 and
arrived stride10 LiDAR scans,180-point deterministic per-scan display cap;
R3 uses its frozen PRE_FIRST_LOOP independent-local map. Neither is replaced
by optimized common-frame poses. R1 local trajectory uses a causal GPU draw
range; later cached map chunks are hidden when seeking backward.

Global remains unaligned/empty until first accepted loop. R1KF283↔R3KF252
gets candidate → actual frozen GICP inset → accepted loop/common initialization
→ first persistent joint estimate. Later maps use the corresponding real saved
post-loop/checkpoint state. No future FINAL morph, no new backend update.
Displayed accepted count and saved state K/KF are distinct and explicitly
labeled during the accepted-before-backend subphase. GICP values are not exposed
in the preceding candidate-only phase. K2/K5/K10/K20/K40/K75/FINAL jumps are
supported alongside pre-first/candidate/GICP/first-loop jumps.

Two compact authentic RGB views connect the first common place to the existing
SC/GICP phases. Labels explicitly deny simultaneous physical encounter/live
Robot3 operation. Blue R1/orange R3/green loop provenance stays visible. Global
top view is **gauge XY**, not a geodetic/gravity-level bird's-eye map.

## Why does the 3D trajectory look steep/uneven?

This is not adequately explained by terrain alone. Source code in Stage12D
constructs `inverse(first EKF pose @ IMU_FROM_LIDAR) @ current pose`, preserving
KF0-relative LiDAR axes. Z in that gauge is not guaranteed geographic altitude.
The renderer uses uniform metres/XYZ and does not exaggerate Z.

Frozen local Z spans are296.914m(R1)/363.867m(R3),and frozen final spans299.902m/
364.064m. Offline reference elevation spans are only7.426m/12.771m. An offline-only
per-run rigid(no-scale) diagnostic alignment gives Z-axis-to-reference-up angles
101.479°/101.676°; aligned vertical RMSE0.212m/0.457m and correlations0.9945/
0.9899. Together with the recorded transform construction, this strongly
supports **coordinate gauge orientation as the dominant apparent height
effect**. Real terrain/elevation variation exists but cannot explain hundreds
of metres. Remaining EKF/GNSS errors,attitude effects and graph deformation
are not ruled out. This is not proof of physically accurate maps or a new
extrinsic correction.

`trajectory_height_diagnosis.json/.png` contain this offline-only analysis.
Its GT files/diagnostic transform never enter the demo,graph,RGB selection or
solver. No trajectory flattening,gauge repair or historical-data rewrite occurred.

## Performance, validation and reproduction

Map buffers survive ordinary RAF frames; local scan buffers are cached once
and toggled causally, replaced global GPU geometry is disposed. Only two next
images prefetch;≤2 simultaneous decodes,24 Image objects LRU,≤6 queued jobs.
No sequence-wide startup decode. Seek tokens guard both state and images.
Browser HTTP cache is separate from the bounded decoded Image-object cache.

Independent saved-asset auditor:
`src/demo/audit_stage19_rgb_demo.py`. Pure RGB unit test:
`node --experimental-default-type=module src/demo/test_stage19_rgb.mjs`.
Browser checkpoint evidence/screenshots,prototype/full export costs,RGB hashes,
performance and final validation are in `outputs/cumulti_v1/19_rgb_three_map/`.
Browser performance is presentation-only,never SC/GICP/iSAM2 timing. Automated
RAF conditions and foreground smoothness limitations must be read with the
actual measurements; no universal60FPS/real-time assertion.

Two20-second local-HTTP Edge observations are preserved, not cherry-picked:
initial replay KF150→283:31.080FPS,CPU mean0.924ms,p951.800ms,max11.400ms;
fusion KF1014→1115:1.399FPS,CPU mean2.268ms,p954.600ms,max5.900ms.
The latter's low callback rate despite small CPU cost is consistent with the
previously observed automation/background scheduling limitation; the exact
cause is not established. Consequently **smoothness NOT VERIFIED**, including
the fusion phase. No60FPS or before/after speedup claim is made.

Observed startup413.3ms is this local browser/cache context, not remote WAN or
cold-disk performance. Session state-preparation/seek p95 was23.3ms in the first
sample and19.5ms in the second,max105.8ms; these exclude waiting for new images
to finish decoding/displaying and are not end-to-end seek latency. Both samples
had zero image decode errors. RGB counters and seek distributions are
session-cumulative (including checkpoint tests); RAF/CPU statistics cover the
20-second measurement windows. These are warm replay measurements after
checkpoint validation, not controlled cold-start or SLAM throughput benchmarks.
Foreground hardware-accelerated rehearsal remains necessary.

Serve the **parent demo directory** with `scripts/run_stage19_demo.sh`, open
`http://127.0.0.1:8021/stage19_rgb_three_map/`. Windows absolute-path command is
in the new README; serving only the child directory will break frozen shared
asset URLs. No ROS/GTSAM installation is needed by viewers. A Python HTTP server
is local by default; no public exposure or sharing/credential changes were made.

Stage13B INCREMENTAL_BACKEND_NOT_READY,Stage15 NO_INFORMATIVE_SPARSE_POLICY and
Stage17 diagnostic-line closure remain unchanged. Stop after Stage19.
