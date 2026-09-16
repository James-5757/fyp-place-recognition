# CU-Multi Stage 7.5 — same-view modality control and LiDAR azimuth audit

## Scope and controls

Stage 7.5 is a supervisor-requested diagnostic. Stages 4--7 are read-only. It
reuses the frozen Robot1 -> Robot3 protocol: 2,000 Robot1 queries, 4,180 Robot3
database keyframes, `d_xy < 5 m` offline positives, frozen Scan Context results,
and the existing Stage-5 OpenCLIP cache. No image was decoded or encoded for visual
features. No fusion, policy tuning, Ring-Key, GICP, PGO, CVTNet, or VLM was run.

GT yaw and position are used only after ranking to make predeclared offline strata.
The primary same-view subset is absolute nearest-positive heading difference
**<=10 degrees**; additional cumulative subsets <=5, <=15 and <=30 degrees were
declared before metrics were inspected. A separate <=10-degree position control
uses GT nearest-positive distance <2, <3 and <5 m. These are not deployable
filters.

## Same-view modality result

The <=10-degree subset contains **482** valid queries. Frozen SC achieves R@1/R@5
**1.000000 / 1.000000** (MRR 1.000000). On the same frozen Top-20 candidate
protocol, Single RGB is **0.906639 / 0.981328** (MRR 0.939558), RGB5 Mean is
**0.883817 / 0.970954** (MRR 0.919650), and Cross-Max is
**0.890041 / 0.962656** (MRR 0.919292).

SC has zero Rank-1 failures in this primary stratum. Single RGB has 45, RGB5 Mean
56, and Cross-Max 53. Consequently there are no RGB-correct/SC-wrong cases in
this stratum, while Single RGB has 45 SC-correct/RGB-wrong cases. The deterministic
case table uses the first five query IDs in each nonempty category; no visually
dramatic examples were cherry-picked.

The close-position control does not remove the difference. At <=10 degrees and
distance <2 m (437 queries), SC/Single RGB/Cross-Max R@1 is
**1.000000 / 0.913043 / 0.897025**. At <3 m (479), it is
**1.000000 / 0.906054 / 0.893528**; at <5 m, it returns the primary values.

Visual nearest-positive similarity rises at small heading difference: Single RGB
median similarity is 0.848595 at <=5 degrees and 0.841539 at <=10 degrees;
Cross-Max is 0.889561 and 0.881198. This does not make visual ranking equal to SC:
higher positive similarity can still leave visually similar non-positive candidates
above the positive.

## Actual LiDAR azimuth audit

This audits actual validated float32 XYZI PointCloud2-derived frames, not a sensor
specification. Twenty uniformly distributed frozen keyframes per robot (60 total)
were read from the processed LiDAR cache. Azimuth is `degrees(atan2(y,x))` wrapped
to [-180, 180), with 36 fixed 10-degree bins.

The criterion was declared beforehand: approximately full azimuth requires at least
34/36 occupied bins and longest circular empty gap <=20 degrees. Robot1, Robot2,
and Robot3 each pass on **20/20 frames (100%)**, with mean **36.00/36** occupied
bins and largest observed blind gap **0 degrees**. The same result holds for the
0--20 m, 0--40 m, and 0--80 m slices: all robots have mean occupied-bin fraction
1.0 and no observed gap in this sample. No systematic blind sector is observed at
this 10-degree occupancy resolution.

## Interpretation

The audit confirms a **sensor/FoV advantage**: processed LiDAR geometry is
approximately 360-degree in these samples, including at 0--20 m, and preserves
surrounding context under heading changes. It does not alone prove universal Scan
Context superiority. The same-view result adds separate evidence of **geometry
robustness**: SC still exceeds these generic visual methods when heading and
position are controlled. Finally, this is method-specific: Stage-5 OpenCLIP is a
generic visual representation, not a dedicated place-recognition model. The gap is
not a universal claim about all camera place-recognition methods.

Artifacts are in `outputs/cumulti_v1/07_5_supervisor_validation/`. Raw archives,
embedding caches, and Stage 4--7 outputs remain untouched.
