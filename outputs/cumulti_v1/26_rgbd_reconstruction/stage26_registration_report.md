# Stage26 bounded RGB-D registration

Recovery gate passed for all 748 assets. Message provenance and SHA256 are in the export receipts.
The boundary cohort uses the original Stage24E fit/holdout frame IDs. For the four non-adjacent cases, the candidate ranking and alternating split were frozen before decoding. The baseline uses identity when no frozen fit-only PnP proposal is available.
Residual columns are directional medians among held-out correspondences within 12 cm. Inlier counts and overlap columns retain the two directions. A depth pass on an aggregate submap does not by itself qualify an edge.

| Method | Trials | Aggregate depth passes on independent frames | Qualified bridges | Non-adjacent qualified |
|---|---:|---:|---:|---:|
| baseline | 28 | 13 | 1 | 1 |
| fpfh_ransac | 28 | 15 | 0 | 0 |

Frozen Stage24E baseline depth-gate decisions reproduced: 24/24.
Stage24D components: 22. Temporary diagnostic graph: 22.
Largest component valid RGB-D frame coverage: 19.52% → 19.52%.

Rejected method-case outcomes:

- DEPTH_GATE_FAIL: 22
- INDEPENDENT_FITS_OR_SE3_INCONSISTENT: 19
- INSUFFICIENT_INDEPENDENT_FRAMES_OR_TIME_SPAN: 8
- REUSED_VALIDATION_FRAME: 6
- INCONSISTENT_CYCLE: 1

Independent repetition failures (a repetition may fail several checks):

- baseline, 33 attempts: HELDOUT_DEPTH_GATE=17, SE3_DISAGREEMENT=14, NO_INDEPENDENT_FIT=1, WEAK_SPATIAL_SPREAD=1
- fpfh_ransac, 40 attempts: SE3_DISAGREEMENT=37, HELDOUT_DEPTH_GATE=30, WEAK_SPATIAL_SPREAD=1
- robot1:0->2 (baseline) passed independent validation but disagreed with the frozen path by 0.200321 m and 1.371 deg (limits 0.2 m and 3 deg).

No bridge was admitted and no merged preview was generated. FPFH RANSAC increased aggregate depth passes but produced no independently repeatable bridge. The baseline depth parity supports the implementation convention check. The held-out gates cannot by themselves distinguish absent physical overlap from a wrong proposal. No threshold sensitivity study was run.

Next experiment: rank boundary windows by measured depth coverage, spatial spread and independent temporal support. Expand the windows only within their original epochs, freeze a new disjoint split, and test depth-backed RGB feature matches as a coarse initializer under the same gates.

Full-recording reconstruction remains NO-GO. The experiment is deliberately bounded.
