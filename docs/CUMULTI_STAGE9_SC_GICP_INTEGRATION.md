# CU-Multi Stage 9: Fast Scan Context–GICP Integration

## Scope and frozen components

Stage 9 integrates the selected Stage-8 fixed `M=1000` Ring-Key/KD-tree Scan
Context frontend with the teammate's existing `robot13_gicp.py` backend. It does
not change Scan Context scoring, candidate ranking, GICP, calibration, or any
Stage 4--8.1 result. No PGO, map merging, OpenCLIP, Cross-Max, CVTNet, VLM, or
GICP parameter sweep was run. Ground truth is excluded from retrieval, yaw
initialization, registration, quality gating, and duplicate suppression; it is
used only for offline labels and error analysis.

The frame gate is resolved in `CUMULTI_STAGE9_FRAME_CONVENTION.md`. Each output
edge is `T_query_from_candidate`, meaning
`p_query = T_query_from_candidate * p_candidate`.

## Method

- The unchanged Stage-8 frontend forms the exact M=1000 shortlist using a 20-D
  Ring Key and `cKDTree`, then applies the frozen exact Scan Context score and
  sector shift. Its top three candidates are written to
  `candidate_interface.csv` with IDs, timestamps, score, margin, shift, yaw
  initialization, and cloud paths.
- The signed Scan Context initialization is `best_shift * 6 degrees`, wrapped
  after mapping shifts above 30 to the negative equivalent. It is derived from
  the descriptor convention, not from ground truth.
- The existing teammate backend is reused directly with its frozen settings:
  voxel 0.75 m, coarse correspondence 3.0 m, fine correspondence 1.0 m,
  30 iterations, and quality `fitness / (1 + inlier_RMSE)`. The fixed acceptance
  threshold is 0.6091.
- The first accepted candidate in SC rank order 1--3 is selected. For export of
  sanitized candidate loop constraints only, temporal duplicates are clustered if adjacent accepted edges are
  within three query frames and five database-frame IDs; each cluster retains
  its highest-quality edge. This does not use GT and does not alter retrieval.

## Reproducibility and frontend comparison

The run uses the accepted Stage-8.1 frontend commit `c52354db7ddc74f7d42ab422486e5eb2922dcf50`.
Against the historical exhaustive candidate record, M=1000 Rank-1 agreement is
0.994500 and mean Top-3 set overlap is 0.994000. The changed shortlist caused
no loss of an offline-valid candidate among valid-overlap queries. Thirty-six
new candidate pairs were registered with the reused backend; otherwise existing
exact-pair backend results were reused.

## Results: Robot1 to Robot3

### Initialization ablation (2,000 rank-1 pairs)

| Initialization | Acceptance | Median latency | Median translation error | Median rotation error |
|---|---:|---:|---:|---:|
| Identity | 21.60% | 137.01 ms | 3.677 m | 168.02 deg |
| Scan Context yaw | 61.65% | 21.60 ms | 0.633 m | 4.612 deg |

This is a strong initialization effect under the frozen backend, not a changed
retrieval result. Among rank-1 candidates, accepted true loops have median
SC-to-GICP yaw difference 0.968 deg (p95 3.241 deg); the one accepted false
rank-1 loop has 7.855 deg.

### Rank-ordered Top-3 early stop

Of 2,000 queries, 1,403 have an accepted candidate: 1,233 at SC rank 1, 113 at
rank 2, and 57 at rank 3. The rank-2/3 path contributes 142 offline true-loop
recoveries. Offline evaluation gives TP=1,374, FP=29, FN=459, TN=138, hence
precision 97.93% and recall 74.96% over the predeclared `d_xy < 5 m` condition.
The mean number of backend calls is 1.710 per query in the complete evaluation.

Duplicate sanitation reduces 1,403 accepted temporal edges to 139 sanitized
candidate loop constraints across 139 clusters; 78 clusters contain more than one
edge and 1,264 redundant edges are removed. `sanitized_loop_edges.csv` has finite
numeric transform fields and unit-quaternion representation from the Open3D
matrix. These are registration outputs that passed the frozen Stage-9
acceptance/sanitation pipeline, not verified-correct or guaranteed loop closures;
their robustness must be evaluated inside a separately frozen Stage-10 pose graph.

The reverse Robot3-to-Robot1 rank-1 sanity direction remains deliberately
asymmetric and is only a diagnostic: TP/FP/FN/TN is 1,174/29/510/2,467 over
1,684 offline-positive and 2,496 offline-negative queries (precision 97.59%,
recall 69.71%, no-overlap rejection 98.84%). It must not be presented as a
symmetric benchmark result.

## Integrated latency measurement

The 100-query deterministic harness measures one sequential invocation of the
actual M=1000 retrieval followed by direct teammate `robot13_gicp.run()` calls,
including cloud access. It does **not** add separately measured average times.

| Pipeline | Mean | Median | p95 | Throughput | Mean / 500 ms budget |
|---|---:|---:|---:|---:|---:|
| Rank-1 | 387.85 ms | 365.28 ms | 538.40 ms | 2.58 q/s | 77.57% |
| Top-3 early stop | 672.20 ms | 416.61 ms | 1301.32 ms | 1.49 q/s | 134.44% |

This is a host-specific integration-harness measurement, not a deployment
claim. Rank-1 is mean-budget compatible on the tested host, while its 538.40 ms
p95 exceeds the 500 ms budget; Top-3 early stop does not satisfy the mean 2 Hz
sequential budget. Stage 9 must therefore not be described as fully real-time at
2 Hz.

## Outputs

Machine-readable Stage 9 artifacts are under
`outputs/cumulti_v1/09_sc_gicp_integration/`. The key lightweight/reproducible
files are the configuration and frame audit JSON, candidate interface,
initialization summary, Top-3 outcomes/confusion, yaw and duplicate analyses,
sanitized edges, latency CSV/JSON, validation report, and summary. Large
per-pair backend traces remain on the server and are intentionally not added to
Git.

## Stop point

Stage 9 stops after producing registration constraints and diagnostics. Do not
run PGO or map merging until a separately declared graph/robustness protocol is
reviewed.
