# CU-Multi Stage 9.1: Integration Cleanup and Reproducibility Audit

Stage 9.1 is a cleanup checkpoint, not a new experiment. It does not rerun
Scan Context, GICP, retrieval, or latency measurements; it does not run PGO,
map merging, graph optimization, or visual methods.

## Corrections and provenance

The Stage-8.1 frontend commit is
`c52354db7ddc74f7d42ab422486e5eb2922dcf50`. A previous Stage-9 document had an
extra trailing `0`; the correction is documentation/metadata only.

The external teammate backend is deliberately not copied into this repository:
it is a separately maintained implementation. Its immutable Stage-9.1
provenance record, including SHA256, size, modification timestamp, runtime, and
frozen parameters, is
`outputs/cumulti_v1/09_1_integration_cleanup/gicp_backend_provenance.json`.
No earlier Stage-9 hash was captured. The recovery audit verifies that the
current backend modification time predates the Stage-9 GICP outputs; this is the
strongest available non-invasive version-consistency evidence and is recorded
explicitly rather than silently rerunning with a possibly changed backend.

## Recomputed confusion checks

The primary Robot1-to-Robot3 Top-3 output is recomputed directly from
`top3_early_stop_results.csv`: TP/FP/FN/TN = **1374/29/459/138**, with 1,833
offline-positive and 167 offline-negative queries.

The Robot3-to-Robot1 rank-1 sanity output is recomputed directly from
`reverse_direction_sanity.csv`: TP/FP/FN/TN = **1174/29/510/2467**, over 1,684
offline-positive and 2,496 offline-negative queries. Its precision is 97.59%,
recall 69.71%, and no-overlap rejection is 98.84%. This corrects the prior prose
arithmetic inconsistency; it does not change any GICP result or threshold.

## Candidate loop-constraint audit

`sanitized_loop_edges.csv` remains 139 rows. The audit verifies required IDs,
timestamps, Scan Context metadata, GICP metrics, finite transforms, normalized
quaternions, one explicit transform direction, correct Robot1-to-Robot3 IDs,
no exact duplicate rows, and `accepted_after_sanitation=true` for every export.

The retained convention is `T_query_from_candidate`:

`p_query = T_query_from_candidate * p_candidate`.

The unchanged SC yaw initialization is
`wrap((best_shift if best_shift <= 30 else best_shift - 60) * 6 degrees)`.
No 180-degree correction is introduced: the previous apparent offset is the UTM
quaternion-yaw convention established by the TF-based frame audit.

The 139 outputs are **sanitized candidate loop constraints**. Their SE(3)
transforms passed the frozen Stage-9 registration acceptance and sanitation
pipeline, but they are not verified-correct or guaranteed loop closures.
Stage 10 must evaluate robustness inside a separately frozen pose graph.

## Latency interpretation and handoff

The Stage-9 numerical measurements are unchanged. Rank-1 is mean-budget
compatible on the tested host (387.85 ms mean) but its p95 (538.40 ms) exceeds
the 500-ms/2-Hz budget. Top-3 early stop is 672.20 ms mean and 1301.32 ms p95,
so it does not meet the mean sequential 2-Hz budget. Stage 9 is not a fully
real-time 2-Hz deployment result.

The machine-readable Stage-10 handoff is
`outputs/cumulti_v1/09_1_integration_cleanup/stage10_handoff_manifest.json`.
It freezes the frontend, backend provenance, transform convention, loop-candidate
file, GT restriction, and the status `PGO: NOT RUN`.
