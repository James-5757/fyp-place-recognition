# CU-Multi Stage 10.1a reproducibility and artifact cleanup

This is an artifact and source-reporting cleanup, not a new PGO experiment. No
solver was rerun and the frozen Stage-9.1 139 loop constraints, graph policy,
initialization, IMU-to-LiDAR transform, map sampling, and offline-only GT policy
were retained.

The Stage-10.1 runner now derives `VALIDATION_REPORT.txt` and `summary.txt` from
the solver summaries. It reports a PASS correctness layer only after the
initialization-edge, extrinsic, frame, frozen-loop, GT-separation, and
pre-solver-objective checks pass. It reports `NOT_CONVERGED_AT_FROZEN_CAP` unless
both solvers converge, and retains the accepted historical `SYSTEM_RESULT=MIXED`
label without deriving it from GT.

All three 6,180-row trajectories pass the schema audit (2,000 Robot1 and 4,180
Robot3 rows, finite values, unit quaternions, complete keyframe IDs, and no
duplicates). They remain server-only because the approximately 1 MiB generated
CSVs are covered by the repository output-size policy; their paths, sizes,
columns, row counts, and SHA256 values are committed in
`trajectory_artifact_manifest.json`. The eight accepted figures are committed
and hashed. The four server-only PLY hashes match `map_artifact_manifest.json`.

The accepted numerical regression check is unchanged. Scientific status remains:
`CORRECTNESS_PASS`, `SOLVER_NOT_CONVERGED_AT_FROZEN_CAP`, and
`SYSTEM_RESULT_MIXED`. It is not live-demo ready and no GT-based tuning was
performed.

The future-demo handoff remains the teammate backend
`woshanli351-afk/fyp-lidar-registration` at supplied commit
`e9f3781b46a6f8c52bedc254bf7b2a24d14a7731`, with the interface
`T_query_from_candidate` and `p_query = T_query_from_candidate * p_candidate`.
This cleanup does not replace any frozen loop edge; a future Stage 10.2/live
integration may use that persistent backend only under a separately declared
protocol.
