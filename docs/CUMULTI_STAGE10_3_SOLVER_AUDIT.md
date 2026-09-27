# CU-Multi Stage 10.3 pose-graph solver methodology audit

Stage 10.3 audited available SLAM-native solvers before any new graph run. In
the `fyp_slam` environment, GTSAM, g2o, Open3D, Ceres, and pyceres are absent;
no GTSAM/g2o/Ceres package was found in the local pip cache. Open3D 0.19.0 exists
only in another project's virtual environment. Its pose-graph API does not expose
the frozen protocol's explicit Huber-wrapped loop Gaussian factor, so it is not a
methodologically equivalent substitute.

## Initial environment audit / resolved blocker

Stage 10.3 was initially **BLOCKED_NO_SLAM_NATIVE_SOLVER**: no suitable native
solver was installed in `fyp_slam`, so no optimizer, custom SciPy fallback,
full graph, GT evaluation, or live demo was run at that point. This is historical
context, not the final Stage 10.3 status. The blocker was subsequently resolved
by installing GTSAM 4.2 in the isolated `/home/cas/.venvs/fyp_gtsam` environment.
The existing `fyp_slam` environment was not modified, and the final native GTSAM
Stage 10.3 experiment was subsequently completed.

Stage-10.2 source cleanup: future reruns label the fallback figure
`Illustrative non-converged TOP3 state — NO POLICY SELECTED` when the pre-GT
decision is `NO_POLICY_READY`. The historical figure is retained unchanged and
must not be interpreted as a selected demo candidate. The source comments now
correctly state that `pre_gt_policy_decision.json` precedes every GT-based
diagnostic; subsequent non-robust ATE/RPE work is offline-only. Stage-10.2 RPE
values are combined values duplicated into per-robot rows and must not be cited
as per-robot RPE. A future native-solver experiment must calculate Robot1,
Robot3, and combined RPE separately.

The future-demo backend provenance remains
`woshanli351-afk/fyp-lidar-registration` at supplied commit
`e9f3781b46a6f8c52bedc254bf7b2a24d14a7731`, using
`T_query_from_candidate` (`p_query = T_query_from_candidate * p_candidate`).
# Stage 10.3 final native GTSAM audit

Stage 10.3 replaces the hour-scale frozen SciPy replay solve with a native sparse GTSAM 4.2 `LevenbergMarquardtOptimizer` replay in the isolated `/home/cas/.venvs/fyp_gtsam` environment. It preserves the frozen LiDAR/os_sensor graph convention, `Z_qc = X_query^-1 X_candidate`, the 139 Top3 and 120 Rank1 sanitized GICP loops, 1 m / 5 degree factor sigmas, Huber delta 1.345, and the Robot1 keyframe-0 anchor. The Pose3 tangent ordering is `[rx, ry, rz, tx, ty, tz]`.

The matrix crosscheck found an audit implementation error, not a measurement or solver error: the earlier 6.289883e-06 m discrepancy compared a Pose3 Logmap translation-tangent component with matrix translation. Correct matrix-to-matrix residual evaluation uses `Z^-1 X_q^-1 X_c` and the equivalent `Z^-1.compose(Xq.between(Xc))`; all 10 checked frozen edges pass the retained translation tolerance of 1e-8 m and rotation tolerance of 1e-8 rad.

Both policies pass a non-trivial 1,000-node, two-robot graph with a deterministic nonzero Robot3 initialization perturbation and at least ten frozen loops. Full 6,180-node native GTSAM runs are reproducible through `scripts/run_stage103_gtsam.sh`; it runs the frozen full solver and then the post-solve audit. The audit writes the pre-GT decision before offline ATE/RPE. Both policies are GT-free ready, with Rank1 preferred because it uses fewer inter-robot factors, has lower measured replay runtime, and retains the frozen online Rank1+GICP architecture. No live deployment/demo was run.

SciPy and GTSAM objective values are not compared directly because their factor normalization/implementation is not asserted identical. The valid comparison is operational: native sparse GTSAM reduced this CU-Multi offline full-graph replay from the frozen hour-scale SciPy runs to single-digit-second solves. GT metrics are offline-only and are not used for policy or parameter selection.

Artifacts in `outputs/cumulti_v1/10_3_solver_audit/` include source-level crosscheck diagnosis, nontrivial graph sanity results, loop residual diagnostics, map/trajectory manifests, GT-free pre-decision, separate Robot1/Robot3/combined 10-keyframe RPE, backend comparison and six final figures.

## Final status

**Stage 10.3 status: FINAL PASS.** Both frozen 6,180-node graphs were solved
using GTSAM. The pre-GT decision was `BOTH_READY_PREFER_RANK1`; offline GT was
evaluated only after that pre-GT decision. No live demo was run.
