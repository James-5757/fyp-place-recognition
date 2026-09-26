# CU-Multi Stage 10.3 pose-graph solver methodology audit

Stage 10.3 audited available SLAM-native solvers before any new graph run. In
the `fyp_slam` environment, GTSAM, g2o, Open3D, Ceres, and pyceres are absent;
no GTSAM/g2o/Ceres package was found in the local pip cache. Open3D 0.19.0 exists
only in another project's virtual environment. Its pose-graph API does not expose
the frozen protocol's explicit Huber-wrapped loop Gaussian factor, so it is not a
methodologically equivalent substitute.

Therefore the audit status is **BLOCKED_NO_SLAM_NATIVE_SOLVER**. No optimizer was
installed, no custom SciPy fallback was used, and no full graph, GT evaluation,
or live demo was run in Stage 10.3. Frozen inputs and their provenance are
recorded for a future run after a documented, isolated installation of a suitable
SLAM-native solver (preferably GTSAM Pose3).

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
