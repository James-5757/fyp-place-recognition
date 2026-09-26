# CU-Multi Stage 10.2 PGO convergence and loop-policy selection

Stage 10.2 preserves the frozen Stage-10.1 outputs and Stage-9.1 transforms.
Both policies reuse non-GT EKF IMU/GNSS odometry, the `os_sensor` graph frame,
the official URDF IMU-to-LiDAR transform, and `Z_qc = X_q^-1 X_c`. GT was loaded
only after `pre_gt_policy_decision.json` was written.

`TOP3_SANITIZED` is the exact 139-edge Stage-9.1 input. `RANK1_SANITIZED` was
made from 1,233 frozen Rank-1 GICP registrations satisfying quality >= 0.6091,
using the identical Stage-9 temporal-clustering rule; it contains 120
representatives. The policies share 103 exact edges. Their initialization edge
passes at approximately 7.32e-15 m / 2.51e-15 degrees.

Robust PGO was tested only at the registered schedule 25, 50, 100, and 200, with
unchanged noise, Huber delta, outer iterations, and map sampling. Neither policy
formally converged by 200. Consequently the pre-GT demo decision is
`NO_POLICY_READY`; no live demo is authorized. This conclusion is independent of
offline GT evaluation.

At the final non-converged states, symmetric map-NN median/p95 distances are
9.556/142.640 m for single-loop, 2.962/115.283 m for Top-3, and 2.875/121.945 m
for Rank-1. Offline-only joint ATE is 28.595 m, 27.609 m, and 29.029 m,
respectively. The one non-robust diagnostic per policy is not used to choose a
deployment policy.

The future integration provenance remains
`woshanli351-afk/fyp-lidar-registration` at supplied commit
`e9f3781b46a6f8c52bedc254bf7b2a24d14a7731`, using
`T_query_from_candidate` (`p_query = T_query_from_candidate * p_candidate`).
It did not replace any frozen Stage-9.1 edge in this experiment.
