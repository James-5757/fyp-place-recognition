# CU-Multi Stage 10 Pose-Graph Convention

Each node pose `X_i` maps LiDAR sensor coordinates to the Stage-10 graph frame:
`p_global = X_i * p_sensor`. Sequential EKF factors use
`Z_ij = X_i^-1 * X_j`. The frozen Stage-9 loop is
`Z_qc = T_query_from_candidate = X_q^-1 * X_c`, therefore
`X_c = X_q * Z_qc` initializes Robot3 from the highest online-quality candidate.
Robot1 keyframe zero is fixed to remove gauge freedom.

EKF odometry is published for `imu_link`; the fixed URDF `imu_link`--Ouster
extrinsic is applied to create sensor poses. This includes its physical mounting
rotation and is not an artificial 180-degree correction. Loop measurements are
used unchanged. Scan Context initialization remains
`wrap((best_shift if best_shift <= 30 else best_shift - 60) * 6 degrees)`.

The graph policy is frozen in `outputs/cumulti_v1/10_offline_map_merge/graph_policy.json`:
1 m / 5 degree odometry and loop sigmas; no robust odometry kernel; Huber
delta 1.345 on loop factors only. These values were fixed before GT evaluation.
