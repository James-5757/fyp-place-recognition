# CU-Multi Stage 10.1 Pose-Graph Correctness

Stage 10.1 replays the frozen Stage-10 policy in a new directory and preserves the original result. It fixes only: cross-robot initialization (`S=X_q Z_qc L_c^-1`), the official URDF IMU-to-LiDAR extrinsic, and pre-solver objective bookkeeping. The initialization residual is 7.32e-15 m / 2.51e-15 deg. Graph nodes, loops and clouds use `os_sensor`; no GT-derived correction is applied.

Frozen policy is unchanged: 1 m/5 deg noise, Huber 1.345, 4 outer iterations, max_nfev 25, and unchanged map sampling. The same 139 candidate loops and highest-quality initialization edge are used. The teammate future-demo backend is `woshanli351-afk/fyp-lidar-registration` at `e9f3781b46a6f8c52bedc254bf7b2a24d14a7731`; its `T_query_from_candidate` interface is compatible, but Stage 10.1 does not replace frozen loops.

Correctness layer: PASS. Both solvers reach the frozen cap without convergence. Non-robust objective reduces 84.42%; robust reduces 99.67%. Joint ATE is 28.60 m single-loop, 20.66 m non-robust, 23.65 m robust. Robust map NN median/p95 is 2.920/127.071 m versus 9.556/142.640 m for single-loop. The result is mixed and not a validated fully converged system; no tuning or live demo is authorized.
