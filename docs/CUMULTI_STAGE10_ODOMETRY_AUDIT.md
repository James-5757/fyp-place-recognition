# CU-Multi Stage 10A: Local Odometry / SLAM Audit

## Status: PASS — non-GT odometry gate resolved

Robot3's IMU/GNSS archive was subsequently transferred and passed archive
integrity checks. It contains `robot3/ekf/odometry_map`
(`nav_msgs/msg/Odometry`, 210,153 messages), so the mandatory gate is resolved.
Stage 10 uses only the two EKF trajectories for graph construction; GT is still
excluded until offline evaluation.

## Frozen handoff checked

The Stage-9.1 handoff fixes the Stage-8.1 frontend commit
`c52354db7ddc74f7d42ab422486e5eb2922dcf50`, Stage-9 commit
`968546aa8269452fbe75f76ade973216d361f722`, Stage-9.1 audit commit
`cf2032f954e8fa3122856e58f59bda14608e2696`, and 139 sanitized candidate loop
constraints. These inputs were only read.

## Search performed

The audit searched `/home/cas/CU-Multi/`,
`/home/cas/fyp_robot13_geometry_20260915/`, and this repository for odometry,
LIO, SLAM, local/estimated poses, trajectories, pose graphs, GTSAM, and g2o.
It also read ROS2 metadata from the available Robot1 and Robot3 archives without
modifying raw data. The teammate workspace contains GICP evaluation code only;
no reusable local-SLAM trajectory or PGO implementation was found.

## Robot1: an admissible candidate exists

`robot1_main_campus_imu_gps.zip` contains the non-GT topic
`robot1/ekf/odometry_map` (`nav_msgs/msg/Odometry`, 100,279 messages). Its bag
duration is 1002.924100791 s, corresponding to approximately 100 Hz. A temporary
read-only decode of one sample confirms the header frame `robot1_map`, child
frame `imu_link`, ROS header timestamp, position, and XYZW orientation fields.
It is an EKF output from the IMU/GNSS recording, not the UTM CSV or ground-truth
`/tf`. It is therefore a candidate local trajectory for a later, unblocked
Stage 10 run. It has **not** been aligned to the frozen 2-Hz keyframes because
the required Robot3 counterpart is unavailable.

## Robot3: admissible source after transfer

`robot3_main_campus_imu_gps.zip` provides
`robot3/ekf/odometry_map` in `robot3_map -> imu_link`. It is the same non-GT
EKF IMU/GNSS product used for Robot1. Nearest-timestamp synchronization to the
4,180 frozen Robot3 keyframes has mean/median/p95/max error
2.925/3.043/5.017/9.255 ms. The Robot3 GT-relative pose bag remains excluded:
its `robot3/lio_sam/mapping/odometry` topic has zero messages and its path/TF
content is not used as odometry.

Robot1 nearest-timestamp synchronization to its 2,000 keyframes is
2.237/1.951/4.717/9.316 ms. Both topics are used with the URDF-calibrated
`imu_link`-to-`os_sensor` transform; this fixed physical extrinsic is not a
ground-truth-derived yaw correction.
