# CU-Multi Stage 10A: Local Odometry / SLAM Audit

## Status: `BLOCKED_NO_NON_GT_ODOMETRY`

Stage 10 stops at its mandatory odometry gate. No pose graph, map merge,
trajectory evaluation, or live-robot work was run.

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

## Robot3: no admissible local trajectory is locally available

The present Robot3 raw directory contains LiDAR, RGB, UTM GT CSV, and
`robot3_main_campus_gt_rel_poses.zip`; it does not contain a Robot3 IMU/GNSS
archive. The relative-pose archive has `robot3/lio_sam/mapping/path` messages,
but it is explicitly a GT-relative-pose archive and cannot be certified as an
independent local-SLAM estimate. Its `robot3/lio_sam/mapping/odometry` topic has
zero messages. The official CU-Multi topic documentation identifies the matching
pose archive as ground-truth path/odometry and `/tf`, so these topics are
excluded by the Stage-10 policy rather than being repurposed as odometry.

## Required next input

Download **only** the Robot3 non-GT IMU/GNSS archive:

`/main_campus/robot3/robot3_main_campus_imu_gps.zip`

to:

`/home/cas/CU-Multi/raw/main_campus/robot3/robot3_main_campus_imu_gps.zip`

Then rerun the Stage-10A audit to verify an EKF local odometry topic, its frame,
timestamp alignment to the 4,180 frozen Robot3 keyframes, and its non-GT status.
Do not use the GT-relative-pose bag, UTM CSV, or ground-truth `/tf` as a
substitute.
