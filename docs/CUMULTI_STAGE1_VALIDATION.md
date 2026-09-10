# CU-Multi Main Campus — Stage 1 validation

Completed on the FYP server using only `/home/cas/CU-Multi/raw/` as read-only input.  No KITTI output or raw CU-Multi file was changed, and no place-recognition, VLM, CVTNet, GICP, Scan Context, OpenCLIP, or Cross-Max algorithm was run.

## Raw data inspected

Each LiDAR, RGB, and relative-pose archive is a ZIP containing `metadata.yaml` plus a ROS 2 `rosbag2` SQLite (`.db3`) file; it is not an image-directory dataset.

| Robot | LiDAR | RGB | Relative pose |
| --- | --- | --- | --- |
| robot1 | `robot1_main_campus_lidar.zip` | `robot1_main_campus_camera_rgb.zip` | `robot1_main_campus_gt_rel_poses.zip` |
| robot2 | `robot2_main_campus_lidar.zip` | `robot2_main_campus_camera_rgb.zip` | `robot2_main_campus_gt_rel_poses.zip` |

LiDAR topics are `robot1/ouster/points` and `robot2/ouster/points`, both `sensor_msgs/msg/PointCloud2`. RGB topics are `robot1/camera/color/image_raw` and `robot2/camera/color/image_raw`, both `sensor_msgs/msg/Image`. The RGB bags also contain `camera_info` and RealSense metadata. LiDAR bags contain Ouster metadata and IMU. The relative-pose bags contain `/tf`, `robot{1,2}/lio_sam/mapping/path`; their odometry topic is recorded with zero messages.

The UTM GT CSV schema is `timestamp, x, y, z, qx, qy, qz, qw` after two comment lines. It has 19,998 robot1 rows and 22,299 robot2 rows (about 20 Hz). Analysis yaw is derived only from its quaternion with the standard Z-yaw formula:

`atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))`.

Archive metadata reports 19,998 LiDAR / 9,997 RGB robot1 messages over 1,000.605 s (about 20 / 10 Hz) and 22,299 LiDAR / 11,149 RGB robot2 messages over 1,115.796 s (about 20 / 10 Hz).

## Calibration and coordinates

`calib/robot_description.zip` contains `robot_description/urdf/robot1.urdf` and `robot2.urdf`. Each defines `base_link`, `chassis`, `mounting_plate`, `os_sensor`, `imu_link`, GNSS antenna links, and `camera_link`. The fixed chain includes:

- `mounting_plate -> camera_link`: xyz `(0.2, -0.02, -0.023)`, rpy `(0, 0, 0)`.
- `os_sensor -> ouster_riser_plate`: xyz `(0, 0, -0.05)`.
- `ouster_riser_plate -> mounting_plate`: xyz `(-0.02286, 0, -0.0175)`.
- `mounting_plate -> chassis`: xyz `(-0.35, 0, -0.08)`.

Processed clouds remain in their raw LiDAR sensor coordinate system: no GT transformation or candidate selection used GT. The GT coordinate values are treated as UTM easting/northing/elevation for trajectory analysis only. The robots begin roughly 2.2 mm apart in that frame and follow overlapping coordinate ranges, strong evidence of a common world/UTM frame; the precise UTM zone, datum, and GT measurement-frame origin remain to be confirmed against upstream documentation before metric alignment.

## Processed subset

`/home/cas/CU-Multi/processed_v1/` contains exactly 100 uniformly sampled LiDAR keyframes per robot, as float32 `.npy` arrays shaped `[N, 4]` with `[x, y, z, intensity]`, plus exactly 100 nearest matched RGB PNG files per robot. Original source message indices and nanosecond timestamps are retained in `sync_index.csv`; it also records nearest RGB and GT timestamps, signed offsets, pose and analysis yaw. `timestamps.csv` and `poses.csv` are derived per-robot views of that index.

To query ROS2 SQLite safely, the adapter temporarily expands one required `.db3` member under `processed_v1/.work`, decodes the selected frames, and deletes it before moving to the next sensor. It never extracts the raw archives into the final dataset tree.

Validation artifacts are under `/home/cas/CU-Multi/processed_v1/00_validation/`, including manifest, copied sync indices, robot and combined GT trajectory plots, three LiDAR plots and three synchronized RGB samples per robot, and text calibration/validation summaries.

## Verified results

| Metric | robot1 | robot2 |
| --- | ---: | ---: |
| Extracted LiDAR / RGB | 100 / 100 | 100 / 100 |
| Point fields | `x,y,z,intensity,ambient,range,reflectivity,ring,t` | same |
| First saved cloud | `(65536, 4)`, `float32`, finite | `(65536, 4)`, `float32`, finite |
| PNG decode | `1280 x 800 x 3`, `uint8` | `1280 x 800 x 3`, `uint8` |
| RGB nearest-frame absolute error, mean / median / p95 / max ms | 24.884 / 25.490 / 45.251 / 58.556 | 25.532 / 25.080 / 44.469 / 55.173 |
| GT nearest-pose absolute error, mean / median / p95 / max ms | 10.928 / 10.051 / 15.570 / 58.972 | 10.831 / 10.046 / 15.755 / 60.455 |

This is a Stage 1 adapter validation only. It is not a place-recognition benchmark and must not be used to claim retrieval performance.

## Reproduction

Activate `fyp_slam` and run:

```bash
python src/cumulti/prepare_stage1_validation.py \
  --raw-root /home/cas/CU-Multi/raw \
  --processed-root /home/cas/CU-Multi/processed_v1 \
  --keyframes 100
```

The script intentionally rejects any keyframe count other than 100. Do not rerun it over an existing processed directory unless deliberately replacing that derived validation subset.
