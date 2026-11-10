# Full RGB-D data audit

This gate qualifies complete available inputs, not continuous tracking or successful fusion.

- robot1: 9997 RGB, 8765 depth clouds, 999.789161 recorded seconds; paired 8765/9997 within 25 ms.
- robot3: 20977 RGB, 19918 depth clouds, 2098.463330 recorded seconds; paired 19918/20977 within 25 ms.

Each camera start is independently normalized to zero. Side-by-side replay does not establish simultaneous physical recording.
A shorter recording ends explicitly; no frames or poses are fabricated to reach the Stage20 duration.
Every archive DB was read to EOF with ZIP CRC validation. All recorded CameraInfo/extrinsics messages were inspected.
Source header timestamps and bag timestamps are both preserved in the per-robot CSVs.
Calibration is unchanged; the established depth-to-color projection and 1000-feature baseline are retained.

## Complete timestamp ranges

| Robot | Stream | First bag ns | Last bag ns | Earliest header ns | Latest header ns | Overall Hz | Max bag gap ms |
|---|---|---:|---:|---:|---:|---:|---:|
| robot1 | RGB | 1722894200722237916 | 1722895200511398663 | 1722894200527811289 | 1722895200401253223 | 9.998108 | 249.858563 |
| robot1 | DEPTH | 1722894200716325970 | 1722895200509760995 | 1722894200527811289 | 1722895200401253223 | 8.765811 | 7258.844765 |
| robot3 | RGB | 1722894200732167328 | 1722896299195497751 | 1722894200605798006 | 1722896299062812090 | 9.995886 | 397.264952 |
| robot3 | DEPTH | 1722894200881230201 | 1722896299189132654 | 1722894200707788706 | 1722896299062812090 | 9.491934 | 4808.453327 |

## Actual topics and formats

- robot1 RGB: `robot1/camera/color/image_raw`, `sensor_msgs/msg/Image`, frame `robot1_camera_color_optical_frame`.
- robot1 DEPTH: `robot1/camera/depth/color/points`, `sensor_msgs/msg/PointCloud2`, frame `robot1_camera_depth_optical_frame`.
- robot3 RGB: `robot3/camera/color/image_raw`, `sensor_msgs/msg/Image`, frame `robot3_camera_color_optical_frame`.
- robot3 DEPTH: `robot3/camera/depth/color/points`, `sensor_msgs/msg/PointCloud2`, frame `robot3_camera_depth_optical_frame`.

Depth is an unorganized PointCloud2 camera cloud, not a native depth Image topic. Metric XYZ is projected through the recorded factory depth-to-color extrinsic and actual color K/D. Color is rgb8 at 1280×800; the established input scale is 640×400. No camera calibration is retuned.

All paired original header deltas are recorded in the decision JSON. Every recorded depth cloud has a unique RGB partner within the 25 ms bag-time gate; some RGB frames have no corresponding recorded depth. Those frames remain in the full tracking sequence with zero/unknown depth.

This is input qualification only. Full-duration tracking quality, reset fragmentation, map consistency and browser rendering require their own reports. No historical 120-second results are replaced.
