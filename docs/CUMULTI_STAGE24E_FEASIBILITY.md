# Stage24E mapping feasibility

| Route | Decision | Evidence |
|---|---|---|
| Pure RGB-D full-route continuity | **NO-GO for full-route continuity** | 1,301 sequential frame pairs tested, 14 passed single-pair geometry, zero new sequential bridges met the multi-frame gate. |
| LiDAR-assisted RGB-D projection | **NO-GO for projection with available calibration** | Original clock ranges overlap, but the static transform from the color optical frame to the LiDAR frame is unverified. |

Stage24D at `ca1af67` remains the camera-only baseline with 26 epochs, 22 connected components, three intra-robot bridges and one inter-robot bridge. Stage24E is an offline audit using saved poses and measured depth.

## A. Boundary audit

There are **24 within-recording transitions**, five for Robot1 and nineteen for Robot3. Twenty-five would require treating one robot's start as a continuation of the other recording. Both initializations are listed separately in `native_log_evidence.json`. `epoch_boundary_audit.csv` records the last and first valid frames, original RGB bag timestamps, endpoint and window depth availability, map IDs, initial KF IDs and invalid-frame intervals. The loss interval is measured from the first intervening invalid frame to the next valid frame. The last-valid to next-valid gap is reported separately.

Fifteen transitions changed map ID. Nine kept the ID and changed the initial KF ID. Of the latter, Robot1 4→5 and Robot3 8→9 and 18→19 have consecutive valid frames and nearby native merge messages. These are plausible map bookkeeping or coordinate-gauge changes associated with a merge, so an ID match alone cannot establish a common transform. The other six same-ID transitions have nearby native active-reset evidence, including three with only one intervening invalid frame. Native log lines are bracketed by progress markers. Where available, the reset context frame IDs are preserved separately, and the bracket does not establish an exact per-frame reset timestamp.

## B. Sequential measured RGB-D

Each side uses a window of up to 3 seconds, retaining the four closest valid frames before sampling additional frames at least 450 ms apart. All selected depth-paired endpoint combinations are tested without the historical 2-second exclusion. The existing mutual SIFT, PnP/RANSAC, reprojection and bidirectional depth-cloud gates apply unchanged. Epoch admission retains the Stage24D minimum of three distinct frames per side, 0.8 s span per side and 0.2 m / 3° agreement for intra-robot support.

| First limiting evidence | Boundaries |
|---|---:|
| Insufficient visual correspondence evidence | 5 |
| Too few measured-depth feature correspondences | 13 |
| PnP, spatial support or reprojection gate | 3 |
| Single-pair passes but insufficient independent support or SE(3) agreement | 3 |

Every boundary had selected depth-bearing endpoints and candidate coverage. Insufficient visual correspondence evidence does not establish physical non-overlap. The 14 single-pair passes occurred at Robot1 4→5 and Robot3 14→15 and 18→19. Per-boundary outcomes and per-pair stage diagnostics are saved.

As a separate diagnostic, depth submaps use alternating fit and validation frames where available, 5 cm voxel sampling and rigid point-to-point registration initialized by a fit-frame PnP hypothesis or identity in the two boundary-camera coordinates. The bidirectional validation gate remains 12 cm, at least 300 inliers and at least 35% support per direction. Thirteen boundary submaps met the depth agreement gate, ten with disjoint validation frames on both sides. The other three reused frames with offset pixel sampling because of short epochs. These are useful registration hypotheses, with a single aggregate transform per boundary and no replacement for the required independent multi-frame bridge evidence. In particular, a one-frame epoch cannot supply three distinct frames. No new edge was added.

## C. LiDAR-assisted projection

The main project at `ca1af67` and the separate held-out LiDAR/GICP checkout at `2ff22b8` were inspected. The canonical Stage12D global trajectory is `outputs/cumulti_v1/12d_clean_backend/clean_all75_gtsam_trajectory.csv`. Its poses are `T_world_from_os_sensor`. Every timestamp matched its corresponding original integer LiDAR timestamp in the 2 Hz keyframe index. The overlap audit separately evaluates original RGB/depth bag and header clocks and fits no offset.

| Robot | RGB bag overlap (s) | RGB frames with paired depth in range | Nearest trajectory timestamp median / p95 (ms) | Trajectory bracket p95 (ms) |
|---|---:|---:|---:|---:|
| Robot1 | 924.361 | 8,010 | 123.37 / 236.84 | 505.95 |
| Robot3 | 1,972.427 | 18,659 | 127.86 / 237.07 | 507.18 |

Nearest-pose residuals describe sampling against a 2 Hz trajectory. They do not measure hardware synchronization accuracy. A later trial should use timestamp-bracketed interpolation within the retained intervals and verify residual alignment independently.

The calibration archive's URDFs and Xacros provide the mounting chain to `camera_link`. Saved RealSense depth-to-color transforms pass the SO(3) checks. Neither source provides the measured last link from the color optical frame to `camera_link`. Robot3 LiDAR/RGB/depth bag metadata contain no static TF topic. Robot1 original ZIPs are currently absent from the inspected location, so its audit relies on retained timestamps and calibration records.

The required composition is `T_world_from_color = T_world_from_os_sensor @ T_os_sensor_from_color`, with `T_os_sensor_from_color = inverse(T_camera_link_from_os_sensor) @ T_camera_link_from_color`. The final factor is unavailable. The IMU–LiDAR transform belongs to the LiDAR pose derivation and cannot supply this factor. The 100–200-frame projection, alignment and route coverage evaluation remain gated on measured camera mounting calibration and verified synchronization.

## Reproduction

Run `bash scripts/run_stage24e_feasibility.sh` against the existing Stage24 inputs. Stage24E artifacts are under `outputs/cumulti_v1/24e_rgbd_feasibility/`.
