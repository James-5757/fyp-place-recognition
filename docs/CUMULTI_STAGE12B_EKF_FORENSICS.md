# CU-Multi Stage 12B — upstream EKF anomaly forensic audit

## 1. Why this audit was needed

Stage 12A found physically implausible local-odometry transitions that were already present before graph optimization and were accentuated, not created, by the replay's stride-10 drawing. This audit reads the original Robot1/Robot3 IMU–GNSS ROS2 archives without changing any pose, edge, graph, or demo. Ground truth is never loaded. The machine-readable evidence and six figures are in `outputs/cumulti_v1/12b_ekf_forensics/`; the reproducible reader is `src/cumulti/run_stage12b_ekf_forensics.py`.

## 2. Stage 12A findings and event selection

The frozen local 2-Hz steps have maximum adjacent XY displacement 291.454 m at Robot1 KF57→58 (~0.502 s) and 333.575 m at Robot3 KF42→43 (~0.500 s). We reused `upstream_jump_source_audit.csv`, `trajectory_jump_lineage.csv`, `summary.json`, and the full-resolution local step tables. The exact top ten local XY steps for each robot, not hand-picked timestamps, are listed in `audited_event_manifest.csv`. Each is corroborated by its Stage 12A source and lineage row.

## 3. Available raw topics and format

Both `robot{1,3}_main_campus_imu_gps.zip` archives contain a ROS2 rosbag2 SQLite `.db3` and `metadata.yaml`, with CDR-serialized messages. `raw_topic_inventory.json` records every actual topic, type, count, and first/last bag timestamp. Relevant recorded topics include `robot#/ekf/odometry_map`, `robot#/ekf/odometry_earth`, `robot#/ekf/llh_position`, `robot#/ekf/velocity`, `robot#/ekf/imu/data`, `robot#/ekf/status`, `robot#/mip/ekf/status`, both `robot#/gnss_{1,2}/llh_position` and `/odometry_earth`, and `robot#/imu/data`. The pose stream is approximately 100 Hz, the two raw GNSS positions approximately 2 Hz, and raw IMU approximately 500 Hz. The inventory, not these illustrative names, is authoritative.

Neither specified archive contains `/tf` or `/tf_static`: **NOT AVAILABLE IN ARCHIVE**. A map→odom or odom→base transform sequence cannot be reconstructed from these ZIPs. The EKF odometry header frame is `robot1_map` or `robot3_map`, with child `imu_link`. We do not infer a globally metric map origin or transform merely from the topic/frame name.

## 4. Event windows and synchronization

For each event, the reader queries all available EKF/GNSS/IMU/localization-related topic messages from KF-i timestamp −5 s through KF-j timestamp +5 s. It records bag timestamp, decoded header timestamp when available, topic, message type, archive message ID as sequence index, and time relative to the largest raw EKF incremental step. The detailed EKF table contains ±1 s around that step. The fixed nearest-time limits, set from topic rates, are 0.05 s for EKF and 0.35 s for GNSS. A same raw GNSS sample at both keyframe endpoints is not counted as independent displacement. Matching errors are in `source_alignment_at_jump.csv` and policy in `source_matching_policy.json`.

## 5. Robot1 primary jump

At KF57→58, matched raw `ekf/odometry_map` positions differ by 298.101 m XYZ / 291.258 m XY across ~0.502 s. The slight difference from Stage 12A's 291.454 m is nearest-message versus its frozen keyframe pose sampling. The largest *single* adjacent raw EKF position step in that interval is 7.068 m; approximately 60 high-rate messages contribute. Hence “instantaneous at 2 Hz” is not a one-message teleportation at 100 Hz. Pose-implied mean speed is roughly 594 m/s, while the reported endpoint twist magnitudes are ~0.066 and ~0.178 m/s. Relative EKF orientation changes 13.849°. `ekf/odometry_earth` moves 298.101 m over the same timestamps.

Both independent raw antenna LLH samples move 0.000 m at their matched endpoints (all endpoint matching errors ≤0.069 s). The EKF position-covariance trace is already very large, ~324,575 m² before and ~279,659 m² after; it does not newly blow up from near zero at this boundary. Both frame IDs remain constant, with no header-time regression in the 10-s window. Within the following five seconds the pose moves back near its pre-event position: 39.8 m from baseline after a 366.3 m peak, but the excursion stays above half peak for ~3.84 s. The conservative shape label is `OTHER`: a multi-second excursion/recovery, **not** an isolated spike or proven discrete origin switch. TF is **NOT AVAILABLE IN ARCHIVE**.

## 6. Robot3 primary jump

At KF42→43, matched raw EKF map positions differ by 344.384 m XYZ / 333.560 m XY in ~0.500 s, versus Stage 12A's frozen 333.575 m XY. The maximum adjacent high-rate raw EKF increment is 7.636 m across approximately 60 messages; implied mean speed is ~689 m/s, versus reported endpoint twist ~0.059 and ~0.083 m/s. Relative rotation is only 0.115°. The `ekf/odometry_earth` displacement matches at 344.384 m.

Raw GNSS1/2 LLH matched endpoint motion is 0.000/0.011 m (maximum matching error ≤0.168 s). The position-covariance trace is ~374,793 m² before and ~308,232 m² after. The frame remains `robot3_map`→`imu_link`, with no header-time regression. The 5-s-later offset is 164.4 m versus a 571.2 m peak; the excursion remains above half peak for ~4.50 s. It is also `OTHER`, a multi-second excursion/recovery. TF is **NOT AVAILABLE IN ARCHIVE**.

## 7. GNSS evidence

`gnss_event_audit.csv` records raw dual-antenna NavSatFix LLH, fix status, covariance, and consecutive horizontal displacement using a short-range local ENU calculation (WGS84-scale Earth-radius approximation), rather than comparing degrees. The two independent raw antenna streams have <0.06 m matched displacement in each of the 20 selected event intervals. Their Earth-odometry streams are also near-stationary. By contrast, the fused `ekf/llh_position` changes by ~3.446 m and ~16.594 m at the primary Robot1/3 events; it is **not** treated as independent raw GNSS. These data rule against a hundreds-of-metres jump in the measured raw antenna positions during the same keyframe intervals. They do not prove the internals of the vendor filter.

## 8. TF, frame, and status evidence

The `tf_event_audit.csv` and `tf_frame_graph.json` explicitly report **NOT AVAILABLE IN ARCHIVE** for `/tf` and `/tf_static`. We therefore cannot affirm or exclude a dynamic parent-frame reset directly. No EKF `frame_id`/`child_frame_id` switch or header-time regression occurs across the audited windows. ROS2 `Header` has no sequence counter, and node-session restart metadata is unavailable. Both `odometry_map` and `odometry_earth` carry a same-size position excursion, which argues against a *map-output-only* reset but is not a TF measurement.

`ekf/status` and `mip/ekf/status` topics are present and their timestamps are indexed. The recorded vendor payload could not be decoded safely: registering the field layout from the official `microstrain_inertial_msgs_common` revision `ed9cb178b1be0ff110e41548ca2165b593976146` against the bag's `HumanReadableStatus` type reaches a CDR end-of-buffer (first inspected 177-byte sample requires at least 184 bytes). This is a recorded-versus-available schema mismatch; we do **not** fabricate filter-state or GNSS-quality values. `ekf_status_audit.csv` and `decode_limitations.json` retain that limitation. Resolving the *recording-time* message definitions and TF source is the key next forensic step.

## 9. Covariance, orientation, IMU, and other top events

`ekf_covariance_audit.csv` contains before/at-peak/after position trace, orientation proxy, maximum diagonal, and finite checks. The high primary-event position traces are evidence of poor estimated position certainty, not alone proof of reset. `imu_sanity_at_jump.csv` shows raw acceleration magnitudes around gravity (~10 m/s²) and small angular rates over the half-second intervals; this is qualitative support only, not an inertial integration or formal motion bound.

All 20 top transitions exhibit the same high-level pattern: large EKF map and earth displacement, raw dual-GNSS displacement below 1 m, stable pose frame IDs, and elevated EKF covariance. The selected windows cluster at the start of the recordings; many rank positions are adjacent steps along the **same** multi-second excursion, not 20 independent resets. `post_jump_behavior.csv` uses conservative finite-window shape labels: `PERSISTENT_OFFSET` means the position at window end remains ≥75% of the window peak relative to its local baseline; `ONE_OR_FEW_SAMPLE_SPIKE` requires ≤20% and ≤0.1 s above half peak. A `PERSISTENT_OFFSET` label on an internal step does not establish a permanent run-wide offset. Per-event classification and supporting evidence are in `event_pattern_summary.csv`.

## 10. Root-cause classification

Both primary events and the top-ten clusters are classified `EKF_STATE_RESET_OR_REINITIALIZATION`, **MEDIUM** confidence. Specifically, the most likely *family* is unstable early EKF position initialization/convergence, visible in raw high-rate odometry during the first ~30–40 s, while raw GNSS remains stable. This label does **not** assert a discrete estimator reset. Independent GNSS, EKF map/earth, covariance, and IMU make an estimator-output anomaly credible; missing TF, recording-time vendor status schema, and internal filter logs prevent a HIGH-confidence trigger diagnosis. `root_cause.json` preserves the distinction. An exact map-origin, odom-frame, vendor filter-state, or timestamp-causality claim is unsupported.

## 11. Impact on earlier stages

The Stage 8 Scan Context frontend and Stage 9 frozen LiDAR registration are not invalidated by this upstream EKF odometry defect. The Stage 10.3 GTSAM solve remains numerically valid for its supplied factors; GTSAM did **not** create the original jump. Physical interpretation of the merged trajectory/map is **COMPROMISED** because its odometry input is implausible. Existing results remain frozen and are not recalculated here.

## 12. Recommended future repair and sparse-loop readiness

Recommendation: `MORE_SOURCE_INVESTIGATION_REQUIRED`; **no repair is applied**. First recover the recording-time vendor message definitions and any external TF, then predeclare and separately validate an odometry-repair or independent replacement policy. The observed anomaly lasts several seconds, so dropping one or two points is not justified; actual segment/reset boundaries are unconfirmed, so breaking into piecewise frames is premature. A LiDAR-based replacement may eventually be examined in a separate stage, but is not run here. `sparse_loop_readiness.json` marks current-odometry sparse-loop study **NO**: reducing loop count cannot make 300-m half-second odometry factors physically valid.

## 13. Limitations and next stage

Only the two specified IMU/GNSS ZIPs were read; no raw archive was modified. No GT, new GICP, PGO, demo rendering, or sparse-loop experiment was used. `/tf` and `/tf_static` are **NOT AVAILABLE IN ARCHIVE**. Vendor filter status payload is not reliably decoded. The finite ±5-s windows characterize excursions but do not certify long-term stability or the precise vendor estimator implementation. A future, separately authorized source-schema/TF investigation should resolve those points before any frozen repair design.

Reproduce on the CU-Multi server in its existing `fyp_slam` Python environment with `python src/cumulti/run_stage12b_ekf_forensics.py`. The script reads frozen Stage 12A tables and raw archives and writes only the Stage 12B output directory. `VALIDATION_REPORT.txt` records the final checks. No Stage 12A or earlier output is overwritten.
