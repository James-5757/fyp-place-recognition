# Stage26 Phase 1 data recovery

**Final recovery gate: PASS.** All 748 requested files were recovered from the original ROS2 archives and verified. Robot1 provided 106 RGB and 106 depth files. Robot3 provided 268 RGB and 268 depth files. There were no missing assets or mismatches.

All 374 pairs have identical RGB and depth header timestamps. Maximum absolute bag timestamp differences were 16.489495 ms for Robot1 and 14.712457 ms for Robot3, within the frozen 25 ms association gate. The verifier checked source message IDs, both timestamps, topics, archive identities, optical frames at export, decoded 640 × 400 dimensions, JPEG/PNG signatures, pixel types and SHA256. Depth is the original PointCloud2 data projected using the frozen Stage24 depth-to-color procedure. Per-frame nonzero-depth fractions match the frozen Stage24 records.

See [the recovery gate](stage26_gate1.json), the four per-file receipts and [the registration report](stage26_registration_report.md). Raw archives, SQLite work copies and decoded images remain on the server.

## Historical inventory before recovery

Baseline: `ff03354801ed43a589e9125d2703e98ca62e2379`. Status: **Gate 1 blocked by missing original frame assets**.

The manifest uses the Stage24E fit/holdout receipts and last/first-valid boundary anchors. This covers all 13 submap depth-agreement cases and all 11 rejected controls. It joins each frame to the frozen per-robot pose and RGB/depth synchronization indices, checks epoch membership, compares original RGB bag/header and relative timestamps exactly, and verifies every required depth association against the original 25 ms bag-time gate. Pose-only anchors and reused fit/holdout frames are explicitly marked. Preserved table timestamps have been verified. Recovered ROS-message timestamps and image contents have yet to be verified.

| Robot | Required RGB-D frame indices for the 13 cases | Additional failure-control memberships | Unique required RGB-D frame indices | Missing RGB / depth assets |
|---|---:|---:|---:|---:|
| Robot1 | 29 | 54 | 83 | 83 / 83 |
| Robot3 | 128 | 125 | 244 | 244 / 244 |

The manifest has 336 boundary memberships including pose-only anchors. The missing-frame CSV lists 654 unique expected JPEG/PNG paths with original ROS message IDs and both timestamps. These figures describe the bounded boundary experiment. Non-adjacent candidates may require additional frames.

## Sources checked

- Windows project outputs and the baseline Git checkout contain the frozen pose and timestamp tables. They contain no Stage24 tracking-resolution RGB/depth cache for the required frames.
- Server search covered the CU-Multi dataset tree, the accessible home workspace, temporary archives, and standard backup/mount paths. The located Stage24 tar files contain source code, reports, or browser presentation assets. No original tracking RGB/depth cache or extracted camera SQLite work copy was found.
- Robot3 has both original camera ZIPs under `/home/cas/CU-Multi/raw/main_campus/robot3/`. The RGB SQLite member lists 64,568,975,360 bytes and CRC32 `f3f3c891`. The depth member lists 140,031,242,240 bytes and CRC32 `047c2655`. Both match the Stage24 expansion receipts. This check uses the ZIP directory metadata. It does not revalidate payload CRC or prove message-level recovery.
- Robot1's raw directory currently contains `robot1_main_campus_imu_gps.zip`. Missing originals are `robot1_main_campus_camera_rgb.zip` and `robot1_main_campus_camera_depth.zip`. The official Globus collection is the original acquisition route. Current access to the two individual entries was not verified in this audit. A verified Stage24 cache from an earlier backup is the other recovery route.

## Resume criteria

For Robot3, perform a selective export in separate scratch space. The two SQLite members need roughly 191 GiB if expanded together. The server had approximately 550 GiB free at audit time. The DBs can be expanded one at a time, validated to ZIP EOF, and queried read-only for the **exact** message IDs in the manifest. Check topic, bag timestamp and message header timestamp against the frozen indices before decoding. Use the frozen Stage24 RGB export and depth-to-color projection and record asset SHA256s and original message provenance. The same workflow applies to Robot1 once its archives are restored. Any backup cache requires provenance connecting the files to the frozen message IDs and timestamps. Presentation JPEGs and pilot frames are excluded.

Complete the verified source recovery for the bounded cohort before fitting or comparing registrations. No new SE(3) estimate, pose graph, or map was produced. Stage24D remains the comparison baseline with 22 components and four verified bridges. Readiness for full global reconstruction is **NO-GO**.

Generate the manifest from repository root with `python src/cumulti/rgbd_full/stage26_manifest.py`. An optional `--cache-root` accepts a separate root containing the original repository-relative cache paths. File presence alone leaves the asset unverified.
