# CU-Multi Stage 9 Frame-Convention Audit

## Status: RESOLVED

The mandatory frame gate is resolved. Stage 9 registration used this documented
interface; it did not run PGO or map merging.

## Evidence and resolved convention

- Frozen Stage-2/Stage-4 PointCloud2 header audits identify the cached cloud
  frames as `robot1_os_sensor` and `robot3_os_sensor`. Cached float32 XYZI is
  copied from those PointCloud2 messages and remains in its LiDAR sensor frame.
- `calib/robot_description.zip` provides both URDFs. For Robot1 and Robot3 the
  fixed chain `os_sensor -> ouster_riser_plate -> mounting_plate -> chassis ->
  base_link` has zero net yaw; the URDF does not create a 180-degree correction.
- The official CU-Multi documentation and both locally inspected relative-pose
  ROS bags provide `/tf` transforms `world -> robot*_os_sensor` at LiDAR time.
- The deterministic UTM-versus-`/tf` comparison passes for both robots after
  the documented UTM quaternion convention adjustment: Robot1 maximum/mean
  residual is `5.684341886080802e-14` / `1.5631940186722205e-14`, and Robot3
  maximum/mean residual is `5.684341886080802e-14` /
  `2.4158453015843406e-14`. The apparent 180-degree discrepancy is therefore
  the UTM CSV quaternion-yaw convention, not a Scan Context shift, an ordering
  inversion, or a sensor-mounting rotation.

The audit program and machine-readable evidence are respectively
`src/cumulti/run_stage9_frame_audit.py` and
`outputs/cumulti_v1/09_sc_gicp_integration/frame_convention_audit.json`.

## Registration interface

Stage 9 uses `T_query_from_candidate`, with

`p_query = T_query_from_candidate * p_candidate`.

GICP receives the database candidate as source and the query cloud as target,
so the returned Open3D transform has that exact direction. The frozen Scan
Context matcher is `score(s) = sum_k query[k] dot candidate[k-s]`. With 60
sectors:

`signed_shift = best_shift if best_shift <= 30 else best_shift - 60`

`yaw_SC = wrap(+ signed_shift * 6 degrees)`.

This is derived from the descriptor sector convention and the common Ouster
sensor frame, not fitted with ground truth. Ground truth is only used after
registration for offline error/overlap analysis.

## Scope

This audit establishes a metric, cross-robot LiDAR-frame transform convention
for Stage 9 constraints. It does not validate a PGO solution; PGO remains out
of scope for this stage.
