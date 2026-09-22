# CU-Multi Stage 9 Frame-Convention Audit

## Status: UNRESOLVED — Stage 9 stopped at the mandatory gate

Stage 9 does not generate GICP constraints, PGO-ready loop edges, or run the
registration experiment until this status becomes `RESOLVED`.

## Evidence available locally

- The frozen Stage-2 and Stage-4 PointCloud2 header audits identify the cached
  Robot1 and Robot3 cloud frames as `robot1_os_sensor` and `robot3_os_sensor`.
  Cached XYZI is copied directly from PointCloud2 and therefore remains in those
  sensor frames.
- `calib/robot_description.zip` contains `robot1.urdf` and `robot3.urdf`. Their
  respective fixed chains `os_sensor -> ouster_riser_plate -> mounting_plate ->
  chassis -> base_link` have zero net yaw. Thus no 180-degree yaw is introduced
  by the provided sensor-to-body extrinsics.
- The official CU-Multi README documents `/tf` from `world` to the robot LiDAR
  frame at LiDAR timestamps. The locally available Robot1 relative-pose bag
  directly records `world -> robot1_os_sensor`.
- For deterministic samples from Robot1, the UTM CSV quaternion yaw differs from
  the recorded `/tf` sensor yaw by exactly 180 degrees (up to numerical
  precision). This explains the prior 180-degree diagnostic discrepancy as a
  UTM-pose quaternion convention, not an SC shift correction, candidate-order
  inversion, or URDF sensor mounting rotation.

## Missing blocking evidence

`/home/cas/CU-Multi/raw/main_campus/robot3/robot3_main_campus_gt_rel_poses.zip`
is not present. It is the required Robot3 recording containing `/tf`, so the
same direct Robot3 `world -> robot3_os_sensor` convention check cannot be made.
Robot3's PointCloud2 frame and its URDF alone are not enough to assert a
cross-robot metric transform convention for PGO.

## Frozen definitions prepared but not executed

If the audit is resolved, the interface will use:

`T_query_from_candidate`, defined by
`p_query = T_query_from_candidate * p_candidate`.

Open3D GICP will receive candidate as `source` and query as `target`, so its
returned transformation has exactly that direction. The existing SC matcher
computes `score(s) = sum_k query[k] dot candidate[k-s]`; with 60 sectors, the
non-GT-derived initialization would be:

`signed_shift = best_shift if best_shift <= 30 else best_shift - 60`

`yaw_SC = wrap(+ signed_shift * 6 degrees)`.

This follows directly from the candidate sector moving into the query sector;
the fixed frame offset is zero because the two cached cloud frames use the same
Ouster `os_sensor` convention. It is documented here but is not used to run
GICP while the Robot3 TF confirmation is missing.

## Required next input

Transfer only the missing small relative-pose archive from the CU-Multi source:

`/main_campus/robot3/robot3_main_campus_gt_rel_poses.zip`

to:

`/home/cas/CU-Multi/raw/main_campus/robot3/robot3_main_campus_gt_rel_poses.zip`

No Robot3 LiDAR, RGB, depth, labels, or other archive is required for this gate.
