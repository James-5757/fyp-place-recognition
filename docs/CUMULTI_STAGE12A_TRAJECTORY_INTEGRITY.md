# CU-Multi Stage 12A: trajectory integrity and discontinuity audit

## Motivation and scope

The frozen final replay shows exceptionally long Robot1 (blue) and Robot3
(orange) trajectory lines. Stage 12A traces those lines through local odometry,
single-loop initialization, Rank1 GTSAM optimization, and the browser asset.
This is a GT-free, read-only audit. No pose, graph factor, retrieval result, or
demo rendering was repaired or changed.

## Sources and method

| Level | Exact source | Rows |
|---|---|---:|
| Local | `run_stage10_offline_map_merge.py:aligned(robot)`; nearest non-GT EKF IMU/GNSS odometry, transformed to the LiDAR graph frame | Robot1 2,000; Robot3 4,180 |
| Pre-PGO | `outputs/cumulti_v1/10_2_pgo_policy_selection/single_loop_trajectory.csv` | 6,180 |
| Post-PGO | `outputs/cumulti_v1/10_3_solver_audit/rank1_gtsam_trajectory.csv` | 6,180 |
| Demo | `demo/final_replay/data/replay.json`, three trajectory representations | 200 Robot1 and 418 Robot3 rows per representation |

The audit first verifies strictly increasing timestamps and keyframe IDs, no
duplicates, and exactly 1-keyframe full-resolution or 10-keyframe browser steps.
Every source passes. The browser JSON contains only XY positions; its timestamp,
Z, and orientation are explicitly matched by `(robot_id, keyframe_id)` to the
corresponding frozen full-resolution source. Every browser XY value agrees with
that source within 1e-6 m. Hence the asset generator did not invent a new
coordinate jump.

For each consecutive pose, the audit computes XY/XYZ translation, timestamp
delta, speed, and the geodesic SO(3) relative angle from unit quaternions. It
reports median, p90, p95, p99, p99.5, maximum, and MAD. Candidate flags use
`median + 10*MAD`, with p99.5 fallback if MAD is unsuitable, plus a separate
top-20 ranking. A flag is a request for investigation, not a declaration that
the robot moved incorrectly. A recording gap means an individual underlying
2-Hz interval exceeds twice its measured median; this is based on the expected
sample cadence and does not treat the browser's 10-frame stride as missing data.

## Results and lineage

| Source | Robot1 maximum adjacent XY step | Robot3 maximum adjacent XY step |
|---|---:|---:|
| Local EKF | 291.454 m | 333.575 m |
| Pre-PGO | 291.454 m | 333.826 m |
| Post-PGO | 287.045 m | 336.712 m |

The largest local events are Robot1 `57→58` (291.454 m XY in about 0.502 s)
and Robot3 `42→43` (333.575 m XY in about 0.500 s). The raw EKF endpoint
translation for those same intervals is 298.101 m and 344.384 m in 3-D,
respectively; it agrees with the aligned local 3-D displacement within a few
millimetres. The source archives are
`/home/cas/CU-Multi/raw/main_campus/robot{1,3}/robot{1,3}_main_campus_imu_gps.zip`,
topics `robot1/ekf/odometry_map` and `robot3/ekf/odometry_map`, with
`frame_id=robot1_map/robot3_map` and `child_frame_id=imu_link`. The individual
raw messages and nearest timestamps are recorded in
`upstream_jump_source_audit.csv`. This establishes that the large displacement
already exists in the non-GT EKF source; its physical cause requires a separate
sensor/estimator audit.

There are no missing-keyframe recording gaps at these events. The median local
intervals are 0.500031 s (Robot1) and 0.499931 s (Robot3), with maxima only
0.550519 s and 0.558694 s. No underlying interval exceeds twice its median.
The extreme implied speeds therefore cannot be explained by a long recording
pause. We do not infer a specific EKF failure mechanism from this audit alone.

The pre-PGO state introduces no new 3-D step: every local versus pre-PGO
adjacent XYZ distance agrees within `2.3e-13 m`. Robot3's maximum *XY*
projection changes from 333.575 to 333.826 m because its common-frame rigid
rotation mixes vertical and horizontal components. That is a coordinate
projection effect, not a second discontinuity.

GTSAM changes some steps, including 43 smaller newly flagged robust outliers,
but none of either robot's 20 largest post-PGO translations was a previously
unflagged pre-PGO transition. The maximum post-PGO events correspond to the
same pre-existing local anomalies. Thus the answer to “did PGO create a new
catastrophic long jump?” is **NO** under the declared top-20 plus robust
source-lineage test. The new smaller changes remain visible in
`pgo_step_change_audit.csv` and are not silently dismissed.

## Why the browser draws those lines

The demo draws one straight segment between every tenth keyframe. Its longest
post-PGO displayed segments are:

| Robot | Browser endpoints | Straight chord | Full 2-Hz path | Maximum deviation from chord | Diagnosis |
|---|---|---:|---:|---:|---|
| Robot1 | `50→60` | 414.346 m | 424.611 m | 7.466 m | upstream EKF jump, visible in demo |
| Robot1 | `60→70` | 413.038 m | 413.180 m | 4.253 m | upstream EKF jump, visible in demo |
| Robot3 | `40→50` | 566.742 m | 670.314 m | 96.852 m | upstream EKF jump plus conspicuous chord shortcut |
| Robot3 | `50→60` | 256.644 m | 343.617 m | 92.931 m | upstream EKF jump plus conspicuous chord shortcut |

These are genuine differences between stored trajectory endpoints, so browser
downsampling alone cannot explain their length. In Robot3, the intervening
full-resolution path also bends far away from the rendered straight chord.
The browser therefore makes the *shape* misleading even though its endpoints
are correct. Twelve displayed segments across the three representations exceed
their source/robot-specific p99.5 intermediate-chord-deviation diagnostic;
several are repeated views of the same physical interval. There are no
browser-only coordinate jumps.

`demo_polyline_downsampling_audit.csv` reports each chord length, actual path
length, chord/path ratio, maximum deviation, underlying maximum step, and
classification. `demo_downsampling_artifact.png` overlays an actual 2-Hz path
and its browser chord. `demo_longest_segments.csv` directly lists the ten
longest displayed lines for each robot at each replay level.

## Impact and required action

| Question | Assessment | Reason |
|---|---|---|
| Stage 8 retrieval invalidated? | **NO** | Ring Key / Scan Context rankings use frozen LiDAR descriptors, not EKF poses. |
| Stage 9 GICP invalidated? | **NO** | Registration uses LiDAR clouds and SC yaw initialization, not this local odometry; existing registration metrics are unchanged. |
| Stage 10.3 PGO invalidated? | **NEEDS_FOLLOWUP** | The native solver reproduced its frozen graph result, but the graph's odometry measurements contain upstream implausible motion. Physical trajectory/map quality needs a separate robustness audit. |
| Only visualization affected? | **NO** | The source trajectories contain the large steps; the browser also shortcuts their shape. |
| Demo affected? | **YES** | Long lines and map bounds can mislead viewers about physical motion. |

The diagnosis is **MULTIPLE_CAUSES** for the visible lines: the primary pose
discontinuity begins in **UPSTREAM_LOCAL_ODOMETRY**, and the browser's stride-10
polyline accentuates the appearance. The required next action is
**INVESTIGATE_LOCAL_ODOMETRY_UPSTREAM**. Do not delete or smooth poses in the
frozen artifacts. A later, separately reviewed demo change may mark or break
implausible displayed segments without representing that as a data repair.

## Reproduction and next stage

Run `bash scripts/run_stage12a_trajectory_integrity.sh` from the repository
root on the server. The script reads the frozen archives and artifacts and
writes only `outputs/cumulti_v1/12a_trajectory_integrity/`: 12 complete step
tables, ordering/statistics/candidate/lineage/PGO/demo/raw-source tables,
`summary.json`, `VALIDATION_REPORT.txt`, and six figures. GT is never read.

After review of this audit, the planned research direction remains a separate
sparse first-loop / incremental-loop study. This Stage 12A commit does not
start it or change the final demo.
