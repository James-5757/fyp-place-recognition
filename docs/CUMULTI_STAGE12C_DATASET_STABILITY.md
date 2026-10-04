# CU-Multi Stage 12C — dataset completeness and GT-free EKF stable-segment audit

## 1. Why check the incomplete-data hypothesis?

Stage 12B found 291–334 m adjacent XY jumps in the upstream Robot1/Robot3 EKF trajectory. It found no `/tf` inside the IMU/GPS ZIPs, but this alone did not show a missing download. Stage 12C checks the official layout against the actual local files and then uses a frozen, GT-free rule to test whether later odometry is fit for a *future* backend study. It does not delete poses, repair odometry, run GTSAM, or change the demo.

## 2. Official CU-Multi structure

The [official CU-Multi repository](https://github.com/arpg/CU-Multi) describes calibration/URDF and, per environment and robot, UTM/reference poses, LiDAR, IMU/GPS, RGB, and depth. Its topic table places `/tf` in the **separate ground-truth/pose bag**, publishing world→LiDAR `base_link`, not in the IMU/GPS bag. The latter records GNSS, IMU, and EKF-related sensor messages. `official_expected_dataset_structure.json` captures the source and expected components. The official conceptual folder layout is distributed here as ZIPs containing ROS2 SQLite bags; the naming is not assumed identical to the documentation's examples.

## 3. Local dataset inventory

The audit inspected `/home/cas/CU-Multi/raw/` and `/home/cas/CU-Multi/processed_v1/`, writing `local_dataset_inventory.csv`. For the current Robot1↔Robot3 pipeline, both robots have LiDAR, IMU/GPS, RGB, UTM GT CSV, and separate relative-pose bag. Calibration/URDF is present in `raw/calib/robot_description.zip`. Robot1 and Robot2 also have depth and LiDAR-label archives; Robot3 has neither depth nor label ZIP locally. Robot4 has only its UTM GT CSV locally. Thus the **entire official four-robot collection is not locally complete**, but the frozen Robot1↔Robot3 LiDAR/RGB/IMU plus offline-GT study has its required components. Robot3 depth/labels and Robot4 sensors are outside this study; no download was started.

`dataset_completeness_report.json` uses the required classification `COMPLETE_FOR_CURRENT_PIPELINE` and separately records the missing optional/full-four-robot components. This is not a claim that all official modalities for all robots are present.

## 4. Archive and stream integrity

`archive_integrity_report.csv` covers every ZIP consumed by the current R1↔R3 sensor/GT/calibration pipeline: each robot's LiDAR, IMU/GPS, RGB, relative-pose bag, and the shared calibration ZIP. Every member was read to EOF, which validates ZIP CRC without permanently extracting it. Each temporary ROS2 `.db3` was opened read-only for `PRAGMA integrity_check`, and actual message count was compared with `metadata.yaml`; YAML was parsed. Temporary database copies were removed. The large-file tests are expensive but do not alter raw inputs.

`stream_duration_audit.csv` records per-topic metadata counts and the **bag-level recording span** (explicitly labeled; not a per-topic first/last timestamp), plus exact UTM CSV timestamp span. For Robot1, LiDAR/relative-pose/RGB spans are ~1000 s and IMU/GPS ~1003 s; for Robot3 they are ~2090 s and ~2102 s. These modest start/end differences are not evidence of truncation. ZIP/SQLite integrity and message-count checks are the stronger completeness tests. The audit does not require identical sensor start/stop timestamps.

## 5. Why `/tf` was absent from `imu_gps`

This absence is **expected** according to the official topic table. `ground_truth_source_inventory.json` confirms that local `robot1_main_campus_gt_rel_poses.zip` contains `/tf` (19,998 messages) and `robot3_main_campus_gt_rel_poses.zip` contains `/tf` (41,793 messages). They also contain LIO-SAM mapping path topics; their mapping-odometry topics have zero recorded messages. The UTM CSVs are present separately. `/tf_static` was not found in those pose-bag topic inventories. These GT/TF sources are used only to establish dataset completeness and optional offline diagnosis, never as an EKF-stability feature or graph input.

## 6. Full-run GT-free EKF analysis

The reader starts from the frozen 2-Hz Stage 12A local step tables, then samples nearest raw `ekf/odometry_map`, two raw antenna `NavSatFix` streams, raw IMU magnitude, covariance, twist, header/frame IDs, and timestamp order across the **entire** Robot1 (2,000) and Robot3 (4,180) runs. Maximum header-time match errors are 0.05 s for EKF, 0.35 s for GNSS, and 0.02 s for the high-rate IMU. IMU acceleration (including gravity) and angular-rate norms are retained as sanity context, not integrated or used as a stable-start gate. A repeated GNSS message at adjacent keyframes is not falsely counted as zero motion; GNSS/EKF step-magnitude disagreement is evaluated only when a new raw sample supports it. `robot1_full_ekf_stability.csv` and `robot3_full_ekf_stability.csv` retain every keyframe, raw-match error and the frozen-rule flags. The early 0–120 s and full-run figures show translation, implied/reported speed, covariance, and raw GNSS step.

## 7. Frozen stable-start policy

`stable_start_policy.json` was written and fsynced **before** selecting a start. It fixes a 10-s / 20-keyframe look-back, then requires 30 s / 60 consecutive passing look-back windows. For the final 50% of each run (a GT-free reference), XY step, relative rotation, implied speed, and EKF-vs-GNSS step-magnitude disagreement must remain below `median + 10 × 1.4826 × MAD`; position covariance trace must remain at or below `5 × final-half median`. Both raw GNSS antennas must have finite valid fixes, frames and timestamps must stay ordered, and the EKF covariance must be finite. The first keyframe **after** the sustained 30-s certification is the declared start; the rule does not backdate it. The resulting numerical thresholds are saved in `stable_segment_selection.json`. They were not tuned to GT or to a desired start timestamp.

Late-run recurrence is separately tested against each frozen limit. Ordinary exceedances are counted, while `>5×` the corresponding limit is the predeclared major-recurrence signal. This conservative compound rule can fail on covariance even when physical translation is continuous; such a failure must be reported, not retuned away after inspection.

## 8. Robot1 start

The frozen rule declares **KF150**, 75.034 s after the first LiDAR keyframe. It excludes 150 initial keyframes and retains 1,850/2,000 (92.5%) and ~924.50 s. Its final-half XY-step reference median is 0.206 m with a 1.223 m threshold; the covariance median is 0.0639 m² with a 0.3196 m² threshold. `robot1_ekf_stability_timeline.png` shows the earlier 300-m excursion and the later candidate start without hiding the full-run context.

## 9. Robot3 start

The independent rule declares **KF234**, 117.003 s after its first LiDAR keyframe. It excludes 234 initial keyframes and retains 3,946/4,180 (94.4%) and ~1972.51 s. Its final-half XY-step median is 0.394 m with a 3.456 m threshold; covariance median is 0.0846 m² with a 0.4232 m² threshold. This is a conservative certification time, not a hand-selected point at the end of the visible early spike.

## 10. Late-run recurrence

`post_stable_start_recurrence.csv` finds **no major later XY translation, rotation, implied-speed, or EKF-vs-GNSS step-disagreement event** after either declared start. Robot1's largest later XY step is 1.128 m; Robot3's is 1.470 m. However the predeclared `>5×` covariance criterion catches **one** Robot1 keyframe (~771.5 s, trace 1.690 m²) and **three** adjacent Robot3 keyframes (~1357.5–1358.5 s, trace up to 3.039 m²). Their simultaneous XY steps are small. These are *covariance-only health excursions*, not a recurrence of the 300-m pose failure, but they fail the declared all-signal no-major-recurrence condition. There are additional ordinary rotation/covariance threshold exceedances; none reaches the major rotation threshold. No threshold was relaxed after seeing this.

## 11. Frozen frontend coverage after the cut

`stable_segment_frontend_coverage.json` filters existing Stage 9 Rank-1 GICP records and frozen Stage 10.2 Rank-1 sanitized loops by **both** endpoint IDs; it does not rerun Scan Context or GICP. Remaining are 1,850 Robot1 query keyframes, 3,946 Robot3 database keyframes, **1,101 frozen Rank-1 GICP accepted pairs**, and **75 sanitized loops**. Thus K=1,2,5,10,20 is feasible **by loop count only**; geometric distribution and graph observability have not been tested. The current graph was not rebuilt.

## 12. Offline GT diagnostic — never used for stable-start selection

`offline_gt_jump_diagnostic.csv` was generated only after `stable_segment_selection.json` existed on disk. Nearest UTM reference poses move ~0.0035 m XY during Robot1 KF57→58 and ~0.0055 m during Robot3 KF42→43, with <0.009 s timestamp offsets. This independently confirms the platform did not travel hundreds of metres. The CSV is labeled `OFFLINE DIAGNOSTIC ONLY; NOT USED FOR STABLE-START SELECTION`. It cannot change the frozen start rule or any graph factor.

## 13. Dataset download decision

`dataset_download_action.json` says `NO_DOWNLOAD_NEEDED` **for the current Robot1↔Robot3 study**. The full official four-robot/modalities collection would require additional optional Robot3 depth/labels and Robot4 sensor archives, listed with exact missing paths in `local_dataset_inventory.csv`; no such transfer was started. Absence of TF inside the IMU/GPS ZIPs is not a missing component.

## 14. Is delayed-start backend reconstruction defensible?

Under the **predeclared compound health rule**, the decision is `DELAYED_START_INSUFFICIENT`: stable starts exist, archive/coverage checks pass, and no large pose-step recurrence occurs, but later covariance-only excursions exceed the frozen major limit. This is not evidence that the whole run is unusable; it is a reason to investigate those isolated health events and formulate a separately frozen policy before a clean-graph rebuild. The sparse-loop experiment on current odometry remains **NO**; readiness after a future independently validated clean graph is **CONDITIONAL**, not yet a pass. This stage makes no odometry replacement, map, or PGO output.

## 15. Next stage and reproducibility

Resolve the late covariance alerts and inspect recording-time filter status/TF if a future repair decision requires them. Any new gating or graph-construction policy must be fixed before backend and GT evaluation. The Stage 12C script supports `inventory`, `integrity`, `stability`, and `finalize` modes, writing only `outputs/cumulti_v1/12c_dataset_stability/`. See `VALIDATION_REPORT.txt` for exact checks. Do not treat this audit as authorization to run sparse-loop/iSAM2 or a demo.
