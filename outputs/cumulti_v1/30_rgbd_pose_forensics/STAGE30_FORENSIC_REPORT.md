# Stage30 Robot3 RGB-D pose forensics

Baseline: Stage29 `9b527682bb1616ec2886a2327872c9b4f350cf0a`. The investigation reads original Robot3 RGB, depth and IMU ROS2 messages and the frozen pose and placement tables. No pose, graph constraint, point-cloud chunk or combined map was changed.

## Confirmed observations

The audit examined consecutive valid-pose endpoints **within the same native Epoch**, including endpoints separated by invalid frames. The prespecified diagnostic trigger was a translation increment ≥1 m or rotation increment ≥35°. Seven intervals triggered it. The CSV contains exact original RGB bag/header times, apparent linear/angular rates, state and fingerprint context, sensor statistics and directional geometry. Eighty-five original RGB frames were retrieved. Associated depth and nearest-but-unsynchronized depth were decoded separately and their hashes, IDs and time errors recorded. The latter is excluded from geometry, even when packets exist.

| Epoch | Original frames | Gap (s) | Apparent translation / rotation | Apparent speed (m/s) | Invalid frames | Gyro norm integral |
|---|---|---:|---:|---:|---:|---:|
| 6 | 5135→5169 | 3.058 | 5.023 m / 72.23° | 1.64 | 33 | 59.27° |
| 6 | 5172→5178 | 0.765 | 1.322 m / 0.77° | 1.73 | 5 | 3.46° |
| 7 | 6906→6907 | 0.115 | 1.083 m / 1.79° | 9.41 | 0 | 0.62° |
| 9 | 9935→9936 | 0.100 | 2.056 m / 2.67° | 20.52 | 0 | 1.36° |
| 9 | 10070→10071 | 0.100 | 1.645 m / 1.76° | 16.37 | 0 | 0.74° |
| 9 | 12056→12057 | 0.093 | 1.235 m / 0.52° | 13.30 | 0 | 0.49° |
| 12 | 14081→14082 | 0.091 | 1.080 m / 0.97° | 11.83 | 0 | 0.31° |

The raw IMU comparison uses angular-velocity and acceleration norms on the original bag clock, so no IMU-to-optical extrinsic is assumed. The integral estimates angular path length and is not a signed camera rotation or an exact bound from a calibrated camera model. Nearest GNSS Doppler speed samples, where present within the event plus a ±150 ms margin, are separate supporting observations. No GNSS sample in that window was available for frames 9935→9936 or 10070→10071. IMU and GNSS stay outside the camera fitting and placement.

## Supported explanations

**Epoch 6 spans tracking loss during genuine motion.** Both endpoints of 5135→5169 retain Map ID 2 and initial KF 403, while the 33 intervening frames have no valid pose. The RGB view turns substantially. The IMU records 59.27° angular path across 1,529 samples and nearby GNSS speed reaches 2.19 m/s. The apparent 1.64 m/s is compatible with real travel over the gap. Neither endpoint has synchronized depth. Only 1/35 frames in the inclusive interval passes the frozen RGB-depth pairing rule, and endpoint bag deltas are 1,784 and 704 ms. The 5172→5178 interval has the same fingerprint, five invalid frames, unavailable synchronized endpoint depth and nearby GNSS speed up to 1.78 m/s. It is unsafe to infer a 5 m pose error from endpoint separation alone. The 72.23° camera estimate is larger than the measured gyro path estimate, so the endpoint rotation is also unverified. Recovery into the same map is observed. An internal relocalization or map update cannot be attributed to a particular frame from the retained logs.

**Epoch 9 contains native pose increments inconsistent with the sensor observations.** At 9935→9936 both frames and their neighbors are valid, tracking state 2, Map ID 4 and initial KF 403. Their depth header times match RGB, and bag-time offsets are 0.78 and 1.94 ms. The RGB views are visually similar and produce 790 mutual ORB matches, but only seven 3D matches support the local RANSAC hypothesis, below the fit threshold. At 12 cm, untransformed depth has 72.2% / 74.2% bidirectional overlap and the reported pose gives 1.22% / 0.77%. The two neighboring frozen-pose steps are 0.159 and 0.116 m with 76–82% overlap. The other two Epoch 9 jumps also have much higher identity overlap than frozen-pose overlap. For 10070→10071, a local visual fit estimates 0.163 m rather than the reported 1.645 m, while identity performs better than that fit on sampled depth. These observations support native tracking/pose inconsistency despite the valid state. They supply no independently validated replacement pose.

The consecutive-frame Epoch 7 and 12 anomalies show the same pattern. Their estimated RGB-D motions are 0.068 and 0.127 m and nearby GNSS speeds are approximately 1.61 and 1.22 m/s. The much larger recorded increments are poorly supported. The event fingerprints are unchanged. There is no evidence of a reset or a timestamp mismatch causing the Epoch 9, 7 or 12 cases. The tracker emits `Twc = inverse(Tcw)` and the direction used here restores good geometry on neighboring control intervals. A systematic conversion error is therefore unsupported by this evidence.

## Epoch 16–19 conflict

All transforms are `T_a_from_b`. The adjacent chain is `T_16_from_17 @ T_17_from_18 @ T_18_from_19` and matches the placement quotient `inverse(T_display_from_16) @ T_display_from_19`. The direct Stage24D relation and this chain disagree by **44.388 m / 48.878° in the Epoch transform comparison**. The Stage29 **38.474 m / 48.586°** number evaluates a different quantity, the camera boundary jump when the direct placement is substituted at 18→19. The difference between the two metrics is expected when rotation acts around different origins.

Original RGB-D observations were evaluated in each camera pair with `inverse(T_epoch_a_from_camera_i) @ T_epoch_a_from_epoch_b @ T_epoch_b_from_camera_j`. For each adjacent edge the audit takes up to four disjoint fit pairs and four held-out pairs spanning the frozen Stage24E windows. The direct edge uses three near-support held-out pairs with frame indices offset by seven on both sides, with no reuse of the support IDs. These are frame-disjoint and close in viewpoint, so they support the local direct relation without establishing full-Epoch alignment. The JSON records pair IDs and exact times, fitted and frozen SE(3), correspondence counts, directional overlap, inlier RMSE, and cross-pair predictions. ORB metric fitting and seeded ICP are reported separately. ICP shares the frozen initialization and is not independent evidence of global registration. The 35% line in the plot shows one part of the previous gate. Fitness alone cannot verify a bridge.

| Edge | Frozen placement validation | Seeded ICP fit repeatability, largest SE(3) disagreement |
|---|---|---:|
| 16→17 | 2/4 held-out pairs clear 35% each way | 1.153 m / 12.19° |
| 17→18 | 1/4 clears both directions | 0.634 m / 4.29° |
| 18→19 | 1/4 clears both directions, later pairs fall to 15.6%, 0.7%, and 0% in the weaker direction | 0.407 m / 1.20° |
| Direct 16→19 | 3/3 above 35% both ways, with directional inlier RMSE 5.65–6.38 cm | 1.328 m / 2.59° |

The composed chain provides **zero inliers in both directions on all three direct holdout pairs**. The direct Stage24D placement has stronger local geometry than the exploratory chain. To test single-edge blame, the audit substitutes the transform that would be required to close the cycle into each adjacent edge algebraically and evaluates it on the same held-out frames. **All three forced substitutions yield zero inliers on all tested pairs.** The conflict therefore cannot be isolated to a single adjacent edge with the other two placements fixed. The evidence shows limited geometric support across the entire exploratory path. The 18→19 bridge is weakest across the sampled time range, while 16→17 has the largest fit disagreement. Error may be distributed across bridges and within-Epoch tracking. No new edge was promoted.

## Unverified hypotheses and remaining uncertainties

The exact native ORB-SLAM3 event that produced the isolated pose jumps is unknown. Local map adjustment, inaccurate tracking association and drift are plausible and could coexist. The nearest-depth outage and real turn explain why the Epoch 6 gap is hard to constrain. They do not establish how much of the reported rotation is physical. A missing optical mount prevents attributing IMU axes to camera axes, and the near-support 16→19 retest cannot resolve errors far from those viewpoints. The frozen pose export is sufficient to demonstrate an inconsistency with original observations, but it does not contain frame-level native optimizer state for causal attribution.

## Decision and next experiment

- **Retain existing geometry and transforms.** Stage30 findings do not authorize a map rebuild or pose adjustment.
- Prioritize a bounded Epoch 9 segment study. Use disjoint temporal windows around all three jumps, depth-backed visual motion and independent withheld frames to test whether the offset persists or is localized. Require improved relative-pose and geometry residuals before considering a segment repair.
- Treat Epoch 6 as an unobserved interval for metric correction until synchronized endpoint or alternative independent motion evidence is available. Do not replace its endpoint motion with identity.
- Preserve the verified 16→19 Stage24D relation. Future cross-Epoch work should jointly investigate multiple conflicting links on wider, temporally separated windows rather than forcing one edge to satisfy closure.

## Reproduce

Run `stage30_pose_forensics.py --scratch work --imu-db work/stage30/robot3_main_campus_imu_gps.db3` in the Stage24 Python environment with the Stage26 Open3D runtime on `PYTHONPATH`. Use the previously CRC-checked camera database copies and an IMU archive member extracted successfully to ZIP EOF. All inputs are opened read-only. The JSON/CSV and PNG files in this directory are generated under Stage30. The depth panels show synchronized values on a fixed 0.3–10 m scale. Empty panels explicitly indicate missing synchronized depth.
