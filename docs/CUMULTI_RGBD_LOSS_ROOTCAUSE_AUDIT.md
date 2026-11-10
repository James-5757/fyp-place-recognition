# RGB-D tracking-loss cause audit

Date: 2026-11-10 (Asia/Shanghai). Frozen result parent: `6f57e35`. No SLAM, retrieval, graph optimization or GT evaluation rerun.

**These are lost tracking states, not missing image files.** All2400 RGB/depth inputs in the previous study exist and were consumed; paired exposure headers match. The accepted NO_GO decision is unchanged.

## What is established

The native source separates image detection, existing-map tracking, relocalization and map lifecycle. If visual tracking fails, the non-inertial RGB-D branch enters RECENTLY_LOST and attempts relocalization. After more than3.0s without recovery, it becomes LOST and starts a new map or resets a small active map. The observed roughly3.1s intervals are this recovery policy plus the next input/reinitialization frame, not three seconds of missing sensor data. The constructor’s5s recently-lost parameter is used by a different inertial branch; it is not the active RGB-D timeout. Two one-frame gaps happen after creation of tiny new maps and coincide with active-reset logs.

Native feature extraction has now been checked directly using the actual ORB-SLAM3 ORBextractor, saved1000/2000 YAML and original640x400 JPEG/depth images. This does NOT instantiate the SLAM system. The selected cohort includes every loss interval±20 frames and one-per-second controls: 1112 offline feature records across both capacities. Selection is failure-conditioned diagnostics, not a fresh unbiased benchmark.

| Robot/config | First bad frame | Test time s | Gap s | Saved map associations before→bad | Native features / with depth | Adjacent 2D F inliers | Motion px |
|---|---:|---:|---:|---:|---:|---:|---:|
| robot1 fallback2000 | 345 | 34.500 | 3.116 | 32→27 | 1933 / 379 | 834 | 3.79 |
| robot1 fallback2000 | 377 | 37.699 | 0.103 | 399→0 | 1968 / 427 | 719 | 7.28 |
| robot1 fallback2000 | 957 | 95.704 | 3.100 | 30→22 | 2003 / 537 | 706 | 2.40 |
| robot1 fallback2000 | 989 | 98.930 | 0.090 | 526→0 | 1984 / 526 | 698 | 3.52 |
| robot3 baseline1000 | 255 | 25.588 | 3.110 | 117→22 | 1006 / 269 | 237 | 14.93 |
| robot3 fallback2000 | 9 | 0.899 | 3.099 | 72→44 | 2005 / 924 | 540 | 3.22 |
| robot3 fallback2000 | 327 | 32.696 | 3.207 | 31→25 | 2005 / 1445 | 546 | 2.44 |

Frame IDs here are RGB-D runner input indices, not LiDAR keyframe IDs. Add roughly120s to these test times for the original camera timeline; exact source camera times and original RGB IDs are in loss_episode_evidence.csv.

## Robot3 default configuration: strongest localized evidence

At frame255 (test25.588s, original RGB1455), saved map associations fall117→22. But the same image still has1006 native keypoints,269 measured-depth keypoints,292 mutual adjacent matches and237 fundamental-matrix inliers. Only42 adjacent matches have measured depth in both frames. From frames250–255, median matched image displacement rises from about12.1 to14.9px per frame, and mean absolute image difference rises from21.4 to26.6 gray levels. This is compatible with viewpoint/motion and photometric change stressing existing-map tracking, not an absence of image texture. It is not a unique causal proof.

The existing map is not recovered: the observer sees RECENTLY_LOST, then NO_IMAGES_YET as the native internal LOST/new-map path executes, then OK in map1 at28.697s. A transient internal LOST state need not appear as state4 in the post-call CSV. Do not join the new identity-frame poses onto the old trajectory.

## Why increasing feature count did not cure it

All six fallback first-bad frames retain hundreds/thousands of native keypoints and substantial adjacent-image geometric matches. Robot3 fallback fails as early as0.899s despite2005 detected features and924 with depth, so fast motion or absent depth cannot explain every episode. Robot1 baseline tracks the SAME input images through frames where fallback fails. Feature capacity changes which spatial features and map points are admitted, reference/map history, keyframe decisions and workload; more detections are not a monotonic guarantee of more reliable established-map inliers. Causal allocation among those mechanisms remains unmeasured.

## Limits and next diagnostic

The saved tracked_points count is nonbad associated MapPoints, not native mnMatchesInliers. The log “Fail to track local map!” is printed when bOK is false, even when an earlier motion/reference tracking step already failed. Native RGB-D normally requires30 local-map inliers and50 near recent relocalization; there is also a >10-inlier early-success branch in RECENTLY_LOST. These state-dependent rules cannot be reconstructed from saved association counts alone. Independent adjacent-frame fundamental inliers are also not native map/PnP inliers or GT-correct matches.

Previous OpenCV-ORB depth-support proxies are not native extraction measurements. They also used rounding; native Frame::ComputeStereoFromRGBD truncates the original pixel coordinates. This audit uses the exact native detector and lookup rule. A3x3 depth consistency count is a diagnostic, not an applied filter or calibrated confidence.

A conclusive next intervention is a separate logging-only replay of the same failing interval with its initialization history, recording TrackWithMotionModel/TrackReferenceKeyFrame/TrackLocalMap return values, pre/post optimization inliers, reprojection/depth residuals and map/keyframe statistics. No such replay was run here. Merely increasing the3s timeout changes recovery waiting time; it does not repair the first tracking failure. No threshold or historical metric has been altered, and no fusion was started.
