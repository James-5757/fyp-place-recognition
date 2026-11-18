# Stage32 native tracking forensics

**Status BLOCKED_NATIVE_REPLAY_DEPENDENCIES.** Original-camera evidence and independent native feature extraction completed. Full native tracking replay, pose-optimizer instrumentation and Local Mapping instrumentation could not be executed.

## Provenance and blocker

Parent commit 1859291a0d448ccf62b9e3317f2b2055c5497b25. The original SDK directory, baseline native executable and original ORB-SLAM3 library were absent in the current server inventory. Project backups and the available source archives did not contain them. A fresh public source checkout at the historically recorded SDK commit 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4 recovered source for inspection. Four SHA256 values match the historical Tracking.cc, Frame.cc, ORBextractor.cc and Tracking.h receipts after normalizing the checkout to original LF line endings. System.cc and System.h differ from the original working-tree receipts. Earlier audit metadata explicitly records preexisting SDK changes. Their actual patches are unavailable, so the public tree cannot establish the original System behavior.

| Required original component | Historical SHA256 | Current status |
|---|---|---|
| Tracking executable | 4833b5c64656d62549f26c3494ccd5e96bf724e6acfb173ea0450525b9c1728f | Missing |
| ORB-SLAM3 library | ddb134dca4a630c702af5cf4d5acc0cd4d6159433e78a09884d27163605e8252 | Missing |
| Vocabulary | f8dd027f7a6cb88129821341194d7f2c75b77b3394257ddd0d2229863d1a3570 | Original path missing; recovered copy audited separately |

The committed camera.yaml and 1000-feature settings were recovered. The original sequence.txt is absent. The sequence hash in provenance is explicitly a reconstruction from frozen sync rows and original absolute paths, not a fresh verification of the missing file. Original RGB/depth bags and the CRC-validated SQLite copies remain available. Selected input JPEGs use the original Stage24 resize/JPEG85 procedure; historical full-resolution JPEG byte hashes were not retained.

## Actual source path

The hash-matched GrabImageRGBD converts BGR to grayscale and uint16 depth into float metres with factor 1/1000. Frame::ComputeStereoFromRGBD truncates keypoint coordinates for lookup, accepts d>0 and forms the virtual right-image coordinate kpU.x−mbf/d. The tracker can attempt motion-model tracking, reference-keyframe tracking, local-map tracking and relocalization depending on state. The actual frame-by-frame branches are unavailable.

TrackLocalMap recomputes mnMatchesInliers from associated, non-outlier MapPoints. The nominal non-inertial RGB-D threshold is 30, with 50 after recent relocalization and an earlier success branch above 10 in RECENTLY_LOST. These tests do not themselves validate physical inter-frame speed. The saved tracked_points field comes from GetTrackedMapPoints and MapPoint validity votes and cannot reconstruct native mnMatchesInliers.

The recovered upstream PoseOptimization uses g2o pose-only mono/stereo edges, with four scheduled rounds of 10 iterations and chi-square thresholds 5.991 and 7.815. Robust kernels are removed after the third classification pass, and a small edge count can end the rounds early. These are static reference settings, not executed iteration counts. The historical optimizer file has no retained content hash, so its identity with that source cannot be asserted. Source locations and SHA256 values are recorded in stage32_run_provenance.json.

## Directly measured image/depth evidence

The standalone program compiles the hash-matched native ORBextractor source. It executes that image detector with exactly 1000 features, scale 1.2, eight levels and FAST 20/7 on 33 restored target/control-window images. It also executes the documented native pixel truncation and depth lookup. This produces detector and depth-support measurements. It never creates a System or calls TrackRGBD, and observes no map associations, optimizer or Local Mapping thread. The feature table keeps historical associated-point counts in a separately named column. Current native tracking fields remain empty.

| Event frame | Native detector keypoints | With measured depth | Prior frame depth support | Following frame depth support |
|---|---:|---:|---:|---:|
| 9936 | 1006 | 331 | 329 | 288 |
| 10071 | 1001 | 640 | 616 | 636 |
| 12057 | 1004 | 296 | 319 | 289 |

The detector-level depth counts remain comparable to neighboring controls in all three events. This supports absence of a sudden detector-level depth-support collapse. The optimizer-retained depth constraints remain unmeasured. Native keypoint depth order statistics use sorted native float depths, and close keypoints use the unchanged Stereo.b × Stereo.ThDepth = 2 m definition. The feature-support and depth-region plots include the six neighboring normal intervals. Image-quality CSVs contain brightness, contrast, Laplacian variance and forward/backward-consistent Shi-Tomasi/LK flow. These are source-image indicators with explicitly identified algorithms. A 0.3 m 3×3 depth discontinuity count is diagnostic only and filters no depth.

## Research questions

1. **Why State 2 can coexist with a bad pose.** Confirmed static Tracking.cc logic accepts based on native map inlier criteria rather than an independent sensor-geometry or speed check. The actual criterion and counts satisfied at each target remain unavailable.
2. **Wrong association or optimizer failure.** Unresolved. No native pre/post optimization poses, MapPoint IDs, chi-square distributions or executed branch traces are available.
3. **Depth support collapse.** The table reports directly re-executed detector support. This can test a detector-level collapse, but the number of depth-supported correspondences retained by the actual historical optimizer remains unmeasured.
4. **Local Mapping effects.** Unresolved. Historical native keyframe/MapPoint/BA events are absent and no replay was started.
5. **Following-frame recovery.** Historical geometry is more consistent on adjacent control pairs in Stage30. Native map update or tracking recovery cannot be attributed without executed stage logs.
6. **Shared mechanism.** Unresolved. Similar external symptoms do not establish identical internal causes.
7. **Smallest justified next intervention.** Restore the authenticated System.cc/System.h working tree or patch, baseline native dependencies and original build context. Then make a separate SDK copy, add logging to the inspected native tracking/optimizer/lifecycle operations and replay frame 0 through at least 12087. No threshold or pose correction is justified.

## Evidence levels and decisions

Native historical root-cause classifications for all three events remain UNRESOLVED_NATIVE_EVIDENCE_UNAVAILABLE. The historical parent/source hashes, missing dependencies and direct feature-extractor observations are confirmed. Feature association error, optimizer instability, depth-correspondence weakness, asynchronous Local Mapping and visual ambiguity remain unverified hypotheses at the tracking stage. No causal level is assigned from image correlation.

The CSV files for native tracking, optimization and Local Mapping explicitly mark unavailable measurements. No pose-before/after, reprojection/chi-square distribution, native-inlier or native lifecycle plot is fabricated. The three delivered plots show available image/extractor measurements. Stage32 therefore remains incomplete at the native replay gate.

## Preservation and reproduction

Run stage32_preflight.py --scratch work after installing the frozen baseline receipts and recovered reference files under work/stage32. The feature probe builds a separate executable in that scratch tree. Original data, camera poses, SDK locations and baseline executables are never written. stage32_preservation_audit.json verifies protected input hashes. The report is a committed blocker checkpoint that can be resumed after the authenticated native dependencies are restored.
