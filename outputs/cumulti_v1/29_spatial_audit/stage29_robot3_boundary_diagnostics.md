# Stage29 spatial consistency audit

Baseline is Stage28 `718be3e256763fb4d650adcf9d6adefb923563b0`. Stage29 reads the frozen assets and writes only its own audit outputs and viewer. The calibrated RGB-D chunks were reused byte for byte.

## Robot3 boundary diagnosis

The per-Epoch CSV covers all 26 native Epochs with points and valid pose ranges, frame IDs, display transforms, placement provenance, the two directional overlap results, tracking events and per-Epoch plane evidence. Recomposition of the original camera poses exactly reproduces the browser trajectories within 1e-8 m. All four Stage24D forest transforms agree with the directed atlas composition. Reversing the 16→19 edge changes its result by 0.357 m, which does not explain the 38 m disagreement.

| Robot3 boundary | Evidence | Decision |
|---|---|---|
| 0→1 | Map ID changed and 31 frames lacked poses. Stage24D verified the relative constraint. | Retain the verified relative placement. |
| 1→2 | Map ID changed after 31 invalid frames and Epoch 2 has a single valid observation. | Retain the estimated continuation and report insufficient geometric support. |
| 2→3, 3→4, 4→5 | Map ID stays at 2 while the initial keyframe changes. Epochs 2–5 have 1, 1, 2 and 2 valid poses. These transitions lack independent temporal support. | Retain the existing display estimates. |
| 8→9 | Original frames 9178 and 9179 are 92.4 ms apart with zero invalid frames. The Map ID stays at 4. FPFH bidirectional aggregate overlap is 65.1% / 45.4%, with zero qualifying independent repetitions. Baseline overlap is 59.8% / 26.5%. | Retain the 0.136 m motion-based display step. The initial-KF change has no accompanying tracking loss. |
| 18→19 | Original frames 18910 and 18911 are 101.7 ms apart and no frames were lost. Map ID stays at 11 while the initial KF changes. Both Stage26 methods fail depth validation. Baseline overlap is 40.7% / 22.0%, FPFH overlap is 32.9% / 21.3%. | Retain the 0.171 m / 0.31° display-only continuation. Keep the frozen 16→19 relationship for verified component analysis. |

The 38.474 m / 48.59° gap appears when a verified 16→19 placement is compared against the route propagated through the exploratory 16→17 and 17→18 placements. That incompatibility is measured in the current display chain. Available data cannot apportion it between local drift, exploratory edge error and native internal map updates. The boundary audit brackets native merge events but does not attribute the exact frame. Neither the frame pairing nor the matrix-direction check supports relabeling this as a tracking reset or reversing a transform.

## Within-Epoch geometry and gravity

The isolated Robot3 snapshots render every sampled Epoch in a local frame recovered by inverting its rigid display placement. Native camera pose order and point distances are preserved. Epoch 6 has a maximum native pose increment of 5.02 m and Epoch 9 reaches 2.06 m, so inter-Epoch placement cannot explain all discontinuities. These are pose-step diagnostics, not measured spatial errors against an independent reconstruction. Shrinking a gap between two Epochs does not repair internal drift or native pose jumps.

There is still no verified IMU-to-color-optical transform in the Stage28 bag and URDF audit. A global RGB-D leveling plane remains unsupported. Robot3 Epochs 6, 8 and 9 pass the local normal stability criterion, with p90 spreads of 3.97°, 2.24° and 4.26°. Robot1 Epoch 4 also passes. For these Epochs the isolated view applies the local rigid rotation to both cloud and trajectory. The paired transform (T_{display←leveled}=T_{display←epoch} R^{-1}) is exported, so recomposition preserves the historical global placement. The failed windows can reflect blended terrain, sampling bias or camera drift. No frame mounting or absolute gravity is inferred from these fits.

## Robot3 LiDAR diagnosis

Robot3's largest display-only replacements are two adjacent 1.625 m steps near 26 s, replacing raw EKF steps of 92.76 m and 130.88 m. The replacements can smear the early scan accumulation. They lie before the frozen Stage12D start at 117 s. The stable Robot3 LiDAR trajectory already spans 363.87 m in local Z before optimization and 364.24 m after optimization. Timestamp-matched valid GNSS1 altitude observations span 12.39 m over the same retained keyframe sequence. The multi-hundred-metre vertical excursion therefore predates the PGO result and cannot plausibly be explained by terrain alone. GNSS was inspected offline only and does not override the pose gauge. No Stage29 LiDAR pose correction was applied.

## Metric comparison and deliverables

The new Stage29 viewer supports Auto Fit and a shared per-sensor Fixed Metric View. Fixed uses an orthographic camera with one vertical metre span, origin, orientation and reset zoom for Robot1, Robot3 and Combined. The optional full-recording boxes show the actual envelopes and their center displacement. RGB-D box center difference R3−R1 is approximately (74.9, −12.4, 76.3) m in display coordinates. These are envelope statistics, not a robot-to-robot alignment measurement. Playback marker separations are shown only with an independent-clock label. Perspective fit and the metric comparison use unchanged model geometry.

Use the same HTTP server as Stage28 and open `/stage29_spatial_audit/?sensor=rgbd&metric=1`. Use **End** for full geometry and **Top** for the shared XY view. Under Auto Fit, choose an Epoch and enable local isolation. The screenshot set contains Auto Fit and Fixed Metric views plus the 20 isolated Robot3 Epochs.

No independently validated replacement was found. The individual and combined exploratory RGB-D maps therefore reference the byte-identical Stage28 artifacts in `map_reference_manifest.json`. Stage29 adds local attitude inspection and corrects the scale comparison. It makes zero edits to the global poses. The six PLYs remain stored with Stage28.

## Reproduce

On the dataset server run `src/cumulti/rgbd_full/stage29_spatial_audit.py` with the Stage28 environment and runtime assets. The companion `stage29_lidar_audit.py` only needs the frozen CSVs. Copy the generated `metric_view.json` and `stage29_epoch_transforms.json` to `demo/stage29_spatial_audit/`. Run `stage29_validate.py` against the new viewer to generate the browser validation record and screenshots.
