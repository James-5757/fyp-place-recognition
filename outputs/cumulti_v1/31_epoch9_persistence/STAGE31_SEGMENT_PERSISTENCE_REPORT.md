# Stage31 Epoch9 segment persistence analysis

Baseline Stage30 commit 99bef28a83df8f855b65b46295a73aa001659e77. Original camera poses, Epoch transforms, graph constraints and maps were preserved.

## Frozen protocol

Each event uses 30 valid frames before and 31 from the first post-event frame. Fit frames are pre offsets −30/−26/−21 and post offsets +10/+14/+19. Validation frames are pre −17/−10/−3 and post +0/+3/+6/+23/+26/+30. All timestamps are original bag/header times. Fit and validation IDs are disjoint with at least 300 ms exclusion. There are nine cross-boundary fit pairs, eighteen validation pairs and six within-segment controls per event. The exact frame plan is written before registration and remains available with the policy.

RGB-D ORB metric RANSAC, identity-seeded ICP and FPFH/RANSAC-seeded ICP use original measured depth with frozen color calibration. No frozen pose initializes either geometric method. Fit geometry samples pixel offset 0 and geometry validation samples offset 4, at stride 8. The prior minimums remain 300 inliers, 35% overlap in both directions and spatial spread 0.05 at 12 cm. Visual hypotheses additionally require 60 mutual / 40 metric / 30 inlier correspondences, 50% support and fit RMSE at most 0.1 m. Each accepted candidate needs three unique frame IDs on each side spanning at least 0.8 s in fit and withheld support. Identity is an explicit initialization, not a forced motion estimate.

A supplementary pass uses Shi-Tomasi corners selected on valid depth and bidirectional RGB optical flow, followed by measured-depth rigid RANSAC. This was added after sparse depth-backed ORB support was observed in attempt1. The original attempt is preserved. The same frame cohorts, exclusion buffer and geometric acceptance gates are reused. Candidate transforms without a stable fit cluster are explicitly unsupported; they are still evaluated and stored to expose failure rather than skipped.

## Decisions

| Event | Classification | Candidate fit/validation support |
|---|---|---|
| 9935→9936 | INCONCLUSIVE | orb_metric: 0/0, icp_identity: 9/0, fpfh_ransac_icp: 8/0, rgbd_flow_ransac: 0/0 |
| 10070→10071 | INCONCLUSIVE | orb_metric: 0/0, icp_identity: 9/0, fpfh_ransac_icp: 8/0, rgbd_flow_ransac: 0/0 |
| 12056→12057 | INCONCLUSIVE | orb_metric: 0/0, icp_identity: 9/0, fpfh_ransac_icp: 9/0, rgbd_flow_ransac: 0/0 |

## Fit-only candidates evaluated on held-out data

All candidate matrices below failed stable fit support. They remain UNSUPPORTED_DIAGNOSTIC_CANDIDATE. Zero held-out pairs met the combined candidate, transform-consistency and geometric gates. A numerical improvement alone cannot qualify a segment correction.

| Event / method | Frozen → candidate translation residual median (m) | Rotation median (deg) | Bidirectional overlap median |
|---|---|---|---|
| 9936 / icp_identity | 5.530 → 1.953 | 4.10 → 3.67 | 2.8% / 5.8% |
| 9936 / fpfh_ransac_icp | 5.356 → 3.215 | 16.54 → 17.01 | 0.0% / 0.0% |
| 10071 / icp_identity | 1.104 → 3.389 | 2.51 → 3.19 | 0.0% / 0.0% |
| 10071 / fpfh_ransac_icp | 1.077 → 81.795 | 3.92 → 178.37 | 0.0% / 0.0% |
| 12057 / icp_identity | 4.273 → 3.201 | 8.67 → 6.29 | 0.1% / 0.1% |
| 12057 / fpfh_ransac_icp | 4.243 → 2.226 | 8.32 → 9.93 | 4.7% / 17.9% |

## Cross-boundary persistence findings

Neither fit-only geometric method produced a consistent correction cluster spanning three distinct frames on each side. ORB produced no qualified fit pairs, and depth-masked optical flow produced no qualified pairs in this cohort. The direct ORB validation pair 10068→10071 passes the single-pair gate, but is insufficient for a segment conclusion.

Per-fixed-pre-frame residual trends are in temporal_progression_evidence. Repeated post-event geometric estimates often continue to disagree with the frozen trajectory beyond the first frame and grow with temporal separation. The within-segment controls also disagree with frozen relative motion. This is drift-like diagnostic evidence, but depth registration ambiguity remains unresolved, including a roughly 176 degree fit-only FPFH correction for event 10071. The experiment does not establish an isolated spike, constant segment offset, or accumulated drift with independent evidence. All three events therefore remain INCONCLUSIVE.

## Interpretation and limitations

The decision tables retain every rejected pair. Motion estimates sharing a frame are correlated. Frame ID/time exclusions prevent fitting on held-out observations, but nearby viewpoints still observe the same scene. ICP and FPFH corroboration share depth and an objective, so they provide independent initialization evidence rather than independent sensors. Inlier RMSE is censored to 12 cm and must be interpreted together with overlap and counts. Missing values mean no supported transform or no inliers.

A rigid candidate uses C_epoch_left_on_post = T_epoch_from_cam_i @ T_cam_i_from_cam_j @ inverse(T_epoch_from_cam_j). It is fitted only on the fit cohort. Withheld predicted motion is inverse(T_epoch_from_cam_i) @ C @ T_epoch_from_cam_j. Only a few seconds of continuation are tested. No candidate justifies extrapolation over the rest of Epoch9 without further held-out windows. Rejected evidence cannot establish whether a one-frame spike, constant segment offset or varying drift is present.

## Recommended action per event

- Frames 9935→9936: Retain original poses; candidate matrices remain diagnostic. Observed scope 9906–9966; no pose edits. The frozen multi-frame gates or method consistency requirements were not met.
- Frames 10070→10071: Retain original poses; candidate matrices remain diagnostic. Observed scope 10041–10101; no pose edits. The frozen multi-frame gates or method consistency requirements were not met.
- Frames 12056→12057: Retain original poses; candidate matrices remain diagnostic. Observed scope 12027–12087; no pose edits. The frozen multi-frame gates or method consistency requirements were not met.

## Reproduce

Run stage31_persistence.py --scratch work in the isolated Stage24 Python environment with the Stage26 Open3D runtime on PYTHONPATH. Original CRC-validated camera SQLite copies and frozen Stage24 pose/sync/coverage/calibration tables are required. The runner verifies original message IDs, timestamps and per-frame depth coverage. Preservation SHA256s are written after completion.
