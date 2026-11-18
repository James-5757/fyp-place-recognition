# Stage31 Epoch9 segment persistence analysis

Baseline Stage30 commit 99bef28a83df8f855b65b46295a73aa001659e77. Original camera poses, Epoch transforms, graph constraints and maps were preserved.

## Frozen protocol

Each event uses 30 valid frames before and 31 from the first post-event frame. Fit frames are pre offsets −30/−26/−21 and post offsets +10/+14/+19. Validation frames are pre −17/−10/−3 and post +0/+3/+6/+23/+26/+30. All timestamps are original bag/header times. Fit and validation IDs are disjoint with at least 300 ms exclusion. There are nine cross-boundary fit pairs, eighteen validation pairs and six within-segment controls per event. The exact frame plan is written before registration and remains available with the policy.

RGB-D ORB metric RANSAC, identity-seeded ICP and FPFH/RANSAC-seeded ICP use original measured depth with frozen color calibration. No frozen pose initializes either geometric method. Fit geometry samples pixel offset 0 and geometry validation samples offset 4, at stride 8. The prior minimums remain 300 inliers, 35% overlap in both directions and spatial spread 0.05 at 12 cm. Visual hypotheses additionally require 60 mutual / 40 metric / 30 inlier correspondences, 50% support and fit RMSE at most 0.1 m. Each accepted candidate needs three unique frame IDs on each side spanning at least 0.8 s in fit and withheld support. Identity is an explicit initialization, not a forced motion estimate.

## Decisions

| Event | Classification | Candidate fit/validation support |
|---|---|---|
| 9935→9936 | INCONCLUSIVE | orb_metric: 0/0, icp_identity: 9/0, fpfh_ransac_icp: 8/0 |
| 10070→10071 | INCONCLUSIVE | orb_metric: 0/0, icp_identity: 9/0, fpfh_ransac_icp: 8/0 |
| 12056→12057 | INCONCLUSIVE | orb_metric: 0/0, icp_identity: 9/0, fpfh_ransac_icp: 9/0 |

## Interpretation and limitations

The decision tables retain every rejected pair. Motion estimates sharing a frame are correlated. Frame ID/time exclusions prevent fitting on held-out observations, but nearby viewpoints still observe the same scene. ICP and FPFH corroboration share depth and an objective, so they provide independent initialization evidence rather than independent sensors. Inlier RMSE is censored to 12 cm and must be interpreted together with overlap and counts. Missing values mean no supported transform or no inliers.

A rigid candidate uses C_epoch_left_on_post = T_epoch_from_cam_i @ T_cam_i_from_cam_j @ inverse(T_epoch_from_cam_j). It is fitted only on the fit cohort. Withheld predicted motion is inverse(T_epoch_from_cam_i) @ C @ T_epoch_from_cam_j. Only a few seconds of continuation are tested. No candidate justifies extrapolation over the rest of Epoch9 without further held-out windows. Rejected evidence cannot establish whether a one-frame spike, constant segment offset or varying drift is present.

## Recommended action per event

- Frames 9935→9936: Retain original poses; candidate matrices remain diagnostic. Observed scope 9906–9966; no pose edits. The frozen multi-frame gates or method consistency requirements were not met.
- Frames 10070→10071: Retain original poses; candidate matrices remain diagnostic. Observed scope 10041–10101; no pose edits. The frozen multi-frame gates or method consistency requirements were not met.
- Frames 12056→12057: Retain original poses; candidate matrices remain diagnostic. Observed scope 12027–12087; no pose edits. The frozen multi-frame gates or method consistency requirements were not met.

## Reproduce

Run stage31_persistence.py --scratch work in the isolated Stage24 Python environment with the Stage26 Open3D runtime on PYTHONPATH. Original CRC-validated camera SQLite copies and frozen Stage24 pose/sync/coverage/calibration tables are required. The runner verifies original message IDs, timestamps and per-frame depth coverage. Preservation SHA256s are written after completion.
