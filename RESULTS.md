# Results

Verified values below are recorded in README_zh.md and existing named historical outputs.

| Experiment | Result |
|---|---|
| Scan Context | R@1 151/158 = 0.955696; MRR 0.963608 |
| Scan Context recall | R@5 154/158; R@10 156/158; R@20 158/158 |
| Visual-only | BEV R@1 0.917722; RGB 0.892405; RGB-3 mean 0.867089; RGB-5 mean 0.854430 |
| Temporal Cross-Max | R@1 0.930380; MRR 0.948312; 1 correction, 5 regressions |
| SC + Cross-Max filtering | 0.6*SC + 0.4*Cross-Max; Top-20 to Top-5: 157/158 retention, 3 rescues, 0 losses |
| SC + Cross-Max ranking | SC weight 0.7: R@1 153/158 = 0.968354; MRR 0.977321; 2 corrections, 0 regressions |
| VLM v2 override | 2 corrections, 3 regressions; R@1 150/158 = 0.949367 |

Initial Top-5 misses are 115, 380, 955 and 1565; their first-positive ranks are 15, 10, 12 and 7. Filtering rescues 115, 955 and 1565; 380 remains a miss.

Historical notes requiring direct output-file recovery: the requested SC + Cross-Max R@1 about 0.9620; canonical Top-20 about 156/158 (README instead records Top-10 156/158 and Top-20 158/158); and BEV yaw mean error about 1.67 degrees. RGB viewpoint invariance is not established conclusively.

## CU-Multi Stage 3 pair-selection result (verified GT-only analysis)

This is not a retrieval metric. Four-robot Main Campus trajectory analysis at 2 Hz with offline `d_xy < 5 m` positives recommends robot1–robot2 as an easy pair (weaker-direction valid queries: 1,907; mean directed overlap: 93.5%) and robot1–robot3 as the hard-but-usable pair (weaker direction: 1,686 valid queries; mean nearest-positive heading difference above 90°: 64.9%; above 150°: 60.9%). Robot1–robot4 is an optional asymmetric stress test (weaker direction: 1,432 valid queries; mean overlap: 57.3%). See `docs/CUMULTI_STAGE3_PAIR_SELECTION.md` and `outputs/cumulti_v1/03_pair_selection/`.
