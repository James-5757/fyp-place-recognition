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

## CU-Multi Stage 4 Robot1-to-Robot3 Scan Context baseline (verified)

This is the first LiDAR-only retrieval result for the Stage 3 hard-but-usable pair. Robot1 uses the frozen Stage 2 2 Hz cache (2,000 queries); Robot3 uses 4,180 actual-LiDAR-timestamp 2 Hz keyframes. With the frozen Scan Context configuration and whole Robot3 database ranking, there are 1,833 valid-overlap queries and 167 no-overlap queries under offline `d_xy < 5 m` positives. Robot1-to-Robot3 Recall@1/@5/@10/@20 is **0.993453 / 0.996181 / 0.997272 / 0.998363**, MRR is **0.994682**, and the median/worst first-positive ranks are **1 / 467**. The reverse Robot3-to-Robot1 sanity direction has 1,686 valid queries and R@1 0.998814. GT is never used for descriptor construction, database filtering or ranking; RGB is synchronized only for audit. See `docs/CUMULTI_STAGE4_R1_R3_SC_BASELINE.md` and `outputs/cumulti_v1/04_robot1_robot3_sc/`.

## CU-Multi Stage 5 Visual Viewpoint Analysis (verified, corrected)

Stage 5 measures visual evidence reliability on the frozen Robot1-to-Robot3 SC Top-20 protocol. OpenCLIP ViT-B-32-quickgelu (laion400m_e32) encodes RGB for all frozen 2 Hz keyframes. Three visual methods rerank only the frozen SC Top-20 candidates.

Bug fixes applied: (1) rank -1 (no GT-positive in Top-20) is never counted as Recall success; (2) rescue/regression computed only within candidate-available queries, excluding candidate-generation failures; (3) end-to-end R@K <= candidate-conditioned R@K asserted; (4) sync-clean checks both Robot1 and Robot3 observations.

Candidate-conditioned (CANDIDATE_AVAILABLE queries only, 1,830 of 1,833 valid):
- Frozen SC: R@1=0.995082, R@5=0.997814, MRR=0.996281
- Single RGB: R@1=0.703279, R@5=0.940984, MRR=0.803501
- RGB5 Mean: R@1=0.683607, R@5=0.914208, MRR=0.777015
- Cross-Max: R@1=0.715847, R@5=0.902732, MRR=0.793528

End-to-end (ALL 1,833 valid queries, candidate-miss = unavoidable miss):
- Frozen SC: R@1=0.993453, R@5=0.996181
- Single RGB: R@1=0.702128, R@5=0.939444
- RGB5 Mean: R@1=0.682488, R@5=0.912711
- Cross-Max: R@1=0.714675, R@5=0.901255

Rescues (corrected): Single=4, Mean5=2, CrossMax=1
Regressions: Single=538, Mean5=572, CrossMax=512
Candidate-generation failures: 3 (zero visual rescues)

12 Stage-4 SC failures: 9 recoverable, 3 candidate-generation failures.

Sync-quality sensitivity: Single RGB 1833/1833 clean (identical); Mean5/CrossMax 1831/1833 clean (slight difference). Sync-clean checks both query AND candidate RGB observations.

See `docs/CUMULTI_STAGE5_VISUAL_VIEWPOINT_ANALYSIS.md` and `outputs/cumulti_v1/05_visual_viewpoint_analysis/`.

## CU-Multi Stage 6 selective visual verification (verified)

Stage 6 keeps Stages 4/5 frozen and uses cached Stage-5 OpenCLIP embeddings with GT-free confidence/viewpoint gates over the frozen SC Top-20. SC stays at **0.993453** R@1 (1,821/1,833); the Top-20 ceiling is **0.998363** (1,830/1,833), with 1847/1848/1849 impossible to rescue. The best-shift proxy is geometrically calibrated offline (MAE 8.604°, median 1.689°, p90 11.047°, Pearson/Spearman 0.923/0.886), but the fixed ablation shows no viewpoint-gating advantage beyond confidence.

Single RGB with SC-margin q5%, visual-margin q50% and compatibility delta 0.05 gives the only zero-regression rescue point: R@1/R@5 **0.993999/0.996727**, 4.31% valid-query invocation, 9 overrides, 1 rescue, 0 regressions, and 1.741 ms/query Model-A added compute. Cross-Max has no zero-regression rescue point. Global fusion at alpha_SC=0.50 is harmful (Single 1 rescue/7 regressions; Cross-Max 0/41). See `docs/CUMULTI_STAGE6_SELECTIVE_VISUAL_VERIFICATION.md` and `outputs/cumulti_v1/06_selective_visual_verification/`.

## CU-Multi Stage 7 held-out generalization validation (verified)

Stage 7 transfers the predeclared Stage-6 Single-RGB strict gate to independent
Robot2-to-Robot3, using frozen 2-Hz grids, unchanged Scan Context, and a one-time
Robot2 OpenCLIP cache. On 2,192 valid-overlap queries, SC achieves **R@1 0.997263,
R@5 1.000000, MRR 0.998223**. Strict transfer invokes visual verification on 115
queries (4.973%), makes 14 overrides, and has **0 rescues / 0 regressions**, leaving
R@1 unchanged. This is neutral held-out transfer, not evidence of a general visual
gain. Robot3-to-Robot2 SC-only sanity has R@1 0.996807 over 1,879 valid queries. See
`docs/CUMULTI_STAGE7_GENERALIZATION_VALIDATION.md` and
`outputs/cumulti_v1/07_generalization_validation/`.
