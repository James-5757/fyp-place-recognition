# CU-Multi Stage 2.5 — Scan Context diagnostic analysis

Stage 2.5 is an offline analysis of the frozen Stage 2 Robot1-to-Robot2 Scan Context baseline. It does not alter the frozen baseline, raw data, or KITTI outputs, and does not run OpenCLIP, Cross-Max, CVTNet, VLM, GICP or a new retrieval experiment.

## Metadata fix

The Stage 2 generator previously parsed `robot1_to_robot2` with a generic underscore split, writing `database_robot=to`. `src/cumulti/run_stage2_sc_baseline.py` now splits on `_to_`, so future generated rows correctly state `query_robot=robot1` and `database_robot=robot2`. The frozen Stage 2 CSV and metric values are intentionally unchanged.

## Why canonical Scan Context is already high

The primary protocol has 1,907 valid-overlap queries out of 2,000 Robot1 queries (95.35%) against a dense 2,230-frame Robot2 database. The two runs use the same platform, common globally aligned world frame, and very dense 2 Hz sampling. Those conditions produce many geometrically near-duplicate LiDAR observations.

There is a shared static/common-start component: Robot1 remains within 5 m of its initial position for 97.0 s and Robot2 for 112.5 s. The shared window is 97.0 s; the maximum early Robot1 positive count is 231 Robot2 frames. But this is not the dominant explanation: R@1 changes only from 0.994756 (all valid overlap queries) to 0.994159 after excluding the common-start period and 0.993835 for moving Robot1 queries. The strong score is therefore principally a consequence of dense spatial overlap and repeated route geometry, with initial stationarity providing a modest easy-query contribution.

Motion uses an offline threshold of local speed <= 0.10 m/s over the prior 0.5-second interval. It finds 287/2,000 stationary-or-near-stationary Robot1 frames (14.3%) and 373/2,230 Robot2 frames (16.7%). Conditional valid-overlap fraction remains high after retaining only moving Robot1 queries (94.7%) or only queries outside common start (94.8%).

## Existing-ranking-only cohorts

| Cohort | Valid queries | R@1 | R@5 | R@10 | R@20 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Canonical all valid | 1,907 | 0.994756 | 0.995281 | 0.995281 | 0.997378 |
| Moving Robot1 query | 1,622 | 0.993835 | 0.994451 | 0.994451 | 0.996917 |
| Outside common start | 1,712 | 0.994159 | 0.994743 | 0.994743 | 0.997079 |

These values are recomputed only from frozen per-query ranks; no full retrieval was rerun.

## Failures and heading analysis

All ten canonical Rank-1 failures are independently recovered: 811–814, 1754–1755 and 1847–1850. Their Rank-1 false candidates are 228–268 m away, while their nearest positive is 0.18–4.75 m away. The first six failures have nearest-positive heading difference about 179 degrees; the remaining four are about 76–78 degrees. Their rank-1 score margins over the best positive are 0.016–0.111, demonstrating strong LiDAR structural aliases rather than GT-selection mistakes.

For valid-overlap queries, nearest-positive heading differences are concentrated in 0–30 degrees (784, 41.1%) and 150–180 degrees (975, 51.1%). The ten SC Rank-1 failures occur only in 60–90 degrees (4) or 150–180 degrees (6). This is the proper stratification protocol for the future visual/viewpoint study; headings remain strictly post-retrieval analysis labels.

Each failure has an RGB contact sheet and LiDAR BEV panel showing the query, highest-SC GT-positive candidate, and false Rank-1 candidate. RGB is diagnostic-only and was not provided to retrieval.

## Hard-pair planning

Only robot1 and robot2 UTM GT CSVs are local. Robot1–robot2 has 95.35% query overlap and 57.5% of valid overlaps exceed 90 degrees heading difference, yet Scan Context remains near-saturated. Do not download robot3/4 sensor archives. First download only `robot3_main_campus_gt_utm_poses.csv` and `robot4_main_campus_gt_utm_poses.csv`, then estimate overlap and heading distributions for all candidate pairs before selecting a harder pair.

## Outputs

`outputs/cumulti_v1/02_stage2_diagnostics/` contains motion statistics, cohort recall, all failure records, heading bins, hard-pair availability/recommendation tables, trajectory and heading figures, per-failure RGB/LiDAR panels, and the detailed summary. Ground truth is used only for offline motion/overlap/heading analysis and evaluation; it is never used in candidate generation or ranking.
