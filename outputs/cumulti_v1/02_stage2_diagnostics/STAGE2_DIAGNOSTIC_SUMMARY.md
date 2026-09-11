# Stage 2.5 diagnostic summary

## Frozen baseline preserved

Stage 2 ranking and metric files were read only. The historical `database_robot=to` metadata bug is fixed in the generator for future runs, but frozen Stage 2 CSV values were not overwritten.

## Why primary R@1 is high

Robot1 has 97.0s and Robot2 has 112.5s continuously within 5m of its own start. The shared common-start window is 97.0s. Early Robot1 queries therefore have many Robot2 frames in the same small start region (maximum positive count: 231), which inflates easy-match contribution. The canonical all-query score remains frozen; this diagnosis also reports moving and outside-common-start cohorts.

Motion threshold: stationary/near-stationary means local speed <=0.10 m/s across the preceding 0.5s interval.

Canonical/moving/outside-common-start valid-overlap counts are 1907/1622/1712. Their overlap fractions relative to all Robot1 queries are 0.954/0.811/0.856; conditional overlap fractions after retaining only moving/outside-common-start queries are 0.947/0.948. Recall@K for those cohorts is in `moving_only_recall.csv`.

| robot | stationary frames | fraction | moving frames | travelled distance m |
|---|---:|---:|---:|---:|
| robot1 | 287 | 0.143 | 1713 | 1130.5 |
| robot2 | 373 | 0.167 | 1857 | 1367.0 |

See `moving_only_recall.csv`, `failure_cases.csv`, `heading_bin_statistics.csv`, and `failure_case_panels/` for the exact diagnostic records. All ten canonical Rank-1 failures are independently found at query IDs 811, 812, 813, 814, 1754, 1755, 1847, 1848, 1849, 1850.

## Viewpoint protocol

For the next stage, retain this fixed 2Hz Robot1->Robot2 protocol and its offline `<5m` GT labels. Evaluate visual methods by the heading bins in `heading_bin_statistics.csv`, reporting all valid overlap queries plus Rank-1 SC failures separately. Do not provide GT headings, distances, or overlap status to candidate generation/ranking; use them only after rankings for stratified evaluation.

## Hard-pair planning

Only robot1/robot2 GT CSVs are present. Do not download LiDAR/RGB for robot3/4. Download only `robot3_main_campus_gt_utm_poses.csv` and `robot4_main_campus_gt_utm_poses.csv` first, then run the same lightweight overlap/heading inventory to choose a harder pair.
