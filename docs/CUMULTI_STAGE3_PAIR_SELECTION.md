# CU-Multi Stage 3 — four-robot GT-only pair selection

## Scope

Stage 3 uses only the four small Main Campus `*_gt_utm_poses.csv` pose files for offline pair selection. No robot3/4 LiDAR, RGB, depth, label, or ROS-bag archive was downloaded, read, or extracted. No Scan Context, OpenCLIP, Cross-Max, GICP, CVTNet, or VLM algorithm was run.

Robot3 and robot4 pose files were placed under the raw dataset root without modifying any existing raw content. Their sizes are 6.4 MB and 8.4 MB respectively, and their documented commented schema is `timestamp,x,y,z,qx,qy,qz,qw`.

## Protocol

- Robot1/robot2 preserve the already-fixed Stage-2 LiDAR timestamp grid (2,000 / 2,230 keyframes), with nearest GT associated only for offline analysis.
- Robot3/robot4 have no sensor data locally. They are sampled from GT at the first record at or after a 0.5 s timestamp grid (4,180 / 5,544 samples). This is strictly timestamp-based, not pose-distance sampling.
- A directed query is valid if it has one or more database samples satisfying `d_xy < 5 m`.
- For heading statistics only, the nearest valid GT-positive is selected, then standard ZYX yaw is derived from its xyzw quaternion and wrapped absolute difference is reported in `[0,180]` degrees.
- GT positions, headings, overlaps, and positive labels are offline analysis/evaluation data only. They do not select candidates or enter a retrieval/reranking method.

## Directed overlap results

| Direction | Valid / query | Overlap | Positive count median / p95 | Heading >90 / >150 |
|---|---:|---:|---:|---:|
| robot1 → robot2 | 1,907 / 2,000 | 95.35% | 18 / 226 | 57.52% / 51.13% |
| robot2 → robot1 | 2,043 / 2,230 | 91.61% | 15 / 195 | 59.62% / 54.92% |
| robot1 → robot3 | 1,832 / 2,000 | 91.60% | 15 / 147 | 66.38% / 60.75% |
| robot3 → robot1 | 1,686 / 4,180 | 40.33% | 14 / 195 | 63.35% / 61.09% |
| robot1 → robot4 | 1,776 / 2,000 | 88.80% | 12 / 77 | 50.96% / 47.52% |
| robot4 → robot1 | 1,432 / 5,544 | 25.83% | 14 / 194 | 53.00% / 50.07% |
| robot2 → robot3 | 2,192 / 2,230 | 98.30% | 17 / 147 | 15.74% / 15.19% |
| robot3 → robot2 | 1,879 / 4,180 | 44.95% | 19 / 226 | 11.60% / 10.27% |
| robot2 → robot4 | 2,041 / 2,230 | 91.52% | 13 / 77 | 21.61% / 19.84% |
| robot4 → robot2 | 1,698 / 5,544 | 30.63% | 15 / 112.15 | 20.73% / 17.26% |
| robot3 → robot4 | 3,767 / 4,180 | 90.12% | 16 / 57 | 27.61% / 24.77% |
| robot4 → robot3 | 3,528 / 5,544 | 63.64% | 18 / 43 | 30.90% / 26.67% |

## Recommendation

1. **Easy:** robot1–robot2. Its weaker direction has 1,907 valid queries and mean directed overlap is 93.5%.
2. **Hard-but-usable:** robot1–robot3. Its weaker direction retains 1,686 valid queries while its mean nearest-positive heading difference above 90° is 64.9% (above 150°: 60.9%). It was selected for a strong viewpoint challenge with meaningful bilateral support, not merely because overlap is lower.
3. **Optional extreme stress test:** robot1–robot4. Its weaker direction has 1,432 valid queries and mean directed overlap is 57.3%; it should be interpreted as a more asymmetric evaluation.

The machine-readable audit tables and figures are in `outputs/cumulti_v1/03_pair_selection/`. The selection must remain a GT-only diagnostic until the user explicitly authorizes downloading the required robot3 or robot4 sensor archives and starting a retrieval experiment.
