# CU-Multi Main Campus four-robot pair recommendation

## Scope and protocol

This is GT-only offline pair selection. No LiDAR/RGB/depth/labels/ROS archive was read, extracted, or downloaded; no retrieval or registration algorithm was run.

All ground-truth CSVs use `timestamp,x,y,z,qx,qy,qz,qw`. Yaw is the standard ZYX yaw derived from each GT xyzw quaternion. robot1/robot2 retain their exact Stage-2 LiDAR timestamp grid with nearest GT association; robot3/robot4 use first GT record at/after an anchored 0.5 s grid because no sensor data were downloaded.

For every ordered direction, a query is valid when at least one database sample has `d_xy < 5 m`. Heading analysis picks the *nearest* such GT-positive only after forming labels. GT positions and headings are not inputs to a retrieval method.

## Recommendations

1. **Easy pair:** `robot1-robot2` — weaker-direction valid queries: 1907; mean directional overlap: 0.935; mean nearest-positive heading >90°: 0.586; >150°: 0.530. Chosen for substantial mutual overlap, not for its retrieval score.
2. **Hard-but-usable pair:** `robot1-robot3` — weaker-direction valid queries: 1686; mean directional overlap: 0.660; mean nearest-positive heading >90°: 0.649; >150°: 0.609. Chosen by high viewpoint reversal while retaining a meaningful bilateral evaluation set; it is not selected merely for low overlap.
3. **Extreme stress-test pair:** `robot1-robot4` — weaker-direction valid queries: 1432; mean directional overlap: 0.573; mean nearest-positive heading >90°: 0.520; >150°: 0.488. This is optional and should be interpreted with its lower overlap/sample support in mind.

## Output interpretation

- `pair_selection_summary.csv` has 12 directional records with all requested counts, overlap, positive-count quantiles, and heading-tail fractions.
- `pair_heading_statistics.csv` has the six requested heading bins per direction.
- `pair_overlap_matrix.csv` is a directional long-form matrix suitable for direct auditing; the corresponding PNG uses query rows and database columns.

All counts are determined from the sampled GT trajectories and therefore are useful only for offline dataset/pair selection and later evaluation stratification.
