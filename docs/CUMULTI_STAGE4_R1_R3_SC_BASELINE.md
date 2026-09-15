# CU-Multi Stage 4 — Robot1-to-Robot3 LiDAR-only Scan Context baseline

## Scope and safeguards

Stage 4 is the first sensor-backed retrieval experiment for the Stage 3 hard-but-usable pair. It uses Robot1 as the query stream and Robot3 as the database stream on CU-Multi Main Campus. Robot1 reuses the exact frozen Stage 2 2 Hz keyframe cache and descriptors. Robot3 is sampled from its actual LiDAR timestamps at 2 Hz. The raw archives under `/home/cas/CU-Multi/raw/` are read-only; temporary ROS2 SQLite expansions are removed after decoding and the selected keyframe cache remains outside Git.

Only Scan Context is used for retrieval. OpenCLIP, RGB retrieval, Cross-Max, VLM, CVTNet, GICP and pose-graph optimization are not run. Ground truth is strictly post-retrieval: it defines the offline `d_xy < 5 m` positive labels, overlap, headings and reported metrics, but does not filter the database, select candidates or rerank any candidate. RGB is synchronized for audit only and is not passed to Scan Context.

## Sensor integration and keyframes

Robot3 LiDAR is ROS2 `sensor_msgs/msg/PointCloud2` on `robot3/ouster/points`, with 41,793 source messages. The selected keyframes preserve original message indices and nanosecond timestamps in `/home/cas/CU-Multi/processed_v1/full_2hz/robot3/keyframes.csv`; 4,180 keyframes cover 2,089.511 s, an effective 1.99999 Hz. The PointCloud2 frame is `robot3_os_sensor` and the available fields are `x,y,z,intensity,ambient,range,reflectivity,ring,t`. Processed clouds are finite float32 `[x,y,z,intensity]` arrays in raw sensor coordinates.

Robot3 RGB is ROS2 `sensor_msgs/msg/Image` on `robot3/camera/color/image_raw`, with 20,977 messages. No RGB archive is expanded into a persistent image dataset. Five nearest RGB samples decoded as 1280 x 800 x 3 `uint8`; the synchronization index is recorded outside Git. Nearest-RGB absolute offsets are 26.723 ms mean, 27.552 ms median, 46.900 ms p95 and 127.949 ms maximum. Nearest-GT absolute offsets are 10.623 ms mean, 10.215 ms median, 15.894 ms p95 and 24.795 ms maximum.

The pre-retrieval Robot3 radial (5–30 m) ground-plane histogram mode is -0.550 m in LiDAR coordinates, consistent with Robot1/2 reference modes of about -0.51/-0.47 m. Therefore the frozen fixed +0.50 m Scan Context height handling was retained before evaluation. This is a platform-geometry consistency check, not Recall tuning.

## Frozen Scan Context protocol

The canonical descriptor is unchanged: 20 rings, 60 sectors, 80 m maximum radius, maximum z-plus-fixed-height per cell, 60 circular shifts, and maximum column-wise cosine similarity over valid non-zero columns. Robot1 ranks the entire Robot3 descriptor database (4,180 frames); no GT database filtering occurs. Positives are offline horizontal GT distance below 5 m. Analysis yaw uses the standard ZYX quaternion formula and wrapped absolute difference in [0,180] degrees.

## Primary results

Robot1-to-Robot3 has 2,000 total Robot1 queries, 1,833 valid-overlap queries (91.65%), and 167 no-overlap queries.

| Metric | Value |
| --- | ---: |
| Recall@1 / @5 / @10 / @20 | 0.993453 / 0.996181 / 0.997272 / 0.998363 |
| Recall@50 / @100 / @200 | 0.998909 / 0.998909 / 0.999454 |
| MRR | 0.994682 |
| Median / worst first-positive rank | 1 / 467 |
| Rank-1 failures | 12 |
| Retrieval mean / median / p95 | 155.885 / 155.462 / 162.180 ms per query |
| Total retrieval time | 311.770 s |

The reverse sanity direction, Robot3-to-Robot1, has 1,686 valid-overlap and 2,494 no-overlap queries. Its Recall@1/@5/@10/@20 is 0.998814 / 0.999407 / 0.999407 / 0.999407, with MRR 0.999139. It is a directional sanity result, not averaged into the primary headline.

## Heading-stratified analysis

Heading is analysis-only and uses the nearest GT-positive Robot3 frame. The primary-direction valid-query bins are: 0–30°: 575 queries, R@1 1.000000; 30–60°: 22, 1.000000; 60–90°: 21, 0.809524; 90–120°: 57, 1.000000; 120–150°: 47, 1.000000; 150–180°: 1,111, 0.992799. The 12 Rank-1 failures occur only in the 60–90° (4) and 150–180° (8) bins. This is a diagnostic association, not an input to retrieval.

## Outputs and validation

Lightweight reproducibility outputs are in `outputs/cumulti_v1/04_robot1_robot3_sc/`, including configuration, dataset statistics, positive-pair/overlap audits, per-query evaluation, Recall, latency, heading-stratified metrics, first-positive ranks, failure cases, trajectory/metric figures and five selected LiDAR BEV panels. `VALIDATION_REPORT.txt` records PASS for frozen Robot1 cache reuse, LiDAR timestamp keyframes, finite clouds, GT isolation, RGB non-use, descending rankings, independent first-positive rank agreement, Recall monotonicity and correct valid-overlap denominator.

The 2 Hz selected cloud cache, descriptors, keyframe timestamp tables, RGB synchronization index, raw archives and any temporary bag expansions are intentionally excluded from Git. This stage does not claim a controlled visual/viewpoint improvement; it establishes the hard-pair LiDAR-only reference.

## Reproduction boundary

The completed run must not be rerun over existing derived outputs. If an intentional fresh run is needed, use a new processed/output version:

```bash
conda activate fyp_slam
cd /home/cas/fyp_place_recognition
python src/cumulti/run_stage4_r1_r3_sc_baseline.py
```
