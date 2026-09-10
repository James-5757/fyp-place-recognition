# CU-Multi Stage 2 — canonical LiDAR-only Scan Context baseline

## Purpose and frozen protocol

Stage 2 establishes the first inter-robot LiDAR retrieval baseline on CU-Multi Main Campus. It uses Robot1 as query and Robot2 as database. The only retrieval modality is LiDAR; RGB, OpenCLIP, Cross-Max, VLM, CVTNet, GICP, PGO and KITTI outputs are out of scope.

**Ground truth is used only for offline overlap definition and evaluation. It is never used for candidate retrieval or ranking.**

The raw data in `/home/cas/CU-Multi/raw/` remains read-only. The generated 2 Hz cache is `/home/cas/CU-Multi/processed_v1/full_2hz/`; it is deliberately excluded from Git.

## Sensor and coordinate validation

The actual PointCloud2 frame IDs are `robot1_os_sensor` and `robot2_os_sensor`. Point fields are `x,y,z,intensity,ambient,range,reflectivity,ring,t`; Scan Context uses x/y as the horizontal plane and z as vertical. The sensor-coordinate ground-plane modes measured before retrieval were approximately -0.51 m and -0.47 m. A fixed 0.50 m height offset was therefore selected before any Recall result and used only for the canonical Scan Context height handling. It is not the old KITTI 2.0 m value.

The URDF gives an `os_sensor` to `base_link` vertical offset of 0.1475 m, but wheel/ground geometry is not sufficiently explicit to use it as physical height by itself. The official CU-Multi README documents 20 Hz `/tf` `world -> lidar/base_link` transforms and RTK-aligned trajectories with a common fixed start. The GT coordinates are therefore used as a common metric world frame for the prescribed `<5 m` offline test. Exact UTM zone/datum metadata was not located in the downloaded files or README and remains an explicit limitation.

## Keyframes and GT labels

LiDAR timestamps alone select a fixed 0.5-second temporal grid, selecting the first LiDAR message at or after each grid time. This yields:

| Robot | Role | Keyframes | Effective rate |
| --- | --- | ---: | ---: |
| robot1 | query | 2,000 | 1.99993 Hz |
| robot2 | database | 2,230 | 1.99992 Hz |

Every cache row preserves robot/keyframe IDs, original LiDAR message index, raw timestamp, nearest GT timestamp and error, position, quaternion and analysis-only yaw. A positive is an offline pair with horizontal GT distance `< 5.0 m`. No-overlap queries are retained; only valid-overlap queries are the Recall denominator.

## Scan Context and computation

The descriptor is the project’s canonical formulation: 20 rings, 60 sectors, 80 m radius; maximum z-plus-fixed-height per cell; 60 circular shifts; column-wise cosine similarity; zero-norm columns excluded; mean valid-column score; best shift is the maximum. The implementation is checked against an explicit 60-shift loop and uses FFT correlation only as an algebraically equivalent acceleration—not a flattened descriptor cosine.

The database is never prefiltered by GT. Each Robot1 query ranks all 2,230 Robot2 descriptors. Compact Top-200 ranks/scores/shifts are stored locally as `full_rankings_top200.npz`; it is intentionally not committed.

## Results

Primary direction, **Robot1 -> Robot2**: 1,907 valid-overlap and 93 no-overlap queries.

| Metric | Value |
| --- | ---: |
| Recall@1 | 0.994756 |
| Recall@5 | 0.995281 |
| Recall@10 | 0.995281 |
| Recall@20 | 0.997378 |
| Recall@50 | 0.998427 |
| Recall@100 | 0.998951 |
| Recall@200 | 0.999476 |
| MRR | 0.995204 |
| Median / worst first-positive rank | 1 / 221 |
| Mean / median / p95 retrieval latency | 42.477 / 42.074 / 45.556 ms per query |
| Total primary retrieval time | 84.954 s |

Secondary reverse sanity check, **Robot2 -> Robot1**: 2,043 valid-overlap queries. R@1/R@5/R@10/R@20 are 0.999021 / 0.999511 / 0.999511 / 0.999511; this is not averaged with the primary headline.

Descriptor construction averaged 1.680 ms per selected keyframe. One descriptor is 4,800 bytes; Robot2’s descriptor database is 10,704,000 bytes.

## Outputs and reproduction

Lightweight results are under `outputs/cumulti_v1/01_sc_baseline/`: protocol, config, statistics, GT positive pairs, overlap status, recalls, query evaluation, first-positive ranks, latency CSV, trajectory/recall/rank figures, summary and validation report. The validation report records PASS for GT isolation, finite clouds and GT matches, namespaced IDs, descending rankings, independent rank-label agreement, monotonic Recall, correct denominator, modality restriction, and non-modification of raw/KITTI data.

Run on the FYP server:

```bash
conda activate fyp_slam
cd /home/cas/fyp_place_recognition
python src/cumulti/run_stage2_sc_baseline.py
```

The implementation creates temporary one-sensor ROS2 SQLite expansions under `processed_v1/.work_stage2`, then deletes them. It caches only selected 2 Hz `.npy` point clouds. Do not run it against an existing `full_2hz` directory unless intentionally replacing this derived subset.
