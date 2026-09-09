import argparse
import csv
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from scan_context_canonical import (
    MAX_RADIUS,
    LIDAR_HEIGHT,
    NUM_RINGS,
    NUM_SECTORS,
    aligned_similarity,
    load_kitti_bin,
    make_descriptor,
)

ROOT = Path.home() / "fyp_place_recognition"
KITTI = ROOT / "data/kitti/dataset"
VELO = KITTI / "sequences/00/velodyne"
POSE = KITTI / "poses/00.txt"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out", type=Path,
        default=ROOT / "outputs/canonical_v2/01_scan_context",
    )
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    log = []

    def say(message):
        print(message)
        log.append(message)

    positions = np.loadtxt(POSE).reshape(-1, 3, 4)[:, :, 3]
    sampled = [int(path.stem) for path in sorted(VELO.glob("*.bin"))][::5]
    robot_a = [frame for frame in sampled if frame <= 2400]
    robot_b = [frame for frame in sampled if frame >= 2600]
    tree = cKDTree(positions[robot_b][:, [0, 2]])
    valid = []
    nearest = {}
    for query in robot_a:
        distance, index = tree.query(positions[query][[0, 2]], k=1)
        nearest[query] = (float(distance), int(robot_b[int(index)]))
        if distance < 5.0:
            valid.append(query)
    say(f"Total queries={len(robot_a)} Robot B={len(robot_b)} valid={len(valid)}")

    def build_database(frame_ids, name):
        database = []
        for index, frame in enumerate(frame_ids, 1):
            descriptor = make_descriptor(load_kitti_bin(VELO / f"{frame:06d}.bin"))
            database.append((frame, descriptor))
            if index % 100 == 0 or index == len(frame_ids):
                say(f"{name} descriptors {index}/{len(frame_ids)}")
        return database

    database_a = build_database(robot_a, "Robot A")
    database_b = build_database(robot_b, "Robot B")
    descriptors_a = dict(database_a)
    rows = []
    for index, query in enumerate(robot_a, 1):
        ranked = []
        for candidate, descriptor in database_b:
            score, shift = aligned_similarity(descriptors_a[query], descriptor)
            dx = positions[query][0] - positions[candidate][0]
            dz = positions[query][2] - positions[candidate][2]
            distance = float(np.hypot(dx, dz))
            ranked.append((score, candidate, shift, distance, int(distance < 5.0)))
        ranked.sort(key=lambda item: (-item[0], item[1]))
        for rank, (score, candidate, shift, distance, positive) in enumerate(ranked[:20], 1):
            rows.append([
                query, candidate, rank, score, shift, distance, positive,
                int(query in valid), nearest[query][0], nearest[query][1],
            ])
        if index % 50 == 0 or index == len(robot_a):
            say(f"Ranked {index}/{len(robot_a)}")

    columns = [
        "query_frame", "candidate_frame", "rank", "scan_context_score",
        "best_shift", "gt_distance", "is_positive",
        "query_has_positive_in_B", "nearest_gt_dist_to_B",
        "nearest_gt_frame_in_B",
    ]
    import pandas as pd
    dataframe = pd.DataFrame(rows, columns=columns)
    output_csv = args.out / "canonical_sc_top20_all_queries.csv"
    dataframe.to_csv(output_csv, index=False)
    valid_df = dataframe[dataframe.query_has_positive_in_B == 1]
    recalls = {
        k: float(valid_df[valid_df.rank <= k].groupby("query_frame").is_positive.max().mean())
        for k in [1, 5, 10, 20]
    }
    precisions = {
        k: float(valid_df[valid_df.rank <= k].groupby("query_frame").is_positive.mean().mean())
        for k in [1, 5, 10, 20]
    }
    first_ranks = []
    for _, group in valid_df.groupby("query_frame"):
        first_ranks.append(int(group[group.is_positive == 1].rank.min()))
    metrics = {
        "total_queries": len(robot_a), "robot_b": len(robot_b),
        "valid_queries": len(valid), "no_overlap_queries": len(robot_a) - len(valid),
        "overlap_coverage": len(valid) / len(robot_a),
        **{f"Recall@{k}_valid": recalls[k] for k in recalls},
        **{f"Precision@{k}_valid": precisions[k] for k in precisions},
        "MRR_valid": float(np.mean([1.0 / rank for rank in first_ranks])),
    }
    (args.out / "canonical_sc_metrics.json").write_text(json.dumps(metrics, indent=2))
    config = {
        "dataset": "KITTI Odometry Sequence 00", "split_frame": 2500,
        "boundary_gap": 100, "frame_step": 5, "positive_threshold": "<5.0m x-z",
        "robot_a_rule": "frames <= 2400", "robot_b_rule": "frames >= 2600",
        "descriptor": {"num_rings": NUM_RINGS, "num_sectors": NUM_SECTORS,
                       "max_radius": MAX_RADIUS, "lidar_height": LIDAR_HEIGHT,
                       "radius_keep": "r > 0.0 and r <= 80.0"},
        "similarity": "sector-wise 20D cosine mean over valid columns and circular shift",
        "code_file": "src/canonical_v2/retrieval_canonical.py",
        "created_at": datetime.now(timezone.utc).isoformat(),
    }
    (args.out / "canonical_sc_config.json").write_text(json.dumps(config, indent=2))
    (args.out / "canonical_sc_run.log").write_text("\n".join(log) + "\n")
    expected = {"valid_queries": 158, "Recall@1_valid": 149 / 158,
                "Recall@5_valid": 153 / 158, "Recall@10_valid": 154 / 158}
    mismatch = [key for key, value in expected.items() if
                (len(valid) if key == "valid_queries" else metrics[key]) != value and
                abs((len(valid) if key == "valid_queries" else metrics[key]) - value) > 0.02]
    if mismatch:
        summary = ROOT / "outputs/canonical_v2/09_summary/SC_REPRODUCTION_MISMATCH.md"
        summary.parent.mkdir(parents=True, exist_ok=True)
        summary.write_text("# Canonical Scan Context Reproduction Mismatch\n\n" + json.dumps(metrics, indent=2) + "\n")
        say("REPRODUCTION_MISMATCH_STOP")
        return 2
    say("REPRODUCTION_SANITY_PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
