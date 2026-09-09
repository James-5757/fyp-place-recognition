import argparse
import csv
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from scan_context import load_kitti_bin, make_scan_context, scan_context_similarity


ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
VELODYNE_DIR = KITTI_ROOT / "sequences/00/velodyne"
POSE_PATH = KITTI_ROOT / "poses/00.txt"
OUTPUT_DIR = ROOT / "outputs"

TOP_K_LIST = [1, 5, 10]


def load_poses(pose_path):
    poses = np.loadtxt(pose_path).reshape(-1, 3, 4)
    positions = poses[:, :, 3]
    return poses, positions


def get_sampled_frame_ids(velodyne_dir, frame_step):
    files = sorted(velodyne_dir.glob("*.bin"))
    frame_ids = [int(f.stem) for f in files]
    return frame_ids[::frame_step]


def split_robot_a_b(sampled_frame_ids, split_frame, boundary_gap):
    robot_a = [f for f in sampled_frame_ids if f <= split_frame - boundary_gap]
    robot_b = [f for f in sampled_frame_ids if f >= split_frame + boundary_gap]
    return robot_a, robot_b


def build_descriptor_database(frame_ids):
    db = []
    for i, fid in enumerate(frame_ids):
        bin_path = VELODYNE_DIR / f"{fid:06d}.bin"
        points = load_kitti_bin(bin_path)
        sc = make_scan_context(points)
        db.append({
            "frame_id": fid,
            "sc": sc,
        })

        if (i + 1) % 100 == 0 or (i + 1) == len(frame_ids):
            print(f"Built descriptors: {i + 1}/{len(frame_ids)}")

    return db


def xy_distance(p1, p2):
    dx = p1[0] - p2[0]
    dz = p1[2] - p2[2]
    return float(np.sqrt(dx * dx + dz * dz))


def retrieve_top_k(query_sc, db_b, top_k=10):
    results = []
    for item in db_b:
        score, shift = scan_context_similarity(query_sc, item["sc"])
        results.append({
            "frame_id": item["frame_id"],
            "score": score,
            "shift": shift,
        })
    results.sort(key=lambda x: x["score"], reverse=True)
    return results[:top_k]


def compute_valid_queries(robot_a_ids, robot_b_ids, positions, positive_threshold):
    b_pos_xz = positions[robot_b_ids][:, [0, 2]]
    tree = cKDTree(b_pos_xz)

    valid_info = {}

    for q in robot_a_ids:
        q_pos_xz = positions[q][[0, 2]]
        nearest_dist, nearest_idx = tree.query(q_pos_xz, k=1)

        valid_info[q] = {
            "query_has_positive_in_B": bool(nearest_dist < positive_threshold),
            "nearest_gt_dist_to_B": float(nearest_dist),
            "nearest_gt_frame_in_B": int(robot_b_ids[int(nearest_idx)]),
        }

    return valid_info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--split_frame", type=int, default=2500)
    parser.add_argument("--frame_step", type=int, default=5)
    parser.add_argument("--boundary_gap", type=int, default=100)
    parser.add_argument("--positive_threshold", type=float, default=5.0)
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    poses, positions = load_poses(POSE_PATH)

    sampled_frame_ids = get_sampled_frame_ids(VELODYNE_DIR, args.frame_step)
    robot_a_ids, robot_b_ids = split_robot_a_b(
        sampled_frame_ids,
        args.split_frame,
        args.boundary_gap
    )

    print("=== Formal Baseline Setting ===")
    print(f"split_frame = {args.split_frame}")
    print(f"frame_step = {args.frame_step}")
    print(f"boundary_gap = {args.boundary_gap}")
    print(f"positive_threshold = {args.positive_threshold}")
    print(f"Total sampled frames: {len(sampled_frame_ids)}")
    print(f"Robot A frames: {len(robot_a_ids)}")
    print(f"Robot B frames: {len(robot_b_ids)}")

    valid_info = compute_valid_queries(
        robot_a_ids,
        robot_b_ids,
        positions,
        args.positive_threshold
    )

    valid_query_ids = [
        q for q in robot_a_ids
        if valid_info[q]["query_has_positive_in_B"]
    ]

    print(f"Valid queries with at least one GT positive in Robot B: {len(valid_query_ids)}")
    print(f"Overlap coverage: {len(valid_query_ids) / len(robot_a_ids):.4f}")

    if len(valid_query_ids) == 0:
        print("No valid queries found. Try another split_frame.")
        return

    print("\nBuilding Robot A descriptor database...")
    db_a = build_descriptor_database(robot_a_ids)

    print("\nBuilding Robot B descriptor database...")
    db_b = build_descriptor_database(robot_b_ids)

    suffix = f"formal_split{args.split_frame}_gap{args.boundary_gap}_step{args.frame_step}"
    csv_path = OUTPUT_DIR / f"{suffix}_candidates.csv"
    metrics_path = OUTPUT_DIR / f"{suffix}_metrics.txt"

    recall_hits = {k: 0 for k in TOP_K_LIST}
    precision_sum = {k: 0.0 for k in TOP_K_LIST}

    num_valid_queries = len(valid_query_ids)
    valid_query_set = set(valid_query_ids)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "query_frame",
            "candidate_frame",
            "rank",
            "scan_context_score",
            "best_shift",
            "gt_distance",
            "is_positive",
            "query_has_positive_in_B",
            "nearest_gt_dist_to_B",
            "nearest_gt_frame_in_B",
        ])

        processed_valid = 0

        for q_idx, q_item in enumerate(db_a):
            query_frame = q_item["frame_id"]
            query_sc = q_item["sc"]
            query_pos = positions[query_frame]

            top_results = retrieve_top_k(query_sc, db_b, top_k=max(TOP_K_LIST))

            positives_in_topk = {k: False for k in TOP_K_LIST}
            positive_counts = {k: 0 for k in TOP_K_LIST}

            for rank, cand in enumerate(top_results, start=1):
                cand_frame = cand["frame_id"]
                cand_pos = positions[cand_frame]
                dist = xy_distance(query_pos, cand_pos)
                is_positive = int(dist < args.positive_threshold)

                writer.writerow([
                    query_frame,
                    cand_frame,
                    rank,
                    cand["score"],
                    cand["shift"],
                    dist,
                    is_positive,
                    int(valid_info[query_frame]["query_has_positive_in_B"]),
                    valid_info[query_frame]["nearest_gt_dist_to_B"],
                    valid_info[query_frame]["nearest_gt_frame_in_B"],
                ])

                if query_frame in valid_query_set:
                    for k in TOP_K_LIST:
                        if rank <= k and is_positive == 1:
                            positives_in_topk[k] = True
                            positive_counts[k] += 1

            if query_frame in valid_query_set:
                processed_valid += 1

                for k in TOP_K_LIST:
                    if positives_in_topk[k]:
                        recall_hits[k] += 1
                    precision_sum[k] += positive_counts[k] / k

                if processed_valid % 50 == 0 or processed_valid == num_valid_queries:
                    print(f"Processed valid queries: {processed_valid}/{num_valid_queries}")

    recalls = {k: recall_hits[k] / num_valid_queries for k in TOP_K_LIST}
    precisions = {k: precision_sum[k] / num_valid_queries for k in TOP_K_LIST}

    with open(metrics_path, "w") as f:
        f.write("Formal Baseline Retrieval Metrics\n")
        f.write(f"split_frame = {args.split_frame}\n")
        f.write(f"frame_step = {args.frame_step}\n")
        f.write(f"boundary_gap = {args.boundary_gap}\n")
        f.write(f"positive_threshold = {args.positive_threshold}\n")
        f.write(f"Robot A size = {len(robot_a_ids)}\n")
        f.write(f"Robot B size = {len(robot_b_ids)}\n")
        f.write(f"Total queries = {len(robot_a_ids)}\n")
        f.write(f"Valid queries = {num_valid_queries}\n")
        f.write(f"Overlap coverage = {num_valid_queries / len(robot_a_ids):.4f}\n")
        for k in TOP_K_LIST:
            f.write(f"Recall@{k}_valid = {recalls[k]:.4f}\n")
        for k in TOP_K_LIST:
            f.write(f"Precision@{k}_valid = {precisions[k]:.4f}\n")

    print("\n=== Formal Baseline Results ===")
    print(f"Total queries: {len(robot_a_ids)}")
    print(f"Valid queries: {num_valid_queries}")
    print(f"Overlap coverage: {num_valid_queries / len(robot_a_ids):.4f}")
    for k in TOP_K_LIST:
        print(f"Recall@{k}_valid: {recalls[k]:.4f}")
    for k in TOP_K_LIST:
        print(f"Precision@{k}_valid: {precisions[k]:.4f}")

    print(f"\nSaved candidates to: {csv_path}")
    print(f"Saved metrics to: {metrics_path}")


if __name__ == "__main__":
    main()
