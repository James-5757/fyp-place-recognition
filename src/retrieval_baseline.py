import csv
from pathlib import Path

import numpy as np

from scan_context import load_kitti_bin, make_scan_context, scan_context_similarity


ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
VELODYNE_DIR = KITTI_ROOT / "sequences/00/velodyne"
POSE_PATH = KITTI_ROOT / "poses/00.txt"
OUTPUT_DIR = ROOT / "outputs"

FRAME_STEP = 5
TOP_K_LIST = [1, 5, 10]
POSITIVE_THRESHOLD = 5.0  # meters


def load_poses(pose_path):
    poses = np.loadtxt(pose_path).reshape(-1, 3, 4)
    positions = poses[:, :, 3]
    return poses, positions


def get_sampled_frame_ids(velodyne_dir, frame_step=5):
    files = sorted(velodyne_dir.glob("*.bin"))
    frame_ids = [int(f.stem) for f in files]
    sampled = frame_ids[::frame_step]
    return sampled


def split_robot_a_b(sampled_frame_ids):
    """
    Debug split:
    even-indexed sampled frames -> Robot A
    odd-indexed sampled frames  -> Robot B
    """
    robot_a = sampled_frame_ids[::2]
    robot_b = sampled_frame_ids[1::2]
    return robot_a, robot_b


def build_descriptor_database(frame_ids):
    db = []
    for fid in frame_ids:
        bin_path = VELODYNE_DIR / f"{fid:06d}.bin"
        points = load_kitti_bin(bin_path)
        sc = make_scan_context(points)
        db.append({
            "frame_id": fid,
            "sc": sc,
        })
    return db


def xy_distance(p1, p2):
    # KITTI mostly moves in x-z plane
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


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    poses, positions = load_poses(POSE_PATH)

    sampled_frame_ids = get_sampled_frame_ids(VELODYNE_DIR, FRAME_STEP)
    robot_a_ids, robot_b_ids = split_robot_a_b(sampled_frame_ids)

    print(f"Total sampled frames: {len(sampled_frame_ids)}")
    print(f"Robot A frames: {len(robot_a_ids)}")
    print(f"Robot B frames: {len(robot_b_ids)}")

    print("Building Robot A descriptor database...")
    db_a = build_descriptor_database(robot_a_ids)

    print("Building Robot B descriptor database...")
    db_b = build_descriptor_database(robot_b_ids)

    csv_path = OUTPUT_DIR / "baseline_candidates.csv"
    metrics_path = OUTPUT_DIR / "baseline_metrics.txt"

    recall_hits = {k: 0 for k in TOP_K_LIST}
    num_queries = len(db_a)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "query_frame",
            "candidate_frame",
            "rank",
            "scan_context_score",
            "best_shift",
            "gt_distance",
            "is_positive"
        ])

        for q_idx, q_item in enumerate(db_a):
            query_frame = q_item["frame_id"]
            query_sc = q_item["sc"]
            query_pos = positions[query_frame]

            top_results = retrieve_top_k(query_sc, db_b, top_k=max(TOP_K_LIST))

            # Save all top-K candidates
            positives_in_topk = {k: False for k in TOP_K_LIST}

            for rank, cand in enumerate(top_results, start=1):
                cand_frame = cand["frame_id"]
                cand_pos = positions[cand_frame]
                dist = xy_distance(query_pos, cand_pos)
                is_positive = int(dist < POSITIVE_THRESHOLD)

                writer.writerow([
                    query_frame,
                    cand_frame,
                    rank,
                    cand["score"],
                    cand["shift"],
                    dist,
                    is_positive
                ])

                for k in TOP_K_LIST:
                    if rank <= k and is_positive == 1:
                        positives_in_topk[k] = True

            for k in TOP_K_LIST:
                if positives_in_topk[k]:
                    recall_hits[k] += 1

            if (q_idx + 1) % 50 == 0 or (q_idx + 1) == num_queries:
                print(f"Processed queries: {q_idx + 1}/{num_queries}")

    recalls = {k: recall_hits[k] / num_queries for k in TOP_K_LIST}

    with open(metrics_path, "w") as f:
        f.write("Baseline Retrieval Metrics\n")
        f.write(f"FRAME_STEP = {FRAME_STEP}\n")
        f.write(f"POSITIVE_THRESHOLD = {POSITIVE_THRESHOLD}\n")
        f.write(f"Number of queries = {num_queries}\n")
        f.write(f"Robot A size = {len(robot_a_ids)}\n")
        f.write(f"Robot B size = {len(robot_b_ids)}\n")
        for k in TOP_K_LIST:
            f.write(f"Recall@{k} = {recalls[k]:.4f}\n")

    print("\n=== Baseline Results ===")
    print(f"Number of queries: {num_queries}")
    for k in TOP_K_LIST:
        print(f"Recall@{k}: {recalls[k]:.4f}")

    print(f"\nSaved candidates to: {csv_path}")
    print(f"Saved metrics to: {metrics_path}")


if __name__ == "__main__":
    main()
