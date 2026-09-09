import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

from scan_context import load_kitti_bin, make_scan_context, scan_context_similarity


ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
VELODYNE_DIR = KITTI_ROOT / "sequences/00/velodyne"
POSE_PATH = KITTI_ROOT / "poses/00.txt"
K_VALUES = [1, 5, 10, 20, 30, 50]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze full Scan Context candidate ranks for formal split."
    )
    parser.add_argument("--split-frame", type=int, default=2500)
    parser.add_argument("--frame-step", type=int, default=5)
    parser.add_argument("--boundary-gap", type=int, default=100)
    parser.add_argument("--positive-threshold", type=float, default=5.0)
    parser.add_argument(
        "--existing-candidates",
        type=Path,
        default=Path("outputs/formal_split2500_gap100_step5_thr5_candidates.csv"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/sc_candidate_recall_analysis"),
    )
    parser.add_argument("--overwrite-results", action="store_true")
    return parser.parse_args()


def load_poses(path):
    poses = np.loadtxt(path).reshape(-1, 3, 4)
    return poses, poses[:, :, 3]


def sampled_frame_ids(frame_step):
    frame_ids = [int(path.stem) for path in sorted(VELODYNE_DIR.glob("*.bin"))]
    return frame_ids[::frame_step]


def split_robot_a_b(frame_ids, split_frame, boundary_gap):
    robot_a = [frame for frame in frame_ids if frame <= split_frame - boundary_gap]
    robot_b = [frame for frame in frame_ids if frame >= split_frame + boundary_gap]
    return robot_a, robot_b


def xz_distance(position_a, position_b):
    dx = position_a[0] - position_b[0]
    dz = position_a[2] - position_b[2]
    return float(np.sqrt(dx * dx + dz * dz))


def valid_query_ids(robot_a, robot_b, positions, threshold):
    b_positions = positions[robot_b][:, [0, 2]]
    tree = cKDTree(b_positions)
    valid = []
    for query_frame in robot_a:
        distance, _ = tree.query(positions[query_frame][[0, 2]], k=1)
        # Preserve the existing formal baseline's strict threshold semantics.
        if distance < threshold:
            valid.append(query_frame)
    return valid


def build_database(frame_ids):
    database = []
    for index, frame_id in enumerate(frame_ids, start=1):
        points = load_kitti_bin(VELODYNE_DIR / f"{frame_id:06d}.bin")
        database.append({"frame_id": frame_id, "sc": make_scan_context(points)})
        if index % 100 == 0 or index == len(frame_ids):
            print(f"Built descriptors: {index}/{len(frame_ids)}")
    return database


def full_rank(query_descriptor, database_b, query_position, positions, threshold):
    records = []
    for item in database_b:
        score, shift = scan_context_similarity(query_descriptor, item["sc"])
        candidate_frame = item["frame_id"]
        distance = xz_distance(query_position, positions[candidate_frame])
        records.append(
            {
                "candidate_frame": candidate_frame,
                "scan_context_score": float(score),
                "best_shift": int(shift),
                "gt_distance": distance,
                "is_positive": int(distance < threshold),
            }
        )
    records.sort(key=lambda record: (-record["scan_context_score"], record["candidate_frame"]))
    for rank, record in enumerate(records, start=1):
        record["rank"] = rank
    return records


def output_paths(output_dir):
    return [
        output_dir / "recall_curve.csv",
        output_dir / "query_first_positive_rank.csv",
        output_dir / "top5_miss_analysis.csv",
        output_dir / "top1_failure_analysis.csv",
        output_dir / "summary.txt",
    ]


def main():
    args = parse_args()
    if args.output_dir.exists() and not args.overwrite_results:
        existing = [path for path in output_paths(args.output_dir) if path.exists()]
        if existing:
            raise FileExistsError(
                "Refusing to overwrite existing analysis: "
                + ", ".join(str(path) for path in existing)
            )
    if not args.existing_candidates.is_file():
        raise FileNotFoundError(args.existing_candidates)

    _, positions = load_poses(POSE_PATH)
    sampled = sampled_frame_ids(args.frame_step)
    robot_a, robot_b = split_robot_a_b(
        sampled, args.split_frame, args.boundary_gap
    )
    valid_ids = valid_query_ids(
        robot_a, robot_b, positions, args.positive_threshold
    )
    print(f"Robot A frames: {len(robot_a)}")
    print(f"Robot B frames: {len(robot_b)}")
    print(f"Valid queries: {len(valid_ids)}")

    database_a = build_database(robot_a)
    database_b = build_database(robot_b)
    descriptors_a = {item["frame_id"]: item["sc"] for item in database_a}

    query_rows = []
    for index, query_frame in enumerate(valid_ids, start=1):
        ranked = full_rank(
            descriptors_a[query_frame],
            database_b,
            positions[query_frame],
            positions,
            args.positive_threshold,
        )
        first_positive = next(record for record in ranked if record["is_positive"])
        rank1 = ranked[0]
        query_rows.append(
            {
                "query_frame": query_frame,
                "first_positive_rank": first_positive["rank"],
                "first_positive_frame": first_positive["candidate_frame"],
                "first_positive_gt_distance": first_positive["gt_distance"],
                "first_positive_sc_score": first_positive["scan_context_score"],
                "sc_rank1_frame": rank1["candidate_frame"],
                "sc_rank1_gt_distance": rank1["gt_distance"],
                "sc_rank1_score": rank1["scan_context_score"],
                "sc_rank1_positive": rank1["is_positive"],
            }
        )
        if index % 25 == 0 or index == len(valid_ids):
            print(f"Ranked valid queries: {index}/{len(valid_ids)}")

    queries = pd.DataFrame(query_rows)
    recall_rows = []
    for k in K_VALUES:
        recovered = int((queries.first_positive_rank <= k).sum())
        recall_rows.append(
            {
                "k": k,
                "queries_with_positive": recovered,
                "recall_at_k": recovered / len(queries),
            }
        )
    recall_curve = pd.DataFrame(recall_rows)

    recall_at_1 = float(recall_curve.loc[recall_curve.k == 1, "recall_at_k"].iloc[0])
    recall_at_5 = float(recall_curve.loc[recall_curve.k == 5, "recall_at_k"].iloc[0])
    existing = pd.read_csv(args.existing_candidates)
    existing_valid = existing[
        (existing.query_has_positive_in_B.astype(int) == 1)
        & (existing["rank"].astype(int) <= 5)
    ]
    expected_queries = int(existing_valid.query_frame.nunique())
    expected_r1 = float(
        existing_valid[existing_valid["rank"] == 1].is_positive.mean()
    )
    expected_r5 = float(existing_valid.groupby("query_frame").is_positive.max().mean())
    tolerance = 1e-12
    if (
        len(queries) != expected_queries
        or abs(recall_at_1 - expected_r1) > tolerance
        or abs(recall_at_5 - expected_r5) > tolerance
    ):
        raise RuntimeError(
            "Sanity check failed; refusing to write analysis. "
            f"computed queries/R1/R5={len(queries)}/{recall_at_1}/{recall_at_5}, "
            f"existing={expected_queries}/{expected_r1}/{expected_r5}"
        )

    top5_misses = queries[queries.first_positive_rank > 5].copy()
    top5_misses["score_gap"] = (
        top5_misses.sc_rank1_score - top5_misses.first_positive_sc_score
    )
    for k in [10, 20, 30, 50]:
        top5_misses[f"recovered_by_top{k}"] = (
            top5_misses.first_positive_rank <= k
        )
    top5_misses = top5_misses.sort_values("query_frame")

    top1_failures = queries[queries.sc_rank1_positive == 0].copy()
    top1_failures["positive_in_top5"] = top1_failures.first_positive_rank <= 5
    for k in [10, 20, 30, 50]:
        top1_failures[f"positive_in_top{k}"] = (
            top1_failures.first_positive_rank <= k
        )
    top1_failures = top1_failures.sort_values("query_frame")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    recall_curve.to_csv(args.output_dir / "recall_curve.csv", index=False)
    queries.to_csv(args.output_dir / "query_first_positive_rank.csv", index=False)
    top5_misses.to_csv(args.output_dir / "top5_miss_analysis.csv", index=False)
    top1_failures.to_csv(args.output_dir / "top1_failure_analysis.csv", index=False)

    recovered = {
        k: int(recall_curve.loc[recall_curve.k == k, "queries_with_positive"].iloc[0])
        for k in K_VALUES
    }
    increments = {
        "Top5_to_Top10": recovered[10] - recovered[5],
        "Top10_to_Top20": recovered[20] - recovered[10],
        "Top20_to_Top30": recovered[30] - recovered[20],
        "Top30_to_Top50": recovered[50] - recovered[30],
    }
    if recovered[20] >= len(queries) - 1:
        classification = "SC_TOP20_SUFFICIENT_FOR_HIGH_RECALL"
    elif recovered[50] >= len(queries) - 1:
        classification = "SC_TOP50_SUFFICIENT_BUT_TOP20_NOT"
    else:
        classification = "SC_RETRIEVAL_MISSES_REQUIRE_INDEPENDENT_PROPOSAL"

    lines = [
        "SCAN CONTEXT CANDIDATE RECALL ANALYSIS",
        f"Valid queries: {len(queries)}",
        *[
            f"Recall@{k} = {recovered[k] / len(queries):.6f} ({recovered[k]}/{len(queries)})"
            for k in K_VALUES
        ],
        f"Top-5 miss queries: {len(top5_misses)}",
        f"Top-1 failure queries: {len(top1_failures)}",
        "Recovered when expanding candidate pool:",
        *[f"{label} = {value}" for label, value in increments.items()],
        "Top-5 misses:",
    ]
    for row in top5_misses.itertuples(index=False):
        lines.append(
            f"Query {row.query_frame}: first positive rank = {row.first_positive_rank}, "
            f"positive frame = {row.first_positive_frame}, "
            f"GT distance = {row.first_positive_gt_distance:.3f}, "
            f"recovered by Top10/20/30/50 = "
            f"{bool(row.recovered_by_top10)}/{bool(row.recovered_by_top20)}/"
            f"{bool(row.recovered_by_top30)}/{bool(row.recovered_by_top50)}"
        )
    lines.append(f"classification: {classification}")
    (args.output_dir / "summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
