import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
POSE_PATH = KITTI_ROOT / "poses/00.txt"
OUT_DIR = ROOT / "outputs/case_analysis"


def load_positions():
    poses = np.loadtxt(POSE_PATH).reshape(-1, 3, 4)
    return poses[:, :, 3]


def plot_topk_case(query_frame, group, positions, out_path, title, topk=5):
    group = group.sort_values("rank").head(topk)

    q_pos = positions[int(query_frame)]

    plt.figure(figsize=(8.5, 8.5))
    plt.plot(positions[:, 0], positions[:, 2], linewidth=0.7, alpha=0.25, label="KITTI 00 trajectory")

    plt.scatter(q_pos[0], q_pos[2], s=100, marker="*", label=f"Robot A query {query_frame}")

    for _, row in group.iterrows():
        c = int(row["candidate_frame"])
        c_pos = positions[c]
        rank = int(row["rank"])
        dist = float(row["gt_distance"])
        score = float(row["scan_context_score"])
        is_pos = int(row["is_positive"])

        marker = "o" if is_pos == 1 else "x"
        label = f"Rank {rank}: {c}, dist={dist:.1f}m, score={score:.3f}"
        plt.scatter(c_pos[0], c_pos[2], s=70, marker=marker, label=label)
        plt.plot([q_pos[0], c_pos[0]], [q_pos[2], c_pos[2]], linestyle="--", linewidth=0.8, alpha=0.6)

        plt.text(c_pos[0], c_pos[2], f"R{rank}", fontsize=8)

    plt.xlabel("x position")
    plt.ylabel("z position")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.legend(fontsize=7, loc="best")
    plt.tight_layout()
    plt.savefig(out_path, dpi=220)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--candidate_csv",
        type=str,
        default=str(ROOT / "outputs/formal_split2500_gap100_step5_thr5_candidates.csv")
    )
    parser.add_argument("--topk", type=int, default=5)
    parser.add_argument("--max_cases", type=int, default=5)
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    positions = load_positions()
    df = pd.read_csv(args.candidate_csv)

    # Only analyze valid queries that actually have a GT positive candidate in Robot B
    valid_df = df[df["query_has_positive_in_B"] == 1].copy()

    groups = list(valid_df.groupby("query_frame"))

    top1_success = []
    top1_failure = []
    reranking_opportunity = []
    top10_failure = []

    for query_frame, group in groups:
        group = group.sort_values("rank")

        rank1 = group[group["rank"] == 1]
        if rank1.empty:
            continue

        rank1_positive = int(rank1.iloc[0]["is_positive"]) == 1
        topk_positive = (group[group["rank"] <= args.topk]["is_positive"] == 1).any()
        top10_positive = (group[group["rank"] <= 10]["is_positive"] == 1).any()

        if rank1_positive:
            top1_success.append(query_frame)
        else:
            top1_failure.append(query_frame)

        if (not rank1_positive) and topk_positive:
            reranking_opportunity.append(query_frame)

        if not top10_positive:
            top10_failure.append(query_frame)

    summary_path = OUT_DIR / "case_summary.txt"
    with open(summary_path, "w") as f:
        f.write("Retrieval Case Analysis\n")
        f.write(f"candidate_csv = {args.candidate_csv}\n")
        f.write(f"valid queries = {len(groups)}\n")
        f.write(f"top1 success = {len(top1_success)}\n")
        f.write(f"top1 failure = {len(top1_failure)}\n")
        f.write(f"reranking opportunities, top-{args.topk} positive but rank-1 wrong = {len(reranking_opportunity)}\n")
        f.write(f"top-10 retrieval failures = {len(top10_failure)}\n")

    # Save reranking opportunity query list
    rerank_csv = OUT_DIR / "reranking_opportunity_queries.csv"
    pd.DataFrame({"query_frame": reranking_opportunity}).to_csv(rerank_csv, index=False)

    print("=== Retrieval Case Analysis ===")
    print("Valid queries:", len(groups))
    print("Top-1 success:", len(top1_success))
    print("Top-1 failure:", len(top1_failure))
    print(f"Reranking opportunities, Top-{args.topk} contains positive but Rank-1 wrong:", len(reranking_opportunity))
    print("Top-10 failures:", len(top10_failure))
    print("Saved summary to:", summary_path)
    print("Saved reranking query list to:", rerank_csv)

    # Plot several cases
    def plot_cases(case_list, folder_name, title_prefix):
        case_dir = OUT_DIR / folder_name
        case_dir.mkdir(parents=True, exist_ok=True)

        for i, q in enumerate(case_list[:args.max_cases]):
            group = valid_df[valid_df["query_frame"] == q]
            out_path = case_dir / f"{folder_name}_{i}_query{q}.png"
            plot_topk_case(
                query_frame=q,
                group=group,
                positions=positions,
                out_path=out_path,
                title=f"{title_prefix}: Query {q}",
                topk=args.topk
            )

    plot_cases(top1_success, "top1_success", "Top-1 Success")
    plot_cases(top1_failure, "top1_failure", "Top-1 Failure")
    plot_cases(reranking_opportunity, "reranking_opportunity", f"Reranking Opportunity, Top-{args.topk} Has Positive")
    plot_cases(top10_failure, "top10_failure", "Top-10 Failure")

    print("Saved case figures to:", OUT_DIR)


if __name__ == "__main__":
    main()
