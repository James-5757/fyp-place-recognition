import argparse
import os
from pathlib import Path

import pandas as pd
import torch


WINDOW_OFFSETS = [-20, -15, -10, -5, 0]
CASES = {
    1545: [(4525, "WRONG"), (4540, "CORRECT")],
    1550: [
        (3555, "WRONG"),
        (3550, "FALSE_CANDIDATE"),
        (4540, "CORRECT"),
    ],
    1560: [(3545, "WRONG"), (4535, "CORRECT")],
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Three-case temporal cross-frame RGB diagnostic"
    )
    parser.add_argument(
        "--candidate-csv",
        type=Path,
        default=Path(
            "outputs/formal_split2500_gap100_step5_thr5_candidates.csv"
        ),
    )
    parser.add_argument(
        "--yaw-csv",
        type=Path,
        default=Path("outputs/yaw_validation/all_candidate_yaw_validation.csv"),
    )
    parser.add_argument(
        "--single-scores-csv",
        type=Path,
        default=Path("outputs/rgb_clip_sanity/case_scores.csv"),
    )
    parser.add_argument(
        "--mean-scores-csv",
        type=Path,
        default=Path("outputs/temporal_rgb_reranking/candidate_scores.csv"),
    )
    parser.add_argument(
        "--frame-cache",
        type=Path,
        default=Path(
            "outputs/temporal_rgb_reranking/cache/rgb_frame_embeddings.pt"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/temporal_cross_frame_cases"),
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def require_file(path, description):
    if not path.is_file():
        raise FileNotFoundError(f"Missing {description}: {path}")


def unique_row(dataframe, query_frame, candidate_frame, description):
    rows = dataframe[
        (dataframe.query_frame.astype(int) == query_frame)
        & (dataframe.candidate_frame.astype(int) == candidate_frame)
    ]
    if len(rows) != 1:
        raise ValueError(
            f"Expected one {description} row for {query_frame}->{candidate_frame}; "
            f"found {len(rows)}"
        )
    return rows.iloc[0]


def load_cache(path):
    payload = torch.load(path, map_location="cpu", weights_only=True)
    entries = payload.get("entries", {})
    return {int(frame): entry["embedding"].float() for frame, entry in entries.items()}


def temporal_frames(target, embeddings):
    return [
        target + offset
        for offset in WINDOW_OFFSETS
        if target + offset >= 0 and target + offset in embeddings
    ]


def similarity_matrix(query_frames, candidate_frames, embeddings):
    query = torch.stack([embeddings[frame] for frame in query_frames])
    candidate = torch.stack([embeddings[frame] for frame in candidate_frames])
    return query @ candidate.T


def cross_scores(query_frames, candidate_frames, embeddings):
    matrix = similarity_matrix(query_frames, candidate_frames, embeddings)
    flat = matrix.reshape(-1)
    top_n = min(3, flat.numel())
    top_values, top_indices = torch.topk(flat, k=top_n)
    max_index = int(torch.argmax(flat))
    max_query_index = max_index // matrix.shape[1]
    max_candidate_index = max_index % matrix.shape[1]
    query_to_candidate_values, query_to_candidate_indices = matrix.max(dim=1)
    candidate_to_query_values, candidate_to_query_indices = matrix.max(dim=0)
    query_to_candidate = float(query_to_candidate_values.mean())
    candidate_to_query = float(candidate_to_query_values.mean())
    symmetric = 0.5 * (query_to_candidate + candidate_to_query)
    return {
        "cross_max": float(flat.max()),
        "cross_top3": float(top_values.mean()),
        "cross_symmetric": symmetric,
        "best_query_temporal_frame": query_frames[max_query_index],
        "best_candidate_temporal_frame": candidate_frames[max_candidate_index],
        "max_pair_similarity": float(flat.max()),
        "query_to_candidate_score": query_to_candidate,
        "candidate_to_query_score": candidate_to_query,
        "symmetric_score": symmetric,
        "query_temporal_frame_count": len(query_frames),
        "candidate_temporal_frame_count": len(candidate_frames),
        "matrix_rows": matrix.shape[0],
        "matrix_cols": matrix.shape[1],
        "top3_matrix_indices": ";".join(str(int(index)) for index in top_indices),
    }


def main():
    args = parse_args()
    for path, description in [
        (args.candidate_csv, "candidate CSV"),
        (args.yaw_csv, "yaw CSV"),
        (args.single_scores_csv, "single RGB case scores"),
        (args.mean_scores_csv, "temporal mean candidate scores"),
        (args.frame_cache, "individual RGB frame cache"),
    ]:
        require_file(path, description)
    if args.output_dir.exists() and not args.overwrite:
        raise FileExistsError(
            f"Output directory exists; use --overwrite: {args.output_dir}"
        )
    args.output_dir.mkdir(parents=True, exist_ok=True)

    candidates = pd.read_csv(args.candidate_csv)
    yaw = pd.read_csv(args.yaw_csv)
    single = pd.read_csv(args.single_scores_csv)
    mean_scores = pd.read_csv(args.mean_scores_csv)
    embeddings = load_cache(args.frame_cache)
    records = []
    for query_frame, candidate_list in CASES.items():
        query_frames = temporal_frames(query_frame, embeddings)
        for candidate_frame, label in candidate_list:
            candidate = unique_row(
                candidates, query_frame, candidate_frame, "candidate"
            )
            yaw_row = unique_row(yaw, query_frame, candidate_frame, "yaw")
            single_row = unique_row(
                single, query_frame, candidate_frame, "single case score"
            )
            mean_row = unique_row(
                mean_scores, query_frame, candidate_frame, "temporal mean score"
            )
            candidate_frames = temporal_frames(candidate_frame, embeddings)
            cross = cross_scores(query_frames, candidate_frames, embeddings)
            row = {
                "query_frame": query_frame,
                "candidate_frame": candidate_frame,
                "label": label,
                "sc_rank": int(candidate["rank"]),
                "scan_context_score": float(candidate["scan_context_score"]),
                "gt_distance": float(candidate["gt_distance"]),
                "is_positive": int(candidate["is_positive"]),
                "gt_relative_yaw_deg": float(yaw_row["gt_relative_yaw_deg"]),
                "single_rgb_similarity": float(
                    single_row["rgb_cosine_similarity"]
                ),
                "temporal_5_mean_similarity": float(
                    mean_row["temporal_5_rgb_similarity"]
                ),
                **cross,
            }
            records.append(row)
    scores = pd.DataFrame(records)

    summaries = []
    for query_frame, group in scores.groupby("query_frame", sort=True):
        wrong = group[group.label == "WRONG"].iloc[0]
        correct = group[group.label == "CORRECT"].iloc[0]
        false = group[group.label != "CORRECT"]
        strongest = false.loc[false.cross_symmetric.idxmax()]
        for method in [
            "single_rgb_similarity",
            "temporal_5_mean_similarity",
            "cross_max",
            "cross_top3",
            "cross_symmetric",
        ]:
            strongest_false = float(false[method].max())
            correct_similarity = float(correct[method])
            wrong_similarity = float(wrong[method])
            summaries.append(
                {
                    "query_frame": int(query_frame),
                    "representation": method,
                    "wrong_candidate_frame": int(wrong.candidate_frame),
                    "correct_candidate_frame": int(correct.candidate_frame),
                    "strongest_false_candidate_frame": int(
                        false.loc[false[method].idxmax(), "candidate_frame"]
                    ),
                    "wrong_similarity": wrong_similarity,
                    "correct_similarity": correct_similarity,
                    "strongest_false_similarity": strongest_false,
                    "pairwise_gap": correct_similarity - wrong_similarity,
                    "full_case_gap": correct_similarity - strongest_false,
                    "pairwise_corrected": bool(correct_similarity > wrong_similarity),
                    "case_corrected": bool(correct_similarity > strongest_false),
                }
            )
    summary = pd.DataFrame(summaries)
    scores.to_csv(args.output_dir / "case_scores.csv", index=False)
    summary.to_csv(args.output_dir / "case_summary.csv", index=False)
    report = "\n".join(
        [
            "TEMPORAL CROSS-FRAME RGB CASE SANITY CHECK",
            "",
            summary.to_string(
                index=False, float_format=lambda value: f"{value:.6f}"
            ),
            "",
            "Cross-frame score definitions:",
            "cross_max = max of all query/candidate frame similarities",
            "cross_top3 = mean of three highest matrix values",
            "cross_symmetric = 0.5 * (query-to-candidate + candidate-to-query)",
        ]
    )
    (args.output_dir / "summary.txt").write_text(report + "\n")
    print("\n" + report)
    print(f"Saved results to: {args.output_dir}")


if __name__ == "__main__":
    main()
