import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import pandas as pd
import torch


WINDOW_OFFSETS = [-20, -15, -10, -5, 0]
DEFAULT_ALPHAS = [0.9, 0.8, 0.7, 0.6, 0.5]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Full temporal cross-frame RGB OpenCLIP reranking"
    )
    parser.add_argument(
        "--candidate-csv",
        type=Path,
        default=Path(
            "outputs/formal_split2500_gap100_step5_thr5_candidates.csv"
        ),
    )
    parser.add_argument(
        "--single-scores-csv",
        type=Path,
        default=Path("outputs/full_rgb_reranking/candidate_scores.csv"),
    )
    parser.add_argument(
        "--temporal-scores-csv",
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
        default=Path("outputs/temporal_cross_frame_reranking"),
    )
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--alphas", type=float, nargs="+", default=DEFAULT_ALPHAS)
    parser.add_argument("--overwrite-results", action="store_true")
    return parser.parse_args()


def require_file(path, description):
    if not path.is_file():
        raise FileNotFoundError(f"Missing {description}: {path}")


def validate_candidates(dataframe, top_k):
    required = {
        "query_frame",
        "candidate_frame",
        "rank",
        "scan_context_score",
        "gt_distance",
        "is_positive",
        "query_has_positive_in_B",
    }
    missing = sorted(required - set(dataframe.columns))
    if missing:
        raise ValueError(f"Candidate CSV missing columns: {missing}")
    valid = dataframe[
        (dataframe.query_has_positive_in_B.astype(int) == 1)
        & (dataframe["rank"].astype(int) <= top_k)
    ].copy()
    for column in [
        "query_frame",
        "candidate_frame",
        "rank",
        "is_positive",
        "query_has_positive_in_B",
    ]:
        valid[column] = valid[column].astype(int)
    counts = valid.groupby("query_frame").size()
    if not counts[counts != top_k].empty:
        raise ValueError(f"Not every valid query has {top_k} candidates")
    expected = set(range(1, top_k + 1))
    for query_frame, group in valid.groupby("query_frame"):
        if set(group["rank"]) != expected:
            raise ValueError(f"Bad rank set for query {query_frame}")
    if valid.duplicated(["query_frame", "candidate_frame"]).any():
        raise ValueError("Duplicate query/candidate pairs")
    return valid.sort_values(["query_frame", "rank"]).reset_index(drop=True)


def merge_existing_scores(candidates, single_scores, temporal_scores):
    keys = ["query_frame", "candidate_frame"]
    single = single_scores[keys + ["rgb_similarity"]].copy().rename(
        columns={"rgb_similarity": "single_rgb_similarity"}
    )
    temporal = temporal_scores[
        keys
        + [
            "temporal_5_rgb_similarity",
            "temporal_5_query_frame_count",
            "temporal_5_candidate_frame_count",
        ]
    ].copy().rename(
        columns={"temporal_5_rgb_similarity": "temporal_5_mean_similarity"}
    )
    for dataframe in [single, temporal]:
        dataframe["query_frame"] = dataframe.query_frame.astype(int)
        dataframe["candidate_frame"] = dataframe.candidate_frame.astype(int)
    merged = candidates.merge(single, on=keys, validate="one_to_one").merge(
        temporal, on=keys, validate="one_to_one"
    )
    if merged[
        ["single_rgb_similarity", "temporal_5_mean_similarity"]
    ].isna().any().any():
        raise ValueError("Missing existing RGB scores")
    return merged


def load_cache(path):
    payload = torch.load(path, map_location="cpu", weights_only=True)
    entries = payload.get("entries", {})
    if not entries:
        raise ValueError(f"Empty frame cache: {path}")
    return (
        {int(frame): entry["embedding"].float() for frame, entry in entries.items()},
        payload.get("identity"),
    )


def temporal_frames(target, embeddings):
    frames = [
        target + offset
        for offset in WINDOW_OFFSETS
        if target + offset >= 0 and target + offset in embeddings
    ]
    if not frames:
        raise ValueError(f"No temporal frames for target {target}")
    return frames


def cross_scores(query_frames, candidate_frames, embeddings):
    query = torch.stack([embeddings[frame] for frame in query_frames])
    candidate = torch.stack([embeddings[frame] for frame in candidate_frames])
    matrix = query @ candidate.T
    flat = matrix.reshape(-1)
    top_n = min(3, flat.numel())
    top_values = torch.topk(flat, k=top_n).values
    max_index = int(torch.argmax(flat))
    max_query_index = max_index // matrix.shape[1]
    max_candidate_index = max_index % matrix.shape[1]
    query_to_candidate = float(matrix.max(dim=1).values.mean())
    candidate_to_query = float(matrix.max(dim=0).values.mean())
    symmetric = 0.5 * (query_to_candidate + candidate_to_query)
    return {
        "temporal_cross_max": float(flat.max()),
        "temporal_cross_top3": float(top_values.mean()),
        "temporal_cross_symmetric": symmetric,
        "best_query_temporal_frame": query_frames[max_query_index],
        "best_candidate_temporal_frame": candidate_frames[max_candidate_index],
        "max_pair_similarity": float(flat.max()),
        "query_to_candidate_score": query_to_candidate,
        "candidate_to_query_score": candidate_to_query,
        "symmetric_score": symmetric,
        "query_temporal_frame_count": len(query_frames),
        "candidate_temporal_frame_count": len(candidate_frames),
        "matrix_pair_count": int(matrix.numel()),
    }


def add_cross_scores(candidates, embeddings):
    target_frames = sorted(
        set(candidates.query_frame.astype(int))
        | set(candidates.candidate_frame.astype(int))
    )
    window_cache = {
        frame: temporal_frames(frame, embeddings) for frame in target_frames
    }
    rows = []
    for index, row in enumerate(candidates.itertuples(index=False), start=1):
        record = row._asdict()
        record.update(
            cross_scores(
                window_cache[int(row.query_frame)],
                window_cache[int(row.candidate_frame)],
                embeddings,
            )
        )
        rows.append(record)
        if index % 100 == 0 or index == len(candidates):
            print(f"Computed cross-frame matrices: {index}/{len(candidates)}")
    return pd.DataFrame(rows), window_cache


def evaluate(scored, method, score_column):
    query_rows = []
    hits = []
    reciprocal_ranks = []
    corrections = 0
    regressions = 0
    for query_frame, group in scored.groupby("query_frame", sort=True):
        baseline = group.sort_values("rank", kind="stable")
        reranked = group.sort_values(
            [score_column, "rank"], ascending=[False, True], kind="stable"
        ).reset_index(drop=True)
        baseline_hit = int(baseline.iloc[0].is_positive)
        method_hit = int(reranked.iloc[0].is_positive)
        positive_indices = reranked.index[
            reranked.is_positive.astype(int) == 1
        ].tolist()
        first_positive_rank = positive_indices[0] + 1 if positive_indices else None
        reciprocal_rank = 1 / first_positive_rank if first_positive_rank else 0.0
        correction = int(baseline_hit == 0 and method_hit == 1)
        regression = int(baseline_hit == 1 and method_hit == 0)
        hits.append(method_hit)
        reciprocal_ranks.append(reciprocal_rank)
        corrections += correction
        regressions += regression
        query_rows.append(
            {
                "method": method,
                "query_frame": int(query_frame),
                "baseline_top1_frame": int(baseline.iloc[0].candidate_frame),
                "baseline_top1_positive": baseline_hit,
                "method_top1_frame": int(reranked.iloc[0].candidate_frame),
                "method_top1_positive": method_hit,
                "first_positive_rank": first_positive_rank,
                "reciprocal_rank": reciprocal_rank,
                "correction": correction,
                "regression": regression,
            }
        )
    queries = len(hits)
    return {
        "method": method,
        "queries": queries,
        "r_at_1": sum(hits) / queries,
        "mrr": sum(reciprocal_ranks) / queries,
        "corrections": corrections,
        "regressions": regressions,
        "net_gain": corrections - regressions,
        "top1_hits": sum(hits),
    }, query_rows


def result_paths(output_dir):
    return [
        output_dir / "metrics.csv",
        output_dir / "metrics.txt",
        output_dir / "candidate_scores.csv",
        output_dir / "query_results.csv",
        output_dir / "run_config.json",
    ]


def main():
    args = parse_args()
    if args.top_k < 1:
        raise ValueError("top-k must be positive")
    if any(alpha < 0 or alpha > 1 for alpha in args.alphas):
        raise ValueError("alpha must be in [0, 1]")
    for path, description in [
        (args.candidate_csv, "candidate CSV"),
        (args.single_scores_csv, "single RGB scores"),
        (args.temporal_scores_csv, "temporal mean scores"),
        (args.frame_cache, "frame embedding cache"),
    ]:
        require_file(path, description)
    existing = [path for path in result_paths(args.output_dir) if path.exists()]
    if existing and not args.overwrite_results:
        raise FileExistsError("Refusing to overwrite: " + ", ".join(map(str, existing)))

    start = time.time()
    candidates = validate_candidates(pd.read_csv(args.candidate_csv), args.top_k)
    candidates = merge_existing_scores(
        candidates,
        pd.read_csv(args.single_scores_csv),
        pd.read_csv(args.temporal_scores_csv),
    )
    embeddings, cache_identity = load_cache(args.frame_cache)
    scored, windows = add_cross_scores(candidates, embeddings)

    methods = [
        ("scan_context_baseline", "scan_context_score"),
        ("single_frame_rgb_only", "single_rgb_similarity"),
        ("temporal_5_mean_pooling", "temporal_5_mean_similarity"),
        ("temporal_cross_max", "temporal_cross_max"),
        ("temporal_cross_top3", "temporal_cross_top3"),
        ("temporal_cross_symmetric", "temporal_cross_symmetric"),
    ]
    for representation in ["max", "top3", "symmetric"]:
        source_column = f"temporal_cross_{representation}"
        for alpha in args.alphas:
            column = f"cross_{representation}_fusion_alpha_{alpha:.1f}"
            scored[column] = (
                alpha * scored.scan_context_score
                + (1 - alpha) * scored[source_column]
            )
            methods.append((f"sc_cross_{representation}_fusion_alpha_{alpha:.1f}", column))

    metric_rows = []
    query_rows = []
    for method, column in methods:
        metric, rows = evaluate(scored, method, column)
        metric_rows.append(metric)
        query_rows.extend(rows)
    metrics = pd.DataFrame(metric_rows)
    query_results = pd.DataFrame(query_rows)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    metrics.to_csv(args.output_dir / "metrics.csv", index=False)
    scored.to_csv(args.output_dir / "candidate_scores.csv", index=False)
    query_results.to_csv(args.output_dir / "query_results.csv", index=False)

    primary = [
        "scan_context_baseline",
        "single_frame_rgb_only",
        "temporal_5_mean_pooling",
        "temporal_cross_max",
        "temporal_cross_top3",
        "temporal_cross_symmetric",
    ]
    primary_table = metrics.set_index("method").loc[primary].reset_index()
    report = "\n".join(
        [
            "Full Temporal Cross-Frame RGB Reranking Results",
            f"Valid queries: {candidates.query_frame.nunique()}",
            f"Candidate pairs: {len(candidates)}",
            "",
            "Primary comparison",
            primary_table.to_string(
                index=False, float_format=lambda value: f"{value:.6f}"
            ),
            "",
            "All methods including SC fusion",
            metrics.to_string(
                index=False, float_format=lambda value: f"{value:.6f}"
            ),
            "",
            "No mean pooling is used for cross-frame scores.",
            "Cross symmetric is the primary cross-frame representation.",
        ]
    )
    (args.output_dir / "metrics.txt").write_text(report + "\n")

    window_counts = pd.Series([len(value) for value in windows.values()]).value_counts().sort_index()
    config = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "candidate_csv": str(args.candidate_csv),
        "single_scores_csv": str(args.single_scores_csv),
        "temporal_scores_csv": str(args.temporal_scores_csv),
        "frame_cache": str(args.frame_cache),
        "cache_identity": cache_identity,
        "window_offsets": WINDOW_OFFSETS,
        "window_count_distribution": {
            str(int(count)): int(number) for count, number in window_counts.items()
        },
        "valid_queries": int(candidates.query_frame.nunique()),
        "candidate_pairs": len(candidates),
        "cached_frame_embeddings": len(embeddings),
        "alphas": args.alphas,
        "fusion_formula": "alpha * scan_context_score + (1-alpha) * cross_frame_score",
        "cross_max": "maximum over full frame similarity matrix",
        "cross_top3": "mean of highest three matrix values, or all if fewer than three",
        "cross_symmetric": "0.5 * (mean row maxima + mean column maxima)",
        "rgb_rotation": "none",
        "elapsed_seconds": time.time() - start,
    }
    with open(args.output_dir / "run_config.json", "w") as file:
        json.dump(config, file, indent=2)

    print("\n" + report)
    print(f"Saved results to: {args.output_dir}")


if __name__ == "__main__":
    main()
