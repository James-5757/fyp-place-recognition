import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import open_clip
import pandas as pd
import torch
import torch.nn.functional as F
from PIL import Image


MODEL_NAME = "ViT-B-32-quickgelu"
DEFAULT_ALPHAS = [0.9, 0.8, 0.7, 0.6, 0.5]
YAW_BIN_EDGES = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.000001]
YAW_BIN_LABELS = [
    "0-30",
    "30-60",
    "60-90",
    "90-120",
    "120-150",
    "150-180",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Full single-frame KITTI RGB OpenCLIP reranking over a fixed "
            "Scan Context Top-K candidate pool."
        )
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
        "--rgb-dir",
        type=Path,
        default=Path("data/kitti/dataset/sequences/00/image_2"),
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("models/openclip/vit_b32_laion400m_e32.pt"),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("outputs/full_rgb_reranking")
    )
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--alphas", type=float, nargs="+", default=DEFAULT_ALPHAS)
    parser.add_argument("--rebuild-cache", action="store_true")
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
        "best_shift",
        "gt_distance",
        "is_positive",
        "query_has_positive_in_B",
    }
    missing = sorted(required - set(dataframe.columns))
    if missing:
        raise ValueError(f"Candidate CSV is missing columns: {missing}")

    valid = dataframe[
        dataframe["query_has_positive_in_B"].astype(int) == 1
    ].copy()
    valid = valid[valid["rank"].astype(int) <= top_k].copy()
    integer_columns = [
        "query_frame",
        "candidate_frame",
        "rank",
        "best_shift",
        "is_positive",
        "query_has_positive_in_B",
    ]
    for column in integer_columns:
        valid[column] = valid[column].astype(int)

    counts = valid.groupby("query_frame").size()
    bad_counts = counts[counts != top_k]
    if not bad_counts.empty:
        raise ValueError(
            f"Expected exactly {top_k} rows per valid query; "
            f"bad counts: {bad_counts.head().to_dict()}"
        )
    expected_ranks = set(range(1, top_k + 1))
    for query_frame, group in valid.groupby("query_frame", sort=False):
        ranks = set(group["rank"].tolist())
        if ranks != expected_ranks:
            raise ValueError(
                f"Query {query_frame} has ranks {sorted(ranks)}, "
                f"expected {sorted(expected_ranks)}"
            )
    if valid.duplicated(["query_frame", "candidate_frame"]).any():
        raise ValueError("Duplicate query/candidate pairs in valid Top-K pool")
    return valid.sort_values(["query_frame", "rank"]).reset_index(drop=True)


def add_yaw_metadata(candidates, yaw_data):
    required = {"query_frame", "candidate_frame", "gt_relative_yaw_deg"}
    missing = sorted(required - set(yaw_data.columns))
    if missing:
        raise ValueError(f"Yaw CSV is missing columns: {missing}")
    yaw_subset = yaw_data[
        ["query_frame", "candidate_frame", "gt_relative_yaw_deg"]
    ].copy()
    yaw_subset["query_frame"] = yaw_subset["query_frame"].astype(int)
    yaw_subset["candidate_frame"] = yaw_subset["candidate_frame"].astype(int)
    if yaw_subset.duplicated(["query_frame", "candidate_frame"]).any():
        raise ValueError("Duplicate query/candidate pairs in yaw CSV")
    merged = candidates.merge(
        yaw_subset,
        on=["query_frame", "candidate_frame"],
        how="left",
        validate="one_to_one",
    )
    if merged["gt_relative_yaw_deg"].isna().any():
        missing_rows = merged[merged["gt_relative_yaw_deg"].isna()][
            ["query_frame", "candidate_frame"]
        ]
        raise ValueError(
            "Missing GT relative yaw for candidate pairs: "
            f"{missing_rows.head().to_dict('records')}"
        )
    merged["absolute_wrapped_yaw_deg"] = (
        (merged["gt_relative_yaw_deg"].astype(float) + 180.0) % 360.0
        - 180.0
    ).abs()
    return merged


def required_frames(candidates):
    frames = set(candidates["query_frame"].astype(int).tolist())
    frames.update(candidates["candidate_frame"].astype(int).tolist())
    return sorted(frames)


def validate_rgb_coverage(frames, rgb_dir):
    if not rgb_dir.is_dir():
        raise FileNotFoundError(f"Missing RGB directory: {rgb_dir}")
    missing = [
        frame for frame in frames if not (rgb_dir / f"{frame:06d}.png").is_file()
    ]
    if missing:
        raise FileNotFoundError(
            f"Missing {len(missing)} required RGB frames: {missing[:20]}"
        )
    print(f"RGB coverage: {len(frames)}/{len(frames)} required frames present")


def file_signature(path):
    stat = path.stat()
    return f"{stat.st_size}:{stat.st_mtime_ns}"


def cache_identity(checkpoint):
    return {
        "version": 1,
        "model_name": MODEL_NAME,
        "checkpoint": str(checkpoint.resolve()),
        "checkpoint_signature": file_signature(checkpoint),
        "input_modality": "original_kitti_image_2_rgb_no_rotation",
    }


class EmbeddingCache:
    def __init__(self, path, identity, rebuild=False):
        self.path = path
        self.identity = identity
        self.entries = {}
        if path.is_file() and not rebuild:
            payload = torch.load(path, map_location="cpu", weights_only=True)
            if payload.get("identity") == identity:
                self.entries = payload.get("entries", {})
                print(f"Loaded {len(self.entries)} cached RGB embeddings from {path}")
            else:
                print(f"Ignoring incompatible cache: {path}")

    def get(self, frame, signature):
        entry = self.entries.get(str(frame))
        if entry is None or entry.get("signature") != signature:
            return None
        return entry["embedding"].float()

    def put(self, frame, signature, embedding):
        self.entries[str(frame)] = {
            "signature": signature,
            "embedding": embedding.detach().cpu().float(),
        }

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        torch.save(
            {"identity": self.identity, "entries": self.entries}, temporary
        )
        os.replace(temporary, self.path)


def chunks(items, size):
    for start in range(0, len(items), size):
        yield items[start : start + size]


def encode_batch(model, tensors, device):
    batch = torch.stack(tensors).to(device)
    with torch.inference_mode():
        embeddings = model.encode_image(batch)
        embeddings = F.normalize(embeddings, p=2, dim=-1)
    return embeddings.detach().cpu().float()


def build_rgb_embeddings(
    frames, rgb_dir, model, preprocess, device, cache, batch_size
):
    embeddings = {}
    pending = []
    for frame in frames:
        image_path = rgb_dir / f"{frame:06d}.png"
        signature = file_signature(image_path)
        cached = cache.get(frame, signature)
        if cached is None:
            pending.append((frame, image_path, signature))
        else:
            embeddings[frame] = cached

    print(f"RGB embeddings: {len(embeddings)} cached, {len(pending)} to encode")
    for batch_index, batch_items in enumerate(chunks(pending, batch_size), start=1):
        tensors = []
        for _, image_path, _ in batch_items:
            with Image.open(image_path) as image:
                tensors.append(preprocess(image.convert("RGB")))
        batch_embeddings = encode_batch(model, tensors, device)
        for (frame, _, signature), embedding in zip(
            batch_items, batch_embeddings
        ):
            cache.put(frame, signature, embedding)
            embeddings[frame] = embedding
        cache.save()
        completed = min(batch_index * batch_size, len(pending))
        print(f"Encoded RGB embeddings: {completed}/{len(pending)}")
    return embeddings


def add_rgb_scores(candidates, embeddings):
    similarities = []
    for row in candidates.itertuples(index=False):
        query_embedding = embeddings[int(row.query_frame)]
        candidate_embedding = embeddings[int(row.candidate_frame)]
        similarities.append(float(torch.dot(query_embedding, candidate_embedding)))
    scored = candidates.copy()
    scored["rgb_similarity"] = similarities
    return scored


def evaluate_method(scored, method, score_column):
    query_rows = []
    reciprocal_ranks = []
    top1_hits = []
    corrections = 0
    regressions = 0

    for query_frame, group in scored.groupby("query_frame", sort=True):
        baseline = group.sort_values("rank", kind="stable")
        reranked = group.sort_values(
            [score_column, "rank"], ascending=[False, True], kind="stable"
        ).reset_index(drop=True)
        baseline_hit = int(baseline.iloc[0]["is_positive"])
        method_hit = int(reranked.iloc[0]["is_positive"])
        positive_indices = reranked.index[
            reranked["is_positive"].astype(int) == 1
        ].tolist()
        first_positive_rank = positive_indices[0] + 1 if positive_indices else None
        reciprocal_rank = 1.0 / first_positive_rank if first_positive_rank else 0.0

        corrections += int(baseline_hit == 0 and method_hit == 1)
        regressions += int(baseline_hit == 1 and method_hit == 0)
        reciprocal_ranks.append(reciprocal_rank)
        top1_hits.append(method_hit)
        query_rows.append(
            {
                "method": method,
                "query_frame": int(query_frame),
                "baseline_top1_frame": int(baseline.iloc[0]["candidate_frame"]),
                "baseline_top1_positive": baseline_hit,
                "method_top1_frame": int(reranked.iloc[0]["candidate_frame"]),
                "method_top1_positive": method_hit,
                "first_positive_rank": first_positive_rank,
                "reciprocal_rank": reciprocal_rank,
                "correction": int(baseline_hit == 0 and method_hit == 1),
                "regression": int(baseline_hit == 1 and method_hit == 0),
            }
        )

    num_queries = len(top1_hits)
    metrics = {
        "method": method,
        "queries": num_queries,
        "r_at_1": sum(top1_hits) / num_queries,
        "mrr": sum(reciprocal_ranks) / num_queries,
        "corrections": corrections,
        "regressions": regressions,
        "net_gain": corrections - regressions,
        "top1_hits": sum(top1_hits),
    }
    return metrics, query_rows


def add_rgb_ranks_and_negative_margins(scored):
    output = scored.copy()
    output["rgb_rank_within_top5"] = pd.NA
    output["strongest_negative_rgb_similarity"] = float("nan")
    output["rgb_margin_vs_strongest_negative"] = float("nan")
    output["beats_strongest_negative"] = pd.NA

    for _, group in output.groupby("query_frame", sort=False):
        ranked = group.sort_values(
            ["rgb_similarity", "rank"],
            ascending=[False, True],
            kind="stable",
        )
        rank_map = {
            int(index): rank
            for rank, index in enumerate(ranked.index.tolist(), start=1)
        }
        negative = group[group["is_positive"].astype(int) == 0]
        strongest_negative = (
            float(negative["rgb_similarity"].max()) if len(negative) else None
        )
        for index in group.index:
            output.at[index, "rgb_rank_within_top5"] = rank_map[int(index)]
            if strongest_negative is not None:
                margin = float(output.at[index, "rgb_similarity"]) - strongest_negative
                output.at[index, "strongest_negative_rgb_similarity"] = (
                    strongest_negative
                )
                output.at[index, "rgb_margin_vs_strongest_negative"] = margin
                output.at[index, "beats_strongest_negative"] = bool(margin > 0.0)

    output["rgb_rank_within_top5"] = output[
        "rgb_rank_within_top5"
    ].astype(int)
    return output


def viewpoint_analysis(scored):
    ranked = add_rgb_ranks_and_negative_margins(scored)
    positives = ranked[ranked["is_positive"].astype(int) == 1].copy()
    positives["yaw_bin_deg"] = pd.cut(
        positives["absolute_wrapped_yaw_deg"],
        bins=YAW_BIN_EDGES,
        labels=YAW_BIN_LABELS,
        right=False,
        include_lowest=True,
    )
    if positives["yaw_bin_deg"].isna().any():
        bad = positives[positives["yaw_bin_deg"].isna()][
            ["query_frame", "candidate_frame", "absolute_wrapped_yaw_deg"]
        ]
        raise ValueError(f"Positive pairs outside yaw bins: {bad.to_dict('records')}")

    rows = []
    for label in YAW_BIN_LABELS:
        group = positives[positives["yaw_bin_deg"] == label]
        comparable = group[
            group["rgb_margin_vs_strongest_negative"].notna()
        ]
        rows.append(
            {
                "yaw_bin_deg": label,
                "positive_pairs": len(group),
                "unique_queries": int(group["query_frame"].nunique()),
                "pairs_with_negative_comparator": len(comparable),
                "mean_abs_yaw_deg": (
                    float(group["absolute_wrapped_yaw_deg"].mean())
                    if len(group)
                    else float("nan")
                ),
                "mean_rgb_similarity": (
                    float(group["rgb_similarity"].mean())
                    if len(group)
                    else float("nan")
                ),
                "median_rgb_similarity": (
                    float(group["rgb_similarity"].median())
                    if len(group)
                    else float("nan")
                ),
                "mean_rgb_rank_within_top5": (
                    float(group["rgb_rank_within_top5"].mean())
                    if len(group)
                    else float("nan")
                ),
                "median_rgb_rank_within_top5": (
                    float(group["rgb_rank_within_top5"].median())
                    if len(group)
                    else float("nan")
                ),
                "positive_pair_is_rgb_top1_rate": (
                    float((group["rgb_rank_within_top5"] == 1).mean())
                    if len(group)
                    else float("nan")
                ),
                "mean_margin_vs_strongest_negative": (
                    float(
                        comparable[
                            "rgb_margin_vs_strongest_negative"
                        ].mean()
                    )
                    if len(comparable)
                    else float("nan")
                ),
                "median_margin_vs_strongest_negative": (
                    float(
                        comparable[
                            "rgb_margin_vs_strongest_negative"
                        ].median()
                    )
                    if len(comparable)
                    else float("nan")
                ),
                "beats_strongest_negative_rate": (
                    float(comparable["beats_strongest_negative"].mean())
                    if len(comparable)
                    else float("nan")
                ),
            }
        )
    return positives, pd.DataFrame(rows)


def result_paths(output_dir):
    return [
        output_dir / "candidate_scores.csv",
        output_dir / "query_results.csv",
        output_dir / "metrics.csv",
        output_dir / "metrics.txt",
        output_dir / "viewpoint_positive_pairs.csv",
        output_dir / "viewpoint_bin_summary.csv",
        output_dir / "run_config.json",
    ]


def main():
    args = parse_args()
    if args.top_k < 1:
        raise ValueError("--top-k must be at least 1")
    if args.batch_size < 1:
        raise ValueError("--batch-size must be at least 1")
    if any(alpha < 0.0 or alpha > 1.0 for alpha in args.alphas):
        raise ValueError("All alpha values must be in [0, 1]")
    if args.threads > 0:
        torch.set_num_threads(args.threads)

    require_file(args.candidate_csv, "formal candidate CSV")
    require_file(args.yaw_csv, "yaw validation CSV")
    require_file(args.checkpoint, "local OpenCLIP checkpoint")
    existing = [path for path in result_paths(args.output_dir) if path.exists()]
    if existing and not args.overwrite_results:
        raise FileExistsError(
            "Refusing to overwrite existing full RGB results: "
            + ", ".join(str(path) for path in existing)
        )

    start_time = time.time()
    candidate_data = pd.read_csv(args.candidate_csv)
    yaw_data = pd.read_csv(args.yaw_csv)
    candidates = validate_candidates(candidate_data, args.top_k)
    candidates = add_yaw_metadata(candidates, yaw_data)
    frames = required_frames(candidates)
    validate_rgb_coverage(frames, args.rgb_dir)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Valid queries: {candidates['query_frame'].nunique()}")
    print(f"Top-{args.top_k} candidate pairs: {len(candidates)}")
    print(f"Positive candidate pairs: {int(candidates['is_positive'].sum())}")
    print(f"Unique RGB frames: {len(frames)}")
    print(f"Device: {device}")
    print(f"Model: {MODEL_NAME}")
    print(f"Local checkpoint: {args.checkpoint}")
    print("RGB rotation: none")

    model, _, preprocess = open_clip.create_model_and_transforms(
        MODEL_NAME, pretrained=str(args.checkpoint)
    )
    model = model.to(device).eval()
    cache = EmbeddingCache(
        args.output_dir / "cache/rgb_embeddings.pt",
        cache_identity(args.checkpoint),
        args.rebuild_cache,
    )
    embeddings = build_rgb_embeddings(
        frames,
        args.rgb_dir,
        model,
        preprocess,
        device,
        cache,
        args.batch_size,
    )
    scored = add_rgb_scores(candidates, embeddings)

    metrics_rows = []
    query_rows = []
    baseline_metrics, baseline_queries = evaluate_method(
        scored, "scan_context_baseline", "scan_context_score"
    )
    metrics_rows.append(baseline_metrics)
    query_rows.extend(baseline_queries)
    rgb_metrics, rgb_queries = evaluate_method(
        scored, "single_frame_rgb_only", "rgb_similarity"
    )
    metrics_rows.append(rgb_metrics)
    query_rows.extend(rgb_queries)

    for alpha in args.alphas:
        column = f"fusion_alpha_{alpha:.1f}"
        scored[column] = (
            alpha * scored["scan_context_score"]
            + (1.0 - alpha) * scored["rgb_similarity"]
        )
        method = f"sc_rgb_fusion_alpha_{alpha:.1f}"
        metrics, queries = evaluate_method(scored, method, column)
        metrics_rows.append(metrics)
        query_rows.extend(queries)

    positives, viewpoint_summary = viewpoint_analysis(scored)
    metrics = pd.DataFrame(metrics_rows)
    query_results = pd.DataFrame(query_rows)
    pool_r_at_k = float(
        scored.groupby("query_frame")["is_positive"].max().mean()
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    scored.to_csv(args.output_dir / "candidate_scores.csv", index=False)
    query_results.to_csv(args.output_dir / "query_results.csv", index=False)
    metrics.to_csv(args.output_dir / "metrics.csv", index=False)
    positives.to_csv(
        args.output_dir / "viewpoint_positive_pairs.csv", index=False
    )
    viewpoint_summary.to_csv(
        args.output_dir / "viewpoint_bin_summary.csv", index=False
    )

    run_config = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "candidate_csv": str(args.candidate_csv),
        "yaw_csv": str(args.yaw_csv),
        "rgb_dir": str(args.rgb_dir),
        "checkpoint": str(args.checkpoint),
        "model_name": MODEL_NAME,
        "device": device,
        "rgb_rotation": "none",
        "top_k": args.top_k,
        "alphas": args.alphas,
        "fusion_formula": (
            "alpha * scan_context_score + (1-alpha) * rgb_similarity"
        ),
        "valid_queries": int(candidates["query_frame"].nunique()),
        "candidate_pairs": len(candidates),
        "positive_pairs": int(candidates["is_positive"].sum()),
        "unique_frames": len(frames),
        "candidate_pool_r_at_k": pool_r_at_k,
        "yaw_bin_edges_deg": YAW_BIN_EDGES,
        "elapsed_seconds": time.time() - start_time,
    }
    with open(args.output_dir / "run_config.json", "w") as file:
        json.dump(run_config, file, indent=2)

    report_lines = [
        "Full Single-Frame RGB Reranking Results",
        f"Valid queries: {run_config['valid_queries']}",
        f"SC Top-{args.top_k} candidate-pool recall: {pool_r_at_k:.6f}",
        "",
        metrics.to_string(index=False, float_format=lambda value: f"{value:.6f}"),
        "",
        "Viewpoint-conditioned positive-pair analysis",
        viewpoint_summary.to_string(
            index=False, float_format=lambda value: f"{value:.6f}"
        ),
        "",
        f"Elapsed seconds: {run_config['elapsed_seconds']:.1f}",
    ]
    report = "\n".join(report_lines)
    (args.output_dir / "metrics.txt").write_text(report + "\n")
    print("\n" + report)
    print(f"\nSaved results to: {args.output_dir}")


if __name__ == "__main__":
    main()
