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
WINDOWS = {
    "single": [0],
    "temporal_3": [-10, -5, 0],
    "temporal_5": [-20, -15, -10, -5, 0],
}
DEFAULT_ALPHAS = [0.9, 0.8, 0.7, 0.6, 0.5]
YAW_EDGES = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.000001]
YAW_LABELS = ["0-30", "30-60", "60-90", "90-120", "120-150", "150-180"]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Full causal temporal RGB OpenCLIP Top-K reranking"
    )
    parser.add_argument(
        "--candidate-csv",
        type=Path,
        default=Path("outputs/formal_split2500_gap100_step5_thr5_candidates.csv"),
    )
    parser.add_argument(
        "--yaw-csv",
        type=Path,
        default=Path("outputs/yaw_validation/all_candidate_yaw_validation.csv"),
    )
    parser.add_argument(
        "--single-candidate-scores",
        type=Path,
        default=Path("outputs/full_rgb_reranking/candidate_scores.csv"),
    )
    parser.add_argument(
        "--single-cache",
        type=Path,
        default=Path("outputs/full_rgb_reranking/cache/rgb_embeddings.pt"),
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
        "--output-dir",
        type=Path,
        default=Path("outputs/temporal_rgb_reranking"),
    )
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--alphas", type=float, nargs="+", default=DEFAULT_ALPHAS)
    parser.add_argument("--overwrite-results", action="store_true")
    return parser.parse_args()


def require_file(path, description):
    if not path.is_file():
        raise FileNotFoundError(f"Missing {description}: {path}")


def validate_candidates(dataframe, top_k):
    required = {
        "query_frame", "candidate_frame", "rank", "scan_context_score",
        "gt_distance", "is_positive", "query_has_positive_in_B",
    }
    missing = sorted(required - set(dataframe.columns))
    if missing:
        raise ValueError(f"Candidate CSV missing columns: {missing}")
    valid = dataframe[
        (dataframe.query_has_positive_in_B.astype(int) == 1)
        & (dataframe["rank"].astype(int) <= top_k)
    ].copy()
    for column in [
        "query_frame", "candidate_frame", "rank", "is_positive",
        "query_has_positive_in_B",
    ]:
        valid[column] = valid[column].astype(int)
    counts = valid.groupby("query_frame").size()
    if not counts[counts != top_k].empty:
        raise ValueError(f"Not all valid queries have {top_k} candidates")
    expected = set(range(1, top_k + 1))
    for query_frame, group in valid.groupby("query_frame"):
        if set(group["rank"]) != expected:
            raise ValueError(f"Bad rank set for query {query_frame}")
    if valid.duplicated(["query_frame", "candidate_frame"]).any():
        raise ValueError("Duplicate query/candidate pairs")
    return valid.sort_values(["query_frame", "rank"]).reset_index(drop=True)


def add_yaw(candidates, yaw_data):
    subset = yaw_data[
        ["query_frame", "candidate_frame", "gt_relative_yaw_deg"]
    ].copy()
    subset["query_frame"] = subset.query_frame.astype(int)
    subset["candidate_frame"] = subset.candidate_frame.astype(int)
    if subset.duplicated(["query_frame", "candidate_frame"]).any():
        raise ValueError("Duplicate pairs in yaw data")
    merged = candidates.merge(
        subset,
        on=["query_frame", "candidate_frame"],
        how="left",
        validate="one_to_one",
    )
    if merged.gt_relative_yaw_deg.isna().any():
        raise ValueError("Missing GT relative yaw values")
    merged["absolute_wrapped_yaw_deg"] = (
        (merged.gt_relative_yaw_deg.astype(float) + 180.0) % 360.0 - 180.0
    ).abs()
    return merged


def add_existing_single_scores(candidates, single_scores):
    required = {"query_frame", "candidate_frame", "rgb_similarity"}
    missing = sorted(required - set(single_scores.columns))
    if missing:
        raise ValueError(f"Existing single RGB scores missing columns: {missing}")
    subset = single_scores[
        ["query_frame", "candidate_frame", "rgb_similarity"]
    ].copy().rename(columns={"rgb_similarity": "single_rgb_similarity"})
    subset["query_frame"] = subset.query_frame.astype(int)
    subset["candidate_frame"] = subset.candidate_frame.astype(int)
    merged = candidates.merge(
        subset,
        on=["query_frame", "candidate_frame"],
        how="left",
        validate="one_to_one",
    )
    if merged.single_rgb_similarity.isna().any():
        raise ValueError("Missing existing single RGB scores for candidate pairs")
    return merged


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


class FrameCache:
    def __init__(self, path, identity):
        self.path = path
        self.identity = identity
        self.entries = {}
        if path.is_file():
            payload = torch.load(path, map_location="cpu", weights_only=True)
            if payload.get("identity") == identity:
                self.entries = payload.get("entries", {})
                print(f"Loaded {len(self.entries)} temporal frame-cache entries")
            else:
                raise ValueError(f"Incompatible temporal frame cache: {path}")

    def import_compatible(self, source):
        if self.entries:
            return
        require_file(source, "single-frame RGB embedding cache")
        payload = torch.load(source, map_location="cpu", weights_only=True)
        if payload.get("identity") != self.identity:
            raise ValueError(f"Incompatible single-frame cache: {source}")
        self.entries = payload.get("entries", {}).copy()
        print(f"Imported {len(self.entries)} entries from single-frame cache")
        self.save()

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
        torch.save({"identity": self.identity, "entries": self.entries}, temporary)
        os.replace(temporary, self.path)


def chunks(items, size):
    for start in range(0, len(items), size):
        yield items[start : start + size]


def required_target_frames(candidates):
    frames = set(candidates.query_frame.astype(int))
    frames.update(candidates.candidate_frame.astype(int))
    return sorted(frames)


def required_source_frames(target_frames, rgb_dir):
    needed = set()
    for frame in target_frames:
        for offsets in WINDOWS.values():
            for offset in offsets:
                source = frame + offset
                if source >= 0 and (rgb_dir / f"{source:06d}.png").is_file():
                    needed.add(source)
    return sorted(needed)


def build_frame_embeddings(
    source_frames, rgb_dir, model, preprocess, device, cache, batch_size
):
    embeddings = {}
    pending = []
    for frame in source_frames:
        path = rgb_dir / f"{frame:06d}.png"
        signature = file_signature(path)
        cached = cache.get(frame, signature)
        if cached is None:
            pending.append((frame, path, signature))
        else:
            embeddings[frame] = cached
    print(f"Frame embeddings: {len(embeddings)} cached, {len(pending)} to encode")
    for batch_index, batch_items in enumerate(chunks(pending, batch_size), start=1):
        tensors = []
        for _, path, _ in batch_items:
            with Image.open(path) as image:
                tensors.append(preprocess(image.convert("RGB")))
        batch = torch.stack(tensors).to(device)
        with torch.inference_mode():
            encoded = F.normalize(model.encode_image(batch), p=2, dim=-1)
        encoded = encoded.detach().cpu().float()
        for (frame, _, signature), embedding in zip(batch_items, encoded):
            cache.put(frame, signature, embedding)
            embeddings[frame] = embedding
        cache.save()
        complete = min(batch_index * batch_size, len(pending))
        print(f"Encoded missing RGB frames: {complete}/{len(pending)}")
    return embeddings


def temporal_descriptor(frame, offsets, frame_embeddings):
    used = [
        frame + offset
        for offset in offsets
        if frame + offset >= 0 and frame + offset in frame_embeddings
    ]
    if not used:
        raise ValueError(f"No temporal frames available for {frame}")
    pooled = torch.stack([frame_embeddings[item] for item in used]).mean(dim=0)
    descriptor = F.normalize(pooled.unsqueeze(0), p=2, dim=-1).squeeze(0)
    return descriptor, used


def build_descriptors(target_frames, frame_embeddings):
    descriptors = {}
    counts = {}
    used_frames = {}
    for representation, offsets in WINDOWS.items():
        for frame in target_frames:
            descriptor, used = temporal_descriptor(frame, offsets, frame_embeddings)
            descriptors[(representation, frame)] = descriptor
            counts[(representation, frame)] = len(used)
            used_frames[(representation, frame)] = used
    return descriptors, counts, used_frames


def add_temporal_scores(candidates, descriptors, counts):
    output = candidates.copy()
    recomputed_single = []
    temporal_3 = []
    temporal_5 = []
    for row in output.itertuples(index=False):
        query = int(row.query_frame)
        candidate = int(row.candidate_frame)
        for representation, target in [
            ("single", recomputed_single),
            ("temporal_3", temporal_3),
            ("temporal_5", temporal_5),
        ]:
            target.append(
                float(
                    torch.dot(
                        descriptors[(representation, query)],
                        descriptors[(representation, candidate)],
                    )
                )
            )
    output["single_rgb_recomputed_similarity"] = recomputed_single
    output["temporal_3_rgb_similarity"] = temporal_3
    output["temporal_5_rgb_similarity"] = temporal_5
    for representation in WINDOWS:
        output[f"{representation}_query_frame_count"] = [
            counts[(representation, int(frame))] for frame in output.query_frame
        ]
        output[f"{representation}_candidate_frame_count"] = [
            counts[(representation, int(frame))] for frame in output.candidate_frame
        ]
    max_error = float(
        (
            output.single_rgb_recomputed_similarity
            - output.single_rgb_similarity
        ).abs().max()
    )
    if max_error >= 5e-7:
        raise ValueError(
            f"Recomputed single RGB scores differ from existing results: {max_error}"
        )
    print(f"Single-score consistency max abs error: {max_error:.3e}")
    return output, max_error


def evaluate(scored, method, score_column):
    query_rows = []
    top1_hits = []
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
        corrections += correction
        regressions += regression
        top1_hits.append(method_hit)
        reciprocal_ranks.append(reciprocal_rank)
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
    queries = len(top1_hits)
    return {
        "method": method,
        "queries": queries,
        "r_at_1": sum(top1_hits) / queries,
        "mrr": sum(reciprocal_ranks) / queries,
        "corrections": corrections,
        "regressions": regressions,
        "net_gain": corrections - regressions,
        "top1_hits": sum(top1_hits),
    }, query_rows


def representation_positive_pairs(scored, representation, score_column):
    records = []
    for _, group in scored.groupby("query_frame", sort=True):
        ranked = group.sort_values(
            [score_column, "rank"], ascending=[False, True], kind="stable"
        )
        rank_map = {
            int(index): rank
            for rank, index in enumerate(ranked.index.tolist(), start=1)
        }
        negatives = group[group.is_positive.astype(int) == 0]
        strongest_negative = (
            float(negatives[score_column].max()) if len(negatives) else None
        )
        for index, row in group[group.is_positive.astype(int) == 1].iterrows():
            margin = (
                float(row[score_column]) - strongest_negative
                if strongest_negative is not None
                else float("nan")
            )
            records.append(
                {
                    "representation": representation,
                    "query_frame": int(row.query_frame),
                    "candidate_frame": int(row.candidate_frame),
                    "absolute_wrapped_yaw_deg": float(
                        row.absolute_wrapped_yaw_deg
                    ),
                    "rgb_similarity": float(row[score_column]),
                    "rgb_rank_within_top5": rank_map[int(index)],
                    "strongest_negative_rgb_similarity": (
                        strongest_negative
                        if strongest_negative is not None
                        else float("nan")
                    ),
                    "margin_vs_strongest_negative": margin,
                    "beats_strongest_negative": (
                        bool(margin > 0) if strongest_negative is not None else pd.NA
                    ),
                    "temporal_frame_count_query": int(
                        row[f"{representation}_query_frame_count"]
                    ),
                    "temporal_frame_count_candidate": int(
                        row[f"{representation}_candidate_frame_count"]
                    ),
                }
            )
    return pd.DataFrame(records)


def viewpoint_tables(scored):
    details = pd.concat(
        [
            representation_positive_pairs(
                scored, "single", "single_rgb_similarity"
            ),
            representation_positive_pairs(
                scored, "temporal_3", "temporal_3_rgb_similarity"
            ),
            representation_positive_pairs(
                scored, "temporal_5", "temporal_5_rgb_similarity"
            ),
        ],
        ignore_index=True,
    )
    details["yaw_bin_deg"] = pd.cut(
        details.absolute_wrapped_yaw_deg,
        bins=YAW_EDGES,
        labels=YAW_LABELS,
        right=False,
        include_lowest=True,
    )
    rows = []
    for representation in WINDOWS:
        for yaw_bin in YAW_LABELS:
            group = details[
                (details.representation == representation)
                & (details.yaw_bin_deg == yaw_bin)
            ]
            comparable = group[group.margin_vs_strongest_negative.notna()]
            rows.append(
                {
                    "representation": representation,
                    "yaw_bin_deg": yaw_bin,
                    "positive_pairs": len(group),
                    "unique_queries": int(group.query_frame.nunique()),
                    "pairs_with_negative_comparator": len(comparable),
                    "mean_rgb_similarity": (
                        float(group.rgb_similarity.mean()) if len(group) else float("nan")
                    ),
                    "median_rgb_similarity": (
                        float(group.rgb_similarity.median()) if len(group) else float("nan")
                    ),
                    "mean_rank_within_top5": (
                        float(group.rgb_rank_within_top5.mean()) if len(group) else float("nan")
                    ),
                    "rgb_top1_rate": (
                        float((group.rgb_rank_within_top5 == 1).mean())
                        if len(group) else float("nan")
                    ),
                    "mean_margin_vs_strongest_negative": (
                        float(comparable.margin_vs_strongest_negative.mean())
                        if len(comparable) else float("nan")
                    ),
                    "median_margin_vs_strongest_negative": (
                        float(comparable.margin_vs_strongest_negative.median())
                        if len(comparable) else float("nan")
                    ),
                    "beats_strongest_negative_rate": (
                        float(comparable.beats_strongest_negative.mean())
                        if len(comparable) else float("nan")
                    ),
                }
            )
    return details, pd.DataFrame(rows)


def best_method(metrics, prefix):
    candidates = metrics[metrics.method.str.startswith(prefix)].copy()
    return candidates.sort_values(
        ["r_at_1", "mrr", "net_gain", "method"],
        ascending=[False, False, False, True],
    ).iloc[0]


def recommendation(metrics):
    single = metrics[metrics.method == "single_frame_rgb_only"].iloc[0]
    temporals = metrics[
        metrics.method.isin(["temporal_3_rgb_only", "temporal_5_rgb_only"])
    ].sort_values(["r_at_1", "mrr"], ascending=[False, False])
    best = temporals.iloc[0]
    helpful = (
        best.r_at_1 > single.r_at_1
        and best.mrr > single.mrr
        and best.regressions < single.regressions
        and best.corrections >= single.corrections
    )
    return "TEMPORAL_CONTEXT_HELPFUL" if helpful else "TEMPORAL_CONTEXT_NOT_SUFFICIENT"


def output_paths(output_dir):
    return [
        output_dir / "metrics.csv",
        output_dir / "metrics.txt",
        output_dir / "candidate_scores.csv",
        output_dir / "query_results.csv",
        output_dir / "viewpoint_positive_pairs.csv",
        output_dir / "viewpoint_bin_summary.csv",
        output_dir / "run_config.json",
    ]


def main():
    args = parse_args()
    if args.top_k < 1 or args.batch_size < 1:
        raise ValueError("top-k and batch-size must be positive")
    if any(alpha < 0 or alpha > 1 for alpha in args.alphas):
        raise ValueError("alpha must be in [0, 1]")
    if args.threads > 0:
        torch.set_num_threads(args.threads)
    for path, description in [
        (args.candidate_csv, "candidate CSV"),
        (args.yaw_csv, "yaw CSV"),
        (args.single_candidate_scores, "existing single RGB scores"),
        (args.checkpoint, "OpenCLIP checkpoint"),
    ]:
        require_file(path, description)
    existing = [path for path in output_paths(args.output_dir) if path.exists()]
    if existing and not args.overwrite_results:
        raise FileExistsError("Refusing to overwrite: " + ", ".join(map(str, existing)))

    start = time.time()
    candidates = validate_candidates(pd.read_csv(args.candidate_csv), args.top_k)
    candidates = add_yaw(candidates, pd.read_csv(args.yaw_csv))
    candidates = add_existing_single_scores(
        candidates, pd.read_csv(args.single_candidate_scores)
    )
    target_frames = required_target_frames(candidates)
    source_frames = required_source_frames(target_frames, args.rgb_dir)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Valid queries: {candidates.query_frame.nunique()}")
    print(f"Candidate pairs: {len(candidates)}")
    print(f"Target frames: {len(target_frames)}")
    print(f"Required source RGB frames: {len(source_frames)}")
    print(f"Device: {device}")
    print(f"Model: {MODEL_NAME}")
    print("Windows: " + json.dumps(WINDOWS))
    print("RGB rotation: none")

    model, _, preprocess = open_clip.create_model_and_transforms(
        MODEL_NAME, pretrained=str(args.checkpoint)
    )
    model = model.to(device).eval()
    cache = FrameCache(
        args.output_dir / "cache/rgb_frame_embeddings.pt",
        cache_identity(args.checkpoint),
    )
    cache.import_compatible(args.single_cache)
    frame_embeddings = build_frame_embeddings(
        source_frames, args.rgb_dir, model, preprocess, device, cache, args.batch_size
    )
    descriptors, counts, used_frames = build_descriptors(
        target_frames, frame_embeddings
    )
    scored, single_error = add_temporal_scores(candidates, descriptors, counts)

    methods = [
        ("scan_context_baseline", "scan_context_score"),
        ("single_frame_rgb_only", "single_rgb_similarity"),
        ("temporal_3_rgb_only", "temporal_3_rgb_similarity"),
        ("temporal_5_rgb_only", "temporal_5_rgb_similarity"),
    ]
    for representation in ["temporal_3", "temporal_5"]:
        for alpha in args.alphas:
            column = f"{representation}_fusion_alpha_{alpha:.1f}"
            scored[column] = (
                alpha * scored.scan_context_score
                + (1 - alpha) * scored[f"{representation}_rgb_similarity"]
            )
            methods.append((f"sc_{representation}_fusion_alpha_{alpha:.1f}", column))

    metric_rows = []
    query_rows = []
    for method, column in methods:
        metric, queries = evaluate(scored, method, column)
        metric_rows.append(metric)
        query_rows.extend(queries)
    metrics = pd.DataFrame(metric_rows)
    query_results = pd.DataFrame(query_rows)
    viewpoint_details, viewpoint_summary = viewpoint_tables(scored)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    metrics.to_csv(args.output_dir / "metrics.csv", index=False)
    scored.to_csv(args.output_dir / "candidate_scores.csv", index=False)
    query_results.to_csv(args.output_dir / "query_results.csv", index=False)
    viewpoint_details.to_csv(
        args.output_dir / "viewpoint_positive_pairs.csv", index=False
    )
    viewpoint_summary.to_csv(
        args.output_dir / "viewpoint_bin_summary.csv", index=False
    )

    best_t3_fusion = best_method(metrics, "sc_temporal_3_fusion")
    best_t5_fusion = best_method(metrics, "sc_temporal_5_fusion")
    selected_names = [
        "scan_context_baseline", "single_frame_rgb_only",
        "temporal_3_rgb_only", "temporal_5_rgb_only",
        best_t3_fusion.method, best_t5_fusion.method,
    ]
    compact = metrics.set_index("method").loc[selected_names].reset_index()
    result_recommendation = recommendation(metrics)
    report = "\n".join(
        [
            "Full Temporal RGB Reranking Results",
            f"Valid queries: {candidates.query_frame.nunique()}",
            f"Candidate pairs: {len(candidates)}",
            "",
            "Compact comparison",
            compact.to_string(index=False, float_format=lambda value: f"{value:.6f}"),
            "",
            "All methods",
            metrics.to_string(index=False, float_format=lambda value: f"{value:.6f}"),
            "",
            "Viewpoint-conditioned analysis",
            viewpoint_summary.to_string(
                index=False, float_format=lambda value: f"{value:.6f}"
            ),
            "",
            f"recommendation: {result_recommendation}",
        ]
    )
    (args.output_dir / "metrics.txt").write_text(report + "\n")

    partial_counts = {
        representation: sum(
            1
            for frame in target_frames
            if counts[(representation, frame)] < len(offsets)
        )
        for representation, offsets in WINDOWS.items()
    }
    config = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "model_name": MODEL_NAME,
        "checkpoint": str(args.checkpoint),
        "rgb_dir": str(args.rgb_dir),
        "candidate_csv": str(args.candidate_csv),
        "yaw_csv": str(args.yaw_csv),
        "existing_single_scores": str(args.single_candidate_scores),
        "windows": WINDOWS,
        "alphas": args.alphas,
        "fusion_formula": "alpha * scan_context_score + (1-alpha) * temporal_rgb_similarity",
        "valid_queries": int(candidates.query_frame.nunique()),
        "candidate_pairs": len(candidates),
        "target_frames": len(target_frames),
        "source_frames": len(source_frames),
        "partial_window_target_counts": partial_counts,
        "single_score_consistency_max_abs_error": single_error,
        "recommendation_rule": "best temporal RGB-only must improve R@1 and MRR, reduce regressions, and preserve/increase corrections versus single RGB",
        "recommendation": result_recommendation,
        "elapsed_seconds": time.time() - start,
    }
    with open(args.output_dir / "run_config.json", "w") as file:
        json.dump(config, file, indent=2)

    single = metrics[metrics.method == "single_frame_rgb_only"].iloc[0]
    temporal_3 = metrics[metrics.method == "temporal_3_rgb_only"].iloc[0]
    temporal_5 = metrics[metrics.method == "temporal_5_rgb_only"].iloc[0]
    print("\n" + report)
    print(f"\nSingle RGB regressions: {int(single.regressions)}")
    print(f"Temporal-3 regressions: {int(temporal_3.regressions)}")
    print(f"Temporal-5 regressions: {int(temporal_5.regressions)}")
    print(f"Single RGB corrections: {int(single.corrections)}")
    print(f"Temporal-3 corrections: {int(temporal_3.corrections)}")
    print(f"Temporal-5 corrections: {int(temporal_5.corrections)}")
    print(f"recommendation: {result_recommendation}")
    print(f"Saved results to: {args.output_dir}")


if __name__ == "__main__":
    main()
