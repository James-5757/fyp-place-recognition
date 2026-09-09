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

from generate_bev import generate_single_frame


DEFAULT_ALPHAS = [0.9, 0.8, 0.7, 0.6, 0.5]
NUM_SECTORS = 60
DEG_PER_SHIFT = 360.0 / NUM_SECTORS


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Rerank Scan Context Top-K candidates with yaw-aligned BEV "
            "OpenCLIP embeddings."
        )
    )
    parser.add_argument(
        "--candidate-csv",
        type=Path,
        default=Path(
            "outputs/formal_split2500_gap100_step5_thr5_candidates.csv"
        ),
    )
    parser.add_argument("--bev-dir", type=Path, default=Path("data/kitti_bev/00"))
    parser.add_argument(
        "--velodyne-dir",
        type=Path,
        default=Path("data/kitti/dataset/sequences/00/velodyne"),
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("models/openclip/vit_b32_laion400m_e32.pt"),
    )
    parser.add_argument("--model-name", default="ViT-B-32-quickgelu")
    parser.add_argument(
        "--output-dir", type=Path, default=Path("outputs/full_bev_reranking")
    )
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--alphas", type=float, nargs="+", default=DEFAULT_ALPHAS)
    parser.add_argument(
        "--generate-missing-bev",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--rebuild-cache", action="store_true")
    return parser.parse_args()


def validate_candidate_csv(df, top_k):
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
    missing = sorted(required - set(df.columns))
    if missing:
        raise ValueError(f"Candidate CSV is missing columns: {missing}")

    valid = df[df["query_has_positive_in_B"].astype(int) == 1].copy()
    valid = valid[valid["rank"].astype(int) <= top_k].copy()
    valid["query_frame"] = valid["query_frame"].astype(int)
    valid["candidate_frame"] = valid["candidate_frame"].astype(int)
    valid["rank"] = valid["rank"].astype(int)
    valid["best_shift"] = valid["best_shift"].astype(int)
    valid["is_positive"] = valid["is_positive"].astype(int)

    counts = valid.groupby("query_frame").size()
    bad_counts = counts[counts != top_k]
    if not bad_counts.empty:
        raise ValueError(
            f"Expected {top_k} candidates per valid query; bad counts: "
            f"{bad_counts.head().to_dict()}"
        )

    expected_ranks = set(range(1, top_k + 1))
    for query_frame, group in valid.groupby("query_frame", sort=False):
        ranks = set(group["rank"].tolist())
        if ranks != expected_ranks:
            raise ValueError(f"Query {query_frame} has ranks {sorted(ranks)}")

    return valid.sort_values(["query_frame", "rank"]).reset_index(drop=True)


def required_frames(candidates):
    frames = set(candidates["query_frame"].tolist())
    frames.update(candidates["candidate_frame"].tolist())
    return sorted(frames)


def ensure_bev_files(frames, bev_dir, velodyne_dir, generate_missing):
    bev_dir.mkdir(parents=True, exist_ok=True)
    missing = [
        frame for frame in frames if not (bev_dir / f"{frame:06d}.png").is_file()
    ]
    if not missing:
        print(f"BEV coverage: {len(frames)}/{len(frames)} frames present")
        return []

    print(f"Missing BEV files: {len(missing)}")
    if not generate_missing:
        raise FileNotFoundError(
            "Missing required BEV files and --no-generate-missing-bev was used: "
            f"{missing[:20]}"
        )

    for index, frame in enumerate(missing, start=1):
        bin_path = velodyne_dir / f"{frame:06d}.bin"
        output_path = bev_dir / f"{frame:06d}.png"
        if not bin_path.is_file():
            raise FileNotFoundError(f"Missing LiDAR input: {bin_path}")
        generate_single_frame(bin_path, output_path)
        if index % 25 == 0 or index == len(missing):
            print(f"Generated BEV files: {index}/{len(missing)}")
    return missing


def file_signature(path):
    stat = path.stat()
    return f"{stat.st_size}:{stat.st_mtime_ns}"


def cache_identity(model_name, checkpoint):
    return {
        "version": 1,
        "model_name": model_name,
        "checkpoint": str(checkpoint.resolve()),
        "checkpoint_signature": file_signature(checkpoint),
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
                print(f"Loaded {len(self.entries)} cached embeddings from {path}")
            else:
                print(f"Ignoring incompatible cache: {path}")

    def get(self, key, signature):
        entry = self.entries.get(str(key))
        if entry is None or entry.get("signature") != signature:
            return None
        return entry["embedding"].float()

    def put(self, key, signature, embedding):
        self.entries[str(key)] = {
            "signature": signature,
            "embedding": embedding.detach().cpu().float(),
        }

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        torch.save(
            {"identity": self.identity, "entries": self.entries},
            temp_path,
        )
        os.replace(temp_path, self.path)


def encode_batch(model, tensors, device):
    batch = torch.stack(tensors).to(device)
    with torch.inference_mode():
        embeddings = model.encode_image(batch)
        embeddings = F.normalize(embeddings, p=2, dim=-1)
    return embeddings.cpu().float()


def chunks(items, size):
    for start in range(0, len(items), size):
        yield items[start : start + size]


def build_raw_embeddings(
    frames, bev_dir, model, preprocess, device, cache, batch_size
):
    embeddings = {}
    pending = []
    for frame in frames:
        path = bev_dir / f"{frame:06d}.png"
        signature = file_signature(path)
        cached = cache.get(frame, signature)
        if cached is None:
            pending.append((frame, path, signature))
        else:
            embeddings[frame] = cached

    print(f"Raw embeddings: {len(embeddings)} cached, {len(pending)} to encode")
    for batch_index, batch_items in enumerate(chunks(pending, batch_size), start=1):
        tensors = []
        for _, path, _ in batch_items:
            with Image.open(path) as image:
                tensors.append(preprocess(image.convert("RGB")))
        batch_embeddings = encode_batch(model, tensors, device)
        for (frame, _, signature), embedding in zip(batch_items, batch_embeddings):
            cache.put(frame, signature, embedding)
            embeddings[frame] = embedding
        cache.save()
        completed = min(batch_index * batch_size, len(pending))
        print(f"Encoded raw embeddings: {completed}/{len(pending)}")
    return embeddings


def aligned_key(frame, best_shift):
    return f"{frame}:{best_shift % NUM_SECTORS}"


def build_aligned_embeddings(
    candidates,
    bev_dir,
    model,
    preprocess,
    device,
    raw_embeddings,
    cache,
    batch_size,
):
    pairs = sorted(
        {
            (int(row.candidate_frame), int(row.best_shift) % NUM_SECTORS)
            for row in candidates.itertuples()
        }
    )
    embeddings = {}
    pending = []
    for frame, shift in pairs:
        path = bev_dir / f"{frame:06d}.png"
        signature = file_signature(path)
        key = aligned_key(frame, shift)
        if shift == 0:
            embeddings[key] = raw_embeddings[frame]
            cache.put(key, signature, raw_embeddings[frame])
            continue
        cached = cache.get(key, signature)
        if cached is None:
            pending.append((key, frame, shift, path, signature))
        else:
            embeddings[key] = cached

    print(
        f"Aligned embeddings: {len(embeddings)} cached/reused, "
        f"{len(pending)} to encode"
    )
    cache.save()
    for batch_index, batch_items in enumerate(chunks(pending, batch_size), start=1):
        tensors = []
        for _, _, shift, path, _ in batch_items:
            with Image.open(path) as image:
                aligned = image.convert("RGB").rotate(
                    shift * DEG_PER_SHIFT,
                    resample=Image.Resampling.BILINEAR,
                    expand=False,
                    fillcolor=(0, 0, 0),
                )
                tensors.append(preprocess(aligned))
        batch_embeddings = encode_batch(model, tensors, device)
        for (key, _, _, _, signature), embedding in zip(
            batch_items, batch_embeddings
        ):
            cache.put(key, signature, embedding)
            embeddings[key] = embedding
        cache.save()
        completed = min(batch_index * batch_size, len(pending))
        print(f"Encoded aligned embeddings: {completed}/{len(pending)}")
    return embeddings


def add_bev_scores(candidates, raw_embeddings, aligned_embeddings):
    raw_scores = []
    aligned_scores = []
    pil_angles = []
    for row in candidates.itertuples():
        query_embedding = raw_embeddings[int(row.query_frame)]
        candidate_embedding = raw_embeddings[int(row.candidate_frame)]
        shift = int(row.best_shift) % NUM_SECTORS
        aligned_embedding = aligned_embeddings[
            aligned_key(int(row.candidate_frame), shift)
        ]
        raw_scores.append(float(torch.dot(query_embedding, candidate_embedding)))
        aligned_scores.append(float(torch.dot(query_embedding, aligned_embedding)))
        pil_angles.append(shift * DEG_PER_SHIFT)

    scored = candidates.copy()
    scored["pil_rotation_deg"] = pil_angles
    scored["raw_bev_similarity"] = raw_scores
    scored["aligned_bev_similarity"] = aligned_scores
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
        positive_indices = reranked.index[reranked["is_positive"] == 1].tolist()
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

    start_time = time.time()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Reading candidates: {args.candidate_csv}")
    all_candidates = pd.read_csv(args.candidate_csv)
    candidates = validate_candidate_csv(all_candidates, args.top_k)
    frames = required_frames(candidates)
    generated = ensure_bev_files(
        frames,
        args.bev_dir,
        args.velodyne_dir,
        args.generate_missing_bev,
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Valid queries: {candidates['query_frame'].nunique()}")
    print(f"Top-{args.top_k} pairs: {len(candidates)}")
    print(f"Unique required frames: {len(frames)}")
    print(f"Device: {device}")
    print(f"Loading OpenCLIP: {args.model_name}")
    model, _, preprocess = open_clip.create_model_and_transforms(
        args.model_name, pretrained=str(args.checkpoint)
    )
    model = model.to(device).eval()

    identity = cache_identity(args.model_name, args.checkpoint)
    cache_dir = args.output_dir / "cache"
    raw_cache = EmbeddingCache(
        cache_dir / "raw_embeddings.pt", identity, args.rebuild_cache
    )
    aligned_cache = EmbeddingCache(
        cache_dir / "aligned_embeddings.pt", identity, args.rebuild_cache
    )
    raw_embeddings = build_raw_embeddings(
        frames,
        args.bev_dir,
        model,
        preprocess,
        device,
        raw_cache,
        args.batch_size,
    )
    aligned_embeddings = build_aligned_embeddings(
        candidates,
        args.bev_dir,
        model,
        preprocess,
        device,
        raw_embeddings,
        aligned_cache,
        args.batch_size,
    )
    scored = add_bev_scores(candidates, raw_embeddings, aligned_embeddings)

    metrics_rows = []
    query_rows = []
    baseline_metrics, baseline_queries = evaluate_method(
        scored, "scan_context_baseline", "scan_context_score"
    )
    metrics_rows.append(baseline_metrics)
    query_rows.extend(baseline_queries)

    bev_metrics, bev_queries = evaluate_method(
        scored, "aligned_bev_only", "aligned_bev_similarity"
    )
    metrics_rows.append(bev_metrics)
    query_rows.extend(bev_queries)

    for alpha in args.alphas:
        column = f"fusion_alpha_{alpha:.1f}"
        scored[column] = (
            alpha * scored["scan_context_score"]
            + (1.0 - alpha) * scored["aligned_bev_similarity"]
        )
        method = f"sc_bev_fusion_alpha_{alpha:.1f}"
        metrics, queries = evaluate_method(scored, method, column)
        metrics_rows.append(metrics)
        query_rows.extend(queries)

    metrics_df = pd.DataFrame(metrics_rows)
    query_df = pd.DataFrame(query_rows)
    scored.to_csv(args.output_dir / "candidate_scores.csv", index=False)
    query_df.to_csv(args.output_dir / "query_results.csv", index=False)
    metrics_df.to_csv(args.output_dir / "metrics.csv", index=False)

    pool_r_at_k = float(
        scored.groupby("query_frame")["is_positive"].max().mean()
    )
    run_config = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "candidate_csv": str(args.candidate_csv),
        "bev_dir": str(args.bev_dir),
        "checkpoint": str(args.checkpoint),
        "model_name": args.model_name,
        "device": device,
        "top_k": args.top_k,
        "alphas": args.alphas,
        "fusion_formula": (
            "alpha * scan_context_score + (1-alpha) * aligned_bev_similarity"
        ),
        "pil_rotation_formula": "+best_shift * 6 degrees",
        "valid_queries": int(candidates["query_frame"].nunique()),
        "candidate_pairs": len(candidates),
        "unique_frames": len(frames),
        "generated_bev_frames": generated,
        "candidate_pool_r_at_k": pool_r_at_k,
        "elapsed_seconds": time.time() - start_time,
    }
    with open(args.output_dir / "run_config.json", "w") as file:
        json.dump(run_config, file, indent=2)

    report_lines = [
        "Full BEV Reranking Results",
        f"Valid queries: {run_config['valid_queries']}",
        f"SC Top-{args.top_k} candidate-pool recall: {pool_r_at_k:.6f}",
        "",
        metrics_df.to_string(index=False, float_format=lambda value: f"{value:.6f}"),
        "",
        f"Elapsed seconds: {run_config['elapsed_seconds']:.1f}",
    ]
    report = "\n".join(report_lines)
    (args.output_dir / "metrics.txt").write_text(report + "\n")
    print("\n" + report)
    print(f"\nSaved results to: {args.output_dir}")


if __name__ == "__main__":
    main()
