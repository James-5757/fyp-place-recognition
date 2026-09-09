import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import open_clip
import numpy as np
import pandas as pd
import torch
import torch.nn.functional as F
from PIL import Image
from scipy.spatial import cKDTree

from scan_context import load_kitti_bin, make_scan_context, scan_context_similarity


ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
VELODYNE_DIR = KITTI_ROOT / "sequences/00/velodyne"
POSE_PATH = KITTI_ROOT / "poses/00.txt"
MODEL_NAME = "ViT-B-32-quickgelu"
WINDOW_OFFSETS = [-20, -15, -10, -5, 0]
ALPHAS = [0.9, 0.8, 0.7, 0.6, 0.5]
K_VALUES = [3, 5, 10]
RESCUE_QUERIES = [115, 380, 955, 1565]


def args():
    parser = argparse.ArgumentParser(description="SC Top-20 Cross-Max visual filtering")
    parser.add_argument("--split-frame", type=int, default=2500)
    parser.add_argument("--frame-step", type=int, default=5)
    parser.add_argument("--boundary-gap", type=int, default=100)
    parser.add_argument("--positive-threshold", type=float, default=5.0)
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument(
        "--existing-formal-candidates",
        type=Path,
        default=Path("outputs/formal_split2500_gap100_step5_thr5_candidates.csv"),
    )
    parser.add_argument(
        "--source-cache",
        type=Path,
        default=Path("outputs/temporal_rgb_reranking/cache/rgb_frame_embeddings.pt"),
    )
    parser.add_argument(
        "--rgb-dir", type=Path,
        default=Path("data/kitti/dataset/sequences/00/image_2"),
    )
    parser.add_argument(
        "--checkpoint", type=Path,
        default=Path("models/openclip/vit_b32_laion400m_e32.pt"),
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path("outputs/top20_visual_filtering"),
    )
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--overwrite-results", action="store_true")
    return parser.parse_args()


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
            if payload.get("identity") != identity:
                raise ValueError(f"Incompatible local cache: {path}")
            self.entries = payload.get("entries", {})

    def import_compatible(self, source):
        if self.entries:
            return 0
        payload = torch.load(source, map_location="cpu", weights_only=True)
        if payload.get("identity") != self.identity:
            raise ValueError(f"Incompatible source cache: {source}")
        self.entries = payload.get("entries", {}).copy()
        self.save()
        return len(self.entries)

    def get(self, frame, signature):
        item = self.entries.get(str(frame))
        if item is None or item.get("signature") != signature:
            return None
        return item["embedding"].float()

    def put(self, frame, signature, embedding):
        self.entries[str(frame)] = {
            "signature": signature,
            "embedding": embedding.detach().cpu().float(),
        }

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix(".tmp")
        torch.save({"identity": self.identity, "entries": self.entries}, temp)
        os.replace(temp, self.path)


def load_poses():
    poses = np.loadtxt(POSE_PATH).reshape(-1, 3, 4)
    return poses[:, :, 3]


def xz_distance(a, b):
    return float(np.hypot(a[0] - b[0], a[2] - b[2]))


def sampled_ids(step):
    return [int(p.stem) for p in sorted(VELODYNE_DIR.glob("*.bin"))][::step]


def valid_queries(robot_a, robot_b, positions, threshold):
    tree = cKDTree(positions[robot_b][:, [0, 2]])
    return [
        q for q in robot_a
        if tree.query(positions[q][[0, 2]], k=1)[0] < threshold
    ]


def descriptor_db(frames):
    database = []
    for index, frame in enumerate(frames, 1):
        points = load_kitti_bin(VELODYNE_DIR / f"{frame:06d}.bin")
        database.append({"frame": frame, "sc": make_scan_context(points)})
        if index % 100 == 0 or index == len(frames):
            print(f"Built SC descriptors: {index}/{len(frames)}")
    return database


def build_top20(valid_ids, db_a, db_b, positions, threshold, top_k):
    a_map = {item["frame"]: item["sc"] for item in db_a}
    rows = []
    for index, query in enumerate(valid_ids, 1):
        candidates = []
        for item in db_b:
            score, shift = scan_context_similarity(a_map[query], item["sc"])
            distance = xz_distance(positions[query], positions[item["frame"]])
            candidates.append((float(score), item["frame"], int(shift), distance))
        candidates.sort(key=lambda value: (-value[0], value[1]))
        for rank, (score, frame, shift, distance) in enumerate(candidates[:top_k], 1):
            rows.append({
                "query_frame": query,
                "candidate_frame": frame,
                "sc_rank": rank,
                "scan_context_score": score,
                "best_shift": shift,
                "gt_distance": distance,
                "is_positive": int(distance < threshold),
            })
        if index % 25 == 0 or index == len(valid_ids):
            print(f"Built Top-{top_k} pool: {index}/{len(valid_ids)}")
    return pd.DataFrame(rows)


def sanity_check(pool, existing):
    assert pool.query_frame.nunique() == 158, pool.query_frame.nunique()
    assert len(pool) == 3160, len(pool)
    assert (pool.groupby("query_frame").size() == 20).all()
    recalls = {
        k: float(
            pool[pool.sc_rank <= k].groupby("query_frame").is_positive.max().mean()
        )
        for k in [5, 10, 20]
    }
    old = existing[(existing.query_has_positive_in_B == 1) & (existing["rank"] <= 5)]
    old_r5 = float(old.groupby("query_frame").is_positive.max().mean())
    if not (
        abs(recalls[5] - old_r5) < 1e-12
        and abs(recalls[10] - 156 / 158) < 1e-12
        and abs(recalls[20] - 1.0) < 1e-12
    ):
        raise RuntimeError(f"SC pool sanity failed: {recalls}, existing R5={old_r5}")
    return recalls


def temporal_source_frames(targets, rgb_dir):
    return sorted({
        frame + offset
        for frame in targets
        for offset in WINDOW_OFFSETS
        if frame + offset >= 0 and (rgb_dir / f"{frame + offset:06d}.png").is_file()
    })


def build_embeddings(frames, rgb_dir, model, preprocess, device, cache, batch_size):
    embeddings = {}
    pending = []
    reused = 0
    for frame in frames:
        path = rgb_dir / f"{frame:06d}.png"
        signature = file_signature(path)
        cached = cache.get(frame, signature)
        if cached is None:
            pending.append((frame, path, signature))
        else:
            embeddings[frame] = cached
            reused += 1
    for start in range(0, len(pending), batch_size):
        batch_items = pending[start:start + batch_size]
        tensors = []
        for _, path, _ in batch_items:
            with Image.open(path) as image:
                tensors.append(preprocess(image.convert("RGB")))
        with torch.inference_mode():
            encoded = F.normalize(model.encode_image(torch.stack(tensors).to(device)), p=2, dim=-1)
        for (frame, _, signature), embedding in zip(batch_items, encoded.detach().cpu().float()):
            cache.put(frame, signature, embedding)
            embeddings[frame] = embedding
        cache.save()
        print(f"Encoded new RGB frames: {min(start + len(batch_items), len(pending))}/{len(pending)}")
    return embeddings, reused, len(pending)


def window(frame, embeddings):
    frames = [frame + offset for offset in WINDOW_OFFSETS if frame + offset >= 0 and frame + offset in embeddings]
    if not frames:
        raise ValueError(f"No RGB window for {frame}")
    return frames


def score_crossmax(pool, embeddings):
    targets = sorted(set(pool.query_frame.astype(int)) | set(pool.candidate_frame.astype(int)))
    windows = {frame: window(frame, embeddings) for frame in targets}
    rows = []
    for index, row in enumerate(pool.itertuples(index=False), 1):
        qframes, cframes = windows[int(row.query_frame)], windows[int(row.candidate_frame)]
        matrix = torch.stack([embeddings[f] for f in qframes]) @ torch.stack([embeddings[f] for f in cframes]).T
        flat_index = int(matrix.argmax())
        qi, ci = divmod(flat_index, matrix.shape[1])
        record = row._asdict()
        record.update({
            "temporal_cross_max": float(matrix.max()),
            "best_query_temporal_frame": qframes[qi],
            "best_candidate_temporal_frame": cframes[ci],
            "max_pair_similarity": float(matrix.max()),
            "query_temporal_frame_count": len(qframes),
            "candidate_temporal_frame_count": len(cframes),
        })
        rows.append(record)
        if index % 250 == 0 or index == len(pool):
            print(f"Scored Cross-Max pairs: {index}/{len(pool)}")
    return pd.DataFrame(rows)


def first_positive_rank(group, score_column):
    ranked = group.sort_values([score_column, "sc_rank"], ascending=[False, True], kind="stable").reset_index(drop=True)
    positions = ranked.index[ranked.is_positive.astype(int) == 1].tolist()
    return positions[0] + 1 if positions else None


def evaluate_filtering(scored):
    methods = {"sc_only": "scan_context_score", "cross_max_only": "temporal_cross_max"}
    for alpha in ALPHAS:
        column = f"sc_cross_max_alpha_{alpha:.1f}"
        scored[column] = alpha * scored.scan_context_score + (1 - alpha) * scored.temporal_cross_max
        methods[column] = column
    retention_rows, ranking_rows, event_rows, shift_rows, rescue_rows = [], [], [], [], []
    sc_ranks = {}
    method_ranks = {name: {} for name in methods}
    ranked_by_method = {name: {} for name in methods}
    for query, group in scored.groupby("query_frame", sort=True):
        sc_rank = first_positive_rank(group, "scan_context_score")
        sc_ranks[int(query)] = sc_rank
        for name, column in methods.items():
            ranked = group.sort_values([column, "sc_rank"], ascending=[False, True], kind="stable").reset_index(drop=True)
            ranked_by_method[name][int(query)] = ranked
            method_ranks[name][int(query)] = first_positive_rank(group, column)
    for name, column in methods.items():
        top1_hits, reciprocal, corrections, regressions = [], [], 0, 0
        for query, group in ranked_by_method[name].items():
            baseline = ranked_by_method["sc_only"][query]
            base_hit = int(baseline.iloc[0].is_positive)
            method_hit = int(group.iloc[0].is_positive)
            pos = group.index[group.is_positive.astype(int) == 1].tolist()
            top1_hits.append(method_hit); reciprocal.append(1 / (pos[0] + 1) if pos else 0)
            corrections += int(base_hit == 0 and method_hit == 1)
            regressions += int(base_hit == 1 and method_hit == 0)
        ranking_rows.append({"method": name, "r_at_1": sum(top1_hits)/158, "mrr": sum(reciprocal)/158, "top1_hits": sum(top1_hits), "corrections": corrections, "regressions": regressions, "net_gain": corrections-regressions})
        shifts = [sc_ranks[q] - method_ranks[name][q] for q in sc_ranks]
        for subset, ids in [("all_valid_queries", list(sc_ranks)), ("top5_miss_rescue_cases", RESCUE_QUERIES)]:
            values = [sc_ranks[q] - method_ranks[name][q] for q in ids]
            shift_rows.append({"method": name, "subset": subset, "queries": len(ids), "mean_rank_shift": float(np.mean(values)), "median_rank_shift": float(np.median(values))})
        for k in K_VALUES:
            retained = sum(method_ranks[name][q] <= k for q in sc_ranks)
            retention_rows.append({"method": name, "retained_k": k, "positives_retained": retained, "retention_rate": retained/158, "candidate_reduction": 1-k/20})
            rescues = [q for q in sc_ranks if sc_ranks[q] > k and method_ranks[name][q] <= k]
            losses = [q for q in sc_ranks if sc_ranks[q] <= k and method_ranks[name][q] > k]
            for q in rescues:
                event_rows.append({"method": name, "retained_k": k, "event": "RESCUE", "query_frame": q, "sc_first_positive_rank": sc_ranks[q], "method_first_positive_rank": method_ranks[name][q]})
            for q in losses:
                event_rows.append({"method": name, "retained_k": k, "event": "LOSS", "query_frame": q, "sc_first_positive_rank": sc_ranks[q], "method_first_positive_rank": method_ranks[name][q]})
            event_rows.append({"method": name, "retained_k": k, "event": "SUMMARY", "query_frame": pd.NA, "sc_first_positive_rank": len(rescues), "method_first_positive_rank": len(losses), "net_retention_gain": len(rescues)-len(losses)})
        for q in RESCUE_QUERIES:
            positive = ranked_by_method[name][q][ranked_by_method[name][q].is_positive.astype(int) == 1].iloc[0]
            score = float(positive[column])
            rescue_rows.append({"method": name, "query_frame": q, "positive_frame": int(positive.candidate_frame), "original_sc_rank": sc_ranks[q], "new_rank": method_ranks[name][q], "scan_context_score": float(positive.scan_context_score), "cross_max_score": float(positive.temporal_cross_max), "method_score": score, "retained_at_top3": method_ranks[name][q] <= 3, "retained_at_top5": method_ranks[name][q] <= 5, "retained_at_top10": method_ranks[name][q] <= 10})
    return scored, pd.DataFrame(retention_rows), pd.DataFrame(ranking_rows), pd.DataFrame(event_rows), pd.DataFrame(shift_rows), pd.DataFrame(rescue_rows)


def main():
    start_time = time.time()
    parsed = args()
    if parsed.output_dir.exists() and not parsed.overwrite_results:
        if any(parsed.output_dir.iterdir()):
            raise FileExistsError(f"Output exists: {parsed.output_dir}")
    positions = load_poses()
    sampled = sampled_ids(parsed.frame_step)
    robot_a = [f for f in sampled if f <= parsed.split_frame - parsed.boundary_gap]
    robot_b = [f for f in sampled if f >= parsed.split_frame + parsed.boundary_gap]
    valid = valid_queries(robot_a, robot_b, positions, parsed.positive_threshold)
    print("valid_queries",len(valid),"robot_b",len(robot_b))
    db_a, db_b = descriptor_db(robot_a), descriptor_db(robot_b)
    pool = build_top20(valid, db_a, db_b, positions, parsed.positive_threshold, parsed.top_k)
    recalls = sanity_check(pool, pd.read_csv(parsed.existing_formal_candidates))
    print("SC pool sanity passed",recalls)
    parsed.output_dir.mkdir(parents=True, exist_ok=True)
    pool.to_csv(parsed.output_dir / "sc_top20_candidates.csv", index=False)

    targets = sorted(set(pool.query_frame.astype(int)) | set(pool.candidate_frame.astype(int)))
    source_frames = temporal_source_frames(targets, parsed.rgb_dir)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, _, preprocess = open_clip.create_model_and_transforms(MODEL_NAME, pretrained=str(parsed.checkpoint))
    model = model.to(device).eval()
    cache = FrameCache(parsed.output_dir / "cache/rgb_frame_embeddings.pt", cache_identity(parsed.checkpoint))
    imported = cache.import_compatible(parsed.source_cache)
    embeddings, reused, encoded = build_embeddings(source_frames, parsed.rgb_dir, model, preprocess, device, cache, parsed.batch_size)
    scored = score_crossmax(pool, embeddings)
    scored, retention, ranking, events, shifts, rescue = evaluate_filtering(scored)
    scored.to_csv(parsed.output_dir / "candidate_scores.csv", index=False)
    retention.to_csv(parsed.output_dir / "retention_metrics.csv", index=False)
    ranking.to_csv(parsed.output_dir / "ranking_metrics.csv", index=False)
    events.to_csv(parsed.output_dir / "filtering_events.csv", index=False)
    shifts.to_csv(parsed.output_dir / "positive_rank_shifts.csv", index=False)
    rescue.to_csv(parsed.output_dir / "rescue_case_analysis.csv", index=False)

    top5 = retention[retention.retained_k == 5]
    visual = top5[top5.method != "sc_only"]
    best_visual = visual.sort_values(["retention_rate", "method"], ascending=[False, True]).iloc[0]
    sc_top5 = top5[top5.method == "sc_only"].iloc[0]
    if best_visual.retention_rate >= sc_top5.retention_rate and best_visual.retention_rate >= 0.974684:
        classification = "VISUAL_FILTERING_SUCCESSFUL"
    elif any((visual.retention_rate > sc_top5.retention_rate)):
        classification = "VISUAL_FILTERING_HELPS_RESCUE_BUT_CAUSES_TRADEOFF"
    else:
        classification = "CROSS_MAX_NOT_RELIABLE_FOR_TOP20_FILTERING"
    summary_table = retention.pivot(index="method", columns="retained_k", values="retention_rate").reset_index()
    top5_events = events[(events.retained_k == 5) & (events.event == "SUMMARY")]
    lines = ["TOP-20 VISUAL CANDIDATE FILTERING", "Input candidate pool: 158 queries, 20 candidates/query, 3160 pairs", "SC Top20 recall = 100%", "", "Retention table", summary_table.to_string(index=False, float_format=lambda x:f"{x:.6f}"), "", "Candidate reduction: Top20 -> Top10 = 50%; Top20 -> Top5 = 75%; Top20 -> Top3 = 85%", "", "Top5 rescue/loss summary", top5_events.to_string(index=False), "", "Four rescue cases"]
    for q in RESCUE_QUERIES:
        data = rescue[rescue.query_frame == q]
        best = data.sort_values(["new_rank", "method"]).iloc[0]
        lines.append(f"Query {q}: SC positive rank = {int(best.original_sc_rank)}, best new rank = {int(best.new_rank)}, retained Top5 = {bool(best.retained_at_top5)}")
    lines.append(f"classification: {classification}")
    (parsed.output_dir / "summary.txt").write_text("\n".join(lines)+"\n")
    config = {"created_at_utc":datetime.now(timezone.utc).isoformat(),"valid_queries":158,"candidate_pairs":3160,"source_cache_imported_entries":imported,"cached_embeddings_reused":reused,"new_embeddings_encoded":encoded,"total_unique_rgb_frames":len(source_frames),"sc_recalls":recalls,"classification":classification,"elapsed_seconds":time.time()-start_time}
    (parsed.output_dir / "run_config.json").write_text(json.dumps(config,indent=2))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
