import argparse
import os
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
CASES = {
    1545: [(4525, "WRONG"), (4540, "CORRECT")],
    1550: [
        (3555, "WRONG"),
        (3550, "FALSE_CANDIDATE"),
        (4540, "CORRECT"),
    ],
    1560: [(3545, "WRONG"), (4535, "CORRECT")],
}


def args():
    parser = argparse.ArgumentParser(description="Temporal RGB case sanity check")
    parser.add_argument(
        "--rgb-dir",
        type=Path,
        default=Path("data/kitti/dataset/sequences/00/image_2"),
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
        "--checkpoint",
        type=Path,
        default=Path("models/openclip/vit_b32_laion400m_e32.pt"),
    )
    parser.add_argument(
        "--single-cache",
        type=Path,
        default=Path("outputs/full_rgb_reranking/cache/rgb_embeddings.pt"),
    )
    parser.add_argument(
        "--temporal-cache",
        type=Path,
        default=Path(
            "outputs/temporal_rgb_reranking/cache/rgb_frame_embeddings.pt"
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("outputs/temporal_rgb_cases")
    )
    parser.add_argument("--batch-size", type=int, default=16)
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


def collect_metadata(candidate_data, yaw_data):
    records = []
    for query_frame, candidates in CASES.items():
        for candidate_frame, label in candidates:
            candidate = unique_row(
                candidate_data, query_frame, candidate_frame, "candidate"
            )
            yaw = unique_row(yaw_data, query_frame, candidate_frame, "yaw")
            records.append(
                {
                    "query_frame": query_frame,
                    "candidate_frame": candidate_frame,
                    "label": label,
                    "sc_rank": int(candidate["rank"]),
                    "scan_context_score": float(candidate["scan_context_score"]),
                    "gt_distance": float(candidate["gt_distance"]),
                    "is_positive": int(candidate["is_positive"]),
                    "gt_relative_yaw_deg": float(yaw["gt_relative_yaw_deg"]),
                }
            )
    return records


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
    def __init__(self, path, identity):
        self.path = path
        self.identity = identity
        self.entries = {}
        if path.is_file():
            payload = torch.load(path, map_location="cpu", weights_only=True)
            if payload.get("identity") == identity:
                self.entries = payload.get("entries", {})
                print(f"Loaded {len(self.entries)} temporal cache entries from {path}")
            else:
                print(f"Ignoring incompatible temporal cache: {path}")

    def import_compatible(self, source_path):
        if self.entries or not source_path.is_file():
            return
        payload = torch.load(source_path, map_location="cpu", weights_only=True)
        if payload.get("identity") != self.identity:
            print(f"Ignoring incompatible single-frame cache: {source_path}")
            return
        self.entries = payload.get("entries", {}).copy()
        print(f"Imported {len(self.entries)} compatible entries from {source_path}")
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


def encode_frame_embeddings(
    frame_ids, rgb_dir, model, preprocess, device, batch_size, cache
):
    embeddings = {}
    ordered = sorted(frame_ids)
    pending = []
    for frame in ordered:
        path = rgb_dir / f"{frame:06d}.png"
        require_file(path, f"RGB frame {frame:06d}")
        signature = file_signature(path)
        cached = cache.get(frame, signature)
        if cached is None:
            pending.append((frame, path, signature))
        else:
            embeddings[frame] = cached

    print(f"RGB frames: {len(embeddings)} cached, {len(pending)} to encode")
    for start in range(0, len(pending), batch_size):
        batch_items = pending[start : start + batch_size]
        tensors = []
        for _, path, _ in batch_items:
            with Image.open(path) as image:
                tensors.append(preprocess(image.convert("RGB")))
        batch = torch.stack(tensors).to(device)
        with torch.inference_mode():
            batch_embeddings = F.normalize(
                model.encode_image(batch), p=2, dim=-1
            ).detach().cpu().float()
        for (frame, _, signature), embedding in zip(batch_items, batch_embeddings):
            cache.put(frame, signature, embedding)
            embeddings[frame] = embedding
        cache.save()
        print(f"Encoded RGB frames: {min(start + len(batch_items), len(pending))}/{len(pending)}")
    return embeddings


def temporal_descriptor(frame, offsets, frame_embeddings):
    used_frames = [
        frame + offset
        for offset in offsets
        if frame + offset >= 0 and frame + offset in frame_embeddings
    ]
    if not used_frames:
        raise ValueError(f"No usable temporal frames for target {frame}")
    pooled = torch.stack([frame_embeddings[item] for item in used_frames]).mean(dim=0)
    descriptor = F.normalize(pooled.unsqueeze(0), p=2, dim=-1).squeeze(0)
    return descriptor, used_frames


def build_descriptors(records, frame_embeddings):
    target_frames = sorted(
        {record["query_frame"] for record in records}
        | {record["candidate_frame"] for record in records}
    )
    descriptors = {}
    counts = {}
    for representation, offsets in WINDOWS.items():
        for frame in target_frames:
            descriptor, used = temporal_descriptor(frame, offsets, frame_embeddings)
            descriptors[(frame, representation)] = descriptor
            counts[(frame, representation)] = len(used)
    return descriptors, counts


def score_records(records, descriptors, counts):
    output = []
    for representation in WINDOWS:
        for record in records:
            query_descriptor = descriptors[(record["query_frame"], representation)]
            candidate_descriptor = descriptors[(record["candidate_frame"], representation)]
            row = dict(record)
            row["representation"] = representation
            row["temporal_frame_count_query"] = counts[
                (record["query_frame"], representation)
            ]
            row["temporal_frame_count_candidate"] = counts[
                (record["candidate_frame"], representation)
            ]
            row["temporal_rgb_similarity"] = float(
                torch.dot(query_descriptor, candidate_descriptor)
            )
            output.append(row)
    return pd.DataFrame(output)


def summarize(scores):
    rows = []
    for (query_frame, representation), group in scores.groupby(
        ["query_frame", "representation"], sort=True
    ):
        wrong = group[group.label == "WRONG"].iloc[0]
        correct = group[group.label == "CORRECT"].iloc[0]
        false = group[group.label != "CORRECT"]
        strongest = false.loc[false.temporal_rgb_similarity.idxmax()]
        correct_similarity = float(correct.temporal_rgb_similarity)
        wrong_similarity = float(wrong.temporal_rgb_similarity)
        strongest_similarity = float(strongest.temporal_rgb_similarity)
        rows.append(
            {
                "query_frame": int(query_frame),
                "representation": representation,
                "wrong_candidate_frame": int(wrong.candidate_frame),
                "correct_candidate_frame": int(correct.candidate_frame),
                "strongest_false_candidate_frame": int(strongest.candidate_frame),
                "wrong_similarity": wrong_similarity,
                "correct_similarity": correct_similarity,
                "strongest_false_similarity": strongest_similarity,
                "pairwise_gap": correct_similarity - wrong_similarity,
                "full_case_gap": correct_similarity - strongest_similarity,
                "pairwise_corrected": bool(correct_similarity > wrong_similarity),
                "case_corrected": bool(correct_similarity > strongest_similarity),
                "query_temporal_frame_count": int(
                    correct.temporal_frame_count_query
                ),
                "correct_temporal_frame_count": int(
                    correct.temporal_frame_count_candidate
                ),
            }
        )
    return pd.DataFrame(rows)


def main():
    parsed = args()
    for path, description in [
        (parsed.candidate_csv, "candidate CSV"),
        (parsed.yaw_csv, "yaw CSV"),
        (parsed.single_scores_csv, "single RGB case scores"),
        (parsed.checkpoint, "OpenCLIP checkpoint"),
    ]:
        require_file(path, description)
    if parsed.output_dir.exists() and not parsed.overwrite:
        raise FileExistsError(
            f"Output directory exists; use --overwrite explicitly: {parsed.output_dir}"
        )
    parsed.output_dir.mkdir(parents=True, exist_ok=True)

    candidate_data = pd.read_csv(parsed.candidate_csv)
    yaw_data = pd.read_csv(parsed.yaw_csv)
    single_scores = pd.read_csv(parsed.single_scores_csv)
    records = collect_metadata(candidate_data, yaw_data)
    target_frames = sorted(
        {r["query_frame"] for r in records}
        | {r["candidate_frame"] for r in records}
    )
    temporal_frames = sorted(
        {
            frame + offset
            for frame in target_frames
            for offsets in WINDOWS.values()
            for offset in offsets
            if frame + offset >= 0
        }
    )
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Device: {device}")
    print(f"Model: {MODEL_NAME}")
    print(f"Unique target frames: {len(target_frames)}")
    print(f"Unique temporal RGB frames: {len(temporal_frames)}")
    print("RGB rotation: none")

    model, _, preprocess = open_clip.create_model_and_transforms(
        MODEL_NAME, pretrained=str(parsed.checkpoint)
    )
    model = model.to(device).eval()
    cache = EmbeddingCache(parsed.temporal_cache, cache_identity(parsed.checkpoint))
    cache.import_compatible(parsed.single_cache)
    frame_embeddings = encode_frame_embeddings(
        temporal_frames,
        parsed.rgb_dir,
        model,
        preprocess,
        device,
        parsed.batch_size,
        cache,
    )
    descriptors, counts = build_descriptors(records, frame_embeddings)
    scores = score_records(records, descriptors, counts)
    summary = summarize(scores)

    # Preserve the previously measured single-frame scores in a separate column
    # for direct comparison; the fresh single representation is also reported.
    single_reference = single_scores[
        ["query_frame", "candidate_frame", "rgb_cosine_similarity"]
    ].rename(columns={"rgb_cosine_similarity": "previous_single_rgb_similarity"})
    scores = scores.merge(
        single_reference,
        on=["query_frame", "candidate_frame"],
        how="left",
        validate="many_to_one",
    )
    summary = summary.merge(
        single_reference.groupby("query_frame", as_index=False)[
            "previous_single_rgb_similarity"
        ].mean(),
        on="query_frame",
        how="left",
    )

    scores.to_csv(parsed.output_dir / "case_scores.csv", index=False)
    summary.to_csv(parsed.output_dir / "case_summary.csv", index=False)
    lines = [
        "TEMPORAL RGB CASE SANITY CHECK",
        "",
        summary.to_string(index=False, float_format=lambda value: f"{value:.6f}"),
        "",
        "Previous single-frame RGB scores are included in case_scores.csv as "
        "previous_single_rgb_similarity.",
    ]
    (parsed.output_dir / "summary.txt").write_text("\n".join(lines) + "\n")

    print("\n" + "\n".join(lines))
    print(f"Saved results to: {parsed.output_dir}")


if __name__ == "__main__":
    main()
