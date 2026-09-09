import argparse
from pathlib import Path

import open_clip
import pandas as pd
import torch
import torch.nn.functional as F
from PIL import Image


MODEL_NAME = "ViT-B-32-quickgelu"
DEFAULT_CHECKPOINT = Path("models/openclip/vit_b32_laion400m_e32.pt")

CASES = {
    1545: [
        (4525, "WRONG"),
        (4540, "CORRECT"),
    ],
    1550: [
        (3555, "WRONG"),
        (3550, "FALSE_CANDIDATE"),
        (4540, "CORRECT"),
    ],
    1560: [
        (3545, "WRONG"),
        (4535, "CORRECT"),
    ],
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Case-level KITTI RGB OpenCLIP sanity check."
    )
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
        "--bev-scores-csv",
        type=Path,
        default=Path("outputs/full_bev_reranking/candidate_scores.csv"),
    )
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("outputs/rgb_clip_sanity")
    )
    return parser.parse_args()


def require_file(path, description):
    if not path.is_file():
        raise FileNotFoundError(f"Missing {description}: {path}")


def unique_row(dataframe, query_frame, candidate_frame, source_name):
    rows = dataframe[
        (dataframe["query_frame"].astype(int) == query_frame)
        & (dataframe["candidate_frame"].astype(int) == candidate_frame)
    ]
    if len(rows) != 1:
        raise ValueError(
            f"Expected one {source_name} row for "
            f"{query_frame}->{candidate_frame}; found {len(rows)}"
        )
    return rows.iloc[0]


def collect_metadata(candidate_data, yaw_data):
    records = []
    for query_frame, candidates in CASES.items():
        for candidate_frame, label in candidates:
            candidate_row = unique_row(
                candidate_data,
                query_frame,
                candidate_frame,
                "formal candidate",
            )
            yaw_row = unique_row(
                yaw_data,
                query_frame,
                candidate_frame,
                "yaw validation",
            )
            records.append(
                {
                    "query_frame": query_frame,
                    "candidate_frame": candidate_frame,
                    "label": label,
                    "sc_rank": int(candidate_row["rank"]),
                    "scan_context_score": float(
                        candidate_row["scan_context_score"]
                    ),
                    "gt_distance": float(candidate_row["gt_distance"]),
                    "is_positive": int(candidate_row["is_positive"]),
                    "gt_relative_yaw_deg": float(
                        yaw_row["gt_relative_yaw_deg"]
                    ),
                }
            )
    return records


def load_and_encode_unique_frames(
    frame_ids, rgb_dir, model, preprocess, device
):
    tensors = []
    ordered_frames = sorted(frame_ids)
    for frame_id in ordered_frames:
        image_path = rgb_dir / f"{frame_id:06d}.png"
        require_file(image_path, f"RGB frame {frame_id:06d}")
        with Image.open(image_path) as image:
            tensors.append(preprocess(image.convert("RGB")))

    batch = torch.stack(tensors).to(device)
    with torch.inference_mode():
        embeddings = model.encode_image(batch)
        embeddings = F.normalize(embeddings, p=2, dim=-1)
    embeddings = embeddings.detach().cpu().float()
    return {
        frame_id: embedding
        for frame_id, embedding in zip(ordered_frames, embeddings)
    }


def add_rgb_scores(records, embeddings):
    scored_records = []
    for record in records:
        query_embedding = embeddings[record["query_frame"]]
        candidate_embedding = embeddings[record["candidate_frame"]]
        scored = dict(record)
        scored["rgb_cosine_similarity"] = float(
            torch.dot(query_embedding, candidate_embedding)
        )
        scored_records.append(scored)
    return pd.DataFrame(scored_records)


def summarize_cases(scores):
    summary_rows = []
    for query_frame, group in scores.groupby("query_frame", sort=True):
        wrong_rows = group[group["label"] == "WRONG"]
        correct_rows = group[group["label"] == "CORRECT"]
        false_rows = group[group["label"] != "CORRECT"]
        if len(wrong_rows) != 1 or len(correct_rows) != 1:
            raise ValueError(
                f"Query {query_frame} must have exactly one WRONG and one CORRECT"
            )

        wrong = wrong_rows.iloc[0]
        correct = correct_rows.iloc[0]
        strongest_false = false_rows.loc[
            false_rows["rgb_cosine_similarity"].idxmax()
        ]
        correct_similarity = float(correct["rgb_cosine_similarity"])
        wrong_similarity = float(wrong["rgb_cosine_similarity"])
        strongest_false_similarity = float(
            strongest_false["rgb_cosine_similarity"]
        )
        pairwise_gap = correct_similarity - wrong_similarity
        full_case_gap = correct_similarity - strongest_false_similarity

        summary_rows.append(
            {
                "query_frame": int(query_frame),
                "wrong_candidate_frame": int(wrong["candidate_frame"]),
                "correct_candidate_frame": int(correct["candidate_frame"]),
                "strongest_false_candidate_frame": int(
                    strongest_false["candidate_frame"]
                ),
                "wrong_similarity": wrong_similarity,
                "correct_similarity": correct_similarity,
                "strongest_false_similarity": strongest_false_similarity,
                "pairwise_gap": pairwise_gap,
                "full_case_gap": full_case_gap,
                "pairwise_corrected": bool(pairwise_gap > 0.0),
                "case_corrected": bool(full_case_gap > 0.0),
            }
        )
    return pd.DataFrame(summary_rows)


def make_bev_rgb_comparison(scores, bev_data):
    rows = []
    for score in scores.itertuples(index=False):
        bev_row = unique_row(
            bev_data,
            int(score.query_frame),
            int(score.candidate_frame),
            "aligned BEV score",
        )
        rows.append(
            {
                "query_frame": int(score.query_frame),
                "candidate_frame": int(score.candidate_frame),
                "label": score.label,
                "aligned_bev_similarity": float(
                    bev_row["aligned_bev_similarity"]
                ),
                "rgb_similarity": float(score.rgb_cosine_similarity),
            }
        )
    return pd.DataFrame(rows)


def recommendation(summary):
    positive_gaps = int(summary["pairwise_corrected"].sum())
    corrected_cases = int(summary["case_corrected"].sum())
    if positive_gaps >= 2 and corrected_cases >= 2:
        return "PROCEED_TO_FULL_RGB_RERANKING"
    return "VIEWPOINT_LIMITATION_DOMINANT"


def format_bool(value):
    return "yes" if bool(value) else "no"


def build_terminal_summary(scores, summary):
    by_query = summary.set_index("query_frame")
    score_lookup = {
        (int(row.query_frame), int(row.candidate_frame)): float(
            row.rgb_cosine_similarity
        )
        for row in scores.itertuples(index=False)
    }
    lines = ["RGB OPENCLIP SANITY CHECK"]

    row = by_query.loc[1545]
    lines.extend(
        [
            "Query 1545:",
            f"wrong similarity = {row.wrong_similarity:.6f}",
            f"correct similarity = {row.correct_similarity:.6f}",
            f"gap = {row.pairwise_gap:+.6f}",
            f"pairwise corrected = {format_bool(row.pairwise_corrected)}",
            f"case corrected = {format_bool(row.case_corrected)}",
        ]
    )

    row = by_query.loc[1550]
    lines.extend(
        [
            "Query 1550:",
            f"wrong 3555 = {score_lookup[(1550, 3555)]:.6f}",
            f"false 3550 = {score_lookup[(1550, 3550)]:.6f}",
            f"correct 4540 = {score_lookup[(1550, 4540)]:.6f}",
            f"pairwise gap = {row.pairwise_gap:+.6f}",
            f"full-case gap = {row.full_case_gap:+.6f}",
            f"pairwise corrected = {format_bool(row.pairwise_corrected)}",
            f"case corrected = {format_bool(row.case_corrected)}",
        ]
    )

    row = by_query.loc[1560]
    lines.extend(
        [
            "Query 1560:",
            f"wrong similarity = {row.wrong_similarity:.6f}",
            f"correct similarity = {row.correct_similarity:.6f}",
            f"gap = {row.pairwise_gap:+.6f}",
            f"pairwise corrected = {format_bool(row.pairwise_corrected)}",
            f"case corrected = {format_bool(row.case_corrected)}",
        ]
    )

    positive_gaps = int(summary["pairwise_corrected"].sum())
    corrected_cases = int(summary["case_corrected"].sum())
    lines.extend(
        [
            "Overall:",
            f"cases with positive correct-vs-wrong gap = {positive_gaps}/3",
            f"cases actually corrected = {corrected_cases}/3",
            f"recommendation: {recommendation(summary)}",
            "Note: this case-level result does not imply RGB viewpoint invariance.",
        ]
    )
    return "\n".join(lines)


def print_candidate_details(scores):
    print("\nDETAILED CANDIDATE SCORES")
    for row in scores.itertuples(index=False):
        print(
            f"query={int(row.query_frame)} "
            f"candidate={int(row.candidate_frame)} "
            f"label={row.label} "
            f"SC_rank={int(row.sc_rank)} "
            f"SC_score={row.scan_context_score:.6f} "
            f"GT_distance={row.gt_distance:.3f}m "
            f"GT_yaw={row.gt_relative_yaw_deg:+.2f}deg "
            f"RGB_similarity={row.rgb_cosine_similarity:.6f}"
        )


def main():
    args = parse_args()
    require_file(args.checkpoint, "local OpenCLIP checkpoint")
    require_file(args.candidate_csv, "formal candidate CSV")
    require_file(args.yaw_csv, "yaw validation CSV")
    require_file(args.bev_scores_csv, "full BEV candidate scores CSV")
    if not args.rgb_dir.is_dir():
        raise FileNotFoundError(f"Missing RGB directory: {args.rgb_dir}")

    output_files = [
        args.output_dir / "case_scores.csv",
        args.output_dir / "case_summary.csv",
        args.output_dir / "summary.txt",
        args.output_dir / "bev_vs_rgb_case_comparison.csv",
    ]
    existing = [path for path in output_files if path.exists()]
    if existing:
        raise FileExistsError(
            "Refusing to overwrite existing RGB sanity outputs: "
            + ", ".join(str(path) for path in existing)
        )

    candidate_data = pd.read_csv(args.candidate_csv)
    yaw_data = pd.read_csv(args.yaw_csv)
    bev_data = pd.read_csv(args.bev_scores_csv)
    records = collect_metadata(candidate_data, yaw_data)
    frame_ids = {
        frame
        for query_frame, candidates in CASES.items()
        for frame in [query_frame] + [candidate for candidate, _ in candidates]
    }

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Device: {device}")
    print(f"Model: {MODEL_NAME}")
    print(f"Local checkpoint: {args.checkpoint}")
    print(f"Unique RGB frames encoded once: {len(frame_ids)}")
    print("No RGB image rotation is performed.")

    model, _, preprocess = open_clip.create_model_and_transforms(
        MODEL_NAME,
        pretrained=str(args.checkpoint),
    )
    model = model.to(device).eval()
    embeddings = load_and_encode_unique_frames(
        frame_ids, args.rgb_dir, model, preprocess, device
    )
    scores = add_rgb_scores(records, embeddings)
    summary = summarize_cases(scores)
    comparison = make_bev_rgb_comparison(scores, bev_data)
    terminal_summary = build_terminal_summary(scores, summary)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    scores.to_csv(args.output_dir / "case_scores.csv", index=False)
    summary.to_csv(args.output_dir / "case_summary.csv", index=False)
    comparison.to_csv(
        args.output_dir / "bev_vs_rgb_case_comparison.csv", index=False
    )
    (args.output_dir / "summary.txt").write_text(terminal_summary + "\n")

    print_candidate_details(scores)
    print("\n" + terminal_summary)
    print(f"case_scores.csv: {args.output_dir / 'case_scores.csv'}")
    print(f"case_summary.csv: {args.output_dir / 'case_summary.csv'}")
    print(f"summary.txt: {args.output_dir / 'summary.txt'}")
    print(
        "bev_vs_rgb_case_comparison.csv: "
        f"{args.output_dir / 'bev_vs_rgb_case_comparison.csv'}"
    )


if __name__ == "__main__":
    main()
