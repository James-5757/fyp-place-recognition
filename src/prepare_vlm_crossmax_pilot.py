import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image

OFFSETS = [-10, -5, 0]
QUERIES = [1545, 1550, 1560]


def parse_args():
    parser = argparse.ArgumentParser(description="Prepare Cross-Max VLM pilot")
    parser.add_argument(
        "--crossmax-csv",
        type=Path,
        default=Path(
            "outputs/temporal_cross_frame_reranking/candidate_scores.csv"
        ),
    )
    parser.add_argument(
        "--candidate-csv",
        type=Path,
        default=Path(
            "outputs/formal_split2500_gap100_step5_thr5_candidates.csv"
        ),
    )
    parser.add_argument(
        "--rgb-dir",
        type=Path,
        default=Path("data/kitti/dataset/sequences/00/image_2"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/vlm_crossmax_pilot"),
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def temporal_paths(frame, rgb_dir):
    paths = []
    for offset in OFFSETS:
        source = frame + offset
        if source >= 0:
            path = rgb_dir / f"{source:06d}.png"
            if path.is_file():
                paths.append((offset, path))
    if not paths:
        raise FileNotFoundError(f"No RGB frames for {frame}")
    return paths


def make_collage(query_frame, candidate_a, candidate_b, output_path, rgb_dir):
    rows = [("QUERY", query_frame), ("CANDIDATE A", candidate_a), ("CANDIDATE B", candidate_b)]
    figure, axes = plt.subplots(3, 3, figsize=(15, 7), squeeze=False)
    for row_index, (label, frame) in enumerate(rows):
        sources = temporal_paths(frame, rgb_dir)
        for column_index, (offset, path) in enumerate(sources):
            axis = axes[row_index][column_index]
            with Image.open(path) as image:
                axis.imshow(image.convert("RGB"))
            axis.set_axis_off()
            if row_index == 0:
                axis.set_title("t" if offset == 0 else f"t{offset}", fontsize=10)
            if column_index == 0:
                axis.text(
                    -0.035, 0.5, label, transform=axis.transAxes,
                    ha="right", va="center", fontsize=11,
                )
        for column_index in range(len(sources), 3):
            axes[row_index][column_index].set_axis_off()
    figure.suptitle("Temporal RGB Place Verification", fontsize=14, fontweight="bold")
    figure.tight_layout(rect=(0.08, 0.02, 1.0, 0.94))
    figure.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(figure)


def main():
    args = parse_args()
    if not args.crossmax_csv.is_file() or not args.candidate_csv.is_file():
        raise FileNotFoundError("Missing cross-max or candidate input")
    if args.output_dir.exists() and not args.overwrite:
        raise FileExistsError(f"Output exists; use --overwrite: {args.output_dir}")
    collage_dir = args.output_dir / "collages"
    collage_dir.mkdir(parents=True, exist_ok=True)
    crossmax = pd.read_csv(args.crossmax_csv)
    candidates = pd.read_csv(args.candidate_csv)
    rows = []
    for query_frame in QUERIES:
        pool = crossmax[(crossmax.query_frame == query_frame) & (crossmax["rank"] <= 5)].copy()
        pool = pool.sort_values("rank")
        rank1 = pool.iloc[0]
        rank1_frame = int(rank1.candidate_frame)
        challengers = pool[pool.candidate_frame.astype(int) != rank1_frame]
        selected = challengers.sort_values(
            ["temporal_cross_max", "rank"], ascending=[False, True]
        ).iloc[0]
        challenger_frame = int(selected.candidate_frame)
        candidate_meta = candidates[
            (candidates.query_frame == query_frame)
            & (candidates.candidate_frame == challenger_frame)
        ].iloc[0]
        best_q_frame = int(selected.best_query_temporal_frame)
        best_c_frame = int(selected.best_candidate_temporal_frame)
        for order, a_frame, b_frame in [
            ("AB", rank1_frame, challenger_frame),
            ("BA", challenger_frame, rank1_frame),
        ]:
            name = (
                f"q{query_frame:06d}_r1_{rank1_frame:06d}_"
                f"crossmax_{challenger_frame:06d}_{order}.png"
            )
            collage_path = collage_dir / name
            make_collage(query_frame, a_frame, b_frame, collage_path, args.rgb_dir)
            a_meta = candidates[
                (candidates.query_frame == query_frame)
                & (candidates.candidate_frame == a_frame)
            ].iloc[0]
            b_meta = candidates[
                (candidates.query_frame == query_frame)
                & (candidates.candidate_frame == b_frame)
            ].iloc[0]
            rows.append({
                "query_frame": query_frame,
                "sc_rank1_frame": rank1_frame,
                "challenger_frame": challenger_frame,
                "challenger_original_rank": int(candidate_meta["rank"]),
                "order": order,
                "candidate_A_frame": a_frame,
                "candidate_B_frame": b_frame,
                "collage_path": str(collage_path),
                "crossmax_score": float(selected.temporal_cross_max),
                "best_query_temporal_frame": best_q_frame,
                "best_candidate_temporal_frame": best_c_frame,
                "candidate_A_is_positive": int(a_meta.is_positive),
                "candidate_B_is_positive": int(b_meta.is_positive),
                "query_has_positive_in_top5": int(pool.is_positive.astype(int).any()),
            })
    manifest = pd.DataFrame(rows)
    manifest.to_csv(args.output_dir / "manifest.csv", index=False)
    print("Generated collages",len(manifest))
    print("Manifest",args.output_dir / "manifest.csv")
    print(manifest[["query_frame","sc_rank1_frame","challenger_frame","order","best_query_temporal_frame","best_candidate_temporal_frame"]].to_string(index=False))


if __name__ == "__main__":
    main()
