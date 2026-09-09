import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image

OFFSETS = [-10, -5, 0]
QUERIES = [1545, 1550, 1560]


def parse_args():
    parser = argparse.ArgumentParser(description="Prepare VLM verification collages")
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
        default=Path("outputs/vlm_verification_pilot"),
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def require_file(path, description):
    if not path.is_file():
        raise FileNotFoundError(f"Missing {description}: {path}")


def temporal_paths(frame, rgb_dir):
    paths = []
    for offset in OFFSETS:
        source = frame + offset
        if source < 0:
            continue
        path = rgb_dir / f"{source:06d}.png"
        if path.is_file():
            paths.append((offset, path))
    if not paths:
        raise FileNotFoundError(f"No RGB frames available for {frame}")
    return paths


def create_row_image(frame, label, rgb_dir):
    sources = temporal_paths(frame, rgb_dir)
    images = []
    for _, path in sources:
        with Image.open(path) as image:
            images.append(image.convert("RGB").copy())
    return images, sources


def make_collage(query_frame, candidate_a, candidate_b, output_path, rgb_dir):
    rows = [
        ("QUERY", query_frame),
        ("CANDIDATE A", candidate_a),
        ("CANDIDATE B", candidate_b),
    ]
    row_images = []
    max_columns = len(OFFSETS)
    for label, frame in rows:
        images, sources = create_row_image(frame, label, rgb_dir)
        row_images.append((label, frame, images, sources))
        max_columns = max(max_columns, len(images))

    figure, axes = plt.subplots(
        3,
        max_columns,
        figsize=(5.0 * max_columns, 7.0),
        squeeze=False,
    )
    for row_index, (label, frame, images, sources) in enumerate(row_images):
        for column_index in range(max_columns):
            axis = axes[row_index][column_index]
            axis.set_axis_off()
            if column_index >= len(images):
                continue
            axis.imshow(images[column_index])
            offset = sources[column_index][0]
            time_label = "t" if offset == 0 else f"t{offset}"
            if row_index == 0:
                axis.set_title(time_label, fontsize=10)
            if column_index == 0:
                axis.text(
                    -0.035,
                    0.5,
                    label,
                    transform=axis.transAxes,
                    ha="right",
                    va="center",
                    fontsize=11,
                )

    figure.suptitle("Temporal RGB Place Verification", fontsize=14, fontweight="bold")
    figure.tight_layout(rect=(0.08, 0.02, 1.0, 0.94))
    figure.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(figure)


def main():
    args = parse_args()
    require_file(args.candidate_csv, "formal candidate CSV")
    if not args.rgb_dir.is_dir():
        raise FileNotFoundError(f"Missing RGB directory: {args.rgb_dir}")
    if args.output_dir.exists() and not args.overwrite:
        raise FileExistsError(
            f"Output directory exists; use --overwrite: {args.output_dir}"
        )
    collage_dir = args.output_dir / "collages"
    collage_dir.mkdir(parents=True, exist_ok=True)

    candidates = pd.read_csv(args.candidate_csv)
    rows = []
    for query_frame in QUERIES:
        query_rows = candidates[
            (candidates.query_frame.astype(int) == query_frame)
            & (candidates["rank"].astype(int) <= 5)
        ].sort_values("rank")
        if len(query_rows) != 5:
            raise ValueError(f"Expected five Top-5 rows for query {query_frame}")
        rank1 = query_rows.iloc[0]
        for _, challenger in query_rows.iloc[1:].iterrows():
            challenger_frame = int(challenger.candidate_frame)
            challenger_rank = int(challenger["rank"])
            for order, candidate_a, candidate_b in [
                ("AB", int(rank1.candidate_frame), challenger_frame),
                ("BA", challenger_frame, int(rank1.candidate_frame)),
            ]:
                collage_name = (
                    f"q{query_frame:06d}_r1_{int(rank1.candidate_frame):06d}_"
                    f"challenger_{challenger_frame:06d}_{order}.png"
                )
                collage_path = collage_dir / collage_name
                make_collage(
                    query_frame,
                    candidate_a,
                    candidate_b,
                    collage_path,
                    args.rgb_dir,
                )
                rows.append(
                    {
                        "query_frame": query_frame,
                        "sc_rank1_frame": int(rank1.candidate_frame),
                        "challenger_frame": challenger_frame,
                        "challenger_original_rank": challenger_rank,
                        "order": order,
                        "candidate_A_frame": candidate_a,
                        "candidate_B_frame": candidate_b,
                        "collage_path": str(collage_path),
                        # Offline-only fields. These are never passed to the VLM.
                        "candidate_A_is_positive": int(
                            query_rows[
                                query_rows.candidate_frame.astype(int) == candidate_a
                            ].iloc[0].is_positive
                        ),
                        "candidate_B_is_positive": int(
                            query_rows[
                                query_rows.candidate_frame.astype(int) == candidate_b
                            ].iloc[0].is_positive
                        ),
                        "query_has_positive_in_top5": int(
                            query_rows.is_positive.astype(int).any()
                        ),
                    }
                )

    manifest = pd.DataFrame(rows)
    manifest.to_csv(args.output_dir / "manifest.csv", index=False)
    print(f"Generated collages: {len(manifest)}")
    print(f"Manifest: {args.output_dir / 'manifest.csv'}")
    print(f"Collage directory: {collage_dir}")


if __name__ == "__main__":
    main()
