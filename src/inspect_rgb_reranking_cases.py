import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image


CASES = [
    {
        "name": "1545",
        "items": [
            (1545, "QUERY"),
            (4525, "WRONG"),
            (4540, "CORRECT"),
        ],
    },
    {
        "name": "1550",
        "items": [
            (1550, "QUERY"),
            (3555, "WRONG"),
            (3550, "FALSE CANDIDATE"),
            (4540, "CORRECT"),
        ],
    },
    {
        "name": "1560",
        "items": [
            (1560, "QUERY"),
            (3545, "WRONG"),
            (4535, "CORRECT"),
        ],
    },
]

CORRECT_PAIRS = [(1545, 4540), (1550, 4540), (1560, 4535)]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inspect KITTI RGB images for Scan Context reranking cases."
    )
    parser.add_argument(
        "--sequence-dir",
        type=Path,
        default=Path("data/kitti/dataset/sequences/00"),
    )
    parser.add_argument(
        "--camera",
        default="image_2",
        choices=["image_0", "image_1", "image_2", "image_3"],
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
        "--output-dir", type=Path, default=Path("outputs/rgb_case_analysis")
    )
    return parser.parse_args()


def image_files(camera_dir):
    extensions = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
    return sorted(
        path for path in camera_dir.iterdir()
        if path.is_file() and path.suffix.lower() in extensions
    )


def frame_row(candidates, yaw, query_frame, candidate_frame):
    candidate_rows = candidates[
        (candidates["query_frame"].astype(int) == query_frame)
        & (candidates["candidate_frame"].astype(int) == candidate_frame)
    ]
    yaw_rows = yaw[
        (yaw["query_frame"].astype(int) == query_frame)
        & (yaw["candidate_frame"].astype(int) == candidate_frame)
    ]
    if len(candidate_rows) != 1:
        raise ValueError(
            f"Expected one formal candidate row for {query_frame}->{candidate_frame}; "
            f"found {len(candidate_rows)}"
        )
    if len(yaw_rows) != 1:
        raise ValueError(
            f"Expected one yaw row for {query_frame}->{candidate_frame}; "
            f"found {len(yaw_rows)}"
        )
    return candidate_rows.iloc[0], yaw_rows.iloc[0]


def label_for(frame, role, candidate_row, yaw_row):
    lines = [f"{role}\nframe {frame:06d}"]
    if role == "QUERY":
        lines.append("query image")
    else:
        lines.append(f"SC rank {int(candidate_row['rank'])}")
        lines.append(f"GT dist {float(candidate_row['gt_distance']):.3f} m")
        lines.append(f"GT yaw {float(yaw_row['gt_relative_yaw_deg']):+.2f} deg")
        lines.append(f"positive {'yes' if int(candidate_row['is_positive']) else 'no'}")
    return "\n".join(lines)


def make_case_figure(case, camera_dir, candidates, yaw, output_path):
    figure, axes = plt.subplots(
        1,
        len(case["items"]),
        figsize=(6.2 * len(case["items"]), 3.9),
        squeeze=False,
    )
    axes = axes[0]
    for axis, (frame, role) in zip(axes, case["items"]):
        image_path = camera_dir / f"{frame:06d}.png"
        if not image_path.is_file():
            raise FileNotFoundError(image_path)
        if role == "QUERY":
            candidate_row = None
            yaw_row = None
        else:
            candidate_row, yaw_row = frame_row(
                candidates, yaw, case["items"][0][0], frame
            )
        with Image.open(image_path) as image:
            axis.imshow(image.convert("RGB"))
        axis.set_title(label_for(frame, role, candidate_row, yaw_row), fontsize=11)
        axis.set_axis_off()
    figure.suptitle(
        f"KITTI {camera_dir.name} RGB case {case['name']}",
        fontsize=15,
        fontweight="bold",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.88))
    figure.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def wrapped_abs_degrees(angle):
    return abs((float(angle) + 180.0) % 360.0 - 180.0)


def viewpoint_category(abs_yaw):
    if abs_yaw <= 30.0:
        return "similar direction"
    if abs_yaw <= 90.0:
        return "moderately different direction"
    return "opposite / near-opposite direction"


def make_viewpoint_report(candidates, yaw, output_path):
    rows = []
    for query_frame, candidate_frame in CORRECT_PAIRS:
        candidate_row, yaw_row = frame_row(
            candidates, yaw, query_frame, candidate_frame
        )
        relative_yaw = float(yaw_row["gt_relative_yaw_deg"])
        abs_yaw = wrapped_abs_degrees(relative_yaw)
        rows.append(
            {
                "query_frame": query_frame,
                "candidate_frame": candidate_frame,
                "label": "CORRECT",
                "gt_distance": float(candidate_row["gt_distance"]),
                "gt_relative_yaw_deg": relative_yaw,
                "absolute_wrapped_yaw_difference": abs_yaw,
                "viewpoint_category": viewpoint_category(abs_yaw),
                "rgb_available": True,
            }
        )
    report = pd.DataFrame(rows)
    report.to_csv(output_path, index=False)
    return report


def write_summary(
    output_path,
    camera_dir,
    files,
    velodyne_files,
    target_frames,
    viewpoint_report,
    figure_paths,
):
    camera_stems = {path.stem for path in files}
    velodyne_stems = {path.stem for path in velodyne_files}
    aligned = camera_stems == velodyne_stems
    target_available = [
        frame for frame in target_frames if f"{frame:06d}" in camera_stems
    ]
    over_90 = int(
        (viewpoint_report["absolute_wrapped_yaw_difference"] > 90.0).sum()
    )
    resolution = "unknown"
    mode = "unknown"
    if files:
        with Image.open(files[0]) as image:
            resolution = f"{image.width}x{image.height}"
            mode = image.mode

    lines = [
        "RGB Camera Inspection Summary",
        "",
        f"camera directory: {camera_dir}",
        f"camera image count: {len(files)}",
        f"first filename: {files[0].name if files else 'N/A'}",
        f"last filename: {files[-1].name if files else 'N/A'}",
        f"image resolution: {resolution}",
        f"image mode: {mode}",
        f"Velodyne image stem alignment: {'yes' if aligned else 'no'}",
        f"target frames available: {len(target_available)}/{len(target_frames)}",
        f"all target frames available: {'yes' if len(target_available) == len(target_frames) else 'no'}",
        f"correct pairs with >90 degree relative yaw: {over_90}/{len(viewpoint_report)}",
        "",
        "Answers",
        "1. KITTI RGB data is available and frame-aligned with LiDAR: "
        + ("yes" if aligned else "no"),
        "2. All target reranking frames are available: "
        + ("yes" if len(target_available) == len(target_frames) else "no"),
        "3. Correct place matches in these three cases often have large viewpoint differences: "
        + ("yes, all three exceed 90 degrees" if over_90 == len(viewpoint_report) else "not all three"),
        "4. The forward-facing camera is a likely RGB place-recognition limitation for these cases: yes; nearby GT positions can have opposite viewing directions.",
        "5. Proceed to an RGB OpenCLIP sanity check: yes, as a diagnostic only; do not interpret it as viewpoint-invariant performance.",
        "",
        "Generated figures:",
    ]
    lines.extend(f"- {path}" for path in figure_paths)
    lines.append(f"Viewpoint CSV: {output_path.parent / 'rgb_viewpoint_report.csv'}")
    output_path.write_text("\n".join(lines) + "\n")


def main():
    args = parse_args()
    camera_dir = args.sequence_dir / args.camera
    if not camera_dir.is_dir():
        raise FileNotFoundError(f"Camera directory not found: {camera_dir}")
    files = image_files(camera_dir)
    if not files:
        raise FileNotFoundError(f"No image files found in {camera_dir}")

    candidates = pd.read_csv(args.candidate_csv)
    yaw = pd.read_csv(args.yaw_csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    figure_paths = []
    for case in CASES:
        output_path = args.output_dir / f"query{case['name']}_rgb_comparison.png"
        make_case_figure(case, camera_dir, candidates, yaw, output_path)
        figure_paths.append(output_path)

    viewpoint_path = args.output_dir / "rgb_viewpoint_report.csv"
    viewpoint_report = make_viewpoint_report(candidates, yaw, viewpoint_path)
    target_frames = sorted({frame for case in CASES for frame, _ in case["items"]})
    velodyne_files = sorted((args.sequence_dir / "velodyne").glob("*.bin"))
    summary_path = args.output_dir / "rgb_inspection_summary.txt"
    write_summary(
        summary_path,
        camera_dir,
        files,
        velodyne_files,
        target_frames,
        viewpoint_report,
        figure_paths,
    )

    camera_stems = {path.stem for path in files}
    over_90 = int(
        (viewpoint_report["absolute_wrapped_yaw_difference"] > 90.0).sum()
    )
    print("RGB INSPECTION SUMMARY")
    print(f"camera directories: {camera_dir.name}")
    print(f"image_2 exists? {'yes' if args.camera == 'image_2' else 'no'}")
    print(f"image count: {len(files)}")
    with Image.open(files[0]) as image:
        print(f"resolution: {image.width}x{image.height}")
    print(f"first filename: {files[0].name}")
    print(f"last filename: {files[-1].name}")
    print(f"frame-aligned with Velodyne? {'yes' if camera_stems == {p.stem for p in velodyne_files} else 'no'}")
    print(
        "target-frame availability: "
        + ", ".join(
            f"{frame:06d}={'yes' if f'{frame:06d}' in camera_stems else 'no'}"
            for frame in target_frames
        )
    )
    print(f"all target frames available? {'yes' if all(f'{frame:06d}' in camera_stems for frame in target_frames) else 'no'}")
    print(f"correct pairs with >90 degree relative yaw: {over_90}/{len(viewpoint_report)}")
    for path in figure_paths:
        print(f"figure: {path}")
    print(f"rgb_viewpoint_report.csv: {viewpoint_path}")
    print(f"rgb_inspection_summary.txt: {summary_path}")
    print("recommendation: PROCEED_TO_RGB_CLIP_SANITY_CHECK")


if __name__ == "__main__":
    main()
