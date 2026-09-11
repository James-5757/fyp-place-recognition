#!/usr/bin/env python3
"""GT-only four-robot CU-Multi Main Campus pair-selection analysis.

This script intentionally performs no sensor archive access and no retrieval.
It uses ground truth only after timestamp-based 2 Hz sampling to characterise
which robot pairs are suitable for subsequent place-recognition evaluation.
"""
from __future__ import annotations

import argparse
import itertools
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROBOTS = ("robot1", "robot2", "robot3", "robot4")
BIN_EDGES = np.array([0, 30, 60, 90, 120, 150, 180.000001], dtype=float)
BIN_LABELS = ("0-30", "30-60", "60-90", "90-120", "120-150", "150-180")


def wrapped_abs_heading_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.abs((a - b + 180.0) % 360.0 - 180.0)


def yaw_from_quaternion_deg(df: pd.DataFrame) -> np.ndarray:
    # Standard intrinsic ZYX yaw from xyzw quaternion; this is analysis only.
    qx, qy, qz, qw = (df[c].to_numpy(float) for c in ("qx", "qy", "qz", "qw"))
    return np.degrees(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2)))


def load_gt(path: Path) -> pd.DataFrame:
    fields = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    # CU-Multi documents its field header in a comment, so the first un-commented
    # record is data rather than a CSV header row.
    df = pd.read_csv(path, comment="#", header=None, names=fields, skipinitialspace=True)
    timestamp_raw = pd.to_numeric(df["timestamp"], errors="raise").to_numpy(float)
    # CU-Multi GT CSV timestamps are decimal Unix seconds, unlike the existing
    # Stage-2 LiDAR keyframe index which is nanoseconds.
    if np.nanmedian(np.abs(timestamp_raw)) < 1e12:
        df["timestamp"] = np.rint(timestamp_raw * 1e9).astype("int64")
    else:
        df["timestamp"] = np.rint(timestamp_raw).astype("int64")
    df = df.sort_values("timestamp").drop_duplicates("timestamp").reset_index(drop=True)
    df["yaw_deg"] = yaw_from_quaternion_deg(df)
    return df


def grid_sample_gt(gt: pd.DataFrame, period_ns: int = 500_000_000) -> pd.DataFrame:
    """Choose the first GT sample at/after a fixed 0.5 s grid anchor."""
    ts = gt.timestamp.to_numpy(np.int64)
    targets = np.arange(ts[0], ts[-1] + 1, period_ns, dtype=np.int64)
    indices = np.searchsorted(ts, targets, side="left")
    indices = indices[indices < len(ts)]
    indices = np.unique(indices)
    sampled = gt.iloc[indices].copy().reset_index(drop=True)
    sampled["keyframe_id"] = np.arange(len(sampled), dtype=int)
    sampled["sampling_source"] = "gt_timestamp_grid"
    return sampled


def load_robot_samples(robot: str, raw: Path, processed: Path) -> pd.DataFrame:
    gt_path = raw / "main_campus" / robot / f"{robot}_main_campus_gt_utm_poses.csv"
    gt = load_gt(gt_path)
    existing = processed / "full_2hz" / robot / "keyframes.csv"
    if existing.exists():
        # Retain exact Stage-2 LiDAR timestamp sampling for robots 1/2, then
        # attach the nearest GT pose. GT is never used to select the timestamps.
        frames = pd.read_csv(existing)
        lidar_ts = frames["lidar_timestamp_ns"].to_numpy(np.int64)
        gt_ts = gt.timestamp.to_numpy(np.int64)
        right = np.searchsorted(gt_ts, lidar_ts, side="left")
        left = np.clip(right - 1, 0, len(gt_ts) - 1)
        right = np.clip(right, 0, len(gt_ts) - 1)
        choose_right = np.abs(gt_ts[right] - lidar_ts) < np.abs(gt_ts[left] - lidar_ts)
        idx = np.where(choose_right, right, left)
        out = gt.iloc[idx].copy().reset_index(drop=True)
        out["keyframe_id"] = frames["keyframe_id"].to_numpy(int)
        out["sample_timestamp_ns"] = lidar_ts
        out["gt_timestamp_ns"] = out["timestamp"].to_numpy(np.int64)
        out["sampling_source"] = "stage2_lidar_timestamp_grid_nearest_gt"
    else:
        out = grid_sample_gt(gt)
        out["sample_timestamp_ns"] = out["timestamp"].to_numpy(np.int64)
        out["gt_timestamp_ns"] = out["timestamp"].to_numpy(np.int64)
    out["robot"] = robot
    return out


def analyze_direction(query: pd.DataFrame, database: pd.DataFrame) -> tuple[dict, list[dict]]:
    qxy = query[["x", "y"]].to_numpy(float)
    dxy = database[["x", "y"]].to_numpy(float)
    distance = np.hypot(qxy[:, None, 0] - dxy[None, :, 0], qxy[:, None, 1] - dxy[None, :, 1])
    positive = distance < 5.0
    positive_count = positive.sum(axis=1)
    valid = positive_count > 0
    nearest = np.argmin(np.where(positive, distance, np.inf), axis=1)
    heading = wrapped_abs_heading_deg(
        query.yaw_deg.to_numpy(float)[valid], database.yaw_deg.to_numpy(float)[nearest[valid]]
    )
    row = {
        "query_robot": query.robot.iloc[0],
        "database_robot": database.robot.iloc[0],
        "query_keyframe_count": len(query),
        "database_keyframe_count": len(database),
        "valid_overlap_queries": int(valid.sum()),
        "no_overlap_queries": int((~valid).sum()),
        "overlap_fraction": float(valid.mean()),
        "positives_per_valid_query_median": float(np.median(positive_count[valid])) if valid.any() else np.nan,
        "positives_per_valid_query_p95": float(np.percentile(positive_count[valid], 95)) if valid.any() else np.nan,
        "heading_nearest_positive_count": int(len(heading)),
        "heading_fraction_above_90_deg": float((heading > 90.0).mean()) if len(heading) else np.nan,
        "heading_fraction_above_150_deg": float((heading > 150.0).mean()) if len(heading) else np.nan,
    }
    bins = []
    for lo, hi, label in zip(BIN_EDGES[:-1], BIN_EDGES[1:], BIN_LABELS):
        count = int(((heading >= lo) & (heading < hi)).sum())
        bins.append({
            "query_robot": row["query_robot"], "database_robot": row["database_robot"],
            "heading_bin_deg": label, "count": count,
            "fraction_of_valid_overlap_queries": float(count / len(heading)) if len(heading) else np.nan,
        })
    return row, bins


def heatmap(path: Path, matrix: pd.DataFrame, title: str, fmt: str) -> None:
    fig, ax = plt.subplots(figsize=(6.2, 5.2), dpi=160)
    values = matrix.loc[list(ROBOTS), list(ROBOTS)].to_numpy(float)
    masked = np.ma.masked_invalid(values)
    im = ax.imshow(masked, vmin=0, vmax=1, cmap="viridis")
    ax.set_xticks(range(len(ROBOTS)), ROBOTS)
    ax.set_yticks(range(len(ROBOTS)), ROBOTS)
    ax.set_xlabel("database robot")
    ax.set_ylabel("query robot")
    ax.set_title(title)
    for i in range(len(ROBOTS)):
        for j in range(len(ROBOTS)):
            if i != j and np.isfinite(values[i, j]):
                ax.text(j, i, format(values[i, j], fmt), ha="center", va="center", color="white", fontsize=9)
    fig.colorbar(im, ax=ax, fraction=0.047, pad=0.04)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def choose_recommendations(summary: pd.DataFrame) -> tuple[pd.Series, pd.Series | None, pd.Series | None]:
    # Pair score means the weaker direction; this avoids choosing an apparently
    # strong pair which is unusable in the reverse direction.
    pair_rows = []
    for a, b in itertools.combinations(ROBOTS, 2):
        ab = summary[(summary.query_robot == a) & (summary.database_robot == b)].iloc[0]
        ba = summary[(summary.query_robot == b) & (summary.database_robot == a)].iloc[0]
        pair_rows.append({
            "pair": f"{a}-{b}", "a": a, "b": b,
            "min_valid": min(ab.valid_overlap_queries, ba.valid_overlap_queries),
            "mean_overlap": (ab.overlap_fraction + ba.overlap_fraction) / 2,
            "mean_above90": (ab.heading_fraction_above_90_deg + ba.heading_fraction_above_90_deg) / 2,
            "mean_above150": (ab.heading_fraction_above_150_deg + ba.heading_fraction_above_150_deg) / 2,
        })
    pairs = pd.DataFrame(pair_rows)
    easy = pairs.sort_values(["mean_overlap", "min_valid"], ascending=False).iloc[0]
    usable = pairs[(pairs.min_valid >= 250) & (pairs.mean_overlap >= 0.15)].copy()
    usable = usable[usable.pair != easy.pair]
    hard = usable.sort_values(["mean_above90", "mean_above150", "min_valid"], ascending=False).iloc[0] if len(usable) else None
    remaining = pairs[pairs.pair != easy.pair]
    if hard is not None:
        remaining = remaining[remaining.pair != hard.pair]
    stress = remaining.sort_values(["mean_overlap", "min_valid"], ascending=True).iloc[0] if len(remaining) else None
    return easy, hard, stress


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path("/home/cas/fyp_place_recognition"))
    parser.add_argument("--raw", type=Path, default=Path("/home/cas/CU-Multi/raw"))
    parser.add_argument("--processed", type=Path, default=Path("/home/cas/CU-Multi/processed_v1"))
    args = parser.parse_args()
    out = args.repo / "outputs/cumulti_v1/03_pair_selection"
    out.mkdir(parents=True, exist_ok=True)
    samples = {robot: load_robot_samples(robot, args.raw, args.processed) for robot in ROBOTS}
    summary_rows, heading_rows = [], []
    for a, b in itertools.combinations(ROBOTS, 2):
        for query, database in ((samples[a], samples[b]), (samples[b], samples[a])):
            summary, bins = analyze_direction(query, database)
            summary_rows.append(summary)
            heading_rows.extend(bins)
    summary = pd.DataFrame(summary_rows).sort_values(["query_robot", "database_robot"])
    headings = pd.DataFrame(heading_rows).sort_values(["query_robot", "database_robot", "heading_bin_deg"])
    summary.to_csv(out / "pair_selection_summary.csv", index=False)
    headings.to_csv(out / "pair_heading_statistics.csv", index=False)
    # The requested overlap matrix remains directional: rows are queries, columns databases.
    overlap = summary[["query_robot", "database_robot", "query_keyframe_count", "database_keyframe_count", "valid_overlap_queries", "no_overlap_queries", "overlap_fraction", "positives_per_valid_query_median", "positives_per_valid_query_p95"]]
    overlap.to_csv(out / "pair_overlap_matrix.csv", index=False)
    fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
    for robot, df in samples.items():
        ax.plot(df.x, df.y, lw=0.8, label=f"{robot} ({len(df)} @ 2 Hz)")
        ax.scatter(df.x.iloc[0], df.y.iloc[0], s=25, zorder=3)
    ax.set_aspect("equal", adjustable="box")
    ax.set(xlabel="UTM x (m)", ylabel="UTM y (m)", title="CU-Multi Main Campus: 2 Hz GT trajectories")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "four_robot_trajectories.png")
    plt.close(fig)
    overlap_matrix = summary.pivot(index="query_robot", columns="database_robot", values="overlap_fraction").reindex(index=ROBOTS, columns=ROBOTS)
    heading_matrix = summary.pivot(index="query_robot", columns="database_robot", values="heading_fraction_above_90_deg").reindex(index=ROBOTS, columns=ROBOTS)
    heatmap(out / "pair_overlap_matrix.png", overlap_matrix, "Directional overlap fraction (d_xy < 5 m)", ".2f")
    heatmap(out / "pair_heading_matrix.png", heading_matrix, "Nearest-positive heading difference > 90°", ".2f")
    easy, hard, stress = choose_recommendations(summary)
    def show(x: pd.Series | None) -> str:
        if x is None:
            return "No pair met the predeclared usable threshold (>=250 valid queries in each direction and mean overlap >=0.15)."
        return (f"`{x.pair}` — weaker-direction valid queries: {int(x.min_valid)}; mean directional overlap: {x.mean_overlap:.3f}; "
                f"mean nearest-positive heading >90°: {x.mean_above90:.3f}; >150°: {x.mean_above150:.3f}.")
    sampling = "robot1/robot2 retain their exact Stage-2 LiDAR timestamp grid with nearest GT association; robot3/robot4 use first GT record at/after an anchored 0.5 s grid because no sensor data were downloaded."
    report = f"""# CU-Multi Main Campus four-robot pair recommendation

## Scope and protocol

This is GT-only offline pair selection. No LiDAR/RGB/depth/labels/ROS archive was read, extracted, or downloaded; no retrieval or registration algorithm was run.

All ground-truth CSVs use `timestamp,x,y,z,qx,qy,qz,qw`. Yaw is the standard ZYX yaw derived from each GT xyzw quaternion. {sampling}

For every ordered direction, a query is valid when at least one database sample has `d_xy < 5 m`. Heading analysis picks the *nearest* such GT-positive only after forming labels. GT positions and headings are not inputs to a retrieval method.

## Recommendations

1. **Easy pair:** {show(easy)} Chosen for substantial mutual overlap, not for its retrieval score.
2. **Hard-but-usable pair:** {show(hard)} Chosen by high viewpoint reversal while retaining a meaningful bilateral evaluation set; it is not selected merely for low overlap.
3. **Extreme stress-test pair:** {show(stress)} This is optional and should be interpreted with its lower overlap/sample support in mind.

## Output interpretation

- `pair_selection_summary.csv` has 12 directional records with all requested counts, overlap, positive-count quantiles, and heading-tail fractions.
- `pair_heading_statistics.csv` has the six requested heading bins per direction.
- `pair_overlap_matrix.csv` is a directional long-form matrix suitable for direct auditing; the corresponding PNG uses query rows and database columns.

All counts are determined from the sampled GT trajectories and therefore are useful only for offline dataset/pair selection and later evaluation stratification.
"""
    (out / "HARD_PAIR_RECOMMENDATION.md").write_text(report, encoding="utf-8")
    for robot, df in samples.items():
        df[["robot", "keyframe_id", "sample_timestamp_ns", "gt_timestamp_ns", "x", "y", "z", "yaw_deg", "sampling_source"]].to_csv(out / f"{robot}_2hz_gt_samples.csv", index=False)


if __name__ == "__main__":
    main()
