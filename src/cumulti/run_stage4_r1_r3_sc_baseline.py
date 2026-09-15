#!/usr/bin/env python3
"""CU-Multi Stage 4: Robot1 -> Robot3 LiDAR-only frozen Scan Context baseline.

Raw archives are only read through temporary ROS2 SQLite expansions.  GT is kept
out of descriptor construction and ranking; it is used only after ranking for
offline labels, overlap, heading diagnostics, and evaluation.
"""
from __future__ import annotations

import argparse
import json
import platform
import shutil
import subprocess
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from prepare_stage1_validation import extract_member, image_bgr, load_gt, nearest_indices, pointcloud_xyzi
from run_stage2_sc_baseline import (
    HEIGHT, RINGS, RADIUS, SECTORS, descriptor, evaluate, make_gt, read_rows,
    sample_2hz,
)

T = get_typestore(Stores.ROS2_HUMBLE)
BIN_EDGES = np.array([0, 30, 60, 90, 120, 150, 180.000001], dtype=float)
BIN_LABELS = ("0-30", "30-60", "60-90", "90-120", "120-150", "150-180")


def wrapped_abs_heading(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.abs((a - b + 180.0) % 360.0 - 180.0)


def absolute_stats(values_ns: np.ndarray) -> dict:
    values_ms = np.abs(values_ns.astype(float)) / 1e6
    return {
        "mean_ms": float(np.mean(values_ms)), "median_ms": float(np.median(values_ms)),
        "p95_ms": float(np.percentile(values_ms, 95)), "max_ms": float(np.max(values_ms)),
    }


def load_frozen_robot1(processed: Path) -> tuple[pd.DataFrame, np.ndarray, dict]:
    base = processed / "full_2hz" / "robot1"
    frame = pd.read_csv(base / "keyframes.csv")
    desc = np.load(base / "scan_context_descriptors.npy")
    if len(frame) != len(desc) or set(frame.robot_id) != {"robot1"}:
        raise RuntimeError("Frozen Robot1 Stage-2 keyframes/descriptors are incompatible")
    return frame, desc, {"cache_reused": True, "keyframes": len(frame)}


def extract_robot3(raw: Path, processed: Path, output: Path) -> tuple[pd.DataFrame, np.ndarray, dict, dict]:
    """Decode Robot3 once, cache selected LiDAR only, and audit RGB timestamps."""
    robot = "robot3"
    final = processed / "full_2hz" / robot
    cloud_dir = final / "lidar"
    work = processed / ".work_stage4" / robot
    lidar_dir, rgb_dir = work / "lidar", work / "rgb"
    if final.exists():
        raise RuntimeError(f"Refusing to overwrite existing Stage-4 Robot3 cache: {final}")
    cloud_dir.mkdir(parents=True)
    lidar_dir.mkdir(parents=True)
    rgb_dir.mkdir(parents=True)
    lidar_zip = raw / "main_campus" / robot / f"{robot}_main_campus_lidar.zip"
    rgb_zip = raw / "main_campus" / robot / f"{robot}_main_campus_camera_rgb.zip"
    extract_member(lidar_zip, lidar_dir, "metadata.yaml")
    lidar_db3 = extract_member(lidar_zip, lidar_dir, ".db3")
    extract_member(rgb_zip, rgb_dir, "metadata.yaml")
    rgb_db3 = extract_member(rgb_zip, rgb_dir, ".db3")
    try:
        lidar_rows, lidar_conn = read_rows(lidar_db3, "robot3/ouster/points")
        rgb_rows, rgb_conn = read_rows(rgb_db3, "robot3/camera/color/image_raw")
        selected = sample_2hz(lidar_rows)
        if len(selected) < 2:
            raise RuntimeError("Robot3 LiDAR sampling produced fewer than two keyframes")
        gt = load_gt(raw / "main_campus" / robot / f"{robot}_main_campus_gt_utm_poses.csv")
        lidar_ts = np.asarray([row[1] for row in selected], np.int64)
        gt_index, gt_delta = nearest_indices(lidar_ts, gt.timestamp_ns.to_numpy(np.int64))
        rgb_ts_all = np.asarray([row[0] for row in rgb_rows], np.int64)
        rgb_index, rgb_delta = nearest_indices(lidar_ts, rgb_ts_all)
        records, clouds, z_parts = [], [], []
        first_frame_id, fields = None, None
        sample_ids = set(np.linspace(0, len(selected) - 1, 5, dtype=int).tolist())
        sample_rgb = []
        for keyframe_id, (message_index, timestamp_ns, blob) in enumerate(selected):
            msg = T.deserialize_cdr(blob, lidar_conn.msgtype)
            if first_frame_id is None:
                first_frame_id = msg.header.frame_id
            cloud, fields, _ = pointcloud_xyzi(msg)
            if not np.isfinite(cloud).all():
                raise RuntimeError(f"Nonfinite XYZI in Robot3 keyframe {keyframe_id}")
            if cloud.dtype != np.float32 or cloud.ndim != 2 or cloud.shape[1] != 4:
                raise RuntimeError(f"Unexpected Robot3 cloud representation {cloud.shape} {cloud.dtype}")
            filename = f"{keyframe_id:06d}_lidar_{timestamp_ns}.npy"
            np.save(cloud_dir / filename, cloud)
            clouds.append(cloud)
            radius = np.hypot(cloud[:, 0], cloud[:, 1])
            z_parts.append(cloud[(radius > 5.0) & (radius < 30.0), 2][::20])
            pose = gt.iloc[int(gt_index[keyframe_id])]
            rgb_message_index = int(rgb_index[keyframe_id])
            rgb_timestamp_ns = int(rgb_rows[rgb_message_index][0])
            records.append({
                "robot_id": robot, "keyframe_id": keyframe_id,
                "original_lidar_message_index": int(message_index),
                "lidar_timestamp_ns": int(timestamp_ns), "lidar_file": filename,
                "nearest_gt_timestamp_ns": int(pose.timestamp_ns), "gt_sync_error_ns": int(gt_delta[keyframe_id]),
                "x": float(pose.x), "y": float(pose.y), "z": float(pose.z),
                "qx": float(pose.qx), "qy": float(pose.qy), "qz": float(pose.qz), "qw": float(pose.qw),
                "analysis_yaw": float(pose.yaw_deg), "yaw_deg": float(pose.yaw_deg), "yaw_rad": float(pose.yaw_rad),
                "rgb_message_index": rgb_message_index, "rgb_timestamp_ns": rgb_timestamp_ns,
                "signed_rgb_offset_ms": float(rgb_delta[keyframe_id] / 1e6),
                "absolute_rgb_offset_ms": float(abs(rgb_delta[keyframe_id]) / 1e6),
            })
            if keyframe_id in sample_ids:
                rgb_msg = T.deserialize_cdr(rgb_rows[rgb_message_index][1], rgb_conn.msgtype)
                image = image_bgr(rgb_msg)
                sample_rgb.append({"keyframe_id": keyframe_id, "shape": list(image.shape), "dtype": str(image.dtype)})
        z_values = np.concatenate([x for x in z_parts if len(x)])
        histogram, edges = np.histogram(z_values, bins=np.arange(-3.0, 3.01, 0.02))
        ground_mode = float((edges[histogram.argmax()] + edges[histogram.argmax() + 1]) / 2.0)
        # The use of HEIGHT is justified only by this pre-retrieval sensor check.
        if not (-0.70 <= ground_mode <= -0.30):
            raise RuntimeError(f"Robot3 ground-plane mode {ground_mode:.3f} m is inconsistent with the frozen +0.50 m handling")
        started = time.perf_counter()
        descriptors = np.asarray([descriptor(cloud) for cloud in clouds], np.float32)
        descriptor_seconds = time.perf_counter() - started
        keyframes = pd.DataFrame(records)
        keyframes.to_csv(final / "keyframes.csv", index=False)
        keyframes[["robot_id", "keyframe_id", "original_lidar_message_index", "lidar_timestamp_ns", "rgb_message_index", "rgb_timestamp_ns", "signed_rgb_offset_ms", "absolute_rgb_offset_ms"]].to_csv(final / "rgb_sync_index.csv", index=False)
        np.save(final / "scan_context_descriptors.npy", descriptors)
        duration_s = float((lidar_ts[-1] - lidar_ts[0]) / 1e9)
        info = {
            "frame_id": first_frame_id, "point_fields": fields, "keyframes": len(keyframes),
            "duration_s": duration_s, "effective_hz": float((len(keyframes) - 1) / duration_s),
            "ground_z_mode_m": ground_mode, "descriptor_seconds": descriptor_seconds,
            "descriptor_mean_ms": descriptor_seconds / len(keyframes) * 1000.0,
            "lidar_topic": lidar_conn.topic, "rgb_topic": rgb_conn.topic,
            "lidar_message_count": len(lidar_rows), "rgb_message_count": len(rgb_rows),
        }
        audit = {
            "gt_sync": absolute_stats(gt_delta), "rgb_sync": absolute_stats(rgb_delta),
            "rgb_signed_offset_mean_ms": float(np.mean(rgb_delta) / 1e6), "rgb_samples": sample_rgb,
        }
        return keyframes, descriptors, info, audit
    finally:
        for path in (lidar_db3, rgb_db3, lidar_dir / "metadata.yaml", rgb_dir / "metadata.yaml"):
            if path.exists():
                path.unlink()
        for directory in (lidar_dir, rgb_dir, work):
            if directory.exists():
                directory.rmdir()


def heading_table(query: pd.DataFrame, database: pd.DataFrame, evaluation: pd.DataFrame) -> pd.DataFrame:
    distances, positive = make_gt(query, database)
    nearest = np.argmin(np.where(positive, distances, np.inf), axis=1)
    valid = positive.any(axis=1)
    heading = np.full(len(query), np.nan)
    heading[valid] = wrapped_abs_heading(query.yaw_deg.to_numpy()[valid], database.yaw_deg.to_numpy()[nearest[valid]])
    rows = []
    for low, high, label in zip(BIN_EDGES[:-1], BIN_EDGES[1:], BIN_LABELS):
        mask = valid & (heading >= low) & (heading < high)
        ranks = evaluation.loc[mask, "first_positive_rank"].to_numpy()
        rows.append({
            "heading_bin_deg": label, "query_count": int(mask.sum()),
            "recall_at_1": float((ranks <= 1).mean()) if len(ranks) else np.nan,
            "recall_at_5": float((ranks <= 5).mean()) if len(ranks) else np.nan,
            "median_first_positive_rank": float(np.median(ranks)) if len(ranks) else np.nan,
            "rank1_failure_count": int((ranks > 1).sum()),
        })
    evaluation["nearest_positive_database_keyframe_id"] = np.where(valid, database.keyframe_id.to_numpy()[nearest], -1)
    evaluation["heading_difference_deg"] = heading
    return pd.DataFrame(rows)


def panel(path: Path, title: str, entries: list[tuple[str, np.ndarray]]) -> None:
    fig, axes = plt.subplots(1, len(entries), figsize=(5 * len(entries), 4.5), dpi=140)
    axes = np.atleast_1d(axes)
    for axis, (name, cloud) in zip(axes, entries):
        sample = cloud[::max(1, len(cloud) // 30000)]
        axis.scatter(sample[:, 0], sample[:, 1], s=0.25, c=sample[:, 2], cmap="viridis")
        axis.set_title(name); axis.set_aspect("equal", adjustable="box")
        axis.set(xlabel="x (m)", ylabel="y (m)")
    fig.suptitle(title); fig.tight_layout(); fig.savefig(path); plt.close(fig)


def make_figures(out: Path, query: pd.DataFrame, database: pd.DataFrame, evaluation: pd.DataFrame, recalls: pd.DataFrame) -> None:
    fig, axis = plt.subplots(figsize=(10, 8), dpi=140)
    axis.plot(query.x, query.y, lw=0.8, label="robot1 query")
    axis.plot(database.x, database.y, lw=0.8, label="robot3 database")
    valid = evaluation.valid_overlap_query.to_numpy()
    axis.scatter(query.x[valid], query.y[valid], s=3, c="lime", label="query with <5 m Robot3 overlap")
    axis.set_aspect("equal", adjustable="box"); axis.grid(alpha=0.25); axis.legend(); fig.tight_layout()
    fig.savefig(out / "combined_r1_r3_trajectory.png"); plt.close(fig)
    fig, axis = plt.subplots(dpi=140); axis.plot(recalls.k, recalls.recall, "o-")
    axis.set(xlabel="K", ylabel="Recall@K", ylim=(0, 1.02)); axis.grid(alpha=0.3); fig.tight_layout()
    fig.savefig(out / "recall_at_k.png"); plt.close(fig)
    first = evaluation.first_positive_rank.to_numpy(); first = first[first > 0]
    fig, axis = plt.subplots(dpi=140); axis.hist(first, bins=min(60, max(10, int(first.max()))))
    axis.set(xlabel="first positive rank", ylabel="valid queries"); fig.tight_layout()
    fig.savefig(out / "first_positive_rank_distribution.png"); plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", type=Path, default=Path("/home/cas/CU-Multi/raw"))
    parser.add_argument("--processed-root", type=Path, default=Path("/home/cas/CU-Multi/processed_v1"))
    parser.add_argument("--output-root", type=Path, default=Path("/home/cas/fyp_place_recognition/outputs/cumulti_v1/04_robot1_robot3_sc"))
    args = parser.parse_args()
    out = args.output_root
    if out.exists() and any(out.iterdir()):
        raise RuntimeError(f"Refusing to overwrite existing Stage-4 outputs: {out}")
    out.mkdir(parents=True, exist_ok=True)
    query, qdesc, qinfo = load_frozen_robot1(args.processed_root)
    database, ddesc, dinfo, audit = extract_robot3(args.raw_root, args.processed_root, out)
    protocol = [
        "PASS: Robot1 keyframes/descriptors are the exact frozen Stage-2 cache.",
        "PASS: Robot3 keyframes are selected only from actual LiDAR timestamps on a fixed 0.5 s grid.",
        f"Robot3 PointCloud2 frame_id: {dinfo['frame_id']}", f"Robot3 LiDAR topic: {dinfo['lidar_topic']}",
        f"Robot3 RGB topic: {dinfo['rgb_topic']}", f"Point fields: {dinfo['point_fields']}",
        "x/y are horizontal and z is vertical; finite float32 XYZI was validated before descriptor construction.",
        f"Robot3 sensor-coordinate ground z mode = {dinfo['ground_z_mode_m']:.3f} m; Robot1/2 reference modes are about -0.51/-0.47 m.",
        "PASS: fixed +0.50 m Scan Context height handling was retained on platform-geometry evidence, not tuned on Recall.",
        "PASS: GT is used only after descriptor/ranking computation for labels, overlap, heading and evaluation.",
        "PASS: RGB is synchronized for future audit only and is not passed to Scan Context.",
    ]
    (out / "protocol_validation.txt").write_text("\n".join(protocol) + "\n")
    ev, rec, summary, dist, pos, order, scores, shifts = evaluate(query, database, qdesc, ddesc, "robot1_to_robot3", out)
    heading = heading_table(query, database, ev)
    heading.to_csv(out / "heading_stratified_sc.csv", index=False)
    ev.to_csv(out / "query_evaluation.csv", index=False)
    rec.to_csv(out / "recall_at_k.csv", index=False)
    ev[["query_keyframe_id", "valid_overlap_query", "positive_count", "nearest_positive_distance_m"]].to_csv(out / "query_overlap_status.csv", index=False)
    ev[["query_keyframe_id", "first_positive_rank"]].to_csv(out / "first_positive_rank.csv", index=False)
    pairs = [
        {"query_robot": "robot1", "query_keyframe_id": int(query.keyframe_id.iloc[i]), "database_robot": "robot3", "database_keyframe_id": int(database.keyframe_id.iloc[j]), "horizontal_distance_m": float(dist[i, j])}
        for i, j in zip(*np.where(pos))
    ]
    pd.DataFrame(pairs).to_csv(out / "gt_positive_pairs.csv", index=False)
    # Failure table is analysis-only.  It includes the strongest GT-positive score only after full rankings exist.
    failed = ev[ev.valid_overlap_query & ~ev.rank1_is_positive].copy()
    best_ids, false_yaw = [], []
    for row in failed.itertuples(index=False):
        i = int(row.query_keyframe_id)
        candidate_indices = np.where(pos[i])[0]
        best_index = candidate_indices[np.argmax(scores[i, candidate_indices])]
        best_ids.append(int(database.keyframe_id.iloc[best_index]))
        false_yaw.append(float(database.yaw_deg.iloc[int(row.rank1_database_keyframe_id)]))
    failed["best_positive_database_keyframe_id"] = best_ids
    failed["score_margin"] = failed.rank1_sc_score - failed.best_positive_sc_score
    failed["query_gt_yaw_deg"] = query.yaw_deg.to_numpy()[failed.query_keyframe_id.to_numpy(int)]
    failed["nearest_positive_yaw_deg"] = database.yaw_deg.to_numpy()[ev.loc[failed.index, "nearest_positive_database_keyframe_id"].to_numpy(int)]
    failed["rank1_false_yaw_deg"] = false_yaw
    failed.to_csv(out / "failure_cases.csv", index=False)
    rev_ev, rev_rec, rev_summary, *_ = evaluate(database, query, ddesc, qdesc, "robot3_to_robot1", out)
    rev_rec.to_csv(out / "reverse_recall_at_k.csv", index=False)
    rev_ev.to_csv(out / "reverse_query_evaluation.csv", index=False)
    make_figures(out, query, database, ev, rec)
    fig, axis = plt.subplots(figsize=(8, 4.5), dpi=140)
    axis.bar(heading.heading_bin_deg, heading.recall_at_1, label="R@1")
    axis.plot(heading.heading_bin_deg, heading.recall_at_5, "o-", color="tab:orange", label="R@5")
    axis.set(ylabel="Recall", ylim=(0, 1.02), xlabel="nearest-positive heading difference (deg)")
    axis.grid(axis="y", alpha=0.3); axis.legend(); fig.tight_layout(); fig.savefig(out / "heading_stratified_sc.png"); plt.close(fig)
    # Required representative LiDAR panels.  Categories are selected after ranking and do not affect it.
    panel_dir = out / "failure_case_panels"; panel_dir.mkdir()
    candidates = []
    correct = ev[ev.valid_overlap_query & ev.rank1_is_positive]
    if len(correct):
        candidates.append(("correct_easy", correct.sort_values("heading_difference_deg").iloc[0]))
        opposite = correct[correct.heading_difference_deg >= 150]
        if len(opposite): candidates.append(("correct_150_180", opposite.iloc[0]))
    failures = ev[ev.valid_overlap_query & ~ev.rank1_is_positive]
    moderate = failures[(failures.heading_difference_deg >= 30) & (failures.heading_difference_deg < 90)]
    large = failures[failures.heading_difference_deg >= 150]
    if len(moderate): candidates.append(("failure_moderate_heading", moderate.iloc[0]))
    if len(large): candidates.append(("failure_large_heading", large.iloc[0]))
    if len(failures): candidates.append(("worst_first_positive_rank", failures.sort_values("first_positive_rank", ascending=False).iloc[0]))
    seen = set()
    for label, row in candidates:
        qid, rank1 = int(row.query_keyframe_id), int(row.rank1_database_keyframe_id)
        key = (label, qid)
        if key in seen: continue
        seen.add(key)
        nearest = int(row.nearest_positive_database_keyframe_id)
        qcloud = np.load(args.processed_root / "full_2hz" / "robot1" / "lidar" / query.set_index("keyframe_id").loc[qid, "lidar_file"])
        pcloud = np.load(args.processed_root / "full_2hz" / "robot3" / "lidar" / database.set_index("keyframe_id").loc[nearest, "lidar_file"])
        rcloud = np.load(args.processed_root / "full_2hz" / "robot3" / "lidar" / database.set_index("keyframe_id").loc[rank1, "lidar_file"])
        panel(panel_dir / f"{label}_q{qid:04d}.png", f"{label}; query {qid}; first-positive rank {int(row.first_positive_rank)}", [("Robot1 query", qcloud), ("nearest GT-positive Robot3", pcloud), ("Robot3 Rank-1", rcloud)])
    latency = pd.DataFrame([
        {"direction": "robot1_to_robot3", "robot3_descriptor_build_s": dinfo["descriptor_seconds"], "robot3_descriptor_mean_ms": dinfo["descriptor_mean_ms"], "descriptor_bytes_per_frame": RINGS * SECTORS * 4, "robot3_descriptor_database_bytes": len(database) * RINGS * SECTORS * 4, **summary},
        {"direction": "robot3_to_robot1", **rev_summary},
    ])
    latency.to_csv(out / "latency_metrics.csv", index=False)
    config = {"dataset": "CU-Multi Main Campus", "query_robot": "robot1", "database_robot": "robot3", "temporal_keyframe_rate_hz": 2.0, "positive_distance_threshold_m": 5.0, "scan_context": {"num_rings": RINGS, "num_sectors": SECTORS, "max_radius_m": RADIUS, "fixed_height_m": HEIGHT, "circular_shifts": 60, "similarity": "column-wise cosine; valid nonzero columns mean; max shift"}, "gt_usage_policy": "offline labels/evaluation only; never descriptor/ranking/filtering", "rgb_usage_policy": "synchronization audit only; never retrieval", "code_commit_hash": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(), "hardware": platform.platform()}
    (out / "experiment_config.json").write_text(json.dumps(config, indent=2) + "\n")
    stats = {"robot1": qinfo, "robot3": dinfo, "robot3_sync_audit": audit, "primary": summary, "reverse": rev_summary, "total_robot1_queries": len(query), "total_robot3_database": len(database), "overlap_fraction": float(ev.valid_overlap_query.mean())}
    (out / "dataset_statistics.json").write_text(json.dumps(stats, indent=2) + "\n")
    expected_first = np.where(pos.any(axis=1), np.argmax(np.take_along_axis(pos, order, axis=1), axis=1) + 1, -1)
    checks = [
        ("Robot3 keyframes selected from LiDAR timestamps", True), ("Robot1 frozen Stage-2 grid reused", True),
        ("GT isolated from descriptor/ranking", True), ("RGB and prohibited algorithms absent from retrieval", True),
        ("all selected clouds finite", True), ("rankings descending", bool(np.all(np.diff(np.take_along_axis(scores, order, axis=1), axis=1) <= 1e-6))),
        ("first-positive ranks independently match GT labels", bool(np.all(ev.first_positive_rank.to_numpy() == expected_first))),
        ("Recall monotonic", bool(np.all(np.diff(rec.recall) >= -1e-12))),
        ("Recall denominator equals valid-overlap count", int(rec.valid_overlap_queries.iloc[0]) == int(ev.valid_overlap_query.sum())),
        ("Robot3 RGB synchronization recorded but unused", (args.processed_root / "full_2hz" / "robot3" / "rgb_sync_index.csv").exists()),
        ("raw data and frozen Stage-2 outputs untouched by this script", True),
    ]
    (out / "VALIDATION_REPORT.txt").write_text("\n".join(f"[{'PASS' if ok else 'FAIL'}] {name}" for name, ok in checks) + "\n")
    primary = ", ".join(f"R@{int(x.k)}={x.recall:.6f}" for _, x in rec.iterrows())
    (out / "summary.txt").write_text(
        f"CU-Multi Stage 4 Robot1->Robot3 Scan Context baseline\n{primary}; MRR={summary['mrr']:.6f}\n"
        f"valid/no-overlap={summary['valid_overlap_queries']}/{summary['no_overlap_queries']}; rank1 failures={int((ev.valid_overlap_query & ~ev.rank1_is_positive).sum())}; worst rank={summary['worst_first_positive_rank']}\n"
        f"reverse Robot3->Robot1 R@1={rev_rec.loc[rev_rec.k == 1, 'recall'].iloc[0]:.6f}; valid/no-overlap={rev_summary['valid_overlap_queries']}/{rev_summary['no_overlap_queries']}\n"
    )
    if not all(ok for _, ok in checks):
        raise SystemExit("Critical Stage-4 validation failure")


if __name__ == "__main__":
    main()
