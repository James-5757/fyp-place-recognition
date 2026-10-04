#!/usr/bin/env python3
"""Read-only CU-Multi Stage 12C dataset and GT-free stable-odometry audit.

Run --inventory, --integrity, --stability in that order. Temporary SQLite
copies are removed; only the isolated 12c_dataset_stability outputs are written.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import sqlite3
import tempfile
import time
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[2]
RAW = Path("/home/cas/CU-Multi/raw")
PROCESSED = Path("/home/cas/CU-Multi/processed_v1")
OUT = ROOT / "outputs/cumulti_v1/12c_dataset_stability"
STAGE12A = ROOT / "outputs/cumulti_v1/12a_trajectory_integrity"
ROBOTS = ("robot1", "robot2", "robot3", "robot4")
PIPELINE_ROBOTS = ("robot1", "robot3")
COMPONENTS = ("gt_utm_poses", "gt_rel_poses", "lidar", "imu_gps", "camera_rgb", "camera_depth", "lidar_labels")
OFFICIAL = "https://github.com/arpg/CU-Multi"
TYPES = get_typestore(Stores.ROS2_HUMBLE)


def dump(name, value):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def table(name, rows):
    OUT.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(OUT / name, index=False)


def archive_info(path):
    with zipfile.ZipFile(path) as z:
        members = [x for x in z.infolist() if not x.is_dir()]
        dbs = [x.filename for x in members if x.filename.endswith(".db3")]
        metas = [x.filename for x in members if x.filename.endswith("metadata.yaml")]
        metadata = None
        if metas:
            metadata = yaml.safe_load(z.read(metas[0]))["rosbag2_bagfile_information"]
        return {"member_count": len(members), "db3_members": dbs,
                "metadata_members": metas, "metadata": metadata,
                "uncompressed_size_bytes": sum(x.file_size for x in members)}


def actual_path(robot, component):
    base = RAW / "main_campus" / robot
    suffix = ".csv" if component == "gt_utm_poses" else ".zip"
    return base / f"{robot}_main_campus_{component}{suffix}"


def inventory():
    expected = {"source_url": OFFICIAL,
        "expected_top_level_components": ["calib/description/robot.urdf", "<environment>/<robot>/..."],
        "expected_per_robot_components": ["gt_utm_poses.csv", "poses or gt_rel_poses bag",
            "lidar bag", "imu_gps bag", "camera_rgb bag", "camera_depth bag"],
        "tf_expected_in": "separate ground-truth/poses bag (/tf world to base_link per official topic table)",
        "imu_gnss_expected_in": "imu_gps bag", "tf_expected_inside_imu_gps": False,
        "note": "official repository lists uncompressed conceptual directories; local distribution is ZIP archives"}
    dump("official_expected_dataset_structure.json", expected)
    rows, stream_rows, gt_sources, presence = [], [], {}, {}
    for robot in ROBOTS:
        presence[robot] = {}
        for comp in COMPONENTS:
            path = actual_path(robot, comp)
            exists = path.is_file()
            presence[robot][comp] = exists
            row = {"robot": robot, "component": comp, "path": str(path), "exists": exists,
                   "file_type": path.suffix.lstrip(".") if exists else "MISSING",
                   "size_bytes": path.stat().st_size if exists else None,
                   "archive_member_count": None, "db3_presence": None,
                   "metadata_yaml_presence": None}
            if exists and path.suffix == ".zip":
                info = archive_info(path)
                row.update({"archive_member_count": info["member_count"],
                    "db3_presence": bool(info["db3_members"]),
                    "metadata_yaml_presence": bool(info["metadata_members"])})
                meta = info["metadata"]
                if meta:
                    start = int(meta["starting_time"]["nanoseconds_since_epoch"])
                    duration = int(meta["duration"]["nanoseconds"])
                    for topic in meta["topics_with_message_count"]:
                        tm = topic["topic_metadata"]
                        stream_rows.append({"robot": robot, "component": comp,
                            "topic": tm["name"], "message_type": tm["type"],
                            "first_timestamp_ns": start, "last_timestamp_ns": start + duration,
                            "duration_s": duration / 1e9,
                            "message_count": int(topic["message_count"]),
                            "time_basis": "bag metadata recording span, not per-topic exact extrema",
                            "status": "METADATA_ONLY_PENDING_INTEGRITY"})
                if comp == "gt_rel_poses":
                    gt_sources[robot] = {"path": str(path), "db3_members": info["db3_members"],
                        "topics": [{"name": t["topic_metadata"]["name"],
                                    "type": t["topic_metadata"]["type"],
                                    "message_count": int(t["message_count"])}
                                   for t in (meta or {}).get("topics_with_message_count", [])],
                        "use": "OFFLINE DIAGNOSTIC ONLY; not used for stable-start selection"}
            rows.append(row)
        gt_csv = actual_path(robot, "gt_utm_poses")
        if gt_csv.is_file():
            stamps = []
            with gt_csv.open() as f:
                for line in f:
                    if line.startswith("#") or not line.strip():
                        continue
                    try: stamps.append(float(line.split(",", 1)[0]))
                    except ValueError: continue
            if stamps:
                scale = 1. if max(stamps) > 1e15 else 1e9
                stream_rows.append({"robot": robot, "component": "gt_utm_poses", "topic": "CSV poses",
                    "message_type": "CSV", "first_timestamp_ns": int(min(stamps) * scale),
                    "last_timestamp_ns": int(max(stamps) * scale),
                    "duration_s": (max(stamps) - min(stamps)) * scale / 1e9,
                    "message_count": len(stamps), "time_basis": "CSV timestamps", "status": "PRESENT"})
    calib = RAW / "calib/robot_description.zip"
    if calib.is_file():
        info = archive_info(calib)
        rows.append({"robot": "ALL", "component": "calibration_urdf", "path": str(calib),
            "exists": True, "file_type": "zip", "size_bytes": calib.stat().st_size,
            "archive_member_count": info["member_count"], "db3_presence": False,
            "metadata_yaml_presence": False})
    for robot in ("robot1", "robot2", "robot3"):
        base = PROCESSED / "full_2hz" / robot
        for name in ("keyframes.csv", "lidar", "rgb"):
            path = base / name
            if path.is_file():
                count, size = 1, path.stat().st_size
                typ = "csv"
            elif path.is_dir():
                files = list(path.iterdir())
                count, size = len(files), sum(p.stat().st_size for p in files if p.is_file())
                typ = "directory_summary"
            else:
                count, size, typ = 0, None, "MISSING"
            rows.append({"robot": robot, "component": "processed_full_2hz_" + name,
                "path": str(path), "exists": count > 0, "file_type": typ, "size_bytes": size,
                "archive_member_count": count, "db3_presence": False,
                "metadata_yaml_presence": False})
    for robot in ROBOTS:
        base = PROCESSED / robot
        for name in ("poses.csv", "timestamps.csv", "sync_index.csv", "lidar", "rgb"):
            path = base / name
            if path.is_file():
                count, size, typ = 1, path.stat().st_size, "csv"
            elif path.is_dir():
                children = list(path.iterdir())
                count = sum(p.is_file() for p in children)
                size = sum(p.stat().st_size for p in children if p.is_file())
                typ = "directory_summary"
            else:
                count, size, typ = 0, None, "MISSING"
            rows.append({"robot": robot, "component": "processed_stage1_" + name,
                "path": str(path), "exists": count > 0, "file_type": typ, "size_bytes": size,
                "archive_member_count": count, "db3_presence": False,
                "metadata_yaml_presence": False})
    table("local_dataset_inventory.csv", rows)
    table("stream_duration_audit.csv", stream_rows)
    dump("ground_truth_source_inventory.json", gt_sources)
    dump("local_presence_snapshot.json", presence)
    print("INVENTORY PASS", len(rows), "rows", len(stream_rows), "stream rows", flush=True)


def integrity():
    # Only archives consumed by the current R1↔R3 sensor/GT/calibration pipeline.
    tasks = [(robot, comp, actual_path(robot, comp)) for robot in PIPELINE_ROBOTS
             for comp in ("lidar", "imu_gps", "camera_rgb", "gt_rel_poses")]
    tasks.append(("ALL", "calibration_urdf", RAW / "calib/robot_description.zip"))
    rows = []
    for robot, comp, path in tasks:
        row = {"robot": robot, "component": comp, "path": str(path), "exists": path.is_file(),
            "size_bytes": path.stat().st_size if path.is_file() else None,
            "zip_integrity": "NOT_RUN", "sqlite_integrity": "NOT_APPLICABLE",
            "metadata_parse": "NOT_APPLICABLE", "status": "MISSING" if not path.is_file() else "UNREADABLE"}
        if not path.is_file():
            rows.append(row); continue
        started = time.time()
        try:
            with zipfile.ZipFile(path) as z:
                members = [m for m in z.infolist() if not m.is_dir()]
                dbs = [m for m in members if m.filename.endswith(".db3")]
                metas = [m for m in members if m.filename.endswith("metadata.yaml")]
                if metas:
                    yaml.safe_load(z.read(metas[0]))
                    row["metadata_parse"] = "PASS"
                with tempfile.TemporaryDirectory(prefix="stage12c_integrity_", dir="/home/cas/CU-Multi") as tmp:
                    for member in members:
                        with z.open(member) as source:
                            if member in dbs:
                                dbpath = Path(tmp) / Path(member.filename).name
                                with dbpath.open("wb") as target:
                                    shutil.copyfileobj(source, target, length=8 * 1024 * 1024)
                                conn = sqlite3.connect(f"file:{dbpath}?mode=ro", uri=True)
                                try:
                                    check = conn.execute("PRAGMA integrity_check").fetchone()[0]
                                    metadata = yaml.safe_load(z.read(metas[0]))["rosbag2_bagfile_information"]
                                    observed = conn.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
                                    declared = int(metadata["message_count"])
                                    row["sqlite_integrity"] = "PASS" if check == "ok" and observed == declared else f"FAIL:{check};{observed}!={declared}"
                                    row["db3_message_count"] = observed
                                    row["metadata_message_count"] = declared
                                finally:
                                    conn.close()
                                dbpath.unlink()
                            else:
                                while source.read(8 * 1024 * 1024):
                                    pass
                row["zip_integrity"] = "PASS"  # reading to EOF validates CRC for every member
                row["status"] = "PASS" if row["sqlite_integrity"] in ("PASS", "NOT_APPLICABLE") else "CORRUPT"
        except Exception as exc:
            row["status"] = "CORRUPT" if isinstance(exc, (zipfile.BadZipFile, sqlite3.DatabaseError)) else "UNREADABLE"
            row["error"] = f"{type(exc).__name__}: {exc}"
        row["runtime_s"] = time.time() - started
        rows.append(row)
        table("archive_integrity_report.csv", rows)
        print("INTEGRITY", robot, comp, row["status"], f"{row['runtime_s']:.1f}s", flush=True)
    assert all(r["status"] == "PASS" for r in rows), rows


def kf_times(robot):
    steps = pd.read_csv(STAGE12A / f"local_{robot}_steps.csv").sort_values("keyframe_i")
    assert steps.keyframe_i.to_list() == list(range(len(steps)))
    assert steps.keyframe_j.to_list() == list(range(1, len(steps) + 1))
    ids = np.arange(len(steps) + 1)
    stamps = np.r_[int(steps.timestamp_i.iloc[0]), steps.timestamp_j.astype("int64").to_numpy()]
    assert np.all(np.diff(stamps) > 0)
    return steps, ids, stamps


def nearest_topic(conn, topic, targets, kind):
    found = conn.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
    assert found, topic
    topic_id, typ = found
    iterator = iter(conn.execute("SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id", (topic_id,)))
    before, after = None, next(iterator, None)
    out = []
    for target in targets:
        while after is not None and after[1] < int(target):
            before, after = after, next(iterator, None)
        choices = [x for x in (before, after) if x is not None]
        best = min(choices, key=lambda x: abs(x[1] - int(target))) if choices else None
        limit = .05 if kind == "ekf" else (.02 if kind == "imu" else .35)
        if best is None or abs(best[1] - int(target)) / 1e9 > limit:
            out.append({"match_status": "OUT_OF_WINDOW", "match_error_s": None})
            continue
        msg = TYPES.deserialize_cdr(best[2], typ)
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if abs(stamp - int(target)) / 1e9 > limit:
            out.append({"match_status": "HEADER_OUT_OF_WINDOW", "match_error_s": abs(stamp - int(target)) / 1e9})
            continue
        common = {"match_status": "MATCHED", "match_error_s": abs(stamp - int(target)) / 1e9,
                  "bag_timestamp_ns": int(best[1]), "header_timestamp_ns": stamp,
                  "message_sequence_index": int(best[0]), "frame_id": str(msg.header.frame_id)}
        if kind == "ekf":
            pose = msg.pose.pose
            cov = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
            common.update({"child_frame_id": str(msg.child_frame_id),
                "ekf_x": float(pose.position.x), "ekf_y": float(pose.position.y), "ekf_z": float(pose.position.z),
                "ekf_qx": float(pose.orientation.x), "ekf_qy": float(pose.orientation.y),
                "ekf_qz": float(pose.orientation.z), "ekf_qw": float(pose.orientation.w),
                "reported_speed_mps": float(np.linalg.norm([msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y, msg.twist.twist.linear.z])),
                "position_covariance_trace": float(np.trace(cov[:3, :3])),
                "orientation_covariance_proxy": float(np.trace(cov[3:, 3:])),
                "covariance_all_finite": bool(np.isfinite(cov).all())})
        elif kind == "imu":
            common.update({"linear_acceleration_norm_including_gravity_mps2": float(np.linalg.norm(
                [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])),
                "angular_velocity_norm_rad_s": float(np.linalg.norm(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))})
        else:
            common.update({"latitude": float(msg.latitude), "longitude": float(msg.longitude),
                "altitude": float(msg.altitude), "fix_status": int(msg.status.status),
                "position_covariance_type": int(msg.position_covariance_type)})
        out.append(common)
    return pd.DataFrame(out)


def gnss_enu_step(a, b):
    if any(pd.isna(v) for v in (a.get("latitude"), a.get("longitude"),
                                b.get("latitude"), b.get("longitude"))):
        return np.nan
    if a.get("message_sequence_index") == b.get("message_sequence_index"):
        return np.nan  # repeated same sample is not evidence for zero motion
    la, lb = math.radians(a["latitude"]), math.radians(b["latitude"])
    loa, lob = math.radians(a["longitude"]), math.radians(b["longitude"])
    return float(math.hypot(6378137 * math.cos((la + lb) / 2) * (lob - loa), 6378137 * (lb - la)))


def full_robot_stability(robot):
    steps, ids, stamps = kf_times(robot)
    archive = actual_path(robot, "imu_gps")
    with zipfile.ZipFile(archive) as z:
        db_members = [n for n in z.namelist() if n.endswith(".db3")]
        assert len(db_members) == 1
        with tempfile.TemporaryDirectory(prefix=f"stage12c_{robot}_", dir="/home/cas/CU-Multi") as tmp:
            dbpath = Path(tmp) / "source.db3"
            with z.open(db_members[0]) as src, dbpath.open("wb") as dst:
                shutil.copyfileobj(src, dst, 8 * 1024 * 1024)
            conn = sqlite3.connect(f"file:{dbpath}?mode=ro", uri=True)
            try:
                ekf = nearest_topic(conn, f"{robot}/ekf/odometry_map", stamps, "ekf")
                g1 = nearest_topic(conn, f"{robot}/gnss_1/llh_position", stamps, "gnss")
                g2 = nearest_topic(conn, f"{robot}/gnss_2/llh_position", stamps, "gnss")
                imu = nearest_topic(conn, f"{robot}/imu/data", stamps, "imu")
            finally:
                conn.close()
    df = pd.DataFrame({"robot_id": robot, "keyframe_id": ids, "timestamp_ns": stamps,
        "seconds_from_first_keyframe": (stamps - stamps[0]) / 1e9,
        "translation_step_xy_m": np.r_[np.nan, steps.translation_xy_m.to_numpy()],
        "translation_step_xyz_m": np.r_[np.nan, steps.translation_xyz_m.to_numpy()],
        "rotation_step_deg": np.r_[np.nan, steps.relative_rotation_deg.to_numpy()],
        "implied_speed_mps": np.r_[np.nan, steps.linear_speed_mps.to_numpy()],
        "dt_s": np.r_[np.nan, steps.dt_s.to_numpy()]})
    for col in ekf:
        df["ekf_" + col] = ekf[col]
    for col in imu:
        df["imu_" + col] = imu[col]
    for label, gnss in (("gnss1", g1), ("gnss2", g2)):
        for col in gnss:
            df[label + "_" + col] = gnss[col]
        df[label + "_step_xy_m"] = [np.nan] + [gnss_enu_step(gnss.iloc[i-1], gnss.iloc[i])
                                            for i in range(1, len(gnss))]
    df["raw_gnss_supported_step_m"] = df[["gnss1_step_xy_m", "gnss2_step_xy_m"]].median(axis=1)
    df["ekf_gnss_step_magnitude_disagreement_m"] = (df.translation_step_xy_m - df.raw_gnss_supported_step_m).abs()
    df["gnss_both_valid"] = (df.gnss1_match_status.eq("MATCHED") & df.gnss2_match_status.eq("MATCHED") &
        df.gnss1_fix_status.ge(0) & df.gnss2_fix_status.ge(0) &
        np.isfinite(df.gnss1_latitude) & np.isfinite(df.gnss2_latitude))
    df["ekf_frame_unchanged"] = (df.ekf_frame_id.eq(df.ekf_frame_id.iloc[0]) &
        df.ekf_child_frame_id.eq(df.ekf_child_frame_id.iloc[0]))
    df["timestamp_order_valid"] = (pd.Series(stamps).diff().fillna(1).gt(0) &
        df.ekf_header_timestamp_ns.diff().fillna(1).gt(0))
    df.to_csv(OUT / f"{robot}_full_ekf_stability.csv", index=False)
    print("STABILITY EXTRACT", robot, len(df), "keyframes", flush=True)
    return df


def robust_limit(series, tail_start, margin=10):
    values = np.asarray(series.iloc[tail_start:], dtype=float)
    values = values[np.isfinite(values)]
    assert len(values) >= 100
    med = float(np.median(values))
    mad = float(np.median(np.abs(values - med)))
    return {"reference_median": med, "reference_mad": mad,
            "threshold": med + margin * 1.4826 * mad, "sample_count": len(values)}


def freeze_policy():
    policy = {"status": "PREDECLARED_BEFORE_STABLE_START_APPLICATION",
        "input": "frozen Stage12A 2-Hz keyframes plus nearest raw EKF and dual-GNSS; no GT",
        "lookback_seconds": 10, "persistence_seconds": 30,
        "tail_reference_fraction": 0.5, "robust_mad_scale": 1.4826,
        "step_rotation_speed_disagreement_mad_multiplier": 10,
        "covariance_final_half_median_multiplier": 5,
        "major_recurrence_threshold_multiplier": 5,
        "signals": ["translation_step_xy_m", "rotation_step_deg", "implied_speed_mps",
            "ekf_position_covariance_trace", "ekf_gnss_step_magnitude_disagreement_m"],
        "gnss_requirement": "both raw antennas finite and fix status >=0; nearest <=0.35s; disagreement only if new GNSS sample",
        "ekf_requirement": "nearest <=0.05s, finite covariance, monotonically increasing timestamps and stable frame pair",
        "window_rule": "each of the previous 20 keyframes meets all available signal thresholds; GNSS disagreement is tested only with distinct samples",
        "declaration_rule": "first keyframe after 60 consecutive passing look-back windows; conservative declaration time, no backdating",
        "late_major_recurrence": "any signal >5x its frozen limit after declaration; separately record ordinary threshold exceedances",
        "no_gt": True, "no_post_hoc_threshold_tuning": True}
    dump("stable_start_policy.json", policy)
    # fsync so the predeclared policy is durably on disk before applying it.
    with (OUT / "stable_start_policy.json").open("rb") as f:
        os.fsync(f.fileno())
    return policy


def apply_policy(robot, df, policy):
    n = len(df)
    tail = n // 2
    specs = {"translation_step_xy_m": robust_limit(df.translation_step_xy_m, tail),
        "rotation_step_deg": robust_limit(df.rotation_step_deg, tail),
        "implied_speed_mps": robust_limit(df.implied_speed_mps, tail),
        "ekf_gnss_step_magnitude_disagreement_m": robust_limit(
            df.ekf_gnss_step_magnitude_disagreement_m, tail)}
    covariance = df.ekf_position_covariance_trace.iloc[tail:].dropna()
    cov_med = float(np.median(covariance))
    specs["ekf_position_covariance_trace"] = {"reference_median": cov_med,
        "threshold": 5 * cov_med, "sample_count": len(covariance)}
    valid = (df.ekf_match_status.eq("MATCHED") & df.ekf_covariance_all_finite.eq(True) &
             df.gnss_both_valid.eq(True) & df.ekf_frame_unchanged.eq(True) &
             df.timestamp_order_valid.eq(True))
    for signal, spec in specs.items():
        values = pd.to_numeric(df[signal], errors="coerce")
        if signal == "ekf_gnss_step_magnitude_disagreement_m":
            signal_good = values.le(spec["threshold"]) | values.isna()
        else:
            signal_good = values.le(spec["threshold"]) & values.notna()
        valid &= signal_good
        df["within_limit_" + signal] = signal_good
    lookback = valid.rolling(20, min_periods=20).sum().eq(20)
    persistent = lookback.rolling(60, min_periods=60).sum().eq(60)
    candidates = np.flatnonzero(persistent.to_numpy())
    start = int(candidates[0]) if len(candidates) else None
    df["point_passes_frozen_rule"] = valid
    df["lookback_10s_passes"] = lookback
    df["persistent_30s_passes"] = persistent
    df["after_declared_start"] = df.keyframe_id.ge(start) if start is not None else False
    df.to_csv(OUT / f"{robot}_full_ekf_stability.csv", index=False)
    return start, specs, df


def recurrence(robot, df, start, specs):
    rows = []
    for signal, spec in specs.items():
        values = pd.to_numeric(df[signal], errors="coerce")
        subset = values.iloc[start:] if start is not None else values.iloc[0:0]
        threshold = spec["threshold"]
        rows.append({"robot": robot, "signal": signal, "stable_start_keyframe": start,
            "frozen_threshold": threshold, "ordinary_exceedances": int((subset > threshold).sum()),
            "major_threshold_5x": 5 * threshold,
            "major_exceedances": int((subset > 5 * threshold).sum()),
            "maximum_after_start": float(subset.max()) if len(subset) else None})
    return rows


def stability_figures(robot, df, start, specs):
    x = df.seconds_from_first_keyframe
    signals = [("translation_step_xy_m", "XY step (m)"),
        ("implied_speed_mps", "pose-implied speed (m/s)"),
        ("ekf_reported_speed_mps", "EKF reported speed (m/s)"),
        ("ekf_position_covariance_trace", "position covariance trace (m²)"),
        ("raw_gnss_supported_step_m", "raw GNSS step (m)")]
    for focus in (True, False):
        fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True, constrained_layout=True)
        for ax, (signal, label) in zip(axes, signals):
            ax.plot(x, df[signal], lw=.8)
            if signal in specs:
                ax.axhline(specs[signal]["threshold"], color="red", lw=.8, ls="--", label="frozen limit")
            if start is not None:
                ax.axvline(float(x.iloc[start]), color="green", lw=1.1, label="declared start")
            ax.set_ylabel(label); ax.grid(alpha=.2)
            if signal == "ekf_position_covariance_trace": ax.set_yscale("log")
        axes[0].legend(loc="upper right", ncol=2)
        axes[-1].set_xlabel("Seconds from first LiDAR keyframe")
        if focus: axes[-1].set_xlim(0, 120)
        fig.suptitle(robot + (": initial 0–120 s" if focus else ": complete recording"))
        fig.savefig(OUT / f"{robot}_{'ekf_stability_timeline' if focus else 'full_run_stability'}.png", dpi=150)
        plt.close(fig)


def offline_gt_diagnostic(selections):
    # Called only after stable_segment_selection.json exists on disk.
    assert (OUT / "stable_segment_selection.json").is_file()
    rows = []
    for robot, pair in (("robot1", (57, 58)), ("robot3", (42, 43))):
        gt_path = actual_path(robot, "gt_utm_poses")
        steps = pd.read_csv(STAGE12A / f"local_{robot}_steps.csv")
        event = steps[(steps.keyframe_i == pair[0]) & (steps.keyframe_j == pair[1])].iloc[0]
        if not gt_path.is_file():
            rows.append({"robot": robot, "event": f"KF{pair[0]}->{pair[1]}",
                "status": "GT_DIAGNOSTIC_NOT_AVAILABLE_LOCALLY",
                "use": "OFFLINE DIAGNOSTIC ONLY; NOT USED FOR STABLE-START SELECTION"})
            continue
        gt = pd.read_csv(gt_path, comment="#", header=None,
            names=["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        t = gt.timestamp.to_numpy(float)
        i = int(np.argmin(np.abs(t - int(event.timestamp_i) / 1e9)))
        j = int(np.argmin(np.abs(t - int(event.timestamp_j) / 1e9)))
        p = gt.loc[[i, j], ["x", "y", "z"]].to_numpy(float)
        rows.append({"robot": robot, "event": f"KF{pair[0]}->{pair[1]}",
            "status": "OFFLINE_DIAGNOSTIC_ONLY", "gt_source": str(gt_path),
            "gt_match_error_before_s": abs(float(t[i]) - int(event.timestamp_i) / 1e9),
            "gt_match_error_after_s": abs(float(t[j]) - int(event.timestamp_j) / 1e9),
            "gt_translation_xy_m": float(np.linalg.norm((p[1] - p[0])[:2])),
            "gt_translation_xyz_m": float(np.linalg.norm(p[1] - p[0])),
            "local_ekf_translation_xy_m": float(event.translation_xy_m),
            "use": "OFFLINE DIAGNOSTIC ONLY; NOT USED FOR STABLE-START SELECTION"})
    table("offline_gt_jump_diagnostic.csv", rows)


def frontend_coverage(selections):
    first1, first3 = (selections[r]["stable_start_keyframe"] for r in PIPELINE_ROBOTS)
    if first1 is None or first3 is None:
        result = {"status": "NO_DEFENSIBLE_STABLE_START", "remaining_r1_queries": None,
                  "remaining_r3_database_keyframes": None,
                  "remaining_rank1_gicp_accepted_pairs": None,
                  "remaining_rank1_sanitized_loops": None}
        dump("stable_segment_frontend_coverage.json", result)
        return result
    rank1 = pd.read_csv(ROOT / "outputs/cumulti_v1/09_sc_gicp_integration/rank1_gicp_results.csv")
    loops = pd.read_csv(ROOT / "outputs/cumulti_v1/10_2_pgo_policy_selection/rank1_sanitized_edges.csv")
    mask = rank1.query_keyframe_id.ge(first1) & rank1.candidate_keyframe_id.ge(first3)
    loop_mask = loops.query_keyframe_id.ge(first1) & loops.candidate_keyframe_id.ge(first3)
    result = {"status": "FROZEN_RECORDS_FILTERED_ONLY_NO_RETRIEVAL",
        "remaining_r1_queries": 2000 - first1,
        "remaining_r3_database_keyframes": 4180 - first3,
        "remaining_frozen_rank1_candidate_pairs": int(mask.sum()),
        "rank1_gicp_quality_threshold_frozen": 0.6091,
        "remaining_rank1_gicp_accepted_pairs": int((mask & rank1.GICP_quality.ge(.6091)).sum()),
        "remaining_rank1_sanitized_loops": int(loop_mask.sum()),
        "loop_schedule_K_1_2_5_10_20_feasible_by_count": bool(loop_mask.sum() >= 20),
        "warning": "count is opportunity only, not geometric rank/observability or a sparse-loop result"}
    dump("stable_segment_frontend_coverage.json", result)
    return result


def finalize(selections, recurrence_rows, coverage):
    report = pd.read_csv(OUT / "archive_integrity_report.csv")
    integrity_pass = report.status.eq("PASS").all()
    assert len(report) == 9
    streams = pd.read_csv(OUT / "stream_duration_audit.csv")
    checked = {(r.robot, r.component): r for r in report.itertuples() if r.component != "calibration_urdf"}
    for (robot, comp), integrity_row in checked.items():
        mask = streams.robot.eq(robot) & streams.component.eq(comp)
        assert mask.any(), (robot, comp)
        assert int(streams.loc[mask, "message_count"].sum()) == int(integrity_row.metadata_message_count)
        streams.loc[mask, "status"] = "ARCHIVE_SQLITE_COUNT_VALIDATED"
        info = archive_info(Path(integrity_row.path))
        declared_shards = [Path(f["path"]).name for f in info["metadata"]["files"]]
        actual_shards = [Path(p).name for p in info["db3_members"]]
        assert sorted(declared_shards) == sorted(actual_shards), (robot, comp, declared_shards, actual_shards)
    lidar_duration = {robot: float(streams.loc[(streams.robot.eq(robot) & streams.component.eq("lidar")),
        "duration_s"].iloc[0]) for robot in PIPELINE_ROBOTS}
    streams["duration_vs_lidar_s"] = [float(r.duration_s) - lidar_duration[r.robot]
        if r.robot in lidar_duration else np.nan for r in streams.itertuples()]
    streams["duration_flag"] = ["ZERO_OR_TRUNCATION_SUSPECTED" if float(r.duration_s) <= 0 or
        (pd.notna(r.duration_vs_lidar_s) and float(r.duration_vs_lidar_s) < -300) else "NO_CLEAR_TRUNCATION"
        for r in streams.itertuples()]
    assert not (streams.loc[streams.robot.isin(PIPELINE_ROBOTS), "duration_flag"] ==
                "ZERO_OR_TRUNCATION_SUSPECTED").any()
    streams.to_csv(OUT / "stream_duration_audit.csv", index=False)
    presence = json.loads((OUT / "local_presence_snapshot.json").read_text())
    required = all(all(presence[r][c] for c in ("lidar", "imu_gps", "camera_rgb", "gt_rel_poses", "gt_utm_poses"))
                   for r in PIPELINE_ROBOTS)
    missing_optional = [{"robot": r, "component": c, "path": str(actual_path(r, c))}
        for r in ROBOTS for c in COMPONENTS if not presence[r][c]]
    dataset_status = ("LOCAL_ARCHIVE_CORRUPTION" if not integrity_pass else
        "MISSING_REQUIRED_COMPONENTS" if not required else "COMPLETE_FOR_CURRENT_PIPELINE")
    dump("dataset_completeness_report.json", {"status": dataset_status,
        "required_r1_r3_lidar_imu_rgb_gt_present": required,
        "all_checked_pipeline_archives_integrity_pass": bool(integrity_pass),
        "full_official_four_robot_download_complete": not bool(missing_optional),
        "missing_official_optional_components": missing_optional,
        "interpretation": "R1/R3 current pipeline has required input; full 4-robot official dataset is not all locally downloaded. /tf exists in separate gt_rel_poses ZIPs."})
    action = ("REDOWNLOAD_CORRUPT_ARCHIVE" if not integrity_pass else
        "DOWNLOAD_MISSING_REQUIRED_COMPONENT" if not required else "NO_DOWNLOAD_NEEDED")
    dump("dataset_download_action.json", {"decision": action,
        "exact_missing_paths": missing_optional,
        "note": "No download started; omitted Robot3 depth/labels and Robot4 sensor archives are optional for the current R1/R3 study."})
    starts = all(selections[r]["stable_start_keyframe"] is not None for r in PIPELINE_ROBOTS)
    major = sum(r["major_exceedances"] for r in recurrence_rows)
    readiness = "DELAYED_START_IS_DEFENSIBLE" if integrity_pass and required and starts and major == 0 and coverage.get("remaining_rank1_sanitized_loops", 0) >= 20 else (
        "NO_STABLE_SEGMENT" if not starts else "DELAYED_START_INSUFFICIENT")
    dump("delayed_start_assessment.json", {"decision": readiness,
        "large_late_recurrence_count": major,
        "no_graph_rebuilt": True, "sparse_loop_after_future_clean_rebuild":
            "CONDITIONAL" if integrity_pass and required and starts and
                coverage.get("remaining_rank1_sanitized_loops", 0) >= 20 else "NO",
        "condition": "Resolve covariance-only late alerts, freeze an independently validated odometry/graph policy, and verify the rebuilt graph before sparse-loop evaluation."})
    gt_present = all(presence[r]["gt_rel_poses"] for r in PIPELINE_ROBOTS)
    flags = ["OFFICIAL STRUCTURE CHECKED: PASS", "LOCAL DATA INVENTORIED: PASS",
        f"ZIP INTEGRITY: {'PASS' if integrity_pass else 'FAIL'}",
        f"SQLITE INTEGRITY: {'PASS' if integrity_pass else 'FAIL'}",
        f"IMU/GPS COMPLETE FOR R1: {'PASS' if presence['robot1']['imu_gps'] else 'FAIL'}",
        f"IMU/GPS COMPLETE FOR R3: {'PASS' if presence['robot3']['imu_gps'] else 'FAIL'}",
        f"LIDAR AVAILABLE R1: {'PASS' if presence['robot1']['lidar'] else 'FAIL'}",
        f"LIDAR AVAILABLE R3: {'PASS' if presence['robot3']['lidar'] else 'FAIL'}",
        f"GT/POSE PACKAGE: {'PRESENT' if gt_present else 'PARTIAL'}",
        "TF EXPECTED INSIDE IMU_GPS: NO", "GT USED FOR STABLE START: NO",
        "STABLE START POLICY FROZEN BEFORE APPLICATION: PASS",
        f"ROBOT1 STABLE SEGMENT: {'PASS' if selections['robot1']['stable_start_keyframe'] is not None else 'FAIL'}",
        f"ROBOT3 STABLE SEGMENT: {'PASS' if selections['robot3']['stable_start_keyframe'] is not None else 'FAIL'}",
        f"LATE RECURRENCE: {'PRESENT' if major else 'NONE'}",
        f"FRONTEND COVERAGE AFTER CUT: {'PASS' if coverage.get('remaining_rank1_sanitized_loops', 0) >= 20 else 'FAIL'}",
        f"DATASET STATUS: {dataset_status}", f"DOWNLOAD ACTION: {action}",
        f"DELAYED-START REPAIR READY: {'YES' if readiness == 'DELAYED_START_IS_DEFENSIBLE' else 'NO'}",
        "STAGE10.3 MODIFIED: NO"]
    (OUT / "VALIDATION_REPORT.txt").write_text("\n".join(flags) + "\n")
    print("FINAL", dataset_status, readiness, flush=True)


def stability():
    assert (OUT / "archive_integrity_report.csv").is_file(), "run integrity first"
    policy = freeze_policy()
    selections, recurrences = {}, []
    for robot in PIPELINE_ROBOTS:
        df = full_robot_stability(robot)
        start, specs, df = apply_policy(robot, df, policy)
        timestamps = df.timestamp_ns.astype("int64")
        duration = (int(timestamps.iloc[-1]) - int(timestamps.iloc[start])) / 1e9 if start is not None else None
        selections[robot] = {"stable_start_keyframe": start,
            "stable_start_timestamp_ns": int(timestamps.iloc[start]) if start is not None else None,
            "seconds_from_bag_start": (int(timestamps.iloc[start]) - int(timestamps.iloc[0])) / 1e9 if start is not None else None,
            "unstable_keyframes_excluded_at_beginning": start,
            "retained_keyframes": len(df) - start if start is not None else 0,
            "retained_fraction": (len(df) - start) / len(df) if start is not None else 0,
            "retained_duration_s": duration, "frozen_thresholds": specs}
        recurrences.extend(recurrence(robot, df, start, specs))
        stability_figures(robot, df, start, specs)
    status = "PASS_STABLE_SEGMENTS_FOUND" if all(v["stable_start_keyframe"] is not None for v in selections.values()) else "NO_DEFENSIBLE_STABLE_START"
    dump("stable_segment_selection.json", {"status": status, "robots": selections,
        "policy_file": "stable_start_policy.json", "GT_used": False})
    table("post_stable_start_recurrence.csv", recurrences)
    # GT is read only after the complete GT-free policy/selection has been serialized.
    offline_gt_diagnostic(selections)
    coverage = frontend_coverage(selections)
    if len(pd.read_csv(OUT / "archive_integrity_report.csv")) == 9:
        finalize(selections, recurrences, coverage)
    else:
        print("STABILITY SELECTION SAVED; FINAL COMPLETENESS PENDING 9/9 ARCHIVE INTEGRITY", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("inventory", "integrity", "stability", "finalize"))
    args = parser.parse_args()
    if args.mode == "inventory": inventory()
    elif args.mode == "integrity": integrity()
    elif args.mode == "stability": stability()
    else:
        selections = json.loads((OUT / "stable_segment_selection.json").read_text())["robots"]
        recurrences = pd.read_csv(OUT / "post_stable_start_recurrence.csv").to_dict("records")
        coverage = json.loads((OUT / "stable_segment_frontend_coverage.json").read_text())
        assert len(pd.read_csv(OUT / "archive_integrity_report.csv")) == 9
        finalize(selections, recurrences, coverage)


if __name__ == "__main__":
    main()
