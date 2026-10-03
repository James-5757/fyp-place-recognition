#!/usr/bin/env python3
"""Stage 12A: read-only lineage audit of frozen CU-Multi trajectories.

No ground truth is loaded. All writes are confined to the new Stage-12A output.
Run from the repository root with the fyp_slam Python environment.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/cumulti"))
import run_stage10_offline_map_merge as s10

OUT = ROOT / "outputs/cumulti_v1/12a_trajectory_integrity"
PRE = ROOT / "outputs/cumulti_v1/10_2_pgo_policy_selection/single_loop_trajectory.csv"
POST = ROOT / "outputs/cumulti_v1/10_3_solver_audit/rank1_gtsam_trajectory.csv"
DEMO = ROOT / "demo/final_replay/data/replay.json"
ROBOTS = ("robot1", "robot3")
FULL = ("local", "pre_pgo", "post_pgo")
SOURCES = FULL + ("demo_local", "demo_pre_pgo", "demo_post_pgo")
METRICS = ("translation_xy_m", "translation_xyz_m", "relative_rotation_deg", "dt_s", "linear_speed_mps")
EPS = 1e-12


@dataclass
class Track:
    source: str
    robot: str
    key: np.ndarray
    ts: np.ndarray
    xyz: np.ndarray
    quat: np.ndarray
    timestamp_provenance: str = "source"
    z_rotation_provenance: str = "source"

    def index(self):
        return {int(k): i for i, k in enumerate(self.key)}


def dump(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def table(name, rows, columns=None):
    pd.DataFrame(rows, columns=columns).to_csv(OUT / name, index=False)


def make_local(robot):
    k, poses, audit = s10.aligned(robot)
    xyz = poses[:, :3, 3]
    quat = Rotation.from_matrix(poses[:, :3, :3]).as_quat()
    track = Track("local", robot, k.keyframe_id.to_numpy(np.int64),
                  k.lidar_timestamp_ns.to_numpy(np.int64), xyz, quat)
    return track, audit


def make_csv(source, path):
    df = pd.read_csv(path)
    required = {"robot_id", "keyframe_id", "timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"}
    assert required <= set(df.columns), (path, required - set(df.columns))
    tracks = {}
    for robot in ROBOTS:
        d = df[df.robot_id.eq(robot)]
        tracks[(source, robot)] = Track(source, robot, d.keyframe_id.to_numpy(np.int64),
            d.timestamp.to_numpy(np.int64), d[["tx", "ty", "tz"]].to_numpy(float),
            d[["qx", "qy", "qz", "qw"]].to_numpy(float))
    return tracks


def make_demo(data, tracks):
    result = {}
    mapping = {
        "demo_local": ("local_trajectories", "local"),
        "demo_pre_pgo": ("pre_pgo_common_frame", "pre_pgo"),
        "demo_post_pgo": ("post_pgo_global", "post_pgo"),
    }
    matches = {}
    for source, (field, parent) in mapping.items():
        for robot in ROBOTS:
            rows = data[field][robot] if source == "demo_local" else [r for r in data[field] if r["robot_id"] == robot]
            full = tracks[(parent, robot)]
            lookup = full.index()
            key = np.array([int(r["keyframe_id"]) for r in rows], dtype=np.int64)
            assert all(int(k) in lookup for k in key), (source, robot, "unknown keyframe")
            ix = np.array([lookup[int(k)] for k in key], dtype=np.int64)
            xy = np.array([[r["x"], r["y"]] for r in rows], dtype=float)
            error = np.linalg.norm(xy - full.xyz[ix, :2], axis=1)
            assert np.max(error) < 1e-6, (source, robot, "browser XY differs from frozen source", np.max(error))
            xyz = np.column_stack([xy, full.xyz[ix, 2]])
            result[(source, robot)] = Track(source, robot, key, full.ts[ix], xyz, full.quat[ix],
                "matched_full_resolution_source", "matched_full_resolution_source")
            matches[f"{source}/{robot}"] = {"row_count": len(rows), "max_xy_difference_m": float(max(error)),
                "timestamp_and_z_rotation": "matched full-resolution frozen source, absent in replay.json"}
    return result, matches


def ordering(track):
    k = track.key
    dt = np.diff(track.ts)
    dk = np.diff(k)
    counts = Counter(int(v) for v in dk)
    duplicates = int(len(k) - len(set(int(v) for v in k)))
    expected = 10 if track.source.startswith("demo_") else 1
    events = []
    for i, (step, td) in enumerate(zip(dk, dt)):
        if step != expected or td <= 0:
            events.append({"row_i": i, "keyframe_i": int(k[i]), "keyframe_j": int(k[i + 1]),
                           "keyframe_step": int(step), "timestamp_delta_ns": int(td)})
    return {"source": track.source, "robot_id": track.robot, "row_count": len(k),
        "first_keyframe": int(k[0]), "last_keyframe": int(k[-1]), "duplicate_count": duplicates,
        "timestamp_monotonic": bool(np.all(dt > 0)), "keyframe_monotonic": bool(np.all(dk > 0)),
        "keyframe_step_distribution": {str(v): int(n) for v, n in sorted(counts.items())},
        "expected_step": expected, "unexpected_ordering_events": events,
        "timestamp_provenance": track.timestamp_provenance,
        "z_rotation_provenance": track.z_rotation_provenance,
        "unexpected_robot_switch_inside_polyline": False}


def step_frame(track):
    xyz = track.xyz
    delta = np.diff(xyz, axis=0)
    xy = np.linalg.norm(delta[:, :2], axis=1)
    xyz_m = np.linalg.norm(delta, axis=1)
    dt = np.diff(track.ts.astype(np.float64)) / 1e9
    rots = Rotation.from_quat(track.quat)
    rv = (rots[:-1].inv() * rots[1:]).magnitude()
    rot_deg = np.rad2deg(rv)
    valid = dt > 0
    speed = np.divide(xyz_m, dt, out=np.full_like(dt, np.nan), where=valid)
    ang = np.divide(rot_deg, dt, out=np.full_like(dt, np.nan), where=valid)
    rows = pd.DataFrame({"source": track.source, "robot_id": track.robot,
        "keyframe_i": track.key[:-1], "keyframe_j": track.key[1:],
        "timestamp_i": track.ts[:-1], "timestamp_j": track.ts[1:], "dt_s": dt,
        "x_i": xyz[:-1, 0], "y_i": xyz[:-1, 1], "z_i": xyz[:-1, 2],
        "x_j": xyz[1:, 0], "y_j": xyz[1:, 1], "z_j": xyz[1:, 2],
        "delta_x": delta[:, 0], "delta_y": delta[:, 1], "delta_z": delta[:, 2],
        "translation_xy_m": xy, "translation_xyz_m": xyz_m,
        "relative_rotation_deg": rot_deg, "linear_speed_mps": speed,
        "angular_speed_deg_s": ang,
        "timestamp_provenance": track.timestamp_provenance,
        "z_rotation_provenance": track.z_rotation_provenance})
    assert np.isfinite(rows[["translation_xy_m", "translation_xyz_m", "relative_rotation_deg"]].to_numpy()).all()
    return rows


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    assert len(a)
    med = float(np.median(a))
    mad = float(np.median(np.abs(a - med)))
    return {"median": med, "p90": float(np.percentile(a, 90)), "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)), "p99_5": float(np.percentile(a, 99.5)),
            "max": float(np.max(a)), "MAD": mad, "count": len(a)}


def threshold(s):
    if s["MAD"] <= max(1e-9, abs(s["median"]) * 1e-9):
        return s["p99_5"], "P99_5_FALLBACK_ZERO_MAD"
    return s["median"] + 10 * s["MAD"], "MEDIAN_PLUS_10_MAD"


def metric_audit(steps):
    stat_rows, candidate_rows, thresholds = [], [], {}
    for (source, robot), df in steps.items():
        for metric in METRICS:
            s = stats(df[metric])
            limit, method = threshold(s)
            stat_rows.append({"source": source, "robot_id": robot, "metric": metric,
                **s, "candidate_threshold": limit, "threshold_method": method})
            thresholds[(source, robot, metric)] = limit
            vals = df[metric].to_numpy(float)
            ranks = np.argsort(np.nan_to_num(vals, nan=-np.inf))[::-1]
            top = set(int(i) for i in ranks[:20])
            flags = set(int(i) for i in np.flatnonzero(vals > limit))
            if metric not in ("translation_xy_m", "relative_rotation_deg"):
                continue
            for i in sorted(top | flags):
                r = df.iloc[i]
                candidate_rows.append({"source": source, "robot_id": robot, "metric": metric,
                    "keyframe_i": int(r.keyframe_i), "keyframe_j": int(r.keyframe_j),
                    "metric_value": float(vals[i]), "top20_rank": int(np.where(ranks == i)[0][0] + 1) if i in top else None,
                    "robust_outlier_flag": i in flags, "threshold": limit, "threshold_method": method,
                    "translation_xy_m": float(r.translation_xy_m), "relative_rotation_deg": float(r.relative_rotation_deg),
                    "dt_s": float(r.dt_s), "implied_speed_mps": float(r.linear_speed_mps)})
    return stat_rows, candidate_rows, thresholds


def lookup_frames(steps):
    return {name: {(int(r.keyframe_i), int(r.keyframe_j)): r for r in df.itertuples()}
            for name, df in steps.items()}


def direct(track, a, b):
    ix = track.index()
    if a not in ix or b not in ix:
        return None
    i, j = ix[a], ix[b]
    p, q = track.xyz[[i, j]]
    rot = Rotation.from_quat(track.quat[[i, j]])
    return {"xy": float(np.linalg.norm((q - p)[:2])), "xyz": float(np.linalg.norm(q - p)),
            "rotation": float(np.rad2deg((rot[0].inv() * rot[1]).magnitude())),
            "dt": float((track.ts[j] - track.ts[i]) / 1e9), "index_i": i, "index_j": j}


def interval_max_step(df, a, b, field):
    sub = df[df.keyframe_i.ge(a) & df.keyframe_j.le(b)]
    return float(sub[field].max()) if len(sub) else None


def provisional(row, nominal_dt, rot_limit, metric):
    if row.dt_s <= 0:
        return "ORDERING_ANOMALY"
    # More than two nominal intervals means at least one 2-Hz sample is absent.
    if row.dt_s > 2 * nominal_dt:
        return "LARGE_DISPLACEMENT_LARGE_TIME_GAP"
    if metric == "relative_rotation_deg" and row.relative_rotation_deg > rot_limit:
        return "ROTATION_JUMP"
    if row.translation_xy_m > 0:
        return "LARGE_DISPLACEMENT_NORMAL_DT"
    return "NEEDS_SOURCE_COMPARISON"


def lineage(tracks, steps, candidates, limits):
    keys = {(r["robot_id"], r["keyframe_i"], r["keyframe_j"]) for r in candidates}
    # Include every browser segment so the longest visible lines can always be traced.
    for source in ("demo_local", "demo_pre_pgo", "demo_post_pgo"):
        for robot in ROBOTS:
            df = steps[(source, robot)]
            keys.update((robot, int(a), int(b)) for a, b in zip(df.keyframe_i, df.keyframe_j))
    out = []
    for robot, a, b in sorted(keys):
        m = {source: direct(tracks[(source, robot)], a, b) for source in SOURCES}
        if all(x is None for x in m.values()):
            continue
        dt = next(x["dt"] for x in m.values() if x is not None)
        full_peak = {source: interval_max_step(steps[(source, robot)], a, b, "translation_xy_m")
                     for source in FULL}
        flags = {source: full_peak[source] is not None and
                 full_peak[source] > limits[(source, robot, "translation_xy_m")]
                 for source in FULL}
        if flags["local"]:
            first, classification = "LOCAL", "UPSTREAM_LOCAL_ODOMETRY"
        elif flags["pre_pgo"]:
            first, classification = "PRE_PGO", "COMMON_FRAME_OR_PRE_PGO_ASSEMBLY"
        elif flags["post_pgo"]:
            first, classification = "POST_PGO", "PGO_OUTPUT_ANOMALY"
        else:
            source = "demo_post_pgo" if m["demo_post_pgo"] else "demo_pre_pgo"
            dm, fm = m[source], m[source.removeprefix("demo_")]
            if dm and fm and abs(dm["xy"] - fm["xy"]) > 1e-6:
                first, classification = "DEMO", "VISUALIZATION_OR_ASSET_GENERATION"
            elif (interval_max_step(steps[("local", robot)], a, b, "dt_s") >
                  2 * float(steps[("local", robot)].dt_s.median())):
                first, classification = "NONE", "RECORDING_TIME_GAP_OR_SPARSE_INTERVAL"
            else:
                first, classification = "NONE", "LIKELY_REAL_MOTION_OR_UNRESOLVED"
        out.append({"robot_id": robot, "keyframe_i": a, "keyframe_j": b,
            "local_translation_m": m["local"]["xy"] if m["local"] else None,
            "pre_pgo_translation_m": m["pre_pgo"]["xy"] if m["pre_pgo"] else None,
            "post_pgo_translation_m": m["post_pgo"]["xy"] if m["post_pgo"] else None,
            "demo_translation_m": m["demo_post_pgo"]["xy"] if m["demo_post_pgo"] else None,
            "demo_local_translation_m": m["demo_local"]["xy"] if m["demo_local"] else None,
            "demo_pre_pgo_translation_m": m["demo_pre_pgo"]["xy"] if m["demo_pre_pgo"] else None,
            "local_rotation_deg": m["local"]["rotation"] if m["local"] else None,
            "pre_pgo_rotation_deg": m["pre_pgo"]["rotation"] if m["pre_pgo"] else None,
            "post_pgo_rotation_deg": m["post_pgo"]["rotation"] if m["post_pgo"] else None,
            "dt_s": dt, "first_source_where_jump_appears": first, "classification": classification,
            "local_interval_peak_step_m": full_peak["local"],
            "pre_pgo_interval_peak_step_m": full_peak["pre_pgo"],
            "post_pgo_interval_peak_step_m": full_peak["post_pgo"]})
    return pd.DataFrame(out)


def pgo_changes(steps, limits):
    rows = []
    for robot in ROBOTS:
        pre = steps[("pre_pgo", robot)]
        post = steps[("post_pgo", robot)]
        assert np.array_equal(pre[["keyframe_i", "keyframe_j"]], post[["keyframe_i", "keyframe_j"]])
        for i in range(len(pre)):
            a, b = pre.iloc[i], post.iloc[i]
            p, q = float(a.translation_xy_m), float(b.translation_xy_m)
            pr, qr = float(a.relative_rotation_deg), float(b.relative_rotation_deg)
            rows.append({"robot_id": robot, "keyframe_i": int(a.keyframe_i), "keyframe_j": int(a.keyframe_j),
                "pre_translation_m": p, "post_translation_m": q,
                "translation_ratio": q / max(p, EPS), "translation_difference_m": q - p,
                "pre_rotation_deg": pr, "post_rotation_deg": qr,
                "rotation_ratio": qr / max(pr, EPS), "rotation_difference_deg": qr - pr,
                "new_translation_outlier": p <= limits[("pre_pgo", robot, "translation_xy_m")] and
                    q > limits[("post_pgo", robot, "translation_xy_m")],
                "new_rotation_outlier": pr <= limits[("pre_pgo", robot, "relative_rotation_deg")] and
                    qr > limits[("post_pgo", robot, "relative_rotation_deg")]})
    df = pd.DataFrame(rows)
    df["top20_translation_increase"] = False
    df["top20_rotation_increase"] = False
    for robot in ROBOTS:
        idx = df.index[df.robot_id.eq(robot)]
        for col, marker in (("translation_difference_m", "top20_translation_increase"),
                            ("rotation_difference_deg", "top20_rotation_increase")):
            df.loc[df.loc[idx, col].nlargest(20).index, marker] = True
    return df


def point_chord_deviation(points):
    a, b = points[0, :2], points[-1, :2]
    vec = b - a
    den = float(vec @ vec)
    if den < EPS:
        return float(np.max(np.linalg.norm(points[:, :2] - a, axis=1)))
    t = np.clip((points[:, :2] - a) @ vec / den, 0, 1)
    projection = a[None, :] + t[:, None] * vec[None, :]
    return float(np.max(np.linalg.norm(points[:, :2] - projection, axis=1)))


def demo_segments(tracks, steps, limits):
    rows = []
    for source, parent in (("demo_local", "local"), ("demo_pre_pgo", "pre_pgo"),
                           ("demo_post_pgo", "post_pgo")):
        for robot in ROBOTS:
            browser = tracks[(source, robot)]
            full = tracks[(parent, robot)]
            ix = full.index()
            browser_ix = browser.index()
            for a, b in zip(browser.key[:-1], browser.key[1:]):
                a, b = int(a), int(b)
                i, j = ix[a], ix[b]
                path = full.xyz[i:j + 1]
                chord = float(np.linalg.norm(browser.xyz[browser_ix[b], :2] - browser.xyz[browser_ix[a], :2]))
                path_len = float(np.sum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)))
                peak = float(np.max(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)))
                raw_local_peak = interval_max_step(steps[("local", robot)], a, b, "translation_xy_m")
                intermediate_dt_max = float(np.max(np.diff(full.ts[i:j + 1].astype(np.float64)) / 1e9))
                rows.append({"demo_source": source, "full_source": parent, "robot_id": robot,
                    "keyframe_i": a, "keyframe_j": b, "keyframe_stride": b - a,
                    "direct_chord_length_m": chord, "full_resolution_path_length_m": path_len,
                    "chord_to_path_ratio": chord / max(path_len, EPS),
                    "max_intermediate_chord_deviation_m": point_chord_deviation(path),
                    "max_intermediate_step_m": peak, "max_intermediate_local_step_m": raw_local_peak,
                    "max_intermediate_dt_s": intermediate_dt_max,
                    "dt_s": float((full.ts[j] - full.ts[i]) / 1e9),
                    "implied_chord_speed_mps": chord / max(float((full.ts[j] - full.ts[i]) / 1e9), EPS)})
    df = pd.DataFrame(rows)
    for (source, robot), ix in df.groupby(["demo_source", "robot_id"]).groups.items():
        part = df.loc[ix]
        deviation_limit = float(np.percentile(part.max_intermediate_chord_deviation_m, 99.5))
        parent = str(part.full_source.iloc[0])
        for idx in ix:
            r = df.loc[idx]
            chord_shape = bool(r.max_intermediate_chord_deviation_m > deviation_limit)
            if r.max_intermediate_local_step_m > limits[("local", robot, "translation_xy_m")] and parent != "local":
                label = "UPSTREAM_JUMP_VISIBLE_IN_DEMO"
            elif r.max_intermediate_step_m > limits[(parent, robot, "translation_xy_m")]:
                label = "TRUE_TRAJECTORY_JUMP"
            elif chord_shape:
                label = "DOWNSAMPLING_CHORD_ARTIFACT"
            elif r.max_intermediate_dt_s > 2 * float(steps[(parent, robot)].dt_s.median()):
                label = "UNRESOLVED"
            else:
                label = "NORMAL_LONG_SEGMENT"
            df.loc[idx, "classification"] = label
            df.loc[idx, "chord_shape_artifact_flag"] = chord_shape
            df.loc[idx, "chord_deviation_p99_5_m"] = deviation_limit
    longest = pd.concat([df.loc[ix].nlargest(10, "direct_chord_length_m")
                         for _, ix in df.groupby(["demo_source", "robot_id"]).groups.items()])
    return df, longest


def raw_source_check(tracks, steps, limits):
    rows = []
    for robot in ROBOTS:
        df = steps[("local", robot)]
        top = df.nlargest(20, "translation_xy_m")
        outlier = df[df.translation_xy_m.gt(limits[("local", robot, "translation_xy_m")])]
        target = pd.concat([top, outlier]).drop_duplicates(["keyframe_i", "keyframe_j"])
        if target.empty:
            rows.append({"robot_id": robot, "status": "NO_UPSTREAM_LOCAL_JUMP_REQUIRING_RAW_SOURCE_AUDIT"})
            continue
        ekf = s10.extract_ekf(robot)
        time = ekf.timestamp_ns.to_numpy(np.int64)
        pos = ekf[["tx", "ty", "tz"]].to_numpy(float)
        raw_step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
        lookup = tracks[("local", robot)].index()
        for r in target.itertuples():
            ki, kj = int(r.keyframe_i), int(r.keyframe_j)
            t0, t1 = tracks[("local", robot)].ts[[lookup[ki], lookup[kj]]]
            near = []
            for t in (t0, t1):
                j = int(np.searchsorted(time, t))
                j = min(max(j, 1), len(time) - 1)
                near.append(j if abs(int(time[j]) - int(t)) < abs(int(time[j - 1]) - int(t)) else j - 1)
            i, j = near
            raw_delta = float(np.linalg.norm(pos[j] - pos[i]))
            rows.append({"robot_id": robot, "keyframe_i": ki, "keyframe_j": kj,
                "local_translation_xy_m": float(r.translation_xy_m), "local_translation_xyz_m": float(r.translation_xyz_m),
                "lidar_timestamp_i_ns": int(t0), "lidar_timestamp_j_ns": int(t1),
                "raw_ekf_timestamp_i_ns": int(time[i]), "raw_ekf_timestamp_j_ns": int(time[j]),
                "raw_ekf_index_i": i, "raw_ekf_index_j": j,
                "raw_ekf_tx_i": float(pos[i, 0]), "raw_ekf_ty_i": float(pos[i, 1]), "raw_ekf_tz_i": float(pos[i, 2]),
                "raw_ekf_tx_j": float(pos[j, 0]), "raw_ekf_ty_j": float(pos[j, 1]), "raw_ekf_tz_j": float(pos[j, 2]),
                "raw_ekf_endpoint_translation_xyz_m": raw_delta,
                "raw_ekf_max_adjacent_translation_xyz_m": float(np.max(raw_step[min(i, j):max(i, j)])) if i != j else 0,
                "raw_message_dt_s": float((time[j] - time[i]) / 1e9),
                "source_archive": str(s10.RAW / robot / f"{robot}_main_campus_imu_gps.zip"),
                "topic": f"{robot}/ekf/odometry_map", "frame_id": str(ekf.frame.iloc[i]),
                "child_frame_id": str(ekf.child.iloc[i]), "status": "RAW_EKF_MESSAGE_CHECKED"})
    return pd.DataFrame(rows)


def plot_trajectories(stage, tracks, steps):
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    for ax, robot in zip(axes, ROBOTS):
        t = tracks[(stage, robot)]
        df = steps[(stage, robot)]
        ax.plot(t.xyz[:, 0], t.xyz[:, 1], color="#8a96a3", linewidth=0.65, label="all sequential poses")
        top = df.nlargest(20, "translation_xy_m")
        for r in top.itertuples():
            i = t.index()[int(r.keyframe_i)]
            ax.plot(t.xyz[i:i + 2, 0], t.xyz[i:i + 2, 1], color="#d62728", linewidth=1.5)
        for r in top.head(5).itertuples():
            i = t.index()[int(r.keyframe_i)]
            ax.annotate(f"{int(r.keyframe_i)}→{int(r.keyframe_j)}",
                        (t.xyz[i, 0], t.xyz[i, 1]), fontsize=7, color="#a11a1a")
        ax.set_title(f"{stage}: {robot}")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.2)
    fig.suptitle("Sequential trajectories; top 20 translation steps in red (diagnostic only)")
    fig.tight_layout()
    fig.savefig(OUT / f"{stage}_trajectory_jump_audit.png", dpi=170)
    plt.close(fig)


def plot_magnitudes(steps):
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=False)
    colors = {"local": "#596579", "pre_pgo": "#e4942e", "post_pgo": "#267dca"}
    for ax, robot in zip(axes, ROBOTS):
        for source in FULL:
            df = steps[(source, robot)]
            ax.plot(df.keyframe_j, df.translation_xy_m, color=colors[source], linewidth=.8,
                    alpha=.75, label=source)
        ax.set_yscale("symlog", linthresh=.1)
        ax.set_ylabel(f"{robot} step XY (m)")
        ax.grid(alpha=.2)
        ax.legend()
    axes[-1].set_xlabel("keyframe ID")
    fig.suptitle("Adjacent-pose translation; symlog axis exposes ordinary and extreme steps")
    fig.tight_layout()
    fig.savefig(OUT / "jump_magnitude_by_keyframe.png", dpi=170)
    plt.close(fig)


def plot_lineage(lineage_df):
    df = lineage_df[lineage_df.keyframe_j.eq(lineage_df.keyframe_i + 1)].copy()
    df["peak"] = df[["local_translation_m", "pre_pgo_translation_m", "post_pgo_translation_m"]].max(axis=1)
    selected = df.nlargest(12, "peak").sort_values("peak")
    y = np.arange(len(selected))
    fig, ax = plt.subplots(figsize=(12, max(5, len(selected) * .55)))
    for offset, column, color, label in ((-.25, "local_translation_m", "#607489", "local"),
                                          (0, "pre_pgo_translation_m", "#e4942e", "pre-PGO"),
                                          (.25, "post_pgo_translation_m", "#267dca", "post-PGO")):
        ax.barh(y + offset, selected[column].to_numpy(float), height=.23, color=color, label=label)
    ax.set_yticks(y, [f"{r.robot_id} {r.keyframe_i}→{r.keyframe_j}" for r in selected.itertuples()])
    ax.set_xlabel("Adjacent XY translation (m)")
    ax.set_title("Largest suspicious transitions across frozen sources")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "jump_lineage_comparison.png", dpi=170)
    plt.close(fig)


def plot_downsampling(tracks, segments):
    ranked = segments.sort_values("direct_chord_length_m", ascending=False)
    row = ranked.iloc[0]
    parent = tracks[(str(row.full_source), str(row.robot_id))]
    ix = parent.index()
    path = parent.xyz[ix[int(row.keyframe_i)]:ix[int(row.keyframe_j)] + 1, :2]
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(path[:, 0], path[:, 1], "o-", color="#294c7a", markersize=2.5,
            linewidth=1.5, label="full-resolution path")
    ax.plot(path[[0, -1], 0], path[[0, -1], 1], "--", color="#d62728", linewidth=2,
            label="browser chord (stride 10)")
    ax.scatter(path[[0, -1], 0], path[[0, -1], 1], color="#d62728", s=35)
    ax.set_title(f"{row.demo_source} {row.robot_id} #{int(row.keyframe_i)}→#{int(row.keyframe_j)}\n"
                 f"chord {row.direct_chord_length_m:.1f} m; path {row.full_resolution_path_length_m:.1f} m")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(OUT / "demo_downsampling_artifact.png", dpi=170)
    plt.close(fig)


def validate(tracks, ordering_rows, steps, demo_match, raw, pgo, segments):
    assert len(tracks[("local", "robot1")].key) == 2000
    assert len(tracks[("local", "robot3")].key) == 4180
    for source in FULL:
        assert sum(len(tracks[(source, r)].key) for r in ROBOTS) == 6180
    assert all(x["duplicate_count"] == 0 and x["timestamp_monotonic"] and x["keyframe_monotonic"]
               and not x["unexpected_ordering_events"] for x in ordering_rows)
    assert all(v["max_xy_difference_m"] < 1e-6 for v in demo_match.values())
    assert len(pgo) == 6178
    assert len(segments) == sum(len(tracks[(s, r)].key) - 1
                                for s in ("demo_local", "demo_pre_pgo", "demo_post_pgo") for r in ROBOTS)
    assert set(raw.status) <= {"RAW_EKF_MESSAGE_CHECKED", "NO_UPSTREAM_LOCAL_JUMP_REQUIRING_RAW_SOURCE_AUDIT"}
    for df in steps.values():
        assert df.relative_rotation_deg.ge(0).all() and df.relative_rotation_deg.le(180 + 1e-9).all()


def main():
    assert PRE.is_file() and POST.is_file() and DEMO.is_file()
    OUT.mkdir(parents=True, exist_ok=True)
    tracks, local_audit = {}, {}
    for robot in ROBOTS:
        tracks[("local", robot)], local_audit[robot] = make_local(robot)
    tracks.update(make_csv("pre_pgo", PRE))
    tracks.update(make_csv("post_pgo", POST))
    data = json.loads(DEMO.read_text())
    demo_tracks, demo_match = make_demo(data, tracks)
    tracks.update(demo_tracks)

    # Ordering is audited before any step or jump calculation.
    ordering_rows = [ordering(tracks[(source, robot)]) for source in SOURCES for robot in ROBOTS]
    dump("trajectory_ordering_audit.json", {"sources": ordering_rows, "demo_xy_lineage": demo_match,
        "timestamp_unit": "Unix nanoseconds", "full_resolution_expected_step": 1,
        "browser_expected_step": 10})
    assert all(not r["unexpected_ordering_events"] for r in ordering_rows), "ordering requires investigation"

    steps = {(source, robot): step_frame(tracks[(source, robot)]) for source in SOURCES for robot in ROBOTS}
    for (source, robot), df in steps.items():
        table(f"{source}_{robot}_steps.csv", df)
    stat_rows, candidate_rows, limits = metric_audit(steps)
    for row in candidate_rows:
        source, robot = row["source"], row["robot_id"]
        step = pd.Series(row)
        row["provisional_classification"] = provisional(step,
            float(steps[(source, robot)].dt_s.median()),
            limits[(source, robot, "relative_rotation_deg")], row["metric"])
    table("trajectory_step_statistics.csv", stat_rows)
    table("trajectory_jump_candidates.csv", candidate_rows)

    rigidity = {}
    for robot in ROBOTS:
        local, pre = steps[("local", robot)], steps[("pre_pgo", robot)]
        assert np.array_equal(local[["keyframe_i", "keyframe_j"]], pre[["keyframe_i", "keyframe_j"]])
        xyz_error = np.abs(local.translation_xyz_m.to_numpy() - pre.translation_xyz_m.to_numpy())
        assert float(np.max(xyz_error)) < 1e-8, "pre-PGO should be one rigid 3-D transform"
        rigidity[robot] = {"max_xyz_step_difference_m": float(np.max(xyz_error)),
            "max_xy_projection_step_difference_m": float(np.max(np.abs(
                local.translation_xy_m.to_numpy() - pre.translation_xy_m.to_numpy()))),
            "interpretation": "3-D rigid transform preserved; XY projection can change under frame tilt"}
    dump("pre_pgo_rigidity_audit.json", rigidity)

    lin = lineage(tracks, steps, candidate_rows, limits)
    lin.to_csv(OUT / "trajectory_jump_lineage.csv", index=False)
    changes = pgo_changes(steps, limits)
    changes.to_csv(OUT / "pgo_step_change_audit.csv", index=False)
    seg, longest = demo_segments(tracks, steps, limits)
    seg.to_csv(OUT / "demo_polyline_downsampling_audit.csv", index=False)
    longest.to_csv(OUT / "demo_longest_segments.csv", index=False)
    raw = raw_source_check(tracks, steps, limits)
    raw.to_csv(OUT / "upstream_jump_source_audit.csv", index=False)

    for stage in FULL:
        plot_trajectories(stage, tracks, steps)
    plot_magnitudes(steps)
    plot_lineage(lin)
    plot_downsampling(tracks, seg)
    validate(tracks, ordering_rows, steps, demo_match, raw, changes, seg)

    max_step = {(source, robot): float(steps[(source, robot)].translation_xy_m.max())
                for source in FULL for robot in ROBOTS}
    candidate_counts = {source: int(len({(r["robot_id"], r["keyframe_i"], r["keyframe_j"])
        for r in candidate_rows if r["source"] == source and r["robust_outlier_flag"]})) for source in SOURCES}
    new_pgo = changes[changes.new_translation_outlier | changes.new_rotation_outlier]
    # The specified top-20 ranking plus the robust PRE threshold define whether
    # a POST top-20 translation is truly new. A small new robust flag is not
    # automatically a catastrophic discontinuity.
    top20_new = []
    for robot in ROBOTS:
        top = changes[changes.robot_id.eq(robot)].nlargest(20, "post_translation_m")
        top20_new.extend(top[top.pre_translation_m.le(limits[("pre_pgo", robot, "translation_xy_m")])].to_dict("records"))
    visible = longest[longest.demo_source.eq("demo_post_pgo")].groupby("robot_id", group_keys=False).head(2)
    line_origins = set()
    for r in visible.itertuples():
        found = lin[(lin.robot_id.eq(r.robot_id)) & lin.keyframe_i.eq(r.keyframe_i) & lin.keyframe_j.eq(r.keyframe_j)]
        if len(found):
            line_origins.add(str(found.iloc[0].first_source_where_jump_appears))
    if line_origins == {"LOCAL"}:
        root = "MULTIPLE_CAUSES" if visible.chord_shape_artifact_flag.any() else "UPSTREAM_LOCAL_ODOMETRY"
    elif line_origins == {"PRE_PGO"}:
        root = "COMMON_FRAME"
    elif line_origins == {"POST_PGO"}:
        root = "PGO"
    elif line_origins == {"DEMO"}:
        root = "DOWNSAMPLING"
    else:
        root = "UNRESOLVED"
    summary = {"status": "PASS", "robot1_local_max_step_m": max_step[("local", "robot1")],
        "robot3_local_max_step_m": max_step[("local", "robot3")],
        "robot1_pre_pgo_max_step_m": max_step[("pre_pgo", "robot1")],
        "robot3_pre_pgo_max_step_m": max_step[("pre_pgo", "robot3")],
        "robot1_post_pgo_max_step_m": max_step[("post_pgo", "robot1")],
        "robot3_post_pgo_max_step_m": max_step[("post_pgo", "robot3")],
        "number_local_jump_candidates": candidate_counts["local"],
        "number_pre_pgo_jump_candidates": candidate_counts["pre_pgo"],
        "number_post_pgo_jump_candidates": candidate_counts["post_pgo"],
        "number_demo_visual_artifacts": int(seg.chord_shape_artifact_flag.sum()),
        "pgo_introduces_new_catastrophic_jump": bool(len(top20_new)),
        "pgo_top20_new_without_pre_outlier_count": len(top20_new),
        "pgo_new_robust_outlier_count": int(len(new_pgo)),
        "demo_long_line_root_cause": root,
        "primary_pose_discontinuity_source": "UPSTREAM_LOCAL_ODOMETRY" if line_origins == {"LOCAL"} else root,
        "recommendation": "INVESTIGATE_LOCAL_ODOMETRY_UPSTREAM" if root in ("UPSTREAM_LOCAL_ODOMETRY", "MULTIPLE_CAUSES") else "INVESTIGATE_PGO",
        "main_diagnosis_uses_gt": False,
        "source_archives": {r: str(s10.RAW / r / f"{r}_main_campus_imu_gps.zip") for r in ROBOTS},
        "normal_time_step_s": {r: float(np.median(steps[("local", r)].dt_s)) for r in ROBOTS},
        "max_full_resolution_dt_s": {r: float(max(steps[(s, r)].dt_s.max() for s in FULL)) for r in ROBOTS},
        "recording_gap_count_2x_nominal": {r: int((steps[("local", r)].dt_s >
            2 * steps[("local", r)].dt_s.median()).sum()) for r in ROBOTS},
        "figure_count": 6}
    dump("summary.json", summary)
    report = [
        "LOCAL ROBOT1 AUDITED: PASS", "LOCAL ROBOT3 AUDITED: PASS", "PRE-PGO AUDITED: PASS",
        "POST-PGO AUDITED: PASS", "TIMESTAMP ORDER AUDITED: PASS", "KEYFRAME ORDER AUDITED: PASS",
        "SE3 ROTATION DELTA USED: PASS", "TOP-20 JUMPS EXPORTED: PASS",
        "CROSS-SOURCE LINEAGE COMPLETE: PASS", "DEMO DOWNSAMPLING AUDITED: PASS",
        "PGO NEW-JUMP TEST: PASS", "GT USED FOR MAIN DIAGNOSIS: NO",
        "FROZEN STAGE10.3 MODIFIED: NO", "DEMO MODIFIED: NO", f"ROOT CAUSE: {root}",
        f"PGO NEW ROBUST OUTLIERS: {len(new_pgo)}",
        f"PGO TOP-20 NEW WITHOUT PRE OUTLIER: {len(top20_new)}",
    ]
    (OUT / "VALIDATION_REPORT.txt").write_text("\n".join(report) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
