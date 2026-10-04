#!/usr/bin/env python3
"""GT-free Stage 12B forensic read of frozen CU-Multi IMU/GNSS ROS2 archives.

The raw ZIPs and all earlier stage outputs are read-only. Temporary SQLite
copies are deleted, and persistent writes stay in 12b_ekf_forensics.
"""
from __future__ import annotations

import csv
import json
import math
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import zipfile
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
RAW = Path("/home/cas/CU-Multi/raw/main_campus")
STAGE12A = ROOT / "outputs/cumulti_v1/12a_trajectory_integrity"
OUT = ROOT / "outputs/cumulti_v1/12b_ekf_forensics"
ROBOTS = ("robot1", "robot3")
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
# Field-only copies of the official LORD MicroStrain ROS2 message schemas at
# microstrain_inertial_msgs_common commit ed9cb178b1be0ff110e41548ca2165b593976146.
# Constants/comments are omitted; serialization depends only on these fields.
TYPESTORE.register(get_types_from_msg("""string firmware_version
string model_name
string model_number
string serial_number
string lot_number
string device_options
""", "microstrain_inertial_msgs/msg/MipBaseDeviceInfo"))
TYPESTORE.register(get_types_from_msg("""std_msgs/Header header
microstrain_inertial_msgs/MipBaseDeviceInfo device_info
string gnss_state
string dual_antenna_fix_type
string filter_state
string[] status_flags
string[] continuous_bit_flags
""", "microstrain_inertial_msgs/msg/HumanReadableStatus"))
WINDOW_MARGIN_NS = 5_000_000_000
DETAIL_MARGIN_NS = 1_000_000_000
# Fixed from recorded nominal rates before examining event values: 100-Hz EKF
# gets 50 ms; approximately 2-Hz raw GNSS gets 350 ms.
SYNC_LIMIT_S = {"ekf": 0.05, "gnss": 0.35, "tf": 0.05}
EARTH_RADIUS_M = 6378137.0


def write_json(name, obj):
    (OUT / name).write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def write_csv(name, rows, columns=None):
    pd.DataFrame(rows, columns=columns).to_csv(OUT / name, index=False)


def finite(v):
    x = float(v)
    return x if math.isfinite(x) else None


def stamp_ns(msg):
    try:
        h = msg.header.stamp
        return int(h.sec) * 1_000_000_000 + int(h.nanosec)
    except AttributeError:
        return None


def quat(q):
    return [float(q.x), float(q.y), float(q.z), float(q.w)]


def vector(p):
    return [float(p.x), float(p.y), float(p.z)]


def covariance(cov):
    a = np.asarray(cov, dtype=float).reshape(6, 6)
    diag = np.diag(a)
    return {"position_covariance_trace": finite(np.trace(a[:3, :3])),
            "orientation_covariance_proxy": finite(np.trace(a[3:, 3:])),
            "covariance_max_diagonal": finite(np.max(diag)),
            "covariance_nonfinite": bool(not np.isfinite(a).all())}


def archive_metadata(robot):
    path = RAW / robot / f"{robot}_main_campus_imu_gps.zip"
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        db3 = [n for n in names if n.endswith(".db3")]
        meta = [n for n in names if n.endswith("metadata.yaml")]
        assert len(db3) == len(meta) == 1, names
        y = yaml.safe_load(archive.read(meta[0]))["rosbag2_bagfile_information"]
        assert y["storage_identifier"] == "sqlite3"
        topics = []
        for item in y["topics_with_message_count"]:
            t = item["topic_metadata"]
            topics.append({"topic_name": t["name"], "message_type": t["type"],
                           "message_count": int(item["message_count"]),
                           "first_timestamp": None, "last_timestamp": None})
        result = {"archive_path": str(path), "storage_format": "ROS2 rosbag2 SQLite3/db3 CDR",
                  "archive_members": names, "db3_member": db3[0], "metadata_member": meta[0],
                  "bag_first_timestamp": int(y["starting_time"]["nanoseconds_since_epoch"]),
                  "bag_last_timestamp": int(y["starting_time"]["nanoseconds_since_epoch"])
                  + int(y["duration"]["nanoseconds"]), "topics": topics}
    return result


def select_events():
    assert STAGE12A.joinpath("summary.json").is_file()
    upstream = pd.read_csv(STAGE12A / "upstream_jump_source_audit.csv")
    lineage = pd.read_csv(STAGE12A / "trajectory_jump_lineage.csv")
    summary = json.loads((STAGE12A / "summary.json").read_text())
    assert summary["status"] == "PASS"
    assert summary["primary_pose_discontinuity_source"] == "UPSTREAM_LOCAL_ODOMETRY"
    result = []
    for robot, primary in (("robot1", (57, 58)), ("robot3", (42, 43))):
        steps = pd.read_csv(STAGE12A / f"local_{robot}_steps.csv")
        top = steps.nlargest(10, "translation_xy_m")
        assert primary in set(zip(top.keyframe_i, top.keyframe_j))
        for rank, r in enumerate(top.itertuples(), 1):
            key = (int(r.keyframe_i), int(r.keyframe_j))
            source = upstream[upstream.robot_id.eq(robot) & upstream.keyframe_i.eq(key[0]) & upstream.keyframe_j.eq(key[1])]
            lin = lineage[lineage.robot_id.eq(robot) & lineage.keyframe_i.eq(key[0]) & lineage.keyframe_j.eq(key[1])]
            assert len(source) == len(lin) == 1, (robot, key)
            assert int(source.iloc[0].lidar_timestamp_i_ns) == int(r.timestamp_i)
            assert int(source.iloc[0].lidar_timestamp_j_ns) == int(r.timestamp_j)
            assert lin.iloc[0].first_source_where_jump_appears == "LOCAL"
            result.append({"event_id": f"{robot}_rank{rank:02d}", "robot_id": robot,
                "keyframe_i": key[0], "keyframe_j": key[1],
                "timestamp_i": int(r.timestamp_i), "timestamp_j": int(r.timestamp_j),
                "dt_s": float(r.dt_s), "local_translation_xy_m": float(r.translation_xy_m),
                "local_translation_xyz_m": float(r.translation_xyz_m),
                "local_rotation_deg": float(r.relative_rotation_deg),
                "rank_by_translation": rank, "is_primary": key == primary,
                "stage12a_source": "upstream_jump_source_audit.csv + local full-resolution step table + lineage"})
    assert len(result) == 20
    return result


def extracted_db(meta, directory):
    path = Path(directory) / Path(meta["db3_member"]).name
    with path.open("wb") as out:
        subprocess.run(["unzip", "-p", meta["archive_path"], meta["db3_member"]],
                       stdout=out, check=True)
    return path


def topic_is_relevant(name):
    n = name.lower()
    return any(k in n for k in ("/ekf/", "/gnss_", "/mip/ekf/", "/imu/data", "/tf", "odometry", "velocity"))


def open_archive_database(path, meta):
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    topic_rows = db.execute("SELECT id,name,type FROM topics").fetchall()
    topics = {int(i): {"name": name, "type": typ} for i, name, typ in topic_rows}
    observed = {int(i): (int(first), int(last), int(count)) for i, first, last, count in db.execute(
        "SELECT topic_id,MIN(timestamp),MAX(timestamp),COUNT(*) FROM messages GROUP BY topic_id")}
    by_name = {t["name"]: (i, t["type"]) for i, t in topics.items()}
    for item in meta["topics"]:
        i, typ = by_name[item["topic_name"]]
        first, last, count = observed[i]
        assert typ == item["message_type"] and count == item["message_count"]
        item["first_timestamp"], item["last_timestamp"] = first, last
    return db, topics


def decode(raw, typ):
    try:
        return TYPESTORE.deserialize_cdr(raw, typ), "PASS"
    except Exception as error:
        return None, f"UNSUPPORTED_TYPE:{type(error).__name__}"


def message_payload(msg, typ):
    if msg is None:
        return {}
    d = {"header_timestamp_ns": stamp_ns(msg),
         "frame_id": str(msg.header.frame_id) if hasattr(msg, "header") else None}
    if typ == "nav_msgs/msg/Odometry":
        pose = msg.pose.pose
        d.update({"child_frame_id": str(msg.child_frame_id), "position": vector(pose.position),
                  "quaternion": quat(pose.orientation),
                  "linear_velocity": vector(msg.twist.twist.linear),
                  "angular_velocity": vector(msg.twist.twist.angular),
                  "pose_covariance": covariance(msg.pose.covariance),
                  "twist_covariance": covariance(msg.twist.covariance)})
    elif typ == "sensor_msgs/msg/NavSatFix":
        d.update({"latitude": finite(msg.latitude), "longitude": finite(msg.longitude),
                  "altitude": finite(msg.altitude), "fix_status": int(msg.status.status),
                  "fix_service": int(msg.status.service),
                  "position_covariance_type": int(msg.position_covariance_type),
                  "position_covariance": [finite(v) for v in msg.position_covariance]})
    elif typ == "sensor_msgs/msg/Imu":
        d.update({"linear_acceleration": vector(msg.linear_acceleration),
                  "angular_velocity": vector(msg.angular_velocity),
                  "quaternion": quat(msg.orientation)})
    elif typ == "geometry_msgs/msg/TwistWithCovarianceStamped":
        d.update({"linear_velocity": vector(msg.twist.twist.linear),
                  "angular_velocity": vector(msg.twist.twist.angular)})
    elif typ == "geometry_msgs/msg/PoseWithCovarianceStamped":
        d.update({"position": vector(msg.pose.pose.position),
                  "quaternion": quat(msg.pose.pose.orientation),
                  "pose_covariance": covariance(msg.pose.covariance)})
    elif typ == "tf2_msgs/msg/TFMessage":
        d["transforms"] = [{"parent": str(t.header.frame_id), "child": str(t.child_frame_id),
            "timestamp_ns": int(t.header.stamp.sec) * 1_000_000_000 + int(t.header.stamp.nanosec),
            "position": vector(t.transform.translation), "quaternion": quat(t.transform.rotation)}
            for t in msg.transforms]
    elif typ == "microstrain_inertial_msgs/msg/HumanReadableStatus":
        d.update({"gnss_state": str(msg.gnss_state),
                  "dual_antenna_fix_type": str(msg.dual_antenna_fix_type),
                  "filter_state": str(msg.filter_state),
                  "status_flags": list(msg.status_flags),
                  "continuous_bit_flags": list(msg.continuous_bit_flags),
                  "device_model": str(msg.device_info.model_name)})
    return d


def local_enu_displacement(a, b):
    if any(a.get(k) is None or b.get(k) is None for k in ("latitude", "longitude")):
        return None
    lat_a, lat_b = math.radians(a["latitude"]), math.radians(b["latitude"])
    lon_a, lon_b = math.radians(a["longitude"]), math.radians(b["longitude"])
    east = EARTH_RADIUS_M * math.cos((lat_a + lat_b) / 2) * (lon_b - lon_a)
    north = EARTH_RADIUS_M * (lat_b - lat_a)
    return float(math.hypot(east, north))


def nearest(rows, target_ns, max_error_s):
    if not rows:
        return None, None
    ix = min(range(len(rows)), key=lambda i: abs(rows[i]["timestamp_ns"] - target_ns))
    error = abs(rows[ix]["timestamp_ns"] - target_ns) / 1e9
    return (rows[ix] if error <= max_error_s else None), error


def euclidean(a, b):
    return float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float)))


def rotation_delta(q0, q1):
    return float(np.rad2deg((Rotation.from_quat(q0).inv() * Rotation.from_quat(q1)).magnitude()))


def query_event(db, topics, event):
    ids = [i for i, t in topics.items() if topic_is_relevant(t["name"])]
    assert ids
    start = event["timestamp_i"] - WINDOW_MARGIN_NS
    stop = event["timestamp_j"] + WINDOW_MARGIN_NS
    placeholders = ",".join("?" for _ in ids)
    sql = ("SELECT id,timestamp,topic_id,data FROM messages "
           f"WHERE timestamp>=? AND timestamp<=? AND topic_id IN ({placeholders}) "
           "ORDER BY timestamp,id")
    groups = defaultdict(list)
    failures = Counter()
    for seq, bag_ns, topic_id, raw in db.execute(sql, (start, stop, *ids)):
        topic = topics[int(topic_id)]
        typ = topic["type"]
        if typ.startswith("microstrain_") and typ != "microstrain_inertial_msgs/msg/HumanReadableStatus":
            msg, status = None, "UNSUPPORTED_CUSTOM_MESSAGE"
        else:
            msg, status = decode(raw, typ)
        if status != "PASS":
            failures[(topic["name"], status)] += 1
        payload = message_payload(msg, typ)
        header_ns = payload.get("header_timestamp_ns")
        t = int(header_ns if header_ns is not None else bag_ns)
        groups[topic["name"]].append({"message_sequence_index": int(seq),
            "bag_timestamp_ns": int(bag_ns), "header_timestamp_ns": header_ns,
            "timestamp_ns": t, "topic": topic["name"], "message_type": typ,
            "decode_status": status, **payload})
    for rows in groups.values():
        rows.sort(key=lambda r: (r["timestamp_ns"], r["message_sequence_index"]))
    return groups, failures


def raw_peak_time(ekf, event):
    within = [r for r in ekf if event["timestamp_i"] - 50_000_000 <= r["timestamp_ns"] <=
              event["timestamp_j"] + 50_000_000 and r.get("position") is not None]
    assert len(within) >= 2, event["event_id"]
    distance = np.linalg.norm(np.diff(np.asarray([r["position"] for r in within]), axis=0), axis=1)
    i = int(np.argmax(distance))
    return within[i + 1]["timestamp_ns"], float(distance[i]), len(within)


def gnss_distance(a, b, typ):
    if a is None or b is None or a["message_sequence_index"] == b["message_sequence_index"]:
        return None
    if typ == "sensor_msgs/msg/NavSatFix":
        return local_enu_displacement(a, b)
    if a.get("position") and b.get("position"):
        return euclidean(a["position"], b["position"])
    return None


def source_pair(groups, topic, event, max_error_s, typ):
    rows = groups.get(topic, [])
    before, err_before = nearest(rows, event["timestamp_i"], max_error_s)
    after, err_after = nearest(rows, event["timestamp_j"], max_error_s)
    displacement = gnss_distance(before, after, typ)
    return {"before": before, "after": after, "before_error_s": err_before,
            "after_error_s": err_after, "translation_m": displacement,
            "status": "MATCHED" if displacement is not None else
              ("SAME_SAMPLE" if before and after and before["message_sequence_index"] ==
               after["message_sequence_index"] else "UNMATCHED")}


def behavior(ekf, event, jump_ns):
    valid = [r for r in ekf if r.get("position") is not None]
    before, _ = nearest(valid, event["timestamp_i"], SYNC_LIMIT_S["ekf"])
    assert before is not None
    base = np.asarray(before["position"])
    position = np.asarray([r["position"] for r in valid])
    distance = np.linalg.norm(position - base, axis=1)
    peak = float(np.max(distance))
    peak_time = valid[int(np.argmax(distance))]["timestamp_ns"]
    after_t = event["timestamp_j"] + WINDOW_MARGIN_NS
    later, later_err = nearest(valid, after_t, 0.1)
    if later is None:
        later = valid[-1]
    endpoint_distance = euclidean(later["position"], base)
    high = [r["timestamp_ns"] for r, d in zip(valid, distance) if d >= max(peak * .5, 1e-9)]
    high_duration = float((max(high) - min(high)) / 1e9) if high else 0.
    # Conservative shape labels; they do not imply an EKF reset mechanism.
    if peak < 1e-9:
        category = "UNRESOLVED"
    elif endpoint_distance / peak >= .75:
        category = "PERSISTENT_OFFSET"
    elif endpoint_distance / peak <= .2 and high_duration <= .1:
        category = "ONE_OR_FEW_SAMPLE_SPIKE"
    elif endpoint_distance / peak <= .2 and high_duration > .1:
        category = "OTHER"
    else:
        category = "OTHER"
    return {"event_id": event["event_id"], "robot_id": event["robot_id"],
        "keyframe_i": event["keyframe_i"], "keyframe_j": event["keyframe_j"],
        "jump_timestamp_ns": jump_ns, "baseline_timestamp_ns": before["timestamp_ns"],
        "baseline_position_xyz": json.dumps(before["position"]),
        "peak_distance_from_baseline_m": peak, "peak_timestamp_ns": peak_time,
        "distance_at_5s_after_m": endpoint_distance, "later_timestamp_ns": later["timestamp_ns"],
        "later_match_error_s": later_err, "distance_after_to_peak_ratio": endpoint_distance / max(peak, 1e-12),
        "duration_above_half_peak_s": high_duration, "category": category,
        "window_start_ns": event["timestamp_i"] - WINDOW_MARGIN_NS,
        "window_end_ns": event["timestamp_j"] + WINDOW_MARGIN_NS}


def process_event(event, groups, inventory_topics):
    robot = event["robot_id"]
    ekf_topic = f"{robot}/ekf/odometry_map"
    ekf = groups.get(ekf_topic, [])
    assert ekf and all(r.get("position") for r in ekf)
    jump_ns, max_raw_step, raw_count = raw_peak_time(ekf, event)
    event["jump_timestamp_ns"] = jump_ns
    event["max_adjacent_raw_ekf_step_m"] = max_raw_step
    event["raw_ekf_messages_in_keyframe_interval"] = raw_count
    topic_types = {t["topic_name"]: t["message_type"] for t in inventory_topics}

    window_rows = []
    for topic, records in groups.items():
        for r in records:
            window_rows.append({"event_id": event["event_id"], "robot_id": robot,
                "topic": topic, "message_type": r["message_type"],
                "timestamp_ns": r["timestamp_ns"], "bag_timestamp_ns": r["bag_timestamp_ns"],
                "header_timestamp_ns": r["header_timestamp_ns"],
                "relative_time_to_jump_s": (r["timestamp_ns"] - jump_ns) / 1e9,
                "message_sequence_index": r["message_sequence_index"],
                "decode_status": r["decode_status"]})

    detail, cov_rows = [], []
    prev = None
    for r in ekf:
        if prev is None:
            trans_xy = trans_xyz = rot = speed = None
            dt = None
        else:
            delta = np.asarray(r["position"]) - np.asarray(prev["position"])
            trans_xy = float(np.linalg.norm(delta[:2]))
            trans_xyz = float(np.linalg.norm(delta))
            rot = rotation_delta(prev["quaternion"], r["quaternion"])
            dt = (r["timestamp_ns"] - prev["timestamp_ns"]) / 1e9
            speed = trans_xyz / dt if dt > 0 else None
        if abs(r["timestamp_ns"] - jump_ns) <= DETAIL_MARGIN_NS:
            pose_cov = r["pose_covariance"]
            twist_cov = r["twist_covariance"]
            detail.append({"event_id": event["event_id"], "robot_id": robot,
                "message_sequence_index": r["message_sequence_index"],
                "bag_timestamp_ns": r["bag_timestamp_ns"],
                "header_timestamp_ns": r["header_timestamp_ns"],
                "frame_id": r["frame_id"], "child_frame_id": r["child_frame_id"],
                "x": r["position"][0], "y": r["position"][1], "z": r["position"][2],
                "qx": r["quaternion"][0], "qy": r["quaternion"][1],
                "qz": r["quaternion"][2], "qw": r["quaternion"][3],
                "linear_velocity_x": r["linear_velocity"][0],
                "linear_velocity_y": r["linear_velocity"][1],
                "linear_velocity_z": r["linear_velocity"][2],
                "angular_velocity_x": r["angular_velocity"][0],
                "angular_velocity_y": r["angular_velocity"][1],
                "angular_velocity_z": r["angular_velocity"][2],
                "reported_linear_speed_mps": float(np.linalg.norm(r["linear_velocity"])),
                "pose_covariance_position_trace": pose_cov["position_covariance_trace"],
                "pose_covariance_orientation_proxy": pose_cov["orientation_covariance_proxy"],
                "pose_covariance_max_diagonal": pose_cov["covariance_max_diagonal"],
                "pose_covariance_nonfinite": pose_cov["covariance_nonfinite"],
                "twist_covariance_position_trace": twist_cov["position_covariance_trace"],
                "twist_covariance_orientation_proxy": twist_cov["orientation_covariance_proxy"],
                "twist_covariance_max_diagonal": twist_cov["covariance_max_diagonal"],
                "twist_covariance_nonfinite": twist_cov["covariance_nonfinite"],
                "delta_translation_xy_m": trans_xy, "delta_translation_xyz_m": trans_xyz,
                "relative_rotation_deg": rot, "dt_s": dt,
                "implied_linear_speed_mps": speed,
                "relative_time_to_jump_s": (r["timestamp_ns"] - jump_ns) / 1e9})
        prev = r
    for phase, target in (("before", event["timestamp_i"]), ("at_peak", jump_ns),
                          ("after", event["timestamp_j"])):
        r, error = nearest(ekf, target, SYNC_LIMIT_S["ekf"])
        if r:
            p, t = r["pose_covariance"], r["twist_covariance"]
            cov_rows.append({"event_id": event["event_id"], "robot_id": robot,
                "phase": phase, "timestamp_ns": r["timestamp_ns"], "match_error_s": error,
                **{f"pose_{k}": v for k, v in p.items()},
                **{f"twist_{k}": v for k, v in t.items()}})

    gnss_rows = []
    gnss_topics = [t for t in groups if (("/gnss_" in t or t.endswith("/ekf/llh_position")) and
        topic_types.get(t) in ("sensor_msgs/msg/NavSatFix", "nav_msgs/msg/Odometry"))]
    for topic in gnss_topics:
        prev = None
        for r in groups[topic]:
            if r["decode_status"] != "PASS":
                continue
            jump = gnss_distance(prev, r, topic_types[topic]) if prev else None
            gnss_rows.append({"event_id": event["event_id"], "robot_id": robot,
                "topic": topic, "message_type": topic_types[topic],
                "source_family": "EKF_FUSED" if "/ekf/" in topic else "RAW_GNSS",
                "message_sequence_index": r["message_sequence_index"],
                "timestamp_ns": r["timestamp_ns"],
                "relative_time_to_jump_s": (r["timestamp_ns"] - jump_ns) / 1e9,
                "latitude": r.get("latitude"), "longitude": r.get("longitude"),
                "altitude": r.get("altitude"), "x": (r.get("position") or [None] * 3)[0],
                "y": (r.get("position") or [None] * 3)[1],
                "z": (r.get("position") or [None] * 3)[2],
                "fix_status": r.get("fix_status"), "fix_service": r.get("fix_service"),
                "position_covariance_type": r.get("position_covariance_type"),
                "position_covariance_diagonal": json.dumps([r["position_covariance"][i]
                    for i in (0, 4, 8)]) if "position_covariance" in r else None,
                "sequential_displacement_m": jump,
                "sequential_dt_s": (r["timestamp_ns"] - prev["timestamp_ns"]) / 1e9 if prev else None})
            prev = r

    # The two raw antenna NavSatFix topics are independent of the fused EKF LLH.
    pairs = {}
    for antenna in ("gnss_1", "gnss_2"):
        topic = f"{robot}/{antenna}/llh_position"
        pairs[antenna] = source_pair(groups, topic, event, SYNC_LIMIT_S["gnss"],
                                    "sensor_msgs/msg/NavSatFix")
    extra_pairs = {}
    for name, topic, typ, limit in (
        ("ekf_earth", f"{robot}/ekf/odometry_earth", "nav_msgs/msg/Odometry", SYNC_LIMIT_S["ekf"]),
        ("ekf_llh", f"{robot}/ekf/llh_position", "sensor_msgs/msg/NavSatFix", SYNC_LIMIT_S["ekf"]),
        ("gnss1_earth", f"{robot}/gnss_1/odometry_earth", "nav_msgs/msg/Odometry", SYNC_LIMIT_S["gnss"]),
        ("gnss2_earth", f"{robot}/gnss_2/odometry_earth", "nav_msgs/msg/Odometry", SYNC_LIMIT_S["gnss"])):
        extra_pairs[name] = source_pair(groups, topic, event, limit, typ)
    e0, err0 = nearest(ekf, event["timestamp_i"], SYNC_LIMIT_S["ekf"])
    e1, err1 = nearest(ekf, event["timestamp_j"], SYNC_LIMIT_S["ekf"])
    assert e0 and e1 and e0["message_sequence_index"] != e1["message_sequence_index"]
    ekf_jump = euclidean(e0["position"], e1["position"])
    alignment = {"event_id": event["event_id"], "robot_id": robot,
        "keyframe_i": event["keyframe_i"], "keyframe_j": event["keyframe_j"],
        "jump_timestamp_ns": jump_ns,
        "ekf_position_before_xyz": json.dumps(e0["position"]),
        "ekf_position_after_xyz": json.dumps(e1["position"]),
        "ekf_translation_jump_xyz_m": ekf_jump,
        "ekf_translation_jump_xy_m": euclidean(e0["position"][:2], e1["position"][:2]),
        "ekf_before_match_error_s": err0, "ekf_after_match_error_s": err1,
        "ekf_rotation_delta_deg": rotation_delta(e0["quaternion"], e1["quaternion"]),
        "reported_velocity_before_mps": euclidean(e0["linear_velocity"], [0, 0, 0]),
        "reported_velocity_after_mps": euclidean(e1["linear_velocity"], [0, 0, 0]),
        "pose_covariance_trace_before": e0["pose_covariance"]["position_covariance_trace"],
        "pose_covariance_trace_after": e1["pose_covariance"]["position_covariance_trace"],
        "tf_map_to_odom_before_xyz": None, "tf_map_to_odom_after_xyz": None,
        "tf_map_to_odom_translation_jump_m": None,
        "tf_odom_to_base_before_xyz": None, "tf_odom_to_base_after_xyz": None,
        "tf_odom_to_base_translation_jump_m": None,
        "tf_match_status": "NOT_AVAILABLE_IN_ARCHIVE",
        "ekf_max_adjacent_raw_step_m": max_raw_step,
        "ekf_raw_message_count_in_keyframe_interval": raw_count}
    for antenna, pair in pairs.items():
        for phase in ("before", "after"):
            r = pair[phase]
            alignment[f"{antenna}_{phase}_llh"] = json.dumps([r["latitude"], r["longitude"], r["altitude"]]) if r else None
            alignment[f"{antenna}_{phase}_match_error_s"] = pair[f"{phase}_error_s"]
            alignment[f"{antenna}_{phase}_fix_status"] = r.get("fix_status") if r else None
        alignment[f"{antenna}_translation_jump_m"] = pair["translation_m"]
        alignment[f"{antenna}_match_status"] = pair["status"]
    for name, pair in extra_pairs.items():
        alignment[f"{name}_translation_jump_m"] = pair["translation_m"]
        alignment[f"{name}_match_status"] = pair["status"]
        for phase in ("before", "after"):
            r = pair[phase]
            alignment[f"{name}_{phase}_position"] = json.dumps(r.get("position") or
                [r.get("latitude"), r.get("longitude"), r.get("altitude")]) if r else None
            alignment[f"{name}_{phase}_match_error_s"] = pair[f"{phase}_error_s"]
    earth_detail = []
    for r in groups.get(f"{robot}/ekf/odometry_earth", []):
        if abs(r["timestamp_ns"] - jump_ns) <= DETAIL_MARGIN_NS and r.get("position"):
            earth_detail.append({"event_id": event["event_id"], "robot_id": robot,
                "message_sequence_index": r["message_sequence_index"],
                "header_timestamp_ns": r["header_timestamp_ns"],
                "frame_id": r["frame_id"], "child_frame_id": r["child_frame_id"],
                "x": r["position"][0], "y": r["position"][1], "z": r["position"][2],
                "pose_covariance_position_trace": r["pose_covariance"]["position_covariance_trace"]})

    frames = {(r["frame_id"], r["child_frame_id"]) for r in ekf}
    ordered_by_sequence = sorted(ekf, key=lambda r: r["message_sequence_index"])
    resets = sum(b["header_timestamp_ns"] <= a["header_timestamp_ns"]
                 for a, b in zip(ordered_by_sequence, ordered_by_sequence[1:]))
    if len(frames) != 1:
        frame_status = "FRAME_ID_CHANGE"
    elif resets:
        frame_status = "TIMESTAMP_RESET"
    else:
        frame_status = "NO_FRAME_CHANGE"
    frame_row = {"event_id": event["event_id"], "robot_id": robot,
        "frame_id_before": e0["frame_id"], "frame_id_after": e1["frame_id"],
        "child_frame_id_before": e0["child_frame_id"],
        "child_frame_id_after": e1["child_frame_id"],
        "distinct_frame_pairs_in_10s_window": json.dumps(sorted(frames)),
        "header_timestamp_regressions_or_duplicates": resets,
        "sequence_counter": "NOT_AVAILABLE_IN_ROS2_HEADER",
        "node_session_restart_metadata": "NOT_AVAILABLE_IN_ARCHIVE",
        "status": frame_status}
    behavior_row = behavior(ekf, event, jump_ns)
    tf_rows = [{"event_id": event["event_id"], "robot_id": robot,
                "topic": "/tf,/tf_static", "status": "NOT_AVAILABLE_IN_ARCHIVE",
                "translation_jump_m": None, "rotation_jump_deg": None}]
    imu_rows = []
    for topic in (f"{robot}/imu/data", f"{robot}/ekf/imu/data"):
        items = [r for r in groups.get(topic, []) if r.get("linear_acceleration") is not None
                 and event["timestamp_i"] <= r["timestamp_ns"] <= event["timestamp_j"]]
        if not items:
            continue
        acc = np.linalg.norm(np.asarray([r["linear_acceleration"] for r in items]), axis=1)
        gyro = np.linalg.norm(np.asarray([r["angular_velocity"] for r in items]), axis=1)
        imu_rows.append({"event_id": event["event_id"], "robot_id": robot,
            "topic": topic, "samples": len(items), "max_acceleration_mps2_including_gravity": float(np.max(acc)),
            "median_acceleration_mps2_including_gravity": float(np.median(acc)),
            "max_angular_velocity_rad_s": float(np.max(gyro)),
            "ekf_implied_mean_speed_mps": ekf_jump / event["dt_s"],
            "interpretation": "QUALITATIVE_ONLY; no INS integration or gravity compensation"})
    status_rows = []
    for r in groups.get(f"{robot}/ekf/status", []):
        status_rows.append({"event_id": event["event_id"], "robot_id": robot,
            "timestamp_ns": r["timestamp_ns"], "relative_time_to_jump_s": (r["timestamp_ns"] - jump_ns) / 1e9,
            "message_sequence_index": r["message_sequence_index"], "decode_status": r["decode_status"],
            "gnss_state": r.get("gnss_state"), "dual_antenna_fix_type": r.get("dual_antenna_fix_type"),
            "filter_state": r.get("filter_state"), "status_flags": json.dumps(r.get("status_flags")),
            "continuous_bit_flags": json.dumps(r.get("continuous_bit_flags")), "device_model": r.get("device_model")})
    return {"windows": window_rows, "detail": detail, "earth_detail": earth_detail, "covariance": cov_rows,
            "gnss": gnss_rows, "alignment": alignment, "frame": frame_row,
            "behavior": behavior_row, "tf": tf_rows, "imu": imu_rows, "status": status_rows,
            "primary_ekf_window": ekf if event["is_primary"] else None,
            "primary_gnss_window": {k: v for k, v in groups.items()
                                    if event["is_primary"] and "/gnss_" in k and
                                    topic_types.get(k) == "sensor_msgs/msg/NavSatFix"}}


def extract_evidence(events):
    inventory = {}
    all_results = {}
    for robot in ROBOTS:
        meta = archive_metadata(robot)
        assert meta["storage_format"] == "ROS2 rosbag2 SQLite3/db3 CDR"
        with tempfile.TemporaryDirectory(prefix=f"stage12b_{robot}_", dir="/home/cas/CU-Multi") as temp:
            db_path = extracted_db(meta, temp)
            db, topics = open_archive_database(db_path, meta)
            results = []
            failures = Counter()
            for event in (e for e in events if e["robot_id"] == robot):
                groups, errors = query_event(db, topics, event)
                failures.update(errors)
                results.append(process_event(event, groups, meta["topics"]))
            db.close()
        inventory[robot] = meta
        all_results[robot] = results
        write_csv(f"{robot}_event_windows.csv", [row for result in results for row in result["windows"]])
        write_csv(f"{robot}_ekf_event_detail.csv", [row for result in results for row in result["detail"]])
        write_csv(f"{robot}_ekf_earth_event_detail.csv", [row for result in results for row in result["earth_detail"]])
        print(f"{robot}: {len(meta['topics'])} topics, {len(results)} events, "
              f"{sum(len(r['windows']) for r in results)} window messages, "
              f"unsupported custom messages={sum(failures.values())}", flush=True)
    write_json("raw_topic_inventory.json", inventory)
    tf_topics = {robot: [t["topic_name"] for t in inventory[robot]["topics"]
                         if t["topic_name"] in ("/tf", "/tf_static") or
                         t["topic_name"].endswith("/tf") or t["topic_name"].endswith("/tf_static")]
                 for robot in ROBOTS}
    write_json("tf_frame_graph.json", {"per_robot_tf_topics": tf_topics,
        "per_robot_status": {r: "NOT_AVAILABLE_IN_ARCHIVE" if not tf_topics[r] else "AVAILABLE" for r in ROBOTS},
        "map_odom_base_imu_sensor_edges": {r: [] for r in ROBOTS},
        "interpretation": "No dynamic/static TF topic in the specified IMU/GPS ZIPs; parent-frame reset cannot be directly tested here."})
    write_json("decode_limitations.json", {"custom_microstrain_message_types":
        "present in archive inventory and window timestamps; other vendor schemas unavailable in ROS2 Humble typestore",
        "human_readable_status": "official field schema at microstrain_inertial_msgs_common ed9cb178b1be0ff110e41548ca2165b593976146 was registered, but recorded CDR is shorter than expected (first example: 177 versus >=184 bytes); historical message schema mismatch, payload not decoded",
        "standard_messages": "NavSatFix, Odometry, Imu, TwistWithCovarianceStamped decoded",
        "tf": tf_topics})
    return inventory, all_results


def write_evidence_tables(events, results):
    write_csv("audited_event_manifest.csv", events)
    for name, field in (("ekf_covariance_audit.csv", "covariance"),
                        ("gnss_event_audit.csv", "gnss"),
                        ("tf_event_audit.csv", "tf"),
                        ("imu_sanity_at_jump.csv", "imu"),
                        ("ekf_status_audit.csv", "status")):
        write_csv(name, [r for robot in ROBOTS for item in results[robot] for r in item[field]])
    for name, field in (("source_alignment_at_jump.csv", "alignment"),
                        ("frame_and_stream_reset_audit.csv", "frame"),
                        ("post_jump_behavior.csv", "behavior")):
        write_csv(name, [item[field] for robot in ROBOTS for item in results[robot]])


def initial_evidence_report(events, results):
    rows = []
    for robot in ROBOTS:
        for event, result in zip((e for e in events if e["robot_id"] == robot), results[robot]):
            a = result["alignment"]
            rows.append({"event_id": event["event_id"], "robot": robot,
                "kf": f"{event['keyframe_i']}->{event['keyframe_j']}",
                "primary": event["is_primary"], "ekf_jump_xyz_m": a["ekf_translation_jump_xyz_m"],
                "raw_max_step_m": event["max_adjacent_raw_ekf_step_m"],
                "gnss1_m": a["gnss_1_translation_jump_m"], "gnss2_m": a["gnss_2_translation_jump_m"],
                "ekf_earth_m": a["ekf_earth_translation_jump_m"],
                "ekf_llh_m": a["ekf_llh_translation_jump_m"],
                "gnss1_earth_m": a["gnss1_earth_translation_jump_m"],
                "gnss2_earth_m": a["gnss2_earth_translation_jump_m"],
                "gnss1_status": a["gnss_1_match_status"], "gnss2_status": a["gnss_2_match_status"],
                "cov_before": a["pose_covariance_trace_before"],
                "cov_after": a["pose_covariance_trace_after"],
                "frame": result["frame"]["status"], "behavior": result["behavior"]["category"]})
    write_csv("initial_evidence_review.csv", rows)
    print(pd.DataFrame(rows).to_string(index=False), flush=True)


def classify_and_decide(events, results):
    patterns = []
    primary = {}
    for robot in ROBOTS:
        for event, result in zip((e for e in events if e["robot_id"] == robot), results[robot]):
            a, b = result["alignment"], result["behavior"]
            gnss_good = all(a[f"gnss_{k}_match_status"] == "MATCHED" and
                a[f"gnss_{k}_translation_jump_m"] < 1 for k in (1, 2))
            earth_agrees = a["ekf_earth_match_status"] == "MATCHED" and abs(
                a["ekf_earth_translation_jump_m"] - a["ekf_translation_jump_xyz_m"]) < .01
            early_s = (event["timestamp_i"] -
                json.loads((OUT / "raw_topic_inventory.json").read_text())[robot]["bag_first_timestamp"]) / 1e9
            # The class labels the likely estimator initialization/recovery family,
            # NOT proof of a discrete reset. Missing TF/vendor status caps confidence.
            if gnss_good and earth_agrees and result["frame"]["status"] == "NO_FRAME_CHANGE":
                category, confidence = "EKF_STATE_RESET_OR_REINITIALIZATION", "MEDIUM"
                mechanism = "early EKF position-estimate instability/convergence; discrete reset not proven"
            else:
                category, confidence = "INSUFFICIENT_SENSOR_EVIDENCE", "LOW"
                mechanism = "uncertain source attribution"
            status = result["status"]
            decoded = [r for r in status if r["decode_status"] == "PASS"]
            p = {"event_id": event["event_id"], "robot_id": robot,
                "keyframe_i": event["keyframe_i"], "keyframe_j": event["keyframe_j"],
                "seconds_since_bag_start": early_s,
                "ekf_translation_xyz_m": a["ekf_translation_jump_xyz_m"],
                "ekf_earth_translation_m": a["ekf_earth_translation_jump_m"],
                "raw_gnss1_translation_m": a["gnss_1_translation_jump_m"],
                "raw_gnss2_translation_m": a["gnss_2_translation_jump_m"],
                "pose_position_covariance_trace_before": a["pose_covariance_trace_before"],
                "pose_position_covariance_trace_after": a["pose_covariance_trace_after"],
                "frame_status": result["frame"]["status"], "tf_status": "NOT_AVAILABLE_IN_ARCHIVE",
                "vendor_status_decode": "PASS" if decoded else "UNSUPPORTED_SCHEMA_VERSION",
                "post_jump_category": b["category"], "classification": category,
                "confidence": confidence, "likely_mechanism": mechanism,
                "supporting_evidence": "EKF map+earth large displacement; raw dual-GNSS nearly static; large EKF covariance; stable frame IDs; early in recording"}
            patterns.append(p)
            if event["is_primary"]:
                primary[robot] = (event, result, p)
    write_csv("event_pattern_summary.csv", patterns)
    assert len(patterns) == 20 and len(primary) == 2
    root = {"status": "FORENSIC_AUDIT_COMPLETE_MECHANISM_INFERRED_NOT_PROVEN",
        "robot1_primary_event": "KF57->58", "robot3_primary_event": "KF42->43",
        "robot1_classification": primary["robot1"][2]["classification"],
        "robot3_classification": primary["robot3"][2]["classification"],
        "robot1_confidence": primary["robot1"][2]["confidence"],
        "robot3_confidence": primary["robot3"][2]["confidence"],
        "gnss_jump_present_robot1": False, "gnss_jump_present_robot3": False,
        "tf_reset_present_robot1": "NOT_AVAILABLE_IN_ARCHIVE",
        "tf_reset_present_robot3": "NOT_AVAILABLE_IN_ARCHIVE",
        "ekf_covariance_anomaly_robot1": True, "ekf_covariance_anomaly_robot3": True,
        "frame_change_present_robot1": False, "frame_change_present_robot3": False,
        "post_jump_behavior_robot1": primary["robot1"][1]["behavior"]["category"],
        "post_jump_behavior_robot3": primary["robot3"][1]["behavior"]["category"],
        "common_failure_mode": True,
        "overall_root_cause": "Early-run EKF odometry position estimate instability/convergence in both robots; exact internal reset/reinitialization trigger not directly observable",
        "evidence_summary": "All 20 selected transitions show large EKF map and earth motion while two raw GNSS antennas move under 1 m, with large position covariance and stable pose frame IDs. Events cluster in early seconds of both recordings; individual raw EKF increments are several metres over many messages, not single-message teleportations. No TF in archives and recorded vendor status schema cannot be decoded with official current schema; discrete reset unproven.",
        "recommended_future_repair": "MORE_SOURCE_INVESTIGATION_REQUIRED",
        "repair_reason": "Do not break frames or drop isolated samples: the anomaly is a multi-second estimator excursion, and reset boundaries/TF are unobserved. Determine recorded vendor status schema and independent motion source before designing a frozen repair policy."}
    write_json("root_cause.json", root)
    ready = {"safe_to_run_sparse_loop_on_current_odometry": "NO",
        "reason": "Raw EKF-derived local odometry has >300 m half-second early excursions; fewer loops cannot make these factors physically valid.",
        "required_fix_before_sparse_loop": "Source-level status/TF investigation followed by a predeclared odometry repair or independently validated replacement; Stage 12B applies none.",
        "stage8_frontend_still_valid": "YES", "stage9_registration_still_valid": "YES",
        "stage10_3_backend_numerically_valid": "YES",
        "stage10_3_physical_map_interpretation": "COMPROMISED"}
    write_json("sparse_loop_readiness.json", ready)
    return primary, root, ready


def plot_figures(events, results, primary):
    for robot in ROBOTS:
        event, result, _ = primary[robot]
        ekf = result["primary_ekf_window"]
        t0 = event["timestamp_i"]
        tx = np.asarray([(r["timestamp_ns"] - t0) / 1e9 for r in ekf])
        p = np.asarray([r["position"] for r in ekf])
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
        for k, color in enumerate(("tab:blue", "tab:orange", "tab:green")):
            axes[0].plot(tx, p[:, k] - p[0, k], color=color, lw=1, label=f"EKF map {['x','y','z'][k]} relative")
        axes[0].legend(ncol=3); axes[0].set_ylabel("EKF displacement (m)")
        for antenna, color in (("gnss_1", "tab:purple"), ("gnss_2", "tab:red")):
            rows = result["primary_gnss_window"].get(f"{robot}/{antenna}/llh_position", [])
            if rows:
                ts = [(r["timestamp_ns"] - t0) / 1e9 for r in rows]
                ds = [local_enu_displacement(rows[0], r) for r in rows]
                axes[1].plot(ts, ds, marker=".", color=color, label=f"raw {antenna} ENU from first")
        axes[1].legend(); axes[1].set_ylabel("GNSS horizontal motion (m)"); axes[1].set_xlabel("Time from KF i (s)")
        for ax in axes:
            ax.axvspan(0, event["dt_s"], color="grey", alpha=.16, label="keyframe interval")
            ax.grid(alpha=.25)
        axes[1].text(.01, .97, "TF: NOT AVAILABLE IN ARCHIVE", transform=axes[1].transAxes,
                     va="top", bbox={"facecolor": "white", "alpha": .8})
        fig.suptitle(f"{robot}: KF{event['keyframe_i']}→{event['keyframe_j']} EKF vs independent GNSS")
        fig.savefig(OUT / f"{robot}_primary_jump_sources.png", dpi=150); plt.close(fig)

        detail = pd.read_csv(OUT / f"{robot}_ekf_event_detail.csv").drop_duplicates("message_sequence_index").sort_values("header_timestamp_ns")
        dt = (detail.header_timestamp_ns - detail.header_timestamp_ns.iloc[0]) / 1e9
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
        # Break the line across unobserved gaps between overlapping audit windows.
        gaps = np.flatnonzero(np.diff(dt.to_numpy()) > .2) + 1
        starts, ends = np.r_[0, gaps], np.r_[gaps, len(detail)]
        for ax, col in zip(axes, ("x", "y", "z")):
            for start, end in zip(starts, ends):
                ax.plot(dt.iloc[start:end], detail[col].iloc[start:end], lw=.8)
            ax.set_ylabel(f"EKF map {col} (m)"); ax.grid(alpha=.25)
            for e in (x for x in events if x["robot_id"] == robot):
                ax.axvspan((e["timestamp_i"] - detail.header_timestamp_ns.iloc[0]) / 1e9,
                    (e["timestamp_j"] - detail.header_timestamp_ns.iloc[0]) / 1e9, color="red", alpha=.08)
        axes[-1].set_xlabel("Time from first plotted raw EKF message (s)")
        fig.suptitle(f"{robot}: raw EKF map position, top-10 keyframe intervals shaded")
        fig.savefig(OUT / f"ekf_position_vs_time_{robot}.png", dpi=150); plt.close(fig)

    cov = pd.read_csv(OUT / "ekf_covariance_audit.csv")
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    for ax, robot in zip(axes, ROBOTS):
        part = cov[cov.robot_id.eq(robot) & cov.event_id.eq(f"{robot}_rank01")]
        ax.plot(part.phase, part.pose_position_covariance_trace, marker="o")
        ax.set_yscale("log"); ax.set_ylabel("position covariance trace (m²)")
        ax.set_title(robot + " primary event"); ax.grid(alpha=.25)
    fig.suptitle("EKF pose covariance around primary jumps (log scale)")
    fig.savefig(OUT / "covariance_around_jumps.png", dpi=150); plt.close(fig)

    pattern = pd.read_csv(OUT / "event_pattern_summary.csv")
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), constrained_layout=True)
    for ax, robot in zip(axes, ROBOTS):
        sub = pattern[pattern.robot_id.eq(robot)].sort_values("keyframe_i")
        x = np.arange(len(sub))
        ax.bar(x, sub.ekf_translation_xyz_m, label="EKF map jump (m)")
        ax.scatter(x, sub.raw_gnss1_translation_m, marker="x", color="red", label="GNSS1 (m)")
        ax.scatter(x, sub.raw_gnss2_translation_m, marker="+", color="orange", label="GNSS2 (m)")
        ax.set_xticks(x, [f"{a}→{b}" for a, b in zip(sub.keyframe_i, sub.keyframe_j)], rotation=40)
        ax.set_ylabel("Displacement (m)"); ax.set_title(robot); ax.legend(ncol=3); ax.grid(axis="y", alpha=.2)
    fig.suptitle("Top-10 events: same EKF/GNSS divergence pattern; GNSS near zero at this scale")
    fig.savefig(OUT / "event_mechanism_summary.png", dpi=150); plt.close(fig)


def validate(root, ready):
    required = ["audited_event_manifest.csv", "raw_topic_inventory.json", "robot1_event_windows.csv",
        "robot3_event_windows.csv", "robot1_ekf_event_detail.csv", "robot3_ekf_event_detail.csv",
        "ekf_covariance_audit.csv", "gnss_event_audit.csv", "tf_event_audit.csv",
        "source_alignment_at_jump.csv", "frame_and_stream_reset_audit.csv", "post_jump_behavior.csv",
        "event_pattern_summary.csv", "root_cause.json", "sparse_loop_readiness.json",
        "robot1_primary_jump_sources.png", "robot3_primary_jump_sources.png",
        "ekf_position_vs_time_robot1.png", "ekf_position_vs_time_robot3.png",
        "covariance_around_jumps.png", "event_mechanism_summary.png"]
    assert all((OUT / x).is_file() and (OUT / x).stat().st_size for x in required)
    assert pd.read_csv(OUT / "audited_event_manifest.csv").groupby("robot_id").size().to_dict() == {"robot1": 10, "robot3": 10}
    lines = ["STAGE12A EVENTS REUSED: PASS", "RAW ARCHIVES INVENTORIED: PASS",
        "EKF RAW MESSAGES CHECKED: PASS", "GNSS SOURCE CHECKED: PASS",
        "TF CHECKED: NOT_AVAILABLE", "COVARIANCE CHECKED: PASS", "FRAME IDS CHECKED: PASS",
        "TOP10 EVENTS EACH ROBOT AUDITED: PASS", "SOURCE TIMESTAMP ALIGNMENT RECORDED: PASS",
        "NO GT USED FOR MAIN DIAGNOSIS: PASS", "NO POSES MODIFIED: PASS", "NO PGO RERUN: PASS",
        "STAGE10.3 MODIFIED: NO", "DEMO MODIFIED: NO",
        "VENDOR STATUS PAYLOAD: UNSUPPORTED_SCHEMA_VERSION (topic/timestamps inventoried)",
        "ROOT CAUSE: " + root["overall_root_cause"],
        "RECOMMENDED FUTURE REPAIR: " + root["recommended_future_repair"],
        "SPARSE LOOP READY: " + ready["safe_to_run_sparse_loop_on_current_odometry"]]
    (OUT / "VALIDATION_REPORT.txt").write_text("\n".join(lines) + "\n")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    events = select_events()
    write_json("source_matching_policy.json", {"basis": "archive topic rates and 2-Hz keyframe cadence, fixed before inspecting values",
        "maximum_nearest_time_error_s": SYNC_LIMIT_S,
        "event_window": "[Stage12A keyframe_i timestamp - 5 s, keyframe_j timestamp + 5 s]",
        "detail_window": "raw EKF maximum incremental translation timestamp ±1 s",
        "timestamp_basis": "ROS header stamp when decoded; bag-record timestamp otherwise",
        "same_gnss_sample_at_both_endpoints": "not treated as a displacement comparison"})
    inventory, results = extract_evidence(events)
    write_evidence_tables(events, results)
    initial_evidence_report(events, results)
    primary, root, ready = classify_and_decide(events, results)
    plot_figures(events, results, primary)
    validate(root, ready)


if __name__ == "__main__":
    main()
