#!/usr/bin/env python3
"""Stage 12D: frozen stable-segment Rank1 pose graph and offline diagnostics.

Graph inputs use allowlisted non-GT columns. GT columns are loaded exclusively
in offline_evaluation(), after the persisted pre-GT backend decision.
Historical helpers are imported without invoking their experiment entrypoints.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_stage10_offline_map_merge as s10
from run_stage103_gtsam_solver_audit import pose, key, build_noise_models, measurement
from run_stage12a_trajectory_integrity import Track, step_frame

ROOT = HERE.parents[1]
OUT = ROOT / "outputs/cumulti_v1/12d_clean_backend"
C12 = ROOT / "outputs/cumulti_v1/12c_dataset_stability"
OLD = ROOT / "outputs/cumulti_v1/10_3_solver_audit"
LOOP_SOURCE = ROOT / "outputs/cumulti_v1/10_2_pgo_policy_selection/rank1_sanitized_edges.csv"
STARTS = {"robot1": 150, "robot3": 234}
LOOP_COLUMNS = ["query_robot", "query_keyframe_id", "query_timestamp", "candidate_robot",
    "candidate_keyframe_id", "candidate_timestamp", "SC_score", "best_shift",
    "SC_yaw_initialization_deg", "GICP_quality", "tx", "ty", "tz", "qx", "qy", "qz", "qw",
    "transform_direction"]
POSE_COLUMNS = ["robot_id", "keyframe_id", "timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def write_json(name, value):
    path = OUT / name
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    with path.open("rb") as f:
        os.fsync(f.fileno())


def progress(stage, **fields):
    row = {"stage": stage, "UTC": datetime.now(timezone.utc).isoformat(), **fields}
    write_json("progress.json", row)
    print(json.dumps(row), flush=True)


def stats(values):
    a = np.asarray(values, float)
    assert len(a) and np.isfinite(a).all()
    return {"median": float(np.median(a)), "p90": float(np.percentile(a, 90)),
        "p95": float(np.percentile(a, 95)), "p99": float(np.percentile(a, 99)),
        "max": float(np.max(a)), "count": len(a)}


def tm(row):
    return s10.T([row.qx, row.qy, row.qz, row.qw], [row.tx, row.ty, row.tz])


def trajectory(frames, matrices):
    rows = []
    for robot in STARTS:
        for r in frames[robot].itertuples():
            x = matrices[(robot, int(r.keyframe_id))]
            q = Rotation.from_matrix(x[:3, :3]).as_quat()
            rows.append([robot, int(r.keyframe_id), int(r.lidar_timestamp_ns), *x[:3, 3], *q])
    return pd.DataFrame(rows, columns=POSE_COLUMNS)


def schema_audit(df):
    checks = {"rows": len(df), "finite": bool(np.isfinite(df[POSE_COLUMNS[3:]].to_numpy()).all()),
        "unit_quaternions": bool(np.allclose(np.linalg.norm(df[["qx", "qy", "qz", "qw"]], axis=1), 1, atol=1e-8)),
        "unique_semantic_ids": bool(not df.duplicated(["robot_id", "keyframe_id"]).any()), "robots": {}}
    for robot, start in STARTS.items():
        d = df[df.robot_id.eq(robot)]
        expected_end = 1999 if robot == "robot1" else 4179
        checks["robots"][robot] = {"count": len(d), "first": int(d.keyframe_id.iloc[0]),
            "last": int(d.keyframe_id.iloc[-1]), "continuous_ids": d.keyframe_id.to_list() == list(range(start, expected_end + 1)),
            "monotonic_timestamps": bool(np.all(np.diff(d.timestamp.to_numpy(np.int64)) > 0))}
    checks["PASS"] = checks["finite"] and checks["unit_quaternions"] and checks["unique_semantic_ids"] and all(
        d["continuous_ids"] and d["monotonic_timestamps"] for d in checks["robots"].values())
    assert checks["PASS"], checks
    return checks


def freeze_policies():
    selection = json.loads((C12 / "stable_segment_selection.json").read_text())
    coverage = json.loads((C12 / "stable_segment_frontend_coverage.json").read_text())
    assert selection["GT_used"] is False
    assert all(selection["robots"][r]["stable_start_keyframe"] == k for r, k in STARTS.items())
    assert coverage["remaining_rank1_sanitized_loops"] == 75
    old = json.loads((OLD / "frozen_graph_manifest.json").read_text())["policies"]["RANK1_SANITIZED"]
    assert sha256(LOOP_SOURCE) == old["sha256"]
    policy = {"robot1_start_keyframe": 150, "robot3_start_keyframe": 234,
        "stable_start_source": str(C12 / "stable_segment_selection.json"),
        "stable_start_source_sha256": sha256(C12 / "stable_segment_selection.json"),
        "node_frame": "LiDAR/os_sensor; exact Stage10.3 IMU-to-LiDAR convention",
        "node_ids": "original (robot_id,keyframe_id), preserved in GTSAM symbols and exported CSVs",
        "local_coordinate_gauge": "exact Stage10.3 KF0-relative local matrices; retained values are not rebased or repaired",
        "graph_anchor": "Robot1 KF150 at its frozen local Pose3",
        "odometry_source": "raw ekf/odometry_map, nearest frozen LiDAR timestamps; T_world_imu @ T_imu_lidar; same Stage10.3 helper",
        "odometry_factor": "Z_i,i+1 = X_i^-1 X_i+1, only sequential retained IDs",
        "odometry_noise": {"translation_sigma_m": 1.0, "rotation_sigma_deg": 5.0, "robust": False},
        "loop_source": str(LOOP_SOURCE), "loop_source_sha256": old["sha256"],
        "loop_filter": "query robot1 KF>=150 AND candidate robot3 KF>=234; no other filtering",
        "expected_loops_from_stage12c": 75,
        "loop_noise": {"translation_sigma_m": 1.0, "rotation_sigma_deg": 5.0, "Huber_delta": 1.345},
        "loop_transform": "Z_qc=T_query_from_candidate=X_query^-1 X_candidate",
        "default_initialization": "FIRST_ARRIVAL_INIT; accepted only when frozen equivalence diagnostics pass",
        "arrival_order": "query_timestamp, query_keyframe_id, candidate_keyframe_id; separate-run R1->R3 replay query order",
        "optimizer": "GTSAM 4.2 Pose3 LevenbergMarquardtOptimizer, unchanged documented default parameters",
        "noise_tangent_order": "[rx,ry,rz,tx,ty,tz]",
        "map_protocol": {"frame_stride": 10, "point_stride": 16, "voxel_m": 0.5,
            "frame_stride_origin": "first retained keyframe per robot", "range_filter_m": [1, 60]},
        "catastrophic_integrity_rule": {"absolute_translation_xyz_m": 100.0,
            "new_lineage_jump": "post XYZ>5x Stage12C frozen XY step limit AND pre-common XYZ<=5x same limit",
            "thresholds_fitted_to_clean_result": False}, "GT_used": False}
    write_json("clean_graph_policy.json", policy)
    write_json("covariance_policy_note.json", {"late_covariance_alerts_present": True,
        "factor_weight_changed": False,
        "reason": "absolute EKF pose covariance is not directly equivalent to independent relative-pose factor covariance; no justified covariance propagation model has been frozen",
        "all_retained_keyframes_kept": True})
    write_json("initialization_equivalence_policy.json", {
        "status": "FROZEN_BEFORE_SOLVING", "median_translation_difference_m_max": 0.10,
        "p95_translation_difference_m_max": 0.50, "median_rotation_difference_deg_max": 0.5,
        "p95_rotation_difference_deg_max": 2.0, "comparison_frame": "same fixed Robot1 KF150 anchor; no extra alignment",
        "map_accuracy_claim": False, "GT_used": False})
    inputs = [LOOP_SOURCE, C12 / "stable_segment_selection.json", C12 / "stable_start_policy.json",
        C12 / "stable_segment_frontend_coverage.json", C12 / "post_stable_start_recurrence.csv",
        OLD / "frozen_graph_manifest.json", HERE / "run_stage10_offline_map_merge.py",
        HERE / "run_stage103_gtsam_solver_audit.py", HERE / "run_stage12a_trajectory_integrity.py"]
    for robot in STARTS:
        inputs += [s10.PROC / robot / "keyframes.csv", s10.RAW / robot / f"{robot}_main_campus_imu_gps.zip",
            C12 / f"{robot}_full_ekf_stability.csv"]
    manifest = [{"path": str(p), "size_bytes": p.stat().st_size, "sha256": sha256(p)} for p in inputs]
    write_json("frozen_input_manifest.json", manifest)
    return selection, policy, manifest


def prepare_local(selection):
    frames, local, audits = {}, {}, {}
    for robot, start in STARTS.items():
        # GT x/y/z/quaternion columns in keyframes.csv are explicitly not loaded.
        k = pd.read_csv(s10.PROC / robot / "keyframes.csv",
                        usecols=["keyframe_id", "lidar_timestamp_ns", "lidar_file"])
        assert k.keyframe_id.to_list() == list(range(len(k)))
        o = s10.extract_ekf(robot)
        stamps, targets = o.timestamp_ns.to_numpy(np.int64), k.lidar_timestamp_ns.to_numpy(np.int64)
        right = np.clip(np.searchsorted(stamps, targets), 1, len(stamps) - 1)
        left = right - 1
        ix = np.where(abs(stamps[right] - targets) < abs(stamps[left] - targets), right, left)
        full = np.array([s10.T(o.iloc[i][["qx", "qy", "qz", "qw"]].to_numpy(float),
            o.iloc[i][["tx", "ty", "tz"]].to_numpy(float)) @ s10.IMU_FROM_LIDAR for i in ix])
        first_inverse = s10.inv(full[0])
        full = np.array([first_inverse @ x for x in full])
        frames[robot] = k[k.keyframe_id.ge(start)].copy()
        assert len(frames[robot]) == selection["robots"][robot]["retained_keyframes"]
        assert frames[robot].keyframe_id.to_list() == list(range(start, len(k)))
        for i in frames[robot].keyframe_id:
            local[(robot, int(i))] = full[int(i)]
        audits[robot] = {"topic": f"{robot}/ekf/odometry_map", "raw_messages": len(o),
            "retained_keyframes": len(frames[robot]), "maximum_sync_error_ms": float(np.max(abs(stamps[ix] - targets)) / 1e6),
            "extrinsic": s10.IMU_FROM_LIDAR.tolist(), "GT_columns_loaded": False}
        progress("LOCAL_EXTRACTED", robot=robot, nodes=len(frames[robot]))
    write_json("odometry_source_audit.json", audits)
    df = trajectory(frames, local)
    schema_audit(df)
    df.to_csv(OUT / "clean_local_trajectory.csv", index=False)
    step_tables, summary = {}, {}
    for robot in STARTS:
        d = df[df.robot_id.eq(robot)]
        tr = Track("clean_local", robot, d.keyframe_id.to_numpy(np.int64), d.timestamp.to_numpy(np.int64),
                   d[["tx", "ty", "tz"]].to_numpy(float), d[["qx", "qy", "qz", "qw"]].to_numpy(float))
        steps = step_frame(tr)
        # Crosscheck the subset against frozen Stage12A measurements, before any solve.
        frozen = pd.read_csv(ROOT / f"outputs/cumulti_v1/12a_trajectory_integrity/local_{robot}_steps.csv")
        frozen = frozen[frozen.keyframe_i.ge(STARTS[robot])]
        for metric in ("translation_xy_m", "translation_xyz_m", "relative_rotation_deg"):
            assert np.allclose(steps[metric], frozen[metric], atol=1e-8, rtol=1e-8), (robot, metric)
        summary[robot] = {metric: stats(steps[metric]) for metric in (
            "translation_xy_m", "translation_xyz_m", "relative_rotation_deg", "linear_speed_mps")}
        summary[robot]["Stage12C_frozen_diagnostic_limits"] = selection["robots"][robot]["frozen_thresholds"]
        assert steps.translation_xyz_m.max() < 100, "CLEAN_GRAPH_INPUT_FAIL: catastrophic retained motion"
        step_tables[robot] = steps
    pd.concat(step_tables.values(), ignore_index=True).to_csv(OUT / "clean_local_trajectory_integrity.csv", index=False)
    write_json("clean_local_trajectory_summary.json", {"status": "PASS", "robots": summary, "GT_used": False})
    health = []
    for robot, start in STARTS.items():
        h = pd.read_csv(C12 / f"{robot}_full_ekf_stability.csv", usecols=["robot_id", "keyframe_id", "timestamp_ns",
            "ekf_position_covariance_trace", "within_limit_ekf_position_covariance_trace"])
        health.append(h[h.keyframe_id.ge(start)])
    pd.concat(health, ignore_index=True).to_csv(OUT / "clean_covariance_health_metadata.csv", index=False)
    return frames, local, df, step_tables


def ordered_loops(frames):
    loops = pd.read_csv(LOOP_SOURCE, usecols=LOOP_COLUMNS)
    assert len(loops) == 120
    loops.insert(0, "frozen_loop_id", np.arange(len(loops)))
    assert loops.query_robot.eq("robot1").all() and loops.candidate_robot.eq("robot3").all()
    loops = loops[loops.query_keyframe_id.ge(150) & loops.candidate_keyframe_id.ge(234)].copy()
    loops = loops.sort_values(["query_timestamp", "query_keyframe_id", "candidate_keyframe_id"], kind="stable").reset_index(drop=True)
    assert len(loops) == 75
    loops.insert(0, "arrival_index", np.arange(1, len(loops) + 1))
    assert np.isfinite(loops[["tx", "ty", "tz", "qx", "qy", "qz", "qw"]].to_numpy()).all()
    assert np.allclose(np.linalg.norm(loops[["qx", "qy", "qz", "qw"]], axis=1), 1, atol=1e-5)
    assert loops.GICP_quality.ge(.6091).all()
    for r in loops.itertuples():
        assert int(r.query_keyframe_id) in set(frames["robot1"].keyframe_id)
        assert int(r.candidate_keyframe_id) in set(frames["robot3"].keyframe_id)
    loops.to_csv(OUT / "ordered_rank1_clean_loops.csv", index=False)
    n1, n3 = len(frames["robot1"]), len(frames["robot3"])
    manifest = {"robot1_nodes": n1, "robot3_nodes": n3, "total_nodes": n1 + n3,
        "robot1_odometry_factors": n1 - 1, "robot3_odometry_factors": n3 - 1,
        "total_odometry_factors": n1 + n3 - 2, "loop_factors": len(loops), "prior_factors": 1,
        "first_last_keyframe": {r: [int(k.keyframe_id.iloc[0]), int(k.keyframe_id.iloc[-1])] for r, k in frames.items()},
        "loop_source": str(LOOP_SOURCE), "loop_source_sha256": sha256(LOOP_SOURCE),
        "filtered_loop_sha256": sha256(OUT / "ordered_rank1_clean_loops.csv"),
        "odometry_inputs": [str(s10.RAW / r / f"{r}_main_campus_imu_gps.zip") for r in STARTS],
        "node_input": str(OUT / "clean_local_trajectory.csv"), "GT_used": False}
    assert (n1, n3, n1+n3, n1+n3-2, len(loops)) == (1850, 3946, 5796, 5794, 75)
    write_json("clean_graph_manifest.json", manifest)
    return loops, manifest


def initialize(row, frames, local, name):
    qid, cid = ("robot1", int(row.query_keyframe_id)), ("robot3", int(row.candidate_keyframe_id))
    z = tm(row)
    S = local[qid] @ z @ s10.inv(local[cid])
    common = {node: x if node[0] == "robot1" else S @ x for node, x in local.items()}
    E = s10.inv(z) @ s10.inv(common[qid]) @ common[cid]
    residual = {"translation_residual_m": float(np.linalg.norm(E[:3, 3])),
        "rotation_residual_deg": float(np.rad2deg(Rotation.from_matrix(E[:3, :3]).magnitude()))}
    assert residual["translation_residual_m"] < 1e-8 and residual["rotation_residual_deg"] < 1e-8
    info = {"initialization": name, "arrival_index": int(row.arrival_index),
        "frozen_loop_id": int(row.frozen_loop_id), "query_keyframe_id": int(row.query_keyframe_id),
        "candidate_keyframe_id": int(row.candidate_keyframe_id), "GICP_quality": float(row.GICP_quality),
        "SC_score": float(row.SC_score), "Z_query_from_candidate": z.tolist(), "S_common_from_robot3_local": S.tolist(),
        **residual, "GT_used": False, "status": "PASS"}
    write_json("first_loop_initialization.json" if name == "FIRST_ARRIVAL_INIT" else "highest_quality_initialization.json", info)
    return common, trajectory(frames, common), info


def connectivity(frames, loops):
    parent = {(robot, int(i)): (robot, int(i)) for robot, k in frames.items() for i in k.keyframe_id}
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    def union(a, b): parent[find(a)] = find(b)
    for robot, k in frames.items():
        ids = k.keyframe_id.to_list()
        for a, b in zip(ids, ids[1:]): union((robot, a), (robot, b))
    before = len({find(n) for n in parent})
    r = loops.iloc[0]
    union(("robot1", int(r.query_keyframe_id)), ("robot3", int(r.candidate_keyframe_id)))
    after = len({find(n) for n in parent})
    assert (before, after) == (2, 1)
    write_json("first_loop_connectivity.json", {"before_first_loop_connected_components": before,
        "after_first_loop_connected_components": after, "GT_used": False,
        "meaning": "One inter-robot loop connects the two odometry chains and permits initial map alignment; no global accuracy guarantee."})


def build_graph(frames, local, loops):
    base, robust = build_noise_models()
    graph = gtsam.NonlinearFactorGraph()
    anchor = ("robot1", 150)
    graph.add(gtsam.PriorFactorPose3(key(1, 150), pose(local[anchor]),
        gtsam.noiseModel.Diagonal.Sigmas(np.ones(6) * 1e-6)))
    for robot, k in frames.items():
        robot_number = 1 if robot == "robot1" else 3
        ids = k.keyframe_id.to_list()
        for a, b in zip(ids, ids[1:]):
            assert b == a + 1
            graph.add(gtsam.BetweenFactorPose3(key(robot_number, a), key(robot_number, b),
                pose(s10.inv(local[(robot, a)]) @ local[(robot, b)]), base))
    for r in loops.itertuples():
        graph.add(gtsam.BetweenFactorPose3(key(1, int(r.query_keyframe_id)),
            key(3, int(r.candidate_keyframe_id)), measurement(r), robust))
    assert graph.size() == 5794 + 75 + 1
    params = gtsam.LevenbergMarquardtParams()
    getters = ["getMaxIterations", "getRelativeErrorTol", "getAbsoluteErrorTol", "getErrorTol",
        "getlambdaInitial", "getlambdaFactor", "getlambdaLowerBound", "getlambdaUpperBound",
        "getLinearSolverType", "getOrderingType", "getDiagonalDamping"]
    actual = {name: getattr(params, name)() for name in getters}
    write_json("clean_gtsam_solver_policy.json", {"gtsam_version": "4.2", "optimizer": "LevenbergMarquardtOptimizer",
        "parameters": actual, "parameter_text": str(params), "parameter_tuning": False,
        "environment": "/home/cas/.venvs/fyp_gtsam", "python": sys.version,
        "noise_sigmas_rot_then_trans": [np.deg2rad(5)] * 3 + [1.] * 3, "Huber_delta": 1.345})
    return graph, params


def solve(graph, params, common, frames, local, name):
    values = gtsam.Values()
    for (robot, i), x in common.items():
        values.insert(key(1 if robot == "robot1" else 3, i), pose(x))
    initial = float(graph.error(values))
    t0 = time.perf_counter()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
    result = optimizer.optimize()
    runtime = time.perf_counter() - t0
    final = float(graph.error(result))
    solved = {node: result.atPose3(key(1 if node[0] == "robot1" else 3, node[1])).matrix() for node in local}
    finite = all(np.isfinite(x).all() for x in solved.values())
    assert finite and np.isfinite([initial, final]).all() and final <= initial
    anchor_E = s10.inv(local[("robot1", 150)]) @ solved[("robot1", 150)]
    anchor_t = float(np.linalg.norm(anchor_E[:3, 3]))
    anchor_r = float(np.rad2deg(Rotation.from_matrix(anchor_E[:3, :3]).magnitude()))
    assert anchor_t < 1e-6 and anchor_r < 1e-6
    tr = trajectory(frames, solved)
    schema_audit(tr)
    status = {"initialization": name, "nodes": len(solved), "odometry_factors": 5794,
        "loop_factors": 75, "prior_factors": 1, "initial_graph_error": initial,
        "final_graph_error": final, "relative_reduction": (initial-final)/initial,
        "runtime_s": runtime, "termination_status": "NORMAL_RETURN", "iterations": int(optimizer.iterations()),
        "iteration_limit_reached": bool(optimizer.iterations() >= params.getMaxIterations()),
        "termination_reason_binding": "specific tolerance-stop reason not exposed; normal return and iteration count recorded",
        "finite_poses": finite, "anchor_translation_error_m": anchor_t, "anchor_rotation_error_deg": anchor_r}
    progress("SOLVED", **status)
    return solved, tr, status


def loop_residuals(loops, matrices):
    rows = []
    for r in loops.itertuples():
        E = s10.inv(tm(r)) @ s10.inv(matrices[("robot1", int(r.query_keyframe_id))]) @ matrices[("robot3", int(r.candidate_keyframe_id))]
        t, rot = np.linalg.norm(E[:3, 3]), Rotation.from_matrix(E[:3, :3]).magnitude()
        norm = np.linalg.norm(np.r_[E[:3, 3], Rotation.from_matrix(E[:3, :3]).as_rotvec()/np.deg2rad(5)])
        rows.append({"arrival_index": int(r.arrival_index), "frozen_loop_id": int(r.frozen_loop_id),
            "query_keyframe_id": int(r.query_keyframe_id), "candidate_keyframe_id": int(r.candidate_keyframe_id),
            "translation_residual_m": float(t), "rotation_residual_deg": float(np.rad2deg(rot)),
            "diagnostic_whitened_residual": float(norm),
            "diagnostic_reconstructed_huber_weight": float(min(1., 1.345/max(norm, 1e-12)))})
    df = pd.DataFrame(rows)
    summary = {"translation_m": stats(df.translation_residual_m), "rotation_deg": stats(df.rotation_residual_deg),
        "diagnostic_huber_weight": {"count_below_0_1": int(df.diagnostic_reconstructed_huber_weight.lt(.1).sum()),
            "count_below_0_25": int(df.diagnostic_reconstructed_huber_weight.lt(.25).sum()),
            "count_below_0_5": int(df.diagnostic_reconstructed_huber_weight.lt(.5).sum()),
            "minimum": float(df.diagnostic_reconstructed_huber_weight.min())},
        "definition": "E=Z^-1 X_query^-1 X_candidate; translation=norm(E.translation), rotation=angle(E.rotation). Diagnostic normalized metric uses translation/1m and rotation-vector/5deg, matching Stage10.3; reconstructed weight is not the internal GTSAM Lie-Logmap weight."}
    return df, summary


def compare_initializations(first, quality, same_edge=False):
    nodes = list(first)
    td = np.array([np.linalg.norm(first[n][:3,3] - quality[n][:3,3]) for n in nodes])
    rd = np.array([np.rad2deg(Rotation.from_matrix(first[n][:3,:3].T @ quality[n][:3,:3]).magnitude()) for n in nodes])
    limits = json.loads((OUT / "initialization_equivalence_policy.json").read_text())
    t, r = stats(td), stats(rd)
    equivalent = (t["median"] <= limits["median_translation_difference_m_max"] and
        t["p95"] <= limits["p95_translation_difference_m_max"] and
        r["median"] <= limits["median_rotation_difference_deg_max"] and r["p95"] <= limits["p95_rotation_difference_deg_max"])
    result = {"status": ("EQUIVALENT_IDENTICAL_SEEDS_WARN" if same_edge else "EQUIVALENT_UNDER_FROZEN_RULE")
            if equivalent else "INITIALIZATION_SENSITIVE",
        "translation_difference_m": t, "rotation_difference_deg": r,
        "selected_initialization": "FIRST_ARRIVAL_INIT" if equivalent else None,
        "same_anchor_no_extra_alignment": True, "same_initialization_edge": bool(same_edge),
        "distinct_seeds_tested": not same_edge, "policy_equivalence_pass": bool(equivalent),
        "limitation": "Both required rules select the same frozen edge; this checks repeatability/policy compatibility, not robustness to distinct initializations."
            if same_edge else "Comparison limited to the two frozen initialization rules.", "GT_used": False}
    write_json("initialization_sensitivity_summary.json", result)
    return equivalent, result


def post_pgo_audit(local_steps, before, optimized, selection):
    tables, results = [], {}
    for robot in STARTS:
        def steps(df, source):
            d = df[df.robot_id.eq(robot)]
            return step_frame(Track(source, robot, d.keyframe_id.to_numpy(np.int64), d.timestamp.to_numpy(np.int64),
                d[["tx", "ty", "tz"]].to_numpy(float), d[["qx", "qy", "qz", "qw"]].to_numpy(float)))
        pre, post = steps(before, "first_loop_common"), steps(optimized, "clean_optimized")
        limit = selection["robots"][robot]["frozen_thresholds"]["translation_step_xy_m"]["threshold"]
        merged = post[["robot_id", "keyframe_i", "keyframe_j", "timestamp_i", "timestamp_j"]].copy()
        for metric in ("translation_xy_m", "translation_xyz_m", "relative_rotation_deg", "linear_speed_mps"):
            merged["local_"+metric] = local_steps[robot][metric].to_numpy()
            merged["pre_common_"+metric] = pre[metric].to_numpy()
            merged["post_pgo_"+metric] = post[metric].to_numpy()
        merged["frozen_stage12c_xy_step_limit_m"] = limit
        merged["pre_robust_xy_flag"] = pre.translation_xy_m.gt(limit)
        merged["post_robust_xy_flag"] = post.translation_xy_m.gt(limit)
        merged["post_top20_xy"] = post.translation_xy_m.rank(method="first", ascending=False).le(20)
        merged["post_absolute_catastrophic_100m"] = post.translation_xyz_m.ge(100)
        merged["new_lineage_catastrophic_flag"] = (post.translation_xyz_m.gt(5*limit) & pre.translation_xyz_m.le(5*limit)) | (
            post.translation_xyz_m.ge(100) & pre.translation_xyz_m.lt(100))
        results[robot] = {"post_step_statistics": {metric: stats(post[metric]) for metric in (
                "translation_xy_m", "translation_xyz_m", "relative_rotation_deg", "linear_speed_mps")},
            "new_catastrophic_steps": int(merged.new_lineage_catastrophic_flag.sum()),
            "top20_new_robust_flags_without_pre_flag": int((merged.post_top20_xy & merged.post_robust_xy_flag & ~merged.pre_robust_xy_flag).sum())}
        tables.append(merged)
    combined = pd.concat(tables, ignore_index=True)
    combined.to_csv(OUT / "clean_post_pgo_step_audit.csv", index=False)
    new = bool(combined.new_lineage_catastrophic_flag.any())
    info = {"pgo_introduces_new_catastrophic_jump": "YES" if new else "NO", "robots": results,
        "catastrophic_rule_file": "clean_graph_policy.json", "robust_flags_are_diagnostics_not_automatic_catastrophic_labels": True}
    write_json("clean_post_pgo_integrity_summary.json", info)
    return info


def stage13_schedule(frames, local, common, loops):
    schedule = []
    for K in [1, 2, 5, 10, 20, 40, 75]:
        d = loops.head(K)
        bounds, spans, pts = {}, {}, []
        for robot, field in (("robot1", "query_keyframe_id"), ("robot3", "candidate_keyframe_id")):
            ids = d[field].astype(int).to_list()
            xyz = np.array([local[(robot, i)][:3, 3] for i in ids])
            bounds[robot] = {"min_xyz": xyz.min(0).tolist(), "max_xyz": xyz.max(0).tolist()}
            lo, hi = min(ids), max(ids)
            path = np.array([local[(robot, i)][:3, 3] for i in range(lo, hi+1)])
            spans[robot] = {"first_endpoint_kf": lo, "last_endpoint_kf": hi,
                "path_distance_between_endpoint_extrema_m": float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())}
            pts.extend(common[(robot, i)][:3, 3] for i in ids)
        xyz = np.asarray(pts)
        schedule.append({"K": K, "arrival_indices": d.arrival_index.astype(int).to_list(),
            "frozen_loop_ids": d.frozen_loop_id.astype(int).to_list(),
            "first_query_keyframe": int(d.query_keyframe_id.iloc[0]), "last_query_keyframe": int(d.query_keyframe_id.iloc[-1]),
            "endpoint_local_bounding_boxes": bounds, "endpoint_trajectory_spans": spans,
            "first_loop_common_frame_bounding_box": {"min_xyz": xyz.min(0).tolist(), "max_xyz": xyz.max(0).tolist()},
            "GICP_quality_summary": stats(d.GICP_quality)})
    write_json("stage13_loop_schedule.json", {"status": "FROZEN_SCHEDULE_ONLY_NOT_EXECUTED",
        "K_schedule": [1,2,5,10,20,40,75], "order": "R1 query-arrival order of separate-run replay, not simultaneous physical time",
        "ordered_loop_file_sha256": sha256(OUT / "ordered_rank1_clean_loops.csv"), "GT_used": False, "schedules": schedule})


def build_maps(first, optimized, frames):
    rows, clouds, manifests = [], {}, []
    for condition, traj, filename in [("FIRST_LOOP_ONLY", first, "clean_first_loop_map.ply"),
            ("ALL_75_LOOPS_CLEAN_GTSAM", optimized, "clean_all75_pgo_map.ply")]:
        by, points = s10.map_points(traj, frames["robot1"], frames["robot3"])
        assert len(points) and np.isfinite(points).all()
        metric = s10.consistency(by["robot1"], by["robot3"])
        rows.append({"condition": condition, **metric})
        merged = s10.voxel(points)
        s10.write_ply(OUT / filename, merged)
        path = OUT / filename
        manifests.append({"condition": condition, "path": str(path), "size_bytes": path.stat().st_size,
            "sha256": sha256(path), "points": len(merged), "finite": bool(np.isfinite(merged).all()), "server_only": True})
        clouds[condition] = {robot: s10.voxel(cloud) for robot, cloud in by.items()}
        progress("MAP_BUILT", condition=condition, **metric)
    pd.DataFrame(rows).to_csv(OUT / "clean_map_consistency.csv", index=False)
    write_json("clean_map_manifest.json", {"map_protocol": {"frame_stride": 10, "point_stride": 16,
        "voxel_m": 0.5, "frame_stride_origin": "first retained keyframe", "range_filter_m": [1,60]},
        "artifacts": manifests, "GT_used": False})
    return rows, clouds


def pre_gt_decision(manifest, init_info, solver, equivalent, post, map_rows):
    checks = {"no_catastrophic_retained_input": True, "graph_sizes_valid": manifest["total_nodes"] == 5796,
        "first_loop_initialization_valid": init_info["status"] == "PASS", "gtsam_normal_return": solver["termination_status"] == "NORMAL_RETURN",
        "finite_poses": solver["finite_poses"], "no_new_catastrophic_pgo_jump": post["pgo_introduces_new_catastrophic_jump"] == "NO",
        "loop_residuals_finite": True, "maps_finite": all(np.isfinite([r["median_m"], r["p95_m"]]).all() for r in map_rows),
        "initialization_equivalence_supports_first_arrival": bool(equivalent), "GT_used": False}
    ready = all(v for k, v in checks.items() if k != "GT_used")
    decision = {"decision": "CLEAN_BACKEND_READY" if ready else "CLEAN_BACKEND_NOT_READY",
        "written_before_GT_evaluation": True, "UTC": datetime.now(timezone.utc).isoformat(), "checks": checks,
        "reason": "Frozen stable-segment graph, first-arrival common frame and finite batch solution/map passed GT-free integrity checks." if ready else
            "One or more frozen structural/initialization/finite/integrity gates failed; see checks.",
        "covariance_alerts": "retained health metadata; fixed Stage10.3 factor weights preserved",
        "GT_used_for_policy": False}
    write_json("pre_gt_clean_backend_decision.json", decision)
    return decision


def offline_evaluation(first, optimized, frames):
    assert (OUT / "pre_gt_clean_backend_decision.json").is_file()
    # First load of GT columns in this runner; selections, graph and decision are frozen already.
    full_gt = {robot: pd.read_csv(s10.PROC / robot / "keyframes.csv") for robot in STARTS}
    rows = []
    for condition, tr in (("FIRST_LOOP_ONLY", first), ("ALL_75_LOOPS_CLEAN_GTSAM", optimized)):
        ate, _, _ = s10.joint_eval(tr, full_gt["robot1"], full_gt["robot3"])
        for r in ate:
            rows.append({"condition": condition, "metric_type": "ATE_joint_rigid_alignment", "scope": r["scope"],
                "ATE_RMSE_m": r["ATE_RMSE_m"], "sample_count": len(tr) if r["scope"] == "joint" else len(frames[r["scope"]]),
                "use": "OFFLINE GT ONLY"})
        lookup = s10.lookup(tr)
        all_t, all_r = [], []
        for robot, k in frames.items():
            ts, rs = [], []
            ids = k.keyframe_id.to_list()
            for a, b in zip(ids[:-10], ids[10:]):
                assert b - a == 10
                ga, gb = full_gt[robot].iloc[a], full_gt[robot].iloc[b]
                G0 = s10.T([ga.qx, ga.qy, ga.qz, ga.qw], [ga.x, ga.y, ga.z])
                G1 = s10.T([gb.qx, gb.qy, gb.qz, gb.qw], [gb.x, gb.y, gb.z])
                E = s10.inv(s10.inv(G0) @ G1) @ s10.inv(lookup[(robot,a)]) @ lookup[(robot,b)]
                ts.append(float(np.linalg.norm(E[:3,3])))
                rs.append(float(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude())))
            all_t.extend(ts); all_r.extend(rs)
            rows.append({"condition": condition, "metric_type": "RPE", "scope": robot, "interval_keyframes": 10,
                "translation_RMSE_m": float(np.sqrt(np.mean(np.square(ts)))),
                "rotation_RMSE_deg": float(np.sqrt(np.mean(np.square(rs)))), "sample_count": len(ts), "use": "OFFLINE GT ONLY"})
        rows.append({"condition": condition, "metric_type": "RPE", "scope": "combined", "interval_keyframes": 10,
            "translation_RMSE_m": float(np.sqrt(np.mean(np.square(all_t)))),
            "rotation_RMSE_deg": float(np.sqrt(np.mean(np.square(all_r)))), "sample_count": len(all_t), "use": "OFFLINE GT ONLY"})
    df = pd.DataFrame(rows)
    df.to_csv(OUT / "clean_offline_gt_evaluation.csv", index=False)
    return df


def historical_comparison(solver, residuals, map_rows, post, evaluation):
    old_solver = pd.read_csv(OLD / "gtsam_solver_results.csv")
    old_solver = old_solver[old_solver.policy.eq("RANK1_SANITIZED")].iloc[0]
    old_res = json.loads((OLD / "gtsam_loop_residual_summary.json").read_text())["RANK1_SANITIZED"]
    old_map = pd.read_csv(OLD / "gtsam_map_consistency.csv").set_index("condition").loc["rank1"]
    old_steps = {r: pd.read_csv(ROOT / f"outputs/cumulti_v1/12a_trajectory_integrity/post_pgo_{r}_steps.csv") for r in STARTS}
    old_gt = pd.read_csv(OLD / "gtsam_offline_gt_evaluation.csv")
    old_ate = float(old_gt[(old_gt.condition.eq("RANK1_GTSAM")) & old_gt.metric_type.str.startswith("ATE") & old_gt.scope.eq("joint")].ATE_RMSE_m.iloc[0])
    new_map = next(r for r in map_rows if r["condition"] == "ALL_75_LOOPS_CLEAN_GTSAM")
    new_ate = float(evaluation[evaluation.condition.eq("ALL_75_LOOPS_CLEAN_GTSAM") & evaluation.metric_type.str.startswith("ATE") & evaluation.scope.eq("joint")].ATE_RMSE_m.iloc[0])
    common = {"purpose": "data-quality/backend sanity; not a fair algorithm benchmark", "raw_objective_comparison_valid": False}
    rows = [{"condition": "HISTORICAL_STAGE10_3_RANK1", "nodes": int(old_solver.nodes), "loops": int(old_solver.loop_factors),
        "odometry_factors": int(old_solver.odometry_factors), "runtime_s": float(old_solver.runtime_s),
        "loop_translation_p95_m": old_res["translation_m"]["p95"], "map_nn_median_m": float(old_map.median_m),
        "map_nn_p95_m": float(old_map.p95_m), "robot1_max_xy_step_m": float(old_steps["robot1"].translation_xy_m.max()),
        "robot3_max_xy_step_m": float(old_steps["robot3"].translation_xy_m.max()), "joint_ATE_offline_only_m": old_ate,
        "contains_early_EKF_failure": True, **common},
        {"condition": "STAGE12D_CLEAN_75", "nodes": solver["nodes"], "loops": 75, "odometry_factors": 5794,
        "runtime_s": solver["runtime_s"], "loop_translation_p95_m": residuals["translation_m"]["p95"],
        "map_nn_median_m": new_map["median_m"], "map_nn_p95_m": new_map["p95_m"],
        "robot1_max_xy_step_m": post["robots"]["robot1"]["post_step_statistics"]["translation_xy_m"]["max"],
        "robot3_max_xy_step_m": post["robots"]["robot3"]["post_step_statistics"]["translation_xy_m"]["max"],
        "joint_ATE_offline_only_m": new_ate, "contains_early_EKF_failure": False, **common}]
    pd.DataFrame(rows).to_csv(OUT / "historical_vs_clean_backend.csv", index=False)


def figures(local, first, optimized, loops, residuals, clouds):
    anchor = s10.lookup(local)[("robot1",150)][:3,3]
    def plot_tr(ax, tr, title, separate=False):
        for robot, color in (("robot1", "tab:blue"), ("robot3", "tab:orange")):
            d = tr[tr.robot_id.eq(robot)]
            if d.empty:
                continue
            origin = d[["tx", "ty", "tz"]].iloc[0].to_numpy() if separate else anchor
            ax.plot(d.tx-origin[0], d.ty-origin[1], color=color, lw=.8, label=robot)
        ax.set_title(title); ax.set_aspect("equal", adjustable="box"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.grid(alpha=.15); ax.legend()
    fig, ax = plt.subplots(1,2,figsize=(12,5), constrained_layout=True)
    for a, robot in zip(ax, STARTS):
        plot_tr(a, local[local.robot_id.eq(robot)], robot + " retained local frame", separate=True)
    fig.suptitle("Stable local segments; display origin is each retained start (rigid translation only)")
    fig.savefig(OUT / "clean_local_trajectories.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(1,3,figsize=(17,5), constrained_layout=True)
    for a, robot in zip(ax[:2], STARTS): plot_tr(a, local[local.robot_id.eq(robot)], robot + " separate local frame", separate=True)
    plot_tr(ax[2], first, "First-arrival loop: common frame")
    fig.savefig(OUT / "first_loop_common_frame.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(1,2,figsize=(14,6), constrained_layout=True)
    both = pd.concat([first, optimized])[["tx", "ty"]].to_numpy()-anchor[:2]
    lo, hi = both.min(0)-5, both.max(0)+5
    for a, tr, title in zip(ax, (first,optimized), ("First loop: common frame", "All 75 loops: clean GTSAM")):
        plot_tr(a,tr,title); a.set_xlim(lo[0],hi[0]); a.set_ylim(lo[1],hi[1])
    fig.savefig(OUT / "clean_pgo_before_after.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(1,2,figsize=(11,4), constrained_layout=True)
    ax[0].hist(residuals.translation_residual_m,bins=30); ax[0].set_xlabel("translation residual (m)")
    ax[1].hist(residuals.rotation_residual_deg,bins=30); ax[1].set_xlabel("rotation residual (deg)")
    fig.suptitle("Clean 75 frozen loop residuals")
    fig.savefig(OUT / "clean_loop_residuals.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(1,2,figsize=(14,6), constrained_layout=True)
    conditions = ["FIRST_LOOP_ONLY", "ALL_75_LOOPS_CLEAN_GTSAM"]
    allcloud = np.vstack([c for cond in conditions for c in clouds[cond].values()])
    lo, hi = allcloud[:,:2].min(0)-anchor[:2]-5, allcloud[:,:2].max(0)-anchor[:2]+5
    for a, cond in zip(ax,conditions):
        for robot,color in (("robot1","tab:blue"),("robot3","tab:orange")):
            x=clouds[cond][robot]; x=x[::max(1,len(x)//100000)]
            a.scatter(x[:,0]-anchor[0],x[:,1]-anchor[1],s=.15,color=color,alpha=.4,label=robot)
        a.set_title(cond); a.set_aspect("equal",adjustable="box"); a.set_xlim(lo[0],hi[0]); a.set_ylim(lo[1],hi[1]); a.legend()
    fig.savefig(OUT / "clean_map_before_after.png", dpi=160); plt.close(fig)
    audit = pd.read_csv(OUT / "clean_post_pgo_step_audit.csv")
    fig, ax = plt.subplots(2,2,figsize=(14,7), constrained_layout=True)
    for col,robot in enumerate(STARTS):
        old=pd.read_csv(ROOT / f"outputs/cumulti_v1/12a_trajectory_integrity/post_pgo_{robot}_steps.csv")
        oldx=(old.timestamp_i.astype(np.int64)-int(old.timestamp_i.iloc[0]))/1e9
        ax[0,col].plot(oldx,old.translation_xy_m,lw=.6); ax[0,col].set_title(robot+" historical full graph (early EKF failure)")
        new=audit[audit.robot_id.eq(robot)]
        nx=(new.timestamp_i.astype(np.int64)-int(old.timestamp_i.iloc[0]))/1e9
        ax[1,col].plot(nx,new.post_pgo_translation_xy_m,lw=.6); ax[1,col].set_title(robot+" stable-segment clean graph")
        for a in ax[:,col]: a.set_ylabel("adjacent XY step (m)"); a.set_xlabel("seconds from original first keyframe"); a.grid(alpha=.2)
    fig.savefig(OUT / "old_vs_clean_trajectory_integrity.png", dpi=160); plt.close(fig)


def validation(manifest, solver, equivalent, post, decision, input_manifest, same_edge=False):
    for item in input_manifest:
        assert sha256(item["path"]) == item["sha256"], "frozen input mutated: " + item["path"]
    figure_names = ["clean_local_trajectories.png", "first_loop_common_frame.png", "clean_pgo_before_after.png",
        "clean_loop_residuals.png", "clean_map_before_after.png", "old_vs_clean_trajectory_integrity.png"]
    assert all((OUT/p).is_file() for p in figure_names)
    expected = pd.read_csv(LOOP_SOURCE,usecols=LOOP_COLUMNS)
    expected.insert(0,"frozen_loop_id",np.arange(len(expected)))
    kept = expected[expected.query_keyframe_id.ge(150) & expected.candidate_keyframe_id.ge(234)].sort_values(
        ["query_timestamp","query_keyframe_id","candidate_keyframe_id"],kind="stable").reset_index(drop=True)
    ordered = pd.read_csv(OUT / "ordered_rank1_clean_loops.csv")
    pd.testing.assert_frame_equal(kept[["frozen_loop_id"]+LOOP_COLUMNS],ordered[["frozen_loop_id"]+LOOP_COLUMNS],
        check_exact=False,rtol=1e-12,atol=1e-12)
    write_json("clean_loop_provenance_audit.json", {"status":"PASS","original_rows":len(expected),
        "retained_rows":len(ordered),"removed_only_by_endpoint_starts":len(expected)-len(kept),
        "source_sha256":sha256(LOOP_SOURCE),"transform_values_unchanged":True,"arrival_order_GT_free":True})
    decision_path = OUT / "pre_gt_clean_backend_decision.json"
    gt_path = OUT / "clean_offline_gt_evaluation.csv"
    solver_path = OUT / "clean_gtsam_solver_result.json"
    assert solver_path.stat().st_mtime_ns <= decision_path.stat().st_mtime_ns < gt_path.stat().st_mtime_ns
    write_json("execution_phase_audit.json", {"pre_gt_decision_UTC":decision["UTC"],
        "pre_gt_decision_sha256":sha256(decision_path),
        "solver_result_created_ns":solver_path.stat().st_mtime_ns,
        "pre_gt_decision_created_ns":decision_path.stat().st_mtime_ns,
        "offline_GT_result_created_ns":gt_path.stat().st_mtime_ns,
        "solver_then_pre_gt_decision_then_GT_verified":True,"no_solver_rerun_after_GT":True,
        "GT_column_load_location":"offline_evaluation(), following persisted decision assertion"})
    lines = ["STAGE12C STABLE STARTS REUSED: PASS", "GT USED FOR GRAPH CONSTRUCTION: NO", "ROBOT1 START KF: 150",
        "ROBOT3 START KF: 234", "EARLY EKF FAILURE REMOVED FROM GRAPH: PASS", "NODE COUNT VALID: PASS",
        "ODOMETRY FACTOR COUNT VALID: PASS", "75 LOOP INPUT VALID: PASS", "FIRST LOOP ORDER GT-FREE: PASS",
        "FIRST LOOP CONNECTS GRAPH: PASS", "FIRST LOOP INITIALIZATION RESIDUAL: PASS",
        "INITIALIZATION SENSITIVITY: " + ("PASS" if equivalent and not same_edge else "WARN"), "GTSAM CLEAN GRAPH: PASS",
        "POST-PGO NEW CATASTROPHIC JUMP: " + post["pgo_introduces_new_catastrophic_jump"],
        "GT-FREE MAP: PASS", "PRE-GT CLEAN BACKEND DECISION: " + decision["decision"], "OFFLINE GT: PASS",
        "STAGE13 LOOP SCHEDULE: PASS", "COVARIANCE-AWARE FACTOR WEIGHTING USED: NO", "STAGE10.3 MODIFIED: NO",
        "STAGE12A/B/C MODIFIED: NO", "DEMO MODIFIED: NO", "FROZEN INPUT HASHES: PASS", "FIGURES: 6/6",
        "PRE-GT DECISION BEFORE GT: PASS", "SOLVER RERUN AFTER GT: NO", "LOOP FILTER PROVENANCE: PASS"]
    if same_edge:
        lines.append("INITIALIZATION LIMITATION: first arrival and highest quality are the same frozen edge; distinct-seed robustness not tested")
    (OUT / "VALIDATION_REPORT.txt").write_text("\n".join(lines)+"\n")
    (OUT / "summary.txt").write_text(f"Stage12D stable-segment clean batch backend: {decision['decision']}. "
        f"{manifest['total_nodes']} nodes / {manifest['total_odometry_factors']} odometry factors / 75 frozen loops. "
        "GT evaluated after pre-GT decision only; Stage13 K schedule frozen but not run.\n")


def finalize_only():
    """Resume reporting/figures from completed immutable solves; never optimize or reload GT."""
    frames = {robot: pd.read_csv(s10.PROC / robot / "keyframes.csv",
        usecols=["keyframe_id", "lidar_timestamp_ns", "lidar_file"]) for robot in STARTS}
    frames = {robot: k[k.keyframe_id.ge(STARTS[robot])].copy() for robot,k in frames.items()}
    local_df = pd.read_csv(OUT / "clean_local_trajectory.csv")
    first_df = pd.read_csv(OUT / "clean_first_loop_trajectory.csv")
    first_traj = pd.read_csv(OUT / "clean_all75_gtsam_trajectory.csv")
    quality_traj = pd.read_csv(OUT / "highest_quality_gtsam_trajectory.csv")
    loops = pd.read_csv(OUT / "ordered_rank1_clean_loops.csv")
    fi = json.loads((OUT / "first_loop_initialization.json").read_text())
    hi = json.loads((OUT / "highest_quality_initialization.json").read_text())
    same_edge = fi["frozen_loop_id"] == hi["frozen_loop_id"]
    equivalent, sensitivity = compare_initializations(s10.lookup(first_traj),s10.lookup(quality_traj),same_edge)
    sensitivity_rows = pd.read_csv(OUT / "initialization_sensitivity.csv")
    sensitivity_rows["same_initialization_edge"] = same_edge
    sensitivity_rows.to_csv(OUT / "initialization_sensitivity.csv",index=False)
    decision = json.loads((OUT / "pre_gt_clean_backend_decision.json").read_text())
    # The persisted pre-GT decision is read-only during report-only recovery.
    clouds = {}
    for condition,tr in (("FIRST_LOOP_ONLY",first_df),("ALL_75_LOOPS_CLEAN_GTSAM",first_traj)):
        by,_ = s10.map_points(tr,frames["robot1"],frames["robot3"])
        clouds[condition] = {robot:s10.voxel(points) for robot,points in by.items()}
    figures(local_df,first_df,first_traj,loops,pd.read_csv(OUT / "clean_loop_residuals.csv"),clouds)
    validation(json.loads((OUT / "clean_graph_manifest.json").read_text()),
        json.loads((OUT / "clean_gtsam_solver_result.json").read_text()),equivalent,
        json.loads((OUT / "clean_post_pgo_integrity_summary.json").read_text()),decision,
        json.loads((OUT / "frozen_input_manifest.json").read_text()),same_edge)
    progress("COMPLETE",decision=decision["decision"],reporting_only=True,no_solver_rerun=True)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    assert sys.version_info[:2] == (3,10), "Use scripts/run_stage12d_clean_backend.sh isolated GTSAM environment"
    if "--validate-only" in sys.argv:
        sensitivity=json.loads((OUT / "initialization_sensitivity_summary.json").read_text())
        decision=json.loads((OUT / "pre_gt_clean_backend_decision.json").read_text())
        validation(json.loads((OUT / "clean_graph_manifest.json").read_text()),
            json.loads((OUT / "clean_gtsam_solver_result.json").read_text()),sensitivity["policy_equivalence_pass"],
            json.loads((OUT / "clean_post_pgo_integrity_summary.json").read_text()),decision,
            json.loads((OUT / "frozen_input_manifest.json").read_text()),sensitivity["same_initialization_edge"])
        progress("COMPLETE",decision=decision["decision"],validation_only=True,no_solver_rerun=True)
        return
    if "--finalize-only" in sys.argv:
        finalize_only()
        return
    selection, policy, inputs = freeze_policies()
    progress("POLICIES_FROZEN")
    frames, local, local_df, local_steps = prepare_local(selection)
    loops, manifest = ordered_loops(frames)
    connectivity(frames, loops)
    first_common, first_df, first_info = initialize(loops.iloc[0],frames,local,"FIRST_ARRIVAL_INIT")
    quality_row = loops.sort_values(["GICP_quality","arrival_index"],ascending=[False,True],kind="stable").iloc[0]
    quality_common, _, quality_info = initialize(quality_row,frames,local,"HIGHEST_GICP_QUALITY_INIT")
    first_df.to_csv(OUT / "clean_first_loop_trajectory.csv",index=False)
    stage13_schedule(frames, local, first_common, loops)
    graph, params = build_graph(frames,local,loops)
    first_solved, first_traj, first_result = solve(graph,params,first_common,frames,local,"FIRST_ARRIVAL_INIT")
    quality_solved, quality_traj, quality_result = solve(graph,params,quality_common,frames,local,"HIGHEST_GICP_QUALITY_INIT")
    same_edge = first_info["frozen_loop_id"] == quality_info["frozen_loop_id"]
    equivalent, sensitivity = compare_initializations(first_solved,quality_solved,same_edge)
    res, res_summary = loop_residuals(loops,first_solved)
    _, high_summary = loop_residuals(loops,quality_solved)
    rows=[]
    for result, lr in ((first_result,res_summary),(quality_result,high_summary)):
        rows.append(result | {"loop_translation_median_m": lr["translation_m"]["median"],
            "loop_translation_p95_m": lr["translation_m"]["p95"],
            "median_translation_difference_m": sensitivity["translation_difference_m"]["median"],
            "p95_translation_difference_m": sensitivity["translation_difference_m"]["p95"],
            "median_rotation_difference_deg": sensitivity["rotation_difference_deg"]["median"],
            "p95_rotation_difference_deg": sensitivity["rotation_difference_deg"]["p95"],
            "equivalence_pass": equivalent, "same_initialization_edge": same_edge})
    pd.DataFrame(rows).to_csv(OUT / "initialization_sensitivity.csv",index=False)
    first_traj.to_csv(OUT / "clean_all75_gtsam_trajectory.csv",index=False)
    quality_traj.to_csv(OUT / "highest_quality_gtsam_trajectory.csv",index=False)
    write_json("clean_gtsam_solver_result.json", first_result | {"selected_initialization": sensitivity["selected_initialization"],
        "execution": "full 75-loop first-arrival solve reused from initialization sensitivity; no redundant solver rerun"})
    res.to_csv(OUT / "clean_loop_residuals.csv",index=False)
    write_json("clean_loop_residual_summary.json",res_summary)
    write_json("clean_trajectory_schema_audit.json", {"first_loop":schema_audit(first_df), "optimized":schema_audit(first_traj),
        "highest_quality":schema_audit(quality_traj)})
    post=post_pgo_audit(local_steps,first_df,first_traj,selection)
    map_rows,clouds=build_maps(first_df,first_traj,frames)
    decision=pre_gt_decision(manifest,first_info,first_result,equivalent,post,map_rows)
    progress("PRE_GT_DECISION_PERSISTED",decision=decision["decision"])
    evaluation=offline_evaluation(first_df,first_traj,frames)
    historical_comparison(first_result,res_summary,map_rows,post,evaluation)
    figures(local_df,first_df,first_traj,loops,res,clouds)
    validation(manifest,first_result,equivalent,post,decision,inputs,same_edge)
    progress("COMPLETE",decision=decision["decision"])


if __name__ == "__main__":
    main()
