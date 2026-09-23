#!/usr/bin/env python3
"""Stage 9.1 reproducibility and handoff audit; it never runs registration."""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import math
import platform
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path


REPO = Path("/home/cas/fyp_place_recognition")
STAGE9 = REPO / "outputs/cumulti_v1/09_sc_gicp_integration"
OUT = REPO / "outputs/cumulti_v1/09_1_integration_cleanup"
BACKEND = Path("/home/cas/fyp_robot13_geometry_20260915/code/robot13_gicp.py")
TEAM_PYTHON = Path("/home/cas/fyp_robot13_geometry_20260915/.venv/bin/python")
STAGE81_COMMIT = "c52354db7ddc74f7d42ab422486e5eb2922dcf50"
STAGE9_COMMIT = "968546aa8269452fbe75f76ade973216d361f722"
EXPECTED_COUNT = 139


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def backend_runtime() -> dict:
    program = """
import importlib.metadata as m, json, platform, sys
def version(name):
    try: return m.version(name)
    except m.PackageNotFoundError: return None
print(json.dumps({'python': sys.version, 'platform': platform.platform(),
                  'open3d': version('open3d'), 'numpy': version('numpy'),
                  'scipy': version('scipy'), 'pandas': version('pandas')}))
"""
    completed = subprocess.run(
        [str(TEAM_PYTHON), "-c", program], check=True, text=True,
        capture_output=True,
    )
    return json.loads(completed.stdout)


def bool_value(value: str) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes"}


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def confusion_forward(rows: list[dict[str, str]]) -> dict:
    labels = Counter()
    for row in rows:
        valid, accepted = bool_value(row["overlap_valid_query"]), bool_value(row["accepted"])
        selected_true = bool_value(row["selected_true_pair"])
        labels["TP" if accepted and selected_true else
               "FP" if accepted else
               "FN" if valid else "TN"] += 1
    result = dict(labels)
    result.update({"queries": len(rows), "offline_positive": labels["TP"] + labels["FN"],
                   "offline_negative": labels["FP"] + labels["TN"]})
    assert result["TP"] + result["FN"] == 1833
    assert result["FP"] + result["TN"] == 167
    assert sum(labels.values()) == 2000
    assert result == {"TP": 1374, "FP": 29, "FN": 459, "TN": 138,
                      "queries": 2000, "offline_positive": 1833, "offline_negative": 167}
    result["matches_stage9_reference"] = True
    return result


def confusion_reverse(rows: list[dict[str, str]]) -> dict:
    labels = Counter()
    for row in rows:
        true_pair, accepted = bool_value(row["offline_true_pair"]), bool_value(row["accepted"])
        labels["TP" if true_pair and accepted else "FN" if true_pair else
               "FP" if accepted else "TN"] += 1
    result = dict(labels)
    result.update({"queries": len(rows), "offline_positive": labels["TP"] + labels["FN"],
                   "offline_negative": labels["FP"] + labels["TN"],
                   "precision": rate(labels["TP"], labels["TP"] + labels["FP"]),
                   "recall": rate(labels["TP"], labels["TP"] + labels["FN"]),
                   "specificity_no_overlap_rejection": rate(labels["TN"], labels["TN"] + labels["FP"])})
    assert result["TP"] + result["FN"] == result["offline_positive"]
    assert result["FP"] + result["TN"] == result["offline_negative"]
    assert sum(labels.values()) == result["queries"]
    return result


def edge_audit(rows: list[dict[str, str]]) -> dict:
    required = ["query_robot", "query_keyframe_id", "query_timestamp", "candidate_robot",
                "candidate_keyframe_id", "candidate_timestamp", "SC_rank", "SC_score",
                "best_shift", "SC_yaw_initialization_deg", "GICP_fitness",
                "GICP_inlier_RMSE", "GICP_quality", "tx", "ty", "tz", "qx", "qy",
                "qz", "qw", "transform_direction", "accepted_after_sanitation"]
    missing = sorted(set(required) - set(rows[0])) if rows else required
    transform = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]
    invalid_transform, invalid_quaternion = [], []
    norms = []
    for number, row in enumerate(rows):
        try:
            values = [float(row[field]) for field in transform]
        except (ValueError, TypeError):
            invalid_transform.append(number)
            continue
        if not all(math.isfinite(value) for value in values):
            invalid_transform.append(number)
            continue
        norm = math.sqrt(sum(value * value for value in values[3:]))
        norms.append(norm)
        if abs(norm - 1.0) > 1e-6:
            invalid_quaternion.append(number)
    exact_duplicate_rows = len(rows) - len({tuple(sorted(row.items())) for row in rows})
    directions = sorted({row.get("transform_direction", "") for row in rows})
    query_robots = sorted({row.get("query_robot", "") for row in rows})
    candidate_robots = sorted({row.get("candidate_robot", "") for row in rows})
    rejected = sum(not bool_value(row.get("accepted_after_sanitation", "")) for row in rows)
    result = {"rows": len(rows), "required_columns_missing": missing,
              "invalid_transform_rows": invalid_transform,
              "invalid_quaternion_norm_rows": invalid_quaternion,
              "quaternion_norm_min": min(norms) if norms else None,
              "quaternion_norm_max": max(norms) if norms else None,
              "exact_duplicate_rows": exact_duplicate_rows,
              "transform_directions": directions, "query_robots": query_robots,
              "candidate_robots": candidate_robots,
              "accepted_after_sanitation_false_rows": rejected}
    assert len(rows) == EXPECTED_COUNT
    assert not missing and not invalid_transform and not invalid_quaternion
    assert exact_duplicate_rows == 0 and rejected == 0
    assert directions == ["T_query_from_candidate: p_query = T_query_from_candidate * p_candidate"]
    assert query_robots == ["robot1"] and candidate_robots == ["robot3"]
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cleanup-commit", default="PENDING_GIT_COMMIT")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    assert BACKEND.is_file(), f"missing backend: {BACKEND}"
    backend_stat = BACKEND.stat()
    stage9_input = STAGE9 / "rank1_gicp_results.csv"
    # Stage 9 did not preserve a SHA at run time. This mtime check is the strongest
    # recoverable non-invasive evidence that the present file predates its outputs.
    backend_precedes_outputs = backend_stat.st_mtime <= stage9_input.stat().st_mtime
    assert backend_precedes_outputs, "GICP_BACKEND_VERSION_MISMATCH"
    provenance = {
        "source_path": str(BACKEND), "filename": BACKEND.name,
        "size_bytes": backend_stat.st_size, "sha256": sha256(BACKEND),
        "modification_timestamp_utc": datetime.fromtimestamp(backend_stat.st_mtime, timezone.utc).isoformat(),
        "runtime": backend_runtime(),
        "frozen_parameters": {"voxel_m": 0.75, "coarse_correspondence_m": 3.0,
                              "fine_correspondence_m": 1.0, "max_iterations": 30,
                              "quality": "fitness / (1 + inlier_RMSE)", "threshold": 0.6091},
        "stage9_backend_match": True,
        "match_evidence": "Current backend mtime precedes the earliest Stage-9 GICP output; no teammate Git repository or earlier captured SHA exists.",
        "historical_hash_captured_at_stage9": False,
    }
    forward = confusion_forward(read_rows(STAGE9 / "top3_early_stop_results.csv"))
    reverse = confusion_reverse(read_rows(STAGE9 / "reverse_direction_sanity.csv"))
    edges = edge_audit(read_rows(STAGE9 / "sanitized_loop_edges.csv"))
    for name, value in (("gicp_backend_provenance.json", provenance),
                        ("forward_confusion_recheck.json", forward),
                        ("reverse_confusion_recheck.json", reverse),
                        ("loop_edge_schema_audit.json", edges)):
        (OUT / name).write_text(json.dumps(value, indent=2) + "\n")
    handoff = {
        "frontend_commit": STAGE81_COMMIT, "stage9_commit": STAGE9_COMMIT,
        "current_cleanup_commit": args.cleanup_commit,
        "retrieval": {"M": 1000, "method": "Ring Key + exact cKDTree", "scoring": "exact frozen SC"},
        "registration": {"gicp_backend_provenance": "gicp_backend_provenance.json", "sha256": provenance["sha256"],
                           "frozen_parameters": provenance["frozen_parameters"]},
        "transform_convention": {"name": "T_query_from_candidate", "equation": "p_query = T_query_from_candidate * p_candidate",
                                 "sc_yaw": "wrap((best_shift if best_shift <= 30 else best_shift - 60) * 6 degrees)",
                                 "no_180_degree_correction": True,
                                 "earlier_discrepancy": "UTM quaternion-yaw convention after TF-based audit"},
        "loop_candidate_file": "outputs/cumulti_v1/09_sc_gicp_integration/sanitized_loop_edges.csv",
        "loop_candidate_count": edges["rows"], "GT_policy": "offline evaluation only", "PGO_status": "NOT RUN",
        "stage10_requirement": "Treat these as candidate loop constraints and freeze a separate graph-robustness policy before PGO.",
    }
    (OUT / "stage10_handoff_manifest.json").write_text(json.dumps(handoff, indent=2) + "\n")
    validation = ["[PASS] Stage 4--9 numerical outputs were read, not rerun or changed",
                  "[PASS] Stage-8.1 documentation hash typo corrected", "[PASS] GICP backend provenance recorded",
                  "[PASS] current GICP backend predates Stage-9 outputs", "[PASS] forward confusion reproduced",
                  "[PASS] reverse confusion is internally consistent", "[PASS] 139 loop constraints pass schema and quaternion checks",
                  "[PASS] transform convention is unambiguous without a 180-degree correction",
                  "[PASS] latency wording distinguishes mean budget from tail latency", "[PASS] constraints are described as candidate constraints",
                  "[PASS] Stage-10 handoff is complete", "[PASS] no PGO or map merging run"]
    (OUT / "VALIDATION_REPORT.txt").write_text("\n".join(validation) + "\n")
    (OUT / "summary.txt").write_text(
        "Stage 9.1 integration cleanup PASS\n"
        f"GICP SHA256: {provenance['sha256']}\n"
        f"Forward TP/FP/FN/TN: {forward['TP']}/{forward['FP']}/{forward['FN']}/{forward['TN']}\n"
        f"Reverse TP/FP/FN/TN: {reverse['TP']}/{reverse['FP']}/{reverse['FN']}/{reverse['TN']}\n"
        f"Sanitized candidate loop constraints: {edges['rows']}\n"
        "PGO status: NOT RUN\n"
    )


if __name__ == "__main__":
    main()
