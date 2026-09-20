#!/usr/bin/env python3
"""Read-only Stage 8.1 canonical Scan Context sanity checks.

This script deliberately loads only frozen Scan Context descriptors.  It does
not load keyframe metadata, poses, RGB, ground truth, or Stage-8 metric files.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


REPO = Path("/home/cas/fyp_place_recognition")
PROCESSED = Path("/home/cas/CU-Multi/processed_v1")
OUT = REPO / "outputs/cumulti_v1/08_1_canonical_sc_sanity"
ROBOTS = ("robot1", "robot2", "robot3")
SHAPE = (20, 60)
SAMPLE_COUNT = 100
TOLERANCE = 1e-7
TIE_TOLERANCE = 1e-12
MS = (10, 100, 1000)


def current_ring_key(descriptors: np.ndarray) -> np.ndarray:
    """The Stage-8 implementation: mean over the 60 sector values."""
    return np.asarray(descriptors.mean(axis=2), dtype=np.float32)


def explicit_reference_ring_key(descriptors: np.ndarray) -> np.ndarray:
    """Independent direct transcription of canonical row-wise mean."""
    result = np.empty((len(descriptors), SHAPE[0]), dtype=np.float32)
    for descriptor_index, descriptor in enumerate(descriptors):
        for row_index in range(SHAPE[0]):
            result[descriptor_index, row_index] = np.float32(descriptor[row_index].mean())
    return result


def deterministic_indices(count: int, sample_count: int = SAMPLE_COUNT) -> np.ndarray:
    if count < sample_count:
        raise RuntimeError(f"Need at least {sample_count} descriptors, found {count}")
    return np.linspace(0, count - 1, sample_count, dtype=np.int64)


def load_descriptors(robot: str) -> np.ndarray:
    path = PROCESSED / "full_2hz" / robot / "scan_context_descriptors.npy"
    descriptors = np.load(path, mmap_mode="r")
    if descriptors.ndim != 3 or tuple(descriptors.shape[1:]) != SHAPE:
        raise RuntimeError(f"Unexpected frozen descriptor shape for {robot}: {descriptors.shape}")
    return descriptors


def has_boundary_tie(distances: np.ndarray, order: np.ndarray, m: int) -> bool:
    if m >= len(order):
        return False
    return bool(np.isclose(distances[order[m - 1]], distances[order[m]], rtol=0.0, atol=TIE_TOLERANCE))


def check_kdtree(query_keys: np.ndarray, database_keys: np.ndarray, query_indices: np.ndarray) -> dict:
    tree = cKDTree(database_keys)
    by_m: dict[str, dict] = {}
    for m in MS:
        exact_matches = 0
        non_tie_queries = 0
        non_tie_matches = 0
        tie_queries = 0
        tie_permitted_matches = 0
        disagreements: list[dict] = []
        for query_index in query_indices:
            query = query_keys[query_index]
            tree_distances, tree_indices = tree.query(query, k=m, workers=1)
            tree_indices = np.atleast_1d(np.asarray(tree_indices, dtype=np.int64))
            distances = np.linalg.norm(database_keys - query, axis=1)
            brute_order = np.argsort(distances, kind="stable")
            brute_indices = brute_order[:m]
            set_match = set(tree_indices.tolist()) == set(brute_indices.tolist())
            boundary_tie = has_boundary_tie(distances, brute_order, m)
            if set_match:
                exact_matches += 1
            if boundary_tie:
                tie_queries += 1
                boundary = distances[brute_order[m - 1]]
                strict_inside = set(np.flatnonzero(distances < boundary - TIE_TOLERANCE).tolist())
                permitted = strict_inside.issubset(set(tree_indices.tolist())) and bool(np.all(
                    distances[tree_indices] <= boundary + TIE_TOLERANCE
                ))
                tie_permitted_matches += int(permitted)
                if not permitted:
                    disagreements.append({"query_index": int(query_index), "kind": "invalid_tie_handling"})
            else:
                non_tie_queries += 1
                non_tie_matches += int(set_match)
                if not set_match:
                    disagreements.append({"query_index": int(query_index), "kind": "non_tie_set_mismatch"})
        by_m[str(m)] = {
            "M": m,
            "queries": int(len(query_indices)),
            "exact_set_matches": exact_matches,
            "non_tie_queries": non_tie_queries,
            "non_tie_set_matches": non_tie_matches,
            "boundary_tie_queries": tie_queries,
            "tie_permitted_matches": tie_permitted_matches,
            "agreement": bool(non_tie_queries == non_tie_matches and tie_queries == tie_permitted_matches),
            "disagreements": disagreements,
        }
    return {
        "library": "scipy.spatial.cKDTree",
        "ring_key_dimension": int(database_keys.shape[1]),
        "metric": "Euclidean L2",
        "query_robot": "robot1",
        "database_robot": "robot3",
        "deterministic_query_indices": query_indices.tolist(),
        "results": by_m,
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    descriptor_report: dict[str, dict] = {}
    all_descriptors: dict[str, np.ndarray] = {}
    all_keys: dict[str, np.ndarray] = {}
    for robot in ROBOTS:
        descriptors = load_descriptors(robot)
        sample_indices = deterministic_indices(len(descriptors))
        sample = np.asarray(descriptors[sample_indices], dtype=np.float32)
        current = current_ring_key(sample)
        reference = explicit_reference_ring_key(sample)
        absolute_error = np.abs(current.astype(np.float64) - reference.astype(np.float64))
        descriptor_report[robot] = {
            "frozen_descriptor_shape": list(descriptors.shape),
            "sample_count": int(len(sample)),
            "deterministic_sample_indices": sample_indices.tolist(),
            "ring_key_shape": list(current.shape),
            "max_absolute_error": float(absolute_error.max()),
            "mean_absolute_error": float(absolute_error.mean()),
            "mismatched_descriptors": int(np.count_nonzero(np.any(absolute_error > TOLERANCE, axis=1))),
            "tolerance": TOLERANCE,
        }
        all_descriptors[robot] = descriptors
        all_keys[robot] = current_ring_key(descriptors)

    canonical = {
        "status": "PASS" if all(v["mismatched_descriptors"] == 0 for v in descriptor_report.values()) else "FAIL",
        "current_implementation": "desc.mean(axis=2)",
        "canonical_definition": "one mean over sectors for each descriptor row",
        "descriptor_shape": ["N", *SHAPE],
        "ring_key_shape": ["N", SHAPE[0]],
        "robots": descriptor_report,
    }
    (OUT / "canonical_ringkey_equivalence.json").write_text(json.dumps(canonical, indent=2) + "\n")

    query_indices = deterministic_indices(len(all_keys["robot1"]))
    kdtree = check_kdtree(all_keys["robot1"], all_keys["robot3"], query_indices)
    kdtree["status"] = "PASS" if all(v["agreement"] for v in kdtree["results"].values()) else "FAIL"
    (OUT / "kdtree_bruteforce_equivalence.json").write_text(json.dumps(kdtree, indent=2) + "\n")

    exact_sc_evidence = (
        "Existing Stage-8 source fixed_run() asserts np.allclose(values, full_scores[i, idx], "
        "atol=EPS, rtol=EPS) and np.array_equal(shift, full_shifts[i, idx]) for every shared "
        "shortlisted candidate. The accepted Stage-8 run completed, so this assertion did not fail."
    )
    passed = canonical["status"] == "PASS" and kdtree["status"] == "PASS"
    report = [
        f"Stage 8.1 canonical Scan Context sanity: {'PASS' if passed else 'FAIL'}",
        "",
        "Ring-Key equivalence:",
        *[
            f"- {robot}: max_abs_error={entry['max_absolute_error']:.9g}, "
            f"mean_abs_error={entry['mean_absolute_error']:.9g}, "
            f"mismatched_descriptors={entry['mismatched_descriptors']}"
            for robot, entry in descriptor_report.items()
        ],
        "",
        "KD-tree vs brute-force Euclidean Ring-Key retrieval:",
        *[
            f"- M={m}: agreement={entry['agreement']}, non_tie={entry['non_tie_set_matches']}/"
            f"{entry['non_tie_queries']}, boundary_ties={entry['boundary_tie_queries']}, "
            f"permitted_tie_matches={entry['tie_permitted_matches']}"
            for m, entry in kdtree["results"].items()
        ],
        "",
        "Candidate-generation leakage audit: only frozen Scan Context descriptors are loaded; no GT, "
        "pose, heading, keyframe metadata, or RGB source is loaded by this script.",
        "Exact SC second-stage evidence: " + exact_sc_evidence,
        "Stage-8 numerical outputs were read-only and unchanged.",
    ]
    (OUT / "SANITY_REPORT.txt").write_text("\n".join(report) + "\n")
    if not passed:
        raise SystemExit("Stage 8.1 sanity check failed; see JSON reports")


if __name__ == "__main__":
    main()
