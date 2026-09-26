#!/usr/bin/env python3
"""Stage 10.3 native GTSAM audit; consumes frozen Stage-9/10.2 inputs only.

Run only through scripts/run_stage103_gtsam.sh so the isolated GTSAM overlay is
used.  GT evaluation is deliberately outside the pre-GT graph construction.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import sys

import gtsam
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_stage10_offline_map_merge as stage10

REPO = Path('/home/cas/fyp_place_recognition')
OUT = REPO / 'outputs/cumulti_v1/10_3_solver_audit'
TOP3 = REPO / 'outputs/cumulti_v1/09_sc_gicp_integration/sanitized_loop_edges.csv'
RANK1 = REPO / 'outputs/cumulti_v1/10_2_pgo_policy_selection/rank1_sanitized_edges.csv'
SIGMAS = np.array([np.deg2rad(5)] * 3 + [1.0] * 3)  # Pose3.Logmap: rot then trans.
HUBER_DELTA = 1.345


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def pose(matrix: np.ndarray) -> gtsam.Pose3:
    return gtsam.Pose3(gtsam.Rot3(matrix[:3, :3]), gtsam.Point3(*matrix[:3, 3]))


def measurement(row: pd.Series) -> gtsam.Pose3:
    matrix = np.eye(4)
    matrix[:3, 3] = [row.tx, row.ty, row.tz]
    matrix[:3, :3] = stage10.Rotation.from_quat([row.qx, row.qy, row.qz, row.qw]).as_matrix()
    return pose(matrix)


def verify_frozen_inputs() -> dict[str, pd.DataFrame]:
    manifest = json.loads((OUT / 'frozen_graph_manifest.json').read_text())
    paths = {'TOP3_SANITIZED': TOP3, 'RANK1_SANITIZED': RANK1}
    result = {}
    for name, path in paths.items():
        record = manifest['policies'][name]
        assert path.is_file() and sha256(path) == record['sha256'], f'frozen input mismatch: {name}'
        result[name] = pd.read_csv(path)
        assert len(result[name]) == record['expected_rows']
    return result


def build_noise_models() -> tuple[object, object]:
    base = gtsam.noiseModel.Diagonal.Sigmas(SIGMAS)
    robust = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(HUBER_DELTA), base)
    return base, robust


def main() -> None:
    """Pre-GT entrypoint; full graph execution is added only after this gate passes."""
    OUT.mkdir(parents=True, exist_ok=True)
    policies = verify_frozen_inputs()
    base, robust = build_noise_models()
    assert base is not None and robust is not None
    (OUT / 'gtsam_runner_preflight.json').write_text(json.dumps({
        'status': 'PASS', 'gtsam_version': '4.2', 'policies': {k: len(v) for k, v in policies.items()},
        'sigmas_rot_then_trans': SIGMAS.tolist(), 'huber_delta': HUBER_DELTA,
        'gt_access': 'forbidden before pre_gt_gtsam_decision.json',
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
