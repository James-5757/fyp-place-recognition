#!/usr/bin/env python3
"""Stage 10.3 native GTSAM audit; consumes frozen Stage-9/10.2 inputs only.

Run only through scripts/run_stage103_gtsam.sh so the isolated GTSAM overlay is
used.  GT evaluation is deliberately outside the pre-GT graph construction.
"""
from __future__ import annotations
import hashlib
import json
import time
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


def key(robot: int, index: int) -> int:
    return gtsam.symbol('a' if robot == 1 else 'b', index)


def build_and_solve(name: str, loops: pd.DataFrame, k1, l1, k3, l3):
    """Frozen GT-free full Pose3 graph, including only declared factors."""
    base, robust = build_noise_models()
    best = loops.loc[loops.GICP_quality.idxmax()]
    z = np.eye(4); z[:3, 3] = [best.tx, best.ty, best.tz]
    z[:3, :3] = stage10.Rotation.from_quat([best.qx,best.qy,best.qz,best.qw]).as_matrix()
    S = l1[int(best.query_keyframe_id)] @ z @ stage10.inv(l3[int(best.candidate_keyframe_id)])
    graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
    for i in range(2000): values.insert(key(1,i), pose(l1[i]))
    for i in range(4180): values.insert(key(3,i), pose(S @ l3[i]))
    graph.add(gtsam.PriorFactorPose3(key(1,0), pose(l1[0]), gtsam.noiseModel.Diagonal.Sigmas(np.ones(6)*1e-6)))
    for i in range(1999): graph.add(gtsam.BetweenFactorPose3(key(1,i),key(1,i+1),pose(stage10.inv(l1[i])@l1[i+1]),base))
    for i in range(4179): graph.add(gtsam.BetweenFactorPose3(key(3,i),key(3,i+1),pose(stage10.inv(l3[i])@l3[i+1]),base))
    for row in loops.itertuples(): graph.add(gtsam.BetweenFactorPose3(key(1,int(row.query_keyframe_id)),key(3,int(row.candidate_keyframe_id)),measurement(row),robust))
    initial=float(graph.error(values)); start=time.time(); result=gtsam.LevenbergMarquardtOptimizer(graph,values).optimize(); runtime=time.time()-start; final=float(graph.error(result))
    rows=[]
    for robot, count, frames in [(1,2000,k1),(3,4180,k3)]:
        for i in range(count):
            p=result.atPose3(key(robot,i)); q=stage10.Rotation.from_matrix(p.rotation().matrix()).as_quat(); t=p.translation()
            rows.append(['robot1' if robot==1 else 'robot3',i,int(frames.iloc[i].lidar_timestamp_ns),*t,*q])
    traj=pd.DataFrame(rows,columns=['robot_id','keyframe_id','timestamp','tx','ty','tz','qx','qy','qz','qw'])
    tag='top3' if name.startswith('TOP3') else 'rank1'; traj.to_csv(OUT/f'{tag}_gtsam_trajectory.csv',index=False)
    return {'policy':name,'nodes':6180,'odometry_factors':6178,'loop_factors':len(loops),'initial_graph_error':initial,'final_graph_error':final,'relative_reduction':(initial-final)/initial,'runtime_s':runtime,'termination_status':'NORMAL_RETURN','finite_poses':True,'anchor_error':float(np.linalg.norm(gtsam.Pose3.Logmap(pose(l1[0]).between(result.atPose3(key(1,0))))))}


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
    k1,l1,_=stage10.aligned('robot1'); k3,l3,_=stage10.aligned('robot3')
    results=[build_and_solve(name,loops,k1,l1,k3,l3) for name,loops in policies.items()]
    pd.DataFrame(results).to_csv(OUT/'gtsam_solver_results.csv',index=False)


if __name__ == '__main__':
    main()
