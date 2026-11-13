"""Evaluate sequential RGB-D motion and dense submap hypotheses without changing the graph."""
import json
from collections import defaultdict
from functools import lru_cache

import cv2
import numpy as np
from scipy.spatial import cKDTree
from common import OUT, dump, writecsv
from fusion import pose, rigid
from stage24d_graph import residual, verify_links
from stage24d_retrieve import ROBOTS, read
from stage24e_audit import DEST


def evaluate():
    cv2.setNumThreads(1)
    poses = {r:read(OUT/r/'camera_poses.csv') for r in ROBOTS}
    boundaries = read(DEST/'epoch_boundary_audit.csv')
    windows = {w['boundary']:w for w in json.loads((DEST/'boundary_windows.json').read_text())}
    diagnostics = json.loads((DEST/'verification_diagnostics.json').read_text())
    cases = read(DEST/'bridge_checks_dense.csv')
    edges = json.loads((DEST/'camera_edges_dense.json').read_text())
    graph = json.loads((OUT.parent/'24d_rgbd_pose_graph/pose_graph.json').read_text())
    group = defaultdict(dict)
    for edge in edges:
        robot = edge['robot']
        i,j = edge['robot1_index'],edge['robot3_index']
        a,b = poses[robot][i],poses[robot][j]
        nodes = ((robot,int(a['map_epoch'])),(robot,int(b['map_epoch'])))
        matrix = pose(a) @ np.array(edge['robot1_from_robot3']) @ np.linalg.inv(pose(b))
        group[nodes][i,j] = dict(i=i,j=j,T=matrix.tolist(),time_ns=edge['arrival_ns'],
            inliers=edge['inliers'],depth_checks=edge['independent_depth_checks'],
            depth_residual_m=edge['depth_median_m'],overlap_min=edge['depth_overlap_min'])
    verified = {tuple((e['a'],e['b'])):e for e in verify_links(group,poses)}
    calibration = {}
    for robot in ROBOTS:
        camera = json.loads((OUT/robot/'RGB_audit.json').read_text())['calibration']
        k = np.array(camera['K']).reshape(3,3)
        k[:2] *= .5
        calibration[robot] = (k,np.array(camera['D']))

    @lru_cache(maxsize=144)
    def frame_cloud(robot,index,offset):
        depth = cv2.imread(str(OUT/robot/'depth'/f'{index:06d}.png'),-1)
        assert depth is not None
        y,x = np.mgrid[offset:400:8,offset:640:8]
        z = depth[y,x].ravel()/1000.
        mask = (z>=.3)&(z<=10)
        if not mask.any():
            return np.empty((0,3))
        pixels = np.column_stack((x.ravel(),y.ravel()))[mask].astype(np.float32)
        rays = cv2.undistortPoints(pixels,*calibration[robot]).reshape(-1,2)
        return np.column_stack((rays,np.ones(len(rays))))*z[mask,None]

    def submap(robot, indices, anchor, offset):
        inv = np.linalg.inv(pose(poses[robot][anchor]))
        parts=[]
        for index in indices:
            local = inv @ pose(poses[robot][index])
            points = frame_cloud(robot,index,offset)
            parts.append(points @ local[:3,:3].T + local[:3,3])
        if not parts:
            return np.empty((0,3))
        points = np.concatenate(parts)
        _, unique = np.unique(np.floor(points/.05).astype(np.int64),axis=0,return_index=True)
        return points[np.sort(unique)]

    def align(target,source,transform):
        tree = cKDTree(target)
        estimate = transform.copy()
        for _ in range(20):
            moved = source @ estimate[:3,:3].T + estimate[:3,3]
            distances, indices = tree.query(moved)
            mask = distances<=.30   # Fit radius only. Verification remains 12 cm.
            if mask.sum()<300:
                return None
            rot,translation = rigid(moved[mask],target[indices[mask]])
            change = np.eye(4)
            change[:3,:3],change[:3,3] = rot,translation
            estimate = change @ estimate
            dt,dr = residual(np.eye(4),change)
            if dt<.0005 and dr<.02:
                break
        return estimate

    def check(target,source,estimate):
        rot,translation = estimate[:3,:3],estimate[:3,3]
        distances = (cKDTree(target).query(source @ rot.T + translation)[0],
                     cKDTree(source).query((target-translation) @ rot)[0])
        masks = [d<=.12 for d in distances]
        return dict(overlap=[float(m.mean()) for m in masks],
                    inliers=[int(m.sum()) for m in masks],
                    median_m=[float(np.median(d[m])) if m.any() else None for m,d in zip(masks,distances)],
                    geometry_pass=all(m.sum()>=300 and m.mean()>=.35 for m in masks))

    outcomes,submaps = [],[]
    for row in boundaries:
        key,robot = row['boundary'],row['robot']
        a,b = int(row['previous_last_valid']),int(row['next_first_valid'])
        ea,eb = int(row['previous_epoch']),int(row['next_epoch'])
        left,right = (robot,ea),(robot,eb)
        w = windows[key]
        ls,rs = set(w['selected_left']),set(w['selected_right'])
        report = [r for r in cases if r['robot']==robot and int(r['robot1_index']) in ls and int(r['robot3_index']) in rs]
        attempts = [r for r in diagnostics if r['robot']==robot and r['index_a'] in ls and r['index_b'] in rs]
        seeds = [r for r in attempts if 'T_target_from_source' in r]
        fit_lists,hold_lists = [],[]
        for side in ('selected_left','selected_right'):
            indices = w[side]
            fit_lists.append(indices[::2])
            hold_lists.append(indices[1::2] or indices)
        target,source = [submap(robot,indices,anchor,0) for indices,anchor in zip(fit_lists,(a,b))]
        held_target,held_source = [submap(robot,indices,anchor,4) for indices,anchor in zip(hold_lists,(a,b))]
        result=dict(boundary=key,fit_frames=fit_lists,holdout_frames=hold_lists,
                    separate_holdout_frames=all(not (set(x)&set(y)) for x,y in zip(fit_lists,hold_lists)),
                    fit_points=[len(target),len(source)],holdout_points=[len(held_target),len(held_source)],
                    status='NO_VALID_DEPTH_SUPPORT',geometry_pass=False)
        if min(len(target),len(source),len(held_target),len(held_source))>=300:
            guess = np.eye(4)
            result['seed']='identity in boundary camera coordinates (diagnostic initializer)'
            fit_seeds = [s for s in seeds if s['index_a'] in fit_lists[0] and s['index_b'] in fit_lists[1]]
            if fit_seeds:
                seed = max(fit_seeds,key=lambda s:(s['accepted'],s.get('pnp_inliers',0)))
                i,j = seed['index_a'],seed['index_b']
                measured=np.array(seed['T_target_from_source'])
                if seed['forward']:
                    measured=np.linalg.inv(measured)
                guess=(np.linalg.inv(pose(poses[robot][a])) @ pose(poses[robot][i]) @ measured
                       @ np.linalg.inv(pose(poses[robot][j])) @ pose(poses[robot][b]))
                result['seed']=f'PnP frames {i}, {j}'
            estimate=align(target,source,guess)
            result['status']='ICP_NO_FIT'
            if estimate is not None:
                result.update(check(held_target,held_source,estimate))
                result.update(status='HELDOUT_PASS' if result['geometry_pass'] else 'HELDOUT_INCONSISTENT',
                              T_previous_boundary_camera_from_next=estimate.tolist())
        submaps.append(result)
        accepted = sum(r['accepted']=='True' for r in report)
        max_mutual = max((int(r['mutual']) for r in report),default=0)
        if (left,right) in verified:
            reason='VERIFIED_SEQUENTIAL_BRIDGE'
        elif not ls or not rs:
            reason='NO_VALID_DEPTH'
        elif not report:
            reason='CANDIDATE_COVERAGE_FAILURE'
        elif max_mutual<30:
            reason='INSUFFICIENT_VISUAL_OVERLAP_EVIDENCE'
        elif not any(r.get('depth_feature_count',0)>=30 for r in attempts):
            reason='INSUFFICIENT_MEASURED_FEATURE_DEPTH'
        elif not seeds:
            reason='PNP_OR_REPROJECTION_GATE_FAILED'
        elif not accepted:
            reason='GEOMETRIC_INCONSISTENCY'
        else:
            reason='INSUFFICIENT_INDEPENDENT_SUPPORT_OR_TRANSFORM_AGREEMENT'
        outcome=dict(boundary=key,pair_tests=len(report),max_mutual=max_mutual,
                     pnp_hypotheses=len(seeds),pair_geometry_passes=accepted,
                     submap_status=result['status'],submap_disjoint_holdout=result['separate_holdout_frames'],
                     short_epoch=min(int(row['previous_epoch_frames']),int(row['next_epoch_frames']))<3,
                     decision=reason)
        outcomes.append(outcome)
    writecsv(DEST/'sequential_bridge_audit.csv',outcomes)
    dump(DEST/'dense_submap_diagnostics.json',submaps)
    dump(DEST/'sequential_verified_links.json',list(verified.values()))


if __name__=='__main__':
    evaluate()
