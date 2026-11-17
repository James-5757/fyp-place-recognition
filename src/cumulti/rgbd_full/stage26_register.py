"""Bounded Stage26 comparison against frozen Stage24E boundary splits."""
import json
import time
from functools import lru_cache

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from common import OUT, dump
from fusion import pose, rigid
from stage26_manifest import read_csv, write_csv
from stage26_recover import DEST, gate

PARAMETERS = dict(seed=26, voxel_m=.08, normal_radius_m=.20, feature_radius_m=.40,
                  ransac_distance_m=.12, ransac_iterations=10000, ransac_confidence=.999,
                  icp_radius_m=.30, icp_iterations=20,
                  validation_distance_m=.12, validation_min_inliers=300, validation_min_overlap=.35,
                  intra_agreement_m=.2, intra_agreement_deg=3, min_distinct_frames=3,
                  min_span_ns=800_000_000, min_spread_ratio=.05)


def moved(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def fit_icp(target, source, seed):
    tree = cKDTree(target)
    transform = seed.copy()
    for _ in range(PARAMETERS['icp_iterations']):
        transformed = moved(source, transform)
        distance, index = tree.query(transformed)
        keep = distance <= .30
        if keep.sum() < 300:
            return None
        rotation, translation = rigid(transformed[keep], target[index[keep]])
        step = np.eye(4)
        step[:3, :3], step[:3, 3] = rotation, translation
        transform = step @ transform
        if np.linalg.norm(translation) < .0005 and np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1)) < np.deg2rad(.02):
            break
    return transform


def global_seed(target, source):
    reg = o3d.pipelines.registration
    clouds, features = [], []
    for points in (target, source):
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).voxel_down_sample(.08)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.20, max_nn=30))
        cloud.orient_normals_towards_camera_location(np.zeros(3))
        features.append(reg.compute_fpfh_feature(cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=.40, max_nn=100)))
        clouds.append(cloud)
    left, right = (np.asarray(f.data).T for f in features)
    forward = cKDTree(left).query(right)[1]
    backward = cKDTree(right).query(left)[1]
    indices = np.flatnonzero(backward[forward] == np.arange(len(right)))
    if len(indices) < 3:
        return None
    mutual = o3d.utility.Vector2iVector(np.column_stack([indices, forward[indices]]).astype(np.int32))
    o3d.utility.random.seed(PARAMETERS['seed'])
    fit = reg.registration_ransac_based_on_correspondence(
        clouds[1], clouds[0], mutual, .12, reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(.9), reg.CorrespondenceCheckerBasedOnDistance(.12)],
        reg.RANSACConvergenceCriteria(10000, .999))
    return fit.transformation if len(fit.correspondence_set) >= 3 else None


def score(target, source, transform):
    placed = moved(source, transform)
    distances = (cKDTree(target).query(placed)[0], cKDTree(placed).query(target)[0])
    keep = [distance <= .12 for distance in distances]
    spread = []
    for points, mask in zip((placed,target),keep):
        selected = points[mask]
        singular = np.linalg.svd(selected-selected.mean(axis=0),compute_uv=False) if len(selected)>=3 else [0,0]
        spread.append(float(singular[1]/singular[0]) if singular[0]>0 else 0.)
    return dict(inliers=[int(mask.sum()) for mask in keep],
                overlap=[float(mask.mean()) for mask in keep],
                median_m=[float(np.median(d[m])) if m.any() else None for d,m in zip(distances,keep)],
                pass_depth=all(m.sum() >= 300 and m.mean() >= .35 for m in keep), spread=spread)


def run():
    gate()
    cv2.setNumThreads(1)
    dump(DEST/'registration_policy.json',PARAMETERS)
    calibration = json.loads((OUT/'dataset_manifest.json').read_text())['robots']
    pose_rows = {robot: {int(row['index']): row for row in read_csv(OUT/robot/'camera_poses.csv')
                        if row['valid_pose']=='1'} for robot in ('robot1','robot3')}
    poses = {robot: {index: pose(row) for index,row in records.items()} for robot,records in pose_rows.items()}
    evidence = OUT.parent/'24e_rgbd_feasibility'
    cases = json.loads((evidence/'dense_submap_diagnostics.json').read_text())
    diagnostics = json.loads((evidence/'verification_diagnostics.json').read_text())
    boundaries = {row['boundary']:row for row in read_csv(evidence/'epoch_boundary_audit.csv')}

    @lru_cache(maxsize=128)
    def cloud(robot,index,offset):
        frame = DEST/'frames'/robot
        depth = cv2.imread(str(frame/'depth'/f'{index:06d}.png'),-1)
        y,x = np.mgrid[offset:400:8,offset:640:8]
        z = depth[y,x]/1000.
        keep = (z>=.3)&(z<=10)
        rgb = cv2.imread(str(frame/'rgb'/f'{index:06d}.jpg'))
        assert rgb is not None
        color = rgb[y[keep],x[keep],::-1]
        calibration_row = calibration[robot]['RGB']['calibration']
        matrix = np.array(calibration_row['K']).reshape(3,3)
        matrix[:2] *= .5
        uv = np.stack([x[keep],y[keep]],axis=-1).astype(np.float32)
        if not len(uv):
            return np.empty((0,3)), color
        xy = cv2.undistortPoints(uv,matrix,np.array(calibration_row['D'])).reshape(-1,2)
        return np.column_stack([xy,np.ones(len(xy))])*z[keep,None], color

    def submap(robot,frames,anchor,offset):
        origin = np.linalg.inv(poses[robot][anchor])
        parts = [cloud(robot,i,offset) for i in frames]
        xyz = np.concatenate([moved(points,origin@poses[robot][i]) for i,(points,_) in zip(frames,parts)])
        rgb = np.concatenate([colors for _,colors in parts])
        _, indices = np.unique(np.floor(xyz/.05).astype(np.int64),axis=0,return_index=True)
        order = np.sort(indices)
        return xyz[order],rgb[order]

    def repeatability(robot, case, anchors, method, aggregate):
        fit_indices, held_indices = case['fit_frames'], case['holdout_frames']
        if not case['separate_holdout_frames'] or min(map(len,fit_indices+held_indices))<3:
            return [], False
        for side in held_indices:
            times = [int(pose_rows[robot][i]['rgb_bag_ns']) for i in side]
            if max(times)-min(times)<PARAMETERS['min_span_ns']:
                return [], False
        # Each repetition uses disjoint frame IDs on each side. Never initialize
        # from the aggregate fit or use its corresponding validation pair to fit.
        support = []
        for slot, (i,j,h,k) in enumerate(zip(*fit_indices,*held_indices,strict=False)):
            fit_pair = [submap(robot,[frame],anchor,0)[0] for frame,anchor in zip((i,j),anchors)]
            val_pair = [submap(robot,[frame],anchor,4)[0] for frame,anchor in zip((h,k),anchors)]
            row = dict(slot=slot,fit_frames=[i,j],heldout_frames=[h,k],accepted=False)
            if min(map(len,fit_pair+val_pair))<300:
                row['reason']='INSUFFICIENT_POINTS'
                support.append(row)
                continue
            seed = global_seed(*fit_pair) if method=='fpfh_ransac' else np.eye(4)
            estimate = fit_icp(*fit_pair,seed) if seed is not None else None
            if estimate is None:
                row['reason']='NO_INDEPENDENT_FIT'
            else:
                delta = np.linalg.inv(aggregate)@estimate
                distance = float(np.linalg.norm(delta[:3,3]))
                angle = float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()))
                metrics = score(*val_pair,estimate)
                aggregate_metrics = score(*val_pair,aggregate)
                passed = (metrics['pass_depth'] and aggregate_metrics['pass_depth'] and
                          min(metrics['spread']+aggregate_metrics['spread'])>=PARAMETERS['min_spread_ratio'] and
                          distance<=.2 and angle<=3)
                row.update(transform=estimate.tolist(),metrics=metrics,aggregate_metrics=aggregate_metrics,
                           disagreement_m=distance,disagreement_deg=angle,accepted=passed,
                           reason='PASS' if passed else 'VALIDATION_OR_SE3_DISAGREEMENT')
            support.append(row)
        accepted = [row for row in support if row['accepted']]
        span = lambda key,side: np.ptp([int(pose_rows[robot][r[key][side]]['rgb_bag_ns']) for r in accepted])
        enough = len(accepted)>=3 and all(span(key,side)>=PARAMETERS['min_span_ns'] for key in ('fit_frames','heldout_frames') for side in (0,1))
        return support,bool(enough)

    nonadjacent = {}
    for row in read_csv(DEST/'nonadjacent_candidates.csv'):
        nonadjacent.setdefault(row['case'], []).append(row)
    for key, group in nonadjacent.items():
        first = group[0]
        fit = [[int(row[f'index_{side}']) for row in group if row['split']=='fit'] for side in 'ab']
        hold = [[int(row[f'index_{side}']) for row in group if row['split']=='holdout'] for side in 'ab']
        cases.append(dict(boundary=key,fit_frames=fit,holdout_frames=hold,separate_holdout_frames=True,
                          geometry_pass=False,nonadjacent=True))
        boundaries[key] = dict(robot=first['robot'], previous_last_valid=fit[0][0],
                               next_first_valid=fit[1][0],previous_epoch=first['epoch_a'],next_epoch=first['epoch_b'])
    rows, transforms, submaps = [], [], []
    for case in cases:
        key = case['boundary']
        meta = boundaries[key]
        robot = meta['robot']
        a,b = int(meta['previous_last_valid']),int(meta['next_first_valid'])
        fit = [submap(robot, indices, anchor,0) for indices,anchor in zip(case['fit_frames'],(a,b))]
        hold = [submap(robot, indices, anchor,4)[0] for indices,anchor in zip(case['holdout_frames'],(a,b))]
        for side, (xyz,rgb) in enumerate(fit):
            folder = DEST/'submaps'/robot
            folder.mkdir(parents=True,exist_ok=True)
            epochs = (meta['previous_epoch'],meta['next_epoch'])
            points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
            points.colors = o3d.utility.Vector3dVector(rgb.astype(float)/255)
            if len(xyz)>20:
                points,_ = points.remove_statistical_outlier(nb_neighbors=20,std_ratio=2)
            path = folder/f'{epochs[0]}_{epochs[1]}_side{side}.ply'
            if len(points.points):
                assert o3d.io.write_point_cloud(str(path),points)
            fit_indices = case['fit_frames'][side]
            submaps.append(dict(case=key,robot=robot,epoch=epochs[side],anchor=(a,b)[side],
                                frame_ids='|'.join(map(str,fit_indices)),
                                rgb_bag_ns='|'.join(pose_rows[robot][i]['rgb_bag_ns'] for i in fit_indices),
                                coordinate_frame='anchor camera optical',points=len(points.points),
                                asset=path.relative_to(DEST).as_posix() if len(points.points) else ''))
        seeds = [row for row in diagnostics if row['robot']==robot and 'T_target_from_source' in row
                 and row['index_a'] in case['fit_frames'][0] and row['index_b'] in case['fit_frames'][1]]
        baseline_name, baseline = 'identity', np.eye(4)
        if seeds and not case.get('nonadjacent'):
            seed = max(seeds,key=lambda row:(row['accepted'],row['pnp_inliers']))
            i,j = seed['index_a'],seed['index_b']
            measurement = np.array(seed['T_target_from_source'])
            if seed['forward']:
                measurement = np.linalg.inv(measurement)
            baseline = np.linalg.inv(poses[robot][a])@poses[robot][i]@measurement@np.linalg.inv(poses[robot][j])@poses[robot][b]
            baseline_name = f'frozen fit-only PnP {i}/{j}'
        target,source = [points for points,_ in fit]
        for method in ('baseline','fpfh_ransac'):
            start = time.perf_counter()
            seed = baseline if method=='baseline' else (
                global_seed(target,source) if min(len(target),len(source))>=300 else None)
            result = fit_icp(target,source,seed) if seed is not None and min(len(target),len(source))>=300 else None
            elapsed = time.perf_counter()-start
            metrics = score(*hold,result) if result is not None and min(map(len,hold))>0 else dict(
                inliers=[None,None],overlap=[None,None],median_m=[None,None],pass_depth=False)
            independent = case['separate_holdout_frames']
            support, qualified = repeatability(robot,case,(a,b),method,result) if metrics['pass_depth'] and independent else ([],False)
            status = ('NO_GLOBAL_PROPOSAL' if seed is None else 'ICP_NO_FIT' if result is None else
                      'DEPTH_GATE_FAIL' if not metrics['pass_depth'] else
                      'REUSED_VALIDATION_FRAME' if not independent else 'INDEPENDENT_AGGREGATE_PASS')
            eligible_status = ('QUALIFIED' if qualified else status if status!='INDEPENDENT_AGGREGATE_PASS' else
                               'INSUFFICIENT_INDEPENDENT_FRAMES_OR_TIME_SPAN' if not support else
                               'INDEPENDENT_FITS_OR_SE3_INCONSISTENT' if sum(r['accepted'] for r in support)<3 else
                               'INSUFFICIENT_TEMPORAL_SPAN')
            rows.append(dict(boundary=key,cohort='nonadjacent' if case.get('nonadjacent') else 'pass' if case['geometry_pass'] else 'control',
                             method=method,initializer=baseline_name if method=='baseline' else 'mutual FPFH RANSAC',
                             status=status,independent=independent,inliers_a=metrics['inliers'][0],
                             inliers_b=metrics['inliers'][1],overlap_a=metrics['overlap'][0],
                             overlap_b=metrics['overlap'][1],median_a_m=metrics['median_m'][0],
                             median_b_m=metrics['median_m'][1],fit_runtime_s=elapsed,
                             verified_repetitions=sum(r['accepted'] for r in support),
                             bridge_decision=eligible_status,bounded_bridge_qualified=qualified,graph_admitted=False))
            transforms.append(dict(boundary=key,method=method,anchors=[a,b],seed=seed.tolist() if seed is not None else None,
                                   T_previous_boundary_camera_from_next=result.tolist() if result is not None else None,
                                   independent_repeatability=support))
    write_csv(DEST/'submap_manifest.csv',submaps)
    write_csv(DEST/'nonadjacent_registration_results.csv',[row for row in rows if row['cohort']=='nonadjacent'])
    write_csv(DEST/'registration_comparison.csv',rows)
    write_csv(DEST/'registration_failure_cases.csv',[row for row in rows if not row['bounded_bridge_qualified']])
    dump(DEST/'registration_transforms.json',transforms)
    print('Stage26 bounded boundary comparison complete',flush=True)


if __name__ == '__main__':
    run()
