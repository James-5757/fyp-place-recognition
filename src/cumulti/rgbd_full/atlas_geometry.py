"""Camera-only bridge measurements independent of the Atlas pose gauges."""
import argparse
import csv
import json
import sys
from functools import lru_cache

import cv2
import numpy as np
from scipy.spatial import cKDTree
from common import OUT, Path, dump, writecsv

DEST = OUT.parent / '24_rgbd_atlas_v1'


def main(dense=False, local=False, history=False, candidate_file=None, destination=DEST, diagnostics=False):
    cv2.setNumThreads(1)
    cv2.setRNGSeed(24)
    detector = cv2.SIFT_create(nfeatures=1500)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    calibration = {}
    for robot in ('robot1', 'robot3'):
        calib = json.loads((OUT/robot/'RGB_audit.json').read_text())['calibration']
        matrix = np.array(calib['K']).reshape(3, 3)
        matrix[:2] *= .5
        calibration[robot] = matrix, np.array(calib['D'])

    @lru_cache(maxsize=512)
    def features(robot, index):
        rgb = cv2.imread(str(OUT/robot/'rgb'/f'{index:06d}.jpg'), 0)
        depth = cv2.imread(str(OUT/robot/'depth'/f'{index:06d}.png'), -1)
        assert rgb is not None and depth is not None
        keys, descriptors = detector.detectAndCompute(rgb, None)
        pixels = np.array([key.pt for key in keys], np.float32).reshape(-1, 2)
        xy = pixels.astype(int)
        z = depth[xy[:, 1], xy[:, 0]] / 1000.
        rays = cv2.undistortPoints(pixels, *calibration[robot]).reshape(-1, 2) if len(keys) else pixels
        return pixels, descriptors, np.column_stack((rays, np.ones(len(z)))) * z[:, None], (z >= .3) & (z <= 10)

    def ratio(first, second):
        return {a.queryIdx: a.trainIdx for pair in matcher.knnMatch(first, second, k=2)
                if len(pair) == 2 for a, b in [pair] if a.distance < .75*b.distance}

    @lru_cache(maxsize=64)
    def heldout_cloud(robot, index):
        depth = cv2.imread(str(OUT/robot/'depth'/f'{index:06d}.png'), -1)
        y,x = np.mgrid[4:400:8,4:640:8]
        z = depth[y,x].ravel()/1000.
        valid = (z >= .3) & (z <= 10)
        pixels = np.column_stack((x.ravel(),y.ravel()))[valid].astype(np.float32)
        if not len(pixels):
            return np.empty((0,3)), None
        rays = cv2.undistortPoints(pixels,*calibration[robot]).reshape(-1,2)
        points = np.column_stack((rays,np.ones(len(rays))))*z[valid,None]
        return points,cKDTree(points)

    def measure(left, right, source_id, target_id, source_robot, target_robot, detail):
        uv, _, xyz, measured = left
        target, _, other_xyz, other_measured = right
        detail['depth_feature_count'] = int(measured.sum())
        if measured.sum() < 30:
            return None
        matrix, distortion = calibration[target_robot]
        ok, rotation, translation, indices = cv2.solvePnPRansac(
            xyz[measured], target[measured], matrix, distortion,
            iterationsCount=500, reprojectionError=2.5, confidence=.999, flags=cv2.SOLVEPNP_EPNP)
        detail['pnp_inliers'] = len(indices) if ok and indices is not None else 0
        if not ok or len(indices) < 30 or len(indices)/measured.sum() < .5:
            return None
        support = np.flatnonzero(measured)[indices.ravel()]
        if cv2.contourArea(cv2.convexHull(uv[support])) < 4000 or cv2.contourArea(cv2.convexHull(target[support])) < 4000:
            return None
        rotation, translation = cv2.solvePnPRefineLM(xyz[support], target[support], matrix, distortion, rotation, translation)
        rot = cv2.Rodrigues(rotation)[0]
        prediction = xyz @ rot.T + translation.ravel()
        if (prediction[support, 2] <= 0).any():
            return None
        projected = cv2.projectPoints(xyz[support],rotation,translation,matrix,distortion)[0].reshape(-1,2)
        if np.quantile(np.linalg.norm(projected-target[support],axis=1),.9) > 3:
            return None
        transform = np.eye(4)
        transform[:3,:3], transform[:3,3] = rot, translation.ravel()
        detail['T_target_from_source'] = transform.tolist()
        if dense:
            source,source_tree = heldout_cloud(source_robot,source_id)
            destination,target_tree = heldout_cloud(target_robot,target_id)
            detail['dense_samples'] = [len(source),len(destination)]
            if min(len(source),len(destination)) < 300:
                return None
            errors = [target_tree.query(source @ rot.T + translation.ravel())[0],
                      source_tree.query((destination-translation.ravel()) @ rot)[0]]
            inliers = [error <= .12 for error in errors]
            detail['dense_inliers'] = [int(mask.sum()) for mask in inliers]
            detail['dense_overlap'] = [float(mask.mean()) for mask in inliers]
            if any(mask.sum() < 300 or mask.mean() < .35 for mask in inliers):
                return None
            # Report the support count and residual of the more weakly supported side.
            count = min(int(mask.sum()) for mask in inliers)
            residual = max(float(np.median(error[mask])) for error,mask in zip(errors,inliers))
            return transform,len(support),count,residual,min(float(mask.mean()) for mask in inliers)
        both = support[other_measured[support]]
        if len(both) < 6:
            return None
        residual = np.linalg.norm(prediction[both]-other_xyz[both], axis=1)
        if np.median(residual) > .12 or np.mean(residual <= .20) < .8:
            return None
        return transform, len(support), len(both), float(np.median(residual)),None

    candidates = []
    if candidate_file:
        with candidate_file.open() as handle:
            for row in csv.DictReader(handle):
                candidates.append((row["robot_a"],row["robot_b"],int(row["index_a"]),int(row["index_b"]),int(row["arrival_ns"])))
    elif local:
        for robot in ('robot1','robot3'):
            with (OUT/robot/'camera_poses.csv').open() as handle:
                poses = list(csv.DictReader(handle))
            with (OUT/robot/'sync_pairs.csv').open() as handle:
                pairs = list(csv.DictReader(handle))
            epochs = {}
            for row,pair in zip(poses,pairs,strict=True):
                if row['valid_pose']=='1' and pair['paired']=='True':
                    epochs.setdefault(int(row['map_epoch']),[]).append(int(row['index']))
            for left,right in zip(epochs,list(epochs)[1:]):
                left_frames = epochs[left][-60::7]
                right_frames = epochs[right][:60:7]
                for i in left_frames:
                    for j in right_frames:
                        candidates.append((robot,robot,i,j,int(poses[j]['relative_ns'])+25_000_000))
            if history:
                # Shortlist revisits against earlier epochs by normalized RGB appearance.
                def sampled(frames, period):
                    next_stamp = -1
                    for frame in frames:
                        stamp = int(poses[frame]['relative_ns'])
                        if stamp >= next_stamp:
                            yield frame
                            next_stamp = stamp + period

                def descriptor(frame):
                    image = cv2.imread(str(OUT/robot/'rgb'/f'{frame:06d}.jpg'),0)
                    image = cv2.resize(image,(32,20),interpolation=cv2.INTER_AREA).astype(np.float32)
                    image -= image.mean()
                    return image.ravel()/max(1.,np.linalg.norm(image))

                database, descriptors = [], []
                for frames in epochs.values():
                    if database:
                        reference = np.stack(descriptors)
                        first_time = int(poses[frames[0]]['relative_ns'])
                        queries = [f for f in frames if int(poses[f]['relative_ns'])-first_time <= 45_000_000_000]
                        for j in sampled(queries,2_000_000_000):
                            scores = reference @ descriptor(j)
                            order = np.argsort(-scores,kind='stable')[:5]
                            for rank in order:
                                candidates.append((robot,robot,database[rank],j,int(poses[j]['relative_ns'])+25_000_000))
                    for i in sampled(frames,750_000_000):
                        database.append(i)
                        descriptors.append(descriptor(i))
    else:
        with (OUT/'causal_rgbd/rgb_candidates.csv').open() as handle:
            for row in csv.DictReader(handle):
                query,candidate = int(row['query_index']),int(row['candidate_index'])
                i,j = (query,candidate) if row['query_robot']=='robot1' else (candidate,query)
                candidates.append(('robot1','robot3',i,j,int(row['arrival_ns'])))
    edges, records, tested, audit = [], [], set(), []
    for robot_a,robot_b,r1,r3,arrival in candidates:
        key = robot_a,robot_b,r1,r3
        if key in tested:
            continue
        tested.add(key)
        left, right = features(robot_a,r1), features(robot_b,r3)
        if left[1] is None or right[1] is None:
            records.append(dict(robot=robot_a,target_robot=robot_b,robot1_index=r1,robot3_index=r3,mutual=0,accepted=False))
            continue
        forward, reverse = ratio(left[1],right[1]), ratio(right[1],left[1])
        mutual = [(i,j) for i,j in forward.items() if reverse.get(j)==i]
        record = dict(robot=robot_a,target_robot=robot_b,robot1_index=r1,robot3_index=r3,mutual=len(mutual),accepted=False)
        records.append(record)
        if len(mutual) < 30:
            continue
        i,j = np.array(mutual).T
        left = [a[i] for a in left]
        right = [a[j] for a in right]
        for direction in (True,False):
            detail = {}
            fit = measure(left,right,r1,r3,robot_a,robot_b,detail) if direction else measure(right,left,r3,r1,robot_b,robot_a,detail)
            if diagnostics:
                audit.append(dict(robot=robot_a,target_robot=robot_b,index_a=r1,index_b=r3,
                                  forward=direction,accepted=fit is not None,**detail))
            if fit is not None:
                transform, inliers, depth_checks, residual, overlap = fit
                edges.append(dict(robot=robot_a,target_robot=robot_b,robot1_index=r1,robot3_index=r3,
                                  arrival_ns=arrival, inliers=inliers,
                                  independent_depth_checks=depth_checks, depth_median_m=residual, depth_overlap_min=overlap,
                                  method='pnp_with_bidirectional_sampled_cloud_check' if dense else 'pnp_with_matched_target_depth',
                                  robot1_from_robot3=(np.linalg.inv(transform) if direction else transform).tolist()))
                record['accepted'] = True
                break
    writecsv(destination/('local_checks_history.csv' if history else 'local_checks.csv' if local else 'bridge_checks_dense.csv' if dense else 'bridge_checks.csv'), records)
    dump(destination/('local_edges_history.json' if history else 'local_edges.json' if local else 'camera_edges_dense.json' if dense else 'camera_edges.json'), edges)
    if diagnostics:
        dump(destination/'verification_diagnostics.json',audit)
    print(f'Camera edges: {len(edges)}/{len(records)} checked pairs', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dense',action='store_true')
    parser.add_argument('--local',action='store_true')
    parser.add_argument('--history',action='store_true')
    parser.add_argument('--diagnostics',action='store_true')
    parser.add_argument('--candidates',type=Path)
    parser.add_argument('--output',type=Path,default=DEST)
    args = parser.parse_args()
    local = args.local or args.history
    main(args.dense or local or bool(args.candidates),local,args.history,args.candidates,args.output,args.diagnostics)
