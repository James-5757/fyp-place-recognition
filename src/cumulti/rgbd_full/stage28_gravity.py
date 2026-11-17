"""Fit a robust presentation-up direction using independent local ground samples."""
import json
from collections import defaultdict

import numpy as np
from scipy.spatial.transform import Rotation

from common import SITE, dump, sha
from stage26_manifest import write_csv
from stage27_continuous import DTYPE, pack
from stage28_lidar import DEST, REPORT, ROBOTS, lookup, write_ply

VIEW = DEST.parent


def points(path):
    data = np.fromfile(path,dtype=DTYPE)
    return np.column_stack([data[k] for k in ('x','y','z')]), np.column_stack(
        [data[k] for k in ('red','green','blue')])


def samples(root, manifest, trajectories, robot, basis):
    up = np.array([0,0,1])
    poses = np.array([r[:6] for r in trajectories[robot]],float)
    buckets = defaultdict(list)
    for chunk in manifest['chunks']:
        if chunk['robot']!=robot:
            continue
        x,_ = points(root/chunk['file'])
        offset = 0
        for t,end in chunk['steps']:
            frame = x[offset:end]@basis.T
            offset = end
            pose_i = np.searchsorted(poses[:,0],t,side='right')-1
            center = poses[pose_i,3:6]@basis.T
            relative = frame-center
            height = relative@up
            mask = (height<-.2)&(height>-3.5)&(np.linalg.norm(relative[:,:2],axis=1)<16)
            if mask.sum()>=10:
                buckets[(chunk['epoch'],int(t//12))].append(frame[mask])
    return buckets


def plane(data,rng):
    if len(data)<95:
        return None
    data = data[::max(1,len(data)//1200)]
    best_count = 0
    for _ in range(96):
        a,b,c = data[rng.choice(len(data),3,replace=False)]
        normal = np.cross(b-a,c-a)
        norm = np.linalg.norm(normal)
        if norm<1e-7 or abs(normal[2]/norm)<.75:
            continue
        distance = abs((data-a)@(normal/norm))
        inside = distance<.14
        if inside.sum()>best_count:
            best_count = int(inside.sum())
            best = inside
    if best_count<65 or best_count/len(data)<.35:
        return None
    fit = data[best]-data[best].mean(axis=0)
    _,spread,axes = np.linalg.svd(fit,full_matrices=False)
    if spread[1]/max(spread[0],1e-8)<.22 or spread[1]/np.sqrt(len(fit))<.6:
        return None
    normal = axes[-1]*np.sign(axes[-1,-1])
    return dict(normal=normal.tolist(),inliers=best_count,fraction=best_count/len(data),
                residual_m=float(np.sqrt(np.mean((fit@normal)**2))))


def estimate(root,manifest,trajectories,robot,basis):
    candidates = samples(root,manifest,trajectories,robot,basis)
    rng = np.random.default_rng(28)
    valid = []
    for (epoch,window),arrays in candidates.items():
        result = plane(np.concatenate(arrays),rng)
        if result:
            valid.append(dict(epoch=epoch,window=window,**result))
    transform = np.eye(4)
    transform[:3,:3] = basis
    method = 'CONVENTION_UP_ONLY'
    stats = {}
    if valid:
        normals = np.array([r['normal'] for r in valid])
        normal = np.median(normals,axis=0)
        normal /= np.linalg.norm(normal)
        angular = np.degrees(np.arccos(np.clip(normals@normal,-1,1)))
        stats = dict(angular_median_deg=float(np.median(angular)),angular_p90_deg=float(np.quantile(angular,.9)),
                     tilt_deg=float(np.degrees(np.arccos(normal[2]))))
        split = np.array_split(normals,2)
        halves = [np.median(items,axis=0) for items in split if len(items)>=4]
        disagreement = None
        if len(halves)==2:
            a,b = [v/np.linalg.norm(v) for v in halves]
            disagreement = float(np.degrees(np.arccos(np.clip(a@b,-1,1))))
        stats['early_late_window_normal_difference_deg'] = disagreement
        if (disagreement is not None and disagreement<=5 and
                stats['angular_p90_deg']<=9 and stats['tilt_deg']<=35):
            # Stable global roll/pitch tilt correction only; local slopes and all heights remain.
            correction,_ = Rotation.align_vectors([[0,0,1]],[normal])
            transform[:3,:3] = correction.as_matrix()@basis
            method = 'ROBUST_GROUND_NORMAL'
    assert abs(np.linalg.det(transform[:3,:3])-1)<1e-9
    return dict(robot=robot,method=method,candidate_windows=len(candidates),valid_windows=len(valid),
                diagnostics=stats,windows=valid,T_ground_from_display=transform.tolist())


def main():
    VIEW.mkdir(parents=True,exist_ok=True)
    checks={}
    for sensor,root,basis in [('lidar',DEST,np.eye(3)),('rgbd',SITE/'continuous',
                              Rotation.from_euler('x',90,degrees=True).as_matrix())]:
        manifest = lookup(root/'manifest.json')
        trajectories = lookup(root/manifest['trajectory'])
        if sensor=='rgbd':
            for robot in ROBOTS:
                write_csv(REPORT/f'{robot}_rgbd_continuity.csv',
                          [item for item in manifest['boundaries'] if item['robot']==robot])
            dump(REPORT/'rgbd_source_manifest.json',manifest)
        checks[sensor] = {r:estimate(root,manifest,trajectories,r,basis) for r in ROBOTS}
        for r,record in checks[sensor].items():
            dump(REPORT/f'{r}_{sensor}_gravity.json',record)
        # Use one rotation for the combined mode to retain the original cross-robot placement.
        combined = checks[sensor][max(ROBOTS,key=lambda r:(checks[sensor][r]['method']=='ROBUST_GROUND_NORMAL',
                                                          checks[sensor][r]['valid_windows']))]
        checks[sensor]['combined'] = {k:combined[k] for k in ('robot','method','T_ground_from_display')}
        for key in ('robot1','robot3','combined'):
            matrix = np.array(checks[sensor][key]['T_ground_from_display'])
            select = [c for c in manifest['chunks'] if key=='combined' or c['robot']==key]
            def rotated():
                for c in select:
                    x,color = points(root/c['file'])
                    yield pack(x@matrix[:3,:3].T,color)
            path = REPORT/f'{key}_{sensor}_ground.ply'
            write_ply(path,rotated(),sum(c['points'] for c in select))
            checks[sensor][key]['artifact'] = dict(path=path.name,sha256=sha(path),bytes=path.stat().st_size)
        print(sensor,{key:result['method'] for key,result in checks[sensor].items()},flush=True)
    dump(REPORT/'gravity_alignment.json',checks)
    dump(VIEW/'gravity.json',checks)


if __name__=='__main__':
    main()
