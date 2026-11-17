"""Rebuild Stage28 LiDAR playback from cached original scans and frozen poses."""
import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from common import OUT, SITE, dump, sha
from stage26_manifest import read_csv, write_csv
from stage27_continuous import DTYPE, delta, pack, pose7

BASE = OUT.parent
DEST = SITE.parent / 'stage28_multisensor' / 'lidar'
REPORT = BASE / '28_multisensor'
ROBOTS = ('robot1', 'robot3')
COLORS = {'robot1': [66, 191, 255], 'robot3': [255, 160, 70]}


def lookup(path):
    return json.loads(path.read_text(encoding='utf-8'))


def matrices(rows, fields):
    r = Rotation.from_quat([[float(row[k]) for k in fields[3:]] for row in rows])
    out = np.repeat(np.eye(4)[None], len(rows), axis=0)
    out[:, :3, :3] = r.as_matrix()
    out[:, :3, 3] = [[float(row[k]) for k in fields[:3]] for row in rows]
    return out


def write_ply(path, blocks, count):
    with path.open('wb') as stream:
        stream.write((f'ply\nformat binary_little_endian 1.0\nelement vertex {count}\n'
                      'property float x\nproperty float y\nproperty float z\nproperty uchar red\n'
                      'property uchar green\nproperty uchar blue\nend_header\n').encode())
        for block in blocks:
            stream.write(block)


def trajectory(robot, keyframes, stable, odometry):
    audit = read_csv(BASE/'12c_dataset_stability'/f'{robot}_full_ekf_stability.csv')
    assert [(r['keyframe_id'],r['timestamp_ns']) for r in audit]==[
        (r['keyframe_id'],r['lidar_timestamp_ns']) for r in keyframes]
    stamps = np.array([int(r['timestamp_ns']) for r in audit])
    ts = (stamps-stamps[0])/1e9
    fields = ['tx','ty','tz','qx','qy','qz','qw']
    imu_fields = ['ekf_ekf_'+k.removeprefix('t') for k in fields]
    stable = [r for r in stable if r['robot_id']==robot]
    start = int(stable[0]['keyframe_id'])
    assert [int(r['keyframe_id']) for r in stable]==list(range(start,len(ts)))
    assert [int(r['timestamp']) for r in stable]==stamps[start:].tolist()
    prefix = audit[:start+1]
    assert all(r['ekf_match_status']=='MATCHED' and abs(float(r['ekf_match_error_s']))<=.05 for r in prefix)
    ekf = matrices(prefix, imu_fields) @ np.array(odometry[robot]['extrinsic'])
    fixed = matrices(stable, fields)
    rel = np.linalg.inv(ekf[:-1]) @ ekf[1:]
    step_m = np.linalg.norm(rel[:,:3,3],axis=1)
    step_deg = Rotation.from_matrix(rel[:,:3,:3]).magnitude()*180/np.pi
    dt = np.diff(ts[:start+1])
    changes = [False]+[(a['ekf_frame_id'], a['ekf_child_frame_id']) !=
                       (b['ekf_frame_id'], b['ekf_child_frame_id']) for a,b in zip(prefix,prefix[1:])]
    # Presentation thresholds; audit all raw jumps before the verified tail.
    bad = (step_m>np.minimum(2.5,4*dt)) | (step_deg>35) | (dt>1) | changes[1:]
    assert np.isfinite(ekf).all() and (dt>0).all()
    display = np.empty((len(ts),4,4))
    display[start:] = fixed
    epochs = np.zeros(len(ts),dtype=int)
    barriers = sorted(set(np.flatnonzero(bad[:start])+1) | {start})
    for epoch, index in enumerate(barriers,1):
        epochs[index:] = epoch
    for i in range(start-1,-1,-1):
        change = rel[i]
        if bad[i]:
            # Extrapolate from nearby non-spike body-frame increments, without GT.
            window = np.arange(max(0,i-4),min(i+5,len(rel)))
            good = window[~bad[window]]
            change = np.eye(4)
            if len(good):
                change[:3,3] = np.median(rel[good,:3,3]/dt[good,None],axis=0)*min(.5,dt[i])
                angular = Rotation.from_matrix(rel[good,:3,:3]).as_rotvec()/dt[good,None]
                change[:3,:3] = Rotation.from_rotvec(np.median(angular,axis=0)*min(.5,dt[i])).as_matrix()
        display[i] = display[i+1] @ np.linalg.inv(change)
    transforms, events = [], []
    for epoch, index in enumerate([0]+barriers):
        is_stable = index==start
        source = ('STAGE12D_VERIFIED_TAIL' if is_stable else
                  'DISPLAY_ONLY_ESTIMATE' if index else 'EXPLORATORY_EKF_PREFIX')
        certainty = 'FROZEN_LIDAR_PGO' if is_stable else 'DISPLAY_ONLY'
        matrix = None if is_stable else (display[index] @ np.linalg.inv(ekf[index])).tolist()
        transforms.append(dict(robot=robot,epoch=epoch,t=float(ts[index]),source=source,
                               uncertainty=certainty,T_display_from_raw_ekf=matrix,
                               transform_note='Frozen per-pose PGO tail; no single rigid correction' if is_stable else
                               'Rigid transform within the raw EKF segment; propagated backward from the frozen tail'))
        if not index:
            continue
        actual_m,actual_deg = delta(display[index-1],display[index])
        events.append(dict(robot=robot,epoch=epoch,previous_epoch=epoch-1,t=float(ts[index]),last_t=float(ts[index-1]),
                           source=source,uncertainty=certainty,translation_m=actual_m,rotation_deg=actual_deg,
                           raw_translation_m=float(step_m[index-1]),raw_rotation_deg=float(step_deg[index-1]),
                           large_gap=bool(actual_m>2.5 or actual_deg>35),
                           evidence=('Original discontinuous step replaced by a local motion estimate. '
                                     if bad[index-1] else '')+(
                                        'Subsequent poses reproduce the frozen Stage12D tail.' if is_stable else
                                        'The raw local relative motions are preserved within this Epoch.')))
    traj = [[float(t),int(row['keyframe_id']),int(epoch),*pose7(matrix)]
            for t,row,epoch,matrix in zip(ts,keyframes,epochs,display)]
    return traj, epochs, display, transforms, events, stamps


def export(robot, cache, stable, odometry, limit):
    lidar = cache/robot
    keys = read_csv(lidar/'keyframes.csv')
    traj, epochs, matrices_, transforms, events, stamps = trajectory(robot,keys,stable,odometry)
    selected = {}
    for i,(t,_,epoch,*_) in enumerate(traj[:limit]):
        selected.setdefault((epoch,int(t)),i)
    picks = sorted(set(selected.values()) |
                   {i for i in range(1,min(limit,len(traj))) if epochs[i]!=epochs[i-1]} |
                   {min(limit,len(traj))-1})
    buckets = defaultdict(list)
    for i in picks:
        buckets[int(epochs[i]),int(traj[i][0]//25)].append(i)
    chunks, rows, seen = [], [], set()
    for (epoch,bucket), indices in buckets.items():
        data, steps = bytearray(), []
        name = f'{robot}-e{epoch:02d}-{bucket:03d}.bin'
        for i in indices:
            key = keys[i]
            assert key['lidar_file'].endswith(f"_{stamps[i]}.npy")
            scan = np.load(lidar/'lidar'/key['lidar_file'],allow_pickle=False)[::32,:3]
            radius = np.linalg.norm(scan,axis=1)
            scan = scan[np.isfinite(scan).all(axis=1)&(radius>=1)&(radius<=48)]
            x = scan@matrices_[i,:3,:3].T+matrices_[i,:3,3]
            chosen = []
            for n,coord in enumerate(np.floor(x/.34).astype(int)):
                voxel = tuple(coord)
                if voxel not in seen:
                    seen.add(voxel)
                    chosen.append(n)
            data.extend(pack(x[chosen],np.tile(COLORS[robot],(len(chosen),1))))
            steps.append([traj[i][0],len(data)//DTYPE.itemsize])
            rows.append(dict(robot=robot,keyframe_id=i,epoch=epoch,t=traj[i][0],lidar_timestamp_ns=int(stamps[i]),
                             original_message_index=key['original_lidar_message_index'],source_scan=key['lidar_file'],
                             chunk=name,points=len(chosen)))
        if data:
            path = DEST/name
            path.write_bytes(data)
            chunks.append(dict(file=name,robot=robot,epoch=epoch,start=steps[0][0],end=steps[-1][0],
                               points=len(data)//15,steps=steps,bytes=len(data),sha256=sha(path)))
    print(robot,len(picks),'frames',len(chunks),'chunks',len(seen),'points',flush=True)
    return chunks,rows,traj[:limit],transforms,events


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--cache',type=Path,required=True)
    parser.add_argument('--limit',type=int,help='Bounded preflight; writes a separate output directory')
    args=parser.parse_args()
    global DEST,REPORT
    if args.limit:
        DEST=DEST.parent/'lidar_preflight'
        REPORT=REPORT/'preflight'
    DEST.mkdir(parents=True,exist_ok=True)
    REPORT.mkdir(parents=True,exist_ok=True)
    odometry=lookup(BASE/'12d_clean_backend'/'odometry_source_audit.json')
    stable=read_csv(BASE/'12d_clean_backend'/'clean_all75_gtsam_trajectory.csv')
    paths,chunks,frames,transforms,events={},[],[],[],[]
    for robot in ROBOTS:
        c,f,paths[robot],tr,e = export(robot,args.cache,stable,odometry,args.limit or 1000000)
        chunks.extend(c)
        frames.extend(f)
        transforms.extend(tr)
        events.extend(e)
        if not args.limit:
            write_csv(REPORT/f'{robot}_lidar_continuity.csv',e)
            write_ply(REPORT/f'{robot}_lidar.ply',((DEST/x['file']).read_bytes() for x in c),sum(x['points'] for x in c))
    (DEST/'trajectories.json').write_text(json.dumps(paths,separators=(',',':')),encoding='utf-8')
    manifest=dict(robots=list(ROBOTS),compression=10,chunks=chunks,boundaries=events,
                  transforms=transforms,trajectory='trajectories.json',duration=max(v[-1][0] for v in paths.values()),
                  points=sum(c['points'] for c in chunks),voxel_m=.34)
    dump(DEST/'manifest.json',manifest)
    dump(REPORT/'lidar_manifest.json',manifest)
    write_csv(REPORT/'lidar_frames.csv',frames)
    print('LIDAR COMPLETE',flush=True)


if __name__=='__main__':
    main()
