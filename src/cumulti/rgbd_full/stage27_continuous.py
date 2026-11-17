"""Build Stage27 exploratory display assets from frozen Stage24 poses and ROS2 bags."""
import argparse
import csv
import json
import os
import sqlite3
import zipfile
from contextlib import closing
from pathlib import Path

import cv2
import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from scipy.spatial.transform import Rotation

from common import OUT, RAW, SITE, align_depth, dump, sha
from fusion import pose
from stage26_manifest import read_csv, write_csv

ROBOTS = ('robot1', 'robot3')
DEST = SITE / 'continuous'
OUTPUT = OUT.parent / '27_continuous_rgbd'
VOXEL = .15
STRIDE = 24
FIELDS = ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')
LAYOUT = np.diag([1., -1., -1., 1.])  # First camera's optical gauge, Y up for display.
DTYPE = np.dtype([('x','<f4'), ('y','<f4'), ('z','<f4'),
                  ('red','u1'), ('green','u1'), ('blue','u1')])


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def time_ns(row):
    return int(row['relative_ns'])


def pose7(matrix):
    return [*matrix[:3, 3].tolist(), *Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()]


def delta(a, b):
    d = np.linalg.inv(a) @ b
    return float(np.linalg.norm(d[:3, 3])), float(np.degrees(Rotation.from_matrix(d[:3, :3]).magnitude()))


def continuation(history, next_time):
    end = pose(history[-1])
    start = next((row for row in reversed(history[:-1])
                  if time_ns(history[-1]) - time_ns(row) >= 800_000_000), None)
    if start is None:
        return end
    dt = (time_ns(history[-1]) - time_ns(start)) / 1e9
    horizon = min(1., (next_time - time_ns(history[-1])) / 1e9)
    old = pose(start)
    offset = (end[:3, 3] - old[:3, 3]) / dt * horizon
    rotation = Rotation.from_matrix(old[:3,:3].T @ end[:3,:3]).as_rotvec() / dt * horizon
    prediction = end.copy()
    prediction[:3,3] += offset / max(1., np.linalg.norm(offset) / .8)
    prediction[:3,:3] @= Rotation.from_rotvec(rotation / max(1., np.linalg.norm(rotation) / .2)).as_matrix()
    return prediction


def display_graph(rows):
    graph = load(OUT.parent / '24d_rgbd_pose_graph/pose_graph.json')
    atlas = {(n['robot'], n['original_epoch']): (n['component'], np.array(n['T_atlas_from_epoch']))
             for n in graph['nodes']}
    bounded = OUT.parent / '26_rgbd_reconstruction'
    estimates = load(bounded / 'registration_transforms.json')
    results = {(r['boundary'],r['method']):r for r in read_csv(bounded / 'registration_comparison.csv')}
    rejected_cycles = {(r['boundary'],r['method']) for r in read_csv(bounded / 'rejected_edges.csv')
                       if r['final_decision']=='INCONSISTENT_CYCLE'}
    legacy = {r['boundary']:r for r in load(OUT.parent/'24e_rgbd_feasibility/dense_submap_diagnostics.json')}
    groups = {}
    for robot in ROBOTS:
        for row in rows[robot]:
            if row['valid_pose']=='1':
                groups.setdefault((robot, int(row['map_epoch'])), []).append(row)
    indices = {r:{int(row['index']):row for row in rows[r]} for r in ROBOTS}
    assert groups.keys()==atlas.keys()
    anchors, transforms, provenance, boundaries = {}, {}, {}, []
    for robot in ROBOTS:
        previous = None
        for key in (k for k in groups if k[0]==robot):
            epoch = key[1]
            first = groups[key][0]
            start = pose(first)
            component, local = atlas[key]
            prediction = LAYOUT.copy()
            source, evidence = 'DISPLAY_ORIGIN', ''
            if previous is not None:
                prediction = transforms[previous] @ continuation(groups[previous], time_ns(first))
            alternative = prediction
            if component not in anchors and previous is not None:
                source = 'DISPLAY_ONLY_ESTIMATE'
                boundary = f'{robot}:{previous[1]}->{epoch}'
                proposals = []
                for record in estimates:
                    tag = (record['boundary'], record['method'])
                    if (tag[0]!=boundary or tag in rejected_cycles or
                        results[tag]['status']!='INDEPENDENT_AGGREGATE_PASS'):
                        continue
                    i,j = record['anchors']
                    transform = (pose(indices[robot][i]) @ np.array(record['T_previous_boundary_camera_from_next'])
                                 @ np.linalg.inv(pose(indices[robot][j])))
                    proposals.append(('STAGE26_EXPLORATORY', tag[1], transform))
                prior = legacy.get(boundary, {})
                if prior.get('geometry_pass') and prior.get('separate_holdout_frames'):
                    proposals.append(('STAGE24E_RELAXED_RGBD', 'aggregate depth validation only',
                                      pose(groups[previous][-1]) @ np.array(prior['T_previous_boundary_camera_from_next'])
                                      @ np.linalg.inv(start)))
                for proposed_source, detail, candidate in proposals:
                    proposed = transforms[previous] @ candidate @ start
                    distance, angle = delta(prediction, proposed)
                    if distance<=1.5 and angle<=20:
                        alternative, source, evidence = proposed, proposed_source, detail
                        break
            reentry_conflict = None
            if component in anchors and previous is not None and atlas[previous][0]!=component:
                existing = anchors[component][1] @ local @ start
                gap = delta(transforms[previous] @ pose(groups[previous][-1]), existing)
                if gap[0]>2 or gap[1]>30:
                    reentry_conflict = gap
            if reentry_conflict:
                source = 'DISPLAY_ONLY_ESTIMATE'
                evidence = (f'Stage24D component re-entry excluded from continuous display: '
                            f'{reentry_conflict[0]:.3f} m / {reentry_conflict[1]:.2f} deg gap; frozen edge unchanged')
                origin = prediction @ np.linalg.inv(local @ start)
            elif component in anchors:
                parent, origin, inherited = anchors[component]
                same = previous is not None and atlas[previous][0]==component
                source = 'STAGE24D_VERIFIED_RELATIVE' if same or previous is None else 'STAGE24D_COMPONENT_REENTRY'
                evidence = f'rigid Stage24D component via {parent[0]} epoch {parent[1]}; placement: {inherited}'
            else:
                origin = alternative @ np.linalg.inv(local @ start)
                anchors[component] = (key, origin, source)
            matrix = origin @ local
            assert np.isfinite(matrix).all() and abs(np.linalg.det(matrix[:3,:3])-1)<1e-8
            transforms[key] = matrix
            placed = matrix @ start
            uncertainty = ('VERIFIED_RELATIVE_ONLY' if source=='STAGE24D_VERIFIED_RELATIVE' else
                           'EXPLORATORY' if 'EXPLORATORY' in source or 'RELAXED' in source else 'DISPLAY_ONLY')
            provenance[key] = dict(robot=robot,epoch=epoch,component=component,source=source,
                                   uncertainty=uncertainty,evidence=evidence,T_display_from_epoch=matrix.tolist())
            if previous is not None:
                gap_m, gap_deg = delta(transforms[previous] @ pose(groups[previous][-1]), placed)
                residual_m,residual_deg = delta(prediction, placed)
                boundaries.append(dict(robot=robot,epoch=epoch,previous_epoch=previous[1],source=source,
                                       uncertainty=uncertainty,evidence=evidence,t=time_ns(first)/1e9,
                                       last_t=time_ns(groups[previous][-1])/1e9,translation_m=gap_m,rotation_deg=gap_deg,
                                       prediction_residual_m=residual_m,prediction_residual_deg=residual_deg,
                                       large_gap=gap_m>2 or gap_deg>30))
            previous = key
    return groups, transforms, list(provenance.values()), boundaries


def open_bag(robot, records, kind, scratch):
    archive = records[kind]['archive']
    path = RAW / robot / Path(archive['archive']).name
    with zipfile.ZipFile(path) as bag:
        info = bag.getinfo(archive['member'])
        assert (info.file_size, f'{info.CRC:08x}', path.stat().st_size)==(
            archive['db_bytes'], archive['CRC32'], archive['archive_bytes'])
    db = scratch / Path(info.filename).name
    assert db.stat().st_size==info.file_size
    return sqlite3.connect(f'{db.resolve().as_uri()}?mode=ro&immutable=1', uri=True)


def message(conn, store, record, pair, kind):
    ns, topic, fmt, payload = conn.execute(
        'SELECT m.timestamp,t.name,t.type,m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE m.id=?',
        (int(pair[f'{kind}_message_id']),)).fetchone()
    assert ns==int(pair[f'{kind}_bag_ns']) and topic==record['topic']
    data = store.deserialize_cdr(payload, fmt)
    stamp = data.header.stamp
    assert stamp.sec * 10**9 + stamp.nanosec==int(pair[f'{kind}_header_ns'])
    assert data.header.frame_id==record['calibration']['frame_id']
    return data


def rgb_pixels(rgb):
    assert rgb.encoding in ('rgb8','bgr8')
    image = np.frombuffer(rgb.data,np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*3]
    image = image.reshape(rgb.height,rgb.width,3)
    return cv2.resize(image if rgb.encoding=='rgb8' else image[:,:,::-1],(640,400),interpolation=cv2.INTER_AREA)


def depth_pixels(depth, rotation, translation, k, d):
    endian = '>' if depth.is_bigendian else '<'
    fields = {field.name:field for field in depth.fields}
    assert all(fields[key].datatype==7 and fields[key].count==1 for key in 'xyz')
    dtype = np.dtype(dict(names=list('xyz'), formats=[endian+'f4']*3,
                          offsets=[fields[key].offset for key in 'xyz'], itemsize=depth.point_step))
    point = np.ndarray((depth.height,depth.width),dtype=dtype,buffer=depth.data,
                       strides=(depth.row_step,depth.point_step))
    xyz = np.stack([point[key].ravel() for key in 'xyz'],axis=1)
    return np.rint(align_depth(xyz,rotation,translation,k,d)*1000).astype(np.uint16)


def pack(xyz, color):
    block = np.empty(len(xyz),dtype=DTYPE)
    for axis, channel in enumerate(('x','y','z')):
        block[channel] = xyz[:,axis]
    for axis, channel in enumerate(('red','green','blue')):
        block[channel] = color[:,axis]
    return block.tobytes()


def export(robot, epochs, transforms, scratch, records):
    sync = {r['index']:r for r in read_csv(OUT/robot/'sync_pairs.csv')}
    coverage = {r['index']:float(r['valid_depth_fraction']) for r in read_csv(OUT/robot/'depth_coverage.csv')}
    k = np.array(records['RGB']['calibration']['K']).reshape(3,3)
    k[:2] *= .5
    d = np.array(records['RGB']['calibration']['D'])
    extrinsic = records['DEPTH']['extra']['T_color_from_depth']
    rotation, translation = np.array(extrinsic['R']),np.array(extrinsic['t_m'])
    yy,xx = np.mgrid[0:400:STRIDE,0:640:STRIDE]
    uv = np.stack((xx,yy),axis=-1).astype(np.float32)
    rays = np.column_stack((cv2.undistortPoints(uv.reshape(-1,1,2),k,d).reshape(-1,2),np.ones(uv.size//2)))
    store = get_typestore(Stores.ROS2_HUMBLE)
    frames, chunks, trajectory, occupied = [], [], [], set()
    with closing(open_bag(robot,records,'RGB',scratch)) as rgb, closing(open_bag(robot,records,'DEPTH',scratch)) as depth:
        for conn in (rgb,depth):
            for typename,encoding,definition in conn.execute('SELECT topic_type,encoding,encoded_message_definition FROM message_definitions'):
                if encoding=='ros2msg':
                    store.register(get_types_from_msg(definition,typename))
        for key, group in epochs:
            epoch = key[1]
            print(robot, 'epoch', epoch, 'valid poses',len(group), flush=True)
            for row in group:
                trajectory.append([time_ns(row)/1e9,int(row['index']),epoch,*pose7(transforms[key] @ pose(row))])
            valid = [row for row in group if sync[row['index']]['paired']=='True' and coverage[row['index']]>0]
            selected = {}
            for row in valid:
                selected.setdefault(time_ns(row)//10**9,row)
            chosen = list(selected.values())
            if valid and chosen[-1]!=valid[-1]:
                chosen.append(valid[-1])
            actions = {}
            for row in chosen:
                actions.setdefault(time_ns(row)//30_000_000_000,[]).append(row)
            for bucket, segment in actions.items():
                data = bytearray()
                steps = []
                name = f'{robot}-e{epoch:02d}-{bucket:03d}.bin'
                for row in segment:
                    pair = sync[row['index']]
                    colors = rgb_pixels(message(rgb,store,records['RGB'],pair,'rgb'))
                    zmap = depth_pixels(message(depth,store,records['DEPTH'],pair,'depth'),rotation,translation,k,d)
                    assert abs((zmap>0).mean()-coverage[row['index']])<1e-12
                    z = zmap[yy,xx].ravel()/1000.
                    keep = (z>=.3)&(z<=10)
                    camera = rays[keep]*z[keep,None]
                    camera_tf = transforms[key] @ pose(row)
                    xyz = camera @ camera_tf[:3,:3].T + camera_tf[:3,3]
                    keep_indices = []
                    for i, coordinate in enumerate(np.floor(xyz/VOXEL).astype(int)):
                        voxel = tuple(coordinate)
                        if voxel not in occupied:
                            occupied.add(voxel)
                            keep_indices.append(i)
                    data.extend(pack(xyz[keep_indices],colors[yy,xx].reshape(-1,3)[keep][keep_indices]))
                    t = time_ns(row) / 1e9
                    steps.append([t,len(data)//DTYPE.itemsize])
                    frames.append(dict(robot=robot,epoch=epoch,index=row['index'],t=t,chunk=name,
                                       count=len(keep_indices),**{k:pair[k] for k in (
                                           'rgb_message_id','rgb_bag_ns','rgb_header_ns','depth_message_id','depth_bag_ns',
                                           'depth_header_ns','bag_sync_ms')}))
                if data:
                    path = DEST/name
                    path.write_bytes(data)
                    chunks.append(dict(file=name,robot=robot,epoch=epoch,start=steps[0][0],end=steps[-1][0],
                                       points=len(data)//DTYPE.itemsize,steps=steps,bytes=len(data),sha256=sha(path)))
    return frames,chunks,trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scratch',type=Path,required=True,help='CRC-validated immutable Stage26 SQLite copies')
    args=ap.parse_args()
    DEST.mkdir(parents=True,exist_ok=True)
    OUTPUT.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    dataset = load(OUT/'dataset_manifest.json')['robots']
    rows = {r:read_csv(OUT/r/'camera_poses.csv') for r in ROBOTS}
    groups,transforms,provenance,boundaries=display_graph(rows)
    dump(OUTPUT/'epoch_display_transforms.json',provenance)
    write_csv(OUTPUT/'boundary_continuity.csv',boundaries)
    traj, all_chunks, all_frames = {},[],[]
    for robot in ROBOTS:
        frames,chunks,traj[robot] = export(robot,[(k,v) for k,v in groups.items() if k[0]==robot],
                                          transforms,args.scratch,dataset[robot])
        assert {f['epoch'] for f in frames}=={k[1] for k in groups if k[0]==robot}
        write_csv(OUTPUT/f'{robot}_trajectory.csv',[dict(zip(('t','index','epoch',*FIELDS),r)) for r in traj[robot]])
        all_chunks.extend(chunks)
        all_frames.extend(frames)
    write_csv(OUTPUT/'frame_manifest.csv',all_frames)
    manifest=dict(duration=max(f['t'] for f in all_frames),recording_duration=max(time_ns(r[-1])/1e9 for r in rows.values()),
                  robots=list(ROBOTS),compression=10,voxel_m=VOXEL,pixel_stride=STRIDE,points=sum(c['points'] for c in all_chunks),
                  frames={r:sum(f['robot']==r for f in all_frames) for r in ROBOTS},boundaries=boundaries,chunks=all_chunks,
                  transforms=provenance,source_pose_sha256={r:sha(OUT/r/'camera_poses.csv') for r in ROBOTS},
                  source_archives={r:{k:dataset[r][k]['archive'] for k in ('RGB','DEPTH')} for r in ROBOTS},
                  trajectory='trajectories.json')
    (DEST/'trajectories.json').write_text(json.dumps(traj,separators=(',',':')),encoding='utf-8')
    ply = OUTPUT/'combined_exploratory_rgbd.ply'
    with ply.open('wb') as stream:
        stream.write((f"ply\nformat binary_little_endian 1.0\nelement vertex {manifest['points']}\n"
                      "property float x\nproperty float y\nproperty float z\nproperty uchar red\n"
                      "property uchar green\nproperty uchar blue\nend_header\n").encode())
        for chunk in all_chunks:
            stream.write((DEST/chunk['file']).read_bytes())
    manifest['combined_ply']=dict(bytes=ply.stat().st_size,sha256=sha(ply))
    dump(OUTPUT/'chunk_manifest.json',manifest)
    temporary = DEST/'manifest.json.pending'
    dump(temporary,manifest)
    os.replace(temporary,DEST/'manifest.json')
    print('STAGE27_COMPLETE',manifest['frames'],manifest['points'],flush=True)


if __name__=='__main__':
    main()
