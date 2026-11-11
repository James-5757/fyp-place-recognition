"""Reproject measured RGB-D into the final connected Atlas map coordinates."""
import csv
import json
import sys
from collections import Counter, defaultdict

import numpy as np
from scipy.spatial.transform import Rotation
from atlas_geometry import DEST
from common import OUT, SITE, dump, sha, writecsv
from fusion import pose
from maps import build

POSE_KEYS = ('tx','ty','tz','qx','qy','qz','qw')
ROBOTS = ('robot1','robot3')


def read(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def prepare(robot):
    folder = DEST/robot
    assert (folder/'tracking.done').exists()
    online = read(folder/'online.csv')
    pairs = read(OUT/robot/'sync_pairs.csv')
    corrected = defaultdict(list)
    for path in sorted(folder.glob('corrected.csv.*.tum')):
        component = int(path.name.split('.')[-2])
        for line in path.read_text().splitlines():
            stamp, *values = line.split()
            assert len(values) == len(POSE_KEYS)
            corrected[round(float(stamp))].append(dict(zip(POSE_KEYS, values), component=component))
    components = Counter()
    epoch_coverage = defaultdict(set)
    online_valid = 0
    for index, (row, pair) in enumerate(zip(online, pairs, strict=True)):
        assert int(row['index']) == index and row['relative_ns'] == pair['relative_ns']
        online_good = row['valid_pose'] == '1'
        online_valid += online_good
        stamp = int(row['relative_ns'])
        choices = corrected.get(stamp, [])
        final = choices[0] if online_good and len(choices) == 1 else None
        if final:
            values = np.array([float(final[k]) for k in POSE_KEYS])
            assert np.isfinite(values).all() and abs(np.linalg.norm(values[3:])-1) < 1e-5
            component = int(final['component'])
            components[component] += 1
            epoch_coverage[component].add((int(row['map_id']), int(row['map_init_kf'])))
        else:
            component = -1
        row.update(map_epoch=component, valid_pose='1' if final else '0')
        row.update({key: final[key] if final else '' for key in POSE_KEYS})
    assert components, robot
    writecsv(folder/'camera_poses.csv', online)
    duration = int(online[-1]['relative_ns']) / 1e9
    metadata = dict(duration_s=duration, frames=len(online), valid_frames=sum(components.values()),
                    online_valid=online_valid, map_epochs=len(components), components=dict(components),
                    online_fingerprints_per_component={key:sorted(v) for key,v in epoch_coverage.items()},
                    atlas_sha256=sha(folder/'final_atlas.osa'), atlas_bytes=(folder/'final_atlas.osa').stat().st_size,
                    ambiguous_export_timestamps=sum(len(v)>1 for v in corrected.values()))
    dump(folder/'tracking_metrics.json', metadata)
    return online, metadata


def prepare_registered(robot):
    """Connect only neighboring recorded epochs with repeated RGB-D agreement."""
    rows = read(OUT/robot/'camera_poses.csv')
    history = DEST/'local_edges_history.json'
    edges = json.loads((history if history.exists() else DEST/'local_edges.json').read_text())
    groups = defaultdict(list)
    for edge in edges:
        if edge['robot'] != robot:
            continue
        left, right = (rows[edge[key]] for key in ('robot1_index','robot3_index'))
        epochs = int(left['map_epoch']), int(right['map_epoch'])
        if epochs[0] == epochs[1]:
            continue
        transform = pose(left) @ np.array(edge['robot1_from_robot3']) @ np.linalg.inv(pose(right))
        groups[epochs].append((edge,transform))
    qualified = []
    for epochs, candidates in sorted(groups.items()):
        for edge, transform in candidates:
            support = []
            for other, test in candidates:
                delta = np.linalg.inv(transform) @ test
                if np.linalg.norm(delta[:3,3]) <= .2 and np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()) <= 3:
                    support.append(other)
            if all(len({e[key] for e in support}) >= 3
                   and np.ptp([int(rows[e[key]]['relative_ns']) for e in support]) >= 800_000_000
                   for key in ('robot1_index','robot3_index')):
                qualified.append((epochs,transform,support))
                break
    epoch_ids = sorted({int(row['map_epoch']) for row in rows if row['valid_pose']=='1'})
    parent = {epoch:epoch for epoch in epoch_ids}
    graph = defaultdict(list)
    links = []

    def root(epoch):
        while parent[epoch] != epoch:
            epoch = parent[epoch]
        return epoch

    for (left,right),transform,support in sorted(qualified,key=lambda link:(-len(link[2]),link[0])):
        if root(left) == root(right):
            continue
        parent[root(right)] = root(left)
        graph[left].append((right,transform))
        graph[right].append((left,np.linalg.inv(transform)))
        links.append(dict(left=left,right=right,support=support))
    transforms, components = {}, {}
    for epoch in epoch_ids:
        if epoch in components:
            continue
        components[epoch], transforms[epoch] = epoch, np.eye(4)
        queue = [epoch]
        for current in queue:
            for neighbor, transform in graph[current]:
                if neighbor not in components:
                    components[neighbor] = epoch
                    transforms[neighbor] = transforms[current] @ transform
                    queue.append(neighbor)
    count = Counter()
    for row in rows:
        if row['valid_pose'] != '1':
            continue
        original = int(row['map_epoch'])
        matrix = transforms[original] @ pose(row)
        row.update(zip(POSE_KEYS, [*matrix[:3,3],*Rotation.from_matrix(matrix[:3,:3]).as_quat()]))
        row['map_epoch'] = components[original]
        count[components[original]] += 1
    folder = DEST/robot
    writecsv(folder/'camera_poses.csv',rows)
    metadata = dict(duration_s=int(rows[-1]['relative_ns'])/1e9, frames=len(rows),
                    valid_frames=sum(count.values()), map_epochs=len(count),
                    components=dict(count), original_epochs=components,
                    verified_links=links)
    dump(folder/'tracking_metrics.json',metadata)
    return rows,metadata


def bridges(rows):
    edges = json.loads((DEST/'camera_edges_dense.json').read_text())
    by_component = defaultdict(list)
    accepted = []
    for edge in edges:
        i, j = edge['robot1_index'], edge['robot3_index']
        a, b = rows['robot1'][i], rows['robot3'][j]
        if a['valid_pose'] != '1' or b['valid_pose'] != '1':
            continue
        components = a['map_epoch'], b['map_epoch']
        transform = pose(a) @ np.array(edge['robot1_from_robot3']) @ np.linalg.inv(pose(b))
        previous = by_component[components]
        previous.append((edge, transform))
        if any((x['robot1_epoch'], x['robot3_epoch']) == components for x in accepted):
            continue
        for anchor, anchor_transform in previous:
            support = []
            for candidate, candidate_transform in previous:
                delta = np.linalg.inv(anchor_transform) @ candidate_transform
                if np.linalg.norm(delta[:3,3]) <= .5 and np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()) <= 5:
                    support.append(candidate)
            if len({x['robot1_index'] for x in support}) < 3 or len({x['robot3_index'] for x in support}) < 3:
                continue
            if any(np.ptp([int(rows[r][x[r+'_index']]['relative_ns']) for x in support]) < 2e9 for r in ROBOTS):
                continue
            accepted.append(dict(robot1_epoch=components[0], robot3_epoch=components[1],
                                 available_s=edge['arrival_ns']/1e9, transform=anchor_transform.tolist(),
                                 support=support))
            break
    return accepted


def export_merged(bridge):
    """Export measured RGB points for a verified component pair."""
    dtype = np.dtype([('x','<f4'), ('y','<f4'), ('z','<f4'),
                      ('r','u1'), ('g','u1'), ('b','u1')])
    parts = []
    transform = np.array(bridge['transform'])
    for robot in ROBOTS:
        component = bridge[robot+'_epoch']
        with (DEST/robot/'maps'/f'epoch_{component:04d}.ply').open('rb') as source:
            for line in source:
                if line.strip() == b'end_header':
                    break
            else:
                raise ValueError('Missing PLY header')
            points = np.frombuffer(source.read(),dtype=dtype).copy()
        if robot == 'robot3':
            xyz = np.column_stack([points[key] for key in 'xyz'])
            xyz = xyz @ transform[:3,:3].T + transform[:3,3]
            for column,key in enumerate('xyz'):
                points[key] = xyz[:,column]
        parts.append(points)
    name = f"merged_r1_{bridge['robot1_epoch']}_r3_{bridge['robot3_epoch']}.ply"
    path = DEST/name
    count = sum(len(part) for part in parts)
    header = (f'ply\nformat binary_little_endian 1.0\nelement vertex {count}\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
    with path.open('wb') as handle:
        handle.write(header.encode())
        for part in parts:
            handle.write(part.tobytes())
    return dict(file=name, points=count, bytes=path.stat().st_size, sha256=sha(path))


def main():
    site = SITE/'atlas'
    registered = '--registered' in sys.argv
    build_poses = prepare_registered if registered else prepare
    prepared = {robot:build_poses(robot) for robot in ROBOTS}
    rows = {robot:value[0] for robot,value in prepared.items()}
    tracking = {robot:value[1] for robot,value in prepared.items()}
    datasets = {robot:build(robot,output_root=DEST,asset_root=site,reuse_rgb=True) for robot in ROBOTS}
    accepted = bridges(rows)
    for bridge in accepted:
        for robot in ROBOTS:
            assert any(e['epoch']==bridge[robot+'_epoch'] for e in datasets[robot]['epochs'])
        bridge['merged_ply'] = export_merged(bridge)
    summary = dict(robots=tracking, bridges=accepted, attempts_ns=sorted(
        int(r['arrival_ns']) for r in read(OUT/'causal_rgbd/rgb_candidates.csv')),
        geometry_tested=len(read(DEST/'bridge_checks_dense.csv')),
        camera_pairs_verified=len(json.loads((DEST/'camera_edges_dense.json').read_text())),
        pose_source=('recorded online poses registered across verified RGB-D epochs'
                     if registered else 'final Atlas component coordinates'),
        lidar=False,gt=False)
    dump(DEST/'summary.json', summary)
    dump(site/'bridge_summary.json', summary)
    dump(site/'manifest.json', dict(robots={r:f'atlas/{r}/data.json' for r in ROBOTS},
         duration_s=max(m['duration_s'] for m in tracking.values()), stage20_reference_s=1972.508,
         registered_epochs=registered))
    print(json.dumps(summary)[:2000], flush=True)


if __name__ == '__main__':
    main()
