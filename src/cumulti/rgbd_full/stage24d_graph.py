"""Build a persistent, measured SE(3) forest over original tracking epochs."""
import json
from collections import Counter, defaultdict

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from atlas_replay import export_merged
from common import OUT, SITE, dump, writecsv
from fusion import pose
from maps import build
from stage24d_retrieve import BASE, DEST, ROBOTS, read


def residual(expected, measured):
    delta = np.linalg.inv(expected) @ measured
    return float(np.linalg.norm(delta[:3,3])), float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()))


def collect_measurements(rows):
    groups = defaultdict(dict)
    for folder,name in ((BASE,'local_edges_history.json'), (BASE,'camera_edges_dense.json'),
                        (DEST,'camera_edges_dense.json')):
        for edge in json.loads((folder/name).read_text()):
            ra = edge.get('robot','robot1')
            rb = edge.get('target_robot', 'robot3' if folder==BASE and name=='camera_edges_dense.json' else ra)
            ia,ib = edge['robot1_index'],edge['robot3_index']
            a,b = rows[ra][ia],rows[rb][ib]
            left,right = (ra,int(a['map_epoch'])), (rb,int(b['map_epoch']))
            if left == right:
                continue
            transform = pose(a) @ np.array(edge['robot1_from_robot3']) @ np.linalg.inv(pose(b))
            if right < left:
                left,right,ia,ib = right,left,ib,ia
                transform = np.linalg.inv(transform)
            groups[left,right][ia,ib] = dict(i=ia,j=ib,T=transform.tolist(),
                time_ns=edge['arrival_ns'], inliers=edge['inliers'],
                depth_checks=edge['independent_depth_checks'],
                depth_residual_m=edge['depth_median_m'], overlap_min=edge.get('depth_overlap_min'))
    return groups


def verify_links(groups, rows):
    links = []
    for (a,b), measurements in sorted(groups.items()):
        cross = a[0]!=b[0]
        limit_m,limit_deg,span_ns = (.5,5,2e9) if cross else (.2,3,.8e9)
        prefix = []
        for measurement in sorted(measurements.values(),key=lambda m:(m['time_ns'],m['i'],m['j'])):
            prefix.append(measurement)
            candidates = []
            for anchor in prefix:
                support = []
                for match in prefix:
                    dt,dr = residual(anchor['T'],match['T'])
                    if dt<=limit_m and dr<=limit_deg:
                        support.append(dict(match, delta_m=dt, delta_deg=dr))
                if len(support)<3:
                    continue
                if not all(len(indices:={s[key] for s in support})>=3 and
                           np.ptp([int(rows[robot][i]['relative_ns']) for i in indices])>=span_ns
                           for robot,key in ((a[0],'i'),(b[0],'j'))):
                    continue
                candidates.append((support,anchor))
            if candidates:
                support,anchor = min(candidates,key=lambda x:(-len(x[0]),sum(s['delta_m'] for s in x[0]),x[1]['depth_residual_m']))
                links.append(dict(a=a,b=b,T_a_from_b=anchor['T'],support=support,
                                  available_ns=measurement['time_ns'],cross=cross))
                break
    return links


def estimate(nodes,links):
    parents = {node:node for node in nodes}
    adjacency = defaultdict(list)

    def root(node):
        while parents[node]!=node:
            node=parents[node]
        return node

    for link in sorted(links,key=lambda e:(-len(e['support']),e['available_ns'],e['a'],e['b'])):
        a,b = link['a'],link['b']
        transform = np.array(link['T_a_from_b'])
        link['used_in_forest'] = root(a)!=root(b)
        if link['used_in_forest']:
            parents[root(b)] = root(a)
            adjacency[a].append((b,transform))
            adjacency[b].append((a,np.linalg.inv(transform)))
    transforms, components = {},{}
    for node in nodes:
        if node in components:
            continue
        component = len(set(components.values()))
        transforms[node],components[node] = np.eye(4),component
        queue = [node]
        for current in queue:
            for neighbor,transform in adjacency[current]:
                if neighbor not in components:
                    transforms[neighbor] = transforms[current] @ transform
                    components[neighbor] = component
                    queue.append(neighbor)
    for link in links:
        predicted = np.linalg.inv(transforms[link['a']]) @ transforms[link['b']]
        dt,dr = residual(predicted,link['T_a_from_b'])
        link.update(graph_residual_m=dt,graph_residual_deg=dr)
        limit_m,limit_deg = (.5,5) if link['cross'] else (.2,3)
        link['graph_consistent'] = dt<=limit_m and dr<=limit_deg
    return transforms,components


def merged_overlap(component):
    """Directional nearest-neighbor overlap of the two reprojected component maps."""
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','u1',3)])
    clouds=[]
    for robot in ROBOTS:
        path=DEST/robot/'maps'/f'epoch_{component:04d}.ply'
        with path.open('rb') as handle:
            for line in handle:
                if line.strip()==b'end_header':
                    break
            else:
                raise ValueError(f'Missing PLY header: {path}')
            data=np.frombuffer(handle.read(),dtype=dtype)
        clouds.append(np.column_stack([data[k] for k in 'xyz']))
    result={}
    for index,robot in enumerate(ROBOTS):
        source,target=clouds[index],clouds[1-index]
        sample=source[::max(1,int(np.ceil(len(source)/30000)))]
        distance=cKDTree(target).query(sample)[0]
        inliers=distance<=.12
        result[robot]=dict(samples=len(sample),overlap_12cm=float(inliers.mean()),
                           median_nearest_m=float(np.median(distance)),
                           overlap_median_m=float(np.median(distance[inliers])) if inliers.any() else None)
    return result


def main():
    site = SITE/'stage24d'
    rows = {r:read(OUT/r/'camera_poses.csv') for r in ROBOTS}
    nodes = sorted({(r,int(row['map_epoch'])) for r in ROBOTS for row in rows[r] if row['valid_pose']=='1'})
    links = verify_links(collect_measurements(rows),rows)
    transforms,components = estimate(nodes,links)
    members = defaultdict(list)
    for node in nodes:
        members[components[node]].append(node)
    graph = dict(estimator='maximum support spanning forest; non-tree closures audited',
                 nodes=[dict(robot=r,original_epoch=e,component=components[r,e],
                             T_atlas_from_epoch=transforms[r,e].tolist()) for r,e in nodes],
                 edges=links,components=dict(members))
    dump(DEST/'pose_graph.json',graph)
    tracking = {}
    for robot in ROBOTS:
        count = Counter()
        for row in rows[robot]:
            epoch=int(row['map_epoch'])
            row['original_epoch']=epoch
            if row['valid_pose']!='1':
                continue
            node=robot,epoch
            matrix=transforms[node] @ pose(row)
            row.update(zip(('tx','ty','tz','qx','qy','qz','qw'),
                           [*matrix[:3,3],*Rotation.from_matrix(matrix[:3,:3]).as_quat()]))
            row['map_epoch']=components[node]
            count[components[node]]+=1
        metadata=dict(duration_s=int(rows[robot][-1]['relative_ns'])/1e9,frames=len(rows[robot]),
                      valid_frames=sum(count.values()),map_epochs=len(count),components=dict(count))
        writecsv(DEST/robot/'camera_poses.csv',rows[robot])
        dump(DEST/robot/'tracking_metrics.json',metadata)
        tracking[robot]=metadata
    datasets={r:build(r,output_root=DEST,asset_root=site,reuse_rgb=True) for r in ROBOTS}
    merged=[]
    for component,nodes_in_component in members.items():
        if len({r for r,_ in nodes_in_component})!=2:
            continue
        tree=[e for e in links if e['used_in_forest'] and components[e['a']]==component]
        bridge=dict(robot1_epoch=component,robot3_epoch=component,transform=np.eye(4).tolist(),
                    available_s=max(e['available_ns'] for e in tree)/1e9)
        bridge['merged_ply']=export_merged(bridge,destination=DEST)
        bridge['whole_map_overlap']=merged_overlap(component)
        merged.append(bridge)
    events=[dict(time_s=e['available_ns']/1e9,a=e['a'],b=e['b'],support=len(e['support']),
                 component=components[e['a']]) for e in links if e['graph_consistent']]
    events.sort(key=lambda event:event['time_s'])
    summary=dict(robots=tracking,original_epochs=len(nodes),connected_components=len(members),
                 verified_links=len(links),graph_consistent_links=sum(e['graph_consistent'] for e in links),
                 intra_links=sum(not e['cross'] for e in links),inter_links=sum(e['cross'] for e in links),
                 bridges=sorted(merged,key=lambda b:b['available_s']),events=events,
                 attempts_ns=[int(r['arrival_ns']) for r in read(DEST/'candidates.csv')],
                 geometry_tested=len(read(DEST/'bridge_checks_dense.csv')),
                 points={r:sum(e['points'] for e in d['epochs']) for r,d in datasets.items()},
                 pose_source='offline verified RGB-D epoch pose graph',gt=False,lidar=False)
    dump(DEST/'summary.json',summary)
    dump(site/'bridge_summary.json',summary)
    dump(site/'manifest.json',dict(robots={r:f'stage24d/{r}/data.json' for r in ROBOTS},
         duration_s=max(m['duration_s'] for m in tracking.values()),stage20_reference_s=1972.508,
         registered_epochs=True,stage24d=True,components=dict(members),events=events))
    print({k:summary[k] for k in ('original_epochs','connected_components','verified_links','intra_links','inter_links','points')},flush=True)


if __name__=='__main__':
    main()
