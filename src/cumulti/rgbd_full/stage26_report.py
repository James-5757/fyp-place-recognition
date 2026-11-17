"""Finish the bounded evaluation without modifying the Stage24D graph."""
import csv
import json
from collections import Counter, defaultdict, deque

import numpy as np
from scipy.spatial.transform import Rotation

from common import OUT, dump
from fusion import pose
from stage26_manifest import read_csv
from stage26_recover import DEST


def write_rows(path, rows, fields):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def main():
    gate = json.loads((DEST/'stage26_gate1.json').read_text())
    assert gate['status']=='PASS' and gate['verified_assets']==748
    rows = read_csv(DEST/'registration_comparison.csv')
    assert len(rows)==56 and len({(r['boundary'],r['method']) for r in rows})==56
    transforms = {(r['boundary'],r['method']):r for r in json.loads((DEST/'registration_transforms.json').read_text())}
    valid, weights = {}, Counter()
    for robot in ('robot1','robot3'):
        sync = {r['index']:r for r in read_csv(OUT/robot/'sync_pairs.csv')}
        depth = {r['index']:float(r['valid_depth_fraction']) for r in read_csv(OUT/robot/'depth_coverage.csv')}
        for row in read_csv(OUT/robot/'camera_poses.csv'):
            if row['valid_pose']=='1':
                valid[robot,int(row['index'])] = row
                weights[robot,int(row['map_epoch'])] += sync[row['index']]['paired']=='True' and depth[row['index']]>0
    baseline = json.loads((OUT.parent/'24d_rgbd_pose_graph/pose_graph.json').read_text())
    nodes = [(row['robot'],row['original_epoch']) for row in baseline['nodes']]
    adjacency = defaultdict(list)

    def insert(a,b,transform):
        adjacency[a].append((b,transform))
        adjacency[b].append((a,np.linalg.inv(transform)))

    def path(a,b):
        queue = deque([(a,np.eye(4))])
        seen = {a}
        while queue:
            node, transform = queue.popleft()
            if node==b:
                return transform
            for neighbor, edge in adjacency[node]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    queue.append((neighbor,transform@edge))
        return None

    def metrics():
        count = set(nodes)
        components = []
        while count:
            start = min(count)
            component = {node for node in count if path(start,node) is not None}
            count -= component
            components.append(sorted(component))
        largest = max(sum(weights[node] for node in c) for c in components)
        return dict(components=len(components), largest_valid_rgbd_frames=largest,
                    total_valid_rgbd_frames=sum(weights.values()), largest_fraction=largest/sum(weights.values()),
                    membership=components)

    for entry in baseline['edges']:
        if entry['used_in_forest'] and entry['graph_consistent']:
            insert(tuple(entry['a']),tuple(entry['b']),np.array(entry['T_a_from_b']))
    before = metrics()
    accepted, rejected = [], []
    qualified = sorted((r for r in rows if r['bounded_bridge_qualified']=='True'),
                       key=lambda r:(r['boundary'], -int(r['verified_repetitions']),
                                     max(float(r['median_a_m']),float(r['median_b_m'])),r['method']))
    for row in qualified:
        robot, epochs = row['boundary'].split(':')
        a,b = [(robot,int(epoch)) for epoch in epochs.split('->')]
        record = transforms[row['boundary'],row['method']]
        i,j = record['anchors']
        assert (robot,int(valid[robot,i]['map_epoch']))==a and (robot,int(valid[robot,j]['map_epoch']))==b
        t = pose(valid[robot,i])@np.array(record['T_previous_boundary_camera_from_next'])@np.linalg.inv(pose(valid[robot,j]))
        predicted = path(a,b)
        distance,angle = None,None
        if predicted is not None:
            delta = np.linalg.inv(predicted)@t
            distance = float(np.linalg.norm(delta[:3,3]))
            angle = float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()))
        enriched = dict(row,cycle_translation_m=distance,cycle_rotation_deg=angle)
        if distance is not None and (distance>.2 or angle>3):
            rejected.append(dict(enriched,final_decision='INCONSISTENT_CYCLE'))
        elif any(r['boundary']==row['boundary'] for r in accepted):
            rejected.append(dict(enriched,final_decision='DUPLICATE_METHOD_FOR_SAME_EDGE'))
        else:
            if predicted is None:
                insert(a,b,t)
            accepted.append(dict(enriched,final_decision='NEW_CONNECTIVITY' if predicted is None else 'CONSISTENT_REDUNDANT_EDGE',
                                 T_a_from_b=json.dumps(t.tolist())))
    rejected += [dict(row,final_decision=row['bridge_decision']) for row in rows if row['bounded_bridge_qualified']!='True']
    fields = list(rows[0])+['cycle_translation_m','cycle_rotation_deg','final_decision']
    write_rows(DEST/'accepted_edges.csv',accepted,fields+['T_a_from_b'])
    write_rows(DEST/'rejected_edges.csv',rejected,fields)
    after = metrics()
    admitted = {(r['boundary'], r['method']) for r in accepted}
    for row in rows:
        row['graph_admitted'] = str((row['boundary'], row['method']) in admitted)
    write_rows(DEST/'registration_comparison.csv', rows, list(rows[0]))
    write_rows(DEST/'nonadjacent_registration_results.csv',
               [r for r in rows if r['cohort']=='nonadjacent'], list(rows[0]))
    previews = []
    if accepted:
        import open3d as o3d
        submaps = read_csv(DEST/'submap_manifest.csv')
        for edge in accepted:
            record = transforms[edge['boundary'],edge['method']]
            assets = [r['asset'] for r in submaps if r['case']==edge['boundary']]
            if len(assets)!=2 or not all(assets):
                continue
            left,right = [o3d.io.read_point_cloud(str(DEST/name)) for name in assets]
            right.transform(np.array(record['T_previous_boundary_camera_from_next']))
            merged = (left+right).voxel_down_sample(.05)
            folder = DEST/'previews'
            folder.mkdir(exist_ok=True)
            name = edge['boundary'].replace(':','_').replace('->','_')+'.ply'
            assert o3d.io.write_point_cloud(str(folder/name),merged)
            previews.append(dict(boundary=edge['boundary'],points=len(merged.points),asset=f'previews/{name}'))
    dump(DEST/'connectivity_comparison.json',dict(before=before,after=after,accepted_edges=len(accepted),
         new_connecting_edges=sum(r['final_decision']=='NEW_CONNECTIVITY' for r in accepted),
         previews=previews,historical_graph_modified=False,full_reconstruction='NO_GO_BOUNDED_EVIDENCE_ONLY'))
    notes = ['# Stage26 bounded RGB-D registration', '',
             'Recovery gate passed for all 748 assets. Message provenance and SHA256 are in the export receipts.',
             'The boundary cohort uses the original Stage24E fit/holdout frame IDs. For the four non-adjacent cases, '
             'the candidate ranking and alternating split were frozen before decoding. The baseline uses identity '
             'when no frozen fit-only PnP proposal is available.',
             'Residual columns are directional medians among held-out correspondences within 12 cm. '
             'Inlier counts and overlap columns retain the two directions. A depth pass on an aggregate submap '
             'does not by itself qualify an edge.',
             '', '| Method | Trials | Aggregate depth passes on independent frames | Qualified bridges | Non-adjacent qualified |',
             '|---|---:|---:|---:|---:|']
    for method in ('baseline','fpfh_ransac'):
        selected = [r for r in rows if r['method']==method]
        notes.append(f"| {method} | {len(selected)} | {sum(r['status']=='INDEPENDENT_AGGREGATE_PASS' for r in selected)} | "
                     f"{sum(r['bounded_bridge_qualified']=='True' for r in selected)} | "
                     f"{sum(r['bounded_bridge_qualified']=='True' and r['cohort']=='nonadjacent' for r in selected)} |")
    reproduced = {(r['boundary'],r['method']):r for r in rows}
    original = json.loads((OUT.parent/'24e_rgbd_feasibility/dense_submap_diagnostics.json').read_text())
    mismatches = []
    for case in original:
        result = reproduced[case['boundary'],'baseline']
        depth_pass = result['status'] in ('INDEPENDENT_AGGREGATE_PASS','REUSED_VALIDATION_FRAME')
        if depth_pass != case['geometry_pass']:
            mismatches.append(case['boundary'])
    dump(DEST/'baseline_parity.json',dict(cases=len(original), same_depth_gate_decision=len(original)-len(mismatches),
         mismatches=mismatches, convention='T_a_from_b = T_epoch_a_from_camera_a @ T_camera_a_from_camera_b @ inverse(T_epoch_b_from_camera_b)'))
    notes += ['',f"Frozen Stage24E baseline depth-gate decisions reproduced: {len(original)-len(mismatches)}/{len(original)}.",
              f"Stage24D components: {before['components']}. Temporary diagnostic graph: {after['components']}.",
              f"Largest component valid RGB-D frame coverage: {before['largest_fraction']:.2%} → {after['largest_fraction']:.2%}.",
              '', 'Rejected method-case outcomes:', '']
    notes += [f'- {reason}: {count}' for reason,count in Counter(r['final_decision'] for r in rejected).most_common()]
    notes += ['', 'Independent repetition failures (a repetition may fail several checks):', '']
    for method in ('baseline', 'fpfh_ransac'):
        failures = Counter()
        trials = 0
        for record in transforms.values():
            if record['method'] != method:
                continue
            for attempt in record['independent_repeatability']:
                trials += 1
                if attempt['accepted']:
                    continue
                if 'metrics' not in attempt:
                    failures[attempt['reason']] += 1
                    continue
                independent, aggregate = attempt['metrics'], attempt['aggregate_metrics']
                if not (independent['pass_depth'] and aggregate['pass_depth']):
                    failures['HELDOUT_DEPTH_GATE'] += 1
                if min(independent['spread']+aggregate['spread']) < .05:
                    failures['WEAK_SPATIAL_SPREAD'] += 1
                if attempt['disagreement_m'] > .2 or attempt['disagreement_deg'] > 3:
                    failures['SE3_DISAGREEMENT'] += 1
        notes.append(f"- {method}, {trials} attempts: " + ', '.join(f'{key}={value}' for key,value in failures.most_common()))
    for row in rejected:
        if row['final_decision']=='INCONSISTENT_CYCLE':
            notes.append(f"- {row['boundary']} ({row['method']}) passed independent validation but disagreed with "
                         f"the frozen path by {row['cycle_translation_m']:.6f} m and {row['cycle_rotation_deg']:.3f} deg "
                         "(limits 0.2 m and 3 deg).")
    if not accepted:
        notes += ['', 'No bridge was admitted and no merged preview was generated. FPFH RANSAC increased aggregate '
                  'depth passes but produced no independently repeatable bridge. The baseline depth parity supports '
                  'the implementation convention check. The held-out gates cannot by themselves distinguish absent '
                  'physical overlap from a wrong proposal. No threshold sensitivity study was run.',
                  '', 'Next experiment: rank boundary windows by measured depth coverage, spatial spread and '
                  'independent temporal support. Expand the windows only within their original epochs, freeze a new '
                  'disjoint split, and test depth-backed RGB feature matches as a coarse initializer under the same gates.']
    notes += ['', 'Full-recording reconstruction remains NO-GO. The experiment is deliberately bounded.']
    (DEST/'stage26_registration_report.md').write_text('\n'.join(notes)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
