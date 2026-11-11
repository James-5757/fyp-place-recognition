"""Stage22A arrival ordering for the frozen camera-only verification policy."""
import csv
import json
import os
import subprocess
from collections import Counter

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from common import OUT, ROOT, SITE, dump, writecsv
from fusion import ORB, POLICY, verify_candidates


def main():
    cv2.setNumThreads(1)
    out = OUT / 'causal_rgbd'
    out.mkdir(exist_ok=True)
    rows, calibration, events = {}, {}, []
    for robot in ('robot1', 'robot3'):
        folder = OUT / robot
        with (folder / 'camera_poses.csv').open() as handle:
            rows[robot] = list(csv.DictReader(handle))
        with (folder / 'sync_pairs.csv').open() as handle:
            pairs = list(csv.DictReader(handle))
        camera = json.loads((folder / 'RGB_audit.json').read_text())['calibration']
        intrinsics = np.array(camera['K']).reshape(3, 3)
        intrinsics[:2] *= .5
        calibration[robot] = (intrinsics, np.array(camera['D']))
        next_arrival = next_query = 0
        for index, (pose, pair) in enumerate(zip(rows[robot], pairs, strict=True)):
            stamp = int(pose['relative_ns'])
            if pose['valid_pose'] != '1' or pair['paired'] != 'True' or stamp < next_arrival:
                continue
            query = stamp >= next_query
            next_arrival = stamp + 10**9
            if query:
                next_query = stamp + 5 * 10**9
            # The complete 25 ms matching window is observable at this point.
            events.append((stamp + 25_000_000, robot, index, int(query), ROOT / pair['rgb']))
    events.sort()
    sequence = out / 'sequence.txt'
    sequence.write_text(''.join(f'{r} {i} {t} {q} {path}\n' for t, r, i, q, path in events))
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    env['LD_LIBRARY_PATH'] = ':'.join(map(str, [ORB/'lib', ORB/'Thirdparty/DBoW2/lib', ORB/'Thirdparty/g2o/lib', ORB.parent/'pangolin-install/lib']))
    candidates = out / 'rgb_candidates.csv'
    subprocess.run([OUT/'bin/rgbd_propose_causal', ORB/'Vocabulary/ORBvoc.txt', sequence, candidates], env=env, check=True)
    order = {(robot, index): k for k, (_, robot, index, _, _) in enumerate(events)}
    with candidates.open() as handle:
        proposals = list(csv.DictReader(handle))
    for pair in proposals:
        robot = pair['query_robot']
        other = 'robot3' if robot == 'robot1' else 'robot1'
        assert order[other, int(pair['candidate_index'])] < order[robot, int(pair['query_index'])]
    records, accepted = verify_candidates(rows, calibration, proposals)
    availability = {(r, i): t for t, r, i, _, _ in events}
    writecsv(out/'rgbd_geometric_verification.csv', records, list(dict.fromkeys(k for r in records for k in r)))
    groups, seen, admitted, admission_times = {}, set(), [], []
    for edge in accepted:
        pair = (edge['query_index'], edge['candidate_index'])
        if pair in seen:
            continue
        seen.add(pair)
        epochs = (edge['query_epoch'], edge['candidate_epoch'])
        if epochs in admitted:
            continue
        prefix = groups.setdefault(epochs, [])
        prefix.append(edge)
        for anchor in prefix:
            transform = np.array(anchor['T_robot1_epoch_from_robot3_epoch'])
            support = []
            for other in prefix:
                delta = np.linalg.inv(transform) @ np.array(other['T_robot1_epoch_from_robot3_epoch'])
                if np.linalg.norm(delta[:3, 3]) <= POLICY['max_support_translation_m'] and np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude()) <= POLICY['max_support_rotation_deg']:
                    support.append(other)
            if (len({e['query_index'] for e in support}) >= 3 and len({e['candidate_index'] for e in support}) >= 3
                    and np.ptp([e['query_s'] for e in support]) >= 2 and np.ptp([e['candidate_s'] for e in support]) >= 2):
                admitted.append(epochs)
                admission_times.append(dict(robot1_epoch=epochs[0], robot3_epoch=epochs[1],
                                            available_s=max(availability['robot1', pair[0]],
                                                            availability['robot3', pair[1]])/1e9,
                                            transform=transform.tolist(), support=support))
                break
    result = dict(arrivals=Counter(r for _,r,_,_,_ in events),
                  queries=Counter(r for _,r,_,q,_ in events if q),
                  tested=Counter(p['query_robot'] for p in proposals),
                  rejection_reasons=Counter(r['reason'] for r in records if not r['accepted']),
                  pairing_window_ns=25_000_000, future_candidates=0, verified_pairs=len(accepted), bridges=admission_times,
                  attempts_ns=[int(p['arrival_ns']) for p in proposals])
    dump(out/'summary.json', result)
    dump(SITE/'causal_summary.json', result)
    print({k:v for k,v in result.items() if k != 'attempts_ns'})


if __name__ == '__main__':
    main()
