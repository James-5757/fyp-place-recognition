"""Balanced RGB retrieval over every unordered original epoch pair."""
import csv
import itertools
from collections import defaultdict

import cv2
import numpy as np
from common import OUT, dump, writecsv

DEST = OUT.parent/'24d_rgbd_pose_graph'
BASE = OUT.parent/'24_rgbd_atlas_v1'
ROBOTS = ('robot1','robot3')


def read(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def describe(robot, frame):
    image = cv2.imread(str(OUT/robot/'rgb'/f'{frame:06d}.jpg'),0)
    image = cv2.resize(image,(32,20),interpolation=cv2.INTER_AREA).astype(np.float32)
    image -= image.mean()
    appearance = image.ravel()/max(1.,np.linalg.norm(image))
    gradients = np.hypot(cv2.Sobel(image,-1,1,0),cv2.Sobel(image,-1,0,1))
    gradients -= gradients.mean()
    return np.concatenate((appearance,gradients.ravel()/max(1.,np.linalg.norm(gradients))))/np.sqrt(2)


def main():
    cv2.setNumThreads(1)
    poses = {r:read(OUT/r/'camera_poses.csv') for r in ROBOTS}
    db = defaultdict(list)
    for robot in ROBOTS:
        for row,pair in zip(poses[robot],read(OUT/robot/'sync_pairs.csv'),strict=True):
            if row['valid_pose']!='1' or pair['paired']!='True':
                continue
            epoch = robot,int(row['map_epoch'])
            stamp = int(row['relative_ns'])
            if not db[epoch] or stamp-db[epoch][-1][1]>=750_000_000:
                db[epoch].append((int(row['index']),stamp))
    vectors = {key:np.stack([describe(key[0],i) for i,_ in entries]).astype(np.float32)
               for key,entries in db.items()}
    candidates, coverage = [], []
    for a,b in itertools.combinations(sorted(db),2):
        left,right = np.array(db[a]),np.array(db[b])
        scores = vectors[a] @ vectors[b].T
        eligible = np.ones(scores.shape,dtype=bool)
        if a[0]==b[0]:
            eligible &= abs(left[:,1,None]-right[None,:,1])>=2_000_000_000
        eligible_count = int(eligible.sum())
        scores[~eligible] = -np.inf
        chosen = 0
        for rank in range(1,13):
            i,j = np.unravel_index(scores.argmax(),scores.shape)
            score = float(scores[i,j])
            if not np.isfinite(score):
                break
            ia,ta = map(int,left[i])
            ib,tb = map(int,right[j])
            candidates.append(dict(robot_a=a[0],epoch_a=a[1],index_a=ia,robot_b=b[0],epoch_b=b[1],
                                   index_b=ib,arrival_ns=max(ta,tb)+25_000_000,rank=rank,score=score))
            chosen += 1
            # Both sides must move at least one second from each selected frame.
            scores[abs(left[:,1]-ta)<1_000_000_000,:] = -np.inf
            scores[:,abs(right[:,1]-tb)<1_000_000_000] = -np.inf
        coverage.append(dict(robot_a=a[0],epoch_a=a[1],robot_b=b[0],epoch_b=b[1],
                             eligible_frame_pairs=eligible_count,candidates=chosen))
    writecsv(DEST/'keyframes.csv',[dict(robot=r,epoch=e,index=i,stamp_ns=t)
                                  for (r,e),entries in sorted(db.items()) for i,t in entries])
    writecsv(DEST/'epoch_pair_coverage.csv',coverage)
    writecsv(DEST/'candidates.csv',sorted(candidates,key=lambda c:(c['arrival_ns'],c['robot_a'],c['index_a'],c['index_b'])))
    dump(DEST/'retrieval.json',dict(keyframes=sum(map(len,db.values())),epoch_pairs=len(coverage),
         epoch_pairs_with_candidates=sum(bool(c['candidates']) for c in coverage),candidates=len(candidates),
         keyframe_interval_ns=750_000_000,top_k=12,same_robot_exclusion_ns=2_000_000_000,
         per_endpoint_diversity_ns=1_000_000_000))


if __name__=='__main__':
    main()
