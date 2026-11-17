"""Read-only Robot3 RGB-D and IMU pose forensics against frozen Stage29."""
import argparse
import hashlib
import json
import sqlite3
from collections import defaultdict
from contextlib import ExitStack, closing
from functools import lru_cache
from itertools import combinations
from pathlib import Path
import zipfile

import cv2
import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from scipy.spatial import cKDTree

from common import OUT, RAW, dump, sha
from fusion import pose, rigid
from stage26_manifest import read_csv, write_csv
from stage26_register import fit_icp
from stage27_continuous import delta, depth_pixels, message, open_bag, rgb_pixels

ROBOT = 'robot3'
BASE = OUT.parent
DEST = BASE/'30_rgbd_pose_forensics'
MATRIX = np.eye(4)
OVERLAP_RADIUS_M = .12
EDGES = ((16,17),(17,18),(18,19),(16,19))


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def timestamp(header):
    return header.stamp.sec*10**9+header.stamp.nanosec


def geometry(a,b,t):
    transformed=b@t[:3,:3].T+t[:3,3]
    distances=(cKDTree(a).query(transformed)[0],cKDTree(transformed).query(a)[0])
    result=[]
    for d in distances:
        selected=d[d<=OVERLAP_RADIUS_M]
        result.append(dict(overlap=len(selected)/len(d), inliers=len(selected),
                           rmse_m=float(np.sqrt(np.mean(selected**2))) if len(selected) else None))
    return result


def fit_orb(left,right,k,d,a,b):
    images=(left[0],right[0])
    detector=cv2.ORB_create(nfeatures=1200)
    feats=[detector.detectAndCompute(cv2.cvtColor(im,cv2.COLOR_RGB2GRAY),None) for im in images]
    if any(desc is None or len(desc)<2 for _,desc in feats):
        return dict(mutual=0,metric_matches=0),None
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    def ratio(x,y):
        return {m.queryIdx:m.trainIdx for m,n in matcher.knnMatch(x,y,k=2) if m.distance<.75*n.distance}
    matches=ratio(feats[0][1],feats[1][1])
    reverse=ratio(feats[1][1],feats[0][1])
    paired=[(i,j) for i,j in matches.items() if reverse.get(j)==i]
    if not paired:
        return dict(mutual=0,metric_matches=0),None
    points=[]
    for side,image in enumerate((left,right)):
        keypoints=feats[side][0]
        uv=np.array([keypoints[ids[side]].pt for ids in paired],dtype=np.float32)
        xy=uv.astype(int)
        z=image[1][xy[:,1],xy[:,0]]/1000.
        rays=cv2.undistortPoints(uv.reshape(-1,1,2),k,d).reshape(-1,2)
        points.append((np.column_stack((rays,np.ones(len(rays))))*z[:,None],(z>=.3)&(z<=10)))
    good=points[0][1]&points[1][1]
    x,y=(p[0][good] for p in points)
    audit=dict(mutual=len(paired),metric_matches=len(x))
    if len(x)<12:
        return audit,None
    rng=np.random.default_rng(30+a*100000+b)
    best=np.zeros(len(x),dtype=bool)
    for _ in range(256):
        ids=rng.choice(len(x),3,replace=False)
        xx,yy=x[ids],y[ids]
        if min(np.linalg.norm(np.cross(p[1]-p[0],p[2]-p[0])) for p in (xx,yy))<.002:
            continue
        r,t=rigid(yy,xx)
        good=np.linalg.norm(y@r.T+t-x,axis=1)<.10
        if good.sum()>best.sum():
            best=good
    audit['inliers']=int(best.sum())
    if best.sum()<12:
        return audit,None
    r,t=rigid(y[best],x[best])
    matrix=MATRIX.copy()
    matrix[:3,:3],matrix[:3,3]=r,t
    error=np.linalg.norm(y@r.T+t-x,axis=1)
    audit.update(fit_inlier_rmse_m=float(np.sqrt(np.mean(error[best]**2))),
                 fit_fraction=float(best.mean()))
    return audit,matrix


def imu_samples(conn,store,lo,hi):
    topic='robot3/imu/data'
    data=[]
    for mid,ns,payload in conn.execute('SELECT m.id,m.timestamp,m.data FROM messages m JOIN topics t ON '
            'm.topic_id=t.id WHERE t.name=? AND m.timestamp BETWEEN ? AND ? ORDER BY m.timestamp',
            (topic,lo-100_000_000,hi+100_000_000)):
        msg=store.deserialize_cdr(payload,'sensor_msgs/msg/Imu')
        w=msg.angular_velocity
        acc=msg.linear_acceleration
        data.append((mid,ns,timestamp(msg.header),msg.header.frame_id,
                     float(np.linalg.norm([w.x,w.y,w.z])),float(np.linalg.norm([acc.x,acc.y,acc.z]))))
    return data


def imu_metrics(samples,lo,hi):
    x=[r for r in samples if lo<=r[1]<=hi]
    if len(x)<2:
        return dict(imu_count=len(x),imu_status='INSUFFICIENT_SAMPLES')
    t=np.array([r[1]-lo for r in x])/1e9
    w=np.array([r[4] for r in x])
    a=np.array([r[5] for r in x])
    full_t=np.r_[0.,t,(hi-lo)/1e9]
    integrated=np.trapezoid(np.r_[w[0],w,w[-1]],full_t)
    return dict(imu_count=len(x),imu_status='RAW_IMU_NORMS_ONLY',imu_frame=x[0][3],
                imu_first_id=x[0][0],imu_last_id=x[-1][0],imu_gyro_max_deg_s=float(np.degrees(max(w))),
                imu_integrated_gyro_norm_deg=float(np.degrees(integrated)),
                imu_accel_norm_min=float(min(a)),imu_accel_norm_max=float(max(a)),
                imu_max_sampling_gap_ms=float(np.diff(full_t).max()*1000),
                imu_header_to_bag_median_ms=float(np.median([(r[1]-r[2])/1e6 for r in x])))


def gnss_interval(conn,store,lo,hi):
    rows=[]
    topic='robot3/gnss_1/velocity_ecef'
    for mid,ns,payload in conn.execute(
            'SELECT m.id,m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id '
            'WHERE t.name=? AND m.timestamp BETWEEN ? AND ? ORDER BY m.timestamp',
            (topic,lo-150_000_000,hi+150_000_000)):
        msg=store.deserialize_cdr(payload,'geometry_msgs/msg/TwistWithCovarianceStamped')
        v=msg.twist.twist.linear
        rows.append((mid,ns,timestamp(msg.header),float(np.linalg.norm([v.x,v.y,v.z]))))
    if not rows:
        return dict(gnss_velocity_samples=0)
    return dict(gnss_velocity_samples=len(rows),gnss_velocity_first_id=rows[0][0],
                gnss_velocity_last_id=rows[-1][0],gnss_max_speed_m_s=max(r[3] for r in rows),
                gnss_velocity_header_to_bag_median_ms=float(np.median([(r[1]-r[2])/1e6 for r in rows])))


def distinct_pairs(values):
    """Choose at most four time-spanning disjoint pairs from an ordered split."""
    return [values[i] for i in np.unique(np.linspace(0,len(values)-1,min(4,len(values)),dtype=int))] if values else []


def report_edges(poses,display,frame,feature):
    source=BASE/'24d_rgbd_pose_graph'
    graph=load(source/'pose_graph.json')
    direct=next(e for e in graph['edges'] if e['a']==[ROBOT,16] and e['b']==[ROBOT,19] and e['used_in_forest'])
    submaps={p['boundary']:p for p in load(BASE/'24e_rgbd_feasibility'/'dense_submap_diagnostics.json')}
    adjacent=[np.linalg.inv(display[a])@display[b] for a,b in EDGES[:3]]
    chain=adjacent[0]@adjacent[1]@adjacent[2]
    assert np.allclose(chain,np.linalg.inv(display[16])@display[19])
    selected_direct=np.array(direct['T_a_from_b'])
    required = {}
    for slot, edge in enumerate(EDGES[:3]):
        preceding = np.linalg.multi_dot(adjacent[:slot]) if slot==2 else (adjacent[0] if slot==1 else MATRIX)
        following = np.linalg.multi_dot(adjacent[slot+1:]) if slot==0 else (adjacent[2] if slot==1 else MATRIX)
        required[edge] = np.linalg.inv(preceding)@selected_direct@np.linalg.inv(following)
        assert np.allclose(preceding@required[edge]@following,selected_direct)
    results={}
    for (a,b),frozen in zip(EDGES,adjacent+[selected_direct]):
        tag=f'{a}->{b}'
        if (a,b)==(16,19):
            supported=[(r['i'],r['j']) for r in direct['support']]
            trials=[(i+7,j+7) for i,j in supported]
            assert len({i for pair in supported+trials for i in pair})==2*len(supported+trials)
            rows=[dict(frames=pair,split='near_support_unseen_by_frozen_support_ids') for pair in trials]
            for i,j in trials:
                assert int(poses[i]['map_epoch'])==a and int(poses[j]['map_epoch'])==b
            split_for_fit=supported
        else:
            cohort=submaps[f'{ROBOT}:{tag}']
            split_for_fit=distinct_pairs(list(zip(*cohort['fit_frames'])))
            tests=distinct_pairs(list(zip(*cohort['holdout_frames'])))
            for side in (0,1):
                assert {p[side] for p in split_for_fit}.isdisjoint({p[side] for p in tests})
            rows=[dict(frames=pair,split='frozen_disjoint_holdout') for pair in tests]
        estimates=[]
        dense=[]
        for i,j in split_for_fit:
            registration,local=feature(i,j)
            estimate=pose(poses[i])@local@np.linalg.inv(pose(poses[j])) if local is not None else None
            estimates.append(dict(frames=[i,j],fit=registration,
                                  T_a_from_b=estimate.tolist() if estimate is not None else None,
                                  delta_from_frozen=delta(frozen,estimate) if estimate is not None else None))
            pi,pj=pose(poses[i]),pose(poses[j])
            cloud_a,cloud_b=frame(i)[2],frame(j)[2]
            seed=np.linalg.inv(pi)@frozen@pj
            icp=fit_icp(cloud_a,cloud_b,seed) if min(len(cloud_a),len(cloud_b))>=300 else None
            if icp is not None:
                transform=pi@icp@np.linalg.inv(pj)
                dense.append(dict(frames=[i,j],fitted_geometry=geometry(cloud_a,cloud_b,icp),
                                  T_a_from_b=transform.tolist(),delta_from_frozen=delta(frozen,transform)))
            else:
                dense.append(dict(frames=[i,j],T_a_from_b=None,status='NO_ICP_FIT'))
        for entry in rows:
            i,j=entry['frames']
            pi,pj=pose(poses[i]),pose(poses[j])
            entry['timestamps_ns']=[int(poses[k]['rgb_bag_ns']) for k in (i,j)]
            cloud_a,cloud_b=frame(i)[2],frame(j)[2]
            entry['frozen']=geometry(cloud_a,cloud_b,np.linalg.inv(pi)@frozen@pj)
            entry['independent_fit_predictions']=[
                dict(fit_frames=e['frames'],metrics=geometry(cloud_a,cloud_b,np.linalg.inv(pi)@np.array(e['T_a_from_b'])@pj))
                for e in estimates if e['T_a_from_b'] is not None]
            entry['seeded_icp_predictions']=[
                dict(fit_frames=e['frames'],metrics=geometry(cloud_a,cloud_b,np.linalg.inv(pi)@np.array(e['T_a_from_b'])@pj))
                for e in dense if e['T_a_from_b'] is not None]
            if (a,b)==(16,19):
                entry['composed_chain']=geometry(cloud_a,cloud_b,np.linalg.inv(pi)@chain@pj)
            else:
                entry['closure_implied_single_edge']=geometry(cloud_a,cloud_b,np.linalg.inv(pi)@required[a,b]@pj)
        interfit=[delta(np.array(x['T_a_from_b']),np.array(y['T_a_from_b']))
                  for x,y in combinations([x for x in estimates if x['T_a_from_b'] is not None],2)]
        dense_disagree=[delta(np.array(x['T_a_from_b']),np.array(y['T_a_from_b']))
                        for x,y in combinations([x for x in dense if x['T_a_from_b'] is not None],2)]
        results[tag]=dict(T_a_from_b=frozen.tolist(),fit=estimates,heldout=rows,interfit_disagreement=interfit,
                          frozen_seed_icp_fit=dense,frozen_seed_icp_interfit_disagreement=dense_disagree)
        if (a,b) in required:
            results[tag]['closure_implied_T_a_from_b']=required[a,b].tolist()
            results[tag]['closure_implied_delta_from_frozen']=delta(frozen,required[a,b])
    return dict(convention='T_epoch_a_from_epoch_b; camera test = inv(T_epoch_a_from_cam_i) @ T_a_from_b @ T_epoch_b_from_cam_j',
                chain=chain.tolist(),direct=selected_direct.tolist(),closure_disagreement=delta(selected_direct,chain),
                stage24d_support=direct['support'],edges=results)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scratch',type=Path,required=True)
    p.add_argument('--imu-db',type=Path,required=True)
    args=p.parse_args()
    cv2.setNumThreads(1)
    DEST.mkdir(parents=True,exist_ok=True)
    poses=read_csv(OUT/ROBOT/'camera_poses.csv')
    pairs=read_csv(OUT/ROBOT/'sync_pairs.csv')
    coverage=read_csv(OUT/ROBOT/'depth_coverage.csv')
    assert all(a['index']==b['index']==c['index']==str(i) for i,(a,b,c) in enumerate(zip(poses,pairs,coverage,strict=True)))
    chain_info=load(BASE/'29_spatial_audit'/'stage29_epoch_transforms.json')
    display={p['epoch']:np.array(p['T_display_from_epoch']) for p in chain_info if p['robot']==ROBOT}
    stream=load(OUT/'dataset_manifest.json')['robots'][ROBOT]
    imu_zip=RAW/ROBOT/'robot3_main_campus_imu_gps.zip'
    with zipfile.ZipFile(imu_zip) as z:
        item=z.getinfo(f'{imu_zip.stem}/{imu_zip.stem}.db3')
        assert args.imu_db.stat().st_size==item.file_size
        imu_identity=dict(archive=imu_zip.name,sqlite_crc=f'{item.CRC:08x}',sqlite_bytes=item.file_size)
    rcal=stream['RGB']['calibration']
    k=np.array(rcal['K']).reshape(3,3)
    k[:2]*=.5
    d=np.array(rcal['D'])
    ext=stream['DEPTH']['extra']['T_color_from_depth']
    rotation,translation=np.array(ext['R']),np.array(ext['t_m'])
    store=get_typestore(Stores.ROS2_HUMBLE)
    diagnostics={}
    all_steps=[]
    with ExitStack() as stack:
        rgb,dep=[stack.enter_context(closing(open_bag(ROBOT,stream,kind,args.scratch))) for kind in ('RGB','DEPTH')]
        imu=stack.enter_context(closing(sqlite3.connect(f'{args.imu_db.resolve().as_uri()}?mode=ro&immutable=1',uri=True)))
        for db in (rgb,dep):
            for name,encoding,definition in db.execute('SELECT topic_type,encoding,encoded_message_definition FROM message_definitions'):
                if encoding=='ros2msg':
                    store.register(get_types_from_msg(definition,name))
        @lru_cache(maxsize=96)
        def frame(i):
            pair=pairs[i]
            image=rgb_pixels(message(rgb,store,stream['RGB'],pair,'rgb'))
            paired=pair['paired']=='True'
            nearest=(depth_pixels(message(dep,store,stream['DEPTH'],pair,'depth'),rotation,translation,k,d)
                     if pair['depth_message_id'] else None)
            z=nearest if paired else np.zeros((400,640),dtype=np.uint16)
            assert abs((z>0).mean()-float(coverage[i]['valid_depth_fraction']))<1e-12
            yy,xx=np.mgrid[4:400:8,4:640:8]
            zz=z[yy,xx]/1000.
            keep=(zz>=.3)&(zz<=10)
            uv=np.column_stack((xx[keep],yy[keep])).astype(np.float32)
            rays=cv2.undistortPoints(uv.reshape(-1,1,2),k,d).reshape(-1,2) if len(uv) else uv
            cloud=np.column_stack((rays,np.ones(len(rays))))*zz[keep,None]
            diagnostics[i]=dict(index=i,epoch=poses[i]['map_epoch'],tracking_state=poses[i]['tracking_state'],
                                valid_pose=poses[i]['valid_pose'],tracked_points=poses[i]['tracked_points'],
                                map_id=poses[i]['map_id'],map_init_kf=poses[i]['map_init_kf'],
                                map_vote_fraction=poses[i]['map_vote_fraction'],depth_valid_fraction=float((z>0).mean()),
                                rgb_topic=stream['RGB']['topic'],depth_topic=stream['DEPTH']['topic'],
                                nearest_raw_depth_fraction=float((nearest>0).mean()) if nearest is not None else None,
                                nearest_raw_depth_sha256=hashlib.sha256(nearest.astype('<u2').tobytes()).hexdigest()
                                if nearest is not None else None,
                                depth_used_for_geometry=paired,
                                rgb_pixel_sha256=hashlib.sha256(image.tobytes()).hexdigest(),
                                depth_pixel_sha256=hashlib.sha256(z.astype('<u2').tobytes()).hexdigest(),
                                **{k:pair[k] for k in ('rgb_message_id','rgb_bag_ns','rgb_header_ns','depth_message_id',
                                                     'depth_bag_ns','depth_header_ns','bag_sync_ms','header_sync_ms','paired')})
            return image,z,cloud

        @lru_cache(maxsize=128)
        def feature(i,j):
            return fit_orb(frame(i),frame(j),k,d,i,j)

        jumps=[]
        valid=[i for i,r in enumerate(poses) if r['valid_pose']=='1']
        for i,j in zip(valid,valid[1:]):
            a,b=poses[i],poses[j]
            if a['map_epoch']!=b['map_epoch']:
                continue
            t=np.linalg.inv(pose(a))@pose(b)
            m,angle=delta(MATRIX,t)
            lo,hi=int(a['rgb_bag_ns']),int(b['rgb_bag_ns'])
            dt=(hi-lo)/1e9
            epoch=int(a['map_epoch'])
            suspicious=m>=1 or angle>=35
            all_steps.append(dict(epoch=epoch,i=i,j=j,t_s=int(b['relative_ns'])/1e9,m=m,deg=angle,suspicious=suspicious))
            if not suspicious:
                continue
            assert dt>0
            f,g=frame(i),frame(j)
            registration,estimate=feature(i,j)
            if len(f[2]) and len(g[2]):
                registration.update(frozen=geometry(f[2],g[2],t),identity=geometry(f[2],g[2],MATRIX),
                                    fitted=geometry(f[2],g[2],estimate) if estimate is not None else None)
            if estimate is not None:
                registration['observed_rgbd_motion']=delta(MATRIX,estimate)
            comparisons=[]
            for u,v in ((i-1,i),(j,j+1)):
                if not (0<=u<v<len(poses)) or any(
                        poses[n]['valid_pose']!='1' or poses[n]['map_epoch']!=a['map_epoch'] for n in (u,v)):
                    continue
                f0,f1=frame(u)[2],frame(v)[2]
                adjacent_pose=np.linalg.inv(pose(poses[u]))@pose(poses[v])
                comparisons.append(dict(frames=[u,v],pose_step=delta(MATRIX,adjacent_pose),
                    frozen_geometry=geometry(f0,f1,adjacent_pose) if len(f0) and len(f1) else None))
            registration['adjacent_control']=comparisons
            row=dict(epoch=epoch,i=i,j=j,rgb_bag_ns_a=lo,rgb_bag_ns_b=hi,
                     rgb_header_ns_a=a['rgb_header_ns'],rgb_header_ns_b=b['rgb_header_ns'],dt_s=dt,
                     translation_m=m,rotation_deg=angle,linear_speed_m_s=m/dt,angular_speed_deg_s=angle/dt,
                     intervening_invalid_frames=j-i-1,map_id=a['map_id'],map_init_kf=a['map_init_kf'],
                     local_state_context=';'.join(f"{k}:{poses[k]['tracking_state']}/{poses[k]['valid_pose']}/{poses[k]['map_id']}/{poses[k]['map_init_kf']}"
                                                  for k in range(max(0,i-2),min(len(poses),j+3))),
                     registration=json.dumps(registration,separators=(',',':')))
            row.update(imu_metrics(imu_samples(imu,store,lo,hi),lo,hi))
            row.update(gnss_interval(imu,store,lo,hi))
            jumps.append(row)
        edges=report_edges(poses,display,frame,feature)
        edges['imu_source']=imu_identity
        edges['protocol']=dict(overlap_radius_m=OVERLAP_RADIUS_M, seed_icp='frozen pose seed; separately fit frame pairs, disjoint validation IDs',
                               near_support='direct 16->19 holdout uses original support frame indices plus seven on each side; shared nearby viewpoints')
        # Use exact original decoded RGB/depth for the two representative maxima.
        representatives=[]
        for epoch in (6,9):
            event=max([r for r in jumps if r['epoch']==epoch],key=lambda r:r['translation_m'])
            a,b=event['i'],event['j']
            representatives.append(dict(epoch=epoch,ids=[a,b],rgb=[frame(a)[0],frame(b)[0]],
                                        depth=[frame(a)[1],frame(b)[1]]))
        fields=list(dict.fromkeys(k for row in jumps for k in row))
        with (DEST/'stage30_pose_jump_audit.csv').open('w',encoding='utf-8',newline='') as out:
            import csv
            writer=csv.DictWriter(out,fields,lineterminator='\n')
            writer.writeheader()
            writer.writerows(jumps)
        write_csv(DEST/'stage30_rgbd_frame_diagnostics.csv',list(diagnostics.values()))
        dump(DEST/'stage30_epoch_transform_consistency.json',edges)
        from stage30_figures import draw
        draw(DEST,all_steps,jumps,representatives,edges)
    print('STAGE30 COMPLETE',len(jumps),'suspect transitions',len(diagnostics),'original RGB-D frames',flush=True)


if __name__=='__main__':
    main()
