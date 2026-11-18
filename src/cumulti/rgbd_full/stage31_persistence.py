"""Stage31 bounded, frame-disjoint Epoch9 RGB-D segment persistence study."""
import argparse
import csv
import json
from collections import defaultdict
from contextlib import ExitStack, closing
from functools import lru_cache
from itertools import product

import cv2
import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from common import OUT, dump, sha
from fusion import pose
from stage26_manifest import read_csv, write_csv
from stage26_register import fit_icp, global_seed
from stage27_continuous import depth_pixels, message, open_bag, rgb_pixels, delta
from stage30_pose_forensics import fit_orb

DEST=OUT.parent/'31_epoch9_persistence'
EVENTS=(9936,10071,12057)
POLICY=dict(epoch=9,events=EVENTS,pre_valid_frames=30,post_valid_frames=31,
            fit_pre_offsets=[-30,-26,-21],fit_post_offsets=[10,14,19],
            validation_pre_offsets=[-17,-10,-3],validation_post_offsets=[0,3,6,23,26,30],
            temporal_exclusion_ns=300_000_000,min_distinct_frames=3,min_span_ns=800_000_000,
            validation_distance_m=.12,min_inliers=300,min_overlap=.35,min_spread_ratio=.05,
            max_agreement_m=.2,max_agreement_deg=3,
            visual_min_mutual=60,visual_min_metric=40,visual_min_inliers=30,
            visual_min_fraction=.5,visual_max_rmse_m=.1,
            candidate='fit-only medoid cluster, coordinate-wise median translation and SO3 mean rotation',
            seed=26,methods=['orb_metric','icp_identity','fpfh_ransac_icp'],
            fit_pixels_offset=0,validation_pixels_offset=4,pixel_stride=8,
            frozen_poses_used_to_initialize_registration=False,
            frame_cohorts_frozen_before_registration=True,
            correction_scope='candidate only; no original input written')


def matrix_average(transforms):
    x=np.eye(4)
    x[:3,3]=np.median([t[:3,3] for t in transforms],axis=0)
    x[:3,:3]=Rotation.from_matrix(np.array([t[:3,:3] for t in transforms])).mean().as_matrix()
    return x


def evaluate(a,b,t):
    if min(len(a),len(b))<3:
        return dict(pass_geometry=False,inliers=[0,0],overlap=[0.,0.],rmse_m=[None,None],spread=[0.,0.])
    placed=b@t[:3,:3].T+t[:3,3]
    distances=(cKDTree(a).query(placed)[0],cKDTree(placed).query(a)[0])
    masks=[v<=.12 for v in distances]
    spread=[]
    for cloud,keep in zip((placed,a),masks):
        points=cloud[keep]
        s=np.linalg.svd(points-points.mean(0),compute_uv=False) if len(points)>=3 else [0.,0.]
        spread.append(float(s[1]/s[0]) if s[0]>0 else 0.)
    return dict(inliers=[int(m.sum()) for m in masks],overlap=[float(m.mean()) for m in masks],
                rmse_m=[float(np.sqrt(np.mean(v[m]**2))) if m.any() else None for v,m in zip(distances,masks)],
                spread=spread,pass_geometry=all(m.sum()>=300 and m.mean()>=.35 for m in masks) and min(spread)>=.05)


def support(records, poses):
    if len(records)<3:
        return False
    for side in ('i','j'):
        ids={r[side] for r in records}
        if len(ids)<3:
            return False
        ns=[int(poses[i]['rgb_bag_ns']) for i in ids]
        if max(ns)-min(ns)<800_000_000:
            return False
    return True


def build_plan(poses,pairs):
    valid=[i for i,r in enumerate(poses) if r['valid_pose']=='1' and int(r['map_epoch'])==9]
    plan,windows=[],[]
    for event in EVENTS:
        slot=valid.index(event)
        assert valid[slot-1]==event-1
        def frame(offset):
            return valid[slot+offset]
        fit=list(product([frame(i) for i in POLICY['fit_pre_offsets']],
                         [frame(i) for i in POLICY['fit_post_offsets']]))
        held=list(product([frame(i) for i in POLICY['validation_pre_offsets']],
                          [frame(i) for i in POLICY['validation_post_offsets']]))
        fit_ids={i for pair in fit for i in pair}
        val_ids={i for pair in held for i in pair}
        assert fit_ids.isdisjoint(val_ids)
        minimum=min(abs(int(poses[i]['rgb_bag_ns'])-int(poses[j]['rgb_bag_ns'])) for i in fit_ids for j in val_ids)
        assert minimum>=POLICY['temporal_exclusion_ns']
        plan.extend(dict(event=event,cohort=cohort,i=i,j=j,post_elapsed_s=(int(poses[j]['rgb_bag_ns'])-int(poses[event]['rgb_bag_ns']))/1e9)
                    for cohort,chosen in (('fit',fit),('validation',held)) for i,j in chosen)
        # Disjoint within-segment controls at several temporal separations.
        controls=[(frame(-17),frame(-10)),(frame(-17),frame(-3)),(frame(0),frame(6)),
                  (frame(0),frame(23)),(frame(23),frame(30)),(frame(0),frame(30))]
        plan.extend(dict(event=event,cohort='control',i=i,j=j,
                         post_elapsed_s=(int(poses[j]['rgb_bag_ns'])-int(poses[event]['rgb_bag_ns']))/1e9)
                    for i,j in controls)
        for i in range(frame(-30),frame(30)+1):
            p=poses[i]
            split='fit' if i in fit_ids else 'validation' if i in val_ids else 'buffer_or_context'
            windows.append(dict(event=event,index=i,cohort=split,min_fit_validation_gap_ns=minimum,
                           **{k:p[k] for k in ('relative_ns','rgb_bag_ns','rgb_header_ns','valid_pose','tracking_state',
                                                'tracked_points','map_id','map_init_kf','map_vote_fraction')},
                           **{k:pairs[i][k] for k in ('depth_message_id','depth_bag_ns','depth_header_ns',
                                                     'rgb_message_id','paired','bag_sync_ms','header_sync_ms')}))
    return plan,windows


def run(args):
    cv2.setNumThreads(1)
    DEST.mkdir(parents=True,exist_ok=True)
    poses=read_csv(OUT/'robot3/camera_poses.csv')
    pairs=read_csv(OUT/'robot3/sync_pairs.csv')
    coverage=read_csv(OUT/'robot3/depth_coverage.csv')
    watched=[OUT/'robot3/camera_poses.csv',OUT/'robot3/sync_pairs.csv',
             OUT.parent/'24d_rgbd_pose_graph/pose_graph.json',
             OUT.parent/'27_continuous_rgbd/chunk_manifest.json',
             OUT.parent/'29_spatial_audit/stage29_epoch_transforms.json']
    before={str(p):sha(p) for p in watched}
    plan,windows=build_plan(poses,pairs)
    policy=json.loads(json.dumps(POLICY))
    if (DEST/'predeclared_policy.json').exists():
        assert json.loads((DEST/'predeclared_policy.json').read_text())==policy
    else:
        dump(DEST/'predeclared_policy.json',policy)
    write_csv(DEST/'stage31_epoch9_event_windows.csv',windows)
    write_csv(DEST/'frozen_pair_plan.csv',plan)
    cfg=json.loads((OUT/'dataset_manifest.json').read_text())['robots']['robot3']
    k=np.array(cfg['RGB']['calibration']['K']).reshape(3,3)
    k[:2]*=.5
    d=np.array(cfg['RGB']['calibration']['D'])
    extr=cfg['DEPTH']['extra']['T_color_from_depth']
    r,t=np.array(extr['R']),np.array(extr['t_m'])
    store=get_typestore(Stores.ROS2_HUMBLE)
    from stage31_flow import POLICY as FLOW_POLICY, estimate as flow_estimate
    dump(DEST/'supplemental_feature_policy.json',FLOW_POLICY)
    decoded={}
    outcomes=[]
    with ExitStack() as stack:
        rgb,depth=[stack.enter_context(closing(open_bag('robot3',cfg,kind,args.scratch))) for kind in ('RGB','DEPTH')]
        for db in (rgb,depth):
            for name,encoding,definition in db.execute('SELECT topic_type,encoding,encoded_message_definition FROM message_definitions'):
                if encoding=='ros2msg':
                    store.register(get_types_from_msg(definition,name))
        @lru_cache(maxsize=220)
        def frame(i):
            pair=pairs[i]
            image=rgb_pixels(message(rgb,store,cfg['RGB'],pair,'rgb'))
            z=depth_pixels(message(depth,store,cfg['DEPTH'],pair,'depth'),r,t,k,d) if pair['paired']=='True' else np.zeros((400,640),np.uint16)
            assert abs((z>0).mean()-float(coverage[i]['valid_depth_fraction']))<1e-12
            decoded[i]=dict(index=i,depth_paired=pair['paired'],depth_valid_fraction=float((z>0).mean()))
            return image,z
        @lru_cache(maxsize=440)
        def cloud(i,offset):
            _,z=frame(i)
            yy,xx=np.mgrid[offset:400:8,offset:640:8]
            values=z[yy,xx]/1000.
            keep=(values>=.3)&(values<=10)
            uv=np.column_stack((xx[keep],yy[keep])).astype(np.float32)
            if len(uv)==0:
                return np.empty((0,3))
            xy=cv2.undistortPoints(uv.reshape(-1,1,2),k,d).reshape(-1,2)
            return np.column_stack((xy,np.ones(len(xy))))*values[keep,None]

        for i in sorted({row['index'] for row in windows}):
            frame(i)
        for case in plan:
            i,j=case['i'],case['j']
            pi,pj=pose(poses[i]),pose(poses[j])
            predicted=np.linalg.inv(pi)@pj
            a,b=cloud(i,0),cloud(j,0)
            va,vb=cloud(i,4),cloud(j,4)
            frozen=evaluate(va,vb,predicted)
            feature,visual=fit_orb(frame(i),frame(j),k,d,i,j)
            visual_good=(feature.get('mutual',0)>=60 and feature.get('metric_matches',0)>=40 and
                         feature.get('inliers',0)>=30 and feature.get('fit_fraction',0)>=.5 and
                         feature.get('fit_inlier_rmse_m',999)<=.1)
            flow_feature,flow=flow_estimate(frame(i),frame(j),k,d,i,j)
            for method in [*POLICY['methods'],'rgbd_flow_ransac']:
                visual_record=feature if method=='orb_metric' else flow_feature
                if method=='rgbd_flow_ransac':
                    estimate=flow
                    fit_pass=(visual_record.get('mutual',0)>=60 and visual_record.get('metric_matches',0)>=40 and
                              visual_record.get('inliers',0)>=30 and visual_record.get('fit_fraction',0)>=.5 and
                              visual_record.get('fit_inlier_rmse_m',999)<=.1)
                elif method=='orb_metric':
                    estimate=visual
                    fit_pass=visual_good
                else:
                    seed=np.eye(4) if method=='icp_identity' else (
                        global_seed(a,b) if min(len(a),len(b))>=300 else None)
                    estimate=fit_icp(a,b,seed) if seed is not None and min(len(a),len(b))>=300 else None
                    fit_pass=estimate is not None
                measured=evaluate(va,vb,estimate) if estimate is not None else None
                good=fit_pass and measured['pass_geometry']
                correction=pi@estimate@np.linalg.inv(pj) if estimate is not None else None
                difference=delta(predicted,estimate) if estimate is not None else None
                outcomes.append(dict(**case,method=method,accepted=bool(good),
                                     rejection='PASS_DIAGNOSTIC_ESTIMATE' if good else
                                     'NO_TRANSFORM' if estimate is None else
                                     'VISUAL_CORRESPONDENCE_GATE_FAIL' if not fit_pass else 'DEPTH_OR_SPATIAL_GATE_FAIL',
                                     independent_metrics=measured,frozen_metrics=frozen,
                                     T_cam_i_from_cam_j=estimate.tolist() if estimate is not None else None,
                                     C_epoch_left_on_post=correction.tolist() if correction is not None else None,
                                     discrepancy_m=difference[0] if difference else None,
                                     discrepancy_deg=difference[1] if difference else None,
                                     visual_correspondence_counts=visual_record if method in ('orb_metric','rgbd_flow_ransac') else None))
            print(case['event'],case['cohort'],i,j,flush=True)
    flat=[]
    for row in outcomes:
        flat.append({k:json.dumps(v,separators=(',',':')) if isinstance(v,(list,dict)) else v for k,v in row.items()})
    write_csv(DEST/'stage31_pairwise_motion_estimates.csv',flat)
    dump(DEST/'pairwise_motion_details.json',outcomes)
    dump(DEST/'original_asset_decode_audit.json',dict(decoded=list(decoded.values()),input_sha256=before))
    import platform
    import scipy
    import open3d
    dump(DEST/'run_provenance.json',dict(
        python=platform.python_version(),opencv=cv2.__version__,numpy=np.__version__,
        scipy=scipy.__version__,open3d=open3d.__version__,source_sha256={
            filename:sha(__import__('pathlib').Path(__file__).with_name(filename)) for filename in
            ('stage31_persistence.py','stage31_analyze.py','stage31_flow.py','stage26_register.py','stage30_pose_forensics.py')}))
    from stage31_analyze import analyze
    analyze(DEST,outcomes,poses,support,matrix_average,evaluate,cloud)
    assert before=={str(p):sha(p) for p in watched}
    dump(DEST/'preservation_audit.json',dict(status='PASS',before=before,
         after={str(p):sha(p) for p in watched},new_map_generated=False,original_pose_written=False))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--scratch',type=__import__('pathlib').Path,required=True)
    run(p.parse_args())
