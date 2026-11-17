"""Audit native RGB-D geometry and select evidence-backed display placements."""
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from common import OUT, SITE, dump
from fusion import pose
from stage26_manifest import read_csv, write_csv
from stage27_continuous import delta, time_ns
from stage28_gravity import points
from stage28_lidar import BASE, ROBOTS, lookup

SOURCE = SITE/'continuous'
DEST = BASE/'29_spatial_audit'
GRAVITY = BASE/'28_multisensor'/'gravity_alignment.json'


def bounds(array):
    return [array.min(axis=0).tolist(),array.max(axis=0).tolist()] if len(array) else None


def vector_range(array):
    if len(array)==0:
        return []
    return [float(np.median(array)),float(np.quantile(array,.95)),float(np.max(array))]


def encode(value):
    return json.dumps(value,separators=(',',':'))


def fixed_bounds(loaded):
    # A shared center and vertical field size per sensor. No scene geometry moves.
    result={}
    for key,(root,m,rot) in loaded.items():
        full=defaultdict(list)
        for c in m['chunks']:
            xyz,_=points(root/c['file'])
            full[c['robot']].append(xyz@rot.T)
        boxes={r:bounds(np.concatenate(v)) for r,v in full.items()}
        a=np.array(list(boxes.values()))
        low,high=a[:,0].min(axis=0),a[:,1].max(axis=0)
        result[key]=dict(boxes=boxes,center=((low+high)/2).tolist(),
                         vertical_span=float(np.linalg.norm(high-low)*1.09),
                         robot_bbox_center_delta=(a[1].mean(axis=0)-a[0].mean(axis=0)).tolist(),
                         T_ground_from_display=rot.tolist())
    return result


def per_epoch_geometry(xyz, rows, matrix, window_normals, prior):
    inv=np.linalg.inv(matrix)
    local=xyz@inv[:3,:3].T+inv[:3,3]
    native=np.array([pose(row) for row in rows])
    dt=np.diff([time_ns(row)/1e9 for row in rows])
    steps=np.linalg.norm(np.diff(native[:,:3,3],axis=0),axis=1)
    angles=np.degrees((Rotation.from_matrix(native[:-1,:3,:3]).inv()*
                       Rotation.from_matrix(native[1:,:3,:3])).magnitude()) if len(native)>1 else []
    speed=steps[dt<.35]/dt[dt<.35] if len(steps) else []
    ground=dict(windows=len(window_normals),mode='INSUFFICIENT_LOCAL_GROUND_SUPPORT')
    if len(window_normals):
        normal=np.array([w['normal'] for w in window_normals])
        median=np.median(normal,axis=0)
        median/=np.linalg.norm(median)
        angular=np.degrees(np.arccos(np.clip(normal@median,-1,1)))
        if len(normal)>=8:
            half=[np.median(a,axis=0) for a in np.array_split(normal,2)]
            early_late=float(np.degrees(np.arccos(np.clip(
                half[0]@half[1]/np.linalg.norm(half[0])/np.linalg.norm(half[1]),-1,1))))
        else:
            early_late=None
        angular_p90=float(np.quantile(angular,.9))
        tilt=float(np.degrees(np.arccos(np.clip(median[2],-1,1))))
        trusted=early_late is not None and early_late<=5 and angular_p90<=9 and tilt<=35
        native_normal=matrix[:3,:3].T@prior.T@median
        native_up=matrix[:3,:3].T@prior.T@np.array([0,0,1])
        correction,_=Rotation.align_vectors([native_up],[native_normal])
        local_correction=np.eye(4)
        local_correction[:3,:3]=correction.as_matrix()
        # Local inspection uses a reparameterization; recomposition is explicitly invariant.
        assert np.allclose((matrix@np.linalg.inv(local_correction))@local_correction,matrix)
        ground.update(p90_deg=angular_p90,early_late_deg=early_late,tilt_deg=tilt,
                      rmse_m=vector_range(np.array([w['residual_m'] for w in window_normals])),
                      mode='PASS_LOCAL_ONLY' if trusted else 'REJECT_LOCAL',
                      T_leveled_local_from_epoch=local_correction.tolist() if trusted else None,
                      T_display_from_leveled_local=(matrix@np.linalg.inv(local_correction)).tolist() if trusted else None)
    return dict(local_bbox=bounds(local),display_bbox=bounds(xyz),native_pose_bounds=bounds(native[:,:3,3]),
                spatial_step_m_quantiles=vector_range(steps),rotation_step_deg_quantiles=vector_range(np.asarray(angles)),
                consecutive_speed_mps_quantiles=vector_range(np.asarray(speed)),
                trajectory_length_m=float(sum(steps)),ground=ground,
                recovered_point_count=len(local))


def main():
    DEST.mkdir(parents=True,exist_ok=True)
    manifest=lookup(SOURCE/'manifest.json')
    trajectories=lookup(SOURCE/manifest['trajectory'])
    frozen=lookup(BASE/'24d_rgbd_pose_graph'/'pose_graph.json')
    gravity=lookup(GRAVITY)
    old={(n['robot'],n['original_epoch']):n for n in frozen['nodes']}
    proposals=read_csv(BASE/'26_rgbd_reconstruction'/'registration_comparison.csv')
    registrations=defaultdict(list)
    for p in proposals:
        registrations[p['boundary']].append(p)
    provenance=read_csv(BASE/'24e_rgbd_feasibility'/'epoch_boundary_audit.csv')
    audits={(r['robot'],int(r['next_epoch'])):r for r in provenance}
    boundaries={(r['robot'],r['epoch']):r for r in manifest['boundaries']}
    frames=defaultdict(list)
    for robot in ROBOTS:
        for row in read_csv(OUT/robot/'camera_poses.csv'):
            if row['valid_pose']=='1':
                frames[robot,int(row['map_epoch'])].append(row)
    clouds=defaultdict(list)
    for c in manifest['chunks']:
        xyz,_=points(SOURCE/c['file'])
        clouds[c['robot'],c['epoch']].append(xyz)
    prior=np.array(gravity['rgbd']['combined']['T_ground_from_display'])[:3,:3]
    table, placements, directions=[],[],[]
    for epoch in manifest['transforms']:
        robot,i=epoch['robot'],epoch['epoch']
        key=robot,i
        x=np.array(epoch['T_display_from_epoch'])
        rows=frames[key]
        xyz=np.concatenate(clouds[key]) if clouds[key] else np.empty((0,3))
        evidence=audits.get(key,{})
        ev=boundaries.get(key,{})
        stage26=registrations[evidence.get('boundary','')]
        known=[{k:r[k] for k in ('method','overlap_a','overlap_b','verified_repetitions','bridge_decision','graph_admitted')}
               for r in stage26]
        windows=[r for r in gravity['rgbd'][robot]['windows'] if r['epoch']==i]
        geo=per_epoch_geometry(xyz,rows,x,windows,prior)
        display=[row for row in trajectories[robot] if row[2]==i]
        assert [row[1] for row in display]==[int(row['index']) for row in rows]
        replay_error=max(np.linalg.norm(np.array(row[3:6])-(x@pose(native))[:3,3]) for row,native in zip(display,rows))
        assert replay_error<1e-8
        metric=dict()
        if ev:
            previous=frames[robot,ev['previous_epoch']][-1]
            prev_display=next(e for e in manifest['transforms'] if (e['robot'],e['epoch'])==(robot,ev['previous_epoch']))
            m,d=delta(np.array(prev_display['T_display_from_epoch'])@pose(previous),x@pose(rows[0]))
            assert abs(m-ev['translation_m'])<1e-8 and abs(d-ev['rotation_deg'])<1e-8
            metric=dict(prior_epoch=ev['previous_epoch'],transition_translation_m=m,transition_rotation_deg=d,
                        native_reset=evidence.get('classification'),map_id_change=evidence.get('map_id_changed'),
                        initial_kf_change=evidence.get('init_kf_changed'),untracked_interval_ns=evidence.get('untracked_interval_ns'),
                        prediction_residual_m=ev['prediction_residual_m'],prediction_residual_deg=ev['prediction_residual_deg'])
        old_gauge=old[key]['component']
        # Do not promote aggregate-only evidence. Stage26 admitted zero extra constraints.
        placement=dict(robot=robot,epoch=i,source=epoch['source'],confidence=epoch['uncertainty'],
                       stage24d_component=old_gauge,T_display_from_epoch=x.tolist(),decision='RETAIN_STAGE27',
                       reason='No independently admitted replacement for this boundary',local_gravity=geo['ground'])
        placements.append(placement)
        table.append(dict(robot=robot,epoch=i,first_frame=rows[0]['index'],last_frame=rows[-1]['index'],
                          duration_s=(time_ns(rows[-1])-time_ns(rows[0]))/1e9,valid_frames=len(rows),
                          source=epoch['source'],confidence=epoch['uncertainty'],stage24d_component=old_gauge,
                          transition=encode(metric),stage26=encode(known),geometry=encode(geo),
                          T_display_from_epoch=encode(x.tolist()),recomposition_error_m=float(replay_error),
                          convention='T_display_from_epoch @ T_epoch_from_color_optical'))
    for edge in frozen['edges']:
        if not edge['used_in_forest'] or not edge['graph_consistent']:
            continue
        a,b=tuple(edge['a']),tuple(edge['b'])
        predicted=np.linalg.inv(np.array(old[a]['T_atlas_from_epoch']))@np.array(old[b]['T_atlas_from_epoch'])
        tf=np.array(edge['T_a_from_b'])
        directed=delta(tf,predicted)
        inverted=delta(np.linalg.inv(tf),predicted)
        assert directed[0]<1e-7 and directed[1]<1e-7
        directions.append(dict(a=a,b=b,direct_error=directed,inverse_error=inverted))
    old16=old['robot3',16]
    old19=old['robot3',19]
    assert old16['component']==old19['component']
    chains={e['epoch']:np.array(e['T_display_from_epoch']) for e in placements if e['robot']=='robot3'}
    reentry=chains[16]@np.linalg.inv(np.array(old16['T_atlas_from_epoch']))@np.array(old19['T_atlas_from_epoch'])
    reentry_gap=delta(chains[18]@pose(frames['robot3',18][-1]),reentry@pose(frames['robot3',19][0]))
    fallback_gap=delta(chains[18]@pose(frames['robot3',18][-1]),chains[19]@pose(frames['robot3',19][0]))
    assert reentry_gap[0]>38 and fallback_gap[0]<.2
    write_csv(DEST/'stage29_epoch_spatial_audit.csv',table)
    dump(DEST/'stage29_epoch_transforms.json',placements)
    dump(DEST/'stage29_directions.json',dict(verified_forest_checks=directions,robot3_18_19_stage24d_reentry=reentry_gap,
                                            robot3_18_19_stage27_fallback=fallback_gap))
    rgbd=SOURCE
    lidar=SITE.parent/'stage28_multisensor'/'lidar'
    metric=fixed_bounds({key:(root,lookup(root/'manifest.json'),
                               np.array(gravity[key]['combined']['T_ground_from_display'])[:3,:3])
                         for key,root in [('lidar',lidar),('rgbd',rgbd)]})
    dump(DEST/'metric_view.json',metric)
    dump(DEST/'map_reference_manifest.json',dict(
        points_rebuilt=False,stage27_poses_changed=0,
        assets=gravity['rgbd'],reason='No validated relative bridge replacement; shared metric camera correction only'))
    print('STAGE29 AUDIT COMPLETE',len(table),reentry_gap,fallback_gap,flush=True)


if __name__=='__main__':
    main()
