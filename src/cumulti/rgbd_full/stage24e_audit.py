"""Read-only Stage24E boundary and hybrid feasibility inventories."""
import json
import re
import subprocess
import zipfile
from collections import defaultdict
from pathlib import Path

import numpy as np
from common import OUT, RAW, ROOT, dump, sha, stats, writecsv
from stage24d_retrieve import ROBOTS, read

DEST = OUT.parent/'24e_rgbd_feasibility'
DATA = Path('/home/cas/CU-Multi')


def read_native_events(path, count):
    markers = []
    logs = path.read_text().splitlines()
    processed = 0
    pending = []
    for line_number,line in enumerate(logs,1):
        progress = re.search(r'PROGRESS frames=(\d+)',line)
        if progress:
            current = int(progress.group(1))
            markers.extend(dict(line=n,start_frame=processed,end_frame=current,
                                message=message) for n,message in pending)
            processed,pending = current,[]
        elif any(token in line for token in ('Active map reset recieved','Merge detected','Merge finished!',
                                               'Creation of new map with id:', 'First KF:', 'mnInitialFrameId =')):
            pending.append((line_number,line.strip()))
    markers.extend(dict(line=n,start_frame=processed,end_frame=count,message=message)
                   for n,message in pending)
    return markers


def boundary_inventory():
    boundaries,candidates,windows,events,initial = [],[],[],{},[]
    for robot in ROBOTS:
        folder = OUT/robot
        poses,pairs = read(folder/'camera_poses.csv'),read(folder/'sync_pairs.csv')
        depths = read(folder/'depth_coverage.csv')
        epochs = defaultdict(list)
        for i,(pose,pair,depth) in enumerate(zip(poses,pairs,depths,strict=True)):
            assert i==int(pose['index'])==int(pair['index'])==int(depth['index'])
            if pose['valid_pose']=='1':
                epochs[int(pose['map_epoch'])].append(i)
        native = read_native_events(folder/'tracking_stdout.log',len(poses))
        events[robot] = native
        first_valid = next(iter(epochs.values()))[0]
        initial.append(dict(robot=robot,first_valid_frame=first_valid,
                            timestamp_ns=poses[first_valid]['rgb_bag_ns']))
        for ea,eb in zip(epochs,list(epochs)[1:]):
            a,b = epochs[ea][-1],epochs[eb][0]
            pa,pb = poses[a],poses[b]
            ta,tb = (int(p['rgb_bag_ns']) for p in (pa,pb))
            loss = poses[a+1:b]
            bracket = [e for e in native if e['start_frame']<=b and e['end_frame']>=a]
            reset = [e['line'] for e in bracket if 'Active map reset' in e['message']]
            merge = [e['line'] for e in bracket if 'Merge ' in e['message']]
            reset_frames = [int(e['message'].split('=')[-1]) for e in bracket
                            if 'mnInitialFrameId =' in e['message']]
            id_change = pa['map_id']!=pb['map_id']
            init_change = pa['map_init_kf']!=pb['map_init_kf']
            fingerprint = ('POSSIBLE_BOOKKEEPING_ONLY' if not id_change and init_change and not reset and not merge
                           else 'SAME_MAP_ID_NEAR_NATIVE_EVENT' if not id_change else 'MAP_ID_CHANGED')
            key = f'{robot}:{ea}->{eb}'
            # Keep adjacent samples, then several spaced frames for the unchanged support span.
            def window(indices, anchor, backward):
                order = list(reversed(indices)) if backward else indices
                nearby = [i for i in order if abs(int(poses[i]['rgb_bag_ns'])-anchor)<=3_000_000_000]
                selected = nearby[:4]
                last = int(poses[selected[-1]]['rgb_bag_ns'])
                for i in nearby[4:]:
                    t = int(poses[i]['rgb_bag_ns'])
                    if abs(t-last)>=450_000_000:
                        selected.append(i)
                        last=t
                return nearby,selected
            left,la = window(epochs[ea],ta,True)
            right,lb = window(epochs[eb],tb,False)
            def paired(index):
                return pairs[index]['paired']=='True' and float(depths[index]['valid_depth_fraction'])>0
            good_a,good_b = list(filter(paired,la)),list(filter(paired,lb))
            for i in good_a:
                for j in good_b:
                    candidates.append(dict(boundary=key,robot_a=robot,robot_b=robot,index_a=i,index_b=j,
                                           arrival_ns=int(poses[j]['relative_ns'])+25_000_000))
            boundaries.append(dict(boundary=key,robot=robot,previous_epoch=ea,next_epoch=eb,
                previous_last_valid=a,next_first_valid=b,previous_bag_ns=ta,next_bag_ns=tb,
                gap_ns=tb-ta,previous_map_id=pa['map_id'],next_map_id=pb['map_id'],map_id_changed=id_change,
                previous_init_kf=pa['map_init_kf'],next_init_kf=pb['map_init_kf'],init_kf_changed=init_change,
                native_reset_bracket_lines=' '.join(map(str,reset)),native_merge_bracket_lines=' '.join(map(str,merge)),
                native_reset_context_frame_ids=' '.join(map(str,reset_frames)),
                native_log_resolution='progress-bracket only; not exact attribution',
                invalid_frames_between=len(loss),untracked_interval_ns=tb-int(loss[0]['rgb_bag_ns']) if loss else 0,
                previous_depth=paired(a),next_depth=paired(b),
                previous_window_depth_frames=sum(map(paired,left)),next_window_depth_frames=sum(map(paired,right)),
                previous_window_frames=len(left),next_window_frames=len(right),
                previous_epoch_frames=len(epochs[ea]),next_epoch_frames=len(epochs[eb]),
                pair_candidates=len(good_a)*len(good_b),classification=fingerprint))
            windows.append(dict(boundary=key,left=left,right=right,selected_left=good_a,selected_right=good_b))
    assert len(boundaries)==24
    writecsv(DEST/'epoch_boundary_audit.csv',boundaries)
    writecsv(DEST/'boundary_candidates.csv',candidates)
    dump(DEST/'boundary_windows.json',windows)
    dump(DEST/'native_log_evidence.json',dict(initializations=initial,events=events,
         actual_transitions=len(boundaries),reason='26 epochs across two independent recordings gives 24 transitions'))


def hybrid_inventory():
    stage = OUT.parent/'12d_clean_backend'
    trajectory = stage/'clean_all75_gtsam_trajectory.csv'
    mount = json.loads((OUT.parent/'24a_rgb_semantic/calibration_audit.json').read_text())
    archive = DATA/'raw/calib/robot_description.zip'
    overlap,calibration = [],{}
    for robot in ROBOTS:
        index = DATA/f'processed_v1/full_2hz/{robot}/keyframes.csv'
        ids = {int(row['keyframe_id']):int(row['lidar_timestamp_ns']) for row in read(index)}
        rows = [r for r in read(trajectory) if r['robot_id']==robot]
        lidar = np.array([int(r['timestamp']) for r in rows],np.int64)
        assert np.all(np.diff(lidar)>0)
        assert all(ids[int(row['keyframe_id'])]==int(row['timestamp']) for row in rows)
        pairs = read(OUT/robot/'sync_pairs.csv')
        for clock in ('rgb_bag_ns','rgb_header_ns','depth_bag_ns','depth_header_ns'):
            frames = pairs if clock.startswith('rgb') else [row for row in pairs if row['paired']=='True']
            time = np.array([int(r[clock]) for r in frames],np.int64)
            inside = (time>=lidar[0]) & (time<=lidar[-1])
            samples = time[inside]
            after = np.searchsorted(lidar,samples,side='right').clip(1,len(lidar)-1)
            nearest = np.minimum(samples-lidar[after-1],abs(lidar[after]-samples))
            gaps = lidar[after]-lidar[after-1]
            ns = stats(nearest/1e6)
            overlap.append(dict(robot=robot,clock=clock,lidar_first_ns=int(lidar[0]),lidar_last_ns=int(lidar[-1]),
                lidar_poses=len(lidar),camera_first_ns=int(time.min()),camera_last_ns=int(time.max()),
                overlap_first_ns=int(samples.min()),overlap_last_ns=int(samples.max()),
                overlap_s=(int(samples.max())-int(samples.min()))/1e9,frames_in_range=len(samples),
                frames_in_range_paired_depth=sum(bool(flag) and r['paired']=='True' for flag,r in zip(inside,frames)),
                nearest_median_ms=ns['median'],nearest_p95_ms=ns['p95'],nearest_max_ms=ns['max'],
                bracket_p95_ms=float(np.percentile(gaps/1e6,95)),bracket_max_ms=float(gaps.max()/1e6),
                timestamp_id_match=True))
        camera = mount['robots'][robot]
        with zipfile.ZipFile(archive) as z:
            import hashlib
            urdf = z.read(camera['URDF_member'])
            assert hashlib.sha256(urdf).hexdigest()==camera['URDF_sha256']
        topic = next(t for t in json.loads((OUT/robot/'DEPTH_calibration_audit.json').read_text())['topics'] if t['topic'].endswith('depth_to_color'))
        assert topic['topic'].endswith('depth_to_color') and topic['distinct_values']==1
        value = topic['variants'][0]['value']
        rotation = np.array(value['rotation']).reshape(3,3,order='F')
        assert np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) and abs(np.linalg.det(rotation)-1)<1e-6
        topics = {}
        missing_archives = []
        for sensor in ('lidar','camera_rgb','camera_depth'):
            path = RAW/robot/f'{robot}_main_campus_{sensor}.zip'
            if not path.is_file():
                missing_archives.append(str(path))
                continue
            with zipfile.ZipFile(path) as z:
                metadata = [z.read(name).decode() for name in z.namelist() if name.endswith('metadata.yaml')]
            topics[sensor] = sorted(set(re.findall(r'^\s*name:\s*["\']?([^\s"\']+)', '\n'.join(metadata),re.M)))
        calibration[robot]=dict(urdf_member=camera['URDF_member'],urdf_sha256=camera['URDF_sha256'],
             camera_link_from_os_sensor=camera['T_camera_link_from_os_sensor'],
             T_lidar_from_color_optical=None,missing='camera_link to measured color optical transform',
             depth_to_color=dict(rotation_row_major=rotation.tolist(),translation_m=value['translation'],
                                 determinant=float(np.linalg.det(rotation)),topic=topic['topic']),
             bag_topics=topics,unavailable_original_archives=missing_archives,verified_complete_extrinsic=False)
    writecsv(DEST/'timestamp_overlap.csv',overlap)
    dump(DEST/'sensor_calibration_audit.json',dict(robots=calibration,calibration_archive=str(archive),
        trajectory=str(trajectory.relative_to(ROOT)),trajectory_sha256=sha(trajectory),
        trajectory_convention='T_world_from_os_sensor, translation metres, quaternion xyzw, original LiDAR bag ns',
        required_formula='T_world_from_color = T_world_from_os_sensor @ T_os_sensor_from_color',
        transform_chain='T_os_sensor_from_color = inverse(T_camera_link_from_os_sensor) @ T_camera_link_from_color; last term unverified',
        clock_basis='original integer bag ns; header differences audited separately; no clock offset fitted',
        repo_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        additional_lidar_checkout_head=subprocess.check_output(
            ['git','-C','/home/cas/fyp_place_recognition_gicp_value','rev-parse','HEAD'],text=True).strip(),
        imu_extrinsic_substituted=False,proof_of_concept='NOT_RUN_MISSING_VERIFIED_CAMERA_LIDAR_EXTRINSIC'))


if __name__=='__main__':
    boundary_inventory()
    hybrid_inventory()
