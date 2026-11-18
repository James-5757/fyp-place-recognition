"""Stage32 dependency audit and original-image/native-extractor evidence collection."""
import argparse
import csv
import hashlib
import json
import platform
import sqlite3
import subprocess
from contextlib import ExitStack, closing
from pathlib import Path

import cv2
import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from common import OUT, dump, sha
from stage26_manifest import read_csv, write_csv
from stage27_continuous import depth_pixels, message, open_bag, rgb_pixels

BASE=OUT.parent
DEST=BASE/'32_native_tracking_forensics'
SDK=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')
ORIGINAL_ROOT=Path('/home/cas/fyp_place_recognition')
EVENTS=(9936,10071,12057)
PARENT='1859291a0d448ccf62b9e3317f2b2055c5497b25'


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def flow(previous,current):
    a=cv2.cvtColor(previous,cv2.COLOR_BGR2GRAY)
    b=cv2.cvtColor(current,cv2.COLOR_BGR2GRAY)
    corners=cv2.goodFeaturesToTrack(a,800,.01,5)
    if corners is None:
        return dict(flow_seed_count=0,flow_tracked_count=0,flow_fb_valid_count=0)
    kwargs=dict(winSize=(21,21),maxLevel=3)
    target,valid,_=cv2.calcOpticalFlowPyrLK(a,b,corners,None,**kwargs)
    back,reverse,_=cv2.calcOpticalFlowPyrLK(b,a,target,None,**kwargs)
    error=np.linalg.norm(back-corners,axis=2).ravel()
    keep=(valid.ravel()!=0)&(reverse.ravel()!=0)&(error<=1.)
    magnitude=np.linalg.norm(target-corners,axis=2).ravel()
    return dict(flow_seed_count=len(corners),flow_tracked_count=int(valid.sum()),
                flow_fb_valid_count=int(keep.sum()),flow_fb_valid_fraction=float(keep.mean()),
                flow_mean_px=float(magnitude[keep].mean()) if keep.any() else None,
                flow_fb_error_median_px=float(np.median(error[keep])) if keep.any() else None)


def source_locations(reference):
    patterns={
        'Tracking.cc': ['Tracking::GrabImageRGBD(', 'Tracking::TrackReferenceKeyFrame(',
                        'Tracking::TrackWithMotionModel(', 'Tracking::TrackLocalMap(',
                        'Tracking::Relocalization(', 'mnMatchesInliers<30','mnMatchesInliers<50'],
        'Frame.cc': ['Frame::ComputeStereoFromRGBD(', 'imDepth.at<float>(v,u)'],
        'Optimizer.cc': ['Optimizer::PoseOptimization(', 'const float chi2Mono[4]', 'const float chi2Stereo[4]',
                         'const int its[4]', 'return nInitialCorrespondences-nBad'],
        'LocalMapping.cc': ['LocalMapping::Run(', 'LocalMapping::ProcessNewKeyFrame(',
                            'LocalMapping::MapPointCulling(', 'LocalMapping::KeyFrameCulling(',
                            'Optimizer::LocalBundleAdjustment(']}
    result={}
    for filename,lookups in patterns.items():
        path=reference/'src'/filename
        lines=path.read_text().splitlines()
        result[filename]=dict(sha256=sha(path),matches={p:[i+1 for i,line in enumerate(lines) if p in line] for p in lookups})
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scratch',type=Path,required=True)
    args=p.parse_args()
    work=args.scratch/'stage32'
    reference=work/'source_reference'
    baseline=work/'baseline_reference'
    DEST.mkdir(parents=True,exist_ok=True)
    work.mkdir(parents=True,exist_ok=True)
    legacy=load(baseline/'run_provenance.json')['native_dependencies']['robot3']
    feature_provenance=load(baseline/'feature_provenance.json')
    original_native=load(baseline/'native_provenance.json')
    paths={
        'sdk_directory':SDK,'tracking_source':SDK/'src/Tracking.cc','system_source':SDK/'src/System.cc',
        'system_header':SDK/'include/System.h','library':SDK/'lib/libORB_SLAM3.so',
        'vocabulary':SDK/'Vocabulary/ORBvoc.txt',
        'executable':ORIGINAL_ROOT/'outputs/cumulti_v1/24_rgbd_full_v1/bin/rgbd_full',
        'sequence':ORIGINAL_ROOT/'outputs/cumulti_v1/24_rgbd_full_v1/robot3/sequence.txt'}
    current={k:dict(path=str(v),exists=v.exists(),sha256=sha(v) if v.is_file() else None) for k,v in paths.items()}
    known={Path(path).relative_to(SDK).as_posix():value for path,value in feature_provenance['source_hashes'].items()
           if path.startswith(str(SDK)) and not path.endswith('.so')}
    known.update({Path(row['path']).relative_to(SDK).as_posix():row['sha256'] for row in original_native['files']
                  if row['path'].startswith(str(SDK)) and row['path'].endswith(('System.h','System.cc'))})
    normalization_receipt=work/'reference_lf_normalization.json'
    normalized=[]
    for path in sorted(reference.rglob('*')):
        if path.suffix not in ('.cc','.h'):
            continue
        raw=path.read_bytes()
        canonical=raw.replace(b'\r\n',b'\n')
        normalized.append(dict(path=path.relative_to(reference).as_posix(),
                               original_git_blob_sha256=hashlib.sha256(raw).hexdigest(),
                               lf_sha256=hashlib.sha256(canonical).hexdigest()))
        if canonical!=raw:
            path.write_bytes(canonical)
    if not normalization_receipt.exists():
        dump(normalization_receipt,normalized)
    comparison=[]
    for path,expected in known.items():
        actual=sha(reference/path)
        comparison.append(dict(path=path,historical_sha256=expected,recovered_reference_sha256=actual,
                               matches_historical=actual==expected))
    assert all(row['matches_historical'] for row in comparison if row['path'] in
               ('src/Tracking.cc','src/Frame.cc','src/ORBextractor.cc','include/Tracking.h'))
    missing=[key for key in ('sdk_directory','library','executable') if not current[key]['exists']]
    if not missing:
        raise RuntimeError('Native dependencies became available; inspect and instrument before replay')
    config=baseline/'camera.yaml'
    poses=read_csv(OUT/'robot3/camera_poses.csv')
    pairs=read_csv(OUT/'robot3/sync_pairs.csv')
    coverage=read_csv(OUT/'robot3/depth_coverage.csv')
    cfg=load(OUT/'dataset_manifest.json')['robots']['robot3']
    settings=legacy['settings']
    assert settings['ORBextractor.nFeatures']==1000 and settings['Camera.RGB']==0 and settings['RGBD.DepthMapFactor']==1000.
    watched=[OUT/'robot3/camera_poses.csv',OUT/'robot3/sync_pairs.csv',
             BASE/'24d_rgbd_pose_graph/pose_graph.json',BASE/'29_spatial_audit/stage29_epoch_transforms.json',
             BASE/'30_rgbd_pose_forensics/stage30_pose_jump_audit.csv',BASE/'31_epoch9_persistence/preservation_audit.json',
             BASE/'26_rgbd_reconstruction/registration_policy.json',BASE/'28_multisensor/gravity_alignment.json']
    before={str(path):sha(path) for path in watched}
    historical_sequence=''.join(f"{row['relative_ns']} {ORIGINAL_ROOT/row['rgb']} "
        f"{ORIGINAL_ROOT/'outputs/cumulti_v1/24_rgbd_full_v1/robot3/depth'/f'{i:06d}.png'}\n"
        for i,row in enumerate(pairs)).encode()
    k=np.array(cfg['RGB']['calibration']['K']).reshape(3,3)
    k[:2]*=.5
    d=np.array(cfg['RGB']['calibration']['D'])
    assert np.allclose(k[[0,1],[0,1]],[settings['Camera1.fx'],settings['Camera1.fy']])
    extr=cfg['DEPTH']['extra']['T_color_from_depth']
    rotation,translation=np.array(extr['R']),np.array(extr['t_m'])
    selected=sorted({i for event in EVENTS for i in range(event-5,event+6)})
    all_images={}
    input_rows=[]
    depth_rows=[]
    image_rows=[]
    frame_dir=work/'frames'
    frame_dir.mkdir(exist_ok=True)
    store=get_typestore(Stores.ROS2_HUMBLE)
    with ExitStack() as stack:
        rgb,depth=[stack.enter_context(closing(open_bag('robot3',cfg,kind,args.scratch))) for kind in ('RGB','DEPTH')]
        for db in (rgb,depth):
            for name,encoding,definition in db.execute('SELECT topic_type,encoding,encoded_message_definition FROM message_definitions'):
                if encoding=='ros2msg':
                    store.register(get_types_from_msg(definition,name))
        for i in selected:
            pair=pairs[i]
            raw=rgb_pixels(message(rgb,store,cfg['RGB'],pair,'rgb'))
            image_path=frame_dir/f'{i:06d}.jpg'
            depth_path=frame_dir/f'{i:06d}.png'
            assert cv2.imwrite(str(image_path),raw[:,:,::-1],[cv2.IMWRITE_JPEG_QUALITY,85])
            image=cv2.imread(str(image_path))
            z=(depth_pixels(message(depth,store,cfg['DEPTH'],pair,'depth'),rotation,translation,k,d)
               if pair['paired']=='True' else np.zeros((400,640),np.uint16))
            assert abs((z>0).mean()-float(coverage[i]['valid_depth_fraction']))<1e-12
            assert cv2.imwrite(str(depth_path),z)
            all_images[i]=image
            gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            image_rows.append(dict(index=i,rgb_bag_ns=pair['rgb_bag_ns'],
                                   mean_brightness=float(gray.mean()),contrast_std=float(gray.std()),
                                   laplacian_variance=float(cv2.Laplacian(gray,cv2.CV_64F).var()),
                                   image_source='restored Stage24 JPEG85 procedure; original baseline JPEG byte receipt unavailable'))
            valid=z[z>0]/1000.
            depth_rows.append(dict(index=i,rgb_bag_ns=pair['rgb_bag_ns'],rgb_header_ns=pair['rgb_header_ns'],
                                   depth_bag_ns=pair['depth_bag_ns'],depth_header_ns=pair['depth_header_ns'],
                                   sync_ms=pair['bag_sync_ms'],width=640,height=400,encoded_depth_unit='mm uint16',
                                   valid_depth_pixels=int((z>0).sum()),invalid_depth_pixels=int((z==0).sum()),
                                   coverage=float((z>0).mean()),minimum_m=float(valid.min()) if len(valid) else None,
                                   median_m=float(np.median(valid)) if len(valid) else None,
                                   p95_m=float(np.quantile(valid,.95)) if len(valid) else None,
                                   maximum_m=float(valid.max()) if len(valid) else None,
                                   geometry_used_for_tracking=False))
            input_rows.append(dict(index=i,rgb=str(image_path.resolve()),depth=str(depth_path.resolve()),
                                   rgb_sha256=sha(image_path),depth_sha256=sha(depth_path),
                                   **{key:pair[key] for key in ('rgb_message_id','depth_message_id','rgb_bag_ns','rgb_header_ns',
                                                              'depth_bag_ns','depth_header_ns')}))
    for row in image_rows:
        i=row['index']
        if i-1 in all_images:
            row.update(flow(all_images[i-1],all_images[i]))
        else:
            row.update(dict(flow_seed_count=None,flow_tracked_count=None,flow_fb_valid_count=None,
                            flow_fb_valid_fraction=None,flow_mean_px=None,flow_fb_error_median_px=None))
    input_list=work/'feature_inputs.txt'
    input_list.write_text(''.join(f"{row['index']} {row['rgb']} {row['depth']}\n" for row in input_rows))
    binary=work/'stage32_native_feature_probe'
    command=['g++','-std=c++14','-O2',str(Path(__file__).with_name('stage32_feature_probe.cc')),
             str(reference/'src/ORBextractor.cc'),'-I'+str(reference/'include')]
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True).split()
    compiled=subprocess.run(command+flags+['-o',str(binary)],capture_output=True,text=True)
    (work/'feature_build.log').write_text(compiled.stdout+compiled.stderr)
    compiled.check_returncode()
    extracted=work/'feature_counts.csv'
    subprocess.run([str(binary),str(input_list),str(extracted)],check=True)
    detector={int(row['index']):row for row in read_csv(extracted)}
    associations=[]
    for i in selected:
        original=poses[i]
        associations.append(dict(index=i,rgb_bag_ns=pairs[i]['rgb_bag_ns'],
                                 detector_measurement='STANDALONE_HASH_MATCHED_ORB_EXTRACTOR_REEXECUTION',
                                 native_tracking_associations=None,native_mnMatchesInliers=None,native_outliers=None,
                                 historical_saved_nonbad_associated_points=original['tracked_points'],
                                 **{k:v for k,v in detector[i].items() if k!='index'}))
    for row in depth_rows:
        row.update({key:detector[row['index']][key] for key in ('depth_supported_keypoints',
                    'missing_depth_keypoints','features_near_depth_discontinuity','depth_keypoint_min_m',
                    'depth_keypoint_median_m','depth_keypoint_p95_m','depth_keypoint_max_m','close_depth_keypoints')})
    trace=[dict(index=i,rgb_bag_ns=pairs[i]['rgb_bag_ns'],measurement_status='NATIVE_REPLAY_NOT_RUN',
                historical_saved_state=poses[i]['tracking_state'],historical_map_id=poses[i]['map_id'],
                historical_init_kf=poses[i]['map_init_kf'],native_state_before=None,native_state_after=None,
                motion_model_attempted=None,motion_model_return=None,reference_attempted=None,reference_return=None,
                local_map_attempted=None,local_map_return=None,relocalization_attempted=None,relocalization_return=None,
                native_reference_keyframe=None,native_last_reloc_frame=None,native_map_change_index=None) for i in selected]
    optimizer=[dict(index=i,measurement_status='UNAVAILABLE_NO_NATIVE_REPLAY',T_before=None,T_after=None,
                    native_initial_correspondences=None,native_final_inliers=None,native_outliers=None,
                    native_chi2_distribution=None,native_reprojection_residuals=None,native_depth_residuals=None,
                    native_acceptance=None,executed_iterations=None) for i in selected]
    mapping=[dict(index=i,measurement_status='UNAVAILABLE_NO_NATIVE_REPLAY',event_type=None,keyframe_id=None,
                  created_map_points=None,culled_map_points=None,replaced_map_points=None,
                  local_keyframes=None,local_map_points=None,local_BA_begin=None,local_BA_end=None) for i in selected]
    for filename,records in (('stage32_frame_tracking_trace.csv',trace),('stage32_feature_association.csv',associations),
                             ('stage32_pose_optimization.csv',optimizer),('stage32_depth_support.csv',depth_rows),
                             ('stage32_local_mapping_events.csv',mapping)):
        write_csv(DEST/filename,records)
    write_csv(DEST/'stage32_image_quality.csv',image_rows)
    write_csv(DEST/'stage32_sensor_recovery_receipts.csv',input_rows)
    restore=dict(required_sdk_commit=legacy['sdk_revision'],
                 actual_run_system_cc_sha256=known['src/System.cc'],
                 actual_run_system_h_sha256=known['include/System.h'],
                 original_library_sha256=legacy['library_sha256'],
                 original_binary_sha256=legacy['binary_sha256'],
                 original_vocabulary_sha256=legacy['vocabulary_sha256'],
                 note='Restore authenticated SDK working tree or its original patch and baseline native dependencies. Core extractor and tracking sources were recovered.')
    recovered_vocab=work/'vocabulary/ORBvoc.txt'
    assert recovered_vocab.exists() and sha(recovered_vocab)==legacy['vocabulary_sha256']
    provenance=dict(native_opencv=subprocess.check_output(['pkg-config','--modversion','opencv4'],text=True).strip(),
                    compiler=subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
                    source_code_sha256={p.name:sha(p) for p in Path(__file__).parent.glob('stage32_*') if p.suffix in ('.py','.cc')},
                    recovered_vocabulary_sha256=sha(recovered_vocab),recovered_vocabulary_bytes=recovered_vocab.stat().st_size,status='BLOCKED_NATIVE_REPLAY_DEPENDENCIES',parent_commit=PARENT,
                    original_dependency_inventory=current,historical_native_receipt=legacy,
                    source_reference_commit=legacy['sdk_revision'],source_reference_hash_comparison=comparison,source_lf_normalization=load(normalization_receipt),
                    static_source_locations=source_locations(reference),
                    current_camera_settings_sha256=sha(config),historical_camera_settings=settings,
                    original_sequence_sha256=None,reconstructed_full_sequence_sha256=hashlib.sha256(historical_sequence).hexdigest(),
                    reconstructed_sequence_frames=len(pairs),required_replay_last_frame=12087,
                    restored_selected_frames=len(selected),native_tracker_frames_executed=0,
                    native_extractor_only_frames=len(associations),feature_probe_binary_sha256=sha(binary),
                    full_tracking_instrumented_binary=None,feature_probe_build_command=command+flags+['-o',str(binary)],
                    restoration_requirements=restore,python=platform.python_version(),opencv_python=cv2.__version__)
    dump(DEST/'stage32_run_provenance.json',provenance)
    compare={}
    for event in EVENTS:
        records={i:dict(image=next(r for r in image_rows if r['index']==i),
                        depth=next(r for r in depth_rows if r['index']==i),
                        features=next(r for r in associations if r['index']==i))
                 for i in range(event-2,event+2)}
        compare[str(event)]=dict(anomalous_pair=[event-1,event],controls=[[event-2,event-1],[event,event+1]],
                                observations=records,root_cause='UNRESOLVED_NATIVE_EVIDENCE_UNAVAILABLE',
                                native_evidence_level=None,native_replay_complete=False,
                                measured_observation_level='CONFIRMED_DETECTOR_REEXECUTION_ONLY',
                                supported_explanation='No sudden detector-level depth-support collapse',
                                plausible_unverified_hypotheses=[
                                    'Incorrect feature-to-MapPoint association','Pose optimization instability',
                                    'Unreliable retained depth correspondences','Local Mapping interference',
                                    'Visual ambiguity or geometric weakness'])
    dump(DEST/'stage32_event_comparison.json',compare)
    assert before=={str(path):sha(path) for path in watched}
    dump(DEST/'stage32_preservation_audit.json',dict(status='PASS',before=before,
         after={str(path):sha(path) for path in watched},original_sdk_written=False,
         baseline_executable_written=False,tracking_replayed=False,global_maps_generated=False))
    from stage32_report import report
    report(DEST,provenance,compare,image_rows,associations)
    print('STAGE32 DEPENDENCY AUDIT AND FEATURE PROBE COMPLETE; NATIVE REPLAY BLOCKED',flush=True)


if __name__=='__main__':
    main()
