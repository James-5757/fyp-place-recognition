"""Independent saved-artifact audit; no tracking/SLAM/GT evaluator is invoked."""
import subprocess
import cv2,pandas as pd
from common import *
from projection import tests

def run():
    checks=[]
    def check(name,value):
        checks.append(dict(name=name,passed=bool(value)));assert value,name
    manifest=json.loads((B/'camera_data_manifest.json').read_text());policy=manifest['policy']
    keys=pd.read_csv(B/'visual_keyframe_manifest.csv',dtype={'camera_bag_timestamp_ns':str})
    for robot,data in manifest['robots'].items():
        timeline=pd.read_csv(B/f'{robot}_camera_timestamp_index.csv',dtype={'camera_bag_timestamp_ns':str})
        ts=timeline.camera_bag_timestamp_ns.map(int).to_numpy(np.int64)
        check(robot+'_complete_RGB_timeline',len(timeline)==data['original_frames'] and np.all(np.diff(ts)>0))
        expected=[]
        for t in range(int(ts[0]),int(ts[-1])+1,10**9):
            j=min(int(np.searchsorted(ts,t)),len(ts)-1)
            if not expected or j!=expected[-1]:expected.append(j)
        actual=keys[keys.robot_id==robot]
        check(robot+'_independent_camera_keyframes',actual.rgb_message_index.tolist()==expected)
        for frame in data['pilot_frames']:
            p=ROOT/frame['file'];check(p.name+'_hash',sha(p)==frame['sha256'])
            check(p.name+'_decode',cv2.imread(str(p)).shape[:2]==(frame['height'],frame['width']))
        check(robot+'_native_CameraInfo_resolution',[data['camera_info']['width'],data['camera_info']['height']]==[1280,800])
    runtime=json.loads((B/'runtime_memory_pilot.json').read_text());runs=runtime['native_runs'];check('six_actual_native_windows',len(runs)==6)
    forbidden=('gt_utm','gt_rel','/ground_truth/','imu_gps','imu.csv','gnss','/full_2hz/','scan_context','/22b_','_lidar.zip','clean_local_trajectory')
    for item in runs:
        p=ROOT/item['trajectory'];df=pd.read_csv(p);valid=df.valid_pose==1;values=df.loc[valid,['tx','ty','tz','qx','qy','qz','qw']].to_numpy()
        check(p.parent.name+'_track_count',len(df)==60 and int(valid.sum())==item['valid_poses'])
        check(p.parent.name+'_finite_poses',np.isfinite(values).all())
        check(p.parent.name+'_unit_rotations',not len(values) or np.max(abs(np.linalg.norm(values[:,3:],axis=1)-1))<1e-5)
        trace=(p.parent/'native_file_access.log').read_text()
        check(p.parent.name+'_native_read_guard_audit',not any(s in trace.lower() for s in forbidden))
        check(p.parent.name+'_no_metric_scale',item['metric_scale'] is False)
        # The first initial frames are invalid rather than fabricated identities.
        check(p.parent.name+'_missing_poses_explicit',df.loc[~valid,['tx','ty','tz']].isna().all().all())
    geo=pd.read_csv(B/'visual_geometric_pilot.csv');check('72_image_only_candidates',len(geo)==72 and (geo.candidate_database_size==36).all())
    check('geometry_counts_bounded',((geo.cheirality_inliers<=geo.inliers)&(geo.inliers<=geo.matches)).all())
    check('no_metric_visual_factors',runtime['metric_visual_loop_factors_created']==0)
    deps=json.loads((B/'visual_frontend_dependency_audit.json').read_text());check('native_and_Python_no_forbidden_sensor_reads',not deps['native_forbidden_reads'] and all(deps[k] is False for k in ['LiDAR','SC','GICP','LiDAR_odometry','GT']))
    # Exercise the real open guard in a child process before an actual OS read.
    code="from common import camera_guard; camera_guard([]); open('/home/cas/CU-Multi/raw/main_campus/robot1/robot1_main_campus_gt_utm_poses.csv')"
    result=subprocess.run([sys.executable,'-c',code],cwd=Path(__file__).parent,capture_output=True,text=True)
    check('GT_open_rejected_before_read',result.returncode!=0 and 'Stage24 camera-only guard' in result.stderr)
    seg=json.loads((A/'segmentation_runtime.json').read_text());check('12_actual_semantic_images',len(seg['samples'])==12)
    for sample in seg['samples']:
        p=ROOT/sample['array_file'];check(p.name+'_semantic_confidence_hash',sha(p)==sample['array_sha256'])
        a=np.load(p);prob=a['confidence'];cat=a['category'];check(p.name+'_probability_and_unknown',np.isfinite(prob).all() and (prob>=0).all() and (prob<=1).all() and (cat[prob<.5]==0).all())
    dist=pd.read_csv(A/'semantic_class_distribution.csv');check('pixel_distributions_sum_one',np.allclose(dist.groupby('sample').fraction.sum(),1))
    projection=json.loads((A/'rgb_projection_validation.json').read_text());check('no_guessed_projection',projection['real_LiDAR_RGB_projection_performed'] is False and projection['status']=='BLOCKED')
    check('synthetic_projection_tests',tests()['status']=='PASS')
    asset=json.loads((A/'map_asset_manifest.json').read_text())
    check('P5_trajectory_frozen',sha(asset['trajectory_path'])==asset['trajectory_sha256'])
    check('source_scan_selection580',asset['source_scan_count']==580)
    for row in asset['artifacts']:check(row['path']+'_asset_hash',sha(ROOT/row['path'])==row['sha256'])
    check('all_3D_attributes_unknown',asset['real_RGB_colored_vertices']==asset['real_semantic_3D_vertices']==0)
    preservation=json.loads((A/'historical_preservation_audit.json').read_text());check('historical_files_unchanged',preservation['status']=='PASS')
    dump(A/'independent_validation.json',dict(status='PASS',checks=checks,scope='artifact/data/dependency correctness; does NOT promote blocked RGB/semantic3D mapping or vision-only multi-robot SLAM'))
    dump(B/'independent_validation.json',dict(status='PASS',checks=len(checks),camera_guard_test=True,GT_evaluation_run=False,native_solver_repeated=False,scientific_promotion=False))
    print('INDEPENDENT_VALIDATION_PASS',len(checks),flush=True)

if __name__=='__main__':run()
