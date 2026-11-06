"""Exact availability/calibration audit and honest P5 presentation package."""
import shutil,subprocess,zipfile,xml.etree.ElementTree as ET
import pandas as pd,yaml
from scipy.spatial.transform import Rotation
from common import *
from projection import tests

def chain(joints,source,target):
    todo=[(source,np.eye(4),[])];seen=set()
    while todo:
        frame,T,path=todo.pop()
        if frame==target:return dict(matrix=T.tolist(),path=path)
        if frame in seen:continue
        seen.add(frame)
        for j in joints:
            if frame==j['parent']:todo.append((j['child'],np.linalg.inv(np.array(j['T_parent_child']))@T,path+[j['name']+' inverse']))
            elif frame==j['child']:todo.append((j['parent'],np.array(j['T_parent_child'])@T,path+[j['name']]))
    return None

def run():
    A.mkdir(parents=True,exist_ok=True);SITE.mkdir(parents=True,exist_ok=True)
    man=json.loads((B/'camera_data_manifest.json').read_text());depth=json.loads((B/'depth_camera_audit.json').read_text());available=[];robotcal={}
    calib=RAW/'calib/robot_description.zip'
    with zipfile.ZipFile(calib) as z:
        optical_mentions={n:z.read(n).decode(errors='replace').lower().count('optical') for n in z.namelist() if n.endswith(('.urdf','.xacro'))}
        for robot in ['robot1','robot2','robot3']:
            member=f'robot_description/urdf/{robot}.urdf';xml=z.read(member);root=ET.fromstring(xml);joints=[]
            for j in root.findall('joint'):
                origin=j.find('origin');T=np.eye(4)
                if origin is not None:
                    T[:3,3]=[float(x) for x in origin.get('xyz','0 0 0').split()]
                    T[:3,:3]=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
                joints.append(dict(name=j.get('name'),type=j.get('type'),parent=j.find('parent').get('link'),child=j.find('child').get('link'),T_parent_child=T.tolist()))
            target=f'{robot}_camera_color_optical_frame';source=f'{robot}_os_sensor'
            robotcal[robot]=dict(URDF_member=member,URDF_sha256=hashlib.sha256(xml).hexdigest(),
                all_joints=joints,T_camera_link_from_os_sensor=chain(joints,source,f'{robot}_camera_link'),
                T_color_optical_from_os_sensor=chain(joints,source,target),
                status='BLOCKED_MISSING_CAMERA_LINK_TO_COLOR_OPTICAL',
                optical_frame='CameraInfo/Image frame_id; optical x right,y down,z forward per ROS convention; no measured link-to-optical chain present in inspected URDF')
            for sensor in ['camera_rgb','camera_depth','lidar','imu_gps','gt_utm_poses.csv']:
                name=f'{robot}_main_campus_{sensor}'+('' if sensor.endswith('.csv') else '.zip');p=RAW/'main_campus'/robot/name
                row=dict(robot_id=robot,sensor=sensor,path=str(p),available=p.exists(),size_bytes=p.stat().st_size if p.exists() else None,mtime_ns=p.stat().st_mtime_ns if p.exists() else None)
                if p.exists() and sensor!='gt_utm_poses.csv':
                    with zipfile.ZipFile(p) as zp:
                        metadata=yaml.safe_load(zp.read(next(n for n in zp.namelist() if n.endswith('metadata.yaml'))))['rosbag2_bagfile_information']
                        row['topics']=[dict(topic=t['topic_metadata']['name'],type=t['topic_metadata']['type'],count=t['message_count']) for t in metadata['topics_with_message_count']]
                        row['metadata_nominal_duration_s']=metadata['duration']['nanoseconds']/1e9
                        row['archive_format']='ZIP containing ROS2 SQLite3 db3 + metadata.yaml'
                        row['optical_static_tf_topic_present']=any(t['topic'] in ['/tf_static',f'{robot}/tf_static'] for t in row['topics'])
                available.append(row)
    audit=dict(status='BLOCKED_COMPLETE_OPTICAL_MOUNTING_EXTRINSIC_NOT_LOCALLY_VERIFIED',robots=robotcal,
        T_target_source='p_target = R_target_source @ p_source + t_target_source. URDF origin is T_parent_child; use inverse to map parent coordinates into child.',
        LiDAR_point_frame='robotN_os_sensor; existing unmodified XYZI frame audit',
        RGB_calibration={r:x['camera_info'] for r,x in man['robots'].items()},depth_camera_audit=depth,
        searched='all URDF joints; camera RGB/depth/LiDAR/non-GT IMU archive topic metadata. No /tf_static in these archives.',
        no_guessed_camera_link_equals_optical=True,visual_mesh_rotation_not_measurement_extrinsic=True,
        source=dict(archive=str(calib),sha256=sha(calib),optical_mentions_in_all_URDF_xacro=optical_mentions),
        missing=['measured T_camera_color_optical_from_camera_link (including translation)','Robot3 camera-depth archive',
                 'native depth Image topic/scale or explicitly validated camera-cloud-to-depth adapter','camera exposure-to-LiDAR hardware clock calibration'])
    dump(A/'calibration_audit.json',audit);dump(A/'data_availability_report.json',dict(assets=available,decode_scope=dict(RGB=['robot1','robot3'],depth_cloud=['robot1'],robot2='archive topic inventory plus historical decoded RGB validation only; no new depth message decode'),
        no_redownload=True,raw_modified=False,GT_pose_files='existence/size only; contents NOT read'))
    write_csv(A/'data_availability_report.csv',[{k:v for k,v in row.items() if k not in ['topics']} for row in available],list(dict.fromkeys(k for row in available for k in row if k!='topics')))
    intrinsic={}
    for r,x in man['robots'].items():
        info=x['camera_info'];K=np.array(info['K']).reshape(3,3);image=x['image']
        ok=K[0,0]>0 and K[1,1]>0 and np.linalg.det(K)>0 and [info['width'],info['height']]==[image['width'],image['height']] and info['frame_id']==image['frame_id']
        intrinsic[r]=dict(status='PASS_INTRINSICS_AND_DECODE' if ok else 'FAIL',actual=info,image=image,resize_K_rule='rows0/1 multiplied by0.5 for640x400; distortion coefficients unchanged')
    dump(B/'camera_calibration_validation.json',dict(intrinsics=intrinsic,inter_robot_calibration='independent monocular cameras do not require LiDAR mounting transform',
        RGBD='BLOCKED_NO_ROBOT3_CAMERA_DEPTH_AND_NO_NATIVE_DEPTH_IMAGE',metric_scale='NOT_OBSERVABLE_FROM_MONOCULAR_RGB_ALONE'))
    sync=[]
    rgb=pd.read_csv(B/'robot1_camera_timestamp_index.csv');cloud=pd.read_csv(B/'robot1_depth_pointcloud_timestamp_index.csv')
    rt=rgb.camera_bag_timestamp_ns.to_numpy(np.int64);dt=cloud.depth_bag_timestamp_ns.to_numpy(np.int64);j=np.searchsorted(rt,dt);j=np.clip(j,0,len(rt)-1);left=np.maximum(j-1,0);j=np.where(abs(rt[left]-dt)<=abs(rt[j]-dt),left,j)
    for d,t in zip(dt,rt[j]):sync.append(dict(depth_bag_timestamp_ns=str(d),RGB_bag_timestamp_ns=str(t),absolute_delta_ms=abs(int(d)-int(t))/1e6))
    write_csv(B/'robot1_camera_depth_RGB_sync.csv',sync)
    first_depth=depth['robots']['robot1']['sample_pointcloud'];rgb_sample=min(man['robots']['robot1']['pilot_frames'],key=lambda f:abs(int(f['camera_bag_timestamp_ns'])-int(first_depth['bag_timestamp_ns'])))
    dump(B/'camera_depth_sync_summary.json',dict(robot1_nearest_bag_delta_ms=stats([x['absolute_delta_ms'] for x in sync]),
        first_real_decoded_depth_RGB_header_delta_ns=int(first_depth['header_timestamp_ns'])-int(rgb_sample['camera_header_timestamp_ns']),
        header_agreement_scope='one actually decoded nearest RGB/depth-cloud pair only; not a full exposure-clock calibration',
        basis='recording bag times, not hardware exposure synchronization',robot3='MISSING_DEPTH'))
    historical=dict(model='ViT-B-32-quickgelu',weights='laion400m_e32',single='cosine of L2-normalized embeddings',RGB5_Mean='causal [k-4..k], start padding; average then L2 normalize; cosine',Cross_Max='max of all25 pairwise cosine similarities for two causal5-frame windows',
        stage5_dependency='all three methods rerank only frozen Scan Context Top20; RGB sampled nearest2Hz LiDAR keyframes; candidate-conditioned1830/1833 and end-to-end distinguished',
        stage5=dict(SC_end_to_end_R1=.993453,Single_RGB_R1=.702128,Mean5_R1=.682488,CrossMax_R1=.714675,Single_rescues=4,Single_regressions=538,Mean_rescues=2,Mean_regressions=572,CrossMax_rescues=1,CrossMax_regressions=512),
        stage6=dict(SC_R1=.993453,strict_single_gate_R1=.993999,invocation_fraction=.0431,rescues=1,regressions=0,limitation='small strict selective gain; no general viewpoint invariance; not camera tracking'),
        not_visual_SLAM=['no visual motion estimation','no metric visual local trajectory','no visual map reconstruction','SC candidate dependency','no independent image-only retrieval in historical5/6'],
        source_files={str(p.relative_to(ROOT)):sha(p) for p in [ROOT/'src/cumulti/run_stage5_visual_viewpoint.py',ROOT/'src/cumulti/run_stage6_selective_visual_verification.py',ROOT/'docs/CUMULTI_STAGE5_VISUAL_VIEWPOINT_ANALYSIS.md',ROOT/'docs/CUMULTI_STAGE6_SELECTIVE_VISUAL_VERIFICATION.md']})
    dump(B/'crossmax_baseline_review.json',historical)
    # Reuse exact frozen P5 display buffers, no point deformation or scene alignment.
    source=ROOT/'demo/stage23d_policy_map_compare';old=json.loads((source/'data.json').read_text());state=old['states']['P5_incremental']
    maps={};artifacts=[];coverage=[]
    for r in ['robot1','robot3']:
        spec=state['maps'][r];src=source/spec['file'];dst=SITE/'assets'/src.name;dst.parent.mkdir(exist_ok=True);shutil.copyfile(src,dst)
        assert sha(src)==spec['sha256']==sha(dst)
        maps[r]=dict(file='assets/'+dst.name,points=spec['points'],full_voxel_points=spec['full_voxel_points'])
        for region in ['ALL']+list(state['regions']):
            coverage.append(dict(robot_id=r,region=region,display_points=spec['points'] if region=='ALL' else None,
                RGB_colored_points=0,semantic_labeled_points=0,unknown_fraction=1.0,
                status='BLOCKED_UNVERIFIED_OPTICAL_EXTRINSIC',regional_point_count='NOT_COMPUTED_UNVALIDATED_PROJECTION'))
    write_csv(A/'color_coverage_by_region.csv',coverage)
    projection=dict(status='BLOCKED',real_LiDAR_RGB_projection_performed=False,reason=audit['status'],unobserved_policy='all 3D points UNKNOWN; grey in RGB/semantic mode',
        synthetic_implementation_tests=tests(),real_projection_accuracy='NOT_EVALUATED',
        implemented_rules=['positive depth and image FoV','plumb_bob distortion via OpenCV','nearest-depth exact-pixel z-buffer','reject >50ms bag sync; not exposure guarantee','no inferred occluded-surface colors'],
        duplicate_observations='no real observations merged while calibration blocked; original P5 voxel representatives retained unchanged',
        remaining_validation='camera exposure alignment, occlusion/edge overlay, multi-observation fusion/conflicts, segmentation domain shift')
    dump(A/'rgb_projection_validation.json',projection)
    seg=json.loads((A/'segmentation_runtime.json').read_text());taxonomy=json.loads((A/'semantic_model_policy.json').read_text())
    dump(SITE/'data.json',dict(reference='frozen Stage22B P5 incremental',maps=maps,trajectory=state['trajectory'],loops=state['loops'],
        bounds=old['bounds'],categories=['unknown','building','road/asphalt','grass','vegetation','vehicle','dirt/other'],
        samples=seg['samples'],status='GEOMETRY_READY • RGB / SEMANTIC 3D PROJECTION BLOCKED',
        reason='Missing verified camera_link → color optical frame mounting transform. All 3D attributes remain UNKNOWN; images below are actual 2D model predictions only.',
        semantics='SegFormer B0 ADE20K, confidence >=0.5; no semantic GT or accuracy claim',
        metric_coordinate_policy='Original metric XYZ and P5 poses, no flattening/GT alignment; gauge axes are not geographic ENU',
        colors=dict(robot1='#39bbff',robot3='#ffa43a',unknown='#77838f'),taxonomy=taxonomy))
    scan_manifest=json.loads((BASE/'23d_policy_map_compare/comparison_input_manifest.json').read_text())['selected_scans']
    trajectory=BASE/'22b_budget_aware/policies/P5_BUDGET_AWARE_C75_B80/online_incremental_trajectory.csv'
    for p in sorted(SITE.rglob('*')):
        if p.is_file():artifacts.append(dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=sha(p)))
    dump(A/'map_asset_manifest.json',dict(reference='P5_BUDGET_AWARE_C75_B80 incremental',trajectory_path=str(trajectory),trajectory_sha256=sha(trajectory),
        original_selected_scans=scan_manifest,source_scan_count=len(scan_manifest),frame_stride=10,point_stride=16,voxel_m=.5,range_m=[1,60],
        display_sampling='exact existing capped25000/source P5 buffers, SHA-identical to frozen23D export',artifacts=artifacts,
        real_RGB_colored_vertices=0,real_semantic_3D_vertices=0,mesh_reconstruction=False,GT_used=False))
    print('AUDIT_AND_PACKAGE_COMPLETE',flush=True)

if __name__=='__main__':run()
