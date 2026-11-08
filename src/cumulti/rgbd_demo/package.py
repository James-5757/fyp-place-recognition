"""RGB-D-only local map replay, using recorded tracked poses, never LiDAR poses."""
import csv, hashlib, json, shutil
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from export import ROOT, OUT, dump

SITE=ROOT/'demo/rgbd_camera_mapping'

def run():
    if (OUT/'map_manifest.json').exists():raise RuntimeError('Refuse to overwrite a completed RGB-D package')
    SITE.mkdir(exist_ok=True)
    specs={};artifacts=[]
    for robot in ['robot1','robot3']:
        out=OUT/robot;data=json.loads((out/'export.json').read_text());tracking=json.loads((out/'paced/tracking.json').read_text())
        poses=list(csv.DictReader((out/'paced/frame_poses.csv').open()))
        asset=SITE/'assets'/robot;asset.mkdir(parents=True,exist_ok=True)
        K=np.array(data['scaled_K']);D=np.array(data['camera_info']['D']);rows=[];submaps={};cap=100000
        # Presentation includes truthful failed segments. The full-window tracking
        # acceptance gate is NOT relaxed. Every post-gap epoch has a separate map.
        backprojection_errors=[]
        for f,p in zip(data['frames'],poses):
            rgb=cv2.imread(str(ROOT/f['rgb']));d=cv2.imread(str(ROOT/f['depth']),cv2.IMREAD_UNCHANGED).astype(np.float32)/1000
            dst=asset/f"{f['frame']:03d}.jpg";shutil.copyfile(ROOT/f['rgb'],dst)
            heat=cv2.applyColorMap(np.uint8(np.clip(d/10,0,1)*255),cv2.COLORMAP_TURBO);heat[d==0]=0
            depthpath=asset/f"{f['frame']:03d}_depth.jpg";assert cv2.imwrite(str(depthpath),heat,[cv2.IMWRITE_JPEG_QUALITY,82])
            good=p['valid_pose']=='1';epoch=int(p['tracking_epoch'])
            sub=submaps.setdefault(epoch,dict(seen=set(),xyz=[],colors=[],total=0,map_id=None))
            pose=None
            if good:
                pose=[float(p[k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]
                mapid=int(p['map_id'])
                if sub['map_id'] is not None:assert sub['map_id']==mapid
                sub['map_id']=mapid
                # Sample every second RGB frame, pixels at stride 4, 5cm voxels.
                if (f['frame']%2==0 or not sub['total']) and sub['total']<cap:
                    v,u=np.mgrid[0:400:4,0:640:4];z=d[v,u];valid=z>0
                    uv=np.stack([u[valid],v[valid]],1).astype(np.float64)
                    normalized=cv2.undistortPoints(uv.reshape(-1,1,2),K,D).reshape(-1,2)
                    local=np.c_[normalized,np.ones(len(normalized))]*z[valid,None]
                    projected=cv2.projectPoints(local,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
                    if len(uv):backprojection_errors.append(float(np.max(np.linalg.norm(projected-uv,axis=1))))
                    world=local@Rotation.from_quat(pose[3:]).as_matrix().T+pose[:3]
                    colors=rgb[v[valid],u[valid],::-1]
                    keep=[]
                    for j,key in enumerate(np.floor(world/.05).astype(np.int64)):
                        key=tuple(key)
                        if key not in sub['seen'] and sub['total']+len(keep)<cap:sub['seen'].add(key);keep.append(j)
                    if keep:sub['xyz'].append(world[keep]);sub['colors'].append(colors[keep]);sub['total']+=len(keep)
            rows.append(dict(index=f['frame'],elapsed_s=(int(f['rgb_bag_ns'])-int(data['frames'][0]['rgb_bag_ns']))/1e9,
                rgb=dst.relative_to(SITE).as_posix(),depth=depthpath.relative_to(SITE).as_posix(),
                rgb_original_index=f['rgb_original_index'],rgb_bag_ns=f['rgb_bag_ns'],rgb_header_ns=f['rgb_header_ns'],
                depth_bag_ns=f['depth_bag_ns'],depth_header_ns=f['depth_header_ns'],
                bag_sync_ms=f['bag_sync_ms'],header_sync_ms=f['header_sync_ms'],valid_depth_fraction=f['valid_depth_fraction'],
                valid_pose=good,tracking_state=int(p['tracking_state']),map_id=int(p['map_id']),tracking_epoch=epoch,
                pose=pose,points_end=sub['total']))
        maps=[]
        for epoch,sub in submaps.items():
            xyz=np.vstack(sub['xyz']).astype('<f4') if sub['xyz'] else np.empty((0,3),'<f4')
            colors=np.vstack(sub['colors']).astype(np.uint8) if sub['colors'] else np.empty((0,3),np.uint8)
            assert len(xyz)==sub['total']==len(colors) and np.isfinite(xyz).all()
            pos=asset/f'epoch{epoch}_xyz.f32';color=asset/f'epoch{epoch}_rgb.u8';xyz.tofile(pos);colors.tofile(color)
            maps.append(dict(epoch=epoch,map_id=sub['map_id'],points=sub['total'],
                positions=pos.relative_to(SITE).as_posix(),colors=color.relative_to(SITE).as_posix(),
                cap_reached=sub['total']==cap))
        specs[robot]=dict(status=tracking['status'],tracking=tracking,frames=rows,points=sum(s['total'] for s in submaps.values()),
            submaps=maps,source_camera_rgb_archive=f'/home/cas/CU-Multi/raw/main_campus/{robot}/{robot}_main_campus_camera_rgb.zip',
            source_camera_depth_archive=data['input_archive'],camera_info=data['camera_info'],scaled_K=data['scaled_K'],
            T_color_from_depth=data['T_color_from_depth'],range_m=[.3,10],voxel_m=.05,
            point_cap_per_submap=cap,pixel_stride=4,map_frame_stride=2,
            max_roundtrip_pixel_error=max(backprojection_errors,default=None),
            bag_sync_ms=dict(median=float(np.median([f['bag_sync_ms'] for f in rows])),p95=float(np.percentile([f['bag_sync_ms'] for f in rows],95))),
            coordinate_frame='first camera optical local gauge; not gravity aligned',
            pose_source='recorded TrackRGBD Twc per frame; not final BA corrected, no pose interpolation')
        for path in asset.iterdir():artifacts.append(dict(path=path.relative_to(ROOT).as_posix(),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    manifest=dict(stage='bounded RGB-D camera mapping prototype',robots=specs,merged_map=False,
        inputs=['original RGB-derived camera JPEGs','original RealSense depth PointCloud2','recorded camera intrinsics and depth_to_color','ORB vocabulary'],
        forbidden_inputs_used=[],GT=False,LiDAR=False,IMU=False,semantics=False,
        common_robot_frame_established=False,inter_robot_visual_loops=0,
        limitations=['12s / 60 frames per robot, not full-run SLAM','independent local maps, no cross-robot fusion',
            'tracking gaps/resets are conservatively separated into independent submap epochs; never merged',
            'recorded tracking poses include local backend updates; earlier dense map samples are not reoptimized',
            'CPU offline processing with traced native inputs; not live playback',
            'sparse camera-depth reprojected to color leaves holes; no hole filling',
            'no GT accuracy evaluation or hardware exposure clock validation'])
    dump(SITE/'data.json',manifest)
    dump(OUT/'map_manifest.json',dict(robots={r:{k:v for k,v in s.items() if k not in ('frames','trajectory_segments')} for r,s in specs.items()},artifacts=artifacts,scientific_policy=manifest['limitations']))
    print('PACKAGED',[(r,v['points'],v['status']) for r,v in specs.items()],flush=True)

if __name__=='__main__':run()
