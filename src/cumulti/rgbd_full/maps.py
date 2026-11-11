"""Segment-aware metric RGB-D maps; no inter-epoch transforms or pose filling."""
import csv,fcntl,json,sys
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from common import *

def build(robot, *, output_root=OUT, asset_root=SITE, reuse_rgb=False):
 sys.addaudithook(guard);cv2.setNumThreads(1);out=output_root/robot;site=asset_root/robot;site.mkdir(parents=True,exist_ok=True)
 raw=OUT/robot
 prefix_path=site.relative_to(SITE).as_posix()

 lock=(out/'map_export.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
 source_sha=sha(Path(__file__));pose_sha=sha(out/'camera_poses.csv')
 if (site/'data.json').exists():
  existing=json.loads((site/'data.json').read_text())
  if existing.get('mapping_source_sha256')==source_sha and existing.get('pose_source_sha256')==pose_sha:return existing
 metrics=json.loads((out/'tracking_metrics.json').read_text());source=json.loads((raw/'RGB_audit.json').read_text());poses=list(csv.DictReader((out/'camera_poses.csv').open()))
 pairs=list(csv.DictReader((raw/'sync_pairs.csv').open()));K=np.array(source['calibration']['K']).reshape(3,3);K[:2]*=.5;D=np.array(source['calibration']['D'])
 yy,xx=np.mgrid[0:400:8,0:640:8];uv=np.stack([xx,yy],-1).astype(np.float32);xy=cv2.undistortPoints(uv.reshape(-1,1,2),K,D).reshape(-1,2);rays=np.column_stack([xy,np.ones(len(xy))])
 frames=[];preview=[];last_preview=-1e9;last_map={};clouds={};counts={};manifest=[]
 for i,(p,s) in enumerate(zip(poses,pairs)):
  t=int(p['relative_ns'])/1e9;good=p['valid_pose']=='1';epoch=int(p['map_epoch']);frame=dict(index=i,t=t,valid=good,epoch=epoch,map_id=int(p['map_id']),map_init_kf=int(p['map_init_kf']),rgb_bag_ns=s['rgb_bag_ns'],rgb_header_ns=s['rgb_header_ns'],paired=s['paired']=='True',sync_ms=float(s['bag_sync_ms']))
  if good:
   frame['pose']=[float(p[k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]
   if frames and frames[-1]['valid'] and frames[-1]['epoch']==epoch:
    previous=frames[-1];distance=float(np.linalg.norm(np.array(frame['pose'][:3])-previous['pose'][:3]));dt=t-previous['t'];angle=float(np.rad2deg((Rotation.from_quat(previous['pose'][3:]).inv()*Rotation.from_quat(frame['pose'][3:])).magnitude()))
    frame['visual_discontinuity']=distance>2 or distance/max(dt,1e-9)>10 or angle>45
  frames.append(frame)
  need_rgb=not reuse_rgb and (t-last_preview>=.5 or i==len(poses)-1)
  need_map=good and t-last_map.get(epoch,-1e9)>=1.
  if not(need_rgb or need_map):continue
  image=cv2.imread(str(ROOT/s['rgb']));assert image is not None
  if need_rgb:
   target=site/'rgb'/f'{i:06d}.jpg';target.parent.mkdir(exist_ok=True);assert cv2.imwrite(str(target),cv2.resize(image,(320,200)),[cv2.IMWRITE_JPEG_QUALITY,70]);preview.append(dict(index=i,t=t,rgb_message_id=s['rgb_message_id'],rgb_bag_ns=s['rgb_bag_ns'],rgb_header_ns=s['rgb_header_ns'],depth_bag_ns=s['depth_bag_ns'],depth_header_ns=s['depth_header_ns'],depth_paired=s['paired']=='True',bag_sync_ms=float(s['bag_sync_ms']),file=f'{robot}/rgb/{i:06d}.jpg',bytes=target.stat().st_size,sha256=sha(target)));last_preview=t
  if not need_map:continue
  last_map[epoch]=t;depth=cv2.imread(str(raw/'depth'/f'{i:06d}.png'),cv2.IMREAD_UNCHANGED);assert depth is not None
  z=depth[yy,xx].ravel()/1000.;valid=(z>=.3)&(z<=10);local=rays[valid]*z[valid,None]
  q=Rotation.from_quat(frame['pose'][3:]).as_matrix();world=local@q.T+np.array(frame['pose'][:3]);colors=image[yy,xx].reshape(-1,3)[valid,::-1]
  voxels=clouds.setdefault(epoch,{});counts[epoch]=counts.get(epoch,0)+1
  for xyz,rgb,key in zip(world,colors,np.floor(world/.1).astype(np.int64)):
   voxels.setdefault(tuple(key),(xyz.astype(np.float32),rgb.copy(),i))
 epochs=[];plydir=out/'maps';plydir.mkdir(exist_ok=True)
 for epoch,voxels in sorted(clouds.items()):
  values=list(voxels.values());xyz=np.array([v[0] for v in values],np.float32).reshape(-1,3);rgb=np.array([v[1] for v in values],np.uint8).reshape(-1,3);first=np.array([v[2] for v in values],np.uint32)
  ply=plydir/f'epoch_{epoch:04d}.ply'
  dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('r','u1'),('g','u1'),('b','u1')]);data=np.empty(len(xyz),dtype=dtype)
  for j,name in enumerate(['x','y','z']):data[name]=xyz[:,j]
  for j,name in enumerate(['r','g','b']):data[name]=rgb[:,j]
  with ply.open('wb') as f:f.write(('ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(data))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode());f.write(data.tobytes())
  # Uniform chronological LOD spans the whole epoch; never stop at an early cap.
  select=np.arange(0,len(xyz),max(1,int(np.ceil(len(xyz)/60000))))
  prefix=site/f'epoch_{epoch:04d}';files={}
  for suffix,array in [('xyz.f32',xyz[select].astype('<f4')),('rgb.u8',rgb[select]),('first.u32',first[select].astype('<u4'))]:
   file=Path(str(prefix)+'.'+suffix);file.write_bytes(array.tobytes());files[suffix]=f'{prefix_path}/{file.name}';manifest.append(dict(path=str(file.relative_to(ROOT)),bytes=file.stat().st_size,sha256=sha(file)))
  epochs.append(dict(epoch=epoch,points=len(xyz),display_points=len(select),map_frames=counts[epoch],files=files,bounds=[xyz.min(0).tolist(),xyz.max(0).tolist()] if len(xyz) else None,gauge='independent native map fingerprint; never glued to another epoch'))
  manifest.append(dict(path=str(ply.relative_to(ROOT)),bytes=ply.stat().st_size,sha256=sha(ply),server_only=True))
 if reuse_rgb:
  preview=json.loads((SITE/robot/'data.json').read_text())['previews']
 result=dict(robot=robot,duration_s=metrics['duration_s'],tracking=metrics,frames=frames,previews=preview,epochs=epochs,mapping_source_sha256=source_sha,pose_source_sha256=pose_sha,
  mapping=dict(map_sampling_hz=1,pixel_stride=8,voxel_m=.1,depth_range_m=[.3,10],inverse_projection='cv2.undistortPoints with actual scaled color K and D',poses='recorded online Twc; previous points not retrospectively BA corrected',gauge='camera optical, not gravity-aligned',inter_epoch_fusion=False),source_archives=[source['archive']['archive'],json.loads((raw/'DEPTH_audit.json').read_text())['archive']['archive']])
 dump(site/'data.json',result);dump(out/'map_artifact_manifest.json',manifest);dump(out/'presentation_rgb_manifest.json',preview)
 return result

def run():
 SITE.mkdir(parents=True,exist_ok=True)
 lock=(OUT/'mapping.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
 if (OUT/'dense_mapping_complete.json').exists() and (SITE/'manifest.json').exists():return
 robots={r:build(r) for r in ['robot1','robot3']}
 dump(SITE/'manifest.json',dict(robots={r:f'{r}/data.json' for r in robots},duration_s=max(v['duration_s'] for v in robots.values()),playback_compression=10,stage20_reference_s=1972.508,physical_simultaneity_claimed=False,global_fusion='NOT_VERIFIED_LOCAL_MAPS_ONLY',tracking_qualification_separate_from_rendering=True))
 dump(OUT/'dense_mapping_complete.json',dict(robots={r:dict(epochs=len(v['epochs']),points=sum(e['points'] for e in v['epochs']),display_points=sum(e['display_points'] for e in v['epochs']),preview_frames=len(v['previews'])) for r,v in robots.items()},inter_epoch_gluing=False))
 print('SEGMENT_AWARE_DENSE_MAPPING_COMPLETE',flush=True)

if __name__=='__main__':run()
