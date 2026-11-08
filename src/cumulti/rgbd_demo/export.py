"""Bounded camera-only RGB-D export; raw archives and historical outputs are read-only."""
import argparse, fcntl, json, os, shutil, sqlite3, sys, time, zipfile
from pathlib import Path
import cv2
import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'outputs/cumulti_v1/24_rgbd_demo_v1'
OLD = ROOT / 'outputs/cumulti_v1/24b_vision_feasibility'
RAW = Path('/home/cas/CU-Multi/raw/main_campus')

def dump(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')

def guard(event, args):
    if event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
        name = str(args[0]).lower()
        if name.endswith(('.py', '.pyc')): return
        if any(s in name for s in ('_lidar.zip','gt_utm','gt_rel','imu_gps','/full_2hz/','/22b_','/12d_','clean_local_trajectory')):
            raise PermissionError('Camera-only RGB-D input guard: '+name)

def align_depth(xyz, R, t, K, D, width=640, height=400):
    """Project real depth-optical XYZ to distorted color pixels; nearest Z, no fill."""
    valid = np.isfinite(xyz).all(1) & (xyz[:,2] >= .3) & (xyz[:,2] <= 10.)
    p = xyz[valid] @ R.T + t
    p = p[np.isfinite(p).all(1) & (p[:,2] >= .3) & (p[:,2] <= 10.)]
    depth = np.full(width*height, np.inf, np.float32)
    if len(p):
        uv = cv2.projectPoints(p, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1,2)
        uv = np.floor(uv+.5).astype(np.int64)
        ok = (uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
        np.minimum.at(depth, uv[ok,1]*width+uv[ok,0], p[ok,2])
    depth[~np.isfinite(depth)] = 0
    return depth.reshape(height,width)

def run(robot):
    sys.addaudithook(guard)
    out = OUT/robot
    out.mkdir(parents=True,exist_ok=True)
    lock=(out/'.export.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if (out/'export.json').exists():
        print('EXPORT_ALREADY_COMPLETE',robot,flush=True); return
    # Reuse exactly the fixed non-GT camera windows from the accepted camera pilot.
    manifest = json.loads((OLD/'camera_data_manifest.json').read_text())['robots'][robot]
    frames = [f for f in manifest['pilot_frames'] if 120 <= f['elapsed_s'] < 132.15]
    assert len(frames)==60
    policy = dict(window_camera_elapsed_s=[120,132],frames=60,sampling_hz=5,
        image_size=[640,400],range_m=[.3,10.],depth_unit='metres float32; uint16 PNG millimetres',
        occlusion='nearest color-camera Z per rounded pixel',hole_filling=False,
        association='nearest camera-depth bag timestamp; tie by ordered message ID',max_bag_sync_ms=25,
        hardware_exposure_clock_verified=False,LiDAR=False,GT=False,IMU=False,
        selection='fixed Stage24 pilot window; no GT or scene-dependent selection')
    dump(out/'policy.json',policy)
    archive=RAW/robot/f'{robot}_main_campus_camera_depth.zip'
    before=archive.stat(); store=get_typestore(Stores.ROS2_HUMBLE)
    work=out/'work';work.mkdir(exist_ok=True); db=work/'camera_depth.db3'
    start=time.monotonic()
    with zipfile.ZipFile(archive) as z:
        member=next(n for n in z.namelist() if n.endswith('.db3'))
        info=z.getinfo(member)
        receipt=work/'expansion_receipt.json'
        if not (receipt.exists() and db.exists() and db.stat().st_size==info.file_size):
            print('EXPANDING',robot,info.file_size,flush=True)
            done=0;last=0
            with z.open(member) as src,db.open('wb') as dest:
                while True:
                    chunk=src.read(32*1024**2)
                    if not chunk:break
                    dest.write(chunk);done+=len(chunk)
                    if done-last>5*1024**3:
                        dump(out/'progress.json',dict(phase='EXPANDING',bytes=done,total=info.file_size,elapsed_s=time.monotonic()-start));last=done
            dump(receipt,dict(CRC_validated_at_EOF=True,bytes=done,archive=str(archive),member=member,CRC32=f'{info.CRC:08x}'))
    c=sqlite3.connect(f'file:{db}?mode=ro&immutable=1',uri=True)
    for typ,encoding,definition in c.execute('select topic_type,encoding,encoded_message_definition from message_definitions'):
        if encoding=='ros2msg':store.register(get_types_from_msg(definition,typ))
    topics=c.execute('select id,name,type from topics').fetchall()
    def first(suffix):
        tid,name,typ=next(t for t in topics if t[1].endswith(suffix))
        row=c.execute('select timestamp,data from messages where topic_id=? order by id limit 1',(tid,)).fetchone()
        return name,store.deserialize_cdr(row[1],typ)
    _,ext=first('depth_to_color'); R=ext.rotation.reshape(3,3,order='F');t=ext.translation
    assert np.max(abs(R.T@R-np.eye(3)))<1e-6 and abs(np.linalg.det(R)-1)<1e-6
    _,ci=first('/depth/camera_info')
    tid,topic,typ=next(t for t in topics if t[1].endswith('/depth/color/points'))
    index=c.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall()
    ts=np.array([v[1] for v in index],dtype=np.int64)
    camera=manifest['camera_info'];K=np.array(camera['K']).reshape(3,3);K[:2]*=.5;D=np.array(camera['D'])
    rows=[]
    for i,frame in enumerate(frames):
        stamp=int(frame['camera_bag_timestamp_ns']); j=int(np.searchsorted(ts,stamp))
        j=min((k for k in (j-1,j) if 0<=k<len(ts)),key=lambda k:(abs(int(ts[k])-stamp),k))
        mid,dt=index[j];sync=abs(dt-stamp)/1e6
        assert sync<=25,('sync gate',sync)
        m=store.deserialize_cdr(c.execute('select data from messages where id=?',(mid,)).fetchone()[0],typ)
        assert m.header.frame_id==ci.header.frame_id,('cloud/calibration frame mismatch',m.header.frame_id,ci.header.frame_id)
        fields={f.name:f for f in m.fields};endian='>' if m.is_bigendian else '<'
        assert all(fields[n].datatype==7 and fields[n].count==1 for n in ['x','y','z'])
        dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,offsets=[fields[n].offset for n in ['x','y','z']],itemsize=int(m.point_step)))
        a=np.ndarray((int(m.height),int(m.width)),dtype=dtype,buffer=m.data,strides=(int(m.row_step),int(m.point_step)))
        xyz=np.stack([a[n].ravel() for n in ['x','y','z']],1)
        depth=align_depth(xyz,R,t,K,D)
        rgb=cv2.imread(str(ROOT/frame['file']));assert rgb.shape[:2]==depth.shape
        # Lossless RGB copy for this pipeline; JPEG source was already exported at q85.
        rgbpath=out/'frames'/f'{i:03d}.jpg';rgbpath.parent.mkdir(exist_ok=True);shutil.copyfile(ROOT/frame['file'],rgbpath)
        dpath=out/'frames'/f'{i:03d}_depth.png';assert cv2.imwrite(str(dpath),np.rint(depth*1000).astype(np.uint16))
        positive=depth>0
        header=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
        row=dict(frame=i,rgb_original_index=frame['rgb_message_index'],rgb_message_id=frame['rgb_message_id'],
            rgb_bag_ns=str(stamp),rgb_header_ns=frame['camera_header_timestamp_ns'],depth_message_id=mid,
            depth_bag_ns=str(dt),depth_header_ns=str(header),bag_sync_ms=sync,
            header_sync_ms=abs(int(frame['camera_header_timestamp_ns'])-header)/1e6,
            elapsed_s=frame['elapsed_s'],rgb=str(rgbpath.relative_to(ROOT)),depth=str(dpath.relative_to(ROOT)),
            valid_depth_fraction=float(positive.mean()),depth_pixels=int(positive.sum()),
            rejected_saturation_fraction=float(np.mean(xyz[:,2]>=65.534)),source_cloud_points=len(xyz),
            depth_z_median=float(np.median(depth[positive])) if positive.any() else None)
        rows.append(row)
        if i in [0,30,59]:
            # Explicit overlay: valid depth only, no completion/invented pixels.
            heat=cv2.applyColorMap(np.uint8(np.clip(depth/10,0,1)*255),cv2.COLORMAP_TURBO)
            overlay=rgb.copy();overlay[positive]=cv2.addWeighted(rgb,.55,heat,.45,0)[positive]
            cv2.imwrite(str(out/f'alignment_{i:03d}.jpg'),np.hstack([rgb,overlay]))
    c.close()
    result=dict(robot=robot,policy=policy,frames=rows,camera_info=camera,scaled_K=K.tolist(),
        depth_camera_info=dict(K=ci.k.tolist(),D=ci.d.tolist(),width=int(ci.width),height=int(ci.height),frame=ci.header.frame_id),
        T_color_from_depth=dict(R=R.tolist(),t_m=t.tolist(),rotation_storage='column-major RealSense message'),
        input_archive=str(archive),archive_bytes=before.st_size,archive_mtime_ns=before.st_mtime_ns,
        source_topic=topic,source_frame=m.header.frame_id,CRC_validated_at_EOF=True,
        export_s=time.monotonic()-start,LiDAR_used=False,GT_used=False,
        limitations=['bag-time association is not hardware exposure calibration','unorganized cloud reprojection leaves holes','fixed .3-10m engineering gate is not an accuracy specification'])
    assert archive.stat().st_size==before.st_size and archive.stat().st_mtime_ns==before.st_mtime_ns
    dump(out/'export.json',result)
    db.unlink() # Only this new run's known temporary file; raw is unchanged.
    dump(out/'progress.json',dict(phase='EXPORTED',frames=len(rows),temporary_db_removed=True))
    print('EXPORT_COMPLETE',robot,len(rows),'seconds',result['export_s'],flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('robot',choices=['robot1','robot3']);run(parser.parse_args().robot)
