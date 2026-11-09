"""Native-rate 120s camera-only cohort; immutable source and short demo."""
import argparse,fcntl,importlib.util,json,os,shutil,sqlite3,sys,time,zipfile
from pathlib import Path
import cv2,numpy as np
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('rgbd_short_projection',ROOT/'src/cumulti/rgbd_demo/export.py')
short=importlib.util.module_from_spec(spec);spec.loader.exec_module(short)
align_depth,guard=short.align_depth,short.guard
OUT=ROOT/'outputs/cumulti_v1/24_rgbd_long_v1'
RAW=Path('/home/cas/CU-Multi/raw/main_campus')

def dump(path,data):
    path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix(path.suffix+f'.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');tmp.replace(path)

def policy():
    value=dict(window_camera_elapsed_s=[120,240],sampling='every original color frame, native nominal10Hz; preserve dropped frames and timestamp gaps',
        resized_image=[640,400],range_m=[.3,10],depth='nearest camera depth bag timestamp <=25ms; otherwise explicit zero/missing depth, RGB frame retained',
        native_backend='existing unchanged ORB-SLAM3 RGBD, original-timestamp pacing',
        settings=dict(virtual_RGBD_baseline_m=.05,close_far_multiplier=40,depth_PNG_factor=1000,scaleFactor=1.2,nLevels=8,FAST=[20,7]),
        configurations=[dict(name='baseline1000',ORB_features=1000),dict(name='fallback2000',ORB_features=2000,run_if='either baseline robot fails; run both robots without changing frames')],
        selection='first shared configuration passing on BOTH robots; no GT',
        fusion_gate=dict(valid_pose_fraction_min=.95,time_weighted_tracking_fraction_min=.95,longest_continuous_tracking_s_min=90,
            actual_coordinate_frame_changes_max=0,max_untracked_gap_s=1.,finite_unit_poses=True,forbidden_native_reads=0),
        reset_policy='map ID + initial-keyframe ID fingerprint; track gaps separately; never connect reset gauges',
        fusion='only after both robots pass the shared configuration; new camera-only candidates and RGB-D geometry, no historical LiDAR loops',
        GT=False,LiDAR=False,IMU=False,semantics=False,core_retrieval_or_GICP_changed=False,
        historic_checkpoint='88dfda9c5544b1e2e7f5d1e6c02619094f6f4ebe',full_sequence_run=False)
    path=OUT/'policy.json'
    if path.exists():assert json.loads(path.read_text())==value
    else:dump(path,value)
    return value

def expand(archive,out,phase):
    work=out/'work';work.mkdir(exist_ok=True);db=work/(phase+'.db3');receipt=work/(phase+'_receipt.json')
    start=time.monotonic()
    with zipfile.ZipFile(archive) as z:
        member=next(n for n in z.namelist() if n.endswith('.db3'));info=z.getinfo(member)
        if receipt.exists() and db.exists() and db.stat().st_size==info.file_size:return db,json.loads(receipt.read_text())
        # Partial DBs are never queried; a failed expansion restarts from ZIP EOF validation.
        reusable_space=db.stat().st_size if db.exists() else 0
        if shutil.disk_usage(out).free+reusable_space<info.file_size+20*1024**3:raise RuntimeError('Insufficient safe temporary disk headroom')
        done=0;last=0
        with z.open(member) as src,db.open('wb') as dest:
            while True:
                chunk=src.read(32*1024**2)
                if not chunk:break
                dest.write(chunk);done+=len(chunk)
                if done-last>=5*1024**3:
                    dump(out/'progress.json',dict(phase='EXPAND_'+phase,bytes=done,total=info.file_size,elapsed_s=time.monotonic()-start));last=done
        value=dict(archive=str(archive),archive_bytes=archive.stat().st_size,archive_mtime_ns=archive.stat().st_mtime_ns,
            member=member,bytes=done,CRC32=f'{info.CRC:08x}',CRC_validated_at_EOF=True,expansion_s=time.monotonic()-start)
        dump(receipt,value);return db,value

def connect(db,store):
    c=sqlite3.connect(f'file:{db}?mode=ro&immutable=1',uri=True)
    for typ,encoding,definition in c.execute('select topic_type,encoding,encoded_message_definition from message_definitions'):
        if encoding=='ros2msg':store.register(get_types_from_msg(definition,typ))
    return c,c.execute('select id,name,type from topics').fetchall()

def run(robot):
    sys.addaudithook(guard);cv2.setNumThreads(1);p=policy();out=OUT/robot;out.mkdir(exist_ok=True)
    lock=(out/'.export.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if (out/'export.json').exists():print('EXPORT_REUSED',robot,flush=True);return
    store=get_typestore(Stores.ROS2_HUMBLE);rgb_manifest=out/'rgb_export.json'
    if not rgb_manifest.exists():
        archive=RAW/robot/f'{robot}_main_campus_camera_rgb.zip';db,receipt=expand(archive,out,'RGB')
        c,topics=connect(db,store)
        tid,topic,typ=next(t for t in topics if t[1].endswith('/color/image_raw'))
        index=c.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall()
        origin=index[0][1];chosen=[(j,mid,t) for j,(mid,t) in enumerate(index) if origin+120*10**9<=t<origin+240*10**9]
        ci_id,ci_topic,ci_type=next(t for t in topics if t[1].endswith('/color/camera_info'))
        ci=store.deserialize_cdr(c.execute('select data from messages where topic_id=? order by id limit 1',(ci_id,)).fetchone()[0],ci_type)
        info=dict(topic=ci_topic,K=ci.k.tolist(),D=ci.d.tolist(),width=int(ci.width),height=int(ci.height),frame=ci.header.frame_id,distortion_model=ci.distortion_model)
        frames=[];(out/'frames').mkdir(exist_ok=True)
        for i,(j,mid,t) in enumerate(chosen):
            msg=store.deserialize_cdr(c.execute('select data from messages where id=?',(mid,)).fetchone()[0],typ)
            assert msg.encoding in ('rgb8','bgr8')
            a=np.frombuffer(msg.data,np.uint8).reshape(int(msg.height),int(msg.step))[:,:int(msg.width)*3].reshape(int(msg.height),int(msg.width),3)
            if msg.encoding=='rgb8':a=a[:,:,::-1]
            a=cv2.resize(a,(640,400),interpolation=cv2.INTER_AREA)
            path=out/'frames'/f'{i:05d}.jpg';assert cv2.imwrite(str(path),a,[cv2.IMWRITE_JPEG_QUALITY,85])
            frames.append(dict(index=i,original_RGB_index=j,rgb_message_id=mid,rgb_bag_ns=str(t),
                rgb_header_ns=str(int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)),
                elapsed_camera_s=(t-origin)/1e9,rgb=str(path.relative_to(ROOT))))
        c.close();dump(rgb_manifest,dict(robot=robot,archive=receipt,topic=topic,camera_info=info,frames=frames,original_frames=len(index),origin_ns=str(origin)))
        db.unlink();print('RGB_EXPORTED',robot,len(frames),flush=True)
    rgb=json.loads(rgb_manifest.read_text());frames=rgb['frames'];assert 1100<=len(frames)<=1250,len(frames)
    archive=RAW/robot/f'{robot}_main_campus_camera_depth.zip';db,receipt=expand(archive,out,'DEPTH');c,topics=connect(db,store)
    def first(suffix):
        tid,name,typ=next(t for t in topics if t[1].endswith(suffix))
        return store.deserialize_cdr(c.execute('select data from messages where topic_id=? order by id limit 1',(tid,)).fetchone()[0],typ)
    ext=first('depth_to_color');ci=first('/depth/camera_info');R=ext.rotation.reshape(3,3,order='F');t=ext.translation
    assert np.max(abs(R.T@R-np.eye(3)))<1e-6
    K=np.array(rgb['camera_info']['K']).reshape(3,3);K[:2]*=.5;D=np.array(rgb['camera_info']['D'])
    tid,topic,typ=next(t for t in topics if t[1].endswith('/depth/color/points'))
    di=c.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall();ts=np.array([r[1] for r in di],np.int64)
    cache={};rows=[]
    for i,f in enumerate(frames):
        stamp=int(f['rgb_bag_ns']);j=int(np.searchsorted(ts,stamp));j=min((k for k in [j-1,j] if 0<=k<len(ts)),key=lambda k:(abs(int(ts[k])-stamp),k))
        mid,dt=di[j];sync=abs(dt-stamp)/1e6;header=None
        if sync<=25:
            if mid not in cache:
                m=store.deserialize_cdr(c.execute('select data from messages where id=?',(mid,)).fetchone()[0],typ)
                assert m.header.frame_id==ci.header.frame_id
                fields={a.name:a for a in m.fields};assert all(fields[n].datatype==7 for n in ['x','y','z'])
                endian='>' if m.is_bigendian else '<'
                dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,offsets=[fields[n].offset for n in ['x','y','z']],itemsize=int(m.point_step)))
                cloud=np.ndarray((int(m.height),int(m.width)),dtype=dtype,buffer=m.data,strides=(int(m.row_step),int(m.point_step)))
                xyz=np.stack([cloud[n].ravel() for n in ['x','y','z']],1)
                depth=align_depth(xyz,R,t,K,D);header=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
                saturation=float(np.mean(xyz[:,2]>=65.534));cache={mid:(depth,header,saturation)}
            depth,header,saturation=cache[mid];status='PAIRED'
        else:depth=np.zeros((400,640),np.float32);saturation=None;status='MISSING_DEPTH_SYNC_GATE'
        path=out/'frames'/f'{i:05d}_depth.png';assert cv2.imwrite(str(path),np.rint(depth*1000).astype(np.uint16))
        rows.append(dict(**f,depth=str(path.relative_to(ROOT)),depth_message_id=mid,depth_bag_ns=str(dt),
            depth_header_ns=str(header) if header is not None else None,bag_sync_ms=sync,
            header_sync_ms=abs(int(f['rgb_header_ns'])-header)/1e6 if header is not None else None,
            depth_status=status,depth_coverage=float(np.mean(depth>0)),saturated_source_fraction=saturation))
        if i%100==0:dump(out/'progress.json',dict(phase='REGISTER_DEPTH',frames=i+1,total=len(frames)))
    c.close()
    result=dict(robot=robot,frames=rows,camera_info=rgb['camera_info'],scaled_K=K.tolist(),RGB_archive=rgb['archive'],depth_archive=receipt,
        RGB_topic=rgb['topic'],depth_topic=topic,depth_frame=ci.header.frame_id,
        T_color_from_depth=dict(R=R.tolist(),t_m=t.tolist(),rotation_storage='column-major'),policy=p,
        depth_camera_info=dict(K=ci.k.tolist(),D=ci.d.tolist(),width=int(ci.width),height=int(ci.height)),
        GT=False,LiDAR=False,IMU=False)
    dump(out/'export.json',result);db.unlink();dump(out/'progress.json',dict(phase='EXPORTED',frames=len(rows),temporary_DB_removed=True))
    print('DEPTH_EXPORTED',robot,len(rows),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('robot',choices=['robot1','robot3']);run(p.parse_args().robot)
