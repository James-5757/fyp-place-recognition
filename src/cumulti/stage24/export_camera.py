"""Camera-only timeline and bounded real-frame export; no LiDAR keyframes."""
import argparse, shutil, sqlite3, tempfile, time, zipfile, resource
import cv2, yaml
from rosbags.typesys import Stores,get_typestore
from common import *

def run():
    B.mkdir(parents=True,exist_ok=True)
    if (B/'camera_data_manifest.json').exists():raise RuntimeError('Refuse to overwrite completed export')
    policy=dict(version=1,robots=['robot1','robot3'],keyframe_interval_ns=1000000000,
        pilot_windows_camera_elapsed_s=[[0,12],[120,132],[240,252]],pilot_interval_ns=200000000,
        resize_width=640,jpeg_quality=85,selection='first RGB bag message at/after each independent camera-time tick; ties original ID',
        scene_or_GT_selection=False,LiDAR_keyframe_input=False,max_exported_frames_per_robot=200)
    dump(B/'camera_sampling_policy.json',policy)
    opens=[];camera_guard(opens);store=get_typestore(Stores.ROS2_HUMBLE);reports={};kfrows=[];sync=[]
    for robot in policy['robots']:
        start=time.monotonic();archive=RAW/'main_campus'/robot/f'{robot}_main_campus_camera_rgb.zip'
        before=archive.stat();print('EXPANDING_CAMERA_SQLITE',robot,flush=True)
        with zipfile.ZipFile(archive) as z,tempfile.TemporaryDirectory(prefix='camera_',dir=B) as temp:
            meta=yaml.safe_load(z.read(next(n for n in z.namelist() if n.endswith('metadata.yaml'))))['rosbag2_bagfile_information']
            member=next(n for n in z.namelist() if n.endswith('.db3'));path=Path(temp)/'camera.db3'
            with z.open(member) as source,path.open('wb') as target:shutil.copyfileobj(source,target,32*1024**2)
            # EOF validates ZIP member CRC; SQLite opening follows the complete copy.
            c=sqlite3.connect(f'file:{path}?mode=ro&immutable=1',uri=True)
            topics=c.execute('select id,name,type from topics').fetchall()
            tid,topic,typ=next(t for t in topics if t[1].endswith('/color/image_raw'))
            index=c.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall()
            ts=np.array([t for _,t in index],dtype=np.int64);assert np.all(np.diff(ts)>0)
            cameraid,ctopic,ctype=next(t for t in topics if t[1].endswith('/color/camera_info'))
            raw=c.execute('select data from messages where topic_id=? order by id limit 1',(cameraid,)).fetchone()[0]
            info=store.deserialize_cdr(raw,ctype)
            intrinsic=dict(topic=ctopic,width=int(info.width),height=int(info.height),K=info.k.tolist(),D=info.d.tolist(),
                R=info.r.tolist(),P=info.p.tolist(),distortion_model=info.distortion_model,frame_id=info.header.frame_id,
                header_ns=str(int(info.header.stamp.sec)*10**9+int(info.header.stamp.nanosec)))
            selected=[]
            for t in range(int(ts[0]),int(ts[-1])+1,policy['keyframe_interval_ns']):
                j=min(int(np.searchsorted(ts,t)),len(ts)-1)
                if selected and selected[-1]==j:continue
                selected.append(j);kfrows.append(dict(robot_id=robot,visual_keyframe_id=len(selected)-1,
                    rgb_message_id=index[j][0],rgb_message_index=j,camera_bag_timestamp_ns=str(int(ts[j])),
                    camera_elapsed_s=float((int(ts[j])-int(ts[0]))/1e9),basis='RGB_ONLY_1_HZ'))
            pilot=set()
            for lo,hi in policy['pilot_windows_camera_elapsed_s']:
                for dt in range(lo*10**9,hi*10**9,policy['pilot_interval_ns']):
                    pilot.add(min(int(np.searchsorted(ts,int(ts[0])+dt)),len(ts)-1))
            assert len(pilot)<=policy['max_exported_frames_per_robot'];frames=[];image_info=None
            for j in sorted(pilot):
                mid,bag=index[j];raw=c.execute('select data from messages where id=?',(mid,)).fetchone()[0]
                m=store.deserialize_cdr(raw,typ);im=decode_image(m);head=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
                image_info=dict(encoding=m.encoding,width=int(m.width),height=int(m.height),step=int(m.step),frame_id=m.header.frame_id)
                scaled=cv2.resize(im,(640,round(im.shape[0]*640/im.shape[1])),interpolation=cv2.INTER_AREA)
                dest=B/'camera_frames'/robot/f'{j:06d}.jpg';dest.parent.mkdir(parents=True,exist_ok=True)
                ok,enc=cv2.imencode('.jpg',scaled[:,:,::-1],[cv2.IMWRITE_JPEG_QUALITY,85]);assert ok;dest.write_bytes(enc)
                assert cv2.imread(str(dest)).shape==scaled.shape
                frames.append(dict(robot_id=robot,file=str(dest.relative_to(ROOT)),rgb_message_id=mid,rgb_message_index=j,
                    camera_bag_timestamp_ns=str(bag),camera_header_timestamp_ns=str(head),elapsed_s=(bag-int(ts[0]))/1e9,
                    width=scaled.shape[1],height=scaled.shape[0],sha256=sha(dest),bytes=dest.stat().st_size))
                sync.append(dict(robot_id=robot,rgb_message_id=mid,camera_bag_timestamp_ns=str(bag),camera_header_timestamp_ns=str(head),
                    header_minus_bag_ms=(head-bag)/1e6,depth_association_status='NO_DEPTH_IMAGE_TIMELINE_EXPORTED',LiDAR_association='NOT_USED'))
            write_csv(B/f'{robot}_camera_timestamp_index.csv',[dict(rgb_message_id=m,camera_bag_timestamp_ns=str(t)) for m,t in index])
            c.close()
        assert archive.stat().st_size==before.st_size and archive.stat().st_mtime_ns==before.st_mtime_ns
        reports[robot]=dict(archive=str(archive),archive_size=before.st_size,archive_mtime_ns=before.st_mtime_ns,
            db_member=member,db_member_CRC_validated=True,topics=topics,metadata=meta,camera_info=intrinsic,image=image_info,
            original_frames=len(index),camera_interval_ms=stats(np.diff(ts)/1e6),pilot_frames=frames,
            elapsed_s=time.monotonic()-start,export_bytes=sum(f['bytes'] for f in frames))
        print('CAMERA_EXPORTED',robot,len(frames),'seconds',time.monotonic()-start,flush=True)
    write_csv(B/'visual_keyframe_manifest.csv',kfrows);write_csv(B/'camera_time_sync_audit.csv',sync)
    dump(B/'camera_data_manifest.json',dict(robots=reports,GT_read=False,LiDAR_read=False,SC_read=False,
        policy=policy,peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024))
    dump(B/'camera_export_access_audit.json',dict(guard='OS Python audit hook denies GT,LiDAR caches,SC,old backend input files',opens=opens))

if __name__=='__main__':run()
