#!/usr/bin/env python3
"""Read-only RGB presentation export. No GT, retrieval, registration or solver.

Prototype must run and approve a <=120 MiB projection before --mode full.
SQLite needs one temporary expanded member; raw ZIPs are never modified.
"""
import argparse, csv, hashlib, json, sqlite3, sys, tempfile, time, zipfile
from pathlib import Path
import cv2
import numpy as np
import pandas as pd
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'cumulti'))
from prepare_stage1_validation import TYPESTORE, image_bgr

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'outputs/cumulti_v1/19_rgb_three_map'
SITE = ROOT / 'demo/stage19_rgb_three_map'
RAW = Path('/home/cas/CU-Multi/raw/main_campus')
PROC = Path('/home/cas/CU-Multi/processed_v1/full_2hz')

def dump(p, x):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(x, indent=2, allow_nan=False) + '\n')

def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(8*1024*1024), b''): h.update(b)
    return h.hexdigest()

def nearest(ts, value):
    i = int(np.searchsorted(ts, value))
    return min((max(0, i-1), min(len(ts)-1, i)), key=lambda j: (abs(int(ts[j])-value), j))

def run(mode):
    OUT.mkdir(parents=True, exist_ok=True)
    replay = json.loads((ROOT/'demo/progressive_multi_robot/data/replay.json').read_text())
    if mode == 'full':
        gate = json.loads((OUT/'rgb_prototype.json').read_text())
        assert gate['projected_bytes'] < 120*1024**2 and gate['all_decoded']
    report = {}; all_rows = {}
    for robot, start, end in [('robot1',150,1999), ('robot3',234,4179)]:
        archive = RAW/robot/f'{robot}_main_campus_camera_rgb.zip'
        k = pd.read_csv(PROC/robot/'keyframes.csv', usecols=['keyframe_id','lidar_timestamp_ns'])
        k = k[k.keyframe_id.between(start,end)]
        lts = k.lidar_timestamp_ns.to_numpy(np.int64)
        destination = SITE/'rgb'/robot if mode == 'full' else OUT/'prototype'/robot
        destination.mkdir(parents=True, exist_ok=True)
        began = time.perf_counter()
        with zipfile.ZipFile(archive) as z, tempfile.TemporaryDirectory(prefix='stage19_', dir=OUT) as work:
            meta_name = next(n for n in z.namelist() if n.endswith('metadata.yaml'))
            meta = yaml.safe_load(z.read(meta_name))['rosbag2_bagfile_information']
            members = [n for n in z.namelist() if n.endswith('.db3')]
            assert len(members) == 1, members
            member = members[0]
            db = Path(work)/'rgb.db3'
            # Opening to EOF validates this member's CRC; temporary data never enter raw/.
            with z.open(member) as src, db.open('wb') as target:
                import shutil
                shutil.copyfileobj(src,target,32*1024*1024)
            con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
            topics = con.execute('select id,name,type from topics').fetchall()
            matches = [x for x in topics if x[1].endswith('/camera/color/image_raw')]
            assert len(matches)==1, topics
            tid, topic, typ = matches[0]
            index = con.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall()
            assert len(index)==next(x['message_count'] for x in meta['topics_with_message_count'] if x['topic_metadata']['name']==topic)
            ts = np.array([x[1] for x in index],dtype=np.int64)
            assert np.all(np.diff(ts)>0)
            nearest_all = {int(kid):nearest(ts,int(t)) for kid,t in zip(k.keyframe_id,lts)}
            causal_all = {int(kid):max(0,int(np.searchsorted(ts,int(t),side='right'))-1) for kid,t in zip(k.keyframe_id,lts)}
            if robot == 'robot1':
                # Authentic intermediate frames at <=5 Hz; keyframe-event images remain explicit.
                stream=[]; previous=-10**30
                for j,t in enumerate(ts):
                    if lts[0]<=t<=lts[-1] and int(t)-previous>=190_000_000:
                        stream.append(j); previous=int(t)
                selected=set(stream)|set(nearest_all.values())|set(causal_all.values())
            else:
                stream=[]
                selected={nearest_all[int(e['candidate'])] for e in replay['events']}
                selected.add(nearest_all[start])
            full_count=len(selected)
            if mode=='prototype':
                ordered=sorted(selected)
                selected=set(ordered[i] for i in np.linspace(0,len(ordered)-1,min(100,len(ordered)),dtype=int))
            rows=[]; camera_info=[]; first=None
            for j in sorted(selected):
                message_id,bag_ns=index[j]
                raw=con.execute('select data from messages where id=?',(message_id,)).fetchone()[0]
                msg=TYPESTORE.deserialize_cdr(raw,typ)
                header_ns=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
                image=image_bgr(msg)
                if first is None:
                    first=dict(encoding=msg.encoding,width=int(msg.width),height=int(msg.height),step=int(msg.step),frame_id=msg.header.frame_id,is_bigendian=int(msg.is_bigendian))
                image=cv2.resize(image,(384,round(image.shape[0]*384/image.shape[1])),interpolation=cv2.INTER_AREA)
                ok,buf=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,50]); assert ok
                file=f'{j:06d}_{bag_ns}.jpg'; (destination/file).write_bytes(buf.tobytes())
                decoded=cv2.imdecode(buf,cv2.IMREAD_COLOR); assert decoded.shape==image.shape
                q=nearest(lts,int(bag_ns)); row=dict(file=f'rgb/{robot}/{file}',rgb_message_index=j,message_id=message_id,rgb_timestamp_ns=str(bag_ns),header_timestamp_ns=str(header_ns),keyframe_id=int(k.keyframe_id.iloc[q]),lidar_timestamp_ns=str(int(lts[q])),abs_sync_error_ns=str(abs(int(bag_ns)-int(lts[q]))),header_bag_offset_ns=str(header_ns-int(bag_ns)),size_bytes=len(buf),width=image.shape[1],height=image.shape[0],sha256=hashlib.sha256(buf).hexdigest(),source_archive=str(archive))
                rows.append(row)
            for info_id,info_topic,info_type in topics:
                if info_topic.endswith('/camera/color/camera_info'):
                    raw=con.execute('select data from messages where topic_id=? order by timestamp limit 1',(info_id,)).fetchone()
                    if raw:
                        info=TYPESTORE.deserialize_cdr(raw[0],info_type)
                        camera_info.append(dict(topic=info_topic,width=int(info.width),height=int(info.height),distortion_model=info.distortion_model,K=info.k.tolist(),D=info.d.tolist(),frame_id=info.header.frame_id))
            con.close()
        offsets=np.array([int(r['header_bag_offset_ns']) for r in rows])/1e6
        key_errors=np.array([abs(int(ts[j])-int(t)) for t,j in zip(lts,nearest_all.values())])/1e6
        report[robot]=dict(archive=str(archive),archive_size=archive.stat().st_size,archive_mtime_ns=archive.stat().st_mtime_ns,db_member=member,db_crc_validated=True,metadata=meta,topics=topics,image=first,camera_info=camera_info,recorded_count=len(ts),recorded_rate_hz=1e9/float(np.median(np.diff(ts))),exported_count=len(rows),full_projected_count=full_count,total_bytes=sum(r['size_bytes'] for r in rows),seconds=time.perf_counter()-began,header_bag_offset_ms=dict(median=float(np.median(offsets)),max_abs=float(abs(offsets).max())),keyframe_nearest_rgb_error_ms=dict(median=float(np.median(key_errors)),p95=float(np.percentile(key_errors,95)),max=float(key_errors.max())),stream_indices=stream if mode=='full' else [],keyframe_rgb_indices={str(kid):j for kid,j in nearest_all.items()} if mode=='full' else {})
        report[robot]['header_bag_offset_ms'].update(min=float(offsets.min()),max=float(offsets.max()))
        report[robot]['exported_rate_hz']=(len(rows)-1)*1e9/(int(rows[-1]['rgb_timestamp_ns'])-int(rows[0]['rgb_timestamp_ns']))
        all_rows[robot]=rows
        print(robot,mode,len(rows),'images',report[robot]['total_bytes'],'bytes',flush=True)
    result=dict(mode=mode,robots=report,all_decoded=True,GT_used=False,width=384,jpeg_quality=50,recorded_rgb_rate='measured separately; never inferred from LiDAR 2 Hz',projected_bytes=sum(v['total_bytes']/v['exported_count']*v['full_projected_count'] for v in report.values()))
    dump(OUT/f'rgb_{mode}.json',result)
    if mode=='full':
        manifest=dict(policy='bag-clock nearest keyframe, ties earlier; R1 playback causal floor (never future RGB); event comparisons floor RGB at event LiDAR time',robots=report,frames=all_rows)
        dump(SITE/'rgb_manifest.json',manifest)
        for robot,rows in all_rows.items():
            with (OUT/f'{robot}_rgb_manifest.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    print(json.dumps({k:v for k,v in result.items() if k!='robots'}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['prototype','full'],required=True)
    run(p.parse_args().mode)
