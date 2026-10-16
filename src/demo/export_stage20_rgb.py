#!/usr/bin/env python3
"""Bounded, authentic Robot3 stream; policy fsynced before prototype/full export."""
import os, shutil, sqlite3, tempfile, time, zipfile, hashlib, json
from pathlib import Path
import numpy as np
import pandas as pd
import cv2
from export_stage19_rgb import TYPESTORE, image_bgr, nearest, digest, dump, ROOT
OUT=ROOT/'outputs/cumulti_v1/20_dual_stream'
SITE=ROOT/'demo/stage20_dual_stream'

def refresh_metadata():
    p=SITE/'rgb_manifest.json';v=json.loads(p.read_text());rows=v['frames']['robot3'];info=v['robots']['robot3']
    info['stage19_reference_export_count']=71
    offsets=np.array([(int(r['header_timestamp_ns'])-int(r['rgb_timestamp_ns']))/1e6 for r in rows])
    info.update(exported_count=len(rows),full_projected_count=len(rows),total_bytes=sum(r['size_bytes'] for r in rows),exported_rate_hz=(len(rows)-1)*1e9/(int(rows[-1]['rgb_timestamp_ns'])-int(rows[0]['rgb_timestamp_ns'])),seconds=v['seconds'],header_bag_offset_ms=dict(median=float(np.median(offsets)),min=float(offsets.min()),max=float(offsets.max()),max_abs=float(abs(offsets).max())),export_policy=v['policy'],stream_indices=[r['rgb_message_index'] for r in rows],keyframe_rgb_indices={})
    v['robots']['robot3']=info;dump(p,v)
    p=OUT/'stage20b_rgb_export.json'
    if p.exists():report=json.loads(p.read_text());report['manifest_sha256']=digest(SITE/'rgb_manifest.json');dump(p,report)

def main():
    assert json.loads((OUT/'stage20a_backend_decision.json').read_text())['status']=='DUAL_STREAM_BACKEND_READY'
    policy=dict(max_additional_images=2200,max_additional_bytes=24*1024**2,width=320,jpeg_quality=45,prototype='first 10 seconds, native recorded camera rate',full='causal floor at each 1-second own-clock tick plus loop endpoints and final; no interpolation',fallback='Stage19 sparse authentic references, explicitly sparse; never continuous 10Hz',GT_used=False)
    p=OUT/'stage20b_rgb_policy.json'
    if p.exists():assert json.loads(p.read_text())==policy
    else:
        with p.open('w') as f:json.dump(policy,f,indent=2);f.flush();os.fsync(f.fileno())
    archive=Path('/home/cas/CU-Multi/raw/main_campus/robot3/robot3_main_campus_camera_rgb.zip')
    stat=(archive.stat().st_size,archive.stat().st_mtime_ns)
    k=pd.read_csv('/home/cas/CU-Multi/processed_v1/full_2hz/robot3/keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns'])
    k=k[k.keyframe_id.between(234,4179)]; lts=k.lidar_timestamp_ns.to_numpy(np.int64)
    began=time.perf_counter(); destination=SITE/'rgb/robot3';destination.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(archive) as z,tempfile.TemporaryDirectory(prefix='stage20_rgb_',dir=OUT) as work:
        members=[n for n in z.namelist() if n.endswith('.db3')];assert len(members)==1
        member=members[0];db=Path(work)/'rgb.db3'
        print('Expanding one temporary SQLite member; raw ZIP unchanged',flush=True)
        with z.open(member) as a,db.open('wb') as b:shutil.copyfileobj(a,b,32*1024*1024)
        con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        topics=con.execute('select id,name,type from topics').fetchall()
        tid,topic,typ=next(v for v in topics if v[1].endswith('/camera/color/image_raw'))
        index=con.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall();ts=np.array([v[1] for v in index],np.int64)
        floor=lambda t:max(0,int(np.searchsorted(ts,int(t),side='right'))-1)
        loops=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv')
        selected={floor(t) for t in range(int(lts[0]),int(lts[-1])+1,10**9)}|{floor(lts[-1])}|{floor(int(k.set_index('keyframe_id').loc[int(v)].lidar_timestamp_ns)) for v in loops.candidate_keyframe_id}
        assert len(selected)<=policy['max_additional_images']
        def encode(j,folder):
            mid,t=index[j];raw=con.execute('select data from messages where id=?',(mid,)).fetchone()[0];msg=TYPESTORE.deserialize_cdr(raw,typ)
            image=image_bgr(msg);image=cv2.resize(image,(320,round(image.shape[0]*320/image.shape[1])),interpolation=cv2.INTER_AREA)
            ok,buf=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,45]);assert ok
            assert cv2.imdecode(buf,cv2.IMREAD_COLOR).shape==image.shape
            name=f'{j:06d}_{t}.jpg';(folder/name).write_bytes(buf.tobytes());q=nearest(lts,t)
            return dict(file='rgb/robot3/'+name,rgb_message_index=j,message_id=mid,rgb_timestamp_ns=str(t),header_timestamp_ns=str(int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)),keyframe_id=int(k.keyframe_id.iloc[q]),lidar_timestamp_ns=str(int(lts[q])),abs_sync_error_ns=str(abs(t-int(lts[q]))),size_bytes=len(buf),width=image.shape[1],height=image.shape[0],sha256=hashlib.sha256(buf).hexdigest(),source_archive=str(archive),encoding=str(msg.encoding),frame_id=str(msg.header.frame_id))
        proto=OUT/'rgb_prototype';proto.mkdir(exist_ok=True)
        sample=[j for j,t in enumerate(ts) if int(lts[0])<=t<int(lts[0])+10*10**9]
        rows=[encode(j,proto) for j in sample];projection=sum(v['size_bytes'] for v in rows)/len(rows)*len(selected)
        accepted=projection<=policy['max_additional_bytes']
        dump(OUT/'stage20b_rgb_prototype.json',dict(count=len(rows),actual_duration_s=(int(ts[sample[-1]])-int(ts[sample[0]]))/1e9,total_bytes=sum(v['size_bytes'] for v in rows),projected_count=len(selected),projected_bytes=projection,approved=accepted,policy_sha256=digest(p)))
        old=json.loads((ROOT/'demo/stage19_rgb_three_map/rgb_manifest.json').read_text())
        if accepted:
            frames=[encode(j,destination) for j in sorted(selected)]
            assert sum(v['size_bytes'] for v in frames)<=policy['max_additional_bytes']
            status='AUTHENTIC_LOW_RATE_FORWARD_STREAM'
        else:
            frames=old['frames']['robot3'];frames=[{**v,'file':'../stage19_rgb_three_map/'+v['file']} for v in frames];status='SPARSE_HISTORICAL_COVERAGE_ONLY'
        con.close()
    assert stat==(archive.stat().st_size,archive.stat().st_mtime_ns)
    r1=[{**v,'file':'../stage19_rgb_three_map/'+v['file']} for v in old['frames']['robot1']]
    result=dict(status=status,robots=old['robots'],frames=dict(robot1=r1,robot3=frames),GT_used=False,policy=policy,archive=dict(path=str(archive),size=stat[0],mtime_ns=stat[1],db_member=member,crc=z.getinfo(member).CRC,crc_validated=True,topic=topic,type=typ,recorded_count=len(index)),seconds=time.perf_counter()-began)
    dump(SITE/'rgb_manifest.json',result)
    dump(OUT/'stage20b_rgb_export.json',dict(status=status,count=len(frames),total_bytes=sum(v['size_bytes'] for v in frames),seconds=result['seconds'],manifest_sha256=digest(SITE/'rgb_manifest.json'),raw_unchanged=True,recorded_rate_hz=1e9/float(np.median(np.diff(ts))),exported_target_hz=1.0,header_not_future=all(int(v['header_timestamp_ns'])<=int(v['rgb_timestamp_ns']) for v in frames)))
    refresh_metadata()
    print(status,len(frames),flush=True)

if __name__=='__main__':
    import sys
    if '--metadata-only' in sys.argv:refresh_metadata()
    else:main()
