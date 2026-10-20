#!/usr/bin/env python3
"""Gates B/C: actual serialized bytes, round trips and bounded payload timings."""
import csv,json,resource,time,zlib
from pathlib import Path
import numpy as np,pandas as pd
from prepare_stage21_policy import ROOT,OUT,PROC,dump,sha
import stage21_wire as w

def cloud_path(robot,k,frames):return PROC/robot/'lidar'/frames[robot].loc[k,'lidar_file']
def payload(kind,r,k,ts,body,event=None):return w.measured(kind,r,0,r,k,(-k-1 if r==3 else k+1) if event is None else event,int(ts),body)
def read_ply(path):
    with Path(path).open('rb') as f:
        lines=[]
        while True:
            s=f.readline().decode().strip();lines.append(s)
            if s=='end_header':break
        assert 'format binary_little_endian 1.0' in lines
        return np.frombuffer(f.read(),dtype='<f4').reshape(-1,3)

def main():
    policy=json.loads((OUT/'stage21_measurement_policy.json').read_text());assert not (OUT/'cloud_wire_catalog.csv').exists()
    frames={r:pd.read_csv(PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).set_index('keyframe_id') for r in ['robot1','robot3']}
    desc={r:np.load(PROC/r/'scan_context_descriptors.npy') for r in frames};tests={};metrics=[];pointmetrics=[]
    for robot in frames:
        r=int(robot[-1]);a=desc[robot];assert a.dtype==np.float32 and a.shape[1:]==(20,60)
        keys=np.asarray(a.mean(axis=2),np.float32);frozen=np.load(PROC.parent/'ringkey_stage8'/f'{robot}_ringkeys.npy');assert np.array_equal(keys,frozen)
        tests[robot+'_ring_key_matches_frozen']=True
        for k in policy['cloud_samples'][robot]:
            ts=int(frames[robot].loc[k,'lidar_timestamp_ns'])
            for label,kind,array in [('RING_KEY','RING_KEY_UPLOAD',keys[k]),('FULL_SC','FULL_SC_DESCRIPTOR_UPLOAD',a[k])]:
                body=array.astype('<f4',copy=False).tobytes();assert len(body)==array.nbytes
                for repeat in range(3):
                    m=payload(kind,r,k,ts,body);wire=w.encode(kind,r,0,r,k,k+1,ts,body);meta,b=w.decode(wire);assert meta['source_timestamp_ns']==ts and np.array_equal(np.frombuffer(b,dtype='<f4').reshape(array.shape),array)
                    metrics.append(dict(robot_id=robot,keyframe_id=k,representation=label,dtype='float32',shape=str(array.shape),raw_array_bytes=array.nbytes,repeat=repeat,**m))
            p=cloud_path(robot,k,frames);cloud=np.load(p,allow_pickle=False);assert cloud.dtype==np.float32 and cloud.shape[1]==4 and np.isfinite(cloud[:,:3]).all()
            for fields in (3,4):
                body=w.cloud_body(cloud[:,:fields]);assert np.array_equal(w.parse_cloud(body),cloud[:,:fields])
                for repeat in range(3):
                    pointmetrics.append(dict(robot_id=robot,keyframe_id=k,representation='XYZ' if fields==3 else 'XYZI',point_count=len(cloud),native_file_bytes=p.stat().st_size,native_stored_array_bytes=cloud.nbytes,raw_array_bytes=len(cloud)*fields*4,point_metadata_bytes=w.CLOUD.size,algorithm_compatible=True,intensity_consumed_by_frozen_GICP=False,repeat=repeat,source_sha256=sha(p),**payload('POINT_CLOUD_RESPONSE',r,k,ts,body)))
    # Malformed/corrupted frames must fail rather than silently changing measurements.
    wire=w.encode('RING_KEY_UPLOAD',1,0,1,150,0,2**60,b'\0'*80);bad=wire[:-1]+b'1'
    try:w.decode(bad);raise RuntimeError('CRC corruption was accepted')
    except AssertionError:tests['corrupt_CRC_rejected']=True
    try:w.decode(wire[:-2]);raise RuntimeError('truncation was accepted')
    except AssertionError:tests['truncation_rejected']=True
    tests.update(descriptor_float32_lossless=True,pointcloud_XYZ_XYZI_lossless=True,source_timestamp_int64_exact=True)
    # Every declared type is exercised with a synthetic schema fixture; unused
    # categories are NOT subsequently invented as architecture send events.
    fixtures={name:b'' for name in w.TYPES};fixtures.update(RING_KEY_UPLOAD=b'\0'*80,FULL_SC_DESCRIPTOR_UPLOAD=b'\0'*4800,FULL_SC_DESCRIPTOR_RESPONSE=b'\0'*4800,CANDIDATE_REQUEST=b'\xe8\x03\x03\x03',POINT_CLOUD_REQUEST=b'\x03',POINT_CLOUD_RESPONSE=w.cloud_body(np.zeros((3,3),np.float32)),MAP_CHUNK=w.cloud_body(np.zeros((3,3),np.float32)),LOOP_CONSTRAINT=w.factor_body(3,283,252,np.eye(4),np.ones(6),1.345),ODOMETRY_UPDATE=b'\0'*185,GLOBAL_POSE_CORRECTION=b'\0'*56,ACK_OR_CONTROL=b'\0')
    for name,b in fixtures.items():assert w.decode(w.encode(name,1,0,1,150,10,2**60,b))[1]==b
    tests['all12_message_types_roundtrip']=True
    pd.DataFrame(metrics).to_csv(OUT/'descriptor_payload_metrics.csv',index=False);pd.DataFrame(pointmetrics).to_csv(OUT/'pointcloud_payload_metrics.csv',index=False)
    dump('message_serialization_tests.json',dict(status='PASS',tests=tests,header_bytes=w.HEADER.size,compression='whole message zlib1 roundtrip checked for every measured buffer',fixtures='schema unit tests; not simulated protocol traffic'))
    # Full common-workload catalog: actual serialization of every B0 upload, not
    # nbytes scaling from a representative scan or a ROS archive size.
    path=OUT/'cloud_wire_catalog.csv';began=time.perf_counter();count=0
    with path.open('w',newline='') as f:
        writer=None
        for robot,ids in [('robot3',list(frames['robot3'].index)),('robot1',policy['queries'])]:
            r=int(robot[-1])
            for k in ids:
                p=cloud_path(robot,k,frames);cloud=np.load(p,allow_pickle=False);body=w.cloud_body(cloud[:,:3]);assert np.array_equal(w.parse_cloud(body),cloud[:,:3]);stamp=int(frames[robot].loc[k,'lidar_timestamp_ns']);m=payload('POINT_CLOUD_RESPONSE',r,k,stamp,body)
                row=dict(robot_id=robot,keyframe_id=k,source_timestamp_ns=str(stamp),logical_event_id=-k-1 if r==3 else k+1,point_count=len(cloud),fields='XYZ float32; original intensity not used by frozen GICP',native_file_bytes=p.stat().st_size,source_sha256=sha(p),body_sha256=__import__('hashlib').sha256(body).hexdigest(),**m)
                if writer is None:writer=csv.DictWriter(f,fieldnames=list(row));writer.writeheader()
                writer.writerow(row);count+=1
                if count%500==0:print('GATE_C serialized actual full clouds',count,flush=True)
    finish_maps_and_validate(time.perf_counter()-began)

def finish_maps_and_validate(catalog_wall_seconds=None):
    frames={r:pd.read_csv(PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).set_index('keyframe_id') for r in ['robot1','robot3']}
    policy=json.loads((OUT/'stage21_measurement_policy.json').read_text());preparation=[]
    for robot in frames:
        for k in policy['cloud_samples'][robot]:
            p=np.load(cloud_path(robot,k,frames),allow_pickle=False)
            for fields in (3,4):
                for repeat in range(3):
                    t=time.perf_counter_ns();c=time.process_time_ns();body=w.cloud_body(p[:,:fields]);preparation.append(dict(robot_id=robot,keyframe_id=k,fields=fields,repeat=repeat,operation='float32_payload_conversion_and_copy',wall_ms=(time.perf_counter_ns()-t)/1e6,cpu_ms=(time.process_time_ns()-c)/1e6))
    pd.DataFrame(preparation).to_csv(OUT/'payload_preparation_timings.csv',index=False)
    states=json.loads((ROOT/'outputs/cumulti_v1/20_dual_stream/backend_state_manifest.json').read_text());maprows=[]
    for name in ['FIRST_BRIDGE','FINAL']:
        s=next(v for v in states if v['name']==name)
        for art in s['maps']:
            points=read_ply(art['path']);body=w.cloud_body(points);assert np.array_equal(w.parse_cloud(body),points)
            trmap=pd.read_csv(s['trajectory'],usecols=['robot_id','timestamp']);stamp=int(trmap.loc[trmap.robot_id.eq(art['robot']),'timestamp'].iloc[-1])
            r=int(art['robot'][-1]);maprows.append(dict(checkpoint=name,robot_id=art['robot'],point_count=len(points),source=art['path'],source_sha256=sha(art['path']),scope='SERIALIZER_MICROBENCH_ONLY_NOT_A_SIMULATED_MAP_DISTRIBUTION_POLICY',**w.measured('MAP_CHUNK',0,r,r,s['robot1_kf'] if r==1 else s['robot3_kf'],s['event_index'],stamp,body)))
    pd.DataFrame(maprows).to_csv(OUT/'map_chunk_payload_metrics.csv',index=False)
    count=len(pd.read_csv(OUT/'cloud_wire_catalog.csv'))
    dump('payload_catalog_validation.json',dict(status='PASS',all_B0_clouds_actually_serialized=count,expected=4180+100,all_point_values_lossless=True,seconds=catalog_wall_seconds,completion_note='map-report function argument corrected after completed cloud catalog; no cloud numbers overwritten' if catalog_wall_seconds is None else 'normal completion',peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,not_network_measurement=True))
    assert count==4280;print('GATE_B_C_PASS',count,flush=True)
if __name__=='__main__':
    import sys
    if '--finish-map-report' in sys.argv:finish_maps_and_validate()
    else:main()
