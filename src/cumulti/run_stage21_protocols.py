#!/usr/bin/env python3
"""Gate E: buffer-measured application messages + explicit protocol-simulated sends."""
import collections,csv,functools,hashlib,json,struct,sys,time
import numpy as np,pandas as pd
from scipy.spatial.transform import Rotation
from prepare_stage21_policy import ROOT,OUT,PROC,dump,sha
from run_stage21_payloads import cloud_path
import stage21_wire as w
FIELDS=['mode','scope','phase','message_type','sender','receiver','robot_id','original_keyframe_id','logical_event_id','source_timestamp_ns','logical_time_ns','payload_bytes','framing_bytes','serialized_total_bytes','compression_method','compressed_total_bytes','serialization_time_ms','deserialization_time_ms','compression_time_ms','decompression_time_ms','serialization_cpu_ms','optional_source_hash','measurement_origin']

def inv(t):
    x=np.eye(4);x[:3,:3]=t[:3,:3].T;x[:3,3]=-x[:3,:3]@t[:3,3];return x
def matrix(row):
    x=np.eye(4);x[:3,:3]=Rotation.from_quat([row.qx,row.qy,row.qz,row.qw]).as_matrix();x[:3,3]=[row.tx,row.ty,row.tz];return x

def main():
    assert json.loads((OUT/'frontend_instrumentation.json').read_text())['status']=='PASS';assert not (OUT/'message_trace_by_mode.csv').exists()
    p=json.loads((OUT/'stage21_measurement_policy.json').read_text());qids=p['queries']
    contract=dict(status='PROTOCOL_SIMULATION_NOT_NETWORK',common_workload='all4180 R3 database keyframes, prebuilt before R1 query phase; the same original100 query IDs and frozen173 attempted pairs for B0/B1/B2',B0=dict(sensor_clouds='full original float32 XYZ, no crop/voxel before upload; intensity verified unused by frozen SC/GICP',trigger='cloud arrival implicitly requests retrieval, no redundant request',descriptor_extraction_location='central',cloud_cache='all database uploads retained'),B1=dict(database_sync='all R3 full SC descriptors',query='R1 full SC upload plus explicit CANDIDATE_REQUEST',geometric='central requests query cloud once and candidate clouds in frozen attempt order; unbounded received-cloud cache',descriptor_extraction_location='robot-local proxy, measured on server host'),B2=dict(database_sync='all R3 Ring Keys',query='R1 Ring Key + CANDIDATE_REQUEST; central requests query full SC and each actual M1000 shortlist full SC on cache miss',cache='UNBOUNDED primary; NONE/LRU128/LRU1000 sensitivity, no selection/ranking change',geometric='same frozen requests and cloud cache as B1'),B3=dict(scope='ORACLE LOWER BOUND; separate Stage20 dual stream, not a common frontend-workload competitor',updates='5796 initial/relative odometry messages plus75 archived loops; initial R3 pose is node metadata, NOT a second global prior',producer='robot1 logical oracle producer for frozen inter-robot constraints; no claim of online discovery on robot1'),shared=dict(operation_locations='retrieval/GICP/iSAM2 are central; descriptor extraction logical location depends on mode; all new timing measurements physically on server',missing='sensor acquisition/EKF creation cost, physical robot CPU, real transport, correction/map distribution policy NOT_MEASURED',no_automatic_downlink='GLOBAL_POSE_CORRECTION/MAP_CHUNK schemas measured separately; no invented send cadence or ACKs',byte_rules='actual entire encoded buffers, application framing included; raw NPY/ZIP sizes never network counts',compressed='actual zlib1 length for each logical message; no packet/link overhead',rate='query-phase bytes / actual span of100 original query timestamps; DB synchronization phase reported separately, not assigned fictitious instantaneous bandwidth'))
    dump('stage21_architecture_contract.json',contract)
    frames={r:pd.read_csv(PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).set_index('keyframe_id') for r in ['robot1','robot3']};desc={r:np.load(PROC/r/'scan_context_descriptors.npy') for r in frames};keys={r:np.asarray(d.mean(axis=2),np.float32) for r,d in desc.items()}
    catalog=pd.read_csv(OUT/'cloud_wire_catalog.csv',dtype={'source_timestamp_ns':'int64'});lookup={(r.robot_id,int(r.keyframe_id)):r._asdict() for r in catalog.itertuples(index=False)}
    accesses=pd.read_csv(OUT/'measured_shortlist_accesses.csv');byq={int(k):g.sort_values('access_order').candidate_keyframe_id.astype(int).tolist() for k,g in accesses.groupby('query_keyframe_id')};assert all(len(v)==1000 for v in byq.values())
    attempts={q:[a for a in p['frozen_gicp_attempts'] if a['query']==q] for q in qids};summary={};cachemetrics=[];category=collections.defaultdict(lambda:collections.Counter());tracepath=OUT/'message_trace_by_mode.csv'
    @functools.lru_cache(maxsize=16)
    def body_cloud(robot,k):return w.cloud_body(np.load(cloud_path(robot,k,frames),allow_pickle=False)[:,:3])
    with tracepath.open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=FIELDS);writer.writeheader()
        def emit(mode,scope,phase,kind,sender,receiver,robot,k,event,stamp,t,body=None,measured=None,sourcehash=''):
            m=w.measured(kind,sender,receiver,robot,k,event,int(stamp),body) if measured is None else measured
            row=dict(mode=mode,scope=scope,phase=phase,message_type=kind,sender=sender,receiver=receiver,robot_id=f'robot{robot}',original_keyframe_id=k,logical_event_id=event,source_timestamp_ns=str(int(stamp)),logical_time_ns='' if t is None else str(int(t)),optional_source_hash=sourcehash,measurement_origin='ACTUAL_SERIALIZED_BUFFER_CATALOG_SAME_HEADER' if measured is not None else 'ACTUAL_SERIALIZED_BUFFER_THIS_EVENT',**{k:m[k] for k in ['payload_bytes','framing_bytes','serialized_total_bytes','compression_method','compressed_total_bytes','serialization_time_ms','deserialization_time_ms','compression_time_ms','decompression_time_ms','serialization_cpu_ms']})
            writer.writerow(row);a=summary.setdefault(mode,dict(mode=mode,scope=scope,total_messages=0,total_serialized_bytes=0,total_compressed_bytes=0,uplink_bytes=0,downlink_bytes=0,database_synchronization_bytes=0,query_phase_bytes=0,peak_logical_message_bytes=0,serialization_ms=0.,deserialization_ms=0.));a['total_messages']+=1;a['total_serialized_bytes']+=int(m['serialized_total_bytes']);a['total_compressed_bytes']+=int(m['compressed_total_bytes']);a['uplink_bytes' if receiver==0 else 'downlink_bytes']+=int(m['serialized_total_bytes']);a['database_synchronization_bytes' if phase=='DATABASE_SYNC' else 'query_phase_bytes']+=int(m['serialized_total_bytes']);a['peak_logical_message_bytes']=max(a['peak_logical_message_bytes'],int(m['serialized_total_bytes']));a['serialization_ms']+=float(m['serialization_time_ms']);a['deserialization_ms']+=float(m['deserialization_time_ms']);category[(mode,kind)]['messages']+=1;category[(mode,kind)]['serialized_bytes']+=int(m['serialized_total_bytes'])
        scope='HISTORICAL_PREBUILT_FRONTEND'
        # Catalog has exactly the metadata used by B0. These buffers were actually
        # encoded in Gate C; no compression ratio or payload size is extrapolated.
        for row in catalog.itertuples():
            robot=row.robot_id;r=int(robot[-1]);qphase=robot=='robot1';t=int(row.source_timestamp_ns)-int(frames[robot].loc[qids[0] if qphase else 0,'lidar_timestamp_ns']) if qphase else None
            emit('B0_FULL_CLOUD_UPLOAD',scope,'QUERY' if qphase else 'DATABASE_SYNC','POINT_CLOUD_RESPONSE',r,0,r,int(row.keyframe_id),int(row.logical_event_id),int(row.source_timestamp_ns),t,measured=row._asdict(),sourcehash=row.source_sha256)
        variants=[('B1_FULL_SC_DESCRIPTOR_FIRST','FULL_SC',None),('B2_RING_KEY_HIERARCHICAL','RING_KEY','UNBOUNDED'),('B2_RING_KEY_HIERARCHICAL_NONE','RING_KEY','NONE'),('B2_RING_KEY_HIERARCHICAL_LRU128','RING_KEY',128),('B2_RING_KEY_HIERARCHICAL_LRU1000','RING_KEY',1000)]
        for mode,representation,capacity in variants:
            kind='FULL_SC_DESCRIPTOR_UPLOAD' if representation=='FULL_SC' else 'RING_KEY_UPLOAD';array=desc if representation=='FULL_SC' else keys
            for k in frames['robot3'].index:emit(mode,scope,'DATABASE_SYNC',kind,3,0,3,int(k),-int(k)-1,int(frames['robot3'].loc[k,'lidar_timestamp_ns']),None,body=array['robot3'][k].astype('<f4',copy=False).tobytes())
            cache=collections.OrderedDict();cloudcache=set();hits=misses=evictions=0;maxitems=0;lookup_ns=0
            for q in qids:
                stamp=int(frames['robot1'].loc[q,'lidar_timestamp_ns']);t=stamp-int(frames['robot1'].loc[qids[0],'lidar_timestamp_ns'])
                emit(mode,scope,'QUERY',kind,1,0,1,q,q+1,stamp,t,body=array['robot1'][q].astype('<f4',copy=False).tobytes())
                emit(mode,scope,'QUERY','CANDIDATE_REQUEST',1,0,1,q,q+1,stamp,t,body=struct.pack('<HBB',1000,3,3))
                if representation=='RING_KEY':
                    for robot,k in [('robot1',q)]+[('robot3',k) for k in byq[q]]:
                        key=(robot,k);start=time.perf_counter_ns();hit=key in cache
                        if hit:cache.move_to_end(key);hits+=1
                        else:
                            misses+=1
                            if capacity!='NONE':
                                cache[key]=True
                                if isinstance(capacity,int) and len(cache)>capacity:cache.popitem(last=False);evictions+=1
                                maxitems=max(maxitems,len(cache))
                        lookup_ns+=time.perf_counter_ns()-start
                        if not hit:
                            r=int(robot[-1]);ts=int(frames[robot].loc[k,'lidar_timestamp_ns']);emit(mode,scope,'QUERY','FULL_SC_DESCRIPTOR_REQUEST',0,r,r,k,q+1,ts,t,body=b'');emit(mode,scope,'QUERY','FULL_SC_DESCRIPTOR_RESPONSE',r,0,r,k,q+1,ts,t,body=desc[robot][k].astype('<f4',copy=False).tobytes())
                for robot,k in [('robot1',q)]+[('robot3',a['candidate']) for a in attempts[q]]:
                    key=(robot,k)
                    if key in cloudcache:continue
                    cloudcache.add(key);r=int(robot[-1]);ts=int(frames[robot].loc[k,'lidar_timestamp_ns']);body=body_cloud(robot,k);assert hashlib.sha256(body).hexdigest()==lookup[key]['body_sha256']
                    emit(mode,scope,'QUERY','POINT_CLOUD_REQUEST',0,r,r,k,q+1,ts,t,body=b'\x03');emit(mode,scope,'QUERY','POINT_CLOUD_RESPONSE',r,0,r,k,q+1,ts,t,body=body,sourcehash=lookup[key]['source_sha256'])
            a=summary[mode];a.update(cache_policy=str(capacity) if representation=='RING_KEY' else 'all descriptors upfront; unbounded received-cloud cache',cloud_response_cache_entries=len(cloudcache));cachemetrics.append(dict(mode=mode,capacity=str(capacity),descriptor_accesses=hits+misses,cache_hits=hits,cache_misses=misses,evictions=evictions,max_resident_items=maxitems,modelled_max_payload_resident_bytes=maxitems*4800,measured_Python_index_overhead_bytes=sys.getsizeof(cache)+sum(sys.getsizeof(k)+sys.getsizeof(v) for k,v in cache.items()),measured_logical_cache_lookup_ms=lookup_ns/1e6,total_serialized_bytes=a['total_serialized_bytes'],total_messages=a['total_messages'],scope='same100 query shortlist accesses; protocol-simulated cache IDs, no measured remote cache or network'))
            print('GATE_E simulated',mode,a['total_messages'],flush=True)
        # Oracle uses ONLY Stage20 archived graph messages. No SC/GICP work is
        # implied, and these bytes are never a percentage-savings comparator.
        mode='B3_LOOP_FACTOR_ONLY_ORACLE';scope='DUAL_STREAM_ARCHIVED_FACTOR_REPLAY';tr=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv');pose={(r.robot_id,int(r.keyframe_id)):matrix(r) for r in tr.itertuples()};sigma=np.array([np.deg2rad(5)]*3+[1.]*3);schedule=pd.read_csv(ROOT/'outputs/cumulti_v1/20_dual_stream/dual_stream_event_schedule.csv');starts={'robot1':150,'robot3':234};factorchecks=0
        for e in schedule.itertuples():
            r=int(e.robot_id[-1]);k=int(e.keyframe_id);first=k==starts[e.robot_id];kind=1 if first and r==1 else 2 if first else 0;measurement=pose[(e.robot_id,k)] if first else inv(pose[(e.robot_id,k-1)])@pose[(e.robot_id,k)];sig=np.ones(6)*1e-6 if kind==1 else sigma;body=struct.pack('<Bq',kind,-1 if first else k-1)+measurement.astype('<f8').tobytes()+sig.astype('<f8').tobytes();assert np.array_equal(np.frombuffer(body,offset=9,dtype='<f8')[:16].reshape(4,4),measurement)
            emit(mode,scope,'DUAL_STREAM','ODOMETRY_UPDATE',r,0,r,k,int(e.event_index),int(e.timestamp_ns),int(e.normalized_ns),body=body)
        loops=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv');eligible=pd.read_csv(ROOT/'outputs/cumulti_v1/20_dual_stream/loop_eligibility_schedule.csv').set_index('frozen_loop_id')
        for r in loops.itertuples():
            measurement=matrix(r);body=w.factor_body(3,r.query_keyframe_id,r.candidate_keyframe_id,measurement,sigma,1.345);robot,q,c,Z,sig,delta=w.parse_factor(body);assert robot==3 and q==r.query_keyframe_id and c==r.candidate_keyframe_id and np.array_equal(Z,measurement) and np.array_equal(sig,sigma) and delta==1.345;factorchecks+=1;e=eligible.loc[r.frozen_loop_id]
            emit(mode,scope,'DUAL_STREAM','LOOP_CONSTRAINT',1,0,1,int(q),int(e.release_event_index),int(r.query_timestamp),int(e.eligibility_ns),body=body)
        summary[mode].update(cache_policy='oracle measurements already known; no frontend',cloud_response_cache_entries=0)
    duration=(int(frames['robot1'].loc[qids[-1],'lidar_timestamp_ns'])-int(frames['robot1'].loc[qids[0],'lidar_timestamp_ns']))/1e9
    for mode,a in summary.items():
        primary=mode in ['B0_FULL_CLOUD_UPLOAD','B1_FULL_SC_DESCRIPTOR_FIRST','B2_RING_KEY_HIERARCHICAL','B3_LOOP_FACTOR_ONLY_ORACLE'];a['primary_mode']=primary;a['status']='COMPLETE_DECLARED_SLICE_PROTOCOL_SIMULATION' if not mode.startswith('B3') else 'ORACLE_LOWER_BOUND_DIFFERENT_WORKLOAD';a['cache_policy']=a.get('cache_policy','all original clouds centrally retained');a['workload_keyframes']=5796 if mode.startswith('B3') else 4280;a['bytes_per_workload_keyframe']=a['total_serialized_bytes']/a['workload_keyframes'];a['bytes_per_archived_loop']=a['total_serialized_bytes']/75 if mode.startswith('B3') else None;a['query_phase_duration_s']=None if mode.startswith('B3') else duration;a['modelled_query_phase_bytes_per_recorded_second']=None if mode.startswith('B3') else a['query_phase_bytes']/duration;a['network_bytes_actually_transmitted']='NOT_MEASURED'
    pd.DataFrame(summary.values()).to_csv(OUT/'communication_summary_by_mode.csv',index=False);pd.DataFrame(cachemetrics).to_csv(OUT/'cache_sensitivity_summary.csv',index=False);pd.DataFrame([dict(mode=m,message_type=k,**v) for (m,k),v in category.items()]).to_csv(OUT/'message_category_summary.csv',index=False)
    dump('message_trace_manifest.json',dict(path=str(tracepath),size=tracepath.stat().st_size,sha256=sha(tracepath),rows=sum(a['total_messages'] for a in summary.values()),storage='full event-level trace server-only; commit category/architecture summaries and an indexed preview, never a fabricated truncated full trace',actual_buffers=True,events_are='PROTOCOL_SIMULATED_SEND_EVENTS',network='NOT_MEASURED',catalog_source_sha256=sha(OUT/'cloud_wire_catalog.csv'),frozen_loop_roundtrips=factorchecks,graph_reoptimized=False))
    pd.read_csv(tracepath,nrows=2000,dtype={'source_timestamp_ns':'str','logical_time_ns':'str'}).to_csv(OUT/'message_trace_preview_FIRST2000_ONLY.csv',index=False)
    print('GATE_E_PASS',len(summary),factorchecks,flush=True)
if __name__=='__main__':main()
