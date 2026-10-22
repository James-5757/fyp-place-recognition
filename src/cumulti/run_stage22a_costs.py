"""Actual encoded buffers over frozen causal events; no network measurement.

B1 full-SC-first baseline, unchanged lossless XYZI cloud transport, unbounded
logical central cache. Physical teammate per-call cache remains size64/fresh.
Serialization replay is post-hoc accounting, not a second retrieval run.
"""
import resource,struct,time
from scipy.spatial.transform import Rotation
import stage21_wire as wire
from stage22a_common import *

def matrix(row):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([row.qx,row.qy,row.qz,row.qw]).as_matrix();T[:3,3]=[row.tx,row.ty,row.tz];return T

def main():
    verify_freeze();online_guard();f=frames();events=events_for(f)
    logical_attempts=pd.read_csv(OUT/'online_gicp_attempts.csv');logical_keys={(int(r.event_index),int(r.candidate_rank)) for r in logical_attempts.itertuples()}
    physical=[]
    for path in sorted((OUT/'attempts').glob('e*_r?.json')):
        receipt=json.loads(path.read_text());m=receipt['measurement'];physical.append(dict(m,measurement_phase=receipt['phase'],present_in_chronological_verification_log=(m['event_index'],m['candidate_rank']) in logical_keys))
    attempts=pd.DataFrame(physical);byevent={int(i):x.to_dict('records') for i,x in attempts.groupby('event_index')}
    csv('physical_GICP_execution_trace.csv',[dict(m,verified_transform=json.dumps(m['verified_transform']),canonical_transform=json.dumps(m['canonical_transform'])) for m in physical])
    loops=pd.read_csv(OUT/'online_verified_loops.csv');loopevents={int(i):x.to_dict('records') for i,x in loops.groupby('discovery_event')}
    loc=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv');local={(r.robot_id,int(r.keyframe_id)):matrix(r) for r in loc.itertuples()}
    backend=pd.read_csv(OUT/'online_backend_event_trace.csv') if (OUT/'online_backend_event_trace.csv').exists() else pd.DataFrame()
    latency=pd.read_csv(OUT/'online_retrieval_latency.csv').set_index('event_index');backendrows={} if backend.empty else backend.set_index('event_index').to_dict('index')
    cached=set();arrived=set();seen={r:0 for r in STARTS};rows=[];costs=[];messages=[];sigmas=[float(np.deg2rad(5))]*3+[1.]*3
    cachebytes=0;totalhits=0;totalmiss=0;accounting_start=time.monotonic()
    for e in events:
        i=e['event_index'];r=e['robot_id'];k=e['keyframe_id'];rid=int(r[-1]);other='robot3' if rid==1 else 'robot1';arrived.add((r,k));event_messages=[];hits=0;miss=0;requested=0
        def send(kind,sender,receiver,owner,kf,stamp,body):
            rec=dict(event_index=i,message_type=kind,sender=sender,receiver=receiver,robot_id=owner,keyframe_id=kf,source_timestamp_ns=int(stamp),**wire.measured(kind,sender,receiver,owner,kf,i,int(stamp),body));messages.append(rec);event_messages.append(rec)
        p=np.load(PROC/r/'lidar'/f[r].loc[k,'lidar_file'],allow_pickle=False);desc=s2.descriptor(p)
        send('FULL_SC_DESCRIPTOR_UPLOAD',rid,0,rid,k,e['timestamp_ns'],np.asarray(desc,dtype='<f4').tobytes())
        if seen[other]:send('CANDIDATE_REQUEST',rid,0,rid,k,e['timestamp_ns'],struct.pack('<HBB',min(1000,seen[other]),min(3,seen[other]),int(other[-1])))
        for a in byevent.get(i,[]):
            for robot,kf in [(a['query_robot'],int(a['query_keyframe_id'])),(a['candidate_robot'],int(a['candidate_keyframe_id']))]:
                assert (robot,kf) in arrived,'future cloud protocol request'
                if (robot,kf) in cached:hits+=1;continue
                miss+=1;requested+=1;cached.add((robot,kf));owner=int(robot[-1]);stamp=int(f[robot].loc[kf,'lidar_timestamp_ns'])
                send('POINT_CLOUD_REQUEST',0,owner,owner,kf,stamp,bytes([4]))
                cloud=p if robot==r and kf==k else np.load(PROC/robot/'lidar'/f[robot].loc[kf,'lidar_file'],allow_pickle=False)
                body=wire.cloud_body(cloud);assert np.array_equal(wire.parse_cloud(body),cloud)
                send('POINT_CLOUD_RESPONSE',owner,0,owner,kf,stamp,body);cachebytes+=cloud.nbytes
        for m in loopevents.get(i,[]):
            q=int(m['canonical_robot1_kf']);c=int(m['canonical_robot3_kf']);T=np.asarray(json.loads(m['canonical_transform']))
            send('LOOP_CONSTRAINT',rid,0,1,q,int(f['robot1'].loc[q,'lidar_timestamp_ns']),wire.factor_body(3,q,c,T,sigmas,1.345))
        # Non-GT odometry message policy is explicit, including independent first
        # local-pose metadata (only R1 becomes a central prior at first bridge).
        prev=k-1;kind=0;T=local[(r,k)]
        if k==STARTS[r]:prev=-1;kind=1 if rid==1 else 2
        else:T=inv(local[(r,prev)])@T
        body=struct.pack('<Bq',kind,prev)+np.asarray(T,dtype='<f8').tobytes()+np.asarray(sigmas,dtype='<f8').tobytes()
        send('ODOMETRY_UPDATE',rid,0,rid,k,e['timestamp_ns'],body)
        seen[r]+=1;totalhits+=hits;totalmiss+=miss
        rows.append(dict(event_index=i,normalized_time_ns=e['normalized_ns'],query_robot=r,new_descriptors_published=1,full_descriptors_requested=0,point_clouds_requested=requested,actual_GICP_calls=len(byevent.get(i,[])),new_loop_factors=len(loopevents.get(i,[])),messages=len(event_messages),serialized_application_bytes=sum(m['serialized_total_bytes'] for m in event_messages),compressed_application_bytes=sum(m['compressed_total_bytes'] for m in event_messages),cloud_cache_hits=hits,cloud_cache_misses=miss,modelled_cloud_cache_raw_bytes=cachebytes,descriptor_cache_entries=len(arrived),message_send_scope='PROTOCOL_SIMULATED',NETWORK_BYTES_ACTUALLY_TRANSMITTED='NOT_MEASURED'))
        lat=latency.loc[i];back=backendrows.get(i,{})
        costs.append(dict(event_index=i,query_robot=r,descriptor_input_load_ms=float(lat.input_load_ms),descriptor_compute_ms=float(lat.descriptor_compute_ms),ring_key_compute_ms=float(lat.ring_key_compute_ms),candidate_retrieval_ms=float(lat.total_retrieval_ms),database_append_ms=float(lat.own_database_insert_ms),actual_GICP_call_ms=sum(a['call_wall_ms'] for a in byevent.get(i,[])),actual_coarse_fine_ms=sum(a['coarse_ms']+a['fine_ms'] for a in byevent.get(i,[])),actual_isam2_update_ms=back.get('isam2_update_ms',0),backend_event_status=back.get('status','NOT_RUN'),serialization_accounting_ms=sum(m['serialization_time_ms']+m['deserialization_time_ms'] for m in event_messages),posthoc_accounting_not_in_online_critical_path=True))
        if i%1000==0:print('PROTOCOL_BUFFERS',i,flush=True)
    csv('online_communication_cost.csv',rows);csv('online_frontend_compute_cost.csv',costs);csv('online_protocol_message_trace.csv',messages)
    grouped=pd.DataFrame(messages).groupby('message_type')[['payload_bytes','framing_bytes','serialized_total_bytes','compressed_total_bytes','serialization_time_ms','deserialization_time_ms','compression_time_ms','decompression_time_ms']].sum().reset_index();csv('online_protocol_category_summary.csv',grouped)
    dump('online_cost_summary.json',dict(status='MEASURED_BUFFERS_SIMULATED_SENDS',events=5796,messages=len(messages),serialized_application_bytes=sum(x['serialized_application_bytes'] for x in rows),compressed_application_bytes=sum(x['compressed_application_bytes'] for x in rows),cloud_cache_hits=totalhits,cloud_cache_misses=totalmiss,modelled_unbounded_cloud_cache_raw_bytes=cachebytes,physical_actual_GICP_calls=len(physical),physical_actual_GICP_harness_seconds=float(attempts.call_wall_ms.sum()/1000),attempts_in_chronological_log=len(logical_attempts),physical_GICP_cache='unchanged64/fresh per teammate call; logical protocol cache is a different model',accounting_scope='all actual Stage22A calls including pilot; pilot-only tail attempts are NOT completed full-prefix verification when capped',full_descriptor_requests=0,full_descriptor_request_reason='B1 full-descriptor-first baseline, every arriving descriptor published once; no RingKey-only communication claim',NETWORK_BYTES_ACTUALLY_TRANSMITTED='NOT_MEASURED',measured_accounting_elapsed_s=time.monotonic()-accounting_start,peak_RSS_accounting_process_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,physical_robot_memory_or_CPU='NOT_MEASURED'))
if __name__=='__main__':main()
