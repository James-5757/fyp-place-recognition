"""Exact Stage21 encoded buffers for primary policies; fixed-cache sensitivity remains modeled."""
import struct
import time
from common import *
from replay import Cache
import stage21_wire as wire


def main():
    verify_lock();guard()
    original=pd.read_csv(A/'online_protocol_message_trace.csv')
    fixed=original[~original.message_type.isin(['POINT_CLOUD_REQUEST','POINT_CLOUD_RESPONSE','LOOP_CONSTRAINT'])]
    summary=pd.read_csv(OUT/'policy_results_by_budget.csv')
    scans=pd.read_csv(A/'lidar_input_manifest.csv')
    paths={(r.robot,int(r.keyframe)):r.path for r in scans.itertuples()}
    timestamps={r:pd.read_csv(PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns']).set_index('keyframe_id').lidar_timestamp_ns.to_dict() for r in ['robot1','robot3']}
    names=read(OUT/'stage22b_experiment_policy.json')['policies'][:5]+['P5_BUDGET_AWARE_C75_B80']
    rows=[];measurements=[]
    for name in names:
        directory=OUT/'policies'/name;output=directory/'encoded_communication.json'
        if output.exists():rows.append(read(output));continue
        attempts=pd.read_csv(directory/'selected_attempts.csv')
        loops=pd.read_csv(directory/'online_verified_loops.csv')
        cache=Cache();cloud_bytes=0;compressed=0;requests=0;hits=0;elapsed=0.;serialize_ms=0.;compression_ms=0.
        loop_bytes=0;loop_compressed=0
        start=time.monotonic()
        for index,row in enumerate(attempts.to_dict('records')):
            event=int(row['event_index'])
            for robot,kf in [(row['query_robot'],int(row['query_keyframe_id'])),(row['candidate_robot'],int(row['candidate_keyframe_id']))]:
                key=(robot,kf)
                if cache.access(key):hits+=1;continue
                requests+=1;owner=int(robot[-1]);stamp=timestamps[robot][kf]
                p=np.load(paths[key],allow_pickle=False);body=wire.cloud_body(p)
                assert np.array_equal(wire.parse_cloud(body),p)
                req=wire.measured('POINT_CLOUD_REQUEST',0,owner,owner,kf,event,stamp,bytes([4]))
                response=wire.measured('POINT_CLOUD_RESPONSE',owner,0,owner,kf,event,stamp,body)
                cloud_bytes+=req['serialized_total_bytes']+response['serialized_total_bytes']
                compressed+=req['compressed_total_bytes']+response['compressed_total_bytes']
                serialize_ms+=req['serialization_time_ms']+response['serialization_time_ms']
                compression_ms+=req['compression_time_ms']+response['compression_time_ms']
            if index%2000==0:print('WIRE',name,index,'requests',requests,flush=True)
        sigmas=[float(np.deg2rad(5))]*3+[1.]*3
        for row in loops.itertuples():
            q,c=int(row.canonical_robot1_kf),int(row.canonical_robot3_kf)
            body=wire.factor_body(3,q,c,np.array(json.loads(row.canonical_transform)),sigmas,1.345)
            rec=wire.measured('LOOP_CONSTRAINT',int(row.query_robot[-1]),0,1,q,int(row.discovery_event),timestamps['robot1'][q],body)
            loop_bytes+=rec['serialized_total_bytes'];loop_compressed+=rec['compressed_total_bytes']
            serialize_ms+=rec['serialization_time_ms'];compression_ms+=rec['compression_time_ms']
        result=dict(policy=name,application_bytes=int(fixed.serialized_total_bytes.sum())+cloud_bytes+loop_bytes,
            compressed_application_bytes=int(fixed.compressed_total_bytes.sum())+compressed+loop_compressed,
            descriptor_bytes=int(fixed[fixed.message_type=='FULL_SC_DESCRIPTOR_UPLOAD'].serialized_total_bytes.sum()),
            point_cloud_bytes=cloud_bytes,factor_update_bytes=int(fixed[fixed.message_type=='ODOMETRY_UPDATE'].serialized_total_bytes.sum())+loop_bytes,
            cloud_cache_hits=hits,cloud_cache_misses=requests,cache_capacity='unbounded',
            fixed_message_buffers='Reuse exact Stage22A same-event encoded-buffer catalog; fixed fields/payload/publication unchanged',
            variable_message_buffers='All requests/responses/loops newly encoded/decoded/compressed with Stage21 schema at policy event',
            evidence_type='PROTOCOL_SIMULATED',buffer_measurement='ACTUAL_EXECUTED',
            serialization_ms=serialize_ms,compression_ms=compression_ms,accounting_wall_s=time.monotonic()-start,
            network_transmitted_bytes='NOT_MEASURED',physical_network_latency='NOT_MEASURED')
        expected=summary[summary.policy==name].iloc[0]
        assert result['application_bytes']==expected.application_bytes
        assert result['cloud_cache_misses']==expected.cloud_cache_misses and result['cloud_cache_hits']==expected.cloud_cache_hits
        if name=='P0_FULL_TOP3':
            assert result['application_bytes']==6107977604 and result['compressed_application_bytes']==4211894087
        write_json(output,result);rows.append(result)
    byname={r['policy']:r for r in rows}
    final=[]
    for r in summary.to_dict('records'):
        r.update(byname.get(r['policy'],{}))
        if r['policy'] not in byname:r['buffer_measurement']='Reused exact uncompressed Stage22A buffer-length catalog; new compression NOT_MEASURED'
        final.append(r)
    csv(OUT/'communication_by_policy.csv',final)
    write_json(OUT/'communication_validation.json',dict(status='PASS',primary_policies=len(rows),
        exact_P0_bytes_and_compression_reproduced=True,new_full_variable_buffers_encoded=True,
        cache_sensitivity_raw_bytes='PROTOCOL_SIMULATED exact uncompressed catalog lengths; no wireless or compression claim'))

if __name__=='__main__':main()
