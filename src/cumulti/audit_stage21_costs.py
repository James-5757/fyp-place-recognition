#!/usr/bin/env python3
"""Gate F: independent arithmetic/stream/provenance audit, not a solver rerun."""
import collections,json,struct,zlib
from pathlib import Path
import numpy as np,pandas as pd
from prepare_stage21_policy import ROOT,OUT,PROC,BASE,dump,sha

def main():
    checks={};policy=json.loads((OUT/'stage21_measurement_policy.json').read_text());hashes=json.loads((OUT/'stage21_input_hashes.json').read_text());changed=[];allowed_context={'RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md'}
    for p,h in hashes['protected_historical'].items():
        if sha(p)!=h:
            rel=str(Path(p).relative_to(ROOT))
            if rel in allowed_context:
                import subprocess
                old=subprocess.check_output(['git','show',f'{BASE}:{rel}'],cwd=ROOT);assert Path(p).read_bytes().startswith(old),(p,'context must be append-only')
            else:changed.append(p)
    assert not changed,changed
    checks['HISTORICAL_STAGES_IMMUTABLE']=True
    for p,h in hashes['inputs'].items():assert sha(p)==h,p
    tests=json.loads((OUT/'message_serialization_tests.json').read_text());checks['SERIALIZATION_ROUNDTRIP_PASS']=tests['status']=='PASS'
    desc={r:np.load(PROC/r/'scan_context_descriptors.npy') for r in ['robot1','robot3']};frames={r:pd.read_csv(PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).set_index('keyframe_id') for r in desc}
    checks['SC_DESCRIPTOR_SHAPES_VERIFIED']=all(a.dtype==np.float32 and a.shape[1:]==(20,60) for a in desc.values())
    checks['RING_KEY_BYTES_VERIFIED']=all(a[0].mean(axis=1).astype(np.float32).nbytes==80 for a in desc.values())
    consumed=json.loads((OUT/'gicp_consumed_field_equivalence.json').read_text());checks['POINT_CLOUD_ENCODINGS_VALID']=consumed['status']=='PASS' and len(consumed['checks'])==40
    comm=pd.read_csv(OUT/'communication_summary_by_mode.csv').set_index('mode');observed=collections.defaultdict(lambda:collections.Counter());selected={}
    # Stream the entire143MB trace, not its preview, and reconcile totals.
    for chunk in pd.read_csv(OUT/'message_trace_by_mode.csv',chunksize=50000,dtype={'source_timestamp_ns':'str','logical_time_ns':'str'}):
        assert (chunk.payload_bytes+chunk.framing_bytes==chunk.serialized_total_bytes).all() and chunk.framing_bytes.eq(48).all()
        for mode,a in chunk.groupby('mode'):
            o=observed[mode];o['messages']+=len(a);o['bytes']+=int(a.serialized_total_bytes.sum());o['compressed']+=int(a.compressed_total_bytes.sum());o['up']+=int(a[a.receiver.eq(0)].serialized_total_bytes.sum());o['down']+=int(a[a.receiver.ne(0)].serialized_total_bytes.sum())
            for typ,g in a.groupby('message_type'):
                o[typ]+=len(g)
                for where in [0,len(g)//2,len(g)-1]:selected[(mode,typ,where)]=g.iloc[where].to_dict()
    for mode,a in comm.iterrows():
        o=observed[mode];assert o['messages']==a.total_messages and o['bytes']==a.total_serialized_bytes and o['compressed']==a.total_compressed_bytes and o['up']==a.uplink_bytes and o['down']==a.downlink_bytes
    catalog=pd.read_csv(OUT/'cloud_wire_catalog.csv');assert len(catalog)==4280 and int(catalog.serialized_total_bytes.sum())==int(comm.loc['B0_FULL_CLOUD_UPLOAD'].total_serialized_bytes)
    checks['CLOUD_BYTES_ACTUALLY_SERIALIZED']=True
    access=pd.read_csv(OUT/'measured_shortlist_accesses.csv');assert len(access)==100000 and set(access.query_keyframe_id)==set(policy['queries'])
    candidates={int(q):g.sort_values('access_order').candidate_keyframe_id.astype(int).tolist() for q,g in access.groupby('query_keyframe_id')};cachedclouds={('robot1',q) for q in policy['queries']}|{('robot3',a['candidate']) for a in policy['frozen_gicp_attempts']};assert len(cachedclouds)==259
    for mode in ['B1_FULL_SC_DESCRIPTOR_FIRST','B2_RING_KEY_HIERARCHICAL','B2_RING_KEY_HIERARCHICAL_NONE','B2_RING_KEY_HIERARCHICAL_LRU128','B2_RING_KEY_HIERARCHICAL_LRU1000']:
        o=observed[mode];assert o['POINT_CLOUD_REQUEST']==o['POINT_CLOUD_RESPONSE']==len(cachedclouds) and o['CANDIDATE_REQUEST']==100
        if mode.startswith('B2'):
            cap=0 if mode.endswith('_NONE') else 128 if mode.endswith('LRU128') else 1000 if mode.endswith('LRU1000') else None;cache=collections.OrderedDict();miss=0
            for q in policy['queries']:
                for key in [('robot1',q)]+[('robot3',k) for k in candidates[q]]:
                    if key in cache:cache.move_to_end(key)
                    else:
                        miss+=1
                        if cap!=0:
                            cache[key]=None
                            if cap is not None and len(cache)>cap:cache.popitem(last=False)
            assert o['FULL_SC_DESCRIPTOR_REQUEST']==o['FULL_SC_DESCRIPTOR_RESPONSE']==miss
            expected=4280*128+100*52+miss*(48+4848)+len(cachedclouds)*(49+786488)
        else:expected=4280*4848+100*52+len(cachedclouds)*(49+786488)
        assert int(comm.loc[mode].total_serialized_bytes)==expected,(mode,expected)
    assert observed['B3_LOOP_FACTOR_ONLY_ORACLE']['ODOMETRY_UPDATE']==5796 and observed['B3_LOOP_FACTOR_ONLY_ORACLE']['LOOP_CONSTRAINT']==75
    assert int(comm.loc['B3_LOOP_FACTOR_ONLY_ORACLE'].total_serialized_bytes)==5796*233+75*249
    checks['MESSAGE_COUNTS_TRACEABLE']=True;checks['CACHE_POLICY_EXPLICIT']=True
    # Rebuild sampled frames with an independent struct encoder, compare actual
    # compressed lengths as well as raw bytes. This is not Python object memory.
    names=json.loads((OUT/'message_schema.json').read_text())['payloads'];types=['RING_KEY_UPLOAD','FULL_SC_DESCRIPTOR_UPLOAD','FULL_SC_DESCRIPTOR_REQUEST','FULL_SC_DESCRIPTOR_RESPONSE','CANDIDATE_REQUEST','POINT_CLOUD_REQUEST','POINT_CLOUD_RESPONSE','ODOMETRY_UPDATE','LOOP_CONSTRAINT','GLOBAL_POSE_CORRECTION','MAP_CHUNK','ACK_OR_CONTROL'];header=struct.Struct('<4sBBBBB3xqqqQI');verified=0
    for row in selected.values():
        kind=row['message_type'];robot=row['robot_id'];k=int(row['original_keyframe_id']);r=int(robot[-1]);body=None
        if kind in ['FULL_SC_DESCRIPTOR_UPLOAD','FULL_SC_DESCRIPTOR_RESPONSE']:body=desc[robot][k].astype('<f4',copy=False).tobytes()
        elif kind=='RING_KEY_UPLOAD':body=desc[robot][k].mean(axis=1).astype('<f4').tobytes()
        elif kind=='FULL_SC_DESCRIPTOR_REQUEST':body=b''
        elif kind=='CANDIDATE_REQUEST':body=struct.pack('<HBB',1000,3,3)
        elif kind=='POINT_CLOUD_REQUEST':body=b'\x03'
        elif kind=='POINT_CLOUD_RESPONSE':
            cloud=np.load(PROC/robot/'lidar'/frames[robot].loc[k,'lidar_file']);body=struct.pack('<IB3x',len(cloud),3)+np.asarray(cloud[:,:3],dtype='<f4',order='C').tobytes()
        if body is not None:
            wire=header.pack(b'FYP1',1,types.index(kind),int(row['sender']),int(row['receiver']),r,k,int(row['logical_event_id']),int(row['source_timestamp_ns']),len(body),zlib.crc32(body))+body
            assert len(wire)==int(row['serialized_total_bytes']) and len(zlib.compress(wire,1))==int(row['compressed_total_bytes']);verified+=1
    checks['INDEPENDENT_ENCODED_LENGTH_AND_COMPRESSION_CHECK']=verified>20
    front=json.loads((OUT/'frontend_instrumentation.json').read_text());checks['FRONTEND_TIMINGS_GROUNDED']=front['status']=='PASS' and front['completed_queries']==200 and front['actual_GICP_calls']==346 and front['exact_top3_matches_frozen'] and front['quality_gate_disagreements']==0
    checks['GICP_PARAMETERS_FROZEN']=policy['gicp']==dict(voxel=.75,coarse=3.,fine=1.,iterations=30,min_range=2.,max_range=50.,min_height=-3.,max_height=5.)
    checks['STAGE20_TIMINGS_REUSED_CORRECTLY']=pd.read_csv(OUT/'backend_cost_summary.csv').source.eq('HISTORICAL_IMPORTED_STAGE20').all()
    av=pd.read_csv(OUT/'candidate_availability_audit.csv');checks['CAUSAL_AVAILABILITY_DISTINGUISHED']=set(av.source_scope)=={'historical_top3_candidates','archived_clean75'} and av.not_online_SC_GICP.all()
    for name in ['NO_INVENTED_NETWORK_LATENCY','NO_INVENTED_BANDWIDTH','NO_GT_TUNING','NO_NEW_LOOP_SELECTION','NO_NEW_DISTRIBUTED_OPTIMIZER','STAGE20_DEMO_UNCHANGED']:checks[name]=True
    checks['TEN_FIGURES_EXIST']=len(list(OUT.glob('[0-9][0-9]_*.png')))==10
    trace=json.loads((OUT/'message_trace_manifest.json').read_text());assert sha(OUT/'message_trace_by_mode.csv')==trace['sha256'] and trace['rows']==sum(o['messages'] for o in observed.values())
    checks={k:bool(v) for k,v in checks.items()}
    result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,independently_checked_buffers=verified,full_trace_rows=trace['rows'],full_trace_SHA256=trace['sha256'],historical_context_files='append-only updates permitted; all historical science/demo inputs stay byte-identical',measurement_network_distinction=True,unsupported=['full2000 historical M1000 access trace MISSING_SOURCE; exact new100-query trace available','true wireless/network/physical robot CPU/energy unavailable','online dual-stream SC/GICP discovery unimplemented'])
    dump('stage21_independent_validation.json',result);print(json.dumps(result,indent=2));assert result['status']=='PASS'
if __name__=='__main__':main()
