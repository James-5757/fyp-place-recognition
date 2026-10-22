"""Gate22A-1: two independently recomputed causal SC passes, no loop oracle."""
import hashlib,json,resource,time
import numpy as np
from stage22a_common import *

def one_pass(f,events,repeat):
    db={r:Database() for r in STARTS};frozen={r:np.load(PROC/r/'scan_context_descriptors.npy',mmap_mode='r') for r in STARTS}
    rows=[];lat=[];growth=[];scientific=hashlib.sha256()
    for e in events:
        r=e['robot_id'];other='robot3' if r=='robot1' else 'robot1';k=e['keyframe_id']
        cloud,load=timed(lambda:np.load(PROC/r/'lidar'/f[r].loc[k,'lidar_file'],allow_pickle=False))
        d,compute=timed(lambda:s2.descriptor(cloud));assert np.array_equal(d,frozen[r][k]),('descriptor mismatch',r,k)
        _,ring=timed(lambda:s8.ring_key(d[None]))
        candidates,metrics=db[other].query(d)
        sizes={rr:len(x.ids) for rr,x in db.items()}
        for c in candidates:
            assert c['candidate_keyframe_id'] in db[other].ids
            row=dict(event_index=e['event_index'],normalized_time_ns=e['normalized_ns'],query_robot=r,query_keyframe_id=k,query_original_timestamp_ns=e['timestamp_ns'],current_R1_database_size=sizes['robot1'],current_R3_database_size=sizes['robot3'],candidate_robot=other,candidate_original_timestamp_ns=int(f[other].loc[c['candidate_keyframe_id'],'lidar_timestamp_ns']),candidate_available_at_query=True,all_candidate_ids_arrived=True,**c,**metrics)
            rows.append(row)
            scientific.update(json.dumps({key:row[key] for key in ['event_index','query_robot','query_keyframe_id','candidate_robot','candidate_keyframe_id','candidate_rank','SC_score','best_shift']},sort_keys=True).encode())
        _,insert=timed(lambda:db[r].insert(k,d))
        lat.append(dict(event_index=e['event_index'],query_robot=r,query_keyframe_id=k,normalized_time_ns=e['normalized_ns'],input_load_ms=load,descriptor_compute_ms=compute,ring_key_compute_ms=ring,own_database_insert_ms=insert,effective_M=min(1000,sizes[other]),effective_TopK=len(candidates),status='CANDIDATES' if candidates else 'NO_CANDIDATE',**metrics))
        growth.append(dict(event_index=e['event_index'],normalized_time_ns=e['normalized_ns'],query_robot=r,robot1_size=len(db['robot1'].ids),robot3_size=len(db['robot3'].ids)))
        if e['event_index']%500==0:print('SC_PASS',repeat,e['event_index'],flush=True)
    assert [len(db[r].ids) for r in STARTS]==[1850,3946]
    return rows,lat,growth,scientific.hexdigest()

def main():
    verify_freeze();assert not (OUT/'stage22a1_decision.json').exists();online_guard()
    f=frames();events=events_for(f);started=time.monotonic()
    a,lat,growth,h=one_pass(f,events,1)
    _,lat2,_,h2=one_pass(f,events,2);assert h==h2,'non-deterministic scientific trace'
    csv('online_candidate_trace.csv',a);csv('online_retrieval_latency.csv',lat);csv('database_growth.csv',growth);csv('retrieval_repeat_latency.csv',lat2)
    csv('causal_timeline_audit.csv',[dict(event_index=x['event_index'],candidate_count=x['effective_TopK'],future_candidate_count=0,status='PASS') for x in lat])
    summary=dict(status='CAUSAL_RETRIEVAL_READY',events=len(events),queries={'robot1_to_robot3':1850,'robot3_to_robot1':3946},candidate_rows=len(a),no_candidate_queries=sum(not x['effective_TopK'] for x in lat),future_candidates=0,descriptor_bitwise_checks=2*len(events),two_pass_candidate_sha256=[h,h2],repeat_deterministic=True,KD='rebuilt per query, cost included',retrieval_wall_ms=sum(x['total_retrieval_ms'] for x in lat),descriptor_compute_ms=sum(x['descriptor_compute_ms'] for x in lat),elapsed_two_pass_s=time.monotonic()-started,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,GT_access=False,archive_access=False)
    dump('bidirectional_retrieval_summary.json',summary)
    dump('stage22a1_decision.json',dict(status='CAUSAL_RETRIEVAL_READY',candidate_trace_sha256=sha(OUT/'online_candidate_trace.csv'),scientific_sha256=h,requirements=['5796 actual arrivals','two dynamic databases','query before own append','no future indices','bitwise descriptors','two real-scan deterministic passes','runtime oracle access guard'],GT_used=False))
    print(json.dumps(summary),flush=True)
if __name__=='__main__':main()
