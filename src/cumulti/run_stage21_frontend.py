#!/usr/bin/env python3
"""Gate D: instrument unchanged Stage8 scoring and actual teammate GICP calls."""
import functools,json,resource,time
from pathlib import Path
import numpy as np,pandas as pd
from scipy.spatial import cKDTree
from prepare_stage21_policy import ROOT,OUT,PROC,dump
import run_stage8_efficient_sc_retrieval as s8
import run_stage2_sc_baseline as s2
import run_stage9_integrated_latency as s9

def timed(fn):
    a=time.perf_counter_ns();c=time.process_time_ns();v=fn();return v,(time.perf_counter_ns()-a)/1e6,(time.process_time_ns()-c)/1e6

def measured_call(team,row,out):
    import open3d as o3d
    records=[];original_load=np.load;original_lru=team.lru_cache;reg=o3d.pipelines.registration;original_reg=reg.registration_generalized_icp
    def load(*args,**kwargs):
        result,ms,cpu=timed(lambda:original_load(*args,**kwargs));records.append(dict(operation='cloud_load_decode',wall_ms=ms,cpu_ms=cpu,path=str(args[0])));return result
    def lru(maxsize):
        def decorator(fn):
            def measured(*args,**kwargs):
                result,ms,cpu=timed(lambda:fn(*args,**kwargs));records.append(dict(operation='cloud_preprocess_inclusive',wall_ms=ms,cpu_ms=cpu,points=len(result.points)));return result
            return functools.lru_cache(maxsize=maxsize)(measured)
        return decorator
    def registration(*args,**kwargs):
        distance=float(args[2]);assert distance in (3.,1.) and args[5].max_iteration==30
        result,ms,cpu=timed(lambda:original_reg(*args,**kwargs));records.append(dict(operation='GICP_coarse' if distance==3 else 'GICP_fine',wall_ms=ms,cpu_ms=cpu,source_points=len(args[0].points),target_points=len(args[1].points)));return result
    try:
        team.np.load=load;team.lru_cache=lru;reg.registration_generalized_icp=registration
        result,wall,cpu=timed(lambda:s9.call(team,row,out))
    finally:team.np.load=original_load;team.lru_cache=original_lru;reg.registration_generalized_icp=original_reg
    sums={name:sum(r['wall_ms'] for r in records if r['operation']==name) for name in ['cloud_load_decode','cloud_preprocess_inclusive','GICP_coarse','GICP_fine']}
    cpus={name:sum(r['cpu_ms'] for r in records if r['operation']==name) for name in sums}
    prep=sums['cloud_preprocess_inclusive']-sums['cloud_load_decode'];assert prep>=-1e-6
    quality,decision_ms,decision_cpu=timed(lambda:float(result.gicp_fitness)/(1+float(result.gicp_inlier_rmse))>=.6091)
    return result,dict(call_wall_ms=wall,call_cpu_ms=cpu,cloud_load_decode_ms=sums['cloud_load_decode'],cloud_preprocess_ms=prep,coarse_ms=sums['GICP_coarse'],fine_ms=sums['GICP_fine'],registration_ms=sums['GICP_coarse']+sums['GICP_fine'],manifest_index_CSV_and_other_ms=wall-sums['cloud_preprocess_inclusive']-sums['GICP_coarse']-sums['GICP_fine'],decision_arithmetic_micro_ms=decision_ms,quality_accepted=quality,measured_process_cpu_utilization_percent=cpu/wall*100,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024),records

def main():
    assert json.loads((OUT/'message_serialization_tests.json').read_text())['status']=='PASS'
    assert json.loads((OUT/'payload_catalog_validation.json').read_text())['status']=='PASS'
    assert not (OUT/'frontend_benchmark_queries.csv').exists()
    policy=json.loads((OUT/'stage21_measurement_policy.json').read_text());queries=policy['queries'];started=time.monotonic();team=s9.team_module()
    desc={r:np.load(PROC/r/'scan_context_descriptors.npy') for r in ['robot1','robot3']};frames={r:pd.read_csv(PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).set_index('keyframe_id') for r in desc}
    dbu,valid=s8.s5.normalize_columns(desc['robot3']);keys={r:s8.ring_key(desc[r]) for r in desc};tree,tree_ms,tree_cpu=timed(lambda:cKDTree(keys['robot3']));ops=[dict(query_keyframe_id=-1,robot_id='robot3',repeat=0,operation='one_time_KD_tree_build',wall_ms=tree_ms,cpu_ms=tree_cpu)]
    interface=pd.read_csv(ROOT/'outputs/cumulti_v1/09_sc_gicp_integration/candidate_interface.csv');expected={int(q):g.sort_values('SC_rank') for q,g in interface.groupby('query_keyframe_id')}
    for q in queries[:10]:s9.retrieve(desc['robot1'][q],dbu,valid,keys['robot1'][q],tree)
    work=OUT/'frontend_pairs';work.mkdir()
    for j,a in enumerate(policy['frozen_gicp_attempts'][:3]):
        row=type('Row',(),dict(qid=a['query'],cid=a['candidate'],rank=a['rank'],score=0.,shift=a['shift']));s9.call(team,row,work/f'warmup_{j}.csv')
    # Descriptor extraction is separate from retrieval over frozen cached data.
    for robot,ids in [('robot1',queries),('robot3',policy['cloud_samples']['robot3'])]:
        for k in ids[:3]:s2.descriptor(np.load(PROC/robot/'lidar'/frames[robot].loc[k,'lidar_file']))
        for rep in range(2):
            for k in ids:
                p=PROC/robot/'lidar'/frames[robot].loc[k,'lidar_file'];cloud,load_ms,load_cpu=timed(lambda:np.load(p,allow_pickle=False));d,ms,cpu=timed(lambda:s2.descriptor(cloud));assert np.array_equal(d,desc[robot][k]),(robot,k,'frozen SC mismatch')
                ops.append(dict(query_keyframe_id=k,robot_id=robot,repeat=rep,operation='descriptor_input_load_FIRST_OBSERVED_OS_CACHE_UNKNOWN' if rep==0 else 'descriptor_input_load_WARM_REPEAT_OS_CACHE_NOT_CONTROLLED',wall_ms=load_ms,cpu_ms=load_cpu))
                ops.append(dict(query_keyframe_id=k,robot_id=robot,repeat=rep,operation='SC_descriptor_extraction',wall_ms=ms,cpu_ms=cpu))
                ring,ms,cpu=timed(lambda:s8.ring_key(d[None]));assert np.array_equal(ring[0],keys[robot][k]);ops.append(dict(query_keyframe_id=k,robot_id=robot,repeat=rep,operation='Ring_Key_construction',wall_ms=ms,cpu_ms=cpu))
    attempts={q:[a for a in policy['frozen_gicp_attempts'] if a['query']==q] for q in queries};rows=[];pairs=[];access=[];checks=[]
    for rep in range(2):
        for q in queries:
            if time.monotonic()-started>policy['workload_limit']['max_wall_seconds']:
                dump('frontend_incomplete.json',dict(status='INCOMPLETE',reason='predeclared wall limit',completed_queries=len(rows)));break
            start=time.perf_counter_ns();cpustart=time.process_time_ns()
            idx,ms,cpu=timed(lambda:tree.query(keys['robot1'][q],k=1000,workers=1)[1]);idx=np.asarray(idx,int);ops.append(dict(query_keyframe_id=q,robot_id='robot1',repeat=rep,operation='Ring_Key_KD_shortlist',wall_ms=ms,cpu_ms=cpu))
            result,sc_ms,sc_cpu=timed(lambda:s8.score(desc['robot1'][q],dbu[idx],valid[idx]));scores,shifts=result
            order,sort_ms,sort_cpu=timed(lambda:np.argsort(-scores,kind='stable')[:3]);ranked=idx[order]
            assert np.array_equal(ranked,expected[q].candidate_keyframe_id.to_numpy()) and np.array_equal(shifts[order],expected[q].best_shift.to_numpy()),('retrieval mismatch',q)
            if rep==0:access.extend(dict(query_keyframe_id=q,access_order=j,candidate_keyframe_id=int(k)) for j,k in enumerate(idx))
            for name,wall,c in [('full_SC_similarity_and_yaw',sc_ms,sc_cpu),('top3_stable_sort',sort_ms,sort_cpu)]:ops.append(dict(query_keyframe_id=q,robot_id='robot1',repeat=rep,operation=name,wall_ms=wall,cpu_ms=c))
            retrieval_ms=(time.perf_counter_ns()-start)/1e6;reg_ms=0.;rank1_ms=None;matching=True
            for a in attempts[q]:
                r=a['rank'];row=type('Row',(),dict(qid=q,cid=a['candidate'],rank=r,score=float(scores[order[r-1]]),shift=a['shift']));record,m,detail=measured_call(team,row,work/f'rep{rep}_q{q}_r{r}.csv');reg_ms+=m['registration_ms'];matching &= (float(record.gicp_quality)>=.6091)==(a['quality']>=.6091)
                pairs.append(dict(query_keyframe_id=q,candidate_keyframe_id=a['candidate'],SC_rank=r,repeat=rep,historical_quality=a['quality'],measured_quality=float(record.gicp_quality),frozen_request_policy=True,**m))
                for rec in detail:ops.append(dict(query_keyframe_id=q,robot_id='central',repeat=rep,operation=rec['operation'],wall_ms=rec['wall_ms'],cpu_ms=rec['cpu_ms']))
                if r==1:rank1_ms=(time.perf_counter_ns()-start)/1e6
            rows.append(dict(query_keyframe_id=q,repeat=rep,retrieval_ms=retrieval_ms,rank1_actual_harness_ms=rank1_ms,frozen_top3_trace_harness_ms=(time.perf_counter_ns()-start)/1e6,gicp_calls=len(attempts[q]),registration_ms=reg_ms,process_cpu_ms=(time.process_time_ns()-cpustart)/1e6,quality_gate_decisions_match_historical=matching,scope='NEW_MEASURED_SLICE_HISTORICAL_PREBUILT_FRONTEND_FROZEN_REQUEST_TRACE',loading='fresh per-pair backend cache; OS page cache not controlled'))
            if len(rows)%20==0:print('GATE_D actual frontend queries',len(rows),'of200',flush=True)
        else:continue
        break
    pd.DataFrame(rows).to_csv(OUT/'frontend_benchmark_queries.csv',index=False);pd.DataFrame(pairs).to_csv(OUT/'frontend_pair_operations.csv',index=False);pd.DataFrame(ops).to_csv(OUT/'frontend_operation_timings.csv',index=False);pd.DataFrame(access).to_csv(OUT/'measured_shortlist_accesses.csv',index=False)
    dump('frontend_instrumentation.json',dict(status='PASS' if len(rows)==200 else 'INCOMPLETE',completed_queries=len(rows),unique_queries=len(set(r['query_keyframe_id'] for r in rows)),actual_GICP_calls=len(pairs),warmup_excluded=True,exact_top3_matches_frozen=True,descriptor_extraction_bitwise_matches_frozen=True,quality_gate_disagreements=sum(not r['quality_gate_decisions_match_historical'] for r in rows),algorithm_files_modified=False,instrumentation='temporary wrappers around actual np.load, per-miss cloud decorator and Open3D coarse/fine calls; same-call nested timing, no unrelated-mean subtraction',timing_overhead_included=True,GT_used=False,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024))
    assert len(rows)==200;print('GATE_D_PASS',len(rows),len(pairs),flush=True)
if __name__=='__main__':main()
