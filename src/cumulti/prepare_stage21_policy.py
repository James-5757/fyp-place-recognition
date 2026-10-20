#!/usr/bin/env python3
"""Gate A: inventory and fsynced predeclared policy, no performance benchmarking."""
import hashlib,json,os,platform,subprocess,sys
from pathlib import Path
import numpy as np,pandas as pd
from stage21_wire import schema
ROOT=Path(__file__).resolve().parents[2];OUT=Path(os.environ.get('CUMULTI_STAGE21_OUTPUT_ROOT',ROOT/'outputs/cumulti_v1/21_cost_baseline'));PROC=Path('/home/cas/CU-Multi/processed_v1/full_2hz');BASE='5e9316b4bdd9c7a6fc412ad0ba5515f5adf23cf2';TEAM=Path('/home/cas/fyp_robot13_geometry_20260915')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(8*1024**2),b''):h.update(b)
    return h.hexdigest()
def dump(name,v):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('w') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
def main():
    assert not OUT.exists() or not any(OUT.iterdir()),'Existing Stage21 outputs are immutable; choose a new explicit experiment, do not overwrite.'
    OUT.mkdir(parents=True,exist_ok=True);historical=ROOT/'outputs/cumulti_v1/09_sc_gicp_integration';lat=pd.read_csv(historical/'integrated_latency.csv');queries=lat.query_keyframe_id.astype(int).tolist();assert len(queries)==100
    interface=pd.read_csv(historical/'candidate_interface.csv')
    pairs=pd.concat([pd.read_csv(p,usecols=['query_keyframe_id','database_keyframe_id','gicp_quality']) for p in [TEAM/'outputs/gicp_rank1.csv',historical/'sc_yaw_new_pairs.csv']],ignore_index=True).rename(columns={'database_keyframe_id':'candidate_keyframe_id','gicp_quality':'GICP_quality'})
    assert not pairs.duplicated(['query_keyframe_id','candidate_keyframe_id']).any()
    print('audit pair traces',len(pairs),'source is 6000-pair teammate file despite rank1 filename',flush=True)
    attempts=[]
    for q in queries:
        for r in interface[interface.query_keyframe_id.eq(q)].sort_values('SC_rank').itertuples():
            matched=pairs[(pairs.query_keyframe_id.eq(q))&(pairs.candidate_keyframe_id.eq(r.candidate_keyframe_id))];assert len(matched)==1,(q,r.candidate_keyframe_id)
            quality=float(matched.GICP_quality.iloc[0]);attempts.append(dict(query=q,candidate=int(r.candidate_keyframe_id),rank=int(r.SC_rank),shift=int(r.best_shift),quality=quality))
            if quality>=.6091:break
    protected=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().strip('\0').split('\0');protected=[ROOT/p for p in protected if p and not p.startswith('outputs/cumulti_v1/21_')]
    hashes={str(p):sha(p) for p in protected if p.is_file()}
    sources=[historical/n for n in ['candidate_interface.csv','identity_vs_sc_yaw_init.csv','rank1_gicp_results.csv','integrated_latency.csv','integrated_latency_summary.json','experiment_config.json']]
    sources += [ROOT/'outputs/cumulti_v1/08_efficient_sc_retrieval'/n for n in ['fixed_M_latency.csv','fixed_M_accuracy.csv','fixed_M_candidate_recall.csv','fixed_M_pareto.csv','exhaustive_retiming.csv']]
    sources += [ROOT/'outputs/cumulti_v1/20_dual_stream'/n for n in ['central_incremental_event_trace.csv','loop_eligibility_schedule.csv','stage20a_backend_decision.json','final_graph_equivalence.json','incremental_vs_batch_diagnostics.csv','backend_state_manifest.json','first_bridge_fusion.json']]
    sources += [ROOT/'outputs/cumulti_v1/12d_clean_backend'/n for n in ['clean_local_trajectory.csv','ordered_rank1_clean_loops.csv','clean_graph_policy.json']]
    sources += [ROOT/'src/cumulti'/n for n in ['run_stage8_efficient_sc_retrieval.py','run_stage9_integrated_latency.py','run_stage2_sc_baseline.py','run_stage5_visual_viewpoint.py','run_stage12d_clean_backend.py']]
    sources += [ROOT/'docs'/n for n in ['CUMULTI_STAGE8_EFFICIENT_SC_RETRIEVAL.md','CUMULTI_STAGE9_SC_GICP_INTEGRATION.md','CUMULTI_STAGE20_DUAL_STREAM.md']]
    sources += [TEAM/'code/robot13_gicp.py',TEAM/'outputs/gicp_rank1.csv',historical/'sc_yaw_new_pairs.csv']
    frames={};descriptors={};sampling={}
    for robot,n in [('robot1',2000),('robot3',4180)]:
        f=pd.read_csv(PROC/robot/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']);assert len(f)==n;frames[robot]=f
        p=PROC/robot/'scan_context_descriptors.npy';a=np.load(p,mmap_mode='r');assert a.dtype==np.float32 and a.shape==(n,20,60)
        sources += [PROC/robot/'keyframes.csv',p,PROC.parent/'ringkey_stage8'/f'{robot}_ringkeys.npy']
        descriptors[robot]=dict(shape=list(a.shape),dtype=str(a.dtype),ring_key_shape=[n,20]);sampling[robot]=np.linspace(0,n-1,20,dtype=int).tolist()
        ids=queries if robot=='robot1' else f.keyframe_id.astype(int).tolist()
        for k in sorted(set(ids)|set(sampling[robot])):sources.append(PROC/robot/'lidar'/f.set_index('keyframe_id').loc[k].lidar_file)
    optional=[dict(role='historical exact M1000 full descriptor access order',path=str(historical/'M1000_full_access_trace.csv'),status='MISSING_SOURCE',resolution='execute fixed Stage8 retrieval on the predeclared 100-query slice, save actual new shortlist accesses; do not invent full historical requests')]
    inventory=[dict(path=str(p),status='AVAILABLE' if p.exists() else 'MISSING_SOURCE',size=p.stat().st_size if p.exists() else None) for p in dict.fromkeys(sources)];assert all(x['status']=='AVAILABLE' for x in inventory)
    sourcehash={x['path']:sha(x['path']) for x in inventory}
    policy=dict(baseline=BASE,source_commits=dict(stage8_accepted='15f5cae174405f4cf5f0c3bba5c2b72e5454361d',stage9_frontend='c52354db7ddc74f7d42ab422486e5eb2922dcf50',stage20=BASE),scenarios=['HISTORICAL_PREBUILT_FRONTEND: all4180 R3 database + original100 R1 query slice; common B0/B1/B2 workload','DUAL_STREAM_ARCHIVED_FACTOR_REPLAY: B3 only,5796 initial/relative odometry updates +75 frozen loops'],queries=queries,frozen_gicp_attempts=attempts,cloud_samples=sampling,M=1000,quality_threshold=.6091,gicp=dict(voxel=.75,coarse=3.,fine=1.,iterations=30,min_range=2.,max_range=50.,min_height=-3.,max_height=5.),warmup=dict(retrieval_queries=10,GICP_pairs=3,descriptor_frames=3),repeats=dict(frontend=2,representative_payload=3),workload_limit=dict(max_wall_seconds=1800,max_queries=100,no_posthoc_expansion=True),compression='zlib1 whole message, lossless; primary uncompressed bytes',cache=dict(primary_descriptors='unbounded server response cache within this finite slice',sensitivity=['NONE',128,1000,'UNBOUNDED'],pointcloud='unbounded logical transferred-cloud cache within slice; physical serializer buffers LRU16',gicp='historical per-call backend LRU64, fresh invocation per frozen attempted pair'),serialization=schema(),timing='perf_counter_ns wall + process_time_ns CPU; first-observed load is OS_CACHE_UNKNOWN, never claimed drop-cache cold; nested exclusive preprocessing = same-call cloud inclusive minus its own np.load time',CPU=dict(platform=platform.platform(),cpu_model=next(x.split(':',1)[1].strip() for x in Path('/proc/cpuinfo').read_text().splitlines() if x.startswith('model name')),logical_cpus=os.cpu_count(),affinity=sorted(os.sched_getaffinity(0)),thread_environment={k:os.getenv(k) for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']},executable=sys.executable),memory='resource.ru_maxrss process high-water MiB; /proc/self/status when applicable; process CPU/wall ratio is utilization, never energy',GT='no GT in scheduling, payloads, retrieval, GICP, graph or tuning; existing historical offline_true_pair read only AFTER fixed sample for descriptive coverage',comparison='B0/B1/B2 common prebuilt100 workload; B3 separate oracle, not percentage-comparable. No end-to-end sum mixing historical frontend and Stage20 backend.',network={k:'NOT_MEASURED' for k in ['NETWORK_BYTES_ACTUALLY_TRANSMITTED','NETWORK_LATENCY','PACKET_LOSS','WIRELESS_BANDWIDTH']},descriptor_shapes=descriptors,stop='mandatory input/roundtrip/retrieval-equivalence/immutability failure stops; missing historical full trace explicitly scoped, never fabricated')
    dump('stage21_input_inventory.json',dict(required=inventory,optional_missing=optional,teammate_large_traces=[dict(path=str(p),size=p.stat().st_size) for p in sorted((TEAM/'outputs').glob('*gicp*.csv'))]))
    dump('stage21_source_provenance.json',dict(baseline=BASE,sources=inventory,optional_missing=optional,protocols=policy['scenarios'],GT_in_payloads=False))
    dump('stage21_input_hashes.json',dict(inputs=sourcehash,protected_historical=hashes))
    dump('message_schema.json',schema());dump('stage21_measurement_policy.json',policy)
    print('GATE_A_POLICY_FSYNCED',len(queries),len(attempts),len(inventory),flush=True)
if __name__=='__main__':main()
