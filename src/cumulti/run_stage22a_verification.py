"""Gate22A-2: durable exact-once actual frozen GICP; pilot then full budget gate.

Pilot outcomes are NEW measurements, reused only at their original causal
event. Pilot never admits loops; chronological replay alone admits duplicates.
An interrupted uncommitted call is deliberately blocked, never silently rerun.
"""
import argparse,fcntl,functools,importlib.util,resource,time
from stage22a_common import *

def actual_call(team,row,path):
    import open3d as o3d
    manifest=path.with_suffix('.manifest.csv')
    pd.DataFrame([dict(query_robot=row['query_robot'],query_keyframe_id=row['query_keyframe_id'],database_robot=row['candidate_robot'],database_keyframe_id=row['candidate_keyframe_id'],rank=1,sc_score=row['SC_score'],best_shift=row['best_shift'])]).to_csv(manifest,index=False)
    args=argparse.Namespace(manifest=manifest,processed_root=PROC.parent,cache_layout='teammate_stage4',output=path,top_k=1,direction=row['query_robot']+'_to_'+row['candidate_robot'],limit=None,pilot_stride=1,resume=False,hourly_rate=None,voxel_size=.75,min_range=2.,max_range=50.,min_height=-3.,max_height=5.,coarse_distance=3.,fine_distance=1.,iterations=30,cache_size=64)
    records=[];oldload=np.load;oldlru=team.lru_cache;reg=o3d.pipelines.registration;oldreg=reg.registration_generalized_icp
    def load(*a,**kw):
        v,ms=timed(lambda:oldload(*a,**kw));records.append(('load',ms));return v
    def lru(maxsize):
        def wrap(fn):
            def f(*a,**kw):
                v,ms=timed(lambda:fn(*a,**kw));records.append(('prep_inclusive',ms));return v
            return functools.lru_cache(maxsize=maxsize)(f)
        return wrap
    def registration(*a,**kw):
        assert a[2] in (3.,1.) and a[5].max_iteration==30
        v,ms=timed(lambda:oldreg(*a,**kw));records.append(('coarse' if a[2]==3 else 'fine',ms));return v
    cpu=time.process_time_ns()
    try:
        team.np.load=load;team.lru_cache=lru;reg.registration_generalized_icp=registration
        _,wall=timed(lambda:team.run(args))
    finally:team.np.load=oldload;team.lru_cache=oldlru;reg.registration_generalized_icp=oldreg
    result=pd.read_csv(path).iloc[0];sums={k:sum(t for n,t in records if n==k) for k in ['load','prep_inclusive','coarse','fine']}
    T=np.asarray(json.loads(result.gicp_transform),float).reshape(4,4)
    assert np.isfinite(T).all() and np.max(abs(T[3]-[0,0,0,1]))<1e-8
    assert np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))<1e-6 and abs(np.linalg.det(T[:3,:3])-1)<1e-6
    q,c,C=canonical(row['query_robot'],row['query_keyframe_id'],row['candidate_keyframe_id'],T)
    return dict(**row,gicp_quality=float(result.gicp_quality),gicp_fitness=float(result.gicp_fitness),gicp_RMSE=float(result.gicp_inlier_rmse),verified_transform=T.tolist(),canonical_transform=C.tolist(),canonical_robot1_kf=int(q),canonical_robot3_kf=int(c),call_wall_ms=wall,call_cpu_ms=(time.process_time_ns()-cpu)/1e6,cloud_load_decode_ms=sums['load'],cloud_preprocess_ms=sums['prep_inclusive']-sums['load'],coarse_ms=sums['coarse'],fine_ms=sums['fine'],other_harness_ms=wall-sums['prep_inclusive']-sums['coarse']-sums['fine'],peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,result_csv_sha256=sha(path),geometrically_accepted=bool(result.gicp_quality>=.6091))

def attempt_id(row):return f"e{int(row['event_index']):04d}_r{int(row['candidate_rank'])}"
def attempt(team,row,phase,used,cap):
    ident=attempt_id(row);receipt=OUT/'attempts'/f'{ident}.json';raw=OUT/'attempts'/f'{ident}.csv'
    if receipt.exists():
        x=json.loads(receipt.read_text());assert x['candidate_identity']==[row[k] for k in ['event_index','candidate_rank','query_robot','query_keyframe_id','candidate_robot','candidate_keyframe_id','best_shift']]
        assert sha(raw)==x['measurement']['result_csv_sha256'];return x['measurement'],used
    assert not raw.exists() and not (OUT/'attempts'/f'{ident}.intent.json').exists(),f'UNRESOLVED_INFLIGHT_ATTEMPT: {ident}; do not rerun automatically'
    if used>=cap:raise TimeoutError('predeclared GICP budget exhausted')
    if resource.getrusage(resource.RUSAGE_SELF).ru_maxrss>8*1024*1024:raise MemoryError('predeclared8GiB process RSS budget exceeded')
    dump(f'attempts/{ident}.intent.json',dict(status='CALL_INTENT',phase=phase,event=row['event_index'],rank=row['candidate_rank'],policy_sha256=sha(OUT/'stage22a_policy.json')))
    measurement=actual_call(team,row,raw);used+=measurement['call_wall_ms']/1000
    dump(f'attempts/{ident}.json',dict(phase=phase,candidate_identity=[row[k] for k in ['event_index','candidate_rank','query_robot','query_keyframe_id','candidate_robot','candidate_keyframe_id','best_shift']],measurement=measurement))
    return measurement,used

def saved_usage():
    return sum(json.loads(p.read_text())['measurement']['call_wall_ms']/1000 for p in (OUT/'attempts').glob('e*_r?.json'))

def main(phase):
    lock=(OUT/'verification.lock').open('a')
    fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    verify_freeze();assert read('stage22a1_decision.json')['status']=='CAUSAL_RETRIEVAL_READY';assert read('synthetic_unit_tests.json')['status']=='PASS';online_guard()
    spec=importlib.util.spec_from_file_location('frozen_stage22a_teammate',TEAM);team=importlib.util.module_from_spec(spec);spec.loader.exec_module(team)
    trace=pd.read_csv(OUT/'online_candidate_trace.csv');groups={int(i):g.sort_values('candidate_rank').to_dict('records') for i,g in trace.groupby('event_index')}
    events=pd.read_csv(OUT/'dual_stream_event_schedule.csv').to_dict('records');policy=read('stage22a_policy.json');budget=read('stage22a_execution_budget.json');cap=budget['full_GICP_execution_cap_seconds'];used=saved_usage();assert used<cap
    selected=policy['pilot_event_indices'] if phase=='pilot' else list(range(5796))
    if phase=='full' and not read('gicp_pilot_summary.json')['full_run_fits_budget']:
        # Budget outcome is not permission to enlarge the cap or claim full
        # verification. Retain the scattered pilot as pilot only, not admitted
        # chronological online factors.
        pilot=pd.read_csv(OUT/'gicp_pilot_timing.csv');csv('online_gicp_attempts.csv',pilot)
        csv('online_verified_loops.csv',pd.DataFrame(columns=list(pilot.columns)+['discovery_event','discovery_normalized_ns','duplication_status','loop_id']))
        csv('online_loop_discovery_trace.csv',pd.DataFrame(columns=list(pilot.columns)+['discovery_event','discovery_normalized_ns','duplication_status','loop_id']))
        csv('online_rejected_candidates.csv',pilot[~pilot.geometrically_accepted]);csv('online_verification_queries.csv',pd.read_csv(OUT/'gicp_pilot_queries.csv'))
        coverage=dict(status='FRONTEND_VERIFICATION_INCOMPLETE',reason='frozen pilot model exceeds unmodified7200s budget; full phase NOT_RUN',queries_verified=150,events_total=5796,fraction=150/5796,coverage_kind='scattered deterministic pilot only, NOT full chronological verification',actual_calls=len(pilot),accepted_candidates=int(pilot.geometrically_accepted.sum()),rejected_candidates=int((~pilot.geometrically_accepted).sum()),independent_loops=0,duplicates=0,actual_call_wall_seconds=float(pilot.call_wall_ms.sum()/1000),budget_used_seconds=used)
        dump('frontend_execution_coverage.json',coverage);dump('stage22a2_decision.json',dict(status='FRONTEND_VERIFICATION_INCOMPLETE',coverage=coverage,GT_used=False,archived_loops_used=False));print('FRONTEND_VERIFICATION_INCOMPLETE: pilot exceeds budget',flush=True);return
    accepted=[];attempts=[];discoveries=[];queries=[];complete=True
    t=time.monotonic();prior_duration=used
    try:
        for index in selected:
            e=events[index];outcome='NO_CANDIDATE';calls=0;chosen=None
            for row in groups.get(index,[]):
                m,used=attempt(team,row,phase,used,cap);attempts.append(m);calls+=1
                if m['geometrically_accepted']:
                    chosen=m;outcome='FRONTEND_GEOMETRICALLY_ACCEPTED';break
            qrow=dict(event_index=index,normalized_time_ns=e['normalized_ns'],query_robot=e['robot_id'],query_keyframe_id=e['keyframe_id'],gicp_calls=calls,outcome=outcome)
            if chosen is None and calls:qrow['outcome']='NO_ACCEPTED_LOOP'
            if chosen is not None and phase=='full':
                q,c=chosen['canonical_robot1_kf'],chosen['canonical_robot3_kf'];dup=duplicate(q,c,accepted)
                if not dup:accepted.append((q,c))
                discoveries.append(dict(chosen,discovery_event=index,discovery_normalized_ns=e['normalized_ns'],duplication_status='SUPPRESSED_CAUSAL_DUPLICATE' if dup else 'BACKEND_GRAPH_ADMISSIBLE',loop_id=None if dup else len(accepted)))
            queries.append(qrow)
            if len(queries)%100==0:
                dump('verification_progress.json',dict(phase=phase,completed_queries=len(queries),last_event=index,actual_call_seconds_used=used,unique_actual_calls=len(list((OUT/'attempts').glob('e*_r?.json'))),accepted_representatives=len(accepted),elapsed_invocation_s=time.monotonic()-t))
                print('GICP',phase,len(queries),'calls',len(attempts),'newloops',len(accepted),'budget_s',round(used,1),flush=True)
    except (TimeoutError,MemoryError):complete=False
    if phase=='pilot':
        csv('gicp_pilot_pairs.csv',[row for index in selected for row in groups.get(index,[])])
        csv('gicp_pilot_timing.csv',[dict(m,verified_transform=json.dumps(m['verified_transform']),canonical_transform=json.dumps(m['canonical_transform'])) for m in attempts]);csv('gicp_pilot_queries.csv',queries)
        projected=sum(m['call_wall_ms'] for m in attempts)/1000/150*5796
        dump('gicp_pilot_summary.json',dict(status='PILOT_COMPLETE' if complete and len(queries)==150 else 'PILOT_INCOMPLETE',queries=len(queries),actual_calls=len(attempts),direction_counts={r:sum(q['query_robot']==r for q in queries) for r in STARTS},actual_harness_seconds=sum(m['call_wall_ms'] for m in attempts)/1000,projection_kind='MODELLED_PROJECTION',projected_full_seconds=projected,frozen_cap_seconds=cap,full_run_fits_budget=bool(complete and projected<=cap),pilot_results_reused_exactly_once=True,elapsed_s=time.monotonic()-t))
        print('PILOT',projected,'s modelled full',flush=True);return
    serial=lambda rows:[dict(m,verified_transform=json.dumps(m['verified_transform']),canonical_transform=json.dumps(m['canonical_transform'])) for m in rows]
    measurement_cols=list(attempts[0]) if attempts else list(trace.columns)+['geometrically_accepted','canonical_robot1_kf','canonical_robot3_kf','verified_transform','canonical_transform','call_wall_ms']
    discovery_cols=measurement_cols+['discovery_event','discovery_normalized_ns','duplication_status','loop_id']
    csv('online_verified_loops.csv',pd.DataFrame(serial([m for m in discoveries if m['duplication_status']=='BACKEND_GRAPH_ADMISSIBLE']),columns=discovery_cols))
    csv('online_rejected_candidates.csv',pd.DataFrame(serial([m for m in attempts if not m['geometrically_accepted']]),columns=measurement_cols))
    csv('online_loop_discovery_trace.csv',pd.DataFrame(serial(discoveries),columns=discovery_cols));csv('online_verification_queries.csv',queries);csv('online_gicp_attempts.csv',serial(attempts))
    status='ONLINE_GICP_VERIFICATION_READY' if complete and len(queries)==5796 else 'FRONTEND_VERIFICATION_INCOMPLETE'
    dump('frontend_execution_coverage.json',dict(status=status,events_total=5796,queries_verified=len(queries),fraction=len(queries)/5796,actual_calls=len(attempts),accepted_candidates=sum(m['geometrically_accepted'] for m in attempts),rejected_candidates=sum(not m['geometrically_accepted'] for m in attempts),independent_loops=len(accepted),duplicates=len(discoveries)-len(accepted),actual_call_wall_seconds=sum(m['call_wall_ms'] for m in attempts)/1000,physical_actual_calls_including_pilot=len(list((OUT/'attempts').glob('e*_r?.json'))),budget_used_seconds=used,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024))
    dump('stage22a2_decision.json',dict(status=status,coverage=read('frontend_execution_coverage.json'),frozen_results={n:sha(OUT/n) for n in ['online_verified_loops.csv','online_rejected_candidates.csv','online_loop_discovery_trace.csv','online_gicp_attempts.csv']},GT_used=False,archived_loops_used=False))
    print(status,flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('phase',choices=['pilot','full']);main(p.parse_args().phase)
