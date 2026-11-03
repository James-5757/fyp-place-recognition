"""Freeze, GT-free prefix features, bounded pilot and exact frozen Batch ablations."""
from diagnostic_model import *

def metadata(loops,arc):
    index={n:{identity(r):r for r in f.itertuples()} for n,f in loops.items()};sets={n:set(d) for n,d in index.items()}
    assert [len(sets[P0]),len(sets[P5]),len(sets[P0]&sets[P5]),len(sets[P0]-sets[P5]),len(sets[P5]-sets[P0])]==[288,231,147,141,84]
    rows=[]
    for n in [P0,P5]:
        past=[]
        for r in loops[n].sort_values('discovery_event').itertuples():
            q,c=identity(r);s1,s3=arc['robot1',q],arc['robot3',c]
            near=min(past,key=lambda x:(np.hypot((s1-x[2])/50,(s3-x[3])/50),x[0],x[1])) if past else None
            rows.append(dict(policy=n,robot1_kf=q,robot3_kf=c,discovery_event=int(r.discovery_event),normalized_s=int(r.discovery_normalized_ns)/1e9,quality=float(r.gicp_quality),R1_path_m=s1,R3_path_m=s3,R1_bin50=int(s1//50),R3_bin50=int(s3//50),first_bridge=int(r.loop_id)==1,global_cut_edge=False,nearest_previously_admitted_pair=None if near is None else json.dumps(near[:2]),nearest_prior_normalized_path_distance=None if near is None else float(np.hypot((s1-near[2])/50,(s3-near[3])/50)),membership='COMMON' if (q,c) in sets[P0]&sets[P5] else 'P0_ONLY' if n==P0 else 'P5_ONLY'))
            past.append((q,c,s1,s3))
    return rows,index,sets

def freeze():
    assert not (OUT/'stage23b_policy.json').exists(),'Refuse existing Stage23B policy'
    loops,frame,nodes,local,arc,events,params=load();rows,index,sets=metadata(loops,arc)
    missing=sorted([r for r in rows if r['membership']=='P0_ONLY'],key=lambda r:r['normalized_s']);groups=[];previous=None
    for r in missing:
        if previous is None or r['normalized_s']-previous>30:groups.append([])
        r['temporal_cluster']=len(groups)-1;groups[-1].append([r['robot1_kf'],r['robot3_kf']]);previous=r['normalized_s']
    clusters={(r['robot1_kf'],r['robot3_kf']):r['temporal_cluster'] for r in missing}
    p5only=sorted([r for r in rows if r['membership']=='P5_ONLY'],key=lambda r:r['normalized_s']);pc=-1;previous=None
    for r in p5only:
        if previous is None or r['normalized_s']-previous>30:pc+=1
        r['temporal_cluster']=pc;previous=r['normalized_s']
    p5regions={(r['R1_bin50'],r['R3_bin50']) for r in rows if r['policy']==P5};uncovered=[r for r in missing if (r['R1_bin50'],r['R3_bin50']) not in p5regions]
    assert len(groups)==7 and len(uncovered)==4
    for r in rows:
        if r['membership']=='P0_ONLY':r['temporal_cluster']=clusters[r['robot1_kf'],r['robot3_kf']]
        r['P5_region_uncovered']=(r['R1_bin50'],r['R3_bin50']) not in p5regions
    cases=[dict(name='BASE_P0',policy=P0,remove=[],kind='BASELINE'),dict(name='BASE_P5',policy=P5,remove=[],kind='BASELINE')]
    for r in missing:cases.append(dict(name=f"LOO_P0_{r['robot1_kf']}_{r['robot3_kf']}",policy=P0,remove=[[r['robot1_kf'],r['robot3_kf']]],kind='LOO_P0',cluster=r['temporal_cluster']))
    for i,g in enumerate(groups):cases.append(dict(name=f'GROUP_CLUSTER_{i}',policy=P0,remove=g,kind='TEMPORAL_GROUP',cluster=i))
    cases.extend([dict(name='P0_COMMON_ONLY',policy=P0,remove=[list(x) for x in sorted(sets[P0]-sets[P5])],kind='SUPPORT_GROUP'),dict(name='P5_COMMON_ONLY',policy=P5,remove=[list(x) for x in sorted(sets[P5]-sets[P0])],kind='SUPPORT_GROUP')])
    for i in np.linspace(0,len(p5only)-1,7,dtype=int):
        r=p5only[int(i)];cases.append(dict(name=f"LOO_P5_{r['robot1_kf']}_{r['robot3_kf']}",policy=P5,remove=[[r['robot1_kf'],r['robot3_kf']]],kind='LOO_P5'))
    pilot={f'LOO_P0_{g[len(g)//2][0]}_{g[len(g)//2][1]}' for g in groups}
    pilot.update(f"LOO_P0_{r['robot1_kf']}_{r['robot3_kf']}" for r in uncovered)
    pilot.update(f"LOO_P0_{r['robot1_kf']}_{r['robot3_kf']}" for r in sorted(missing,key=lambda r:r['nearest_prior_normalized_path_distance'])[:3])
    oldpaths=list(OUT.glob('*'))+[ROOT/'docs/CUMULTI_STAGE23_INFORMATION_AWARE.md',ROOT/'scripts/run_stage23_information_aware.sh',ROOT/'NEXT_STEPS.md',ROOT/'RESULTS.md',ROOT/'EXPERIMENT_LOG.md']+list((SRC/'stage23').glob('preflight.py'))+list((SRC/'stage23').glob('finalize_plan.py'))
    import preflight
    historical=preflight.historical_digest()
    policy=dict(status='FROZEN_BEFORE_DIAGNOSTICS',cases=cases,pilot_names=sorted(pilot),budget=dict(total_solver_wall_cap_s=900,feature_cap_s=180,RSS_cap_GiB=2,pilot_full_projection_limit_s=900),reference_reproduction_translation_max_m=1e-7,reference_reproduction_rotation_max_rad=1e-7,initialization='EXACT original per-policy first bridge/local odometry initialization; original chronological graph factor order',LM_parameters=read(B/'policies'/P0/'online_backend_policy.json')['LM'],region='union of frozen local odometry +/-50m around removed endpoints, evaluated with ONE joint rigid alignment; no independent regional alignment',GT_inter_robot_error='fixed union372 measured pair IDs; CSV reference Pose3 convention unchanged from Stage22B; no new extrinsic assumption or retuning',GT_free_features='strict earlier P0 factor prefix, common-odometry linearization after first bridge; full joint cross-correlation; predicted measurement=current causal pose between, not actual current GICP transform',GT_free_scope='causal prefix-odometry proxy, not optimized ISAM2 posterior or calibrated physical uncertainty; observed current quality is separate realized diagnostic',map_protocol=dict(frame_stride=10,point_stride=16,voxel_m=.5,range_m=[1,60]),map_cases='baselines, seven temporal groups, both common-only support groups and seven selected P5 removals; full LOO maps not regenerated',criticality_primary='signed delta joint ATE; regional metrics are paired same-cohort deltas; TopK positives require delta>0, no fitted materiality threshold',final_independent_validation='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE',no_selector=True,GT_freeze='GT-free feature file and all solve plans/results written before offline GT evaluator',GT_free_evaluation_pairs='all372 known union pairs; no unknown GICP imputation')
    dump('stage23b_policy.json',policy)
    dump('stage23b_input_manifest.json',dict(preserved_Stage23A={str(p):sha(p) for p in oldpaths if p.is_file()},historical=historical,inputs={str(p):sha(p) for p in [B/'policies'/n/'online_verified_loops.csv' for n in [P0,P5]]+[ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv',A/'dual_stream_event_schedule.csv',B/'stage22b_experiment_policy.json']},policy_sha256=sha(OUT/'stage23b_policy.json')))
    csv('stage23b_GT_free_loop_metadata.csv',rows);print('STAGE23B_FROZEN',len(cases),'cases',len(pilot),'pilot',flush=True)

def features():
    guard();loops,frame,nodes,local,arc,events,params=load();rows,index,sets=metadata(loops,arc);union={**index[P5],**index[P0]};targets={}
    for qc,r in union.items():targets.setdefault(int(r.discovery_event),[]).append(qc)
    base,robust=noise();graph=gtsam.NonlinearFactorGraph();values=gtsam.Values();first=loops[P0].iloc[0];start=int(first.discovery_event);prefix=set();past=[];S=None;out=[];beg=time.monotonic();policy=read(OUT/'stage23b_policy.json')
    selected={}
    for chunk in pd.read_csv(B/'causal_feature_table.csv',usecols=['policy','event_index','candidate_rank','estimated_transfer_bytes','predicted_cpu_ms'],chunksize=50000):
        for r in chunk[(chunk.policy==P0)&chunk.event_index.isin(targets)].itertuples():selected[int(r.event_index),int(r.candidate_rank)]=(int(r.estimated_transfer_bytes),float(r.predicted_cpu_ms))
    p0events={int(r.discovery_event):r for r in loops[P0].itertuples()}
    def add(n):
        values.insert(key(n),gtsam.Pose3(local[n] if n[0]=='robot1' else S@local[n]))
        if n[1]>(150 if n[0]=='robot1' else 234):graph.add(gtsam.BetweenFactorPose3(key((n[0],n[1]-1)),key(n),gtsam.Pose3(inv(local[n[0],n[1]-1])@local[n]),base))
    for e in events:
        n=(e['robot_id'],e['keyframe_id']);prefix.add(n);i=e['event_index']
        if S is not None:add(n)
        for qc in targets.get(i,[]):
            if time.monotonic()-beg>policy['budget']['feature_cap_s']:raise TimeoutError('frozen feature budget')
            q,c=qc;r=union[qc];bytes_,cpu=selected[i,int(r.candidate_rank)];near=min((float(np.hypot((arc['robot1',q]-arc['robot1',a])/50,(arc['robot3',c]-arc['robot3',b])/50)) for a,b,_ in past),default=None)
            priorbins={(int(arc['robot1',a]//50),int(arc['robot3',b]//50)) for a,b,_ in past};newregion=(int(arc['robot1',q]//50),int(arc['robot3',c]//50)) not in priorbins
            row=dict(robot1_kf=q,robot3_kf=c,event_index=i,prior_loop_count=len(past),max_prior_loop_event=max((x[2] for x in past),default=-1),state='CAUSAL_COMMON_ODOMETRY_PREFIX' if S is not None else 'NO_COMMON_GAUGE_BEFORE_FIRST_BRIDGE',coverage_gain=1. if near is None else float(1-np.exp(-near)),region_connectivity_gain=int(newregion),global_connectivity_gain=int(S is None),estimated_marginal_bytes=bytes_,predicted_CPU_ms=cpu,nominal_info_nats=None,relative_uncertainty_trace=None,relative_uncertainty_max_eigenvalue=None,predicted_benefit_per_byte=None,current_GICP_transform_used=False,GT_used=False)
            if S is not None:
                ks=[key(('robot1',q)),key(('robot3',c))];marg=gtsam.Marginals(graph,values);joint=marg.jointMarginalCovariance(gtsam.KeyVector(ks));C=np.block([[joint.at(a,b) for b in ks] for a in ks]);Hq=np.zeros((6,6),order='F');Hc=np.zeros((6,6),order='F');values.atPose3(ks[0]).between(values.atPose3(ks[1]),Hq,Hc);J=np.hstack([Hq,Hc]);V=(J/SIG[:,None])@C@(J/SIG[:,None]).T;V=(V+V.T)/2;ev=np.linalg.eigvalsh(V);assert ev.min()>-1e-5;ev=np.maximum(ev,0);gain=float(.5*np.log1p(ev).sum());row.update(nominal_info_nats=gain,relative_uncertainty_trace=float(ev.sum()),relative_uncertainty_max_eigenvalue=float(ev.max()),predicted_benefit_per_byte=gain/max(1,bytes_))
            assert row['max_prior_loop_event']<i;out.append(row)
        if i in p0events:
            r=p0events[i];q,c=identity(r);Z=np.asarray(json.loads(r.canonical_transform))
            if S is None:
                S=local['robot1',q]@Z@inv(local['robot3',c]);graph.add(gtsam.PriorFactorPose3(key(('robot1',150)),gtsam.Pose3(local['robot1',150]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
                for x in sorted(prefix):add(x)
            graph.add(gtsam.BetweenFactorPose3(key(('robot1',q)),key(('robot3',c)),gtsam.Pose3(Z),robust));past.append((q,c,i))
    assert len(out)==372;csv('gt_free_criticality_features.csv',out);dump('stage23b_feature_freeze.json',dict(sha256=sha(OUT/'gt_free_criticality_features.csv'),rows=len(out),GT_used=False,actual_current_measurement_used=False,elapsed_s=time.monotonic()-beg,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024));print('GT_FREE_FEATURES_FROZEN',flush=True)

def solve():
    guard();loops,frame,nodes,local,arc,events,params=load();pol=read(OUT/'stage23b_policy.json');cache={n:build(n,loops,nodes,local,events) for n in [P0,P5]};states=OUT/'stage23b_states';states.mkdir(exist_ok=True)
    results=[];elapsed=0.;pilot=[];planned=pol['cases'];order=sorted(planned,key=lambda c:(0 if c['kind']=='BASELINE' else 1 if c['name'] in pol['pilot_names'] else 2,planned.index(c)))
    def one(case):
        graph,x0,idx=cache[case['policy']];removed={idx[tuple(pair)] for pair in case['remove']};g=gtsam.NonlinearFactorGraph()
        for j in range(graph.size()):
            if j not in removed:g.add(graph.at(j))
        t=time.monotonic();record=dict(case=case['name'],policy=case['policy'],kind=case['kind'],removed=len(removed),nodes=5796,odom=5794,prior=1,loops=len(loops[case['policy']])-len(removed),factors=g.size(),same_original_initialization=True)
        try:
            opt=gtsam.LevenbergMarquardtOptimizer(g,x0,params);v=opt.optimize();mat=matrices(v,nodes);assert np.isfinite(mat).all();assert np.max(abs(mat[0]-local[nodes[0]]))<1e-8
            if case['kind']=='BASELINE':
                frozen=pd.read_csv(B/'policies'/case['policy']/'online_batch_trajectory.csv');frozen=np.array([matrix(r) for r in frozen.itertuples()]);dt=float(np.linalg.norm(mat[:,:3,3]-frozen[:,:3,3],axis=1).max());dr=float(Rotation.from_matrix(np.swapaxes(mat[:,:3,:3],1,2)@frozen[:,:3,:3]).magnitude().max());assert dt<1e-7 and dr<1e-7,(case['name'],dt,dr)
                record.update(frozen_reproduction_translation_max_m=dt,frozen_reproduction_rotation_max_rad=dr)
            np.savez_compressed(states/(case['name']+'.npz'),matrices=mat)
            record.update(status='NORMAL_RETURN' if opt.iterations()<params.getMaxIterations() else 'ITERATION_CAP',iterations=opt.iterations(),objective_initial=float(g.error(x0)),objective_final=float(g.error(v)),full_reference_objective=float(graph.error(v)),finite=True)
        except Exception as exc:record.update(status='FAILED',reason=type(exc).__name__+': '+str(exc),finite=False)
        record.update(runtime_s=time.monotonic()-t,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024);return record
    for case in order:
        receipt=OUT/'stage23b_case_receipts'/(case['name']+'.json')
        if receipt.exists():r=read(receipt)
        else:
            if elapsed>=pol['budget']['total_solver_wall_cap_s']:break
            r=one(case);elapsed+=r['runtime_s'];dump('stage23b_case_receipts/'+case['name']+'.json',r)
        results.append(r)
        if case['kind']=='BASELINE':assert r['status']=='NORMAL_RETURN',r
        if case['name'] in pol['pilot_names']:pilot.append(r)
        if len(pilot)==len(pol['pilot_names']) and not (OUT/'stage23b_pilot_report.json').exists():
            estimate=float(np.mean([x['runtime_s'] for x in pilot]))*len(planned);rss=max(x['peak_RSS_mib'] for x in pilot);fits=estimate<900 and rss<2048
            dump('stage23b_pilot_report.json',dict(cases=pilot,actual_pilot_solver_s=sum(x['runtime_s'] for x in pilot),projected_all_solver_s=estimate,projection='MODELLED_FROM_BOUNDED_PILOT',peak_RSS_mib=rss,budget_900s_2GiB_fits=fits));print('PILOT_FEASIBILITY',fits,'modelled_s',estimate,'RSS_mib',rss,flush=True)
            if not fits:break
        csv('stage23b_solver_results.csv',results)
        if len(results)%20==0:print('BATCH_DIAGNOSTICS',len(results),'/',len(planned),flush=True)
    csv('stage23b_solver_results.csv',results);dump('stage23b_solve_freeze.json',dict(cases_completed=len(results),cases_planned=len(planned),solver_results_sha256=sha(OUT/'stage23b_solver_results.csv'),features_sha256=sha(OUT/'gt_free_criticality_features.csv'),GT_used=False,elapsed_solver_s=sum(r['runtime_s'] for r in results),nonconverged=sum(r['status']!='NORMAL_RETURN' for r in results)))
if __name__=='__main__':
    if not (OUT/'stage23b_policy.json').exists():freeze()
    if not (OUT/'stage23b_feature_freeze.json').exists():features()
    if not (OUT/'stage23b_solve_freeze.json').exists():solve()
