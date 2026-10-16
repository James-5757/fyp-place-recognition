#!/usr/bin/env python3
"""Stage20A: availability-gated archived-factor replay, genuine dual local chains.

No GT/SC/GICP calls. The frontend is gated by a persisted structural decision.
Frozen batch-agreement limits remain diagnostic, never silently relaxed.
"""
import argparse,hashlib,json,os,resource,subprocess,time
from pathlib import Path
import gtsam,numpy as np,pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import run_stage13b_incremental_isam2 as b
from stage20_schedule import schedule,replay_registry
from test_stage20_schedule import run as scheduler_tests
d=b.d;ROOT=d.ROOT;D12=b.D12;BASE='ee11e20925da35e369a562a598a5761bc68b5cf7'
OUT=ROOT/'outputs/cumulti_v1/20_dual_stream'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('w') as f:json.dump(data,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
def csv(name,rows):pd.DataFrame(rows).to_csv(OUT/name,index=False)
def progress(**kw):dump('progress.json',kw);print(json.dumps(kw),flush=True)
def prefix_frames(frames,last):return {r:f[f.keyframe_id.le(last[r])].copy() for r,f in frames.items()}
def main(output,display_extract=False):
    global OUT;OUT=Path(output)
    assert not OUT.exists(),'Refuse existing Stage20 output directory; choose a clean explicit output path.'
    OUT.mkdir(parents=True);(OUT/'states').mkdir();(OUT/'maps').mkdir()
    # Hash every historical tracked science/demo artifact, including existing unrelated dirt.
    protected=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().strip('\0').split('\0')
    protected=[ROOT/p for p in protected if p.startswith(('demo/','outputs/cumulti_v1/','src/cumulti/')) and not p.startswith(('src/cumulti/run_stage20','src/cumulti/stage20','src/cumulti/test_stage20','outputs/cumulti_v1/20_dual_stream/'))]
    hashes={str(p):sha(p) for p in protected if p.is_file()}
    inputs=[D12/n for n in ['clean_local_trajectory.csv','ordered_rank1_clean_loops.csv','clean_graph_policy.json','clean_gtsam_solver_policy.json']]
    inputs += [b.B13/'isam2_parameter_policy.json'] if hasattr(b,'B13') else [b.OUT/'isam2_parameter_policy.json']
    inputs += [ROOT/'outputs/cumulti_v1/12c_dataset_stability/stable_segment_selection.json']
    for p in inputs[:4]:
        blob=subprocess.check_output(['git','show',f'{BASE}:{p.relative_to(ROOT)}'],cwd=ROOT)
        assert hashlib.sha256(blob).hexdigest()==sha(p),str(p)
    local_tr=pd.read_csv(inputs[0],usecols=d.POSE_COLUMNS);local=d.s10.lookup(local_tr)
    frames={r:pd.read_csv(d.s10.PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).query('keyframe_id>=@start').copy() for r,start in d.STARTS.items()}
    assert [len(frames[r]) for r in ['robot1','robot3']]==[1850,3946]
    loops=pd.read_csv(inputs[1]);assert len(loops)==75
    streams={r:list(zip(f.keyframe_id.map(int),f.lidar_timestamp_ns.map(int))) for r,f in frames.items()}
    loop_rows=[dict(frozen_loop_id=int(x.frozen_loop_id),historical_order=int(x.arrival_index),query_robot=x.query_robot,query_keyframe_id=int(x.query_keyframe_id),candidate_robot=x.candidate_robot,candidate_keyframe_id=int(x.candidate_keyframe_id)) for x in loops.itertuples()]
    events,elig=schedule(streams,loop_rows);logical=replay_registry(events,elig)
    assert len(events)==5796 and elig[0]['frozen_loop_id']==45
    params=gtsam.ISAM2Params();hist=json.loads(inputs[4].read_text());assert str(params)==hist['parameter_text']
    agree=json.loads((b.OUT/'incremental_batch_agreement_policy.json').read_text())
    tests=scheduler_tests();dump('scheduler_unit_tests.json',tests)
    policy=dict(baseline=BASE,clock='each original timestamp minus own first retained timestamp, exact integer ns',event_sort=['normalized_ns','robot_id','original_keyframe_id'],loop_sort=['release_event_index','frozen_loop_id'],loop_release='after both endpoint events; release=max(endpoint event positions)',archived_frontend='AVAILABILITY-GATED REPLAY OF PRECOMPUTED LOOP FACTORS',simultaneous_physical_robots=False,independent_local_graphs='anchored odometry chains; exact frozen sequential composition, no local optimization required',central_prior='only Robot1 KF150',isam2=hist,extra_empty_updates=0,agreement_diagnostic=agree,checkpoint_rules=['PRE_FIRST_BRIDGE','FIRST_BRIDGE','FIRST_POST_BRIDGE_UPDATE','DUAL_STREAM_K2','DUAL_STREAM_K5','DUAL_STREAM_K10','DUAL_STREAM_K20','DUAL_STREAM_K40','DUAL_STREAM_K75','ROBOT1_FINISHED','ROBOT3_FINISHED','FINAL'],batch_initialization='frozen first-bridge canonical local/common x0 restricted to exactly available nodes; default12D LM',integrity=json.loads(inputs[2].read_text())['catastrophic_integrity_rule'],map_protocol=dict(frame_stride=10,point_stride=16,voxel_m=.5,range_m=[1,60]),GT_used=False,NETWORK_COMMUNICATION='NETWORK_COMMUNICATION_NOT_MEASURED',typed_message_categories=['LOCAL_KEYFRAME_METADATA','ODOMETRY_FACTOR','LOOP_CONSTRAINT','MAP_CHUNK_REQUEST','MAP_CHUNK_RESPONSE'],network_payloads_serialized=False)
    dump('stage20_input_manifest.json',dict(inputs=[dict(path=str(p),sha256=sha(p)) for p in inputs],protected_hashes=hashes,GT_used=False))
    policy['exact_display_extraction_replay']=display_extract
    policy['additional_display_snapshot_interval_s']=60 if display_extract else None
    dump('stage20_policy.json',policy) # flush+fsync occurs before graph construction/experiment
    dump('dual_stream_clock_policy.json',dict(normalized_ns='timestamp_ns - own first retained timestamp',origins={r:str(rows[0][1]) for r,rows in streams.items()},tie_break=policy['event_sort'],not_physical_simultaneity=True))
    csv('dual_stream_event_schedule.csv',events);csv('loop_eligibility_schedule.csv',elig)
    d.OUT=OUT;template,lmparams=d.build_graph(frames,local,loops)
    expected_lm=json.loads(inputs[3].read_text())['parameters']
    assert json.loads((OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==expected_lm
    odom={('robot1',k):k-150 for k in range(151,2000)};odom.update({('robot3',k):1850+k-235 for k in range(235,4180)})
    loopidx={int(r.frozen_loop_id):5794+int(r.arrival_index) for r in loops.itertuples()}
    releases={}
    for l in elig:releases.setdefault(l['release_event_index'],[]).append(l)
    first_event=elig[0]['release_event_index'];first=loops[loops.frozen_loop_id.eq(elig[0]['frozen_loop_id'])].iloc[0]
    S=local[('robot1',int(first.query_keyframe_id))]@d.tm(first)@d.s10.inv(local[('robot3',int(first.candidate_keyframe_id))])
    common={n:x if n[0]=='robot1' else S@x for n,x in local.items()}
    selection=json.loads(inputs[-1].read_text())
    starts=d.STARTS;last={r:start-1 for r,start in starts.items()};prefix={r:{} for r in starts};local_graph={r:gtsam.NonlinearFactorGraph() for r in starts}
    inserted=[];central_nodes=set();inserted_loops=set();isam=None;instance=None
    trace=[];local_progress=[];comparisons=[];integrity=[];states=[];allfinite=True;catastrophes=0;firstcost={};last_est=None
    ckK={2,5,10,20,40,75};serial_finish_ns=0;next_display_ns=0
    def snapshot(name,e,estimate,K,diagnose=True):
        available=prefix_frames(frames,last);nodes=[(r,int(k)) for r,f in available.items() for k in f.keyframe_id]
        matrices={n:prefix[n[0]][n] for n in nodes} if estimate is None else b.matrices_from(estimate,nodes)
        tr=d.trajectory(available,matrices);b.schema(tr,available)
        path=OUT/'states'/f'{name}.csv';tr.to_csv(path,index=False)
        start=time.perf_counter();by,_=d.s10.map_points(tr,available['robot1'],available['robot3']);arts=[]
        for robot,p in by.items():
            p=d.s10.voxel(p);target=OUT/'maps'/f'{name}_{robot}.ply';d.s10.write_ply(target,p)
            arts.append(dict(robot=robot,path=str(target),sha256=sha(target),bytes=target.stat().st_size,points=len(p)))
        pub=(time.perf_counter()-start)*1000
        states.append(dict(name=name,event_index=e['event_index'],normalized_ns=e['normalized_ns'],robot1_kf=last['robot1'],robot3_kf=last['robot3'],K=K,global_initialized=estimate is not None,trajectory=str(path),sha256=sha(path),maps=arts,map_publication_ms=pub))
        if estimate is not None and diagnose and not display_extract:
            graph=b.graph_subset(template,inserted);x0=b.to_values({n:common[n] for n in nodes});t=time.perf_counter();opt=gtsam.LevenbergMarquardtOptimizer(graph,x0,lmparams);v=opt.optimize();ms=(time.perf_counter()-t)*1000
            batch=b.matrices_from(v,nodes);bt=d.trajectory(available,batch);bt.to_csv(OUT/'states'/f'{name}_BATCH.csv',index=False)
            for robot in ['ALL','robot1','robot3']:
                inc={n:x for n,x in matrices.items() if robot=='ALL' or n[0]==robot};ref={n:batch[n] for n in inc};diff=b.pose_difference(inc,ref)
                comparisons.append(dict(checkpoint=name,robot_id=robot,nodes=len(inc),factors=graph.size(),incremental_error=float(graph.error(estimate)),batch_initial_error=float(graph.error(x0)),batch_error=float(graph.error(v)),batch_runtime_ms=ms,batch_iterations=opt.iterations(),batch_status='NORMAL_RETURN',batch_iteration_cap_reached=bool(opt.iterations()>=lmparams.getMaxIterations()),same_gauge_no_alignment=True,diagnostic_PASS=all(diff[k]<=agree[k] for k in diff),**diff))
        return pub
    for i,e in enumerate(events):
        r=e['robot_id'];kf=e['keyframe_id'];t=time.perf_counter();assert kf==last[r]+1
        n=(r,kf);key=b.numeric_key(n)
        if kf==starts[r]:
            prefix[r][n]=local[n].copy();local_graph[r].add(gtsam.PriorFactorPose3(key,d.pose(local[n]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
        else:
            idx=odom[n];prefix[r][n]=prefix[r][(r,kf-1)]@template.at(idx).measured().matrix();local_graph[r].add(template.at(idx))
        assert np.max(abs(prefix[r][n]-local[n]))<1e-8
        last[r]=kf;local_ms=(time.perf_counter()-t)*1000
        arrived=releases.get(i,[]);construction=update=extract=pub=audit_ms=0.;kind='LOCAL_ONLY'
        if i==first_event:
            t=time.perf_counter();available=prefix_frames(frames,last);nodes=[(robot,int(k)) for robot,f in available.items() for k in f.keyframe_id]
            indices=[0]+[odom[x] for x in nodes if x[1]>starts[x[0]]]+[loopidx[x['frozen_loop_id']] for x in arrived]
            new=b.graph_subset(template,indices);values=b.to_values({x:common[x] for x in nodes});inserted.extend(indices);central_nodes.update(nodes)
            isam=gtsam.ISAM2(params);instance=id(isam);construction=(time.perf_counter()-t)*1000;kind='FIRST_BRIDGE'
            E=d.s10.inv(d.tm(first))@d.s10.inv(common[('robot1',int(first.query_keyframe_id))])@common[('robot3',int(first.candidate_keyframe_id))]
            residual=dict(translation_m=float(np.linalg.norm(E[:3,3])),rotation_deg=float(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude())))
            assert residual['translation_m']<1e-8 and residual['rotation_deg']<1e-8
            firstcost=dict(loop_id=int(first.frozen_loop_id),event_index=i,normalized_ns=e['normalized_ns'],normalized_time_s=e['normalized_ns']/1e9,current_keyframes=dict(last),local_nodes={r:len(v) for r,v in prefix.items()},joint_nodes=len(nodes),odom_factors=len(nodes)-2,loop_factors=len(arrived),prior_factors=1,components_before=2,components_after=1,initial_alignment_residual=residual,graph_construction_ms=construction)
        elif isam is not None:
            assert id(isam)==instance and n not in central_nodes
            new=gtsam.NonlinearFactorGraph();idx=odom[n];new.add(template.at(idx));inserted.append(idx)
            t=time.perf_counter();previous=isam.calculateEstimatePose3(b.numeric_key((r,kf-1)));extract=(time.perf_counter()-t)*1000
            values=gtsam.Values();values.insert(key,previous.compose(template.at(idx).measured()));central_nodes.add(n)
            for l in arrived:new.add(template.at(loopidx[l['frozen_loop_id']]));inserted.append(loopidx[l['frozen_loop_id']])
            kind='LOOP_EVENT' if arrived else 'ODOMETRY_UPDATE'
        if isam is not None:
            for l in arrived:
                assert l['frozen_loop_id'] not in inserted_loops
                assert ('robot1',l['query_keyframe_id']) in central_nodes and ('robot3',l['candidate_keyframe_id']) in central_nodes
                inserted_loops.add(l['frozen_loop_id'])
            t=time.perf_counter();isam.update(new,values);update=(time.perf_counter()-t)*1000
            t=time.perf_counter();last_est=isam.calculateEstimate();extract+=(time.perf_counter()-t)*1000
            assert last_est.size()==len(central_nodes)==sum(len(v) for v in prefix.values())
            assert isam.getFactorsUnsafe().size()==len(inserted)==len(set(inserted))
            t=time.perf_counter();mat=b.matrices_from(last_est,central_nodes);finite=all(np.isfinite(x).all() for x in mat.values());allfinite &=finite;assert finite
            for robot,f in frames.items():
                ids=list(range(starts[robot],last[robot]+1));xyz=np.array([mat[(robot,k)][:3,3] for k in ids]);pre=np.array([common[(robot,k)][:3,3] for k in ids]);poststep=np.linalg.norm(np.diff(xyz,axis=0),axis=1);prestep=np.linalg.norm(np.diff(pre,axis=0),axis=1)
                limit=selection['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold'];count=int(((poststep>5*limit)&(prestep<=5*limit)|(poststep>=100)&(prestep<100)).sum());catastrophes+=count
                integrity.append(dict(event_index=i,robot_id=robot,current_keyframe=last[robot],max_xyz_step_m=float(poststep.max()) if len(poststep) else 0.,new_catastrophic_steps=count))
            audit_ms=(time.perf_counter()-t)*1000
            if catastrophes:raise RuntimeError('Mandatory gate FAIL: new catastrophic trajectory jump; no Stage20B.')
        for robot in starts:
            if prefix[robot]:assert max(k for rr,k in prefix[robot])==last[robot]
            else:assert last[robot]==starts[robot]-1
        K=len(inserted_loops);names=[]
        if i==first_event-1:names.append('PRE_FIRST_BRIDGE')
        if i==first_event:names.append('FIRST_BRIDGE')
        if i==first_event+1:names.append('FIRST_POST_BRIDGE_UPDATE')
        before=K-len(arrived)
        for kk in sorted(ckK):
            if before<kk<=K:names.append('DUAL_STREAM_K'+str(kk))
        if kf==int(frames[r].keyframe_id.max()):names.append('ROBOT1_FINISHED' if r=='robot1' else 'ROBOT3_FINISHED')
        if i==len(events)-1:names.append('FINAL')
        # Save real loop states for later presentation; no frontend is touched here.
        if arrived and not names:names.append(f'LOOP_{K:02d}')
        if display_extract and all(prefix.values()) and e['normalized_ns']>=next_display_ns:
            if not names:names.append(f'PERIODIC_{e["normalized_ns"]//10**9:04d}')
            next_display_ns=(e['normalized_ns']//60_000_000_000+1)*60_000_000_000
        for name in names:pub+=snapshot(name,e,last_est,K,diagnose=name in policy['checkpoint_rules'] and name!='PRE_FIRST_BRIDGE')
        total=local_ms+construction+update+extract+pub
        tns=e['normalized_ns'];serial_finish_ns=max(serial_finish_ns,tns)+int(total*1e6)
        future_times=[x['normalized_ns'] for x in events] if i==0 else future_times
        queue=max(0,int(np.searchsorted(future_times,serial_finish_ns,side='right'))-i-1)
        row=dict(**e,current_robot1_kf=last['robot1'],current_robot3_kf=last['robot3'],robot1_nodes=len(prefix['robot1']),robot3_nodes=len(prefix['robot3']),central_nodes=len(central_nodes),central_odometry=max(0,len(central_nodes)-2) if isam else 0,new_loop_ids=';'.join(str(l['frozen_loop_id']) for l in arrived),cumulative_loops=K,central_factors=len(inserted),components=1 if isam else sum(bool(v) for v in prefix.values()),global_initialized=isam is not None,event_type=kind,local_processing_ms=local_ms,graph_construction_ms=construction,isam2_update_ms=update,estimate_extraction_ms=extract,map_publication_ms=pub,finite_integrity_audit_ms=audit_ms,measured_service_ms=total,peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,backend_queue_length=0,queue_semantics='serial offline replay, no asynchronous transport',hypothetical_realtime_fifo_backlog=queue,recording_complete_robot1=last['robot1']==1999,recording_complete_robot3=last['robot3']==4179,unseen_loops=sum(1 for l in elig if i<min(l['query_event_index'],l['candidate_event_index'])),pending_loops=sum(1 for l in elig if min(l['query_event_index'],l['candidate_event_index'])<=i<l['release_event_index']))
        trace.append(row);local_progress.append(dict(event_index=i,robot_id=r,keyframe_id=kf,normalized_ns=tns,local_nodes=len(prefix[r]),local_odometry=len(prefix[r])-1,local_priors=1,local_graph_factors=local_graph[r].size(),scan_available_kf=kf,original_timestamp_ns=e['timestamp_ns'],immutable_local_pose=True))
        if i==first_event:firstcost.update(first_isam2_update_ms=update,estimate_extraction_ms=extract,first_map_publication_ms=pub,first_fusion_service_ms=total);dump('first_bridge_fusion.json',firstcost)
        if arrived or i%500==0:progress(event=i,robot=r,kf=kf,nodes=len(central_nodes),loops=K)
    actual=isam.getFactorsUnsafe();equiv=dict(nodes=len(central_nodes),odometry=len(inserted)-K-1,loops=K,prior=1,factors=len(inserted),all_factor_indices_exactly_once=set(inserted)==set(range(template.size())),factor_measurement_noise_equality=all(actual.at(j).equals(template.at(ix),1e-12) for j,ix in enumerate(inserted)),node_key_equality=set(last_est.keys())==set(template.keyVector()),loop_id_equality=inserted_loops==set(loops.frozen_loop_id.map(int)),frozen_extrinsic=d.s10.IMU_FROM_LIDAR.tolist(),original_timestamps_preserved=True)
    equiv['PASS']=equiv['nodes']==5796 and equiv['odometry']==5794 and K==75 and equiv['factors']==5870 and all(equiv[k] for k in ['all_factor_indices_exactly_once','factor_measurement_noise_equality','node_key_equality','loop_id_equality'])
    assert equiv['PASS'];dump('final_graph_equivalence.json',equiv)
    tdf=pd.DataFrame(trace);tdf['next_event_interval_ms']=(tdf.normalized_ns.shift(-1)-tdf.normalized_ns)/1e6;tdf['backend_deadline_met']=tdf.isam2_update_ms<tdf.next_event_interval_ms
    csv('central_incremental_event_trace.csv',tdf);csv('incremental_latency_by_event.csv',tdf);csv('local_graph_progress_by_robot.csv',local_progress);csv('trajectory_integrity_report.csv',integrity);csv('incremental_vs_batch_diagnostics.csv',comparisons);dump('backend_state_manifest.json',states)
    from audit_stage20_backend import availability
    availability(OUT)
    timing=[]
    for label,sub in [('ROBOT1_ODOMETRY',tdf[(tdf.robot_id=='robot1')&(tdf.event_type=='ODOMETRY_UPDATE')]),('ROBOT3_ODOMETRY',tdf[(tdf.robot_id=='robot3')&(tdf.event_type=='ODOMETRY_UPDATE')]),('LOOP_EVENTS',tdf[tdf.new_loop_ids.ne('')]),('FIRST_FUSION',tdf[tdf.event_type=='FIRST_BRIDGE'])]:
        for metric in ['local_processing_ms','isam2_update_ms','estimate_extraction_ms','map_publication_ms','measured_service_ms']:
            x=sub[metric].to_numpy();timing.append(dict(category=label,metric=metric,count=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(max(x))))
    csv('incremental_latency_summary.csv',timing)
    reconstruct=replay_registry(*schedule(streams,loop_rows));trace_valid=all(logical[i]==reconstruct[i] and row['cumulative_loops']==logical[i]['loops'] and row['robot1_nodes']+row['robot3_nodes']==logical[i]['nodes'] for i,row in enumerate(trace))
    immutable=all(sha(p)==h for p,h in hashes.items());assert immutable
    final=b.matrices_from(last_est,central_nodes);anchor=d.s10.inv(local[('robot1',150)])@final[('robot1',150)];assert np.linalg.norm(anchor[:3,3])<1e-6
    ready=bool(equiv['PASS'] and allfinite and not catastrophes and trace_valid and immutable and tests['status']=='PASS')
    decision=dict(status='DUAL_STREAM_BACKEND_READY' if ready else 'DUAL_STREAM_BACKEND_NOT_READY',structural_readiness_only=True,numerical_batch_agreement_all=all(x['diagnostic_PASS'] for x in comparisons) if comparisons else None,finite_estimates=allfinite,new_catastrophic_steps=catastrophes,reproducible_event_trace=trace_valid,one_persistent_isam2=True,updates=sum(row['global_initialized'] for row in trace),GT_used=False,STAGE19_IMMUTABLE=immutable,STAGE12D_INPUTS_IMMUTABLE=immutable,first_bridge=firstcost,first10_loop_ids=[x['frozen_loop_id'] for x in elig[:10]],final_graph=equiv)
    dump('stage20a_backend_decision.json',decision);dump('stage20b_demo_validation.json',dict(status='NOT_RUN',reason='Stage20A persisted; separate gated presentation step required'))
    dump('summary.json',decision);(OUT/'VALIDATION_REPORT.txt').write_text('STAGE20A: '+decision['status']+'\nSTRUCTURAL / FINITE / INTEGRITY / SCHEDULER: '+('PASS' if ready else 'FAIL')+'\nNUMERICAL AGREEMENT: '+str(decision['numerical_batch_agreement_all'])+' (diagnostic, limits unchanged)\nSTAGE20B: NOT_RUN\nGT_USED_FOR_CONSTRUCTION_OR_SELECTION: NO\nNETWORK_COMMUNICATION_NOT_MEASURED\n')
    figure(tdf,elig,local_tr,states)
    progress(status=decision['status'],first10=decision['first10_loop_ids'],first=firstcost)
def figure(t,loops,local,states):
    plots=[('two_robot_timelines',lambda ax:[ax.plot(t[t.robot_id.eq(r)].normalized_ns/1e9,t[t.robot_id.eq(r)].keyframe_id,label=r) for r in ['robot1','robot3']]),('loop_eligibility_timeline',lambda ax:ax.plot([l['eligibility_ns']/1e9 for l in loops],range(1,76),'o-')),('local_node_growth',lambda ax:[ax.plot(t.normalized_ns/1e9,t[r+'_nodes'],label=r) for r in ['robot1','robot3']]),('central_graph_growth',lambda ax:[ax.plot(t.normalized_ns/1e9,t[c],label=c) for c in ['central_nodes','central_factors']]),('latency',lambda ax:ax.scatter(t.normalized_ns/1e9,t.isam2_update_ms,s=2,c=np.where(t.new_loop_ids.ne(''),'green','blue'))),('delayed_loop_availability',lambda ax:[ax.plot([l['query_normalized_ns']/1e9,l['eligibility_ns']/1e9],[l['historical_order']]*2,'o-') for l in loops[:10]])]
    for name,fn in plots:
        fig,ax=plt.subplots(figsize=(9,4),dpi=140);fn(ax);ax.set_xlabel('hypothetical normalized replay seconds');ax.set_title(name);ax.grid(alpha=.2)
        if name in ['two_robot_timelines','local_node_growth','central_graph_growth']:ax.legend()
        fig.tight_layout();fig.savefig(OUT/(name+'.png'));plt.close(fig)
    for name in ['FIRST_BRIDGE','FINAL']:
        tr=pd.read_csv(OUT/'states'/f'{name}.csv');fig,ax=plt.subplots(figsize=(8,6),dpi=140)
        for r,c in [('robot1','blue'),('robot3','orange')]:q=tr[tr.robot_id.eq(r)];ax.plot(q.tx,q.ty,color=c,label=r)
        if name=='FINAL':
            old=pd.read_csv(b.OUT/'snapshots/FINAL_STREAM_END_trajectory.csv')
            for r in ['robot1','robot3']:q=old[old.robot_id.eq(r)];ax.plot(q.tx,q.ty,'--',alpha=.5,label=r+' Stage13B')
        ax.legend();ax.axis('equal');ax.set_title(name+' / frozen gauge XY, not geographic altitude');fig.tight_layout();fig.savefig(OUT/(name+'_trajectory.png'));plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',default=str(OUT));p.add_argument('--display-extract',action='store_true');args=p.parse_args()
    try:main(args.output,args.display_extract)
    except Exception as error:
        if OUT.exists():
            dump('stage20a_backend_decision.json',dict(status='DUAL_STREAM_BACKEND_NOT_READY',exception=repr(error),no_frontend_authorized=True,GT_used=False));dump('stage20b_demo_validation.json',dict(status='NOT_RUN',reason='Mandatory Stage20A failure'))
        raise
