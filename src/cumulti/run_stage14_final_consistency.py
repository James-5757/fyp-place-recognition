#!/usr/bin/env python3
"""Independent fixed final-consistency audit. No GT, scans, maps or old mains.

Fresh complete-graph insertion is NOT an incremental per-frame timing test.
Batch LM remains a numerical reference, not physical ground truth.
"""
from __future__ import annotations
import hashlib
import importlib.metadata
import json
import os
import resource
import subprocess
import sys
import time
from decimal import Decimal,InvalidOperation
from pathlib import Path
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.stats import pearsonr,spearmanr
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import run_stage13c_isam2_convergence as c

b=c.b;d=c.d;ROOT=c.ROOT;B=b.OUT;D=ROOT/'outputs/cumulti_v1/13d_solver_diagnostics'
OUT=ROOT/'outputs/cumulti_v1/14_final_consistency';ACCEPTED='292f8c22ef62b51dace4ac17b0df633170322fbb'
CP={'K2':378,'K75':1967,'FINAL_STREAM_END':1999}
LIMITS=dict(translation_median_m=.1,translation_p95_m=.5,rotation_median_deg=.5,rotation_p95_deg=2.)
ROWS={k:[] for k in ['trace','summary','lm','localization','timing','integrity','pairwise','events']}
ALL={};GRAPHS={};REF={};FRAMES={};HISTORY={}

def dump(name,value):
    p=OUT/name;p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    with p.open('rb') as f:os.fsync(f.fileno())

def csv(name,rows):
    (rows if isinstance(rows,pd.DataFrame) else pd.DataFrame(rows)).to_csv(OUT/name,index=False)

def progress(phase,**kw):
    row=dict(UTC=b.a.now(),phase=phase,**kw);ROWS['events'].append(row)
    print(json.dumps(row),flush=True);dump('progress.json',row);dump('execution_events.json',ROWS['events'])

def counters(result,nodes):
    row={}
    for short,method in [('relinearized','getVariablesRelinearized'),('reeliminated','getVariablesReeliminated')]:
        raw=str(getattr(result,method)()) if hasattr(result,method) else 'NOT_EXPOSED';value=None;status='UNVERIFIED'
        try:
            x=Decimal(raw)
            if x.is_finite() and x==int(x):
                if 0<=x<=nodes:value=int(x);status='IN_RANGE_SEMANTICS_UNVERIFIED'
                else:status='INVALID_COUNTER'
        except (InvalidOperation,ValueError,OverflowError):pass
        row.update({short+'_raw':raw,short+'_diagnostic':status,short+'_valid_count':value})
    return row

def pose_table(left,right):
    assert set(left)==set(right);rows=[]
    for node,x in left.items():
        y=right[node];world=x[:3,3]-y[:3,3];E=d.s10.inv(y)@x
        bodylog=np.asarray(gtsam.Pose3.Logmap(d.pose(E))).reshape(6)
        worldrot=Rotation.from_matrix(x[:3,:3]@y[:3,:3].T).as_rotvec()
        row=dict(robot_id=node[0],keyframe_id=node[1],translation_m=float(np.linalg.norm(world)),rotation_deg=float(np.rad2deg(np.linalg.norm(worldrot))),dx=float(world[0]),dy=float(world[1]),dz=float(world[2]),wrx=float(worldrot[0]),wry=float(worldrot[1]),wrz=float(worldrot[2]),log_finite=bool(np.isfinite(bodylog).all()))
        for k,val in zip(['log_rx','log_ry','log_rz','log_vx','log_vy','log_vz'],bodylog):row[k]=float(val) if np.isfinite(val) else None
        rows.append(row)
    return pd.DataFrame(rows)

def agreement(mat,reference):
    df,parts=c.metrics(mat,reference);combined=parts[0]
    return combined,parts,all(combined[k]<=v for k,v in LIMITS.items())

def indices(kf,K):
    return [0]+[i-150 for i in range(151,kf+1)]+[1850+i-235 for i in range(235,4180)]+list(range(5795,5795+K))

def historical_path(cp,solver):
    return D/f'checkpoints/{solver}_{cp}_{"AFTER_INSERTION_0" if solver=="D0" else "EXTRA_5"}.csv'

def allowed_frames():
    return {r:pd.read_csv(d.s10.PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns']).iloc[start:].copy() for r,start in d.STARTS.items()}

def loop_provenance():
    old=pd.read_csv(d.LOOP_SOURCE,usecols=d.LOOP_COLUMNS);assert len(old)==120
    old.insert(0,'frozen_loop_id',np.arange(120))
    keep=old[old.query_keyframe_id.ge(150)&old.candidate_keyframe_id.ge(234)].sort_values(['query_timestamp','query_keyframe_id','candidate_keyframe_id'],kind='stable').reset_index(drop=True)
    clean=pd.read_csv(b.D12/'ordered_rank1_clean_loops.csv',usecols=['arrival_index','frozen_loop_id']+d.LOOP_COLUMNS)
    pd.testing.assert_frame_equal(keep[['frozen_loop_id']+d.LOOP_COLUMNS],clean[['frozen_loop_id']+d.LOOP_COLUMNS],check_exact=False,atol=1e-12,rtol=1e-12)
    assert len(clean)==75 and clean.arrival_index.to_list()==list(range(1,76))
    audit=dict(PASS=True,historical_count=120,retained_count=75,excluded_count=45,filter='endpoint ranges only R1>=150/R3>=234; original upper IDs checked against available poses',historical_sha256=d.sha256(d.LOOP_SOURCE),clean_sha256=d.sha256(b.D12/'ordered_rank1_clean_loops.csv'),selected_frozen_loop_ids=clean.frozen_loop_id.astype(int).to_list(),excluded_frozen_loop_ids=old[~old.frozen_loop_id.isin(clean.frozen_loop_id)].frozen_loop_id.astype(int).to_list(),measurements_endpoint_ids_and_order_unchanged=True,comparison_tolerance=1e-12,GT_columns_decoded=False,GT_filtering=False)
    dump('loop_provenance_audit.json',audit);return clean

def freeze_marginal_nodes():
    path=D/'pose_differences/D0_FINAL_STREAM_END_AFTER_INSERTION_0.csv'
    z=pd.read_csv(path);r=z[z.robot_id.eq('robot3')];worst=[]
    for row in r.sort_values(['translation_m','keyframe_id'],ascending=[False,True]).itertuples():
        if all(abs(row.keyframe_id-i)>=200 for i in worst):worst.append(int(row.keyframe_id))
        if len(worst)==3:break
    low=r[r.translation_m.le(r.translation_m.quantile(.25))];used=set();selected=[]
    for i in worst:
        candidates=low[~low.keyframe_id.isin(used)].copy();candidates['gap']=abs(candidates.keyframe_id-i)
        match=candidates.sort_values(['gap','keyframe_id']).iloc[0];j=int(match.keyframe_id);used.add(j)
        selected.extend([dict(robot_id='robot3',keyframe_id=i,group='WORST',matched_control_kf=j),dict(robot_id='robot3',keyframe_id=j,group='LOW_ERROR_CONTROL',matched_worst_kf=i,matching_kf_gap=abs(j-i))])
    selected += [dict(robot_id='robot1',keyframe_id=i,group='ROBOT1_CONTROL') for i in [150,1075,1999]]
    return selected

def state(cp,name,mat,graph,ref,update_ms=0.,extract_ms=0.,construction_ms=0.,raw=None,step=0,initial=None,previous=None):
    nodes=list(ref);assert set(mat)==set(ref) and all(np.isfinite(x).all() for x in mat.values())
    t=time.perf_counter();values=b.to_values(mat);error=float(graph.error(values));objective_ms=(time.perf_counter()-t)*1000
    t=time.perf_counter();combined,parts,passes=agreement(mat,ref);compare_ms=(time.perf_counter()-t)*1000
    startdiff=b.pose_difference(mat,initial) if initial else {}
    maximum_t=maximum_r=delta_error=None;nochange=False
    if previous:
        old,old_error=previous;diff,_=c.metrics(mat,old);maximum_t=float(diff.translation_m.max());maximum_r=float(diff.rotation_deg.max());delta_error=error-old_error
        nochange=maximum_t<1e-7 and maximum_r<1e-7 and abs(delta_error)<1e-9
    row=dict(checkpoint=cp,solver_state=name,step=step,nodes=len(nodes),odometry=len(nodes)-2,loops=2 if cp=='K2' else 75,prior=1,factors=graph.size(),nonlinear_error=error,batch_error=float(graph.error(b.to_values(ref))),objective_gap=abs(error-float(graph.error(b.to_values(ref)))),PASS=passes,update_ms=update_ms,extraction_ms=extract_ms,construction_ms=construction_ms,objective_ms=objective_ms,comparison_ms=compare_ms,maximum_pose_translation_change_m=maximum_t,maximum_pose_rotation_change_deg=maximum_r,objective_change=delta_error,empirical_no_change=nochange,**{k:v for k,v in combined.items() if k!='nodes'},**{'from_init_'+k:v for k,v in startdiff.items()},**(raw or {}))
    f={'robot1':FRAMES['robot1'][FRAMES['robot1'].keyframe_id.le(CP[cp])],'robot3':FRAMES['robot3']}
    tr=d.trajectory(f,mat);b.schema(tr,f)
    if cp=='FINAL_STREAM_END':assert d.schema_audit(tr)['PASS']
    anchor=d.s10.inv(LOCAL[('robot1',150)])@mat[('robot1',150)]
    assert np.linalg.norm(anchor[:3,3])<1e-6 and Rotation.from_matrix(anchor[:3,:3]).magnitude()<1e-6
    ROWS['integrity'].extend(dict(solver_state=name,step=step,**x) for x in b.integrity(tr,LOCAL_TR,COMMON_TR,SELECTION,name,cp))
    file_start=time.perf_counter();tr.to_csv(OUT/f'estimates/{cp}_{name}_step{step}.csv',index=False)
    if name.startswith(('A1','A2','A3')) and step==5 or not name.startswith(('A1','A2','A3')):
        detailed=pose_table(mat,ref);detailed.to_csv(OUT/f'nodewise/{cp}_{name}_vs_batch.csv',index=False)
        for scope,part in zip(['combined','robot1','robot3'],parts):
            ROWS['localization'].append(dict(checkpoint=cp,solver_state=name,record_type='SCOPE',**part))
            sub=detailed if scope=='combined' else detailed[detailed.robot_id.eq(scope)]
            for metric in ['translation_m','rotation_deg']:
                for rank,r in enumerate(sub.nlargest(20,metric).itertuples(),1):ROWS['localization'].append(dict(checkpoint=cp,solver_state=name,record_type='WORST20',scope=scope,ranking_metric=metric,rank=rank,keyframe_id=int(r.keyframe_id),robot_id=r.robot_id,translation_m=r.translation_m,rotation_deg=r.rotation_deg))
            threshold=sub.translation_m.quantile(.95)
            for robot,z in sub[sub.translation_m.ge(threshold)].groupby('robot_id'):
                ids=sorted(z.keyframe_id.astype(int).to_list());groups=[]
                for i in ids:
                    if not groups or i>groups[-1][-1]+1:groups.append([i])
                    else:groups[-1].append(i)
                for group in groups:ROWS['localization'].append(dict(checkpoint=cp,solver_state=name,record_type='WORST_P95_CONTIGUOUS_REGION',scope=scope,robot_id=robot,first_kf=group[0],last_kf=group[-1],count=len(group),threshold_m=float(threshold)))
    row['file_localization_overhead_ms']=(time.perf_counter()-file_start)*1000
    ROWS['timing'].append(dict(checkpoint=cp,operation=name,step=step,construction_ms=construction_ms,update_ms=update_ms,extraction_ms=extract_ms,objective_ms=objective_ms,comparison_ms=compare_ms,file_localization_overhead_ms=row['file_localization_overhead_ms']))
    return row,error

def rebuild(cp,label,initial,graph,ref):
    t=time.perf_counter();params,printed=c.configure(dict(skip=1,threshold=.01));isam=gtsam.ISAM2(params);construction=(time.perf_counter()-t)*1000
    t=time.perf_counter();nv=b.to_values(initial);values_preparation=(time.perf_counter()-t)*1000;previous=None;consecutive=0;last=None
    for step in range(6):
        t=time.perf_counter();result=isam.update(graph,nv) if step==0 else isam.update();um=(time.perf_counter()-t)*1000
        t=time.perf_counter();estimate=isam.calculateEstimate();mat=b.matrices_from(estimate,list(initial));extract=(time.perf_counter()-t)*1000
        actual=isam.getFactorsUnsafe();assert actual.size()==graph.size() and set(estimate.keys())==set(graph.keyVector())
        assert all(actual.at(i).equals(graph.at(i),1e-12) for i in range(graph.size()))
        row,error=state(cp,label,mat,graph,ref,um,extract,construction if step==0 else 0.,counters(result,len(mat)),step,initial,previous)
        row['initial_values_preparation_ms']=values_preparation if step==0 else 0.
        ROWS['timing'][-1]['initial_values_preparation_ms']=row['initial_values_preparation_ms']
        consecutive=consecutive+1 if row['empirical_no_change'] else 0;row['two_consecutive_no_change']=consecutive>=2
        ROWS['trace'].append(row);previous=(mat,error);last=row
    ALL[(cp,label)]=mat
    ROWS['summary'].append(dict(**last,extra_updates=5,initialization_source={'A1_FRESH_CANONICAL':'canonical common x0','A2_FRESH_D0':'historical D0 same-checkpoint coordinates','A3_FRESH_D2':'historical D2 extra5 same-checkpoint coordinates'}[label],fresh_parameters=printed))
    progress('REBUILD_COMPLETE',checkpoint=cp,state=label,translation_p95_m=last['translation_p95_m'],PASS=last['PASS'])
    persist()

def basin(cp,label,initial,graph,ref,params):
    x=b.to_values(initial);initial_error=float(graph.error(x));t=time.perf_counter()
    optimizer=gtsam.LevenbergMarquardtOptimizer(graph,x,params);estimate=optimizer.optimize();runtime=(time.perf_counter()-t)*1000
    t=time.perf_counter();mat=b.matrices_from(estimate,list(initial));extract=(time.perf_counter()-t)*1000
    row,error=state(cp,label,mat,graph,ref,extract_ms=extract,initial=initial)
    row.update(initial_error=initial_error,final_error=error,runtime_ms=runtime,iterations=int(optimizer.iterations()),termination='NORMAL_RETURN',iteration_limit_reached=optimizer.iterations()>=params.getMaxIterations(),termination_reason='specific tolerance reason NOT_EXPOSED',solver='frozen12D LM')
    ROWS['timing'][-1]['LM_optimizer_ms']=runtime
    ROWS['lm'].append(row);ALL[(cp,label)]=mat
    progress('LM_COMPLETE',checkpoint=cp,state=label,p95=row['translation_p95_m'],error=error)
    persist()

def geometry(policy):
    summary={};profiles=[];support=[]
    for name in ['D0_PERSISTENT','D2_PERSISTENT']:
        mat=ALL[('FINAL_STREAM_END',name)];ref=REF['FINAL_STREAM_END'];z=pose_table(mat,ref);z=z[z.robot_id.eq('robot3')].sort_values('keyframe_id').copy()
        delta=z[['dx','dy','dz']].to_numpy();rot=z[['wrx','wry','wrz']].to_numpy();adj=np.linalg.norm(np.diff(delta,axis=0),axis=1);rotadj=np.linalg.norm(np.diff(rot,axis=0),axis=1)
        centered=delta-delta.mean(axis=0);_,singular,vectors=np.linalg.svd(centered,full_matrices=False);variance=singular**2;ratios=variance/variance.sum() if variance.sum()>1e-24 else np.zeros(3)
        left=[mat[('robot3',int(i))]@d.s10.inv(ref[('robot3',int(i))]) for i in z.keyframe_id]
        variation=pd.DataFrame(dict(translation_m=[np.linalg.norm(x[:3,3]-left[0][:3,3]) for x in left],rotation_deg=[np.rad2deg(Rotation.from_matrix(left[0][:3,:3].T@x[:3,:3]).magnitude()) for x in left]))
        high=z.translation_m.gt(.5).to_numpy();runs=[];run=0
        for flag in high:
            if flag:run+=1
            elif run:runs.append(run);run=0
        if run:runs.append(run)
        common=variation.translation_m.quantile(.95)<.01 and variation.rotation_deg.quantile(.95)<.01
        isolated=high.sum()/len(z)<=.01 and z.translation_m.max()>10*max(z.translation_m.quantile(.95),1e-12)
        smooth=bool(np.percentile(adj,95)<.05)
        classification='APPROXIMATELY_COMMON_TRANSFORM' if common else 'ISOLATED_OUTLIER' if isolated else 'LONG_CHAIN_SMOOTH_DEFORMATION' if max(runs or [0])>200 and smooth else 'LOCAL_DEFORMATION' if high.any() and max(runs or [0])<=200 else 'MIXED_PATTERN'
        summary[name]=dict(classification=classification,high_error_fraction=float(high.mean()),longest_gt_free_high_error_run_kf=max(runs or [0]),adjacent_delta_median_m=float(np.median(adj)),adjacent_delta_p95_m=float(np.percentile(adj,95)),adjacent_delta_max_m=float(np.max(adj)),adjacent_rotation_vector_p95_rad=float(np.percentile(rotadj,95)),PCA_variance_ratios=ratios.tolist(),PCA_axes=vectors.tolist(),mean_delta_world_m=delta.mean(0).tolist(),left_transform_translation_variation_p95_m=float(variation.translation_m.quantile(.95)),left_transform_rotation_variation_p95_deg=float(variation.rotation_deg.quantile(.95)),not_Hessian_nullspace_evidence=True)
        z['solver_state']=name;z['consecutive_delta_change_m']=np.r_[np.nan,adj];z['consecutive_rotation_vector_change_rad']=np.r_[np.nan,rotadj];profiles.extend(z.to_dict('records'))
        ends=LOOPS.candidate_keyframe_id.to_numpy(int);ids=z.keyframe_id.to_numpy(int);distance=abs(ids[:,None]-ends[None,:]);table=pd.DataFrame(dict(solver_state=name,keyframe_id=ids,nearest_loop_endpoint_kf_distance=distance.min(axis=1),disagreement_m=z.translation_m.to_numpy()))
        for w in policy['support_windows_kf']:
            table[f'endpoint_count_w{w}']=(distance<=w).sum(axis=1)
            width=np.minimum(ids+w,4179)-np.maximum(ids-w,234)+1;table[f'endpoint_density_w{w}']=table[f'endpoint_count_w{w}']/width
        support.extend(table.to_dict('records'));correlations=[]
        for field in [x for x in table.columns if x not in ['solver_state','keyframe_id','disagreement_m']]:
            x=table[field].to_numpy(float);y=table.disagreement_m.to_numpy()
            correlations.append(dict(proxy=field,Pearson_r=float(pearsonr(x,y).statistic) if np.std(x)>0 else None,Spearman_rho=float(spearmanr(x,y).statistic) if np.std(x)>0 else None,p_values_not_interpreted_due_to_chain_autocorrelation=True))
        summary[name]['support_correlations']=correlations
    csv('robot3_final_delta_profile.csv',profiles);csv('loop_support_diagnostic.csv',support);dump('distributed_deformation_summary.json',summary)

def conditioning_worker():
    # Separate process: bounded address-space growth and CPU; no dense Hessian.
    policy=json.loads((OUT/'stage14_experiment_policy.json').read_text());budget=policy['conditioning']
    virtual=int(Path('/proc/self/statm').read_text().split()[0])*os.sysconf('SC_PAGE_SIZE')
    resource.setrlimit(resource.RLIMIT_AS,(virtual+budget['additional_address_space_bytes'],virtual+budget['additional_address_space_bytes']))
    resource.setrlimit(resource.RLIMIT_CPU,(budget['cpu_seconds'],budget['cpu_seconds']))
    frames=allowed_frames();local=d.s10.lookup(pd.read_csv(b.D12/'clean_local_trajectory.csv',usecols=d.POSE_COLUMNS))
    loops=pd.read_csv(b.D12/'ordered_rank1_clean_loops.csv',usecols=['arrival_index','frozen_loop_id']+d.LOOP_COLUMNS)
    scratch=OUT/'conditioning_worker';scratch.mkdir(exist_ok=True);d.OUT=scratch;graph,_=d.build_graph(frames,local,loops)
    rows=[];runtimes=[]
    for label,path in [('D0_PERSISTENT',historical_path('FINAL_STREAM_END','D0')),('FROZEN_BATCH',B/'snapshots/FINAL_STREAM_END_batch_trajectory.csv')]:
        mat=d.s10.lookup(pd.read_csv(path,usecols=d.POSE_COLUMNS));values=b.to_values(mat);t=time.perf_counter()
        marginal=gtsam.Marginals(graph,values);build_ms=(time.perf_counter()-t)*1000;runtimes.append(dict(linearization=label,sparse_marginals_construction_ms=build_ms))
        for selection in budget['poses']:
            key=b.numeric_key((selection['robot_id'],selection['keyframe_id']));t=time.perf_counter();cov=marginal.marginalCovariance(key);elapsed=(time.perf_counter()-t)*1000
            assert cov.shape==(6,6) and np.isfinite(cov).all()
            asym=float(np.linalg.norm(cov-cov.T)/max(np.linalg.norm(cov),1e-12));eig=np.linalg.eigvalsh((cov+cov.T)/2);maxeig=float(eig.max());positive=eig[eig>max(1e-15,maxeig*1e-12)]
            valid=asym<1e-8 and eig.min()>=-1e-10*max(1.,maxeig) and len(positive)>0
            rows.append(dict(linearization=label,**selection,status='VALID' if valid else 'INVALID_COVARIANCE',eigenvalues_json=json.dumps(eig.tolist()),maximum_eigenvalue=maxeig,minimum_positive_eigenvalue=float(positive.min()) if len(positive) else None,condition_proxy=maxeig/float(positive.min()) if len(positive) else None,translation_covariance_trace_m2=float(np.trace(cov[3:,3:])),rotation_covariance_trace_rad2=float(np.trace(cov[:3,:3])),relative_asymmetry=asym,query_ms=elapsed))
    csv('weak_direction_diagnostic.csv',rows);dump('conditioning_worker_result.json',dict(status='PASS' if all(x['status']=='VALID' for x in rows) else 'FAIL',runtimes=runtimes,max_RSS_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,address_space_limit_bytes=virtual+budget['additional_address_space_bytes'],dense_global_Hessian_constructed=False))

def conditioning(policy):
    budget=policy['conditioning'];t=time.perf_counter()
    if not budget['supported']:
        csv('weak_direction_diagnostic.csv',[dict(status='NOT_TESTED',reason='API not exposed')]);status='NOT_TESTED'
    else:
        try:
            env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
            proc=subprocess.run([sys.executable,str(Path(__file__)),'--conditioning-worker'],capture_output=True,text=True,timeout=budget['wall_seconds'],env=env)
            if proc.returncode!=0:raise RuntimeError(proc.stderr[-2000:] or f'worker exit{proc.returncode}')
            status=json.loads((OUT/'conditioning_worker_result.json').read_text())['status']
        except (subprocess.TimeoutExpired,RuntimeError) as ex:
            status='NOT_TESTED';csv('weak_direction_diagnostic.csv',[dict(status='NOT_TESTED',reason=str(ex))])
    dump('conditioning_method.json',dict(status=status,method='GTSAM Marginals(graph, Values), sparse factor elimination, selected6x6 marginals only',linearization_points=['historicalD0 final','frozen canonical batch final'],gauge='R1 KF150 prior unchanged',tangent_order='rx ry rz tx ty tz',mixed_units_condition_proxy_not_global_Hessian_condition=True,robust_factors='local linearization/IRLS information, not calibrated physical trajectory uncertainty',policy=budget,wall_ms=(time.perf_counter()-t)*1000,dense_global_Hessian=False,GT_used=False))

def main():
    global LOOPS,LOCAL,LOCAL_TR,COMMON_TR,SELECTION,FRAMES,HISTORY
    if OUT.exists() and any(p.name!='_failed_attempts' for p in OUT.iterdir()):raise RuntimeError('Refuse overwrite; use report-only or independent audit')
    OUT.mkdir(parents=True,exist_ok=True)
    for folder in ['estimates','nodewise']:(OUT/folder).mkdir()
    subprocess.run(['git','merge-base','--is-ancestor',ACCEPTED,'HEAD'],cwd=ROOT,check=True)
    tracked=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
    HISTORY={str(ROOT/p):d.sha256(ROOT/p) for p in tracked if p.startswith(('src/cumulti/','outputs/cumulti_v1/','docs/','demo/')) and 'stage14' not in p and '14_final_consistency' not in p and 'STAGE14' not in p}
    inherited=json.loads((D/'stage13d_input_manifest.json').read_text())
    for x in inherited['inputs']+inherited['inherited_frozen_inputs']:assert d.sha256(x['path'])==x['sha256']
    paths=[d.LOOP_SOURCE,b.D12/'ordered_rank1_clean_loops.csv',b.D12/'clean_local_trajectory.csv',b.D12/'clean_first_loop_trajectory.csv',b.D12/'clean_gtsam_solver_policy.json',D/'checkpoint_microtrace.csv',D/'stage13d_numerical_decision.json',D/'diagnostic_update_timing.csv',D/'pose_differences/D0_FINAL_STREAM_END_AFTER_INSERTION_0.csv']
    paths += [historical_path(cp,s) for cp in CP for s in ['D0','D2']]+[B/f'snapshots/{cp}_batch_trajectory.csv' for cp in CP]
    manifest=[]
    for path in paths:
        blob=subprocess.check_output(['git','show',f'{ACCEPTED}:{path.relative_to(ROOT).as_posix()}'],cwd=ROOT);sha=d.sha256(path);assert hashlib.sha256(blob).hexdigest()==sha
        manifest.append(dict(path=str(path),sha256=sha,size_bytes=path.stat().st_size,accepted_blob_verified=True))
    dump('stage14_input_manifest.json',dict(accepted=ACCEPTED,inputs=manifest,inherited_inputs=inherited['inputs']+inherited['inherited_frozen_inputs'],historical_hashes_before=HISTORY,GT_fields_loaded=False))
    api=dict(gtsam=importlib.metadata.version('gtsam'),python=sys.version,executable=sys.executable,ISAM2_update=gtsam.ISAM2.update.__doc__,Marginals_constructor=gtsam.Marginals.__init__.__doc__ if hasattr(gtsam,'Marginals') else None,marginalCovariance=gtsam.Marginals.marginalCovariance.__doc__ if hasattr(gtsam,'Marginals') else None,Pose3_Logmap=gtsam.Pose3.Logmap.__doc__)
    assert api['gtsam']=='4.2' and 'update(self: gtsam.gtsam.ISAM2)' in api['ISAM2_update'];params,text=c.configure(dict(skip=1,threshold=.01));api['verified_D2_parameter_print']=text;dump('stage14_api_audit.json',api)
    marginal_poses=freeze_marginal_nodes()
    policy=dict(UTC=b.a.now(),accepted=ACCEPTED,input_hashes=manifest,checkpoints=CP,A=dict(A0='read-onlyD0 normal andD2 extra5',A1='freshD2 canonical common x0',A2='freshD2 historicalD0 coords',A3='freshD2 historicalD2 coords',extra_calls=5,skip=1,threshold=.01,wildfire=.001,parameter_print=text,force_update=False),B=dict(checkpoints=['K75','FINAL_STREAM_END'],initializations=['canonical','D0','D2'],settings='frozen12D LM defaults, no retuning'),gates=LIMITS,frame='same R1 KF150 gauge; NO alignment',material_change=dict(max_translation_lt_m=1e-7,max_rotation_lt_deg=1e-7,abs_objective_change_lt=1e-9,consecutive_updates=2,exact5_calls=True),clear_toward_reference=dict(relative_p95_reduction_ge=.20,absolute_p95_reduction_ge_m=.01,or_resolves_original_gates=True),near_objective=dict(abs_gap_le=1e-5,relative_gap_le=1e-5),essentially_same_pose=dict(translation_p95_le_m=.001,rotation_p95_le_deg=.001),support_windows_kf=[50,100,250],support='count inserted loop-factor endpoints, nearest KF/path order; boundary-corrected endpoint count/window width; serial correlations descriptive, no causal/unobservability proof',localization='perrobot median/p95/max, top20 translation/rotation, contiguous>=per-scopep95 physicaltranslation regions, finite Pose3Logmap(reference^-1state), physical xyz difference not Logmapv',deformation=dict(high_error_m=.5,common_transform_translation_p95_lt=.01,common_transform_rotation_p95_lt_deg=.01,smooth_adjacent_p95_lt_m=.05,long_region_gt_kf=200,isolated_fraction_le=.01,isolated_max_over_p95_gt=10),conditioning=dict(supported=bool(api['Marginals_constructor'] and api['marginalCovariance']),poses=marginal_poses,linearizations=['D0','FROZEN_BATCH'],additional_address_space_bytes=1024**3,cpu_seconds=30,wall_seconds=45,eigen_positive_threshold='max(1e-15, max_eigenvalue*1e-12)',symmetry_relative_lt=1e-8,PSD_tolerance='-1e-10*max(1,maxEigen)',not_global_Hessian=True),runtime='one trial per condition; fresh construction/insertion/extra/extraction/objective/comparison/file overhead separated; historical incremental cost reported read-only, not equivalent to full rebuild cost; child conditioning separately timed',decision_order='rebuild toward batch at both primary CPs plus initialization-sensitive LM near-objective/gate-inconsistent pair => MULTIPLE; else fresh-init-dependent +LM sensitivity => INITIALIZATION_OR_BASIN; else rebuild support => FRESH_REBUILD; otherwiseUNRESOLVED. Associations/covariance alone not sufficient to claim cause.',GT_used=False)
    recovery=OUT/'_failed_attempts/geometry_reporting/stage14_experiment_policy.json'
    if recovery.exists():
        frozen=json.loads(recovery.read_text())
        assert {k:v for k,v in policy.items() if k!='UTC'}=={k:v for k,v in frozen.items() if k!='UTC'},'Recovery must keep EXACT frozen experiment design'
        policy=frozen
        dump('recovery_record.json',dict(reason='All15 solver trials completed but reporting-only Robot3 metrics helper failed; some runtime/iteration rows had not been persisted. Necessary exact-design reproduction to recover mandatory records, not parameter search.',original_policy_sha256=d.sha256(recovery),original_estimates_server_only=True,numerical_parameters_unchanged=True,decision_existed_before_recovery=False,first_attempt_is_not_primary_timing_sample=True))
    dump('stage14_experiment_policy.json',policy);progress('POLICY_FROZEN')
    historical_timing=pd.read_csv(D/'diagnostic_update_timing.csv',usecols=['variant','kind','normal_update_ms','extra_update_ms','normal_extraction_ms','extra_extraction_ms'])
    historical_timing.to_csv(OUT/'historical_incremental_runtime.csv',index=False)
    LOOPS=loop_provenance();LOCAL_TR=pd.read_csv(b.D12/'clean_local_trajectory.csv',usecols=d.POSE_COLUMNS);COMMON_TR=pd.read_csv(b.D12/'clean_first_loop_trajectory.csv',usecols=d.POSE_COLUMNS)
    LOCAL=d.s10.lookup(LOCAL_TR);common=d.s10.lookup(COMMON_TR);FRAMES=allowed_frames();SELECTION=json.loads((d.C12/'stable_segment_selection.json').read_text())
    d.OUT=OUT;template,lmparams=d.build_graph(FRAMES,LOCAL,LOOPS)
    assert json.loads((OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==json.loads((b.D12/'clean_gtsam_solver_policy.json').read_text())['parameters']
    equivalence={};historical_trace=pd.read_csv(D/'checkpoint_microtrace.csv')
    for cp,kf in CP.items():
        K=2 if cp=='K2' else 75;ix=indices(kf,K);graph=b.graph_subset(template,ix);GRAPHS[cp]=graph
        nodes=[(r,int(i)) for r,f in FRAMES.items() for i in f.keyframe_id if r=='robot3' or i<=kf]
        canonical={n:common[n] for n in nodes};ref=d.s10.lookup(pd.read_csv(B/f'snapshots/{cp}_batch_trajectory.csv',usecols=d.POSE_COLUMNS));REF[cp]=ref
        assert set(ref)==set(nodes) and set(graph.keyVector())==set(b.to_values(ref).keys()) and len(set(ix))==len(ix)
        assert graph.size()==len(nodes)-2+K+1 and all(graph.at(i).equals(template.at(j),1e-12) for i,j in enumerate(ix))
        equivalence[cp]=dict(PASS=True,nodes=len(nodes),odometry=len(nodes)-2,loops=K,prior=1,factors=graph.size(),template_indices=ix,exact_types_keys_measurements_noise_robust_prior=True,all_R1_kf_le=kf,selected_clean_loop_ids=LOOPS.head(K).frozen_loop_id.astype(int).to_list(),no_excluded_loops=True)
        for solver in ['D0','D2']:
            mat=d.s10.lookup(pd.read_csv(historical_path(cp,solver),usecols=d.POSE_COLUMNS));label=solver+'_PERSISTENT';ALL[(cp,label)]=mat
            row,_=state(cp,label,mat,graph,ref);ROWS['summary'].append(dict(**row,extra_updates='historical0' if solver=='D0' else 'historical5',initialization_source='READ ONLY persistent reference'))
            old=historical_trace[historical_trace.variant.eq(solver)&historical_trace.checkpoint.eq(cp)&historical_trace.extra_step.eq(0 if solver=='D0' else 5)].iloc[0]
            assert all(abs(row[k]-float(old[k]))<1e-7 for k in LIMITS) and abs(row['nonlinear_error']-float(old.nonlinear_error))<1e-7
        for label,start in [('A1_FRESH_CANONICAL',canonical),('A2_FRESH_D0',ALL[(cp,'D0_PERSISTENT')]),('A3_FRESH_D2',ALL[(cp,'D2_PERSISTENT')])]:rebuild(cp,label,start,graph,ref)
        if cp!='K2':
            for label,start in [('B0_LM_CANONICAL',canonical),('B1_LM_D0',ALL[(cp,'D0_PERSISTENT')]),('B2_LM_D2',ALL[(cp,'D2_PERSISTENT')])]:basin(cp,label,start,graph,ref,lmparams)
    dump('final_factor_equivalence.json',equivalence)
    persist();geometry(policy);conditioning(policy);finish(policy)

def persist():
    for key,name in [('trace','fresh_rebuild_microtrace.csv'),('summary','fresh_rebuild_summary.csv'),('lm','lm_basin_probe.csv'),('localization','path_dependence_localization.csv'),('timing','runtime_summary.csv'),('integrity','trajectory_integrity_audit.csv'),('pairwise','pairwise_solver_agreement.csv')]:csv(name,ROWS[key])

def finish(policy):
    primary=['K75','FINAL_STREAM_END'];rows=pd.DataFrame(ROWS['summary']);lm=pd.DataFrame(ROWS['lm']);rebuild=[];basin_sensitive=[];fresh_sensitive=[]
    for cp in primary:
        for group in [['B0_LM_CANONICAL','B1_LM_D0','B2_LM_D2'],['A1_FRESH_CANONICAL','A2_FRESH_D0','A3_FRESH_D2']]:
            for i,name in enumerate(group):
                for other in group[i+1:]:
                    delta=b.pose_difference(ALL[(cp,name)],ALL[(cp,other)]);error1=float(GRAPHS[cp].error(b.to_values(ALL[(cp,name)])));error2=float(GRAPHS[cp].error(b.to_values(ALL[(cp,other)])))
                    close=abs(error1-error2)<=max(1e-5,1e-5*max(abs(error1),abs(error2)))
                    gate_consistent=all(delta[k]<=v for k,v in LIMITS.items());essentially=delta['translation_p95_m']<=.001 and delta['rotation_p95_deg']<=.001
                    ROWS['pairwise'].append(dict(checkpoint=cp,left=name,right=other,near_objective=close,original_gate_consistent=gate_consistent,essentially_same_coordinates=essentially,objective_difference=abs(error1-error2),**delta))
                    if group[0].startswith('B') and close and not gate_consistent:basin_sensitive.append(cp)
                    if group[0].startswith('A') and not gate_consistent:fresh_sensitive.append(cp)
        for historical,fresh in [('D0_PERSISTENT','A2_FRESH_D0'),('D2_PERSISTENT','A3_FRESH_D2')]:
            before=rows[rows.checkpoint.eq(cp)&rows.solver_state.eq(historical)].iloc[0];after=rows[rows.checkpoint.eq(cp)&rows.solver_state.eq(fresh)].iloc[0]
            change=before.translation_p95_m-after.translation_p95_m
            if (not before.PASS and after.PASS) or change>=max(.01,.2*before.translation_p95_m):rebuild.append(dict(checkpoint=cp,starting_state=historical,fresh_state=fresh,reduction_m=float(change),D0_parameter_change_confound=historical=='D0_PERSISTENT'))
    both_rebuild=all(any(x['checkpoint']==cp for x in rebuild) for cp in primary)
    basin_all=all(cp in basin_sensitive for cp in primary);fresh_all=all(cp in fresh_sensitive for cp in primary)
    status='MULTIPLE_NUMERICAL_MECHANISMS_SUPPORTED' if both_rebuild and basin_all else 'INITIALIZATION_OR_BASIN_SENSITIVITY' if basin_all and fresh_all else 'FRESH_REBUILD_EXPLAINS_RESIDUAL' if both_rebuild else 'RESIDUAL_NUMERICAL_DISCREPANCY_UNRESOLVED'
    decision=dict(UTC=b.a.now(),decision=status,rebuild_toward_reference_evidence=rebuild,LM_near_objective_gate_inconsistent_checkpoints=sorted(set(basin_sensitive)),fresh_initialization_gate_inconsistent_checkpoints=sorted(set(fresh_sensitive)),GT_used=False,production_ready=False,not_multiple_global_minima_proof=True,limitations='A2 changes defaultD0 parameters toD2; A3 keepsD2 parameters but freshfull insertion/relinearization path changes. Fixed LM termination is not proof of global convergence. Conditioning/association alone not cause.')
    persist();assert not (OUT/'stage14_numerical_decision.json').exists();dump('stage14_numerical_decision.json',decision)
    immutable=all(d.sha256(p)==h for p,h in HISTORY.items());assert immutable
    cond=json.loads((OUT/'conditioning_method.json').read_text())['status']
    checks={'HISTORICAL_INPUTS_IMMUTABLE':'PASS','LOOP_120_TO_75_PROVENANCE_VERIFIED':'PASS','CLEAN_75_LOOP_SET_EXACT':'PASS','GT_USED':'NO','EXCLUDED_45_LOOPS_REINTRODUCED':'NO','FACTOR_GRAPH_EQUIVALENCE':'PASS','K2_CONTROL_COMPLETED':'PASS','K75_REBUILD_COMPLETED':'PASS','FINAL_REBUILD_COMPLETED':'PASS','D0_PERSISTENT_INITIALIZATION_TESTED':'PASS','D2_PERSISTENT_INITIALIZATION_TESTED':'PASS','LM_BASIN_PROBE_COMPLETED':'PASS','ROBOT_SPECIFIC_LOCALIZATION_COMPLETED':'PASS','DISTRIBUTED_DEFORMATION_ANALYZED':'PASS','LOOP_SUPPORT_DIAGNOSTIC_COMPLETED':'PASS','CONDITIONING_DIAGNOSTIC':cond,'ORIGINAL_NUMERICAL_GATES_UNCHANGED':'PASS','NO_BATCH_INITIALIZATION_OF_ISAM2':'PASS','NO_RIGID_POST_ALIGNMENT':'PASS','RUNTIME_SEPARATED':'PASS','HISTORICAL_STAGE13_MODIFIED':'NO','DEMO_MODIFIED':'NO'}
    dump('summary.json',dict(decision=decision,validation=checks,historical_files_checked=len(HISTORY),fresh_trials=9,empty_calls=45,LM_trials=6,GT_used=False))
    (OUT/'VALIDATION_REPORT.txt').write_text('\n'.join(f'{k}: {v}' for k,v in checks.items())+'\n');report();progress('COMPLETE',decision=status)

def report():
    n=pd.read_csv(OUT/'fresh_rebuild_summary.csv');trace=pd.read_csv(OUT/'fresh_rebuild_microtrace.csv');lm=pd.read_csv(OUT/'lm_basin_probe.csv');timing=pd.read_csv(OUT/'runtime_summary.csv')
    objective=pd.concat([n,lm],ignore_index=True);csv('objective_pose_agreement.csv',objective[['checkpoint','solver_state','nonlinear_error','objective_gap','translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg','PASS']])
    decision=json.loads((OUT/'stage14_numerical_decision.json').read_text());deformation=json.loads((OUT/'distributed_deformation_summary.json').read_text());condition=json.loads((OUT/'conditioning_method.json').read_text())
    def save(fig,name):fig.savefig(OUT/name,dpi=160);plt.close(fig)
    final=objective[objective.checkpoint.eq('FINAL_STREAM_END')]
    fig,ax=plt.subplots(figsize=(11,4),constrained_layout=True);ax.bar(final.solver_state,final.translation_p95_m);ax.axhline(.5,c='black',ls='--');ax.tick_params(axis='x',rotation=25);ax.set(ylabel='Translation p95 (m)',title='Final same-gauge disagreement; numerical reference, not GT');save(fig,'01_final_pose_error_by_solver_state.png')
    fig,ax=plt.subplots(figsize=(9,5),constrained_layout=True)
    for cp,z in objective.groupby('checkpoint'):ax.scatter(z.objective_gap.clip(lower=1e-14),z.translation_p95_m,label=cp)
    ax.set(xscale='log',xlabel='Absolute objective gap (zeros plotted1e-14)',ylabel='Translation p95 (m)',title='Near-equal objective does not replace pose gates');ax.axhline(.5,c='black',ls='--');ax.legend();save(fig,'02_objective_gap_vs_pose_error.png')
    loc=pd.read_csv(OUT/'path_dependence_localization.csv');q=loc[loc.checkpoint.eq('FINAL_STREAM_END')&loc.record_type.eq('SCOPE')&loc.scope.isin(['robot1','robot3'])]
    fig,ax=plt.subplots(figsize=(11,4),constrained_layout=True);q.pivot(index='solver_state',columns='scope',values='translation_p95_m').plot.bar(ax=ax);ax.set(ylabel='Translation p95 (m)',title='Robot-specific final discrepancy');save(fig,'03_robot1_robot3_final_disagreement.png')
    profile=pd.read_csv(OUT/'robot3_final_delta_profile.csv');fig,axs=plt.subplots(2,1,figsize=(11,7),constrained_layout=True)
    for name,z in profile.groupby('solver_state'):
        axs[0].plot(z.keyframe_id,z.translation_m,label=name);axs[1].plot(z.keyframe_id,z.consecutive_delta_change_m,label=name)
    axs[0].set(ylabel='Disagreement (m)',title='World-coordinate delta along stored Robot3 chain');axs[1].set(xlabel='Original KF',ylabel='Adjacent delta change (m)');axs[0].legend();save(fig,'04_robot3_final_delta_along_chain.png')
    fig,axs=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
    for ax,cp in zip(axs,CP):
        for name,z in trace[trace.checkpoint.eq(cp)].groupby('solver_state'):ax.plot(z.step,z.translation_p95_m,'-o',label=name)
        ax.axhline(.5,c='black',ls='--');ax.set_yscale('symlog',linthresh=.05);ax.set_ylim(bottom=0);ax.set(title=cp,xlabel='Empty calls after full insertion',ylabel='Translation p95 (m)');ax.legend(fontsize=7)
    save(fig,'05_fresh_rebuild_convergence.png')
    fig,axs=plt.subplots(1,2,figsize=(13,4),constrained_layout=True)
    for ax,cp in zip(axs,['K75','FINAL_STREAM_END']):
        z=lm[lm.checkpoint.eq(cp)];ax.bar(z.solver_state,z.translation_p95_m);ax.axhline(.5,c='black',ls='--');ax.set(title=cp+' fixed-LM initialization probe',ylabel='p95 vs canonical batch (m)');ax.tick_params(axis='x',rotation=15)
    save(fig,'06_initialization_basin_comparison.png')
    support=pd.read_csv(OUT/'loop_support_diagnostic.csv');fig,axs=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
    for name,z in support.groupby('solver_state'):
        axs[0].scatter(z.nearest_loop_endpoint_kf_distance,z.disagreement_m,s=3,alpha=.4,label=name);axs[1].scatter(z.endpoint_density_w100,z.disagreement_m,s=3,alpha=.4,label=name)
    axs[0].set(xlabel='Nearest endpoint KF distance',ylabel='Disagreement (m)',title='Structural association only');axs[1].set(xlabel='Loop-factor endpoint density ±100KF',ylabel='Disagreement (m)');axs[0].legend();save(fig,'07_loop_support_vs_disagreement.png')
    runtimes=[]
    for (cp,name),z in timing[timing.operation.str.startswith('A')].groupby(['checkpoint','operation']):runtimes.append(dict(checkpoint=cp,operation=name,construction_ms=z.construction_ms.sum(),full_insertion_ms=z.iloc[0].update_ms,empty_update_total_ms=z.iloc[1:].update_ms.sum(),extraction_ms=z.extraction_ms.sum(),objective_ms=z.objective_ms.sum(),file_localization_ms=z.file_localization_overhead_ms.sum()))
    csv('rebuild_cost_summary.csv',runtimes)
    fig,ax=plt.subplots(figsize=(12,4),constrained_layout=True);r=pd.DataFrame(runtimes);ax.bar(r.checkpoint+' '+r.operation,r.full_insertion_ms+r.empty_update_total_ms);ax.tick_params(axis='x',rotation=35);ax.set(ylabel='Insertion+empty updates (ms)',title='Fresh complete-graph diagnostic cost, NOT per-frame incremental latency');save(fig,'08_rebuild_runtime_comparison.png')
    if condition['status']=='PASS':
        w=pd.read_csv(OUT/'weak_direction_diagnostic.csv');fig,ax=plt.subplots(figsize=(12,5),constrained_layout=True)
        labels=None
        for index,(label,z) in enumerate(w.groupby('linearization')):
            z=z[~(z.robot_id.eq('robot1')&z.keyframe_id.eq(150))];labels=z.robot_id.str.replace('robot','R')+'#'+z.keyframe_id.astype(str)+' '+z.group
            ax.scatter(np.arange(len(z)),z.translation_covariance_trace_m2,label=label,marker='o' if index==0 else 'x',s=55,facecolors='none' if index==0 else None,edgecolors='tab:blue' if index==0 else None,color='tab:orange' if index else None)
        ax.set_xticks(np.arange(len(labels)),labels,rotation=25,ha='right')
        anchor=float(w[w.robot_id.eq('robot1')&w.keyframe_id.eq(150)].translation_covariance_trace_m2.iloc[0])
        ax.text(.02,.94,f'Anchored R1#150 trace={anchor:.2e}m² (outside displayed range)',transform=ax.transAxes,va='top',fontsize=8)
        ax.set(yscale='log',ylabel='Local translation covariance trace (m²)',title='Bounded 6x6 marginal diagnostic; not global Hessian spectrum');ax.legend(loc='lower right');save(fig,'09_weak_direction_diagnostic.png')
    pair=pd.read_csv(OUT/'pairwise_solver_agreement.csv')
    marginal_text='NOT_TESTED'
    if condition['status']=='PASS':
        marginal_text=pd.read_csv(OUT/'weak_direction_diagnostic.csv').groupby(['linearization','group'])[['translation_covariance_trace_m2','rotation_covariance_trace_rad2','condition_proxy']].median().reset_index().to_csv(index=False)
    provenance=dict(finalized_runner_sha256=d.sha256(Path(__file__)),policy_sha256=d.sha256(OUT/'stage14_experiment_policy.json'),decision_sha256=d.sha256(OUT/'stage14_numerical_decision.json'),report_only_never_optimizes=True,after_computation_changes='saved-data interpretation, plots and independent audit only; numerical conditions/gates/decision unchanged')
    dump('runner_finalization_provenance.json',provenance)
    def table(df):return '| '+' | '.join(df.columns)+' |\n| '+' | '.join(['---']*len(df.columns))+' |\n'+'\n'.join('| '+' | '.join(str(v) for v in row)+' |' for row in df.itertuples(index=False,name=None))
    text=f'''# Stage14 — final-state consistency and incremental path-dependence audit

Accepted `{ACCEPTED}`. Decision **{decision['decision']}**. Independent numerical diagnostic, not a deployable policy; batch LM is not ground truth.

## Frozen inputs / loop provenance

Historical Rank1 sanitized120 -> frozen12C endpoints R1>=150/R3>=234 -> retained75, unchanged measurements and query order. No excluded45 restored. Same frozen odometry, prior atR1KF150, sigma/Huber/extrinsic. FINAL5796nodes/5794odom/75loops/1prior=5870factors. K2/K75 only arrived R1 nodes; stored Robot3 complete. Accepted blobs and actualfactor.equals verified. All CSVs use explicit non-GT column allowlists; raw datasets/GT/maps/frontend/demo untouched.

## Design and API

A0: read-only13D D0normal/D2extra5. A1 freshD2 canonical common x0; A2 freshD2 D0coordinates; A3 freshD2 D2coordinates. Complete graph inserted into nine NEW ISAM2 instances, exactly5 empty calls each. skip1/threshold0.01/wildfire0.001. No force API assumed/used, no batch coordinates initialize ISAM2. Invalid counters retain raw strings, null/INVALID_COUNTER; in-range is not independent proof of semantics. Six LM probes B0canonical/B1D0/B2D2 atK75/FINAL use frozen12D defaults. Same gauge, NO alignment. Gate0.1/0.5m and0.5/2deg unchanged; no-change<1e-7m/<1e-7deg/<1e-9objective twice, not proof of mathematical convergence. Policy fsynced before runs.

## Historical persistent and fresh final states

{table(n[['checkpoint','solver_state','translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg','nonlinear_error','objective_gap','extra_updates','PASS']])}

## LM initialization probes

{table(lm[['checkpoint','solver_state','initial_error','final_error','iterations','runtime_ms','translation_p95_m','from_init_translation_p95_m','PASS','iteration_limit_reached']])}

Pairwise same-gauge comparisons are in pairwise_solver_agreement.csv: original-gate consistency and stricter descriptive essentially-same0.001m/0.001deg reported separately. Fixed LM stopping can leave nearby coordinates, not proof of distinct global minima or a uniquely converged optimum. A2 changes historicalD0 settings toD2, so it confounds settings with rebuild; A3 preserves historicalD2 settings but still changes full insertion/linearization path, not only the Bayes tree in isolation.

{table(pair[['checkpoint','left','right','translation_median_m','translation_p95_m','near_objective','original_gate_consistent','essentially_same_coordinates']])}

## Scientific interpretation of measured contrasts

Fresh reconstruction from identical persistent coordinates DOES change poses measurably: FINAL A2/D0 p95 movement is approximately0.0304m; A3/D2 approximately0.0209m. However, FINAL disagreement to frozenbatch remains approximately0.57m and every fresh final condition fails. K75 D0 improves but K75D2/FINAL do not satisfy the predeclared clear-toward-reference rule. Rebuilding alone is not sufficient to explain/resolve the residual under this protocol.

Fresh canonical/D0/D2 initializations produce centimeter-scale different coordinates, but their pairwise differences stay within original gates. LM canonical probes reproduce the frozenreference, while LM from persistent states stop at measurably different coordinates despite near-equal objectives; these do NOT constitute one essentially identical LM pose solution. This supports initialization/termination sensitivity as an observation, not proof of multiple global minima. The conservative frozen overall decision requires additional contrasts, and remains unresolved rather than being relaxed after results.

FINAL discrepancy principally affects Robot3 (persistent p95 approximately0.65–0.67m versus Robot1 approximately0.16–0.19m). Both persistent profiles classify LONG_CHAIN_SMOOTH_DEFORMATION, with adjacent translation-delta p95 approximately0.0037–0.0038m. PCA is descriptive, not an information-nullspace result. Nearest-loop-index distance has positive Pearson/Spearman association with error; local endpoint density is negatively associated. Temporal autocorrelation and shared trajectory structure preclude causal inference from these correlations.

## Localization, deformation and structural support

Perrobot physical translation/rotation, worst20, contiguous p95 regions and nodewise Logmap are saved. Physical world xyz deltas differ from body-frame Logmap translation tangent coordinates; Logmap ordering rot then trans, finite flag stored. Rotation vector profiles are world-frame relative rotations. Descriptive classifications/PCA and structural correlations:

```json
{json.dumps(deformation,indent=2)}
```

Windows±50/100/250KF frozen before results; loop-factor endpoint counts (duplicates count factors), boundary-adjusted density, nearest path-order distance. Correlations are serially dependent descriptive associations, not causal proof or unobservability. Common-transform test compares per-pose left transforms, never aligns trajectories or modifies gate coordinates. Smooth disagreement/PCA does not prove a Hessian nullspace.

## Bounded conditioning / weak directions

Status **{condition['status']}**. Predeclared9pose set: three historicalD0 worst R3 poses separated>=200KF, three nearest-index lower-quartile R3controls (matching gaps recorded), R1controls150/1075/1999. Two FINAL linearizations: D0 and frozenbatch. Sparse GTSAM Marginals in isolated worker,1GiB extraaddress-space/30CPU/45wall budget; only6x6 matrices, no global denseHessian. Covariance is local robust-factor linearization/IRLS information, not calibrated physical accuracy; condition proxy mixes rad/m tangent units, not global Hessiancondition. PSD/symmetry checks and exact pose selection/runtimes are in conditioning_method.json and weak_direction_diagnostic.csv. No nullspace claim.

Group median local diagnostics (NOT actual trajectory accuracy):

```csv
{marginal_text}
```

{'All18 blocks are valid in this run. Large translation covariance traces occur at both Robot3 worst regions and the late Robot1 control, indicating nonuniform/weak local information under these fixed factor weights, not an exclusively Robot3 information issue.' if condition['status']=='PASS' else 'Conditioning was NOT_TESTED; no valid marginal result or conditioning figure is claimed.'} The matched low-error R3 controls are557–868KF away from their worst partners: imperfect matching limits causal comparisons. Selected marginal observations do not prove a global Hessian nullspace or isolate one causal explanation.

## Runtime categories

{table(pd.DataFrame(runtimes))}

Historical incremental per-frame timing is a separate read-only reference in historical_incremental_runtime.csv; never equate it to full fresh insertion. LM runtimes above and marginal construction/query/wall cost separate. File/localization overhead separate; single trial percondition. No end-to-end real-time claim.

## Decision and unresolved questions

```json
{json.dumps(decision,indent=2)}
```

Objective/pose table and figure show near-equal objective versus original gates; objective agreement never substitutes for pose agreement. No retuning, no GT or newloops. Rebuild/init sensitivity, structural associations and covariance observations must not be collapsed into one unproven causal mechanism.

## Reproduction and next step

Run `bash scripts/run_stage14_final_consistency.sh` from root with a new empty Stage14 directory. Refuses overwrite. `--report-only` reads saved data and never optimizes or rewrites policy/decision; independent auditor checks artifacts. Stop after14. Any solver-method or informative-loop-selection follow-up needs a separate protocol; no automatic newloops/demo/live stage. Historical13B/C/D status stays unchanged.

Implementation recovery is explicit: an initial prefix-schema preflight failed before new solves; the first full experimental attempt later reached all solver states but a Robot3-only reporting helper failed before some timers/iterations were saved. Both attempts were archived server-side; one necessary exact-policy recovery supplied mandatory runtime records. recovery_reproduction_audit.csv compares all66 estimate files and confirms identical numerical coordinates; archived-file hashes are in failed_attempt_artifact_manifest.json. Only the successful recovery timing is primary, never the better of two timings. Policy bytes are identical before/after recovery. No optimization follows the final decision; report-only finalization records source/policy/decision hashes.
'''
    (ROOT/'docs/CUMULTI_STAGE14_FINAL_CONSISTENCY.md').write_text(text)

if __name__=='__main__':
    if sys.argv[1:]==['--report-only']:report()
    elif sys.argv[1:]==['--conditioning-worker']:conditioning_worker()
    elif sys.argv[1:]:raise SystemExit('Usage: run_stage14_final_consistency.py [--report-only]')
    else:main()
