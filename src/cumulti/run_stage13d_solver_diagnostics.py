#!/usr/bin/env python3
"""Targeted, independent numerical diagnostics; never changes historical mains.

No GT/scan/map accesses. Saved batch references are evaluation-only. Missing
BEFORE_EVENT references use frozen LM defaults and frozen common initialization.
"""
from __future__ import annotations
import hashlib
import importlib.metadata
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from decimal import Decimal, InvalidOperation
from pathlib import Path
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import run_stage13c_isam2_convergence as c

b=c.b; d=c.d; ROOT=c.ROOT; B=b.OUT; C=c.OUT
OUT=ROOT/'outputs/cumulti_v1/13d_solver_diagnostics'
ACCEPTED='49a53e89e8651ebfb31d2865d6dc9c3b895b2165'
CP={'FIRST_LOOP':283,'K2':378,'K10':638,'K40':1014,'K75':1967,'FINAL_STREAM_END':1999}
VARIANTS=[dict(name='D0',skip=10,threshold=.1,extra=0),dict(name='D1',skip=10,threshold=.1,extra=5),dict(name='D2',skip=1,threshold=.01,extra=5)]
ROWS={k:[] for k in ['counters','trace','timing','localization','residuals','integrity','references','reproduction','events']}
EQUIVALENCE={}; HISTORY={}; LIMITS={}; REFS={}; MANIFEST={}

def dump(name,data):
    p=OUT/name;p.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    with p.open('rb') as f:os.fsync(f.fileno())

def csv(name,data):
    (data if isinstance(data,pd.DataFrame) else pd.DataFrame(data)).to_csv(OUT/name,index=False)

def progress(phase,**kw):
    row=dict(UTC=b.a.now(),phase=phase,**kw);ROWS['events'].append(row)
    print(json.dumps(row),flush=True);dump('progress.json',row);dump('execution_events.json',ROWS['events'])

def checked_counter(raw,nodes,method,context,**kw):
    text=str(raw);value=None;status='UNVERIFIED'
    try:
        x=Decimal(text)
        if x.is_finite() and x==x.to_integral_value():
            integer=int(x)
            if 0<=integer<=nodes:value=integer;status='IN_RANGE_UNVERIFIED_SEMANTICS'
            else:status='INVALID_COUNTER'
        elif text not in ['NOT_EXPOSED','NOT_APPLICABLE']:status='INVALID_COUNTER'
    except InvalidOperation:pass
    record=dict(context=context,method=method,raw_value=text,graph_nodes=nodes,
        diagnostic_status=status,diagnostic_value=value,physical_rule='0 <= integer counter <= graph nodes',**kw)
    ROWS['counters'].append(record)
    return record

def counters(result,nodes,context,**kw):
    out={}
    for short,method in [('relinearized','getVariablesRelinearized'),('reeliminated','getVariablesReeliminated')]:
        raw=getattr(result,method)() if hasattr(result,method) else 'NOT_EXPOSED'
        r=checked_counter(raw,nodes,method,context,**kw)
        out.update({short+'_raw':r['raw_value'],short+'_status':r['diagnostic_status'],short+'_valid_count':r['diagnostic_value']})
    return out

def historical_counters():
    # dtype=str avoids converting a 64-bit unsigned/pointer-like integer to float.
    for name in ['per_event_solver_diagnostics.csv','extra_update_diagnostic.csv','extra_update_effects.csv']:
        table=pd.read_csv(C/name,dtype=str,keep_default_na=False)
        for index,row in table.iterrows():
            if name=='extra_update_effects.csv':nodes=3946+int(loops_for_history[int(row.arrival_index)])-149
            else:nodes=int(row.graph_nodes) if 'graph_nodes' in row else 3946+int(row.kf)-149
            for short,method in [('relinearized','getVariablesRelinearized'),('reeliminated','getVariablesReeliminated')]:
                checked_counter(row['variables_'+short],nodes,method,'HISTORICAL_13C',source_file=name,source_row=int(index)+2,variant=row.get('variant','V4'),extra_step=row.get('extra_step',''))

def tiny_api_test():
    records=[]
    for v in [VARIANTS[0],VARIANTS[2]]:
        for repetition in range(3):
            p,text=c.configure(v);isam=gtsam.ISAM2(p)
            noise=gtsam.noiseModel.Diagonal.Sigmas(np.ones(6))
            graph=gtsam.NonlinearFactorGraph();values=gtsam.Values()
            graph.add(gtsam.PriorFactorPose3(0,gtsam.Pose3(),noise))
            step=gtsam.Pose3(gtsam.Rot3(),np.array([1.,0.,0.]))
            graph.add(gtsam.BetweenFactorPose3(0,1,step,noise));values.insert(0,gtsam.Pose3());values.insert(1,step)
            for phase in ['NEW_FACTORS','EMPTY','REPEATED_EMPTY_2','REPEATED_EMPTY_3','REPEATED_EMPTY_4','REPEATED_EMPTY_5','NEW_LOOP','POST_LOOP_EMPTY','POST_LOOP_EMPTY_2','POST_LOOP_EMPTY_3','POST_LOOP_EMPTY_4','POST_LOOP_EMPTY_5']:
                t=time.perf_counter()
                if phase=='NEW_FACTORS':result=isam.update(graph,values)
                elif phase=='NEW_LOOP':
                    new=gtsam.NonlinearFactorGraph();new.add(gtsam.BetweenFactorPose3(0,1,gtsam.Pose3(gtsam.Rot3(),np.array([1.2,0.,0.])),noise));result=isam.update(new,gtsam.Values())
                else:result=isam.update()
                elapsed=(time.perf_counter()-t)*1000;estimate=isam.calculateEstimate()
                assert estimate.size()==2 and isam.getFactorsUnsafe().size()==(3 if phase.startswith('POST_LOOP') or phase=='NEW_LOOP' else 2)
                records.append(dict(variant=v['name'],repetition=repetition,phase=phase,graph_nodes=2,graph_factors=isam.getFactorsUnsafe().size(),update_ms=elapsed,nonlinear_error=float(isam.getFactorsUnsafe().error(estimate)),finite=bool(np.isfinite(estimate.atPose3(1).matrix()).all()),**counters(result,2,'TINY_API',variant=v['name'],repetition=repetition,phase=phase)))
    dump('counter_api_reproduction.json',dict(gtsam=importlib.metadata.version('gtsam'),update_signature=gtsam.ISAM2.update.__doc__,result_getters={m:getattr(gtsam.ISAM2Result,m).__doc__ for m in ['getVariablesRelinearized','getVariablesReeliminated']},default_enableDetailedResults=bool(gtsam.ISAM2Params().enableDetailedResults),records=records,implementation_initialization_cause='UNVERIFIED: source unavailable; out-of-range returns empirically established, not a proven C++ root cause',in_range_is_not_proof_of_semantics=True))

def reference(template,odom,frames,common,loops,params,kf,K,label,saved=None):
    nodes=[(r,int(i)) for r,f in frames.items() for i in f.keyframe_id if r=='robot3' or i<=kf]
    indices=[0]+[odom[('robot1',i)] for i in range(151,kf+1)]+[odom[('robot3',i)] for i in range(235,4180)]+list(range(5795,5795+K))
    graph=b.graph_subset(template,indices);solve_ms=0.;iterations=None
    if saved:
        tr=pd.read_csv(saved);mat=d.s10.lookup(tr)
    else:
        initial=b.to_values({n:common[n] for n in nodes});t=time.perf_counter()
        optimizer=gtsam.LevenbergMarquardtOptimizer(graph,initial,params);values=optimizer.optimize()
        solve_ms=(time.perf_counter()-t)*1000;iterations=int(optimizer.iterations());mat=b.matrices_from(values,nodes)
        assert iterations<params.getMaxIterations(),'reference LM reached frozen cap'
        f={'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']}
        tr=d.trajectory(f,mat);tr.to_csv(OUT/f'batch_references/{label}.csv',index=False)
    assert set(mat)==set(nodes) and set(graph.keyVector())==set(b.to_values(mat).keys())
    f={'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']};b.schema(tr,f)
    error=float(graph.error(b.to_values(mat)))
    if saved:
        expected=float(BASE.loc[label,'batch_final_error']);assert abs(error-expected)<1e-7
    ROWS['references'].append(dict(label=label,kf=kf,K=K,nodes=len(nodes),factors=len(indices),batch_error=error,reused=bool(saved),reference_only=True,solve_ms=solve_ms,iterations=iterations,initialization='frozen common, never ISAM2',saved_sha256=d.sha256(saved) if saved else d.sha256(OUT/f'batch_references/{label}.csv')))
    return dict(graph=graph,mat=mat,error=error,indices=indices,kf=kf,K=K)

def trace(variant,cp,stage,step,isam,estimate,nodes,K,kf,ref,raw=None,update_ms=0.,extract_ms=0.,previous=None):
    t=time.perf_counter();mat=b.matrices_from(estimate,nodes)
    assert set(mat)==set(ref['mat']) and set(estimate.keys())==set(ref['graph'].keyVector())
    assert isam.getFactorsUnsafe().size()==ref['graph'].size()
    objective_start=time.perf_counter();error=float(ref['graph'].error(estimate));objective_ms=(time.perf_counter()-objective_start)*1000
    comparison_start=time.perf_counter();df,parts=c.metrics(mat,ref['mat']);combined=parts[0]
    nochange=False;max_t=None;max_r=None;delta_error=None
    if previous:
        old,old_error=previous;delta,_=c.metrics(mat,old);max_t=float(delta.translation_m.max());max_r=float(delta.rotation_deg.max());delta_error=error-old_error
        nochange=max_t<1e-7 and max_r<1e-7 and abs(delta_error)<1e-9
    passes=all(combined[k]<=v for k,v in LIMITS.items())
    comparison_ms=(time.perf_counter()-comparison_start)*1000
    row=dict(variant=variant,checkpoint=cp,stage=stage,extra_step=step,current_kf=kf,graph_nodes=len(nodes),odometry_factors=len(nodes)-2,loop_factors=K,prior_factors=1,graph_factors=ref['graph'].size(),nonlinear_error=error,batch_error=ref['error'],objective_gap=abs(error-ref['error']),PASS=passes,normal_update_ms=update_ms if step==0 else 0.,extra_update_ms=update_ms if step>0 else 0.,estimate_extraction_ms=extract_ms,objective_computation_ms=objective_ms,batch_comparison_ms=comparison_ms,maximum_pose_translation_change_m=max_t,maximum_pose_rotation_change_deg=max_r,objective_change=delta_error,empirical_no_change=nochange,**{k:v for k,v in combined.items() if k!='nodes'},**(raw or {}))
    ROWS['trace'].append(row)
    for part in parts:
        ROWS['localization'].append(dict(variant=variant,checkpoint=cp,stage=stage,extra_step=step,record_type='PER_SCOPE',**part))
    for robot in ['robot1','robot3']:
        sub=df[df.robot_id.eq(robot)]
        for rank,r in enumerate(sub.nlargest(20,'translation_m').itertuples(),1):
            ROWS['localization'].append(dict(variant=variant,checkpoint=cp,stage=stage,extra_step=step,record_type='WORST20_TRANSLATION',scope=robot,rank=rank,keyframe_id=int(r.keyframe_id),translation_m=r.translation_m,rotation_deg=r.rotation_deg))
    rr,_=d.loop_residuals(LOOPS.head(K),mat);br,_=d.loop_residuals(LOOPS.head(K),ref['mat'])
    for left,right in zip(rr.to_dict('records'),br.to_dict('records')):
        ROWS['residuals'].append(dict(variant=variant,checkpoint=cp,stage=stage,extra_step=step,arrival_index=left['arrival_index'],query_kf=left['query_keyframe_id'],candidate_kf=left['candidate_keyframe_id'],isam_translation_m=left['translation_residual_m'],batch_translation_m=right['translation_residual_m'],isam_rotation_deg=left['rotation_residual_deg'],batch_rotation_deg=right['rotation_residual_deg']))
    df.to_csv(OUT/f'pose_differences/{variant}_{cp}_{stage}_{step}.csv',index=False)
    f={'robot1':FRAMES['robot1'][FRAMES['robot1'].keyframe_id.le(kf)],'robot3':FRAMES['robot3']}
    tr=d.trajectory(f,mat);b.schema(tr,f);tr.to_csv(OUT/f'checkpoints/{variant}_{cp}_{stage}_{step}.csv',index=False)
    ROWS['integrity'].extend(dict(variant=variant,stage=stage,step=step,**x) for x in b.integrity(tr,LOCAL_TR,COMMON_TR,SELECTION,variant,cp))
    row['full_diagnostic_overhead_ms']=(time.perf_counter()-t)*1000
    return mat,error,row

def run(v,params,template,odom,loop_events,local,common,S):
    name=v['name'];progress('VARIANT_START',variant=name);nodes=[];registered=[];K=0;isam=None
    for index,r in enumerate(FRAMES['robot1'].itertuples()):
        kf=int(r.keyframe_id)
        if kf<283:continue
        cp=next((key for key,val in CP.items() if val==kf),None)
        if cp and kf>283:
            t=time.perf_counter();before=isam.calculateEstimate();ec=(time.perf_counter()-t)*1000
            trace(name,cp,'BEFORE_EVENT',-1,isam,before,nodes,K,kf-1,REFS[cp+'_BEFORE_EVENT'],extract_ms=ec)
        elif cp:
            ROWS['trace'].append(dict(variant=name,checkpoint=cp,stage='BEFORE_EVENT',extra_step=-1,current_kf=282,graph_nodes=0,available_independent_local_nodes=4079,odometry_factors=0,loop_factors=0,prior_factors=0,graph_factors=0,comparison_status='NOT_APPLICABLE_DISCONNECTED_LOCAL_FRAMES',nonlinear_error=None,PASS=None))
        t=time.perf_counter();new=gtsam.NonlinearFactorGraph();nv=gtsam.Values();arrived=loop_events.get(kf)
        if kf==283:
            nodes=[(robot,int(i)) for robot,f in FRAMES.items() for i in f.keyframe_id if robot=='robot3' or i<=kf]
            indices=REFS['FIRST_LOOP']['indices'];new=b.graph_subset(template,indices)
            nv=b.to_values({n:local[n] if n[0]=='robot1' else S@local[n] for n in nodes});isam=gtsam.ISAM2(params);K=1
        else:
            node=('robot1',kf);previous=isam.calculateEstimatePose3(b.numeric_key(('robot1',kf-1)))
            ni=odom[node];nv.insert(b.numeric_key(node),previous.compose(template.at(ni).measured()));new.add(template.at(ni));indices=[ni];nodes.append(node)
            if arrived is not None:
                for loop in arrived.itertuples():
                    assert int(loop.arrival_index)==K+1 and int(loop.query_keyframe_id)==kf
                    K+=1;indices.append(5795+K-1);new.add(template.at(indices[-1]))
        assert not set(registered).intersection(indices);registered.extend(indices)
        construct=(time.perf_counter()-t)*1000;t=time.perf_counter();result=isam.update(new,nv);um=(time.perf_counter()-t)*1000
        t=time.perf_counter();estimate=isam.calculateEstimate();ec=(time.perf_counter()-t)*1000
        raw=counters(result,len(nodes),'STAGE13D_NORMAL',variant=name,kf=kf)
        vt=time.perf_counter();mat=c.safe_matrices(estimate,nodes,common,kf,SELECTION);validation_ms=(time.perf_counter()-vt)*1000
        assert isam.getFactorsUnsafe().size()==len(registered)
        event_row=dict(variant=name,kf=kf,timestamp_ns=int(r.lidar_timestamp_ns),kind='FIRST_FUSION' if kf==283 else 'LOOP' if arrived is not None else 'ODOMETRY',nodes=len(nodes),factors=len(registered),K=K,new_factors=new.size(),new_values=nv.size(),normal_update_ms=um,extra_update_ms=0.,normal_extraction_ms=ec,extra_extraction_ms=0.,construction_ms=construct,validation_ms=validation_ms,**raw)
        if cp:
            mat,error,row=trace(name,cp,'AFTER_INSERTION',0,isam,estimate,nodes,K,kf,REFS[cp],raw,um,ec)
            if name=='D0':
                old=BASE.loc[cp]
                for field in LIMITS:
                    ROWS['reproduction'].append(dict(checkpoint=cp,field=field,new=row[field],historical=float(old[field]),PASS=abs(row[field]-float(old[field]))<1e-7))
                ROWS['reproduction'].append(dict(checkpoint=cp,field='objective',new=error,historical=float(old.incremental_final_error),PASS=abs(error-float(old.incremental_final_error))<1e-7))
            consecutive=0
            for step in range(1,v['extra']+1):
                t=time.perf_counter();er=isam.update();em=(time.perf_counter()-t)*1000
                t=time.perf_counter();estimate=isam.calculateEstimate();ee=(time.perf_counter()-t)*1000
                assert isam.getFactorsUnsafe().size()==len(registered) and estimate.size()==len(nodes),'unexpected extra-update graph change'
                c.safe_matrices(estimate,nodes,common,kf,SELECTION)
                raw=counters(er,len(nodes),'STAGE13D_EMPTY',variant=name,kf=kf,extra_step=step)
                mat,error,row=trace(name,cp,'EXTRA',step,isam,estimate,nodes,K,kf,REFS[cp],raw,em,ee,(mat,error))
                consecutive=consecutive+1 if row['empirical_no_change'] else 0
                row['consecutive_no_change_calls']=consecutive;row['numerical_no_change_state']=consecutive>=2
                event_row['extra_update_ms']+=em;event_row['extra_extraction_ms']+=ee
            progress('CHECKPOINT',variant=name,checkpoint=cp,extra_step=row['extra_step'],translation_p95_m=row['translation_p95_m'],PASS=row['PASS'])
        if index+1<len(FRAMES['robot1']):event_row['next_interval_ms']=(int(FRAMES['robot1'].iloc[index+1].lidar_timestamp_ns)-int(r.lidar_timestamp_ns))/1e6
        else:event_row['next_interval_ms']=None
        event_row['backend_processing_ms']=construct+um+ec+event_row['extra_update_ms']+event_row['extra_extraction_ms']
        ROWS['timing'].append(event_row)
        if kf%400==0:progress('STREAM',variant=name,kf=kf)
    actual=isam.getFactorsUnsafe();same=all(actual.at(i).equals(template.at(j),1e-12) for i,j in enumerate(registered))
    assert same and set(registered)==set(range(5870)) and set(estimate.keys())==set(template.keyVector()) and len(nodes)==5796 and K==75
    EQUIVALENCE[name]=dict(PASS=True,nodes=5796,odometry=5794,loops=75,prior=1,factors=5870,measurements_and_noise_unchanged=same,all_keys_equal=True,no_future_robot1=True,no_duplicate_factors=True)
    progress('VARIANT_COMPLETE',variant=name)
    persist()

def persist():
    for key,file in [('counters','counter_validity_audit.csv'),('trace','checkpoint_microtrace.csv'),('timing','diagnostic_update_timing.csv'),('localization','solver_discrepancy_localization.csv'),('residuals','solver_loop_residual_comparison.csv'),('integrity','trajectory_integrity_audit.csv'),('references','batch_reference_provenance.csv'),('reproduction','D0_reproduction_audit.csv')]:csv(file,ROWS[key])
    dump('final_factor_equivalence.json',EQUIVALENCE)

def main():
    global LOOPS,BASE,FRAMES,LOCAL_TR,COMMON_TR,SELECTION,LIMITS,HISTORY,loops_for_history
    if OUT.exists() and any(OUT.iterdir()):raise RuntimeError('Refuse overwrite; report-only allowed after computation')
    OUT.mkdir(parents=True)
    for folder in ['checkpoints','pose_differences','batch_references']:(OUT/folder).mkdir()
    subprocess.run(['git','merge-base','--is-ancestor',ACCEPTED,'HEAD'],cwd=ROOT,check=True)
    tracked=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
    HISTORY={str(ROOT/p):d.sha256(ROOT/p) for p in tracked if p.startswith(('src/cumulti/','outputs/cumulti_v1/','docs/','demo/')) and '13d_' not in p and 'STAGE13D' not in p and (ROOT/p).is_file()}
    historical_manifest=json.loads((C/'stage13c_input_manifest.json').read_text())
    for entry in historical_manifest['inputs']:assert d.sha256(entry['path'])==entry['sha256']
    paths=[ROOT/'src/cumulti/run_stage13b_incremental_isam2.py',ROOT/'src/cumulti/run_stage13c_isam2_convergence.py']+[C/n for n in ['extra_update_effects.csv','extra_update_diagnostic.csv','numerical_agreement_by_variant.csv','per_event_solver_diagnostics.csv']]+[B/f'snapshots/{cp}_batch_trajectory.csv' for cp in CP]
    inputs=[]
    for path in paths:
        blob=subprocess.check_output(['git','show',f'{ACCEPTED}:{path.relative_to(ROOT).as_posix()}'],cwd=ROOT)
        h=d.sha256(path);assert h==hashlib.sha256(blob).hexdigest();inputs.append(dict(path=str(path),sha256=h,accepted_blob_verified=True,size=path.stat().st_size))
    dump('stage13d_input_manifest.json',dict(accepted_checkpoint=ACCEPTED,inputs=inputs,inherited_frozen_inputs=historical_manifest['inputs'],historical_hashes_before=HISTORY,runner_sha256=d.sha256(Path(__file__)),GT_used=False))
    api=dict(gtsam=importlib.metadata.version('gtsam'),python=sys.version,executable=sys.executable,update_signature=gtsam.ISAM2.update.__doc__,ISAM2Result_API=[x for x in dir(gtsam.ISAM2Result) if not x.startswith('_')])
    assert api['gtsam']=='4.2' and 'update(self: gtsam.gtsam.ISAM2)' in api['update_signature']
    dump('stage13d_api_audit.json',api)
    LIMITS={k:json.loads((B/'incremental_batch_agreement_policy.json').read_text())[k] for k in ['translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg']}
    supported=[]
    for v in VARIANTS:
        params,text=c.configure(v);supported.append((v,params));v['verified_parameter_print']=text
    policy=dict(UTC=b.a.now(),variants=VARIANTS,checkpoints=CP,limits=LIMITS,extra_schedule='exactly5 at all six selected checkpoints including FIRST_LOOP and FINAL; no adaptive calls elsewhere; abort affected diagnostic on exception/nonfinite/graph mutation',no_change=dict(max_translation_m_lt=1e-7,max_rotation_deg_lt=1e-7,abs_objective_change_lt=1e-9,consecutive_calls=2,continue_to_fixed5_for_comparability=True,not_mathematical_convergence=True),counter_rule='retain raw; integer 0..nodes only in-range; invalid=NULL plus INVALID_COUNTER, never clip/zero; in-range does not prove semantics; known invalid counters alone do not imply invalid poses/graph',before_event='old joint graph immediately before grouped odometry+loop insertion; kf-1, only preceding loops; FIRST_LOOP independent frames N/A; FINAL contains no new loop',batch='after states reuse frozen B; pre-event states require5 new evaluation-only LM solves with frozen12D defaults/common x0, no ISAM warmstart',frame='R1 KF150 same gauge, no alignment',timing='ordinary insertion/extraction/construction; extra calls/extraction separate; objective/comparison/serialization/integrity/batch LM excluded backend; all overhead recorded',objective_plateau_abs_gap_le=1e-3,decision='all six post-final-extra states plus integrity => DIAGNOSTIC_CONVERGENCE_EXPLAINED; any material paired pose/objective improvement => PARTIALLY_EXPLAINED; otherwise UNRESOLVED; unavailable empty API/unsafe calls => COUNTER_API_BLOCKER; never production readiness',hessian='NOT_TESTED; no dense/sparse conditioning inference',GT_used=False)
    dump('stage13d_experiment_policy.json',policy);progress('POLICY_FROZEN')
    LOOPS=pd.read_csv(b.D12/'ordered_rank1_clean_loops.csv');assert len(LOOPS)==75 and LOOPS.arrival_index.to_list()==list(range(1,76))
    loops_for_history=dict(zip(LOOPS.arrival_index.astype(int),LOOPS.query_keyframe_id.astype(int)))
    historical_counters();tiny_api_test();progress('COUNTER_API_AUDITED')
    LOCAL_TR=pd.read_csv(b.D12/'clean_local_trajectory.csv');COMMON_TR=pd.read_csv(b.D12/'clean_first_loop_trajectory.csv')
    local=d.s10.lookup(LOCAL_TR);common=d.s10.lookup(COMMON_TR)
    FRAMES={r:pd.read_csv(d.s10.PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns']).iloc[start:].copy() for r,start in d.STARTS.items()}
    SELECTION=json.loads((d.C12/'stable_segment_selection.json').read_text());BASE=pd.read_csv(B/'incremental_vs_batch_checkpoints.csv').set_index('checkpoint')
    assert BASE.robot1_keyframe.to_dict()==c.CP
    S=local[('robot1',283)]@d.tm(LOOPS.iloc[0])@d.s10.inv(local[('robot3',252)])
    assert max(np.max(abs(common[n]-(local[n] if n[0]=='robot1' else S@local[n]))) for n in local)<1e-9
    d.OUT=OUT;template,lmparams=d.build_graph(FRAMES,local,LOOPS)
    assert json.loads((OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==json.loads((b.D12/'clean_gtsam_solver_policy.json').read_text())['parameters']
    odom={('robot1',i):i-150 for i in range(151,2000)};odom.update({('robot3',i):1850+i-235 for i in range(235,4180)})
    for cp,kf in CP.items():
        K=int(BASE.loc[cp,'K']);REFS[cp]=reference(template,odom,FRAMES,common,LOOPS,lmparams,kf,K,cp,B/f'snapshots/{cp}_batch_trajectory.csv')
        if kf>283:REFS[cp+'_BEFORE_EVENT']=reference(template,odom,FRAMES,common,LOOPS,lmparams,kf-1,int(LOOPS.query_keyframe_id.lt(kf).sum()),cp+'_BEFORE_EVENT')
    loop_events={int(q):sub for q,sub in LOOPS.groupby('query_keyframe_id',sort=False)}
    for v,params in supported:
        try:run(v,params,template,odom,loop_events,local,common,S)
        except Exception as ex:
            EQUIVALENCE[v['name']]=dict(PASS=False,status='DIAGNOSTIC_STOPPED',reason=f'{type(ex).__name__}: {ex}')
            progress('VARIANT_STOPPED',variant=v['name'],reason=str(ex));persist()
    finish(policy)

def finish(policy):
    persist();n=pd.DataFrame(ROWS['trace']);summary=[]
    for (variant,cp),sub in n[n.stage.ne('BEFORE_EVENT')].groupby(['variant','checkpoint'],sort=False):
        first=sub.iloc[0];last=sub.iloc[-1];qualifying=sub[sub.PASS.eq(True)]
        invalid=any(sub.get(k,pd.Series(dtype=str)).eq('INVALID_COUNTER').any() for k in ['relinearized_status','reeliminated_status'])
        if bool(last.PASS):status='AGREEMENT_REACHED'
        elif float(last.objective_gap)<=policy['objective_plateau_abs_gap_le']:status='OBJECTIVE_CONVERGED_POSE_GATE_FAILED'
        elif len(sub)>2 and bool(last.get('numerical_no_change_state',False)):status='NO_MATERIAL_CHANGE'
        else:status='STILL_DIVERGING'
        summary.append(dict(variant=variant,checkpoint=cp,normal_translation_p95_m=float(first.translation_p95_m),final_translation_p95_m=float(last.translation_p95_m),normal_PASS=bool(first.PASS),final_PASS=bool(last.PASS),extra_updates_attempted=int(last.extra_step),first_agreement_step=int(qualifying.extra_step.min()) if len(qualifying) else None,initial_objective=float(first.nonlinear_error),final_objective=float(last.nonlinear_error),objective_gap=float(last.objective_gap),classification=status,counter_diagnostic='INVALID_COUNTER_DIAGNOSTIC' if invalid else 'IN_RANGE_SEMANTICS_UNVERIFIED'))
    csv('numerical_convergence_summary.csv',summary)
    counter=pd.DataFrame(ROWS['counters']);invalid=counter[counter.diagnostic_status.eq('INVALID_COUNTER')]
    counts=counter.groupby(['context','method','diagnostic_status']).size().reset_index(name='count').to_dict('records')
    dump('counter_validity_summary.json',dict(invalid_count=len(invalid),invalid_raw_examples=invalid.raw_value.drop_duplicates().head(12).to_list(),groups=counts,empty_reelimination_reliable=False if len(invalid[invalid.method.eq('getVariablesReeliminated')]) else 'UNVERIFIED',corrects_stage13c_interpretation='225 positive reelimination getter returns did NOT establish 225 valid re-elimination events; exclude invalid values. Historical numerical poses/metrics unchanged.',source_initialization_cause='UNVERIFIED',in_range_counts_not_independent_proof=True))
    structural=len(EQUIVALENCE)==3 and all(x['PASS'] for x in EQUIVALENCE.values())
    qualifiers=[v['name'] for v in VARIANTS if len([x for x in summary if x['variant']==v['name']])==6 and all(x['final_PASS'] for x in summary if x['variant']==v['name'])]
    improved=any(x['final_translation_p95_m']<x['normal_translation_p95_m']-1e-7 or x['final_objective']<x['initial_objective']-1e-9 for x in summary)
    decision='COUNTER_API_BLOCKER' if not structural else 'DIAGNOSTIC_CONVERGENCE_EXPLAINED' if qualifiers else 'NUMERICAL_DISCREPANCY_PARTIALLY_EXPLAINED' if improved else 'NUMERICAL_DISCREPANCY_UNRESOLVED'
    record=dict(UTC=b.a.now(),decision=decision,qualifying_diagnostic_schedules=qualifiers,all_selected_checkpoints_pass=bool(qualifiers),requires_extra_updates=any(v!='D0' for v in qualifiers),production_ready=False,GT_used=False,original_B_C_status_unchanged=True,hessian='NOT_TESTED',selection_policy=policy['decision'])
    if (OUT/'stage13d_numerical_decision.json').exists():raise RuntimeError('Frozen decision already exists')
    dump('stage13d_numerical_decision.json',record)
    immutable=all(d.sha256(p)==h for p,h in HISTORY.items());assert immutable
    repro=len(ROWS['reproduction'])==30 and all(x['PASS'] for x in ROWS['reproduction'])
    checks={'HISTORICAL_INPUTS_IMMUTABLE':'PASS' if immutable else 'FAIL','COUNTER_API_AUDITED':'PASS','INVALID_COUNTERS_IDENTIFIED':'YES' if len(invalid) else 'NO','FROZEN_FACTOR_GRAPH_VALID':'PASS' if structural else 'FAIL','ALL_VARIANTS_EXECUTED':'PASS' if structural else 'FAIL','FIRST_LOOP_REPRODUCED':'PASS' if repro else 'FAIL','K2_MICROTRACE_COMPLETED':'PASS' if len(n[n.checkpoint.eq('K2')&n.stage.ne('BEFORE_EVENT')])==13 else 'FAIL','K10_MICROTRACE_COMPLETED':'PASS' if len(n[n.checkpoint.eq('K10')&n.stage.ne('BEFORE_EVENT')])==13 else 'FAIL','FINAL_MICROTRACE_COMPLETED':'PASS' if len(n[n.checkpoint.eq('FINAL_STREAM_END')&n.stage.ne('BEFORE_EVENT')])==13 else 'FAIL','EXTRA_UPDATES_ADD_NO_FACTORS':'PASS' if structural else 'FAIL','NUMERICAL_GATES_UNCHANGED':'PASS','GT_USED':'NO','BATCH_REFERENCE_EQUAL_GRAPH':'PASS','NEW_CATASTROPHIC_JUMPS':'NONE' if structural else 'SEE_STOP_RECORD','SOLVER_TIMING_MEASURED':'PASS','NUMERICAL_AGREEMENT_RESOLVED':'YES' if qualifiers else 'PARTIAL' if improved else 'NO','HISTORICAL_DEMO_MODIFIED':'NO'}
    dump('summary.json',dict(validation=checks,decision=decision,D0_reproduced=repro,convergence=summary,new_batch_reference_solves=5,evaluation_only=True,GT_used=False,remaining_uncertainty='Counter initialization source unverified; no reliable Hessian/conditioning measurement. No claim of sole cause or production readiness.'))
    (OUT/'VALIDATION_REPORT.txt').write_text('\n'.join(f'{k}: {v}' for k,v in checks.items())+'\n')
    dump('historical_immutability_audit.json',dict(PASS=immutable,files_checked=len(HISTORY)))
    report();progress('COMPLETE',decision=decision)

def report():
    # Pure saved-data reporting. No optimizer, initialization, or decision writes.
    trace=pd.read_csv(OUT/'checkpoint_microtrace.csv');counts=pd.read_csv(OUT/'counter_validity_audit.csv',dtype=str,keep_default_na=False)
    summary=pd.read_csv(OUT/'numerical_convergence_summary.csv');timing=pd.read_csv(OUT/'diagnostic_update_timing.csv')
    decision=json.loads((OUT/'stage13d_numerical_decision.json').read_text());counter=json.loads((OUT/'counter_validity_summary.json').read_text())
    evidence=json.loads((ROOT/'docs/STAGE13D_GTSAM42_COUNTER_SOURCE_EVIDENCE.json').read_text())
    dump('counter_source_audit.json',evidence)
    original_source=json.loads((OUT/'stage13d_input_manifest.json').read_text())['runner_sha256']
    dump('runner_finalization_provenance.json',dict(numerical_run_source_sha256=original_source,finalized_runner_sha256=d.sha256(Path(__file__)),changes_after_numerical_run='report-only additions: official-source interpretation, derived schedule/cost tables, figure formatting and provenance. No numerical run/reference/update/gate/decision functions changed; no solver rerun.',numerical_decision_sha256=d.sha256(OUT/'stage13d_numerical_decision.json'),evidence_asset_sha256=d.sha256(ROOT/'docs/STAGE13D_GTSAM42_COUNTER_SOURCE_EVIDENCE.json')))
    schedule=[]
    for row in trace[trace.extra_step.ge(0)].itertuples():
        v=next(x for x in VARIANTS if x['name']==row.variant)
        ordinal=int(CP[row.checkpoint])-282+v['extra']*sum(k<CP[row.checkpoint] for k in CP.values())+int(row.extra_step)
        schedule.append(dict(variant=row.variant,checkpoint=row.checkpoint,extra_step=int(row.extra_step),observed_call_ordinal=ordinal,tag_expected_relinearization_check=ordinal%v['skip']==0,returned_marked_count=row.relinearized_valid_count,translation_p95_m=row.translation_p95_m,nonlinear_error=row.nonlinear_error,interpretation='official4.2 modulo check, observed1-based call sequence; exact binary branch not instrumented'))
    csv('update_schedule_analysis.csv',schedule)
    def save(fig,name):fig.savefig(OUT/name,dpi=160);plt.close(fig)
    q=counts.groupby(['method','diagnostic_status']).size().unstack(fill_value=0)
    fig,ax=plt.subplots(figsize=(10,4),constrained_layout=True);q.plot.bar(ax=ax);ax.set(title='Counter physical validity; in-range semantics remain unverified',ylabel='Recorded returns');ax.tick_params(axis='x',rotation=0);save(fig,'01_counter_validity.png')
    for file,cp,metric in [('02_K2_objective_vs_extra_updates.png','K2','nonlinear_error'),('03_K2_pose_error_vs_extra_updates.png','K2','translation_p95_m'),('04_K10_pose_error_vs_extra_updates.png','K10','translation_p95_m'),('05_final_pose_error_vs_extra_updates.png','FINAL_STREAM_END','translation_p95_m')]:
        fig,ax=plt.subplots(figsize=(8,4),constrained_layout=True)
        for v in VARIANTS:
            s=trace[trace.variant.eq(v['name'])&trace.checkpoint.eq(cp)&trace.extra_step.ge(0)];ax.plot(s.extra_step,s[metric],'-o',label=v['name'])
        if metric=='translation_p95_m':ax.axhline(.5,c='black',ls='--',label='unchanged p95 gate (all4 needed)')
        ax.set(xlabel='Extra calls after normal insertion (0)',ylabel=metric,title=cp+' numerical-only microtrace');ax.legend();save(fig,file)
    fig,axs=plt.subplots(1,2,figsize=(13,4),constrained_layout=True)
    for ax,field in zip(axs,['relinearized','reeliminated']):
        for v in VARIANTS:
            s=trace[trace.variant.eq(v['name'])&trace.checkpoint.eq('K2')&trace.extra_step.ge(0)].copy()
            s=s[s[field+'_status'].eq('IN_RANGE_UNVERIFIED_SEMANTICS')];ax.plot(s.extra_step,s[field+'_valid_count'],'-o',label=v['name'])
        ax.set(xlabel='K2 extra call',ylabel='In-range returned count',title=field+' (invalid excluded; semantics unverified)');ax.legend()
    save(fig,'06_relinearization_activity.png')
    fig,axs=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
    for ax,v in zip(axs,VARIANTS):
        for step in sorted(set([0,v['extra']])):
            path=OUT/f'pose_differences/{v["name"]}_K2_{"AFTER_INSERTION" if step==0 else "EXTRA"}_{step}.csv'
            z=pd.read_csv(path);z=z[z.robot_id.eq('robot3')];ax.plot(z.keyframe_id,z.translation_m,lw=.6,label=f'extra{step}')
        ax.set(title=v['name']+' stored Robot3',xlabel='Original KF',ylabel='Same-gauge translation difference (m)');ax.legend()
    save(fig,'07_robot3_K2_error_distribution.png')
    fig,ax=plt.subplots(figsize=(8,4),constrained_layout=True)
    for v in VARIANTS:
        t=trace[trace.variant.eq(v['name'])&trace.checkpoint.eq('K2')&trace.extra_step.ge(0)].sort_values('extra_step');lat=np.cumsum(t.normal_update_ms.fillna(0)+t.extra_update_ms.fillna(0))
        ax.plot(lat,t.translation_p95_m,'-o',label=v['name'])
    ax.axhline(.5,c='black',ls='--');ax.set(xlabel='K2 cumulative update time (ms), excludes diagnostics',ylabel='Translation p95 (m)',title='Diagnostic agreement/update cost, not deployable policy');ax.legend();save(fig,'08_numerical_agreement_vs_runtime.png')
    def table(df):
        return '| '+' | '.join(df.columns)+' |\n| '+' | '.join(['---']*len(df.columns))+' |\n'+'\n'.join('| '+' | '.join(str(x) for x in row)+' |' for row in df.itertuples(index=False,name=None))
    runtime=[]
    for (variant,kind),z in timing.groupby(['variant','kind'],sort=False):
        runtime.append(dict(variant=variant,kind=kind,count=len(z),normal_update_median_ms=z.normal_update_ms.median(),normal_update_p95_ms=z.normal_update_ms.quantile(.95),extra_update_total_ms=z.extra_update_ms.sum(),extraction_total_ms=(z.normal_extraction_ms+z.extra_extraction_ms).sum(),backend_next_interval_misses=int((z.backend_processing_ms>=z.next_interval_ms).sum())))
    csv('runtime_summary.csv',runtime)
    timeline=json.loads((OUT/'execution_events.json').read_text());wall=[]
    for v in VARIANTS:
        name=v['name'];start=next(x['UTC'] for x in timeline if x['phase']=='VARIANT_START' and x.get('variant')==name)
        end=next(x['UTC'] for x in timeline if x['phase']=='VARIANT_COMPLETE' and x.get('variant')==name)
        elapsed=(datetime.fromisoformat(end)-datetime.fromisoformat(start)).total_seconds()*1000
        z=timing[timing.variant.eq(name)];backend=float(z.backend_processing_ms.sum())
        wall.append(dict(variant=name,replay_elapsed_UTC_ms=elapsed,backend_processing_total_ms=backend,non_backend_diagnostic_logging_overhead_ms=elapsed-backend,extra_update_total_ms=float(z.extra_update_ms.sum()),clock_limitation='derived from recorded UTC start/end, not primary monotonic latency benchmark; excludes startup/reference LM'))
    csv('replay_overhead_summary.csv',wall)
    trace_cost=trace[trace.stage.ne('BEFORE_EVENT')].groupby('variant')[['normal_update_ms','extra_update_ms','estimate_extraction_ms','objective_computation_ms','batch_comparison_ms','full_diagnostic_overhead_ms']].sum().reset_index();csv('checkpoint_cost_breakdown.csv',trace_cost)
    text=f'''# Stage13D — targeted ISAM2 convergence and diagnostic validity audit

Accepted checkpoint `{ACCEPTED}`. Independent runner, historical B/C/demo unchanged.
Decision: **{decision['decision']}**. Qualifying diagnostic schedules: {decision['qualifying_diagnostic_schedules']}. Not a production policy.

## Frozen design and graph provenance

Six checkpoints FIRST_LOOP/K2/K10/K40/K75/FINAL, three fresh persistent instances. D0 default skip10/threshold0.1, D1 same +exactly5 empty calls at these checkpoints, D2 skip1/threshold0.01 +same5 calls. Wildfire0.001, noise, Huber, measurements and KF150 anchor unchanged. Original four gates unchanged. No GT, scans, maps, online frontend or simultaneous robots. Robot3 stored; Robot1 alone streams. Extra calls change schedule and cannot be called one-update-per-event timing. Empirical no-change <1e-7m/<1e-7deg/<1e-9 objective on2 consecutive calls, still record all5; not proof of mathematical convergence.

## Counter validity correction

Invalid records: {counter['invalid_count']} (historical derived tables duplicate some returns; this is not a unique-call count). Examples: {counter['invalid_raw_examples']}. Raw values preserved as exact strings; out-of-range gets INVALID_COUNTER and null diagnostic value, never clipping or zeroing. Physical range0..nodes does not prove API semantics. Tiny independent Pose3 test covers insertions/empty/repeated empty calls, default detailed-results flag recorded. Its initial source-unknown annotation records knowledge at test time; subsequent verified official-tag findings are in counter_source_audit.json.

Official GTSAM4.2 [ISAM2Result.h](https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/nonlinear/ISAM2Result.h) shows that its constructor does not initialize the scalar counts. [ISAM2.cpp](https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/nonlinear/ISAM2.cpp) explicitly zeros the relinearization count but assigns the re-elimination count only inside conditional recalculation branches. Empty no-work paths can therefore expose uninitialized re-elimination memory. Tiny-graph and real-call out-of-range returns are consistent with this source mechanism. Exact wheel build identity and individual C++ branch execution were not instrumented. Even an in-range no-work return may be coincidental garbage. The relinearization getter is marked-set bookkeeping, not proof that every counted pose changed coordinates.

The Stage13C claim of225 positive re-elimination returns does **not** prove225 valid re-elimination events. This new interpretation does not modify historical CSVs or their numerical pose metrics.

## Microtrace/reference conventions

BEFORE_EVENT uses the preceding actual graph (KF-1, before grouped odometry/loop insertion), AFTER_INSERTION uses current KF. They are different graphs; each is compared only to its OWN identical-graph batch reference. FIRST_LOOP before-state has disconnected local frames, no common-frame objective/pose comparison. FINAL has no new loop. Six after-references reused from13B after accepted-blob/keys/objective checks. Five missing pre-event references solved ONCE for evaluation with frozen12D LM settings and common x0, never warm-started from ISAM2 or fed to ISAM2. No additional rigid alignment.

## Numerical results

{table(summary)}

Full step0..5 graph objective, same-gauge median/p95 translation/rotation, counters and cost are in checkpoint_microtrace.csv. Objective plateau classification is descriptive absolute batch gap<=1e-3, not a relaxed pose gate or mathematical convergence test. Near-equal objective is not numerical pose agreement.

K2 complete after-insertion/extra trace:

{table(trace[trace.checkpoint.eq('K2')&trace.extra_step.ge(0)][['variant','extra_step','translation_median_m','translation_p95_m','nonlinear_error','objective_gap','PASS']])}

## Measured schedule/convergence interpretation

Official [ISAM2-impl.h](https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/nonlinear/ISAM2-impl.h) uses a modulo skip check; each empty call advances the counter. D1's K2 normal call is ordinal101 after its five FIRST_LOOP calls, and extras102–106 reach no skip10 boundary. All five return zero relinearization bookkeeping and leave objective/poses unchanged. Thus five empty calls do not automatically mean five nonlinear relinearization iterations. This explains the apparent contrast with Stage13C V4's different all-loop schedule, without modifying it. See update_schedule_analysis.csv.

D2 checks every call. At K2, extra1 sharply reduces objective and pose disagreement; additional calls improve it enough to meet all four original gates, but later steps are not strictly monotonic. At K10 the normal D2 state already passes; K40 extra updates improve agreement. K75 and FINAL remain above the original pose gates even when objectives nearly match batch. Objective convergence and pose agreement do not coincide generally; no condition passes all six selected checkpoints.

## Localization and residuals

Per-scope errors/worst20 each robot and full nodewise differences retained for every state. Loop physical metric E=Z^-1 Xi^-1 Xj, translation norm and rotation angle; only already inserted loops. No future-loop residual evaluation. K2 Robot3 chain distribution is plotted. Hessian conditioning: NOT_TESTED; no dense6N matrix or nullspace claim. Long-chain deformation/path dependence are hypotheses unless separately measured.

K2 per-robot disagreement before/after diagnostics:

{table(pd.read_csv(OUT/'solver_discrepancy_localization.csv').query("checkpoint == 'K2' and record_type == 'PER_SCOPE' and scope != 'combined' and stage != 'BEFORE_EVENT'")[['variant','extra_step','scope','translation_median_m','translation_p95_m','rotation_p95_deg']])}

## Timing

{table(pd.DataFrame(runtime))}

Ordinary and extra update/extraction/construction times distinct. Objective computation, batch comparison, full diagnostics, validation and reference-LM costs recorded separately; never included as ordinary update. Backend next-interval comparison uses actual timestamps and includes extra-call processing, excludes offline diagnostics and whole-system frontend/network/map work. Single replay/first fusion one sample. See diagnostic_update_timing.csv, checkpoint_cost_breakdown.csv and batch_reference_provenance.csv.

{table(pd.DataFrame(wall))}

Replay elapsed/aggregate residual diagnostic overhead above is derived from captured UTC event timestamps, not substituted for monotonic per-call timings. It includes integrity checks, counter collection, microtrace diagnostics, logging and serialization; startup/template and the five reference LM solves are separate. No end-to-end real-time claim.

## Reproduction and limitations

Run `bash scripts/run_stage13d_solver_diagnostics.sh` from root with a new empty stage directory. Runner refuses overwrite. `--report-only` regenerates figures/doc from saved data, without optimizer or decision rewrite. Frozen D0 comparisons, exact final factor identity, no future R1 nodes, unit-quaternion schema, finite poses and frozen catastrophic-step tests are recorded. Current summary: {json.loads((OUT/'summary.json').read_text())['remaining_uncertainty']}

## Next-stage recommendation

Stop after13D. If a diagnostic schedule reaches all selected gates, validate it under a separately predeclared all-checkpoint/held-out protocol before any deployment. Otherwise further GT-free stopping/linearization/weak-direction study is needed; do not loosen gates, choose loops, change demo or tune from GT. Historical13B/C remain negative results regardless of this diagnostic.
'''
    (ROOT/'docs/CUMULTI_STAGE13D_SOLVER_DIAGNOSTICS.md').write_text(text)

if __name__=='__main__':
    if sys.argv[1:]==['--report-only']:report()
    elif sys.argv[1:]:raise SystemExit('Usage: run_stage13d_solver_diagnostics.py [--report-only]')
    else:main()
