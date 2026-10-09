#!/usr/bin/env python3
"""Independent numerical-only fixed ISAM2 ablations. Never calls Stage13B main.

Reuse pure Stage13B/12D helpers and actual immutable factors. No GT, LM solve,
scan loading, map generation, or hidden convergence trials. Batch references
are verified saved Stage13B estimates, evaluation-only.
"""
from __future__ import annotations
import hashlib
import importlib.metadata
import json
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_stage13b_incremental_isam2 as b
d=b.d
ROOT=b.ROOT
B13=b.OUT
OUT=ROOT/'outputs/cumulti_v1/13c_isam2_convergence'
ACCEPTED='b20f214e3eac8436f32bb3f9f6250c75b30c04bb'
VARIANTS=[dict(name='V0_BASELINE_REPLICATION',skip=10,threshold=.1,extra=0),dict(name='V1_EVERY_EVENT_RELINEARIZATION_CHECK',skip=1,threshold=.1,extra=0),dict(name='V2_LOWER_RELINEARIZATION_THRESHOLD',skip=1,threshold=.01,extra=0),dict(name='V3_STRICT_RELINEARIZATION_THRESHOLD',skip=1,threshold=.001,extra=0),dict(name='V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES',skip=10,threshold=.1,extra=3)]
CP={'FIRST_LOOP':283,'K2':378,'K5':512,'K10':638,'K20':664,'K40':1014,'K75':1967,'FINAL_STREAM_END':1999}
EVENTS=[]

def dump(name,data):
    p=OUT/name;p.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    with p.open('rb') as f:os.fsync(f.fileno())

def csv(name,data):
    (data if isinstance(data,pd.DataFrame) else pd.DataFrame(data)).to_csv(OUT/name,index=False)

def event(phase,**kw):
    x=dict(phase=phase,UTC=b.a.now(),**kw);EVENTS.append(x)
    print(json.dumps(x),flush=True);dump('progress.json',x);dump('execution_events.json',EVENTS)

def configure(v):
    p=gtsam.ISAM2Params();p.relinearizeSkip=v['skip'];p.setRelinearizeThreshold(v['threshold'])
    gn=gtsam.ISAM2GaussNewtonParams();gn.setWildfireThreshold(.001);p.setOptimizationParams(gn)
    text=str(p)
    def printed(field):
        m=re.search(r'^'+field+r':\s+(\S+)',text,re.M);assert m,field
        return m.group(1)
    assert int(printed('relinearizeSkip'))==v['skip']
    assert float(printed('relinearizeThreshold'))==v['threshold']
    assert gn.getWildfireThreshold()==.001 and p.getFactorization()=='CHOLESKY' and p.cacheLinearizedFactors
    return p,text

def counter(result,name):
    return int(getattr(result,name)()) if hasattr(result,name) else 'NOT_EXPOSED'

def metrics(matrices,reference):
    rows=[]
    for (robot,i),x in matrices.items():
        y=reference[(robot,i)]
        rows.append(dict(robot_id=robot,keyframe_id=i,translation_m=float(np.linalg.norm(x[:3,3]-y[:3,3])),rotation_deg=float(np.rad2deg(Rotation.from_matrix(x[:3,:3].T@y[:3,:3]).magnitude()))))
    df=pd.DataFrame(rows);summary=[]
    for scope,sub in [('combined',df),('robot1',df[df.robot_id.eq('robot1')]),('robot3',df[df.robot_id.eq('robot3')])]:
        row=dict(scope=scope,nodes=len(sub))
        for metric,unit in [('translation','m'),('rotation','deg')]:
            vals=sub[metric+'_'+unit].to_numpy()
            for s,q in [('median',50),('p90',90),('p95',95),('max',100)]:row[f'{metric}_{s}_{unit}']=float(np.percentile(vals,q))
        summary.append(row)
    return df,summary

def safe_matrices(values,nodes,common,kf,selection):
    m=b.matrices_from(values,nodes)
    assert values.size()==len(nodes) and all(np.isfinite(x).all() for x in m.values()),'nonfinite/node mismatch'
    for robot in d.STARTS:
        ids=range(d.STARTS[robot],kf+1 if robot=='robot1' else 4180)
        xyz=np.array([m[(robot,i)][:3,3] for i in ids]);pre=np.array([common[(robot,i)][:3,3] for i in ids])
        step=np.linalg.norm(np.diff(xyz,axis=0),axis=1);old=np.linalg.norm(np.diff(pre,axis=0),axis=1)
        limit=selection['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold']
        assert not (((step>5*limit)&(old<=5*limit))|((step>=100)&(old<100))).any(),'new catastrophic jump'
    return m

def main():
    if OUT.exists() and any(OUT.iterdir()):raise RuntimeError('Refuse overwrite or undeclared rerun')
    OUT.mkdir(parents=True,exist_ok=True)
    for name in ['checkpoints','trajectories','extra_estimates']:(OUT/name).mkdir()
    subprocess.run(['git','merge-base','--is-ancestor',ACCEPTED,'HEAD'],cwd=ROOT,check=True)
    d.OUT=OUT  # Redirect ONLY Stage12D helper's policy output, never Stage13B.
    # No GT metric table is decoded. Hashes protect historical bytes only.
    tracked=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
    protected={str(ROOT/p):d.sha256(ROOT/p) for p in tracked if p.startswith(('src/cumulti/','outputs/cumulti_v1/','demo/final_replay/')) and not p.startswith('outputs/cumulti_v1/13c_isam2_convergence/') and (ROOT/p).is_file()}
    paths=[ROOT/'src/cumulti'/n for n in ['run_stage13b_incremental_isam2.py','audit_stage13b_incremental_isam2.py','run_stage12d_clean_backend.py','run_stage13a_sparse_loops.py','run_stage103_gtsam_solver_audit.py','run_stage10_offline_map_merge.py','run_stage12a_trajectory_integrity.py']]
    paths += [b.D12/n for n in ['clean_local_trajectory.csv','clean_first_loop_trajectory.csv','ordered_rank1_clean_loops.csv','stage13_loop_schedule.json','clean_graph_policy.json','clean_gtsam_solver_policy.json']]
    paths += [B13/n for n in ['incremental_vs_batch_checkpoints.csv','isam2_parameter_policy.json','numerical_discrepancy_forensics.json','incremental_batch_agreement_policy.json','final_graph_equivalence.json','incremental_event_trace.csv']]
    paths += [B13/f'snapshots/{n}_batch_trajectory.csv' for n in CP]
    records=[]
    for p in paths:
        blob=subprocess.check_output(['git','show',f'{ACCEPTED}:{p.relative_to(ROOT).as_posix()}'],cwd=ROOT)
        h=d.sha256(p);assert h==hashlib.sha256(blob).hexdigest(),str(p)
        records.append(dict(path=str(p),sha256=h,size_bytes=p.stat().st_size,accepted_blob_verified=True))
    dump('stage13c_input_manifest.json',dict(accepted_checkpoint=ACCEPTED,inputs=records,historical_hashes_before=protected,GT_used=False))
    local_tr=pd.read_csv(b.D12/'clean_local_trajectory.csv');common_tr=pd.read_csv(b.D12/'clean_first_loop_trajectory.csv')
    local=d.s10.lookup(local_tr);common=d.s10.lookup(common_tr)
    frames={r:pd.read_csv(d.s10.PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).iloc[start:].copy() for r,start in d.STARTS.items()}
    loops=pd.read_csv(b.D12/'ordered_rank1_clean_loops.csv');assert len(loops)==75 and loops.query_keyframe_id.is_unique
    assert loops.arrival_index.to_list()==list(range(1,76)) and loops.query_robot.eq('robot1').all() and loops.candidate_robot.eq('robot3').all()
    schedule=json.loads((b.D12/'stage13_loop_schedule.json').read_text());assert schedule['ordered_loop_file_sha256']==d.sha256(b.D12/'ordered_rank1_clean_loops.csv')
    basecomp=pd.read_csv(B13/'incremental_vs_batch_checkpoints.csv').set_index('checkpoint');assert basecomp.robot1_keyframe.to_dict()==CP
    original_limits=json.loads((B13/'incremental_batch_agreement_policy.json').read_text())
    limits={k:original_limits[k] for k in ['translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg']}
    first=loops.iloc[0];S=local[('robot1',283)]@d.tm(first)@d.s10.inv(local[('robot3',252)])
    assert max(np.max(abs(common[n]-(local[n] if n[0]=='robot1' else S@local[n]))) for n in local)<1e-9
    E=d.s10.inv(d.tm(first))@d.s10.inv(common[('robot1',283)])@common[('robot3',252)]
    assert np.linalg.norm(E[:3,3])<1e-8 and Rotation.from_matrix(E[:3,:3]).magnitude()<1e-8
    parameter_manifest=[];supported={}
    for v in VARIANTS:
        try:p,text=configure(v);supported[v['name']]=p;status='SUPPORTED'
        except (AttributeError,TypeError,AssertionError) as ex:text=str(ex);status='UNSUPPORTED_API'
        parameter_manifest.append(dict(**v,status=status,parameter_text=text,wildfire=.001,optimizer='GaussNewton',factorization='CHOLESKY',cacheLinearizedFactors=True))
    update_doc=gtsam.ISAM2.update.__doc__ or ''
    empty_exposed='update(self: gtsam.gtsam.ISAM2)' in update_doc
    if not empty_exposed:supported.pop(VARIANTS[-1]['name'],None);parameter_manifest[-1]['status']='UNSUPPORTED_API'
    dump('variant_parameter_manifest.json',parameter_manifest)
    dump('stage13c_isam2_api_audit.json',dict(gtsam=importlib.metadata.version('gtsam'),python=sys.version,executable=sys.executable,CPU=Path('/proc/cpuinfo').read_text().split('model name')[1].split('\n')[0].strip(': '),platform=platform.platform(),parameter_methods=[x for x in dir(gtsam.ISAM2Params()) if not x.startswith('_')],optimization_methods=[x for x in dir(gtsam.ISAM2GaussNewtonParams()) if not x.startswith('_')],empty_update_exposed=empty_exposed,force_relinearization='SUPPORTED' if 'force_relinearize: bool' in update_doc else 'NOT_EXPOSED',force_semantics='installed binding exposes the force_relinearize boolean overload; operational semantics were not empirically verified, no forced-update trial authorized/executed',force_semantics_verification='UNVERIFIED',update_signature=update_doc))
    choose=dict(UTC=b.a.now(),eligible_primary=['V0','V1','V2','V3'],agreement_limits=limits,qualification='all eight checkpoint gates plus structural/finite/schema/catastrophe checks, no GT or hidden LM',ranking='lowest median ordinary post-fusion update latency, p95 first tie break, fewer changes from V0 next',indistinguishable_relative_fraction=.01,V4='OFFLINE DIAGNOSTIC ONLY; excluded from selection',GT_used=False)
    dump('stage13c_selection_policy.json',choose)
    policy=dict(UTC=b.a.now(),variants=VARIANTS,checkpoints=CP,agreement_limits=limits,event_model='prebuilt complete Robot3; streamed Robot1 original order/timestamps; PRECOMPUTED FROZEN LOOP FACTORS, not online discovery',first_graph=dict(nodes=4080,odometry=4078,loops=1,prior=1),final_graph=dict(nodes=5796,odometry=5794,loops=75,prior=1),frame='same Stage12D LiDAR/os_sensor, same URDF extrinsic, KF150 anchor',initialization='same first-loop S; causal previous estimated pose @ new relative odometry thereafter; independent variant instances',noise=json.loads((b.D12/'clean_graph_policy.json').read_text()),batch='reuse accepted Stage13B saved identical-graph batch LM trajectories; verify graph keys, objective and hashes; never initialize ISAM2 from reference',benchmark='one independent replay per variant; normal insertion update, extraction, construction, checkpoint diagnostics and V4 empty updates measured separately; no maps/GT/batch solve',V4='exactly 3 empty calls after every archived loop including first, unless unsafe/API failure; store estimates after0/1/2/3 in server-only NPZ plus manifests and delta/error diagnostics; no forced updates',baseline_reproduction_tolerance=dict(pose_metric_absolute=1e-7,objective_absolute=1e-7),objective_relative_gap='abs difference / max(abs(batch objective),1e-12); numerical-zero objective labelled separately',localization='top20 by translation plus top20 by rotation at every failed checkpoint; same-gauge distance to anchor and KF distance to latest loop endpoints; near means <=50 keyframes, descriptive only',selection=choose,GT_ACCESS='PROHIBITED ENTIRE STAGE',GT_used=False)
    dump('stage13c_experiment_policy.json',policy);event('POLICY_FROZEN')
    template,lmparams=d.build_graph(frames,local,loops)  # construction only; no LM optimizer.
    assert json.loads((OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==json.loads((b.D12/'clean_gtsam_solver_policy.json').read_text())['parameters']
    odom={('robot1',i):i-150 for i in range(151,2000)};odom.update({('robot3',i):1850+i-235 for i in range(235,4180)})
    loop_events={int(q):sub for q,sub in loops.groupby('query_keyframe_id',sort=False)}
    refs={};refmat={};refgraph={};refres={}
    reference_audits=[]
    for name,kf in CP.items():
        f={'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']}
        tr=pd.read_csv(B13/f'snapshots/{name}_batch_trajectory.csv');b.schema(tr,f)
        refs[name]=tr;refmat[name]=d.s10.lookup(tr)
        K=int(basecomp.loc[name,'K']);indices=[0]+[odom[('robot1',i)] for i in range(151,kf+1)]+[odom[('robot3',i)] for i in range(235,4180)]+list(range(5795,5795+K))
        g=b.graph_subset(template,indices);refgraph[name]=g
        assert set(g.keyVector())==set(b.to_values(refmat[name]).keys())
        obj=float(g.error(b.to_values(refmat[name])));expected=float(basecomp.loc[name,'batch_final_error'])
        assert abs(obj-expected)<1e-7
        rr,_=d.loop_residuals(loops.head(K),refmat[name]);refres[name]=rr
        reference_audits.append(dict(checkpoint=name,K=K,kf=kf,nodes=len(tr),factors=g.size(),objective_recomputed=obj,objective_saved=expected,PASS=True,batch_rerun=False))
    csv('batch_reference_reuse_audit.csv',reference_audits)
    state=dict(events=[],numerical=[],objectives=[],robots=[],localization=[],integrity=[],extra=[],extra_manifest=[],variants=[],equivalence={})
    selection=json.loads((d.C12/'stable_segment_selection.json').read_text())
    for v in VARIANTS:
        if v['name'] not in supported:state['variants'].append(dict(variant=v['name'],status='UNSUPPORTED_API',checkpoints_passed=0));continue
        try:run_variant(v,supported[v['name']],template,odom,loop_events,loops,frames,local,common,local_tr,common_tr,S,selection,refs,refmat,refgraph,refres,limits,basecomp,state)
        except Exception as ex:
            # Structural/finite failures stop ONLY this declared independent variant.
            state['variants'].append(dict(variant=v['name'],status='RUNTIME_FAILURE',reason=f'{type(ex).__name__}: {ex}',checkpoints_passed=sum(x['PASS'] for x in state['numerical'] if x['variant']==v['name'])))
            event('VARIANT_STOPPED',variant=v['name'],reason=str(ex))
        persist(state)
    finish(state,policy,protected,basecomp)

def run_variant(v,params,template,odom,loop_events,loops,frames,local,common,local_tr,common_tr,S,selection,refs,refmat,refgraph,refres,limits,basecomp,state):
    variant=v['name'];short=variant.split('_')[0];start_wall=time.perf_counter();event('VARIANT_START',variant=variant)
    isam=None;nodes=[];registered=set();inserted=[];ordered_indices=[];snapshot_count=0
    (OUT/'checkpoints'/short).mkdir();(OUT/'trajectories'/short).mkdir()
    own_rows=[];diagnostic_ms=0.;validation_ms=0.;construction_ms=0.;extra_ms=0.;checkpoint_diag_ms=0.
    for idx,row in enumerate(frames['robot1'].itertuples()):
        kf=int(row.keyframe_id)
        if kf<283:
            # Same independent local-odometry prefix; no joint optimizer.
            if kf==150:prepose=local[('robot1',150)]
            else:prepose=prepose@template.at(odom[('robot1',kf)]).measured().matrix();assert np.max(abs(prepose-local[('robot1',kf)]))<1e-8
            state['events'].append(dict(variant=variant,event_index=idx,robot1_keyframe=kf,timestamp_ns=int(row.lidar_timestamp_ns),update_kind='PRE_FUSION_LOCAL_ONLY',loop_inserted=False,cumulative_loops=0,new_factors=0,new_values=0,graph_nodes=0,graph_factors=0,relinearizeSkip=v['skip'],relinearizationThreshold=v['threshold'],update_ms=None,extraction_ms=None,solver_processing_ms=None,variables_relinearized='NOT_APPLICABLE',variables_reeliminated='NOT_APPLICABLE'))
            continue
        t=time.perf_counter();new=gtsam.NonlinearFactorGraph();nv=gtsam.Values();arrived=loop_events.get(kf)
        if kf==283:
            f={'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']}
            nodes=[(r,int(i)) for r,fr in f.items() for i in fr.keyframe_id]
            indices=[0]+[odom[('robot1',i)] for i in range(151,284)]+[odom[('robot3',i)] for i in range(235,4180)]+[5795]
            new=b.graph_subset(template,indices);nv=b.to_values({n:local[n] if n[0]=='robot1' else S@local[n] for n in nodes})
            assert nv.size()==new.size()==4080;isam=gtsam.ISAM2(params);identity=id(isam);inserted.append(1)
        else:
            assert id(isam)==identity
            node=('robot1',kf);assert node not in nodes
            previous=isam.calculateEstimatePose3(b.numeric_key(('robot1',kf-1)))
            ni=odom[node];nv.insert(b.numeric_key(node),previous.compose(template.at(ni).measured()));new.add(template.at(ni));indices=[ni];nodes.append(node)
            if arrived is not None:
                for r in arrived.itertuples():
                    li=int(r.arrival_index);assert li==len(inserted)+1 and li not in inserted and int(r.query_keyframe_id)==kf
                    assert ('robot3',int(r.candidate_keyframe_id)) in nodes
                    indices.append(5795+li-1);new.add(template.at(indices[-1]));inserted.append(li)
        assert not registered.intersection(indices);registered.update(indices);ordered_indices.extend(indices)
        construct=(time.perf_counter()-t)*1000;construction_ms+=construct
        t=time.perf_counter();result=isam.update(new,nv);update_ms=(time.perf_counter()-t)*1000
        t=time.perf_counter();estimate=isam.calculateEstimate();extract_ms=(time.perf_counter()-t)*1000
        assert estimate.size()==len(nodes) and isam.getFactorsUnsafe().size()==len(registered)
        kind='FIRST_FUSION' if kf==283 else 'LOOP_EVENT' if arrived is not None else 'ODOMETRY_ONLY'
        diag=dict(variant=variant,event_index=idx,robot1_keyframe=kf,timestamp_ns=int(row.lidar_timestamp_ns),update_kind=kind,loop_inserted=arrived is not None,cumulative_loops=len(inserted),new_factors=new.size(),new_values=nv.size(),graph_nodes=len(nodes),graph_factors=len(registered),relinearizeSkip=v['skip'],relinearizationThreshold=v['threshold'],update_ms=update_ms,extraction_ms=extract_ms,graph_newinfo_construction_ms=construct,solver_processing_ms=construct+update_ms+extract_ms,variables_relinearized=counter(result,'getVariablesRelinearized'),variables_reeliminated=counter(result,'getVariablesReeliminated'),extra_update_ms=0.,extra_extraction_ms=0.)
        t=time.perf_counter();mat=safe_matrices(estimate,nodes,common,kf,selection);validation_ms+=(time.perf_counter()-t)*1000
        checkpoint=next((name for name,k in CP.items() if k==kf),None)
        if v['extra'] and arrived is not None:
            tg=time.perf_counter();g=b.graph_subset(template,sorted(registered));base=mat;stored=[np.array([mat[n] for n in nodes])]
            for step in range(4):
                et=0.;ec=0.;relin=diag['variables_relinearized'];reelim=diag['variables_reeliminated']
                if step:
                    t=time.perf_counter();er=isam.update();et=(time.perf_counter()-t)*1000
                    t=time.perf_counter();estimate=isam.calculateEstimate();ec=(time.perf_counter()-t)*1000
                    mat=safe_matrices(estimate,nodes,common,kf,selection);stored.append(np.array([mat[n] for n in nodes]))
                    relin=counter(er,'getVariablesRelinearized');reelim=counter(er,'getVariablesReeliminated')
                    diag['extra_update_ms']+=et;diag['extra_extraction_ms']+=ec;extra_ms+=et
                diff=b.pose_difference(mat,base)
                tobatch=b.pose_difference(mat,refmat[checkpoint]) if checkpoint else {}
                assert isam.getFactorsUnsafe().size()==g.size() and estimate.size()==len(nodes)
                state['extra'].append(dict(variant=variant,kf=kf,arrival_index=len(inserted),checkpoint=checkpoint or 'NON_CHECKPOINT_LOOP',extra_step=step,new_factors=0 if step else new.size(),new_values=0 if step else nv.size(),extra_call_ms=et,extra_estimate_extraction_ms=ec,variables_relinearized=relin,variables_reeliminated=reelim,graph_error=float(g.error(estimate)),translation_median_change_from_step0=diff['translation_median_m'],translation_p95_change_from_step0=diff['translation_p95_m'],rotation_p95_change_from_step0=diff['rotation_p95_deg'],translation_p95_vs_batch=tobatch.get('translation_p95_m')))
                if checkpoint:
                    d.trajectory(f if kf==283 else {'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']},mat).to_csv(OUT/f'checkpoints/{short}/{checkpoint}_extra{step}.csv',index=False)
            path=OUT/f'extra_estimates/loop{len(inserted):02d}.npz'
            np.savez_compressed(path,robot_number=np.array([1 if n[0]=='robot1' else 3 for n in nodes]),keyframe_id=np.array([n[1] for n in nodes]),matrices=np.stack(stored))
            state['extra_manifest'].append(dict(arrival_index=len(inserted),kf=kf,path=str(path),size_bytes=path.stat().st_size,sha256=d.sha256(path),steps=[0,1,2,3],server_only=True))
            diagnostic_ms+=(time.perf_counter()-tg)*1000
        state['events'].append(diag);own_rows.append(diag)
        if checkpoint:
            t=time.perf_counter();f={'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']}
            tr=d.trajectory(f,mat);b.schema(tr,f);tr.to_csv(OUT/f'checkpoints/{short}/{checkpoint}.csv',index=False)
            expected_graph=refgraph[checkpoint]
            assert set(estimate.keys())==set(expected_graph.keyVector()) and len(registered)==expected_graph.size()
            df,parts=metrics(mat,refmat[checkpoint]);combined=parts[0]
            passes=all(combined[k]<=limit for k,limit in limits.items())
            state['numerical'].append(dict(variant=variant,checkpoint=checkpoint,kf=kf,K=len(inserted),PASS=passes,**combined))
            state['robots'].extend(dict(variant=variant,checkpoint=checkpoint,kf=kf,**x) for x in parts)
            incerr=float(expected_graph.error(estimate));batcherr=float(basecomp.loc[checkpoint,'batch_final_error'])
            rr,_=d.loop_residuals(loops.head(len(inserted)),mat);br=refres[checkpoint]
            state['objectives'].append(dict(variant=variant,checkpoint=checkpoint,ISAM2_error=incerr,batch_error=batcherr,absolute_gap=abs(incerr-batcherr),signed_gap=incerr-batcherr,relative_gap=abs(incerr-batcherr)/max(abs(batcherr),1e-12),batch_numerical_zero=batcherr<1e-12,loop_translation_p95_m=float(rr.translation_residual_m.quantile(.95)),batch_loop_translation_p95_m=float(br.translation_residual_m.quantile(.95)),loop_rotation_p95_deg=float(rr.rotation_residual_deg.quantile(.95)),batch_loop_rotation_p95_deg=float(br.rotation_residual_deg.quantile(.95)),finite=True,anchor_translation_m=float(np.linalg.norm(mat[('robot1',150)][:3,3]-local[('robot1',150)][:3,3])),anchor_rotation_deg=float(np.rad2deg(Rotation.from_matrix(local[('robot1',150)][:3,:3].T@mat[('robot1',150)][:3,:3]).magnitude()))))
            state['integrity'].extend(dict(variant=variant,**x) for x in b.integrity(tr,local_tr,common_tr,selection,short,checkpoint))
            if not passes:
                latest=loops.iloc[len(inserted)-1]
                for metric in ['translation_m','rotation_deg']:
                    for rank,r in enumerate(df.nlargest(20,metric).itertuples(),1):
                        endpoint=int(latest.query_keyframe_id if r.robot_id=='robot1' else latest.candidate_keyframe_id)
                        state['localization'].append(dict(variant=variant,checkpoint=checkpoint,ranking_metric=metric,rank=rank,robot_id=r.robot_id,keyframe_id=r.keyframe_id,distance_from_anchor_m=float(np.linalg.norm(refmat[checkpoint][(r.robot_id,r.keyframe_id)][:3,3]-local[('robot1',150)][:3,3])),keyframe_distance_from_latest_endpoint=abs(r.keyframe_id-endpoint),near_latest_endpoint_50kf=abs(r.keyframe_id-endpoint)<=50,translation_m=r.translation_m,rotation_deg=r.rotation_deg,objective_gap=incerr-batcherr))
            # Full nodewise errors for localization figures; no GT.
            df.to_csv(OUT/f'checkpoints/{short}/{checkpoint}_pose_differences.csv',index=False)
            checkpoint_diag_ms+=(time.perf_counter()-t)*1000;snapshot_count+=1
            event('CHECKPOINT',variant=short,checkpoint=checkpoint,PASS=passes,translation_p95_m=combined['translation_p95_m'])
        if kf%300==0:event('STREAM_PROGRESS',variant=short,kf=kf)
    assert snapshot_count==8 and registered==set(range(template.size())) and inserted==list(range(1,76))
    assert len(nodes)==5796 and len(registered)==5870
    actual=isam.getFactorsUnsafe();same=all(actual.at(i).equals(template.at(ix),1e-12) for i,ix in enumerate(ordered_indices))
    assert same and set(estimate.keys())==set(template.keyVector())
    state['equivalence'][variant]=dict(PASS=True,nodes=5796,odom=5794,loops=75,prior=1,factors=5870,same_keys=True,same_measurements_and_noise=same,same_loop_ids=True,same_anchor=True,source_timestamps_preserved=True,identity_tolerance=1e-12)
    tr=d.trajectory(frames,mat);d.schema_audit(tr);tr.to_csv(OUT/f'trajectories/{short}/final_trajectory.csv',index=False)
    passed=sum(x['PASS'] for x in state['numerical'] if x['variant']==variant)
    state['variants'].append(dict(variant=variant,status='ALL_CHECKPOINTS_PASS' if passed==8 else 'SOME_CHECKPOINTS_FAIL' if passed else 'ALL_CHECKPOINTS_FAIL',checkpoints_passed=passed,first_init_ms=own_rows[0]['update_ms'],ordinary_update_total_ms=sum(x['update_ms'] for x in own_rows),extra_update_total_ms=extra_ms,extra_calls=3*75 if v['extra'] else 0,extra_extraction_total_ms=sum(x['extra_extraction_ms'] for x in own_rows),construction_total_ms=construction_ms,estimate_extraction_total_ms=sum(x['extraction_ms'] for x in own_rows),total_incremental_solver_ms=sum(x['solver_processing_ms']+x['extra_update_ms']+x['extra_extraction_ms'] for x in own_rows),integrity_diagnostic_ms=validation_ms,checkpoint_diagnostic_ms=checkpoint_diag_ms,V4_diagnostic_wall_ms=diagnostic_ms,replay_wall_ms=(time.perf_counter()-start_wall)*1000))
    event('VARIANT_COMPLETE',variant=short,checkpoints_passed=passed)

def persist(s):
    for name,key in [('per_event_solver_diagnostics.csv','events'),('numerical_agreement_by_variant.csv','numerical'),('objective_agreement_by_variant.csv','objectives'),('per_robot_pose_difference.csv','robots'),('checkpoint_discrepancy_localization.csv','localization'),('trajectory_integrity_by_variant.csv','integrity'),('extra_update_diagnostic.csv','extra'),('runtime_overhead_by_variant.csv','variants')]:csv(name,s[key])
    dump('final_graph_equivalence_by_variant.json',s['equivalence']);dump('extra_estimate_manifest.json',s['extra_manifest'])

def finish(s,policy,protected,basecomp):
    persist(s);ev=pd.DataFrame(s['events']);post=ev[ev.update_kind.ne('PRE_FUSION_LOCAL_ONLY')].copy();runtime=[]
    for (variant,kind),df in post.groupby(['variant','update_kind'],sort=False):
        for metric in ['update_ms','extraction_ms','solver_processing_ms']:
            x=df[metric].to_numpy(float)
            row=dict(variant=variant,event_type=kind,metric=metric,count=len(x),mean=float(np.mean(x)),median=float(np.median(x)),p90=float(np.percentile(x,90)),p95=float(np.percentile(x,95)),p99=float(np.percentile(x,99)),maximum=float(np.max(x)),sample_limitation='one replay; first fusion one sample, loop events75/74')
            runtime.append(row)
    if s['extra']:
        extra=pd.DataFrame(s['extra']);calls=extra[extra.extra_step.gt(0)]
        for metric in ['extra_call_ms','extra_estimate_extraction_ms']:
            x=calls[metric].to_numpy(float);runtime.append(dict(variant=VARIANTS[-1]['name'],event_type='EXTRA_DIAGNOSTIC_CALL',metric=metric,count=len(x),mean=float(np.mean(x)),median=float(np.median(x)),p90=float(np.percentile(x,90)),p95=float(np.percentile(x,95)),p99=float(np.percentile(x,99)),maximum=float(np.max(x)),sample_limitation='225 diagnostic calls in one replay; excluded from ordinary latency'))
    csv('runtime_by_variant.csv',runtime)
    deadlines=[]
    for v,df in post.groupby('variant',sort=False):
        df=df.copy();ts=df.timestamp_ns.to_numpy(np.int64);interval=np.r_[np.diff(ts)/1e6,np.nan]
        df['next_interval_ms']=interval;valid=np.isfinite(interval)
        update=df.update_ms.to_numpy(float);processing=df.solver_processing_ms.to_numpy(float)+df.extra_update_ms.to_numpy(float)+df.extra_extraction_ms.to_numpy(float)
        deadlines.append(dict(variant=v,events_with_next_interval=int(valid.sum()),update_missed_deadlines=int((update[valid]>=interval[valid]).sum()),full_incremental_solver_processing_missed_deadlines=int((processing[valid]>=interval[valid]).sum()),update_under_fraction=float(np.mean(update[valid]<interval[valid])),full_processing_under_fraction=float(np.mean(processing[valid]<interval[valid])),V4_includes_extra_update_cost=v.startswith('V4')))
    csv('two_hz_backend_timing_check.csv',deadlines)
    n=pd.DataFrame(s['numerical']);v0=n[n.variant.str.startswith('V0')];o=pd.DataFrame(s['objectives'])
    repro=[]
    for r in v0.itertuples():
        old=basecomp.loc[r.checkpoint]
        for field in ['translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg']:
            repro.append(dict(checkpoint=r.checkpoint,metric=field,new=getattr(r,field),reference=float(old[field]),tolerance=1e-7,PASS=abs(getattr(r,field)-float(old[field]))<=1e-7))
        actual=float(o[o.variant.str.startswith('V0')&o.checkpoint.eq(r.checkpoint)].ISAM2_error.iloc[0]);expected=float(old.incremental_final_error)
        repro.append(dict(checkpoint=r.checkpoint,metric='nonlinear_objective',new=actual,reference=expected,tolerance=1e-7,PASS=abs(actual-expected)<=1e-7))
    csv('baseline_reproduction_audit.csv',repro)
    baseline_pass=len(v0)==8 and all(x['PASS'] for x in repro)
    candidates=[]
    for item in s['variants']:
        v=item['variant'];df=post[post.variant.eq(v)&post.update_kind.ne('FIRST_FUSION')]
        candidates.append(dict(**item,post_update_median_ms=float(df.update_ms.median()) if len(df) else None,post_update_mean_ms=float(df.update_ms.mean()) if len(df) else None,post_update_p95_ms=float(df.update_ms.quantile(.95)) if len(df) else None,eligible=not v.startswith('V4') and item['status']=='ALL_CHECKPOINTS_PASS' and s['equivalence'].get(v,{}).get('PASS',False)))
    eligible=[x for x in candidates if x['eligible']];selected=None
    if eligible:
        best=min(x['post_update_median_ms'] for x in eligible);ties=[x for x in eligible if x['post_update_median_ms']<=1.01*best]
        pbest=min(x['post_update_p95_ms'] for x in ties);ties=[x for x in ties if x['post_update_p95_ms']<=1.01*pbest]
        selected=min(ties,key=lambda x:int(x['variant'][1]))['variant']
    failures=any(x['status'] in ['RUNTIME_FAILURE','UNSUPPORTED_API'] for x in candidates if not x['variant'].startswith('V4'))
    decision=dict(decision='STANDARD_ISAM2_CONFIG_FOUND' if selected else 'INCONCLUSIVE_API_OR_RUNTIME_FAILURE' if failures or not baseline_pass else 'NO_STANDARD_CONFIG_MET_GATES',selected_variant=selected,UTC=b.a.now(),selection_policy=policy['selection'],candidate_results=candidates,GT_used_for_selection=False,all_checkpoint_results=s['numerical'],remaining_discrepancy='see original unchanged gate results; no near-pass selected',V4_excluded=True)
    dump('stage13c_numerical_decision.json',decision)
    immutable=all(d.sha256(p)==h for p,h in protected.items());dump('historical_immutability_audit.json',dict(PASS=immutable,files_checked=len(protected),GT_fields_decoded=False))
    checks={'STAGE13B HISTORICAL DATA IMMUTABLE':immutable,'FROZEN INPUT HASHES VERIFIED':True,'PARAMETER API VERIFIED':all(x['status']!='UNSUPPORTED_API' for x in candidates),'PARAMETER POLICY FROZEN':True,'NO GT USED':True,'EVENT SCHEDULE IDENTICAL':all(ev[ev.variant.eq(v['name'])].robot1_keyframe.to_list()==list(range(150,2000)) for v in VARIANTS if v['name'] in set(ev.variant)),'FIRST LOOP IDENTICAL':True,'V0 BASELINE REPRODUCTION':baseline_pass,'FINAL GRAPH EQUIVALENCE':len(s['equivalence'])==5 and all(x['PASS'] for x in s['equivalence'].values()),'ALL CHECKPOINTS TESTED':len(n)==40,'ORIGINAL THRESHOLDS UNCHANGED':True,'UPDATE LATENCY MEASURED':True}
    text='\n'.join(f'{k}: {"PASS" if val else "FAIL"}' for k,val in checks.items())
    for short in ['V1','V2','V3','V4']:
        row=next(x for x in candidates if x['variant'].startswith(short));text+=f'\n{short} COMPLETED: '+('PASS' if row['status'] not in ['RUNTIME_FAILURE','UNSUPPORTED_API'] else row['status'])
    text+=f'\nNUMERICAL PASS FOUND: {"YES" if selected else "NO"}\nSELECTED VARIANT: {selected}\nNEW CATASTROPHIC JUMPS: {"NONE" if len(s["equivalence"])==5 else "SEE FAILURE RECORDS"}\nTWO_HZ_BACKEND_TIMING_CHECK: CONDITIONAL\nGT USED FOR PARAMETER SELECTION: NO\nSTAGE13B MODIFIED: NO\nHISTORICAL DEMO MODIFIED: NO\n'
    (OUT/'VALIDATION_REPORT.txt').write_text(text)
    dump('summary.json',dict(status='PASS' if all(checks.values()) else 'FAIL_EXECUTION_OR_REPRODUCTION',decision=decision['decision'],selected_variant=selected,baseline_reproduced=baseline_pass,variants=candidates,GT_used=False,maps_generated=False,batch_solves=0))
    analysis_summaries(s)
    figures(s);document(s,decision,baseline_pass,deadlines)
    event('COMPLETE',decision=decision['decision'],selected_variant=selected)

def figures(s):
    n=pd.DataFrame(s['numerical']);e=pd.DataFrame(s['events']);o=pd.DataFrame(s['objectives']);runtime=pd.read_csv(OUT/'runtime_by_variant.csv')
    def save(fig,name):fig.savefig(OUT/name,dpi=160);plt.close(fig)
    colors=['tab:blue','tab:orange','tab:green','tab:red','tab:purple']
    fig,ax=plt.subplots(figsize=(11,5),constrained_layout=True)
    for v,c in zip(VARIANTS[:4],colors):
        x=n[n.variant.eq(v['name'])];ax.plot(x.checkpoint,x.translation_p95_m,'-o',color=c,label=v['name'][:2])
    ax.axhline(.5,color='black',ls='--',label='unchanged0.50m');ax.set_yscale('symlog',linthresh=.1);ax.set(ylabel='Translation p95 (m)',title='Fixed relinearization ablations');ax.tick_params(axis='x',rotation=30);ax.legend();save(fig,'01_relinearization_ablation.png')
    fig,ax=plt.subplots(figsize=(11,4),constrained_layout=True)
    for v,c in zip(VARIANTS,colors):
        x=o[o.variant.eq(v['name'])];ax.plot(x.checkpoint,x.absolute_gap,'-o',c=c,label=v['name'][:2])
    ax.set_yscale('symlog',linthresh=1e-5);ax.set(ylabel='Absolute nonlinear objective gap');ax.tick_params(axis='x',rotation=30);ax.legend();save(fig,'02_checkpoint_objective_gaps.png')
    fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True,constrained_layout=True)
    for ax,v,c in zip(axs,VARIANTS[:4],colors):
        x=e[e.variant.eq(v['name'])&e.update_kind.ne('PRE_FUSION_LOCAL_ONLY')];y=pd.to_numeric(x.variables_relinearized,errors='coerce');ax.plot(x.robot1_keyframe,y,lw=.6,c=c);q=x[x.loop_inserted];ax.scatter(q.robot1_keyframe,np.zeros(len(q)),s=8,c='black',label='loop arrival');ax.set(ylabel='Variables',title=v['name'][:2]);ax.legend()
    axs[-1].set_xlabel('Robot1 KF');save(fig,'03_relinearized_variables_timeline.png')
    fig,ax=plt.subplots(figsize=(9,4),constrained_layout=True)
    for kind in ['ODOMETRY_ONLY','LOOP_EVENT']:
        x=runtime[runtime.event_type.eq(kind)&runtime.metric.eq('update_ms')];ax.plot([v[:2] for v in x.variant],x['median'],'-o',label=kind+' median');ax.plot([v[:2] for v in x.variant],x.p95,'--o',label=kind+' p95')
    ax.set(ylabel='Update ms',title='V4 ordinary calls only (extras separate)');ax.legend();save(fig,'04_update_latency_by_variant.png')
    fig,ax=plt.subplots(figsize=(8,5),constrained_layout=True)
    for v,c in zip(VARIANTS[:4],colors):
        x=e[e.variant.eq(v['name'])&e.update_kind.isin(['ODOMETRY_ONLY','LOOP_EVENT'])];y=n[n.variant.eq(v['name'])].translation_p95_m.max();ax.scatter(x.update_ms.median(),y,c=c);ax.annotate(v['name'][:2],(x.update_ms.median(),y))
    ax.axhline(.5,ls='--',c='black');ax.set(xlabel='Median post-fusion update ms',ylabel='Worst checkpoint translation p95 m');save(fig,'05_accuracy_latency_tradeoff.png')
    for checkpoint,num in [('K2','06'),('K10','07')]:
        fig,axs=plt.subplots(1,2,figsize=(14,4),constrained_layout=True)
        for v,c in zip(VARIANTS,colors):
            p=OUT/f'checkpoints/{v["name"][:2]}/{checkpoint}_pose_differences.csv'
            if not p.exists():continue
            df=pd.read_csv(p)
            for ax,r in zip(axs,['robot1','robot3']):
                x=df[df.robot_id.eq(r)];ax.plot(x.keyframe_id,x.translation_m,c=c,label=v['name'][:2]);ax.set(title=r+' '+checkpoint,xlabel='Original KF',ylabel='Translation difference m');ax.legend()
        save(fig,num+'_'+checkpoint+'_divergence_localization.png')
    ref=pd.read_csv(B13/'snapshots/FINAL_STREAM_END_batch_trajectory.csv');allxy=[ref[['tx','ty']].to_numpy()]
    for v in VARIANTS:
        p=OUT/f'trajectories/{v["name"][:2]}/final_trajectory.csv'
        if p.exists():allxy.append(pd.read_csv(p)[['tx','ty']].to_numpy())
    xy=np.vstack(allxy);lo,hi=xy.min(0)-5,xy.max(0)+5;fig,axs=plt.subplots(1,5,figsize=(20,5),constrained_layout=True)
    for ax,v in zip(axs,VARIANTS):
        p=OUT/f'trajectories/{v["name"][:2]}/final_trajectory.csv'
        if not p.exists():ax.set_title(v['name'][:2]+' not completed');continue
        tr=pd.read_csv(p)
        for robot,c in [('robot1','tab:blue'),('robot3','tab:orange')]:
            x=ref[ref.robot_id.eq(robot)];ax.plot(x.tx,x.ty,ls='--',lw=1,c='black',alpha=.5)
            x=tr[tr.robot_id.eq(robot)];ax.plot(x.tx,x.ty,lw=.7,c=c,label=robot)
        ax.set(title=v['name'][:2]+' vs batch (dashed)',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]));ax.set_aspect('equal');ax.legend(fontsize=7)
    save(fig,'08_final_trajectory_difference.png')
    if s['extra']:
        x=pd.DataFrame(s['extra']);fig,axs=plt.subplots(1,2,figsize=(13,4),constrained_layout=True)
        for name in ['K2','K10','K75']:
            df=x[x.checkpoint.eq(name)];axs[0].plot(df.extra_step,df.graph_error,'-o',label=name);axs[1].plot(df.extra_step,df.translation_p95_vs_batch,'-o',label=name)
        axs[0].set(ylabel='Nonlinear error',xlabel='Extra empty calls',title='V4 diagnostic, not guaranteed convergence');axs[1].set(ylabel='Translation p95 vs batch m',xlabel='Extra empty calls');[ax.legend() for ax in axs];save(fig,'09_extra_updates_diagnostic.png')

def document(s,decision,repro,deadlines):
    def table(df):
        return '| '+' | '.join(df.columns)+' |\n| '+' | '.join(['---']*len(df.columns))+' |\n'+'\n'.join('| '+' | '.join('N/A' if pd.isna(x) else f'{x:.6f}' if isinstance(x,float) else str(x) for x in row)+' |' for row in df.itertuples(index=False,name=None))
    n=pd.DataFrame(s['numerical']);e=pd.DataFrame(s['events']);o=pd.DataFrame(s['objectives'])
    result=n.copy();result['variant']=result.variant.str[:2]
    sections=[('Motivation','Independent GT-free numerical convergence study, not trajectory-accuracy or map-consistency evaluation.'),('Stage13B numerical disagreement','Frozen default result remains FAIL_NUMERICAL_AGREEMENT / INCREMENTAL_BACKEND_NOT_READY. Final objectives nearly agree without passing pose gates.'),('Fixed graph and event schedule','Prebuilt Robot3 KF234–4179 plus streamed Robot1 KF150–1999. First graph4080 nodes at KF283; archived loops, not causal online SC/GICP. Same actual Stage12D factors/anchor/noise/frames, causal previous-estimate initial guesses.'),('Reproducibility and environment',f'Accepted `{ACCEPTED}`; isolated GTSAM4.2/Python3.10. Recorded API/configuration/input SHA. Run `bash scripts/run_stage13c_isam2_convergence.sh` with empty output directory; never invokes historical mains, offline GT, map builders or LM optimizer.'),('Frozen ISAM2 parameter variants',table(pd.DataFrame(VARIANTS))),('Baseline replication',f'V0 reproduced original eight checkpoint pose statistics: {repro}. See baseline_reproduction_audit.csv; runtime not required to match another host load/time.'),('Relinearization frequency ablation','V0 vs V1 changes only skip10→1; compare measured K2 and checkpoint differences below. More frequent CHECKS do not guarantee every variable is relinearized.'),('Threshold ablation','V1→V2→V3 threshold0.1→0.01→0.001, skip1 fixed, wildfire0.001 unchanged. No expanded grid.'),('Extra-update diagnostic','V4 defaults +exactly3 supported no-argument empty updates after each of75 loops, including first. Ordinary insertion time separate. Estimates0/1/2/3 stored server-only NPZ with hashes and checkpoint CSVs. Empty calls can be no-ops; exposed counters and changes recorded; V4 never eligible for standard selection.'),('Numerical agreement across checkpoints',table(result[['variant','checkpoint','translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg','PASS']])+'\n\nOriginal gates0.10/0.50m translation and0.5/2deg rotation; all four at all eight checkpoints. Same gauge, no extra alignment. Objective agreement is not pose agreement.'),('K2/K10 discrepancy localization','Per-robot metrics and worst20 translation +worst20 rotation keys, anchor distance/latest-endpoint KF distance saved. <=50KF is a frozen descriptive near-endpoint rule, not a qualification gate. See localization.csv and figures; far-chain errors must not be described as exclusively endpoint-local.'),('Relinearization counts and limitations','Per-event counters measured from binding, NOT_EXPOSED if unavailable. Zero at a checkpoint does not imply zero in preceding events. Variants each use new ISAM2 instance; no warm-start from other trials.'),('Runtime tradeoff',table(pd.DataFrame(decision['candidate_results'])[['variant','status','checkpoints_passed','post_update_median_ms','post_update_mean_ms','post_update_p95_ms']])+'\n\nOne replay each; first fusion one sample. Update/construction/extraction/validation/checkpoint/V4 diagnostic costs separately saved; no batch solves/maps/GT in timings.'),('2Hz engineering diagnostic',table(pd.DataFrame(deadlines))+'\n\nBackend-only, no sleep. ExtraV4 calls included in its solver/update-processing budget, never hidden in ordinary update. Full-system real-time capability NOT established.'),('Final graph equivalence','Every completed variant:5796 nodes/5794 odometry/75 loops/1 prior=5870 factors; GTSAM factor equals checks actual insertion ordering against immutable template, keys/registry/timestamps unchanged. Finite/unit quaternion/IDs and frozen catastrophic-step protocol checked.'),('GT-free selection decision',f'`{decision["decision"]}`; selected `{decision["selected_variant"]}` at {decision["UTC"]}. Only V0–V3 all-checkpoint qualifiers; lowest median ordinary post-fusion update, p95 tie-break, then fewer parameter changes within predeclared1% timing indistinguishability. V4 excluded. GT_used_for_selection=false.'),('Evidence-supported interpretation','Controlled frequency/threshold changes support sensitivity statements only where measured contrasts demonstrate it. Caching, wildfire stopping, weak directions, different nonlinear paths remain alternative hypotheses; no claim that relinearization is the sole cause.'),('Unresolved numerical issues','Numerical agreement, backend runtime, trajectory accuracy, map consistency and full-system real-time capability are distinct. No GT or map inference here. Any unsupported/failed variant remains recorded, not silently repaired with another policy.'),('Next-stage recommendation','Stop after13C. If a primary config meets all gates, independently validate its robustness and engineering budget before demo use. If none passes, separately declare a further GT-free solver-method audit; do not choose near-pass, change loops/demo or tune from GT.')]
    contrast=n[n.checkpoint.isin(['K2','K10'])].merge(o[['variant','checkpoint','absolute_gap']],on=['variant','checkpoint'])
    contrast['variant']=contrast.variant.str[:2]
    localization=pd.read_csv(OUT/'discrepancy_localization_summary.csv')
    effects=json.loads((OUT/'extra_update_effect_summary.json').read_text())
    counts={v['name'][:2]:int(n[n.variant.eq(v['name'])].PASS.sum()) for v in VARIANTS}
    all_pass=[x['variant'] for x in decision['candidate_results'] if x['checkpoints_passed']==8]
    gate_message='NO CONFIGURATION PASSED ALL EIGHT CHECKPOINTS; no standard selection is justified.' if not all_pass else 'All-checkpoint qualifiers: '+', '.join(all_pass)+'; standard selection remains governed by the frozen primary-only rule.'
    observed=f'''\n\nMeasured controlled contrasts:\n\n{table(contrast[['variant','checkpoint','translation_median_m','translation_p95_m','absolute_gap']])}

V0→V1 (skip10→1, threshold0.1 fixed) changes K10 translation p95 from {float(contrast[contrast.variant.eq('V0')&contrast.checkpoint.eq('K10')].translation_p95_m.iloc[0]):.6f} to {float(contrast[contrast.variant.eq('V1')&contrast.checkpoint.eq('K10')].translation_p95_m.iloc[0]):.6f}m, but K2 stays unchanged. V1→V2→V3 threshold reduction gives modest further K10 improvement, not monotonic final improvement. Checkpoints passed: {counts}. {gate_message}

V4 records {effects['extra_calls']} empty calls; {effects['material_pose_change_calls']} materially change the recorded pose estimate, {effects['relinearized_call_count']} expose positive relinearized counts, and {effects['reeliminated_call_count']} expose positive re-eliminated counts. Many calls are pose-negligible despite re-elimination. Empty calls advance the skip counter as well as adding computation; this diagnostic does not isolate nonlinear iteration count alone.

Localization of failing K2/K10 checkpoints:\n\n{table(localization[localization.checkpoint.isin(['K2','K10'])][['variant','checkpoint','metric','largest_robot_p95','top20_far_from_latest_endpoint','top20_max_kf_distance','extent']])}

Translation discrepancy is principally Robot3 and distributed beyond the recent endpoint neighborhood in this replay, not just a new Robot1 pose issue. Long-chain/weak-direction/path-dependence explanations remain hypotheses; no Hessian conditioning study or alternate optimizer path was performed. No GT trajectory accuracy or map consistency conclusion follows.
'''
    sections[16]=(sections[16][0],sections[16][1]+observed)
    text='# Stage13C — ISAM2 numerical convergence and relinearization\n\n'
    text+='\n\n'.join(f'## {i}. {title}\n\n{body}' for i,(title,body) in enumerate(sections,1))+'\n'
    (ROOT/'docs/CUMULTI_STAGE13C_ISAM2_CONVERGENCE.md').write_text(text)

def analysis_summaries(s):
    """Postprocessing recorded estimates only: no optimizer or policy adjustment."""
    loc=pd.DataFrame(s['localization']);robots=pd.DataFrame(s['robots']);summaries=[]
    for (variant,cp),group in loc.groupby(['variant','checkpoint'],sort=False):
        r=robots[robots.variant.eq(variant)&robots.checkpoint.eq(cp)&robots.scope.ne('combined')]
        for metric,field in [('translation','translation_p95_m'),('rotation','rotation_p95_deg')]:
            top=group[group.ranking_metric.eq(metric+'_m' if metric=='translation' else 'rotation_deg')]
            dominant=r.loc[r[field].idxmax(),'scope']
            summaries.append(dict(variant=variant,checkpoint=cp,metric=metric,largest_robot_p95=dominant,top20_far_from_latest_endpoint=int((~top.near_latest_endpoint_50kf).sum()),top20_max_kf_distance=int(top.keyframe_distance_from_latest_endpoint.max()),extent='LONG_CHAIN_NOT_ENDPOINT_ONLY' if (~top.near_latest_endpoint_50kf).sum()>=10 else 'MOSTLY_ENDPOINT_NEAR_OR_MIXED',localization_rule='frozen <=50KF descriptive neighborhood, not GT/qualification'))
    csv('discrepancy_localization_summary.csv',summaries)
    effects=[];extra=pd.DataFrame(s['extra'])
    for artifact in s['extra_manifest']:
        archive=np.load(artifact['path']);m=archive['matrices']
        group=extra[extra.arrival_index.eq(artifact['arrival_index'])].set_index('extra_step')
        for step in [1,2,3]:
            delta=np.linalg.norm(m[step,:,:3,3]-m[step-1,:,:3,3],axis=1)
            rotation=np.rad2deg((Rotation.from_matrix(m[step-1,:,:3,:3]).inv()*Rotation.from_matrix(m[step,:,:3,:3])).magnitude())
            row=group.loc[step];effects.append(dict(arrival_index=artifact['arrival_index'],extra_step=step,translation_p95_change_from_previous_m=float(np.percentile(delta,95)),translation_max_change_m=float(np.max(delta)),rotation_max_change_deg=float(np.max(rotation)),material_change=bool(np.max(delta)>1e-7 or np.max(rotation)>1e-7),nonlinear_error_change=float(row.graph_error-group.loc[step-1].graph_error),variables_relinearized=row.variables_relinearized,variables_reeliminated=row.variables_reeliminated))
    csv('extra_update_effects.csv',effects)
    dump('extra_update_effect_summary.json',dict(extra_calls=len(effects),material_pose_change_calls=int(sum(x['material_change'] for x in effects)),relinearized_call_count=int(sum(isinstance(x['variables_relinearized'],(int,float,np.integer,np.floating)) and x['variables_relinearized']>0 for x in effects)),reeliminated_call_count=int(sum(isinstance(x['variables_reeliminated'],(int,float,np.integer,np.floating)) and x['variables_reeliminated']>0 for x in effects)),descriptive_material_change_threshold='1e-7 m or deg, same numeric scale as frozen replication tolerance; NOT an acceptance gate',convergence_not_guaranteed=True,extra_calls_also_advance_skip_counter=True))

def report_only():
    # Never touch frozen decision/policy/measurements; regenerate prose/derived
    # localization summaries from accepted numerical tables/recorded archives.
    j=lambda name:json.loads((OUT/name).read_text())
    keys={'events':'per_event_solver_diagnostics.csv','numerical':'numerical_agreement_by_variant.csv','objectives':'objective_agreement_by_variant.csv','robots':'per_robot_pose_difference.csv','localization':'checkpoint_discrepancy_localization.csv','extra':'extra_update_diagnostic.csv'}
    s={key:pd.read_csv(OUT/name).to_dict('records') for key,name in keys.items()};s['extra_manifest']=j('extra_estimate_manifest.json')
    api=j('stage13c_isam2_api_audit.json')
    api['force_relinearization']='SUPPORTED' if 'force_relinearize: bool' in api['update_signature'] else 'NOT_EXPOSED'
    api['force_semantics']='installed binding exposes the force_relinearize boolean overload; operational semantics not empirically verified, no forced-update trial executed'
    api['force_semantics_verification']='UNVERIFIED'
    dump('stage13c_isam2_api_audit.json',api)
    analysis_summaries(s)
    figures(s)
    document(s,j('stage13c_numerical_decision.json'),j('summary.json')['baseline_reproduced'],pd.read_csv(OUT/'two_hz_backend_timing_check.csv').to_dict('records'))
    EVENTS[:]=j('execution_events.json')
    event('FINALIZATION_COMPLETE_NO_OPTIMIZATION',decision=j('stage13c_numerical_decision.json')['decision'])

if __name__=='__main__':
    if sys.argv[1:]==['--report-only']:report_only()
    elif sys.argv[1:]:raise SystemExit('Usage: run_stage13c_isam2_convergence.py [--report-only]')
    else:main()
