#!/usr/bin/env python3
"""Independent saved-state audit: no optimization, GT or selection retuning."""
import ast
import hashlib
import json
import os
import sys
from pathlib import Path
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.stats import rankdata
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_stage13b_incremental_isam2 as b
d,a=b.d,b.a
ROOT=d.ROOT
OUT=ROOT/'outputs/cumulti_v1/15_informative_loop_selection'
D12,A13=b.D12,b.A13
SEL=['ARRIVAL_PREFIX','QUALITY_GREEDY','COVERAGE_GREEDY','QUALITY_COVERAGE_GREEDY']
KS=[1,2,5,10,20,40,75]

def j(name):return json.loads((OUT/name).read_text())
def c(name):return pd.read_csv(OUT/name)
def check(v,label):
    assert v,label
    checks[label]=True

def independent_rank(loops,selector):
    pairs=np.column_stack(((loops.query_keyframe_id-150)/1849,(loops.candidate_keyframe_id-234)/3945))
    chosen=[45];ids=loops.frozen_loop_id.to_list();pos={x:i for i,x in enumerate(ids)};scores=[]
    while len(chosen)<75:
        remaining=[x for x in ids if x not in chosen]
        dist=cKDTree(pairs[[pos[x] for x in chosen]]).query(pairs[[pos[x] for x in remaining]])[0]
        quality=np.array([float(loops.iloc[pos[x]].GICP_quality) for x in remaining])
        score=np.sqrt(rankdata(quality,method='average')/len(remaining)*rankdata(dist,method='average')/len(remaining))
        def key(ix):
            x=remaining[ix];r=loops.iloc[pos[x]]
            primary=-ix if selector==SEL[0] else quality[ix] if selector==SEL[1] else dist[ix] if selector==SEL[2] else score[ix]
            return (-primary,-quality[ix],int(r.query_timestamp),x)
        winner=min(range(len(remaining)),key=key)
        # Dense Euclidean and KD distances only compared with tolerance, never
        # use approximate rounding in the actual ranking/tie rule.
        scores.extend(dict(selector=selector,greedy_rank=len(chosen)+1,frozen_loop_id=x,quality=quality[i],coverage_novelty=dist[i],quality_rank=rankdata(quality,method='average')[i]/len(remaining),novelty_rank=rankdata(dist,method='average')[i]/len(remaining),geometric_mean_score=score[i],chosen=i==winner) for i,x in enumerate(remaining))
        chosen.append(remaining[winner])
    return chosen,scores

checks={}
def rotation_details():
    """Saved trajectory explanation only; never changes flags/decision/policy."""
    solver=c('batch_solver_results.csv');common=d.s10.lookup(pd.read_csv(D12/'clean_first_loop_trajectory.csv',usecols=d.POSE_COLUMNS));stable=json.loads((d.C12/'stable_segment_selection.json').read_text());rows=[]
    for r in solver.itertuples():
        tr=pd.read_csv(r.trajectory_path,usecols=d.POSE_COLUMNS);mat=d.s10.lookup(tr)
        for robot in d.STARTS:
            ids=tr[tr.robot_id.eq(robot)].keyframe_id.to_list();M=np.array([mat[(robot,int(k))][:3,:3] for k in ids]);P=np.array([common[(robot,int(k))][:3,:3] for k in ids])
            post=np.rad2deg(Rotation.from_matrix(M[:-1].transpose(0,2,1)@M[1:]).magnitude());pre=np.rad2deg(Rotation.from_matrix(P[:-1].transpose(0,2,1)@P[1:]).magnitude());limit=stable['robots'][robot]['frozen_thresholds']['rotation_step_deg']['threshold']
            for i in np.flatnonzero((post>limit)&(pre<=limit)):
                rows.append(dict(selector=r.selector,K=r.K,robot_id=robot,keyframe_i=ids[i],keyframe_j=ids[i+1],pre_rotation_deg=float(pre[i]),post_rotation_deg=float(post[i]),frozen_threshold_deg=limit,excess_deg=float(post[i]-limit),change_deg=float(post[i]-pre[i]),classification='NEW_ROTATION_THRESHOLD_CROSSING; physical catastrophe NOT_ESTABLISHED'))
    pd.DataFrame(rows).to_csv(OUT/'rotation_threshold_crossing_details.csv',index=False)
    print(json.dumps(dict(rotation_crossings=len(rows),optimization_run=False,policy_or_decision_changed=False)))

def main():
    policy=j('stage15_experiment_policy.json');manifest=j('stage15_input_manifest.json');decision=j('stage15_pre_gt_decision.json');final=j('stage15_decision.json')
    check(policy['budgets']==KS and policy['success']==dict(map_median_ratio=1.10,map_p95_ratio=1.10,heldout_translation_p95_ratio=1.50,heldout_rotation_p95_ratio=1.50,normal_finite_integrity=True),'exact_predeclared_gates')
    for entry in manifest['inputs']:check(d.sha256(entry['path'])==entry['sha256'],'input_hash_'+Path(entry['path']).name)
    check(all(d.sha256(p)==h for p,h in manifest['historical_before'].items()),'all_historical_hashes_preserved')
    # Verify external tables against the historical13A input manifest without
    # decoding any GT columns they may also contain.
    external=[]
    for entry in json.loads((A13/'stage13a_input_manifest.json').read_text())['inputs']:
        if Path(entry['path']).name=='keyframes.csv':
            check(d.sha256(entry['path'])==entry['sha256'],'historical_keyframe_hash_'+Path(entry['path']).parent.name)
            external.append(entry)
    check(len(external)==2,'two_frozen_external_keyframe_tables')
    for entry in j('sampled_lidar_input_manifest.json'):
        check(d.sha256(entry['path'])==entry['sha256'],'sampled_scan_hash_'+entry['robot_id']+str(entry['keyframe_id']))
    source=pd.read_csv(d.LOOP_SOURCE,usecols=d.LOOP_COLUMNS);source.insert(0,'frozen_loop_id',np.arange(len(source)))
    loops=source[source.query_keyframe_id.ge(150)&source.candidate_keyframe_id.ge(234)].sort_values(['query_timestamp','query_keyframe_id','candidate_keyframe_id'],kind='stable').reset_index(drop=True);loops.insert(0,'arrival_index',np.arange(1,76))
    frozen=pd.read_csv(D12/'ordered_rank1_clean_loops.csv',usecols=['arrival_index','frozen_loop_id']+d.LOOP_COLUMNS)
    for col in frozen:
        if pd.api.types.is_float_dtype(frozen[col]):check(np.allclose(loops[col],frozen[col],atol=1e-12,rtol=0),'loop_exact_'+col)
        else:check(loops[col].to_list()==frozen[col].to_list(),'loop_exact_'+col)
    check(len(source)==120 and len(loops)==75 and j('loop_provenance_audit.json')['excluded_ids']==list(range(45)),'120_to_75_excluded45')
    ranks=c('loop_rankings.csv');trace=c('selector_trace.csv');sets=j('selected_loops_by_budget.json')
    check(len(ranks)==300 and len(trace)==11100,'ranking_trace_completeness')
    for selector in SEL:
        ids,scores=independent_rank(loops,selector);z=ranks[ranks.selector.eq(selector)].sort_values('selected_rank')
        check(z.frozen_loop_id.to_list()==ids,'independent_ranking_'+selector)
        expected=pd.DataFrame(scores);actual=trace[trace.selector.eq(selector)].sort_values(['greedy_rank','frozen_loop_id']);expected=expected.sort_values(['greedy_rank','frozen_loop_id'])
        for field in ['quality','coverage_novelty','quality_rank','novelty_rank','geometric_mean_score']:
            check(np.allclose(actual[field],expected[field],atol=1e-12,rtol=0),'trace_'+selector+'_'+field)
        check(actual.chosen.to_list()==expected.chosen.to_list(),'trace_choice_'+selector)
        for K in KS:check(sets[selector][str(K)]==ids[:K] and len(set(ids[:K]))==K and ids[0]==45,'nested_'+selector+'_'+str(K))
    local_tr=pd.read_csv(D12/'clean_local_trajectory.csv',usecols=d.POSE_COLUMNS);common_tr=pd.read_csv(D12/'clean_first_loop_trajectory.csv',usecols=d.POSE_COLUMNS)
    local,common=d.s10.lookup(local_tr),d.s10.lookup(common_tr)
    frames={r:pd.read_csv(d.s10.PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).iloc[start:] for r,start in d.STARTS.items()}
    # Redirect only helper policy output into new audit subdirectory.
    d.OUT=OUT/'audit_helper';d.OUT.mkdir(exist_ok=True)
    template,params=d.build_graph(frames,local,loops)
    check(json.loads((d.OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==policy['LM']['parameters'],'frozen_LM_getters')
    solver=c('batch_solver_results.csv');graphs=c('graph_equivalence_by_condition.csv');maps=c('map_consistency.csv');res=c('loop_residual_details.csv');held=c('heldout_loop_residuals.csv');selected=c('selected_loop_residuals.csv');all75=c('all75_loop_residuals.csv');cov=c('structural_coverage.csv');q=c('policy_comparison.csv')
    for df in [solver,graphs,maps,held,selected,all75,cov,q]:check(len(df)==28 and not df.duplicated(['selector','K']).any(),'condition_rows_'+str(df.columns[2]))
    trajectories={}
    for entry in j('trajectory_artifact_manifest.json'):
        p=Path(entry['path']);check(d.sha256(p)==entry['sha256'],'trajectory_hash_'+entry['condition'])
        tr=pd.read_csv(p,usecols=d.POSE_COLUMNS);check(b.schema(tr,frames),'trajectory_schema_'+entry['condition']);trajectories[str(p)]=d.s10.lookup(tr)
        for robot in d.STARTS:
            z=tr[tr.robot_id.eq(robot)];xyz=z[['tx','ty','tz']].to_numpy();rot=Rotation.from_quat(z[['qx','qy','qz','qw']]).as_matrix()
            check(z.timestamp.to_list()==frames[robot].lidar_timestamp_ns.to_list(),'original_timestamps_'+entry['condition']+'_'+robot)
            check(np.all(np.linalg.det(rot)>1-1e-8) and np.max(abs(rot.transpose(0,2,1)@rot-np.eye(3)))<1e-8,'valid_SO3_'+entry['condition']+'_'+robot)
    stable=json.loads((d.C12/'stable_segment_selection.json').read_text())
    independent=[]
    for r in solver.itertuples():
        ids=sets[r.selector][str(r.K)];sel=loops[loops.frozen_loop_id.isin(ids)];indices=list(range(5795))+[5795+int(x)-1 for x in sel.arrival_index]
        g=b.graph_subset(template,indices);mat=trajectories[r.trajectory_path];values=b.to_values(mat)
        check(g.size()==5795+r.K and values.size()==5796 and len(set(indices))==len(indices),'graph_'+r.selector+str(r.K))
        check(abs(float(g.error(values))-r.final_error)<1e-7 and abs(float(g.error(b.to_values(common)))-r.initial_error)<1e-7,'objectives_'+r.selector+str(r.K))
        anchor=d.s10.inv(local[('robot1',150)])@mat[('robot1',150)]
        check(np.linalg.norm(anchor[:3,3])<1e-6 and Rotation.from_matrix(anchor[:3,:3]).magnitude()<np.deg2rad(1e-6),'anchor_'+r.selector+str(r.K))
        # Auditor computes residual independently, not through runner or helper.
        rr=[]
        for loop in loops.itertuples():
            Z=d.s10.T([loop.qx,loop.qy,loop.qz,loop.qw],[loop.tx,loop.ty,loop.tz]);E=np.linalg.inv(Z)@np.linalg.inv(mat[('robot1',int(loop.query_keyframe_id))])@mat[('robot3',int(loop.candidate_keyframe_id))]
            rr.append(dict(frozen_loop_id=int(loop.frozen_loop_id),translation_residual_m=float(np.linalg.norm(E[:3,3])),rotation_residual_deg=float(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude()))))
        rr=pd.DataFrame(rr);saved=res[(res.selector==r.selector)&(res.K==r.K)]
        check(saved.frozen_loop_id.to_list()==rr.frozen_loop_id.to_list() and np.allclose(saved[['translation_residual_m','rotation_residual_deg']],rr[['translation_residual_m','rotation_residual_deg']],atol=1e-8,rtol=0),'all75_residuals_'+r.selector+str(r.K))
        for df,part in [(selected,rr[rr.frozen_loop_id.isin(ids)]),(held,rr[~rr.frozen_loop_id.isin(ids)]),(all75,rr)]:
            row=df[(df.selector==r.selector)&(df.K==r.K)].iloc[0];check(row['count']==len(part),'residual_counts_'+r.selector+str(r.K)+str(len(part)))
            for field,col in [('translation_p95_m','translation_residual_m'),('rotation_p95_deg','rotation_residual_deg')]:
                check(pd.isna(row[field]) if not len(part) else abs(row[field]-np.percentile(part[col],95))<1e-8,'residual_p95_'+r.selector+str(r.K)+field+str(len(part)))
        safe=True
        for robot in d.STARTS:
            ids_robot=frames[robot].keyframe_id.to_list();M=np.array([mat[(robot,int(k))] for k in ids_robot]);P=np.array([common[(robot,int(k))] for k in ids_robot])
            dt=np.linalg.norm(np.diff(M[:,:3,3],axis=0),axis=1);pt=np.linalg.norm(np.diff(P[:,:3,3],axis=0),axis=1);lim=stable['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold']
            dr=np.rad2deg(Rotation.from_matrix(M[:-1,:3,:3].transpose(0,2,1)@M[1:,:3,:3]).magnitude());pr=np.rad2deg(Rotation.from_matrix(P[:-1,:3,:3].transpose(0,2,1)@P[1:,:3,:3]).magnitude());rlim=stable['robots'][robot]['frozen_thresholds']['rotation_step_deg']['threshold']
            safe &= not ((((dt>5*lim)&(pt<=5*lim))|((dt>=100)&(pt<100))).any() or ((dr>rlim)&(pr<=rlim)).any())
        mm=maps[(maps.selector==r.selector)&(maps.K==r.K)].iloc[0];hr=held[(held.selector==r.selector)&(held.K==r.K)].iloc[0];ar=all75[(all75.selector==r.selector)&(all75.K==r.K)].iloc[0]
        independent.append(dict(selector=r.selector,K=r.K,map_median_m=float(mm.median_m),map_p95_m=float(mm.p95_m),heldout_translation_p95_m=float(hr.translation_p95_m),heldout_rotation_p95_deg=float(hr.rotation_p95_deg),all75_translation_p95_m=float(ar.translation_p95_m),all75_rotation_p95_deg=float(ar.rotation_p95_deg),normal_finite_integrity=bool(safe and r.normal_return and not r.iteration_cap_reached),batch_runtime_ms=r.batch_runtime_ms))
        # Structural metrics recomputed from endpoint IDs alone.
        c_row=cov[(cov.selector==r.selector)&(cov.K==r.K)].iloc[0]
        for robot,col in [('robot1','query_keyframe_id'),('robot3','candidate_keyframe_id')]:
            ends=np.sort(sel[col].unique());dist=np.min(abs(loops[col].to_numpy()[:,None]-ends),axis=1)
            check(abs(c_row[robot+'_p95_nearest_endpoint_KF']-np.percentile(dist,95))<1e-9 and c_row[robot+'_span_KF']==np.ptp(ends),'structural_'+r.selector+str(r.K)+robot)
    ref=next(r for r in independent if r['selector']==SEL[0] and r['K']==75)
    limits={x:factor*ref[source] for x,source,factor in [('map_median_m','map_median_m',1.1),('map_p95_m','map_p95_m',1.1),('heldout_translation_p95_m','all75_translation_p95_m',1.5),('heldout_rotation_p95_deg','all75_rotation_p95_deg',1.5)]}
    check(all(abs(decision['limits'][x]-limits[x])<1e-10 for x in limits),'qualification_limits')
    eligible=[]
    for r in independent:
        ratios=[r[x]/limits[x] for x in limits] if r['K']<75 else []
        qualifies=r['K']<75 and r['normal_finite_integrity'] and all(v<=1 for v in ratios)
        saved=q[(q.selector==r['selector'])&(q.K==r['K'])].iloc[0]
        check(bool(saved.qualifies)==qualifies,'qualification_'+r['selector']+str(r['K']))
        if qualifies:r['worst_normalized_primary_metric']=max(ratios);eligible.append(r)
    winner=min(eligible,key=lambda r:(r['K'],r['worst_normalized_primary_metric'],r['batch_runtime_ms'],[SEL[3],SEL[2],SEL[1],SEL[0]].index(r['selector']))) if eligible else None
    check((winner is None)==(decision['selected'] is None),'winner_existence')
    if winner:check(winner['selector']==decision['selected']['selector'] and winner['K']==decision['selected']['K'] and decision['selected_loop_ids']==sets[winner['selector']][str(winner['K'])],'winner_exact')
    check(decision['status']==final['status'] and decision['selected']==final['selected'] and decision['selected_loop_ids']==final['selected_loop_ids'],'selection_not_changed_after_incremental')
    # Full float64 maps, not rounded PLY; no regenerated optimization.
    map_manifest=c('map_artifact_manifest.csv');mapstats={}
    for tag,entries in map_manifest.groupby('condition'):
        clouds={}
        for r in entries.itertuples():
            check(d.sha256(r.path)==r.sha256,'map_hash_'+tag+r.robot_id);clouds[r.robot_id]=np.load(r.path)['xyz']
        mm=a.nn(clouds['robot1'],clouds['robot3']);saved=maps[maps.source_condition.eq(tag)].iloc[0]
        check(abs(mm['median_m']-saved.median_m)<1e-8 and abs(mm['p95_m']-saved.p95_m)<1e-8,'map_NN_'+tag)
        mapstats[tag]=mm
    # Exact historical implementation versus cached mapper at two controls.
    for K in [1,75]:
        row=solver[(solver.selector==SEL[0])&(solver.K==K)].iloc[0];tr=pd.read_csv(row.trajectory_path,usecols=d.POSE_COLUMNS)
        clouds,_=d.s10.map_points(tr,frames['robot1'],frames['robot3']);clouds={r:d.s10.voxel(p) for r,p in clouds.items()}
        mm=a.nn(clouds['robot1'],clouds['robot3']);check(abs(mm['median_m']-mapstats[row.source_condition]['median_m'])<1e-8 and abs(mm['p95_m']-mapstats[row.source_condition]['p95_m'])<1e-8,'actual_historical_map_helper_K'+str(K))
    check(bool(c('prefix_reproduction.csv').PASS.all()) and len(c('prefix_reproduction.csv'))==7,'seven_prefix_reproductions')
    oldmaps=pd.read_csv(A13/'map_consistency_by_K.csv');oldsolver=pd.read_csv(A13/'gtsam_results_by_K.csv')
    for K in KS:
        row=solver[(solver.selector==SEL[0])&(solver.K==K)].iloc[0]
        frozen_pose=d.s10.lookup(pd.read_csv(A13/f'trajectories/K{K}_optimized.csv',usecols=d.POSE_COLUMNS))
        delta=b.pose_difference(trajectories[row.trajectory_path],frozen_pose)
        saved_map=maps[(maps.selector==SEL[0])&(maps.K==K)].iloc[0];expected=oldmaps[oldmaps.K.eq(K)].iloc[0];expected_obj=oldsolver[oldsolver.K.eq(K)].final_error.iloc[0]
        tol=policy['reproduction_tolerances']
        check(delta['translation_p95_m']<=tol['pose_translation_m'] and delta['rotation_p95_deg']<=np.rad2deg(tol['pose_rotation_rad']) and abs(row.final_error-expected_obj)<=tol['graph_absolute'] and all(abs(saved_map[x]-expected[x])<=tol['map_absolute_m'] for x in ['median_m','p95_m']),'independent_prefix_reproduction_K'+str(K))
    events=j('execution_events.json');phases=[x['phase'] for x in events]
    first=phases.index('POLICY_FROZEN');end=phases.index('PRE_GT_DECISION_PERSISTED')
    check(first<min(i for i,v in enumerate(phases) if v=='CONDITION_COMPLETE') and sum(v=='CONDITION_COMPLETE' for v in phases[:end])==28,'policy_then_28_conditions_then_decision')
    check(not any('GT_ACCESS' in x for x in phases) and final['optional_GT']=='NOT_RUN' and not decision['GT_decoded'],'no_GT_decoded')
    check(d.sha256(OUT/'stage15_experiment_policy.json')==events[first]['policy_sha256'],'policy_bytes_unchanged')
    if winner:
        inc=c('incremental_runtime.csv');diag=c('incremental_checkpoint_diagnostic.csv');status=j('incremental_validation_status.json');parameter=j('incremental_parameter_policy.json')
        check(inc.robot1_keyframe.to_list()==list(range(150,2000)),'incremental_original_stream')
        selected=loops[loops.frozen_loop_id.isin(decision['selected_loop_ids'])]
        check(inc[inc.new_loop_count.gt(0)].robot1_keyframe.to_list()==selected.query_keyframe_id.to_list() and status['inserted_ids']==selected.frozen_loop_id.to_list(),'incremental_exact_archived_arrivals')
        check(status['final_factor_equivalence'] and status['nodes']==5796 and status['loops']==winner['K'] and parameter['empty_calls']==0 and parameter['defaults_unchanged'],'incremental_frozen_graph_parameters')
        check(diag.factor_equivalence.all() and (diag.factors==diag.nodes-1+diag.loop_count).all(),'incremental_same_graph_reference')
        check(bool(c('incremental_trajectory_integrity.csv').PASS.all()),'incremental_integrity')
    check(len(list(OUT.glob('[0-9][0-9]_*.png')))==(10 if winner else 8),'figure_count')
    result=dict(PASS=True,checks=checks,conditions=28,unique_subsets=len(trajectories),GT_decoded=False,optimization_run=False,map_recomputed_from_saved_float64=True,actual_historical_mapper_controls_checked=True,external_keyframe_hash_provenance=external,current_source_hashes={str(p.relative_to(ROOT)):d.sha256(p) for p in [ROOT/'src/cumulti/run_stage15_informative_loop_selection.py',Path(__file__),ROOT/'scripts/run_stage15_informative_loop_selection.sh']},policy_sha256=d.sha256(OUT/'stage15_experiment_policy.json'),decision_sha256=d.sha256(OUT/'stage15_pre_gt_decision.json'),performance_or_integrity_PASS_not_implied_by_execution_audit=True)
    (OUT/'independent_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    rotation_details()
    print(json.dumps(dict(PASS=True,checks=len(checks),decision=decision['status'],selected=decision['selected'])))

if __name__=='__main__':
    if sys.argv[1:]==['--rotation-details-only']:rotation_details()
    elif sys.argv[1:]:raise SystemExit('Usage: audit_stage15_informative_loop_selection.py [--rotation-details-only]')
    else:main()
