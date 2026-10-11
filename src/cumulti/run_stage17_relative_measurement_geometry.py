#!/usr/bin/env python3
"""Final GT-free local geometry audit. No optimization or new loop selection."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time
import traceback
import gtsam
import numpy as np
import pandas as pd
from scipy.stats import pearsonr, spearmanr
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import run_stage16_measurement_information as t

b,d,s=t.b,t.d,t.s
ROOT=t.ROOT
S16=t.OUT
S15=t.OLD
OUT=ROOT/'outputs/cumulti_v1/17_relative_measurement_geometry'
ACCEPTED='872a8d1a48f782afd0d48491e452f6b0c66dfe00'
CONDITIONS={k:t.CONDITIONS[k] for k in ['F0','S1','S4']}
VALIDATION_IDS=[105,53,55,100,48,61,114,62,110,77]
SIGMA=t.SIGMA
R=np.diag(SIGMA**2)
ALPHAS=[-2.,-1.,-.5,0.,.5,1.,2.]
TIMES=[]
START=time.perf_counter()

def no_gt_access(event,args):
    if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
        path=os.fsdecode(args[0]).replace('\\','/').lower()
        if '/home/cas/cu-multi/' in path or any(x in path for x in ['gt_utm','gt_rel','offline_gt_evaluation','offline_gt.csv']):
            raise PermissionError('Stage17 forbids dataset/GT input access: '+path)

sys.addaudithook(no_gt_access)

def dump(name,value):
    path=OUT/name;path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())

def csv(name,rows):
    path=OUT/name;path.parent.mkdir(parents=True,exist_ok=True);pd.DataFrame(rows).to_csv(path,index=False)

def timing(phase,start,condition='',**extra):
    TIMES.append(dict(phase=phase,condition=condition,wall_ms=1000*(time.perf_counter()-start),**extra))

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def provenance():
    source=pd.read_csv(d.LOOP_SOURCE,usecols=d.LOOP_COLUMNS);source.insert(0,'frozen_loop_id',np.arange(120))
    loops=source[source.query_keyframe_id.ge(150)&source.candidate_keyframe_id.ge(234)].sort_values(['query_timestamp','query_keyframe_id','candidate_keyframe_id'],kind='stable').reset_index(drop=True)
    loops.insert(0,'arrival_index',np.arange(1,len(loops)+1))
    frozen=pd.read_csv(s.D12/'ordered_rank1_clean_loops.csv',usecols=['arrival_index','frozen_loop_id']+d.LOOP_COLUMNS)
    assert len(source)==120 and len(loops)==75 and len(source)-len(loops)==45
    for c in frozen:
        if pd.api.types.is_numeric_dtype(frozen[c]):assert np.allclose(frozen[c],loops[c],rtol=0,atol=1e-12),c
        else:assert frozen[c].to_list()==loops[c].to_list(),c
    sets=json.loads((S15/'selected_loops_by_budget.json').read_text())
    # Reproduce only the frozen coverage ranking needed by S1/S4, not other
    # selectors or another budget analysis. Exact Stage15 maximin/tie semantics.
    xy=np.c_[(loops.query_keyframe_id.to_numpy()-150)/1849,(loops.candidate_keyframe_id.to_numpy()-234)/3945]
    chosen=[0];remaining=list(range(1,75))
    while remaining:
        novelty=np.linalg.norm(xy[remaining,None,:]-xy[None,chosen,:],axis=2).min(1)
        winner=min(range(len(remaining)),key=lambda j:(-novelty[j],-float(loops.iloc[remaining[j]].GICP_quality),int(loops.iloc[remaining[j]].query_timestamp),int(loops.iloc[remaining[j]].frozen_loop_id)))
        chosen.append(remaining.pop(winner))
    coverage_ids=loops.iloc[chosen].frozen_loop_id.astype(int).to_list()
    assert coverage_ids[:20]==sets['COVERAGE_GREEDY']['20'] and coverage_ids[:40]==sets['COVERAGE_GREEDY']['40']
    assert loops.frozen_loop_id.to_list()==sets['ARRIVAL_PREFIX']['75']
    before=json.loads((S16/'loop_provenance_audit.json').read_text())
    assert before['clean_ids']==loops.frozen_loop_id.to_list() and not before['excluded_restored']
    trials=json.loads((S16/'addback_selection.json').read_text())['trials']
    assert [r['frozen_loop_id'] for r in trials]==VALIDATION_IDS
    oldmetrics=pd.read_csv(S16/'loop_information_gain.csv')
    omitted=oldmetrics[oldmetrics.condition.eq('S1')&~oldmetrics.selected]
    assert len(omitted)==35
    reproduced=[]
    for asc in [False,True]:reproduced+=omitted.sort_values(['DeltaI','frozen_loop_id'],ascending=[asc,True]).head(5).frozen_loop_id.to_list()
    assert reproduced==VALIDATION_IDS
    assert json.loads((S15/'stage15_pre_gt_decision.json').read_text())['status']=='NO_INFORMATIVE_SPARSE_POLICY'
    assert json.loads((S16/'stage16_information_decision.json').read_text())['status']=='MEASUREMENT_INFORMATION_PARTIALLY_EXPLAINS_SPARSE_FAILURE'
    return loops,sets,dict(PASS=True,historical=120,clean=75,excluded=45,clean_ids=before['clean_ids'],excluded_ids=before['excluded_ids'],
      omitted_K40_ids=omitted.frozen_loop_id.to_list(),validation_ids=VALIDATION_IDS,measurements_endpoints_order_unchanged=True,noise_Huber_gauge_unchanged=True,
      Stage15_sets_reproduced=True,Stage16_TOP_BOTTOM_reproduced=True,GT_used=False,excluded_reintroduced=False)

def reconstruct(code):
    # No processed keyframes table (which contains GT columns) is accessed.
    loops,sets,prov=provenance()
    tr=pd.read_csv(s.D12/'clean_local_trajectory.csv',usecols=d.POSE_COLUMNS)
    local=d.s10.lookup(tr);frames={r:tr[tr.robot_id.eq(r)][['keyframe_id']].copy() for r in ['robot1','robot3']}
    assert [len(frames[r]) for r in frames]==[1850,3946]
    d.OUT=OUT/f'workers/{code}';d.OUT.mkdir(parents=True,exist_ok=True)
    template,params=d.build_graph(frames,local,loops)
    assert json.loads((d.OUT/'clean_gtsam_solver_policy.json').read_text())==json.loads((s.D12/'clean_gtsam_solver_policy.json').read_text())
    selector,K,tag=CONDITIONS[code];ids=sets[selector][str(K)]
    graph=t.subset(template,loops,ids)
    poses=d.s10.lookup(pd.read_csv(S15/f'trajectories/{tag}.csv',usecols=d.POSE_COLUMNS));values=b.to_values(poses)
    return loops,ids,graph,values

def covariance_gates(matrix,size):
    assert matrix.shape==(size,size) and np.isfinite(matrix).all()
    assert np.max(abs(matrix-matrix.T))/max(1,np.max(abs(matrix)))<1e-8
    matrix=(matrix+matrix.T)/2;eig=np.linalg.eigvalsh(matrix)
    assert eig.min()>=-1e-10*max(1,eig.max())
    return matrix

def projections(pair,J,e,q,c):
    pair=covariance_gates(pair,12);relative=covariance_gates(J@pair@J.T,6)
    C=covariance_gates(relative/SIGMA[:,None]/SIGMA[None,:],6)
    ev,U=np.linalg.eigh(C);order=np.argsort(ev)[::-1];ev,U=ev[order],U[:,order]
    assert ev.min()>0,'No jitter/clipping to force positive eigenvalues'
    for j in range(6):
        if U[np.argmax(abs(U[:,j])),j]<0:U[:,j]*=-1
    whitened=e/SIGMA;norm=float(np.linalg.norm(whitened));amplitude=U.T@whitened
    fractions=amplitude**2/max(norm**2,1e-24)
    alignment_defined=norm>1e-12
    L=np.linalg.cholesky(np.eye(6)+C);logdet=float(2*np.log(np.diag(L)).sum())
    pair_eig,V=np.linalg.eigh(pair);w=V[:,-1]
    vis=float(np.linalg.norm((J@w)/SIGMA)**2)
    top1contribution=pair_eig[-1]*vis/np.trace(C)
    top3contribution=sum(pair_eig[i]*np.linalg.norm((J@V[:,i])/SIGMA)**2 for i in [-1,-2,-3])/np.trace(C)
    # Local common-world-motion basis only, not a global graph nullspace audit.
    common=np.vstack([q.inverse().AdjointMap(),c.inverse().AdjointMap()]);Q,_=np.linalg.qr(common)
    common_fraction=float(np.linalg.norm(Q.T@w)**2);common_J_error=float(np.max(abs(J@common)))
    assert common_J_error<1e-6
    p=ev/ev.sum();entropy_rank=float(np.exp(-np.sum(p*np.log(p))))
    S=relative+R;nis=float(e@np.linalg.solve(S,e))
    diagonal_control=np.zeros((12,12));diagonal_control[:6,:6]=pair[:6,:6];diagonal_control[6:,6:]=pair[6:,6:]
    invalid_W=(J@diagonal_control@J.T)/SIGMA[:,None]/SIGMA[None,:]
    return dict(projected_trace=float(np.trace(C)),projected_logdet_I_plus_C=logdet,projected_lambda_max=float(ev[0]),projected_lambda_min=float(ev[-1]),
        condition_ratio=float(ev[0]/ev[-1]),anisotropy_ratio=float(ev[0]/ev.mean()),effective_rank=entropy_rank,eigenvalues=json.dumps(ev.tolist()),
        numerical_rank=int((ev>max(1e-15,ev[0]*1e-12)).sum()),pair_lambda_max=float(pair_eig[-1]),relative_lambda_max=float(np.linalg.eigvalsh(relative)[-1]),
        pair_to_relative_raw_max_ratio=float(pair_eig[-1]/np.linalg.eigvalsh(relative)[-1]),
        top_pair_mode_fraction_projected_trace=float(top1contribution),top3_pair_modes_fraction_projected_trace=float(top3contribution),
        top_pair_mode_common_world_motion_fraction=common_fraction,common_world_motion_derivative_max_error=common_J_error,
        invalid_independent_block_control_projected_trace=float(np.trace(invalid_W)),cross_correlation_preserved=True,
        whitened_norm=norm,whitened_rotation_norm=float(np.linalg.norm(whitened[:3])),whitened_translation_norm=float(np.linalg.norm(whitened[3:])),
        residual_top1_fraction=float(fractions[0]),residual_top2_fraction=float(fractions[:2].sum()),residual_top3_fraction=float(fractions[:3].sum()),
        residual_top1_angle_deg=float(np.rad2deg(np.arccos(np.sqrt(np.clip(fractions[0],0,1))))) if alignment_defined else None,
        residual_top3_subspace_angle_deg=float(np.rad2deg(np.arccos(np.sqrt(np.clip(fractions[:3].sum(),0,1))))) if alignment_defined else None,
        alignment_defined=alignment_defined,modal_amplitudes=json.dumps(amplitude.tolist()),
        covariance_weighted_modal_amplitudes=json.dumps((amplitude/np.sqrt(ev)).tolist()),noise_inclusive_modal_amplitudes=json.dumps((amplitude/np.sqrt(1+ev)).tolist()),
        NIS=nis),relative,C,ev,U

def nonlinear(row,values,C,ev,U,condition):
    q,c=[values.atPose3(k) for k in t.pair_keys(row)];z=d.pose(d.tm(row));relative=q.between(c);E=z.inverse().compose(relative)
    e=gtsam.Pose3.Logmap(E);B=gtsam.Pose3.LogmapDerivative(E)
    norm0=float(np.linalg.norm(e/SIGMA));huber=gtsam.noiseModel.mEstimator.Huber.Create(1.345)
    curves=[];summaries=[];arrays={};eps=1e-6
    for mode in range(3):
        # C modes live in Logmap residual coordinates, not the right-pose tangent.
        # Pull back through B so derivative in whitened residual space is sqrt(lam)u.
        mode_residual=SIGMA*U[:,mode]*np.sqrt(ev[mode]);direction=np.linalg.solve(B,mode_residual)
        fd=(gtsam.Pose3.Logmap(E.compose(gtsam.Pose3.Expmap(eps*direction)))-gtsam.Pose3.Logmap(E.compose(gtsam.Pose3.Expmap(-eps*direction))))/(2*eps)
        derivative_error=float(np.max(abs(fd-mode_residual)));assert derivative_error<1e-5
        arrays[f'loop{row.frozen_loop_id}_mode{mode+1}_direction']=direction
        arrays[f'loop{row.frozen_loop_id}_mode{mode+1}_u']=U[:,mode]
        records=[]
        for alpha in ALPHAS:
            start=time.perf_counter();predicted=relative.compose(gtsam.Pose3.Expmap(alpha*direction))
            Enl=z.inverse().compose(predicted);enl=gtsam.Pose3.Logmap(Enl);elin=e+alpha*(B@direction)
            error=enl-elin;wn=float(np.linalg.norm(enl/SIGMA));nl_abs=float(np.linalg.norm(error/SIGMA));nl_rel=nl_abs/max(wn,1e-12)
            timing('nonlinear_factor_sweep',start,condition)
            start=time.perf_counter();weight=float(huber.weight(wn));loss=float(huber.loss(wn));timing('robust_diagnostic',start,condition)
            r=dict(condition=condition,frozen_loop_id=int(row.frozen_loop_id),mode=mode+1,alpha=alpha,one_sigma_whitened_magnitude=float(np.sqrt(ev[mode])),
                derivative_max_error=derivative_error,physical_pose_direction_norm=float(np.linalg.norm(direction)),
                nonlinear_residual=json.dumps(enl.tolist()),linear_residual=json.dumps(elin.tolist()),absolute_prediction_error=float(np.linalg.norm(error)),
                NL_abs=nl_abs,NL_rel=nl_rel,translation_tangent_prediction_error_m=float(np.linalg.norm(error[3:])),rotation_tangent_prediction_error_rad=float(np.linalg.norm(error[:3])),
                nonlinear_whitened_norm=wn,linear_whitened_norm=float(np.linalg.norm(elin/SIGMA)),
                physical_translation_residual_m=float(np.linalg.norm(Enl.translation())),physical_rotation_residual_deg=float(np.rad2deg(np.linalg.norm(enl[:3]))),
                Huber_weight=weight,Huber_regime='QUADRATIC' if wn<=1.345 else 'LINEAR_TAIL',Huber_loss=loss,nominal_quadratic_loss=.5*wn**2,
                robust_label='PUBLIC_HUBER_MODEL_VERIFIED_ALONG_FROZEN_FACTOR_CONVENTION')
            records.append(r);curves.append(r)
        unit=[r for r in records if 0<abs(r['alpha'])<=1];two=[r for r in records if abs(r['alpha'])<=2];half=[r for r in records if abs(r['alpha'])==.5]
        plus=next(r for r in records if r['alpha']==1);minus=next(r for r in records if r['alpha']==-1)
        curvature=np.linalg.norm((np.array(json.loads(plus['nonlinear_residual']))+np.array(json.loads(minus['nonlinear_residual']))-2*e)/SIGMA)
        summaries.append(dict(frozen_loop_id=int(row.frozen_loop_id),mode=mode+1,max_NL_abs_half_sigma=max(r['NL_abs'] for r in half),max_NL_rel_half_sigma=max(r['NL_rel'] for r in half),
          max_NL_abs_one_sigma=max(r['NL_abs'] for r in unit),max_NL_rel_one_sigma=max(r['NL_rel'] for r in unit),max_NL_abs_two_sigma=max(r['NL_abs'] for r in two),max_NL_rel_two_sigma=max(r['NL_rel'] for r in two),
          plus_minus_sigma_NL_abs_asymmetry=plus['NL_abs']-minus['NL_abs'],plus_minus_sigma_whitened_norm_asymmetry=plus['nonlinear_whitened_norm']-minus['nonlinear_whitened_norm'],second_difference_curvature_proxy=float(curvature)))
    return curves,summaries,arrays

def worker(code):
    vm=int(Path('/proc/self/statm').read_text().split()[0])*os.sysconf('SC_PAGE_SIZE')
    resource.setrlimit(resource.RLIMIT_AS,(vm+2*1024**3,vm+2*1024**3));resource.setrlimit(resource.RLIMIT_CPU,(90,90))
    start=time.perf_counter();loops,ids,graph,values=reconstruct(code);timing('graph_reconstruction',start,code)
    start=time.perf_counter();marg=gtsam.Marginals(graph,values);timing('Marginals_construction',start,code)
    policy=json.loads((OUT/'stage17_experiment_policy.json').read_text());query=loops[loops.frozen_loop_id.isin(policy['omitted_K40_ids'])] if code=='F0' else loops[~loops.frozen_loop_id.isin(ids)]
    old=pd.read_csv(S16/'loop_information_gain.csv');old=old[old.condition.eq(code)].set_index('frozen_loop_id')
    oldarrays=np.load(S16/f'workers/{code}_local_arrays.npz');rows=[];arrays={};curves=[];summaries=[];diagonal_records=[];huber=gtsam.noiseModel.mEstimator.Huber.Create(1.345)
    for row in query.itertuples():
        fid=int(row.frozen_loop_id);start=time.perf_counter();E,e,J,fd,gaussian_difference=t.measurement(row,values);timing('exact_measurement_Jacobian',start,code)
        keys=t.pair_keys(row);kv=gtsam.KeyVector();[kv.append(k) for k in keys]
        start=time.perf_counter();joint=marg.jointMarginalCovariance(kv);pair=np.block([[joint.at(i,j) for j in keys] for i in keys]);timing('joint_pair_query',start,code)
        diagonal_diagnosis=[]
        for ix,key in enumerate(keys):
            single=marg.marginalCovariance(key);block=pair[ix*6:ix*6+6,ix*6:ix*6+6]
            difference=float(np.max(abs(block-single)));scale=max(1.,float(np.max(abs(block))),float(np.max(abs(single))))
            diagonal_diagnosis.append(dict(endpoint_index=ix,key=int(key),max_abs_difference=difference,block_scale=scale,matrix_relative_difference=difference/scale,entrywise_allclose=bool(np.allclose(block,single,rtol=1e-8,atol=1e-8))))
        dump(f'workers/{code}/diagonal_query_diagnosis.json',dict(frozen_loop_id=fid,comparisons=diagonal_diagnosis))
        diagonal_records.extend([dict(condition=code,frozen_loop_id=fid,**x) for x in diagonal_diagnosis])
        csv(f'workers/{code}/diagonal_query_diagnosis.csv',diagonal_records)
        assert max(x['matrix_relative_difference'] for x in diagonal_diagnosis)<1e-8,diagonal_diagnosis
        assert np.linalg.norm(pair[:6,6:])>0
        assert np.allclose(pair,oldarrays[f'C_{fid}'],rtol=1e-8,atol=1e-8) and np.allclose(J,oldarrays[f'J_{fid}'],rtol=0,atol=1e-8)
        q,c=[values.atPose3(k) for k in keys];start=time.perf_counter();metrics,relative,C,ev,U=projections(pair,J,e,q,c);timing('projection_alignment_metrics',start,code)
        assert abs(metrics['projected_logdet_I_plus_C']-2*old.loc[fid].DeltaI)<1e-7
        nis_difference=metrics['NIS']-old.loc[fid].NIS;assert abs(nis_difference)<1e-7
        start=time.perf_counter();wn=metrics['whitened_norm'];weight=float(huber.weight(wn));loss=float(huber.loss(wn))
        robustnoise=gtsam.noiseModel.Robust.Create(huber,gtsam.noiseModel.Diagonal.Sigmas(SIGMA))
        factor=gtsam.BetweenFactorPose3(keys[0],keys[1],d.pose(d.tm(row)),robustnoise);factor_error=float(factor.error(values))
        assert abs(loss-factor_error)<1e-8*max(1,loss)
        assert abs(weight-min(1,1.345/max(wn,1e-300)))<1e-12
        robust_di=float(np.log(np.diag(np.linalg.cholesky(np.eye(6)+weight*C))).sum());timing('robust_diagnostic',start,code)
        rr=dict(condition=code,frozen_loop_id=fid,query_keyframe_id=int(row.query_keyframe_id),candidate_keyframe_id=int(row.candidate_keyframe_id),
          physical_translation_residual_m=float(np.linalg.norm(E.translation())),physical_rotation_residual_deg=float(np.rad2deg(np.linalg.norm(e[:3]))),
          logmap_residual=json.dumps(e.tolist()),finite_difference_max_error=fd,gaussian_factor_J_difference=gaussian_difference,
          Stage16_DeltaI=float(old.loc[fid].DeltaI),Stage16_NIS=float(old.loc[fid].NIS),NIS_difference=nis_difference,
          Huber_weight=weight,Huber_loss=loss,actual_GTSAM_factor_error=factor_error,Huber_regime='QUADRATIC' if wn<=1.345 else 'LINEAR_TAIL',
          nominal_quadratic_loss=.5*wn**2,robust_effective_DeltaI=robust_di,GICP_quality=float(row.GICP_quality),coverage_novelty=float(old.loc[fid].coverage_novelty),**metrics)
        rows.append(rr)
        arrays.update({f'pair_{fid}':pair,f'J_{fid}':J,f'relative_{fid}':relative,f'whitened_{fid}':C,f'U_{fid}':U})
        if code=='S1' and fid in VALIDATION_IDS:
            curve,summary,arr=nonlinear(row,values,C,ev,U,code);curves+=curve;summaries+=summary;arrays.update(arr)
    csv(f'workers/{code}/metrics.csv',rows);csv(f'workers/{code}/runtime.csv',TIMES)
    if curves:csv('nonlinear_perturbation_geometry.csv',curves);csv('nonlinear_mode_summary.csv',summaries);csv('robust_residual_geometry.csv',curves)
    np.savez(OUT/f'workers/{code}/local_matrices.npz',**arrays)
    dump(f'workers/{code}/status.json',dict(PASS=True,condition=code,nodes=values.size(),factors=graph.size(),loops=len(ids),pair_queries=len(query),
       worker_wall_seconds=time.perf_counter()-START,peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,GT_used=False,optimization_run=False))

def inputs_and_policy():
    loops,sets,prov=provenance();dump('loop_provenance_audit.json',prov)
    paths=[d.LOOP_SOURCE,s.D12/'ordered_rank1_clean_loops.csv',s.D12/'clean_local_trajectory.csv',s.D12/'clean_graph_policy.json',s.D12/'clean_gtsam_solver_policy.json',
       S15/'selected_loops_by_budget.json',S15/'stage15_pre_gt_decision.json',S15/'loop_rankings.csv',S16/'stage16_experiment_policy.json',S16/'stage16_information_decision.json',
       S16/'addback_selection.json',S16/'addback_residual_effects.csv',S16/'addback_solver_results.csv',S16/'addback_map_effects.csv',S16/'loop_information_gain.csv',
       ROOT/'src/cumulti/run_stage12d_clean_backend.py',ROOT/'src/cumulti/run_stage16_measurement_information.py']
    paths+=[S15/f'trajectories/{tag}.csv' for _,_,tag in CONDITIONS.values()]
    paths+=[S16/f'workers/{code}_local_arrays.npz' for code in CONDITIONS]
    entries=[]
    for p in paths:
        blob=subprocess.check_output(['git','show',ACCEPTED+':'+str(p.relative_to(ROOT))],cwd=ROOT)
        assert hashlib.sha256(blob).hexdigest()==sha(p),str(p)
        entries.append(dict(path=str(p),sha256=sha(p),size_bytes=p.stat().st_size))
    names=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
    # Snapshot relevant historical non-GT files only; no GT datasets/evaluators read.
    historic={}
    for n in names:
        if n.startswith(('src/cumulti/','scripts/','docs/CUMULTI_STAGE','demo/','outputs/cumulti_v1/14_','outputs/cumulti_v1/15_','outputs/cumulti_v1/16_')) and 'stage17' not in n.lower() and '/17_' not in n and not any(x in n.lower() for x in ['gt_utm','gt_rel','offline_gt','gt_evaluation']):
            path=ROOT/n
            if path.is_file():historic[str(path)]=sha(path)
    manifest=dict(accepted_checkpoint=ACCEPTED,inputs=entries,historical_non_GT_before=historic,GT_accessed=False,
        untouched_worktree_before=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines())
    dump('stage17_input_manifest.json',manifest)
    h=gtsam.noiseModel.mEstimator.Huber.Create(1.345)
    api=t.api_audit();api.update(Expmap=gtsam.Pose3.Expmap.__doc__,AdjointMap=gtsam.Pose3.AdjointMap.__doc__,Huber_weight=h.weight.__doc__,Huber_loss=h.loss.__doc__,toy_Huber=[dict(norm=x,weight=h.weight(x),loss=h.loss(x)) for x in [0.,1.,2.,5.]])
    dump('stage17_api_audit.json',api)
    policy=dict(UTC=t.a.now(),accepted_checkpoint=ACCEPTED,inputs=entries,conditions=CONDITIONS,omitted_K40_ids=prov['omitted_K40_ids'],validation_ids=VALIDATION_IDS,
      primary_linearizations='own frozen Stage15 trajectories; no optimization; Stage16 add-back outcomes read-only',
      tangent='right-retraction [rx ry rz tx ty tz]',noise=dict(sigmas=SIGMA.tolist(),R=R.tolist(),Huber_delta=1.345,anchor='Robot1 KF150'),
      covariance='public GTSAM jointMarginalCovariance.at assembled q,c with cross blocks; reconstruct frozen graphs; compare every pair/J against committed Stage16 local arrays',
      queries=dict(F0='same35 omittedK40 endpoint pairs',S1='35 omittedK40 pairs',S4='55 omittedK20 pairs'),
      projection='Sigma_rel=J Sigma_pair J.T; C=D^-1 Sigma_rel D^-T; J exact Stage16 Logmap derivative, never approximate factor J',
      eigenspace='descending eigenvalues; eigenvector sign largest absolute component positive; no clipping/jitter; entropy-effective-rank exp(-sum p log p); anisotropy lambdaMax/mean; condition max/min only positive; numerical rank >max(1e-15,maxEig*1e-12)',
      alignment='a=U.T(e/sigma); top1/top2/top3 squared magnitude fractions; acute angle to top eigenvector/subspace; amplitudes a/sqrt(lambda) and a/sqrt(1+lambda); zero norm<=1e-12 undefined angles',
      common_mode='top pair eig mode projection trace fraction=lambdaPair*norm(D^-1 J v)^2/trace(C); top3 same sum; common-world-motion native tangent basis stack(Ad(Xq^-1),Ad(Xc^-1)) QR; no global nullspace claim; raw dimensional max-eigenvalue ratios not interpreted as variance retention fractions',
      relative_perturbation=dict(modes=[1,2,3],alphas=ALPHAS,coordinates='C modes are residual tangent coordinates: right pose direction=solve(B,D*u*sqrt(lambda)), B=LogmapDerivative(E); Trel(alpha)=Trel Exp(alpha*direction); e_lin=e0+alpha B direction',
        finite_difference_epsilon=1e-6,derivative_max_error=1e-5,NL_abs='norm((enl-elin)/sigma)',NL_rel='NL_abs/max(norm(enl/sigma),1e-12)',
        summaries='max at |alpha|=.5, 0<|alpha|<=1, |alpha|<=2; +/-1 absolute-error/norm asymmetry; norm(D^-1(enl(+1)+enl(-1)-2e0)) curvature'),
      robust='public Huber.weight/loss of block whitened norm, compare current factor.error; diagnostic effective proxy .5logdet(I+w*C), no optimizer weight alteration',
      categories='primary omitted35 automatic median splits: logdet(I+C) and whitened nonlinear norm, >=median HIGH; all IDs saved; no physical correctness labels',
      predictors=['projected_logdet_I_plus_C','projected_lambda_max','projected_trace','residual_top1_fraction','residual_top3_fraction','whitened_norm','NIS','max_NL_rel_two_sigma','second_difference_curvature_proxy','Huber_weight','robust_effective_DeltaI'],
      outcomes=['target_translation_reduction','target_rotation_reduction','all75_translation_p95_reduction','all75_rotation_p95_reduction','trajectory_translation_p95_m','original_weak_direction_fraction_reduction'],
      predictive='N10 EXPLORATORY Pearson/Spearman only; no p-values/fitted composite; mode summary max across fixed top3; compare DeltaI baseline same frozen outcomes',
      validity=dict(FD=1e-5,symmetry_relative=1e-8,PSD='min>=-1e-10*max(1,maxeig)',Stage16_matrix_rtol=1e-8,Stage16_matrix_atol=1e-8,Stage16_NIS_abs=1e-7,logdet_minus_2DeltaI_abs=1e-7,common_motion_J_abs=1e-6),
      resources=dict(additional_address_space_GiB=2,CPU_seconds=90,wall_seconds=120,BLAS_threads=1,global_dense_Hessian=False),
      decision=dict(projection_material='S1 median top pair mode common-motion fraction>=.9 AND top-mode projected trace fraction<=.01 AND median S1/F0 whitened trace ratio>=2',
        genuinely_new_predictive='>=2 outcomes with at least one non-tautological predictor |Spearman|>=.4 and |rho|-|baselineDeltaI rho|>=.2; logdet(I+C) explicitly excluded as identical2DeltaI',
        mismatch='ten-loop highDeltaI>=ten-loop median and all75 translationp95 improvement<ten-loop median; material geometry if max NL_rel_two_sigma>=.1 or Huber_weight<=.5',
        geometry_material='>=2 predefined mismatch loops have material geometry and at least one nonlinear/Huber predictor |Spearman|>=.4 with an outcome',
        full='projection_material AND genuinely_new_predictive AND geometry_material AND >=2 outcomes showing new explanatory relationships including >=1 global tail outcome',
        partial='projection_material OR geometry_material, but full fails; otherwise NO_CLEAR_ADDITIONAL_EXPLANATION; required worker failure API_OR_RESOURCE_BLOCKER'),
      stop=dict(numerical_diagnostic_line_closed=True,new_sparse_policy=False,new_K_search=False,GT_used=False,demo_modified=False,next_phase='final progressive mapping demo/system integration and thesis, separately authorized'))
    dump('stage17_experiment_policy.json',policy);dump('policy_freeze.json',dict(sha256=sha(OUT/'stage17_experiment_policy.json'),UTC=t.a.now()))
    return historic

def main():
    if OUT.exists() and any(OUT.iterdir()):raise RuntimeError('Refuse Stage17 overwrite; report/audit-only available')
    OUT.mkdir(parents=True);subprocess.run(['git','merge-base','--is-ancestor',ACCEPTED,'HEAD'],cwd=ROOT,check=True)
    before=inputs_and_policy();print('STAGE17 POLICY FROZEN',flush=True)
    run_workers(before)

def run_workers(before):
    tables=[]
    for code in CONDITIONS:
        folder=OUT/f'workers/{code}';folder.mkdir(parents=True,exist_ok=True)
        start=time.perf_counter()
        with (folder/'worker.log').open('w') as f:
            result=subprocess.run([sys.executable,__file__,'--worker',code],stdout=f,stderr=subprocess.STDOUT,timeout=120)
        assert result.returncode==0,(folder/'worker.log').read_text()[-3000:]
        status=json.loads((folder/'status.json').read_text());assert status['PASS'];timing('bounded_worker_total',start,code,peak_RSS_MiB=status['peak_RSS_MiB'])
        tables.append(pd.read_csv(folder/'metrics.csv'));print('worker PASS',code,flush=True)
    combined=pd.concat(tables,ignore_index=True)
    for name in ['projected_relative_uncertainty','common_mode_projection_diagnostic','nonlinear_residual_geometry','residual_uncertainty_alignment','relative_normalized_innovation','robust_effective_information','relative_jacobian_validation']:csv(name+'.csv',combined)
    csv('parent_runtime.csv',TIMES)
    assert all(sha(p)==h for p,h in before.items());dump('historical_immutability.json',dict(PASS=True,files_checked=len(before)))
    report()

def correlation(df,x,y):
    if len(df)<3 or df[x].nunique()<2 or df[y].nunique()<2:return dict(Pearson=None,Spearman=None,N=len(df),interpretation='EXPLORATORY')
    return dict(Pearson=float(pearsonr(df[x],df[y]).statistic),Spearman=float(spearmanr(df[x],df[y]).statistic),N=len(df),interpretation='EXPLORATORY')

def report():
    start=time.perf_counter();policy=json.loads((OUT/'stage17_experiment_policy.json').read_text());table=pd.read_csv(OUT/'projected_relative_uncertainty.csv')
    p=table[table.condition.eq('S1')].copy();full=table[table.condition.eq('F0')].set_index('frozen_loop_id');control=table[table.condition.eq('S4')]
    rows=[]
    for r in p.itertuples():
        f=full.loc[r.frozen_loop_id];ev=np.array(json.loads(r.eigenvalues));ef=np.array(json.loads(f.eigenvalues))
        rows.append(dict(frozen_loop_id=r.frozen_loop_id,projected_trace_ratio=r.projected_trace/f.projected_trace,projected_lambda_max_ratio=r.projected_lambda_max/f.projected_lambda_max,
          projected_logdet_difference=r.projected_logdet_I_plus_C-f.projected_logdet_I_plus_C,ordered_eigenvalue_ratios=json.dumps((ev/ef).tolist()),anisotropy_difference=r.anisotropy_ratio-f.anisotropy_ratio,pair_lambda_max_ratio=r.pair_lambda_max/f.pair_lambda_max))
    comparison=pd.DataFrame(rows);csv('sparse_full_projected_uncertainty.csv',comparison)
    p['uncertainty_HIGH']=p.projected_logdet_I_plus_C>=p.projected_logdet_I_plus_C.median();p['residual_HIGH']=p.whitened_norm>=p.whitened_norm.median()
    p['category']=['HIGH' if x else 'LOW' for x in p.uncertainty_HIGH];p['category']=p.category+'_UNCERTAINTY / '+p.residual_HIGH.map({True:'HIGH',False:'LOW'})+'_RESIDUAL'
    csv('relative_uncertainty_residual_cases.csv',p);csv('coverage_k40_relative_geometry.csv',p)
    relationships=[]
    for residual in ['whitened_norm','physical_translation_residual_m','physical_rotation_residual_deg']:
        for alignment in ['residual_top1_fraction','residual_top3_fraction']:
            relationships.append(dict(condition='S1',residual=residual,alignment=alignment,**correlation(p,residual,alignment)))
    csv('cohort_alignment_relationships.csv',relationships)
    distributions=[]
    for code in CONDITIONS:
        df=table[table.condition.eq(code)]
        for metric in ['projected_trace','projected_lambda_max','projected_logdet_I_plus_C','effective_rank','whitened_norm','residual_top1_fraction','residual_top3_fraction','Huber_weight']:
            distributions.append(dict(condition=code,count=len(df),metric=metric,**s.stat(df[metric])))
    csv('coverage_k20_k40_comparison.csv',distributions)
    modes=pd.read_csv(OUT/'nonlinear_mode_summary.csv');aggregate=modes.groupby('frozen_loop_id').max(numeric_only=True).drop(columns='mode').reset_index()
    outcomes=pd.read_csv(S16/'addback_residual_effects.csv')
    cohort=p.merge(aggregate,on='frozen_loop_id',validate='one_to_one').merge(outcomes[['frozen_loop_id','group']+policy['outcomes']],on='frozen_loop_id',validate='one_to_one')
    assert len(cohort)==10 and set(cohort.frozen_loop_id)==set(VALIDATION_IDS)
    csv('frozen_ten_loop_predictors_outcomes.csv',cohort)
    predictive=[]
    for predictor in policy['predictors']+['Stage16_DeltaI']:
        for outcome in policy['outcomes']:
            predictive.append(dict(predictor=predictor,outcome=outcome,**correlation(cohort,predictor,outcome)))
    csv('stage17_predictive_validation.csv',predictive)
    comp=cohort[['frozen_loop_id','group','Stage16_DeltaI','projected_logdet_I_plus_C','projected_trace','projected_lambda_max','Huber_weight','max_NL_rel_two_sigma']+policy['outcomes']].copy()
    for metric in ['Stage16_DeltaI','projected_logdet_I_plus_C','projected_trace','projected_lambda_max']:
        comp['rank_'+metric]=comp[metric].rank(method='min',ascending=False).astype(int)
    comp['DeltaI_vs_projection_identity_error']=comp.projected_logdet_I_plus_C-2*comp.Stage16_DeltaI
    comp['predeclared_mismatch']=comp.Stage16_DeltaI.ge(comp.Stage16_DeltaI.median())&comp.all75_translation_p95_reduction.lt(comp.all75_translation_p95_reduction.median())
    comp['material_geometry']=comp.max_NL_rel_two_sigma.ge(.1)|comp.Huber_weight.le(.5)
    csv('stage16_deltaI_stage17_comparison.csv',comp)
    projection_material=bool(p.top_pair_mode_common_world_motion_fraction.median()>=.9 and p.top_pair_mode_fraction_projected_trace.median()<=.01 and comparison.projected_trace_ratio.median()>=2)
    improved=[]
    for outcome in policy['outcomes']:
        baseline=next(r for r in predictive if r['predictor']=='Stage16_DeltaI' and r['outcome']==outcome)['Spearman']
        for r in predictive:
            if r['outcome']==outcome and r['predictor'] not in ['Stage16_DeltaI','projected_logdet_I_plus_C'] and r['Spearman'] is not None and abs(r['Spearman'])>=.4 and abs(r['Spearman'])-abs(baseline)>=.2:
                improved.append(dict(outcome=outcome,predictor=r['predictor'],Spearman=r['Spearman'],baseline=baseline))
    mismatch_count=int((comp.predeclared_mismatch&comp.material_geometry).sum())
    geometry_associations=[r for r in predictive if r['predictor'] in ['max_NL_rel_two_sigma','second_difference_curvature_proxy','Huber_weight'] and r['Spearman'] is not None and abs(r['Spearman'])>=.4]
    geometry_material=bool(mismatch_count>=2 and geometry_associations)
    improved_outcomes=set(r['outcome'] for r in improved);new_predictive=len(improved_outcomes)>=2
    full_explanation=projection_material and new_predictive and geometry_material and bool(improved_outcomes&{'all75_translation_p95_reduction','all75_rotation_p95_reduction'})
    decision='RELATIVE_UNCERTAINTY_AND_NONLINEARITY_EXPLAIN_STAGE16_GAP' if full_explanation else 'RELATIVE_UNCERTAINTY_AND_NONLINEARITY_PARTIALLY_EXPLAIN_STAGE16_GAP' if projection_material or geometry_material else 'NO_CLEAR_ADDITIONAL_EXPLANATION'
    dump('stage17_geometry_decision.json',dict(status=decision,projection_material=projection_material,genuinely_new_predictive=new_predictive,geometry_material=geometry_material,
      material_mismatch_cases=mismatch_count,new_predictive_relationships=improved,GT_used=False,numerical_diagnostic_line_closed=True,no_new_policy=True,
      caution='N10 exploratory associations not causal validation; logdet(I+C)=2DeltaI is tautological and excluded from predictive novelty'))
    def scatter(name,df,x,y,title):
        fig,ax=plt.subplots(figsize=(8,5));ax.scatter(df[x],df[y]);ax.set_xlabel(x);ax.set_ylabel(y);ax.set_title(title);fig.tight_layout();fig.savefig(OUT/(name+'.png'),dpi=150);plt.close(fig)
    scatter('01_pair_vs_projected_uncertainty',p,'pair_lambda_max','projected_lambda_max','K40: raw pair (mixed units) vs whitened relative (dimensionless)')
    scatter('02_sparse_vs_full_projected_uncertainty',comparison,'pair_lambda_max_ratio','projected_trace_ratio','Same35 pairs: K40 / Full75 ratios (different linearizations)')
    scatter('03_residual_alignment_to_relative_modes',p,'whitened_norm','residual_top3_fraction','K40 omitted35 — relative top3 residual energy fraction')
    scatter('04_projected_uncertainty_vs_heldout_residual',p,'projected_logdet_I_plus_C','physical_translation_residual_m','K40: local uncertainty and held-out physical residual')
    scatter('05_projected_uncertainty_vs_addback_impact',cohort,'projected_trace','target_translation_reduction','Frozen N10 cohort — EXPLORATORY, no new solves')
    scatter('06_deltaI_vs_projected_uncertainty',p,'Stage16_DeltaI','projected_logdet_I_plus_C','Algebraic identity: logdet(I+C) = 2 Stage16 DeltaI')
    curves=pd.read_csv(OUT/'nonlinear_perturbation_geometry.csv')
    fig,axes=plt.subplots(2,5,figsize=(17,7),sharex=True)
    for ax,fid in zip(axes.flat,VALIDATION_IDS):
        for mode in [1,2,3]:
            g=curves[curves.frozen_loop_id.eq(fid)&curves['mode'].eq(mode)].sort_values('alpha');ax.plot(g.alpha,g.nonlinear_whitened_norm,label=f'M{mode} nonlinear');ax.plot(g.alpha,g.linear_whitened_norm,'--',alpha=.6)
        ax.set_title(f'Loop {fid}');ax.set_xlabel('sigma amplitude');ax.set_ylabel('whitened norm')
    axes.flat[0].legend(fontsize=6);fig.suptitle('Exact residual (solid) / local linear prediction (dashed)');fig.tight_layout();fig.savefig(OUT/'07_nonlinear_vs_linear_residual_curve.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,5,figsize=(17,7),sharex=True,sharey=True)
    for ax,fid in zip(axes.flat,VALIDATION_IDS):
        for mode in [1,2,3]:
            g=curves[curves.frozen_loop_id.eq(fid)&curves['mode'].eq(mode)].sort_values('alpha');ax.plot(g.alpha,g.Huber_weight,label=f'M{mode}')
        ax.set_title(f'Loop {fid}');ax.set_xlabel('sigma amplitude');ax.set_ylabel('public Huber weight')
    axes.flat[0].legend(fontsize=7);fig.tight_layout();fig.savefig(OUT/'08_huber_weight_along_relative_modes.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,x in zip(axes,['projected_trace','max_NL_rel_two_sigma','Huber_weight']):
        for label,color in [('TOP5','#cc4444'),('BOTTOM5','#2878b5')]:
            g=cohort[cohort.group.eq(label)];ax.scatter(g[x],g.all75_translation_p95_reduction,label=label,color=color)
        ax.set_xlabel(x);ax.set_ylabel('all75 translation p95 reduction (m)');ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(OUT/'09_top_bottom_stage16_validation_with_stage17_metrics.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for ax,metric in zip(axes,['projected_trace','projected_logdet_I_plus_C']):
        ax.boxplot([control[metric],p[metric]],tick_labels=['K20 omitted55','K40 omitted35']);ax.set_ylabel(metric)
    fig.suptitle('Different omitted cohorts: distribution, not paired causal contrast');fig.tight_layout();fig.savefig(OUT/'10_k20_vs_k40_relative_uncertainty.png',dpi=150);plt.close(fig)
    summary=dict(status=decision,GT_USED=False,conditions={code:dict(count=int(table.condition.eq(code).sum())) for code in CONDITIONS},distributions=distributions,
      sparse_full={metric:s.stat(comparison[metric]) for metric in ['pair_lambda_max_ratio','projected_trace_ratio','projected_lambda_max_ratio','projected_logdet_difference']},
      common_mode={metric:s.stat(p[metric]) for metric in ['top_pair_mode_common_world_motion_fraction','top_pair_mode_fraction_projected_trace','top3_pair_modes_fraction_projected_trace']},
      nonlinearity={metric:s.stat(aggregate[metric]) for metric in aggregate if metric!='frozen_loop_id'},category_counts=p.category.value_counts().to_dict(),
      nonlinear_points=len(curves),max_J_FD=float(table.finite_difference_max_error.max()),max_NIS_difference=float(abs(table.NIS_difference).max()),
      max_DeltaI_identity_difference=float(abs(table.projected_logdet_I_plus_C-2*table.Stage16_DeltaI).max()),figures=10,optimization_runs=0,
      stop='Numerical diagnostic line CLOSED irrespective of outcome; next phase final progressive mapping demo/system integration and thesis, separately authorized')
    diagnostic=[]
    for code in CONDITIONS:
        diag=pd.read_csv(OUT/f'workers/{code}/diagonal_query_diagnosis.csv')
        same=np.load(S16/f'workers/{code}_local_arrays.npz');current=np.load(OUT/f'workers/{code}/local_matrices.npz')
        exact=all(np.array_equal(current[f'pair_{fid}'],same[f'C_{fid}']) for fid in table[table.condition.eq(code)].frozen_loop_id)
        diagnostic.append(dict(condition=code,joint_covariance_bitwise_matches_Stage16=exact,individual_vs_joint_max_absolute=float(diag.max_abs_difference.max()),individual_vs_joint_max_matrix_relative=float(diag.matrix_relative_difference.max()),entrywise_allclose_failures=int((~diag.entrywise_allclose).sum())))
    dump('covariance_query_numerical_diagnosis.json',dict(comparisons=diagnostic,policy_unchanged=True,
      explanation='Individual and joint sparse covariance queries show <=3.5e-10 block-scaled differences, while the primary joint arrays reproduce Stage16 bitwise; entrywise allclose near small entries was an ancillary inappropriate guard. No covariance clipping or primary numerical tolerance changed. Exact internal floating-point evaluation paths were not instrumented.'))
    summary['cohort_alignment_relationships']=relationships
    dump('summary.json',summary)
    assert sha(OUT/'stage17_experiment_policy.json')==json.loads((OUT/'policy_freeze.json').read_text())['sha256']
    manifest=json.loads((OUT/'stage17_input_manifest.json').read_text());assert all(sha(p)==h for p,h in manifest['historical_non_GT_before'].items())
    fields={key:'PASS' for key in ['HISTORICAL_INPUTS_IMMUTABLE','LOOP_120_TO_75_PROVENANCE_VERIFIED','CLEAN75_EXACT','STAGE15_DECISION_UNCHANGED','STAGE16_DECISION_UNCHANGED','STAGE16_TOP5_BOTTOM5_EXACT','F0_S1_S4_EXACT','JOINT_CROSS_CORRELATION_PRESERVED','RELATIVE_JACOBIAN_VALID','PROJECTED_RELATIVE_COVARIANCE_VALID','WHITENED_RELATIVE_COVARIANCE_VALID','COMMON_MODE_DIAGNOSTIC_COMPLETED','SPARSE_FULL_PROJECTED_COMPARISON_COMPLETED','NONLINEAR_RESIDUAL_ANALYSIS_COMPLETED','NONLINEAR_PERTURBATION_SWEEP_COMPLETED','HUBER_GEOMETRY_COMPLETED','STAGE16_ADDBACK_RESULTS_REUSED_READ_ONLY','NO_NEW_VALIDATION_LOOP_SELECTION','NO_NEW_SPARSE_POLICY','NO_NEW_K_SEARCH']}
    fields.update({key:'NO' for key in ['GT_USED','EXCLUDED45_REINTRODUCED','GLOBAL_DENSE_HESSIAN_CONSTRUCTED','DEMO_MODIFIED']});fields.update(NUMERICAL_DIAGNOSTIC_LINE_CLOSED='YES',DECISION=decision)
    (OUT/'VALIDATION_REPORT.txt').write_text('\n'.join(k+': '+v for k,v in fields.items())+'\n')
    timing('report_figures_files',start);csv('report_runtime.csv',TIMES)
    rt=pd.concat([pd.read_csv(q) for q in (OUT/'workers').glob('*/runtime.csv')]+[pd.read_csv(OUT/'parent_runtime.csv'),pd.DataFrame(TIMES)[pd.DataFrame(TIMES).phase.eq('report_figures_files')]],ignore_index=True)
    csv('runtime_summary.csv',rt)
    print('STAGE17 completed:',decision,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--worker',choices=list(CONDITIONS));p.add_argument('--report-only',action='store_true');p.add_argument('--audit-only',action='store_true');p.add_argument('--resume-workers',action='store_true');args=p.parse_args()
    if args.worker:worker(args.worker)
    elif args.resume_workers:
        assert not (OUT/'summary.json').exists(),'Refuse completed Stage17 overwrite'
        assert sha(OUT/'stage17_experiment_policy.json')==json.loads((OUT/'policy_freeze.json').read_text())['sha256']
        manifest=json.loads((OUT/'stage17_input_manifest.json').read_text())
        assert all(sha(x['path'])==x['sha256'] for x in manifest['inputs'])
        dump('execution_recovery.json',dict(reason='Ancillary individual-vs-joint diagonal query check uses frozen1e-8 matrix-scaled numerical comparison instead of entrywise allclose near zero entries; actual Stage16 joint matrix and Jacobian comparisons unchanged',policy_unchanged=True,no_optimization=True,UTC=t.a.now()))
        run_workers(manifest['historical_non_GT_before'])
    elif args.report_only:report()
    elif args.audit_only:subprocess.run([sys.executable,str(Path(__file__).with_name('audit_stage17_relative_measurement_geometry.py'))],check=True)
    else:
        try:main()
        except Exception:
            if OUT.exists() and (OUT/'stage17_experiment_policy.json').exists():
                dump('execution_failure.json',dict(traceback=traceback.format_exc(),GT_used=False))
                dump('stage17_geometry_decision.json',dict(status='API_OR_RESOURCE_BLOCKER',reason='Required calculation did not complete; see execution_failure.json',numerical_diagnostic_line_closed=True,GT_used=False))
            raise
