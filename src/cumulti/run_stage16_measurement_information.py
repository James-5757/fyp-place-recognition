#!/usr/bin/env python3
"""GT-free local measurement-information audit; all historical inputs read-only."""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata
import itertools
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
from scipy.spatial.transform import Rotation
from scipy.stats import pearsonr, spearmanr
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import run_stage15_informative_loop_selection as s

b,a,d=s.b,s.a,s.d
ROOT=s.ROOT
OLD=s.OUT
OUT=ROOT/'outputs/cumulti_v1/16_measurement_information'
ACCEPTED='cc32583794e3e076d19582311bb1bd53ae5cef74'
CONDITIONS={'F0':('ARRIVAL_PREFIX',75,'P0_K75'), 'S1':('COVERAGE_GREEDY',40,'P2_K40'),
 'S2':('QUALITY_GREEDY',40,'P1_K40'), 'S3':('QUALITY_COVERAGE_GREEDY',40,'P3_K40'),
 'S4':('COVERAGE_GREEDY',20,'P2_K20')}
SIGMA=np.array([np.deg2rad(5)]*3+[1.]*3)
R=np.diag(SIGMA**2)
TIMES=[]

def dump(name,obj):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('w') as f: json.dump(obj,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())

def csv(name,rows):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    pd.DataFrame(rows).to_csv(p,index=False)

def timed(phase,start,condition='',**kw):
    TIMES.append(dict(phase=phase,condition=condition,wall_ms=1000*(time.perf_counter()-start),**kw))

def pair_keys(row):return [d.key(1,int(row.query_keyframe_id)),d.key(3,int(row.candidate_keyframe_id))]

def measurement(row,values):
    kq,kc=pair_keys(row);q,c=values.atPose3(kq),values.atPose3(kc);z=d.pose(d.tm(row))
    hq=np.zeros((6,6),order='F');hc=hq.copy(order='F')
    between=q.between(c,hq,hc);E=z.inverse().compose(between)
    e=gtsam.Pose3.Logmap(E);J=gtsam.Pose3.LogmapDerivative(E)@np.hstack([hq,hc])
    # Audit the actual Gaussian-factor API separately: default BetweenFactor's
    # approximate local chart derivative need not equal the exact Logmap derivative.
    unit=gtsam.BetweenFactorPose3(kq,kc,z,gtsam.noiseModel.Unit.Create(6))
    gaussian=unit.linearize(values);A,bb=gaussian.jacobian()
    keys=list(gaussian.keys());A=np.hstack([A[:,6*keys.index(k):6*keys.index(k)+6] for k in [kq,kc]])
    assert np.allclose(unit.unwhitenedError(values),e,rtol=0,atol=1e-8)
    eps=1e-6;fd=np.empty((6,12))
    for j in range(12):
        delta=np.zeros(6);delta[j%6]=eps
        plus=[q,c];minus=[q,c];plus[j//6]=plus[j//6].retract(delta);minus[j//6]=minus[j//6].retract(-delta)
        ep=gtsam.Pose3.Logmap(z.inverse().compose(plus[0].between(plus[1])))
        em=gtsam.Pose3.Logmap(z.inverse().compose(minus[0].between(minus[1])))
        fd[:,j]=(ep-em)/(2*eps)
    diff=float(abs(J-fd).max());assert diff<1e-5,(int(row.frozen_loop_id),diff)
    assert np.isfinite(J).all() and np.isfinite(e).all() and J.shape==(6,12)
    return E,e,J,diff,float(abs(A-J).max())

def api_audit():
    docs={}
    for cls,names in [(gtsam.Marginals,['jointMarginalCovariance','marginalCovariance']),
      (gtsam.JointMarginal,['at','fullMatrix']),(gtsam.JacobianFactor,['jacobian','keys']),
      (gtsam.BetweenFactorPose3,['linearize','unwhitenedError']),
      (gtsam.Pose3,['between','Logmap','LogmapDerivative'])]:
        for n in names:docs[cls.__name__+'.'+n]=getattr(cls,n).__doc__
    v=gtsam.Values();v.insert(1,gtsam.Pose3());v.insert(2,gtsam.Pose3(gtsam.Rot3.Rz(.2),np.array([1.,2.,0.])))
    graph=gtsam.NonlinearFactorGraph();noise=gtsam.noiseModel.Isotropic.Sigma(6,1.)
    graph.add(gtsam.PriorFactorPose3(1,v.atPose3(1),noise));graph.add(gtsam.BetweenFactorPose3(1,2,v.atPose3(1).between(v.atPose3(2)),noise))
    marg=gtsam.Marginals(graph,v);kv=gtsam.KeyVector();kv.append(1);kv.append(2)
    joint=marg.jointMarginalCovariance(kv)
    C=np.block([[joint.at(i,j) for j in [1,2]] for i in [1,2]])
    assert C.shape==(12,12) and np.isfinite(C).all() and np.linalg.norm(C[:6,6:])>0
    assert np.allclose(C,joint.fullMatrix()) and np.allclose(C[:6,:6],marg.marginalCovariance(1))
    tangent=gtsam.Pose3.Logmap(gtsam.Pose3(gtsam.Rot3.Rz(.3),np.zeros(3)))
    assert np.isfinite(tangent).all() and np.allclose(tangent,[0,0,.3,0,0,0])
    robust=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),gtsam.noiseModel.Diagonal.Sigmas(SIGMA))
    graph.add(gtsam.BetweenFactorPose3(1,2,v.atPose3(1).between(v.atPose3(2)),robust));assert np.isfinite(graph.at(2).linearize(v).jacobian()[0]).all()
    return dict(PASS=True,gtsam=importlib.metadata.version('gtsam'),public_API_docs=docs,
       KeyVector='append integer keys; explicitly assemble joint.at(q,q),at(q,c),at(c,q),at(c,c)',
       joint_crossblock_preserved=True,toy_covariance=C.tolist(),tangent=tangent.tolist(),
       sparse_elimination='GTSAM Marginals default sparse multifrontal elimination; no global dense Hessian',
       Jacobian='exact LogmapDerivative(E) times between endpoint Jacobians; Gaussian factor Jacobian audited separately',
       robust='frozen robust graph Marginals; reconstructed diagnostic weight not asserted exact IRLS')

def load():
    frames,lt,ct,local,common,loops,prov=s.read_inputs()
    sets=json.loads((OLD/'selected_loops_by_budget.json').read_text());_,_,re=s.rankings(loops);assert re==sets
    template,params=d.build_graph(frames,local,loops)
    assert json.loads((OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==json.loads((s.D12/'clean_gtsam_solver_policy.json').read_text())['parameters']
    return frames,local,common,loops,prov,sets,template,params

def subset(template,loops,ids):
    selected=loops[loops.frozen_loop_id.isin(ids)]
    indices=list(range(5795))+[5795+int(r.arrival_index)-1 for r in selected.itertuples()]
    graph=b.graph_subset(template,indices)
    assert graph.size()==5795+len(ids) and all(graph.at(i).equals(template.at(j),1e-12) for i,j in enumerate(indices))
    return graph

def covariance_info(C,J,e):
    scale=max(1,float(abs(C).max()));sym=float(abs(C-C.T).max())/scale
    assert C.shape==(12,12) and np.isfinite(C).all() and sym<1e-8
    eig,V=np.linalg.eigh((C+C.T)/2);assert eig.min()>=-1e-10*max(1,eig.max())
    # Roundoff symmetrization only, no PSD eigenvalue clipping/jitter.
    C=(C+C.T)/2;P=J@C@J.T;W=P/SIGMA[:,None]/SIGMA[None,:];W=(W+W.T)/2
    L=np.linalg.cholesky(np.eye(6)+W);di=float(np.log(np.diag(L)).sum())
    S=P+R;S=(S+S.T)/2;se=np.linalg.eigvalsh(S);assert se.min()>0
    nis=float(e@np.linalg.solve(S,e));H=(J/SIGMA[:,None]).T@(J/SIGMA[:,None])
    assert np.isfinite(H).all() and abs(H-H.T).max()<1e-8
    weight=min(1.,1.345/max(np.linalg.norm(e/SIGMA),1e-300))
    support=[float(V[:,i]@H@V[:,i]) for i in [-1,-2,-3]]
    robust_di=float(np.log(np.diag(np.linalg.cholesky(np.eye(6)+weight*W))).sum())
    sign,logdet=np.linalg.slogdet(C);assert sign>0
    return dict(DeltaI=di,NIS=nis,weak_support_1=support[0],weak_support_top3=sum(support),
      robust_effective_DeltaI_diagnostic=robust_di,robust_effective_weak_support_top3_diagnostic=weight*sum(support),
      reconstructed_Huber_weight_diagnostic=weight,max_covariance_eigenvalue=float(eig[-1]),
      pair_logdet=float(logdet),pair_min_eigenvalue=float(eig[0]),pair_symmetry_relative=sym,
      covariance_translation_trace=float(np.trace(C[np.ix_([3,4,5,9,10,11],[3,4,5,9,10,11])])),
      covariance_rotation_trace=float(np.trace(C[np.ix_([0,1,2,6,7,8],[0,1,2,6,7,8])])),
      prediction_min_eigenvalue=float(se[0]),prediction_max_eigenvalue=float(se[-1]),prediction_logdet=float(np.linalg.slogdet(S)[1]),
      H_trace=float(np.trace(H)),weak_v1=V[:,-1].tolist())

def worker(condition):
    start=time.perf_counter()
    vm=int(Path('/proc/self/statm').read_text().split()[0])*os.sysconf('SC_PAGE_SIZE')
    resource.setrlimit(resource.RLIMIT_AS,(vm+2*1024**3,vm+2*1024**3));resource.setrlimit(resource.RLIMIT_CPU,(90,90))
    frames,local,common,loops,prov,sets,template,params=load()
    if condition in CONDITIONS:
        selector,K,tag=CONDITIONS[condition];ids=sets[selector][str(K)];path=OLD/f'trajectories/{tag}.csv'
    else:
        entry=json.loads((OUT/'addback_selection.json').read_text())['trials'][int(condition[1:])]
        ids=sets['COVERAGE_GREEDY']['40']+[entry['frozen_loop_id']];path=OUT/f'trajectories/{condition}.csv'
    graph=subset(template,loops,ids);tr=pd.read_csv(path,usecols=d.POSE_COLUMNS);values=b.to_values(d.s10.lookup(tr))
    timed('factor_reconstruction',start,condition)
    start=time.perf_counter();marg=gtsam.Marginals(graph,values);timed('Marginals_construction',start,condition)
    rank=pd.read_csv(OLD/'loop_rankings.csv');rank=rank[rank.selector.eq('COVERAGE_GREEDY')].set_index('frozen_loop_id')
    xy=np.c_[(loops.query_keyframe_id-150)/1849,(loops.candidate_keyframe_id-234)/3945];selectedxy=xy[loops.frozen_loop_id.isin(ids)]
    distance=np.linalg.norm(xy[:,None]-selectedxy[None,:],axis=2).min(1)
    rows=[];arrays={}
    queryloops=loops if condition in CONDITIONS else loops[loops.frozen_loop_id.eq(entry['frozen_loop_id'])]
    for row in queryloops.itertuples():
        start=time.perf_counter();E,e,J,fd,gd=measurement(row,values);timed('factor_linearization',start,condition)
        keys=pair_keys(row);kv=gtsam.KeyVector();[kv.append(k) for k in keys]
        start=time.perf_counter();joint=marg.jointMarginalCovariance(kv)
        C=np.block([[joint.at(i,j) for j in keys] for i in keys]);timed('joint_covariance_query',start,condition)
        start=time.perf_counter();metrics=covariance_info(C,J,e);timed('information_metric',start,condition)
        fid=int(row.frozen_loop_id);rr=rank.loc[fid]
        rows.append(dict(condition=condition,frozen_loop_id=fid,query_keyframe_id=int(row.query_keyframe_id),candidate_keyframe_id=int(row.candidate_keyframe_id),
          selected=fid in ids,diagnostic_label='IN_GRAPH_LOCAL_INFORMATION_DIAGNOSTIC' if fid in ids else 'OMITTED_LOCAL_INFORMATION_GAIN_PROXY',
          translation_residual_m=float(np.linalg.norm(E.translation())),rotation_residual_deg=float(np.rad2deg(np.linalg.norm(e[:3]))),
          logmap_translation_norm_m=float(np.linalg.norm(e[3:])),logmap_residual=json.dumps(e.tolist()),
          finite_difference_max_error=fd,gaussian_factor_vs_exact_Logmap_J_max_difference=gd,
          GICP_quality=float(row.GICP_quality),coverage_novelty=float(rr.coverage_novelty),
          nearest_selected_pair_distance=float(distance[int(row.arrival_index)-1]),**metrics))
        arrays[f'C_{fid}']=C;arrays[f'J_{fid}']=J
    regional=[]
    for k in [1308,1635,1108,552,2503,551]:
        start=time.perf_counter();C=marg.marginalCovariance(d.key(3,k));timed('weak_region_marginal_query',start,condition)
        regional.append(dict(condition=condition,robot3_KF=k,max_eigenvalue=float(np.linalg.eigvalsh(C).max()),translation_trace=float(np.trace(C[3:,3:]))))
    csv(f'workers/{condition}_metrics.csv',rows);csv(f'workers/{condition}_regions.csv',regional)
    np.savez(OUT/f'workers/{condition}_local_arrays.npz',**arrays)
    csv(f'workers/{condition}_runtime.csv',TIMES)
    dump(f'workers/{condition}_status.json',dict(PASS=True,peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        worker_wall_s=time.perf_counter()-WORKER_START,nodes=values.size(),factors=graph.size(),loops=len(ids),GT_used=False))

WORKER_START=time.perf_counter()

def run_worker(condition):
    start=time.perf_counter();log=OUT/f'workers/{condition}.log';log.parent.mkdir(exist_ok=True)
    try:
        with log.open('w') as f:
            p=subprocess.run([sys.executable,__file__,'--worker',condition],stdout=f,stderr=subprocess.STDOUT,timeout=120)
        assert p.returncode==0,log.read_text()[-3000:]
        status=json.loads((OUT/f'workers/{condition}_status.json').read_text());assert status['PASS']
        timed('bounded_worker_total',start,condition,peak_RSS_MiB=status['peak_RSS_MiB'],status='PASS',timeout=False)
        return pd.read_csv(OUT/f'workers/{condition}_metrics.csv')
    except Exception as e:
        dump(f'workers/{condition}_failure.json',dict(status='API_OR_RESOURCE_BLOCKER',reason=str(e),wall_s=time.perf_counter()-start))
        raise

def main():
    if OUT.exists() and any(OUT.iterdir()):raise RuntimeError('Refuse Stage16 overwrite; --report-only / --audit-only available')
    OUT.mkdir(parents=True);d.OUT=OUT
    subprocess.run(['git','merge-base','--is-ancestor',ACCEPTED,'HEAD'],cwd=ROOT,check=True)
    api=api_audit();dump('stage16_api_audit.json',api)
    frames,local,common,loops,prov,sets,template,params=load();dump('loop_provenance_audit.json',prov)
    paths=[ROOT/p for p in subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
      if p.startswith(('src/cumulti/','scripts/','outputs/cumulti_v1/','docs/CUMULTI_STAGE','demo/')) and 'stage16' not in p.lower() and '/16_measurement_information/' not in p]
    hashes={str(p):d.sha256(p) for p in paths if p.is_file()}
    inputs=[d.LOOP_SOURCE,s.D12/'ordered_rank1_clean_loops.csv',s.D12/'clean_local_trajectory.csv',s.D12/'clean_first_loop_trajectory.csv',
       s.D12/'clean_graph_policy.json',s.D12/'clean_gtsam_solver_policy.json',OLD/'selected_loops_by_budget.json',OLD/'stage15_experiment_policy.json',
       OLD/'stage15_pre_gt_decision.json',OLD/'loop_rankings.csv']+[OLD/f'trajectories/{tag}.csv' for _,_,tag in CONDITIONS.values()]
    for p in inputs:
        blob=subprocess.check_output(['git','show',ACCEPTED+':'+str(p.relative_to(ROOT))],cwd=ROOT)
        assert hashlib.sha256(blob).hexdigest()==d.sha256(p),p
    manifest=dict(accepted_checkpoint=ACCEPTED,inputs=[dict(path=str(p),sha256=d.sha256(p),size_bytes=p.stat().st_size) for p in inputs],historical_before=hashes,GT_decoded=False)
    dump('stage16_input_manifest.json',manifest)
    policy=dict(UTC=a.now(),accepted_inputs=manifest['inputs'],conditions=CONDITIONS,linearization='each own frozen Stage15 batch trajectory; no five-condition optimization',
      API=api,noise=dict(sigmas=SIGMA.tolist(),Huber_delta=1.345,anchor='Robot1 KF150',tangent='rx ry rz tx ty tz'),
      Jacobian='exact residual Logmap Jacobian via public LogmapDerivative(E) @ [between.Hq,between.Hc]; finite-difference epsilon1e-6; Gaussian-factor Jacobian separately retained because GTSAM factor linearization may use an approximate chart derivative',
      covariance='GTSAM sparse Marginals on exact frozen robust graph; joint.at block assembly preserves endpoint cross correlation; no independent block approximation',
      DeltaI='0.5 logdet(I + D^-1 J Sigma J.T D^-1), Cholesky; D=sigma diag',NIS='e.T solve(J Sigma J.T + R,e)',
      Huber='RECONSTRUCTED_HUBER_WEIGHT_DIAGNOSTIC=min(1,1.345/norm(e/sigma)); H_eff=w H; robust DeltaI uses wW; not claimed internal IRLS weight',
      weak='largest/top3 joint covariance eigenvectors Euclidean-normalized in native right-retraction rad/metre tangent; v.T H v; mixed-unit coordinate-dependent; not global Hessian nullspace',
      numerical_gates=dict(FD_max_abs_lt=1e-5,symmetry_relative_lt=1e-8,PSD_min_ge='-1e-10*max(1,maxeig)',Cholesky_without_jitter=True,finite=True),
      resources=dict(additional_address_space_GiB=2,CPU_seconds=90,wall_seconds=120,BLAS_threads=1,global_dense_Hessian=False),
      addback=dict(primary='CoverageK40',top=5,bottom=5,order='DeltaI descending/ascending then lower frozen ID',canonical_x0='frozen12D first bridge common; no K40 warmstart',independent_K41=True,LM=json.loads((s.D12/'clean_gtsam_solver_policy.json').read_text())),
      weak_regions=dict(A=[1208,1408],B=[1535,1735],C1=[452,652],C2=[2403,2603],C3=[451,651],source='Stage14 frozen1308/1635 and matched552/2503/551; inclusive +/-100KF; overlaps reported separately, not pooled'),
      representative='median split over all75 S1 loops; >=median high; one lowest frozen ID per nonempty category; no manual cases',
      decision=dict(heterogeneity='omitted DeltaI p90-p10 >= 1 nat',alignment='Spearman DeltaI/weak_support_top3 >=0.3',
        impact='TOP median reduction minus BOTTOM median reduction >= 0.1*abs(K40 reference metric), for >=2 of all75 translationp95,rotationp95,targettranslation,weak eigenvaluefraction',
        full='heterogeneity and alignment and >=2 material metrics and >=2 corresponding DeltaI/impact Spearman>=0.3',
        partial='heterogeneity and at least one TOP trial reduces all75 translation or rotation p95 by >=10%; otherwise NO_CLEAR; any required worker failure blocker'),
      GT_used=False,new_sparse_policy=False,map=dict(frame_stride=10,point_stride=16,voxel_m=.5),Stage15_thresholds_changed=False)
    dump('stage16_experiment_policy.json',policy);dump('policy_freeze.json',dict(sha256=d.sha256(OUT/'stage16_experiment_policy.json'),UTC=a.now()))
    print('STAGE16 POLICY FROZEN',flush=True)
    graphs=[]
    for code,(selector,K,tag) in CONDITIONS.items():
        graph=subset(template,loops,sets[selector][str(K)])
        graphs.append(dict(condition=code,nodes=5796,odometry=5794,prior=1,loops=K,factors=graph.size(),PASS=True))
    dump('graph_equivalence.json',graphs)
    tables=[]
    for code in CONDITIONS:
        print('information worker',code,flush=True);tables.append(run_worker(code))
    table=pd.concat(tables,ignore_index=True)
    for name in ['loop_linearized_information','loop_information_gain','normalized_innovation','weak_direction_support','pair_covariance_summary']:csv(name+'.csv',table)
    addbacks(table,frames,common,loops,sets,template,params,hashes)

def addbacks(table,frames,common,loops,sets,template,params,hashes):
    primary=table[table.condition.eq('S1')];om=primary[~primary.selected].copy();assert len(om)==35
    trials=[]
    for group,ascending in [('TOP5',False),('BOTTOM5',True)]:
        for r in om.sort_values(['DeltaI','frozen_loop_id'],ascending=[ascending,True]).head(5).itertuples():
            trials.append(dict(group=group,frozen_loop_id=int(r.frozen_loop_id),DeltaI=float(r.DeltaI)))
    assert len({r['frozen_loop_id'] for r in trials})==10
    policy=json.loads((OUT/'stage16_experiment_policy.json').read_text())
    selection=dict(rule=policy['addback'],policy_sha256=d.sha256(OUT/'stage16_experiment_policy.json'),trials=trials)
    if (OUT/'addback_selection.json').exists():assert json.loads((OUT/'addback_selection.json').read_text())==selection
    else:dump('addback_selection.json',selection)
    maps=s.MapCache.__new__(s.MapCache);s.OUT=OUT;maps.__init__(frames)
    residuals=[];solvers=[];maprows=[]
    (OUT/'trajectories').mkdir(exist_ok=True)
    K40=d.s10.lookup(pd.read_csv(OLD/'trajectories/P2_K40.csv',usecols=d.POSE_COLUMNS));base,_=d.loop_residuals(loops,K40)
    for i,trial in enumerate(trials):
        code=f'A{i}';fid=trial['frozen_loop_id'];ids=sets['COVERAGE_GREEDY']['40']+[fid];graph=subset(template,loops,ids)
        start=time.perf_counter();optimizer=gtsam.LevenbergMarquardtOptimizer(graph,b.to_values(common),params);res=optimizer.optimize();elapsed=1000*(time.perf_counter()-start);timed('addback_LM',start,code)
        mat=b.matrices_from(res,list(common));tr=d.trajectory(frames,mat);assert d.schema_audit(tr)['PASS']
        tr.to_csv(OUT/f'trajectories/{code}.csv',index=False)
        solvers.append(dict(condition=code,**trial,nodes=res.size(),odom_factors=5794,loop_factors=41,prior=1,runtime_ms=elapsed,iterations=int(optimizer.iterations()),initial_error=float(graph.error(b.to_values(common))),final_error=float(graph.error(res)),finite=True))
        start=time.perf_counter();rr,_=d.loop_residuals(loops,mat);timed('residual_evaluation',start,code)
        fidix=int(loops.index[loops.frozen_loop_id.eq(fid)][0]);remaining=~loops.frozen_loop_id.isin(ids)
        diffs=[d.s10.inv(K40[n])@mat[n] for n in common];td=np.array([np.linalg.norm(x[:3,3]) for x in diffs]);rd=np.array([np.rad2deg(Rotation.from_matrix(x[:3,:3]).magnitude()) for x in diffs])
        effects=dict(condition=code,**trial,target_translation_before=float(base.iloc[fidix].translation_residual_m),target_translation_after=float(rr.iloc[fidix].translation_residual_m),target_rotation_before=float(base.iloc[fidix].rotation_residual_deg),target_rotation_after=float(rr.iloc[fidix].rotation_residual_deg))
        for prefix,df in [('all75',rr),('remaining34',rr[remaining])]:
            effects.update({prefix+'_'+k:v for k,v in s.residual_summary(df).items()})
        effects.update({'trajectory_translation_'+k+'_m':v for k,v in s.stat(td).items()});effects.update({'trajectory_rotation_'+k+'_deg':v for k,v in s.stat(rd).items()})
        start=time.perf_counter();cloud=maps.make(mat);timed('map_transform_voxel',start,code)
        start=time.perf_counter();nn=a.nn(cloud['robot1'],cloud['robot3']);timed('map_NN',start,code)
        maprows.append(dict(condition=code,**trial,**nn))
        metrics=run_worker(code).iloc[0]
        before=om[om.frozen_loop_id.eq(fid)].iloc[0]
        effects['weak_max_eigenvalue_fraction_reduction']=1-metrics.max_covariance_eigenvalue/before.max_covariance_eigenvalue
        C=np.load(OUT/f'workers/{code}_local_arrays.npz')[f'C_{fid}'];v=np.array(json.loads(before.weak_v1))
        effects['original_weak_direction_fraction_reduction']=1-float(v@C@v)/before.max_covariance_eigenvalue
        residuals.append(effects)
        csv('addback_solver_results.csv',solvers);csv('addback_residual_effects.csv',residuals);csv('addback_map_effects.csv',maprows)
        print('completed addback',code,fid,flush=True)
    csv('runtime_summary.csv',TIMES)
    assert all(d.sha256(p)==h for p,h in hashes.items())
    dump('historical_immutability.json',dict(PASS=True,files_checked=len(hashes)))
    report()

def corr(df,x,y):
    if len(df)<3 or df[x].nunique()<2 or df[y].nunique()<2:return dict(Pearson=None,Spearman=None,count=len(df))
    return dict(Pearson=float(pearsonr(df[x],df[y]).statistic),Spearman=float(spearmanr(df[x],df[y]).statistic),count=len(df))

def report():
    start=time.perf_counter();table=pd.read_csv(OUT/'loop_information_gain.csv');p=table[table.condition.eq('S1')].copy();om=p[~p.selected].copy()
    for x in ['DeltaI','NIS','weak_support_top3']:om['rank_'+x]=om[x].rank(method='min',ascending=False).astype(int)
    csv('coverage_k40_information_ranking.csv',om.sort_values(['DeltaI','frozen_loop_id'],ascending=[False,True]))
    variables=['translation_residual_m','rotation_residual_deg','DeltaI','NIS','weak_support_1','weak_support_top3','GICP_quality','coverage_novelty','nearest_selected_pair_distance']
    rows=[]
    for code in ['S1','S2','S3','S4']:
        df=table[table.condition.eq(code)&~table.selected]
        for x,y in itertools.combinations(variables,2):rows.append(dict(condition=code,cohort='OMITTED',x=x,y=y,**corr(df,x,y)))
    for x in ['coverage_novelty','GICP_quality']:
        for y in ['DeltaI','weak_support_top3']:rows.append(dict(condition='S1',cohort='ALL75_IN_GRAPH_AND_OMITTED_NOT_EQUIVALENT',x=x,y=y,**corr(p,x,y)))
    csv('information_residual_correlations.csv',rows)
    distributions=[]
    for selected,df in p.groupby('selected'):
        for x in ['DeltaI','weak_support_top3','GICP_quality','coverage_novelty','NIS']:
            distributions.append(dict(selected=bool(selected),metric=x,count=len(df),**s.stat(df[x])))
    csv('selected_vs_omitted_information.csv',distributions)
    full=table[table.condition.eq('F0')].set_index('frozen_loop_id');comparisons=[]
    for r in om.itertuples():
        f=full.loc[r.frozen_loop_id]
        comparisons.append(dict(frozen_loop_id=r.frozen_loop_id,candidate_keyframe_id=r.candidate_keyframe_id,
         max_eigenvalue_sparse_full_ratio=r.max_covariance_eigenvalue/f.max_covariance_eigenvalue,
         logdet_sparse_minus_full=r.pair_logdet-f.pair_logdet,
         translation_trace_sparse_full_ratio=r.covariance_translation_trace/f.covariance_translation_trace,
         rotation_trace_sparse_full_ratio=r.covariance_rotation_trace/f.covariance_rotation_trace))
    cv=pd.DataFrame(comparisons);csv('full75_vs_sparse_covariance.csv',cv)
    p['novelty_high']=p.coverage_novelty>=p.coverage_novelty.median();p['information_high']=p.DeltaI>=p.DeltaI.median()
    cases=[]
    for hn,hi in itertools.product([False,True],repeat=2):
        group=p[p.novelty_high.eq(hn)&p.information_high.eq(hi)].sort_values('frozen_loop_id')
        if len(group):cases.append(dict(category=('HIGH' if hn else 'LOW')+'_NOVELTY_'+('HIGH' if hi else 'LOW')+'_INFORMATION',category_count=len(group),**group.iloc[0].to_dict()))
    csv('representative_information_cases.csv',cases)
    policy=json.loads((OUT/'stage16_experiment_policy.json').read_text());regions=[]
    for code in CONDITIONS:
        df=table[table.condition.eq(code)]
        for name,window in policy['weak_regions'].items():
            if name=='source':continue
            lo,hi=window
            for selected in [False,True]:
                g=df[df.candidate_keyframe_id.between(lo,hi)&df.selected.eq(selected)]
                regions.append(dict(condition=code,region=name,KF_start=lo,KF_end=hi,selected=selected,count=len(g),status='MEASURED' if len(g) else 'EMPTY',
                    DeltaI_median=float(g.DeltaI.median()) if len(g) else None,weak_support_median=float(g.weak_support_top3.median()) if len(g) else None,NIS_median=float(g.NIS.median()) if len(g) else None))
    regiontable=pd.DataFrame(regions)
    for name,window in policy['weak_regions'].items():
        if name=='source':continue
        measured=cv[cv.candidate_keyframe_id.between(*window)]
        regiontable.loc[regiontable.condition.eq('S1')&regiontable.region.eq(name)&~regiontable.selected,'omitted_sparse_full_max_eigenvalue_ratio_median']=float(measured.max_eigenvalue_sparse_full_ratio.median()) if len(measured) else np.nan
    csv('weak_region_analysis.csv',regiontable)
    regional=pd.concat([pd.read_csv(q) for q in (OUT/'workers').glob('*_regions.csv')],ignore_index=True)
    control=regional[regional.condition.eq('S1')].set_index('robot3_KF')
    regional['translation_trace_ratio_vs_K40']=[r.translation_trace/control.loc[r.robot3_KF].translation_trace for r in regional.itertuples()]
    regional['max_eigenvalue_ratio_vs_K40']=[r.max_eigenvalue/control.loc[r.robot3_KF].max_eigenvalue for r in regional.itertuples()]
    csv('weak_region_marginal_addback.csv',regional)
    ad=pd.read_csv(OUT/'addback_residual_effects.csv')
    base_t=float(np.percentile(p.translation_residual_m,95));base_r=float(np.percentile(p.rotation_residual_deg,95))
    ad['all75_translation_p95_reduction']=base_t-ad.all75_translation_p95_m
    ad['all75_rotation_p95_reduction']=base_r-ad.all75_rotation_p95_deg
    ad['target_translation_reduction']=ad.target_translation_before-ad.target_translation_after
    ad['target_rotation_reduction']=ad.target_rotation_before-ad.target_rotation_after
    csv('addback_residual_effects.csv',ad)
    impacts=['all75_translation_p95_reduction','all75_rotation_p95_reduction','target_translation_reduction','target_rotation_reduction','trajectory_translation_p95_m','weak_max_eigenvalue_fraction_reduction','original_weak_direction_fraction_reduction']
    prediction=[]
    for x in impacts:
        prediction.append(dict(metric=x,TOP5_median=float(ad[ad.group.eq('TOP5')][x].median()),BOTTOM5_median=float(ad[ad.group.eq('BOTTOM5')][x].median()),**corr(ad,'DeltaI',x)))
    csv('information_predictive_power.csv',prediction)
    gates=[('all75_translation_p95_reduction',base_t),('all75_rotation_p95_reduction',base_r),('target_translation_reduction',float(ad.target_translation_before.median())),('weak_max_eigenvalue_fraction_reduction',1.)]
    material=[];predictive=[]
    for x,ref in gates:
        row=next(r for r in prediction if r['metric']==x)
        if row['TOP5_median']-row['BOTTOM5_median']>=.1*abs(ref):material.append(x)
        if row['Spearman'] is not None and row['Spearman']>=.3:predictive.append(x)
    hetero=float(np.percentile(om.DeltaI,90)-np.percentile(om.DeltaI,10))>=1
    alignment=corr(om,'DeltaI','weak_support_top3')['Spearman']>=.3
    some=bool(((ad.group.eq('TOP5'))&((ad.all75_translation_p95_reduction>=.1*base_t)|(ad.all75_rotation_p95_reduction>=.1*base_r))).any())
    decision='MEASUREMENT_INFORMATION_EXPLAINS_SPARSE_FAILURE' if hetero and alignment and len(material)>=2 and len(set(material)&set(predictive))>=2 else 'MEASUREMENT_INFORMATION_PARTIALLY_EXPLAINS_SPARSE_FAILURE' if hetero and some else 'NO_CLEAR_INFORMATION_EXPLANATION'
    dump('stage16_information_decision.json',dict(status=decision,heterogeneity=hetero,alignment=alignment,material_metrics=material,predictive_metrics=predictive,some_material_TOP_effect=some,GT_used=False,new_sparse_policy=False))
    figures=[('01_deltaI_distribution_selected_vs_omitted',None,None),('02_structural_novelty_vs_information_gain','coverage_novelty','DeltaI'),('03_GICP_quality_vs_information_gain','GICP_quality','DeltaI'),('04_heldout_residual_vs_information_gain','translation_residual_m','DeltaI'),('05_NIS_vs_information_gain','NIS','DeltaI'),('06_weak_support_vs_information_gain','weak_support_top3','DeltaI')]
    for name,x,y in figures:
        fig,ax=plt.subplots(figsize=(8,5))
        if x:
            df=om if name.startswith(('04','05','06')) else p
            for selected,label,color in [(False,'Omitted gain proxy','#2878b5'),(True,'Selected in-graph diagnostic','#cc4444')]:
                group=df[df.selected.eq(selected)]
                if len(group):ax.scatter(group[x],group[y],label=label,color=color)
            ax.legend(fontsize=8);ax.set_xlabel(x);ax.set_ylabel(y)
        else:ax.boxplot([p[p.selected].DeltaI,om.DeltaI],labels=['Selected40: in-graph','Omitted35: proxy']);ax.set_ylabel('DeltaI (nats)')
        ax.set_title('Coverage K40 — local frozen linearization');fig.tight_layout();fig.savefig(OUT/(name+'.png'),dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,5));g=om.sort_values('DeltaI',ascending=False);ax.bar(np.arange(35),g.DeltaI);ax.set_xticks(np.arange(35),g.frozen_loop_id,rotation=90);ax.set_ylabel('DeltaI (nats)');ax.set_xlabel('Omitted frozen loop ID');fig.tight_layout();fig.savefig(OUT/'07_coverage_k40_omitted_loop_information.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,x,label in zip(axes,['all75_translation_p95_reduction','all75_rotation_p95_reduction','original_weak_direction_fraction_reduction'],['All75 translation p95 reduction (m)','All75 rotation p95 reduction (deg)','Original weak-direction variance reduction']):
        for group,color in [('TOP5','#cc4444'),('BOTTOM5','#2878b5')]:
            sample=ad[ad.group.eq(group)];ax.scatter(sample.DeltaI,sample[x],color=color,label=group)
        ax.axhline(0,color='grey');ax.set_xlabel('DeltaI (nats)');ax.set_ylabel(label);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUT/'08_top_bottom_addback_effect.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5));ax.scatter(cv.frozen_loop_id,cv.max_eigenvalue_sparse_full_ratio);ax.set_yscale('log');ax.set_ylabel('Sparse / Full75 largest joint eigenvalue');ax.set_xlabel('Omitted K40 frozen loop ID');fig.tight_layout();fig.savefig(OUT/'09_sparse_vs_full75_pair_uncertainty.png',dpi=150);plt.close(fig)
    reg=pd.DataFrame(regions);g=reg[reg.condition.eq('S1')&~reg.selected];fig,ax=plt.subplots(figsize=(8,5));valid=g[g['count'].gt(0)];ax.bar(valid.region,valid.DeltaI_median);ax.set_ylabel('Median DeltaI (omitted, K40)');ax.set_title('Frozen Stage14 windows; empty categories not fabricated');fig.tight_layout();fig.savefig(OUT/'10_weak_region_information_support.png',dpi=150);plt.close(fig)
    summary=dict(status=decision,GT_USED=False,primary=dict(selected=40,omitted=35,all75_translation_p95_m=base_t,all75_rotation_p95_deg=base_r,heldout_translation_p95_m=float(np.percentile(om.translation_residual_m,95)),heldout_rotation_p95_deg=float(np.percentile(om.rotation_residual_deg,95))),distributions=distributions,predictive=prediction,
      covariance_ratios={k:s.stat(cv[k]) for k in cv.columns if 'ratio' in k},representative_thresholds=dict(novelty_median=float(p.coverage_novelty.median()),DeltaI_median=float(p.DeltaI.median())),figures=10,
      limitations=['local frozen linearizations, not global observability','joint weak directions dominated by common-mode gauge-correlated uncertainty may be weakly seen by relative factors','mixed rad/metre eigenvectors are coordinate-dependent','frozen robust graph covariance is not physically calibrated','in-graph factor scores are not pre-insertion gains','n=5 groups, shared-variable loops dependent; no significance/causality claims','Stage15 negative decision and rotation-screen caveats unchanged'])
    dump('summary.json',summary)
    assert d.sha256(OUT/'stage16_experiment_policy.json')==json.loads((OUT/'policy_freeze.json').read_text())['sha256']
    fields={k:'PASS' for k in ['HISTORICAL_INPUTS_IMMUTABLE','LOOP_120_TO_75_PROVENANCE_VERIFIED','CLEAN75_EXACT','STAGE15_SELECTIONS_REPRODUCED','FIVE_CONDITIONS_FROZEN','API_AUDIT_COMPLETED','JOINT_PAIR_COVARIANCE_VALID','LOOP_JACOBIANS_VALID','INFORMATION_GAIN_COMPUTED','NORMALIZED_INNOVATION_COMPUTED','WEAK_DIRECTION_SUPPORT_COMPUTED','COVERAGE_K40_PRIMARY_ANALYSIS','TOP5_BOTTOM5_RULE_PREDECLARED','TEN_ADD_BACK_TRIALS_COMPLETED','ADD_BACK_USES_NO_GT','STAGE15_THRESHOLDS_UNCHANGED','STAGE15_DECISION_UNCHANGED']}
    fields.update({k:'NO' for k in ['GT_USED','EXCLUDED45_REINTRODUCED','GLOBAL_DENSE_HESSIAN_CONSTRUCTED','NEW_SPARSE_POLICY_SELECTED','HISTORICAL_STAGE14_MODIFIED','HISTORICAL_STAGE15_MODIFIED','DEMO_MODIFIED']})
    (OUT/'VALIDATION_REPORT.txt').write_text('\n'.join(k+': '+v for k,v in fields.items())+'\nDECISION: '+decision+'\n')
    timed('report_figures_files',start);csv('report_runtime.csv',TIMES)
    parent=pd.read_csv(OUT/'runtime_summary.csv')
    parent=parent[~parent.phase.str.startswith(('Marginals_','joint_','information_','factor_linearization','weak_region_'))].copy()
    # Recovery may not retain parent five-worker totals; persisted worker timing
    # files and status provide the canonical disaggregated measurement costs.
    worker_times=pd.concat([pd.read_csv(q) for q in (OUT/'workers').glob('*_runtime.csv')],ignore_index=True)
    totals=[]
    for q in (OUT/'workers').glob('*_status.json'):
        st=json.loads(q.read_text());totals.append(dict(phase='worker_process_wall',condition=q.name.removesuffix('_status.json'),wall_ms=1000*st['worker_wall_s'],peak_RSS_MiB=st['peak_RSS_MiB'],status='PASS',timeout=False))
    runtime=pd.concat([parent[parent.phase.isin(['addback_LM','residual_evaluation','map_transform_voxel','map_NN'])],worker_times,pd.DataFrame(totals),pd.DataFrame(TIMES)[pd.DataFrame(TIMES).phase.eq('report_figures_files')]],ignore_index=True)
    csv('runtime_summary.csv',runtime)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--worker');parser.add_argument('--report-only',action='store_true');parser.add_argument('--audit-only',action='store_true');parser.add_argument('--resume-addback',action='store_true');args=parser.parse_args()
    d.OUT=OUT
    if args.worker:worker(args.worker)
    elif args.resume_addback:
        assert not (OUT/'summary.json').exists(),'Refuse completed Stage16 overwrite'
        assert d.sha256(OUT/'stage16_experiment_policy.json')==json.loads((OUT/'policy_freeze.json').read_text())['sha256']
        frames,local,common,loops,prov,sets,template,params=load()
        table=pd.read_csv(OUT/'loop_information_gain.csv')
        assert len(table)==375
        dump('execution_recovery.json',dict(reason='loop_residuals helper returns (DataFrame,summary); unpack interface fixed',policy_unchanged=True,information_not_recomputed=True,first_A0_repeated_after_reporting_failure=True,UTC=a.now()))
        addbacks(table,frames,common,loops,sets,template,params,json.loads((OUT/'stage16_input_manifest.json').read_text())['historical_before'])
    elif args.report_only:report()
    elif args.audit_only:subprocess.run([sys.executable,str(Path(__file__).with_name('audit_stage16_measurement_information.py'))],check=True)
    else:
        try:main()
        except Exception:
            if OUT.exists():dump('execution_failure.json',dict(traceback=traceback.format_exc(),GT_used=False))
            raise
