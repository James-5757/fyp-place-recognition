#!/usr/bin/env python3
"""Independent saved-artifact numerical audit; no graph optimization or GT."""
import ast
import hashlib
import json
import os
from pathlib import Path
import sys
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.stats import pearsonr,spearmanr

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/cumulti_v1/17_relative_measurement_geometry'
S16=ROOT/'outputs/cumulti_v1/16_measurement_information'
S15=ROOT/'outputs/cumulti_v1/15_informative_loop_selection'
CHECKS=[]

def no_gt(event,args):
    if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
        path=os.fsdecode(args[0]).replace('\\','/').lower()
        if '/home/cas/cu-multi/' in path or any(x in path for x in ['gt_utm','gt_rel','offline_gt_evaluation','offline_gt.csv']):raise PermissionError('Forbidden GT/dataset input')

sys.addaudithook(no_gt)

def check(name,ok):
    CHECKS.append(dict(name=name,PASS=bool(ok)))
    if not ok:raise AssertionError(name)

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def T(row):
    matrix=np.eye(4);matrix[:3,:3]=Rotation.from_quat(row[['qx','qy','qz','qw']].to_numpy(dtype=float)).as_matrix();matrix[:3,3]=row[['tx','ty','tz']].to_numpy(dtype=float);return matrix

def pose(matrix):return gtsam.Pose3(gtsam.Rot3(matrix[:3,:3]),matrix[:3,3])

def main():
    policy=json.loads((OUT/'stage17_experiment_policy.json').read_text());sig=np.array(policy['noise']['sigmas']);R=np.diag(sig**2)
    check('policy immutable',sha(OUT/'stage17_experiment_policy.json')==json.loads((OUT/'policy_freeze.json').read_text())['sha256'])
    check('conditions exact',set(policy['conditions'])=={'F0','S1','S4'})
    check('frozen ten IDs exact',policy['validation_ids']==[105,53,55,100,48,61,114,62,110,77])
    check('stop line frozen',policy['stop']['numerical_diagnostic_line_closed'] and not policy['stop']['new_sparse_policy'] and not policy['stop']['new_K_search'] and not policy['stop']['GT_used'] and not policy['stop']['demo_modified'])
    manifest=json.loads((OUT/'stage17_input_manifest.json').read_text())
    for entry in manifest['inputs']:check('accepted input '+entry['path'],sha(entry['path'])==entry['sha256'])
    for path,h in manifest['historical_non_GT_before'].items():check('historical immutable '+path,sha(path)==h)
    tree=ast.parse((ROOT/'src/cumulti/run_stage17_relative_measurement_geometry.py').read_text())
    forbidden={'optimize','LevenbergMarquardtOptimizer','offline_gt','map_points','rankings'}
    calls={getattr(n.func,'attr',getattr(n.func,'id','')) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    check('no optimization or GT/map calls',not calls&forbidden)
    columns=['arrival_index','frozen_loop_id','query_keyframe_id','candidate_keyframe_id','tx','ty','tz','qx','qy','qz','qw']
    loops=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv',usecols=columns).set_index('frozen_loop_id')
    sets=json.loads((S15/'selected_loops_by_budget.json').read_text());old=pd.read_csv(S16/'loop_information_gain.csv').set_index(['condition','frozen_loop_id'])
    table=pd.read_csv(OUT/'projected_relative_uncertainty.csv')
    check('125 exact pair queries',len(table)==125 and table.groupby('condition').size().to_dict()=={'F0':35,'S1':35,'S4':55})
    for name in ['relative_jacobian_validation','common_mode_projection_diagnostic','nonlinear_residual_geometry','residual_uncertainty_alignment','relative_normalized_innovation','robust_effective_information']:
        check('consistent primary table '+name,pd.read_csv(OUT/(name+'.csv')).equals(table))
    for code,(selector,K,tag) in policy['conditions'].items():
        records=table[table.condition.eq(code)];expected=set(policy['omitted_K40_ids']) if code=='F0' else set(loops.index)-set(sets[selector][str(K)])
        check(code+' exact cohort',set(records.frozen_loop_id)==expected and not records.frozen_loop_id.duplicated().any())
        arrays=np.load(OUT/f'workers/{code}/local_matrices.npz');frozenarrays=np.load(S16/f'workers/{code}_local_arrays.npz')
        trajectories=pd.read_csv(S15/f'trajectories/{tag}.csv',usecols=['robot_id','keyframe_id','tx','ty','tz','qx','qy','qz','qw']).set_index(['robot_id','keyframe_id'])
        for row in records.itertuples():
            fid=row.frozen_loop_id;prefix=f'{code}#{fid} ';pair=arrays[f'pair_{fid}'];J=arrays[f'J_{fid}'];relative=arrays[f'relative_{fid}'];C=arrays[f'whitened_{fid}'];U=arrays[f'U_{fid}']
            check(prefix+'frozen joint covariance',np.allclose(pair,frozenarrays[f'C_{fid}'],rtol=1e-8,atol=1e-8) and np.linalg.norm(pair[:6,6:])>0)
            check(prefix+'exact frozen Jacobian',np.allclose(J,frozenarrays[f'J_{fid}'],atol=1e-8,rtol=0))
            for name,matrix,size in [('pair',pair,12),('relative',relative,6),('whitened',C,6)]:
                check(prefix+name+' valid',matrix.shape==(size,size) and np.isfinite(matrix).all() and abs(matrix-matrix.T).max()/max(1,abs(matrix).max())<1e-8 and np.linalg.eigvalsh(matrix).min()>=-1e-10*max(1,np.linalg.eigvalsh(matrix).max()))
            check(prefix+'projection equation',np.allclose(relative,J@pair@J.T,rtol=1e-8,atol=1e-8) and np.allclose(C,relative/sig[:,None]/sig[None,:],rtol=1e-8,atol=1e-8))
            check(prefix+'eigendecomposition',np.allclose(U.T@U,np.eye(6),atol=1e-9) and np.allclose(U@np.diag(json.loads(row.eigenvalues))@U.T,C,rtol=1e-8,atol=1e-8))
            loop=loops.loc[fid];q=pose(T(trajectories.loc[('robot1',int(loop.query_keyframe_id))]));c=pose(T(trajectories.loc[('robot3',int(loop.candidate_keyframe_id))]));z=pose(T(loop));E=z.inverse().compose(q.between(c));e=gtsam.Pose3.Logmap(E);r=e/sig
            check(prefix+'physical/logmap residual',np.allclose(e,json.loads(row.logmap_residual),atol=1e-8) and np.isclose(np.linalg.norm(E.translation()),row.physical_translation_residual_m,atol=1e-8))
            sign,ld=np.linalg.slogdet(np.eye(6)+C);check(prefix+'DeltaI identity',sign>0 and abs(ld-2*old.loc[(code,fid)].DeltaI)<1e-7)
            nis=float(r@np.linalg.solve(np.eye(6)+C,r));check(prefix+'NIS reproduction',np.isclose(nis,row.NIS,atol=1e-7) and abs(nis-old.loc[(code,fid)].NIS)<1e-7)
            amplitudes=U.T@r;frac=amplitudes**2/max(float(r@r),1e-24)
            check(prefix+'alignment fractions',np.isclose(frac[0],row.residual_top1_fraction,atol=1e-8) and np.isclose(frac[:3].sum(),row.residual_top3_fraction,atol=1e-8))
            pe,V=np.linalg.eigh(pair);energy=pe[-1]*np.linalg.norm(J@V[:,-1]/sig)**2/np.trace(C)
            check(prefix+'common mode projection',np.isclose(energy,row.top_pair_mode_fraction_projected_trace,atol=1e-7))
            check(prefix+'FD tolerance',row.finite_difference_max_error<=policy['validity']['FD'])
            h=gtsam.noiseModel.mEstimator.Huber.Create(1.345);wn=float(np.linalg.norm(r))
            check(prefix+'public Huber model',np.isclose(h.weight(wn),row.Huber_weight,atol=1e-12) and np.isclose(h.loss(wn),row.Huber_loss,atol=1e-8) and np.isclose(row.Huber_loss,row.actual_GTSAM_factor_error,atol=1e-8))
        status=json.loads((OUT/f'workers/{code}/status.json').read_text())
        check(code+' bounded worker',status['PASS'] and status['worker_wall_seconds']<120 and status['optimization_run'] is False)
    curves=pd.read_csv(OUT/'nonlinear_perturbation_geometry.csv');robust=pd.read_csv(OUT/'robust_residual_geometry.csv')
    check('210 exact nonlinear points',len(curves)==210 and set(curves.frozen_loop_id)==set(policy['validation_ids']) and set(curves['mode'])=={1,2,3} and set(curves.alpha)==set(policy['relative_perturbation']['alphas']) and not curves[['frozen_loop_id','mode','alpha']].duplicated().any())
    check('consistent robust sweep',robust.equals(curves))
    arrays=np.load(OUT/'workers/S1/local_matrices.npz');trajectory=pd.read_csv(S15/'trajectories/P2_K40.csv',usecols=['robot_id','keyframe_id','tx','ty','tz','qx','qy','qz','qw']).set_index(['robot_id','keyframe_id'])
    for row in curves.itertuples():
        loop=loops.loc[row.frozen_loop_id];q=pose(T(trajectory.loc[('robot1',int(loop.query_keyframe_id))]));c=pose(T(trajectory.loc[('robot3',int(loop.candidate_keyframe_id))]));z=pose(T(loop));relative=q.between(c);E=z.inverse().compose(relative);e=gtsam.Pose3.Logmap(E)
        direction=arrays[f'loop{row.frozen_loop_id}_mode{row.mode}_direction'];enl=gtsam.Pose3.Logmap(z.inverse().compose(relative.compose(gtsam.Pose3.Expmap(row.alpha*direction))))
        elin=e+row.alpha*gtsam.Pose3.LogmapDerivative(E)@direction
        check(f'curve#{row.frozen_loop_id}/{row.mode}/{row.alpha} exact prediction',np.allclose(enl,json.loads(row.nonlinear_residual),atol=1e-8) and np.allclose(elin,json.loads(row.linear_residual),atol=1e-8))
        error=float(np.linalg.norm((enl-elin)/sig));wn=float(np.linalg.norm(enl/sig))
        check(f'curve#{row.frozen_loop_id}/{row.mode}/{row.alpha} error metrics',np.isclose(error,row.NL_abs,atol=1e-8) and np.isclose(error/max(wn,1e-12),row.NL_rel,atol=1e-8))
        h=gtsam.noiseModel.mEstimator.Huber.Create(1.345);check(f'curve#{row.frozen_loop_id}/{row.mode}/{row.alpha} robust loss',np.isclose(h.weight(wn),row.Huber_weight,atol=1e-12) and np.isclose(h.loss(wn),row.Huber_loss,atol=1e-8))
    cohort=pd.read_csv(OUT/'frozen_ten_loop_predictors_outcomes.csv');oldout=pd.read_csv(S16/'addback_residual_effects.csv').set_index('frozen_loop_id')
    for outcome in policy['outcomes']:check('read-only frozen outcome '+outcome,np.allclose(cohort[outcome],oldout.loc[cohort.frozen_loop_id,outcome],atol=1e-12,rtol=0))
    predictive=pd.read_csv(OUT/'stage17_predictive_validation.csv')
    check('all predictors/outcomes N10 exploratory',len(predictive)==72 and predictive.N.eq(10).all() and predictive.interpretation.eq('EXPLORATORY').all())
    for row in predictive.itertuples():
        x,y=cohort[row.predictor],cohort[row.outcome]
        check('correlation '+row.predictor+'/'+row.outcome,np.isclose(pearsonr(x,y).statistic,row.Pearson,atol=1e-12) and np.isclose(spearmanr(x,y).statistic,row.Spearman,atol=1e-12))
    cases=pd.read_csv(OUT/'relative_uncertainty_residual_cases.csv');check('automatic median split cases',cases.uncertainty_HIGH.equals(cases.projected_logdet_I_plus_C.ge(cases.projected_logdet_I_plus_C.median())) and cases.residual_HIGH.equals(cases.whitened_norm.ge(cases.whitened_norm.median())))
    check('10 figures measured',len(list(OUT.glob('[0-9][0-9]_*.png')))==10)
    decision=json.loads((OUT/'stage17_geometry_decision.json').read_text());check('diagnostic line CLOSED',decision['numerical_diagnostic_line_closed'] and decision['GT_used'] is False and decision['no_new_policy'])
    p=table[table.condition.eq('S1')];full=table[table.condition.eq('F0')].set_index('frozen_loop_id');comparison=pd.read_csv(OUT/'sparse_full_projected_uncertainty.csv')
    for row in comparison.itertuples():
        sparse=p[p.frozen_loop_id.eq(row.frozen_loop_id)].iloc[0];f=full.loc[row.frozen_loop_id]
        check('sparse/full ratio '+str(row.frozen_loop_id),np.isclose(sparse.projected_trace/f.projected_trace,row.projected_trace_ratio,atol=1e-8) and np.isclose(sparse.projected_lambda_max/f.projected_lambda_max,row.projected_lambda_max_ratio,atol=1e-8))
    projection=bool(p.top_pair_mode_common_world_motion_fraction.median()>=.9 and p.top_pair_mode_fraction_projected_trace.median()<=.01 and comparison.projected_trace_ratio.median()>=2)
    comp=pd.read_csv(OUT/'stage16_deltaI_stage17_comparison.csv');mismatch=comp.Stage16_DeltaI.ge(comp.Stage16_DeltaI.median())&comp.all75_translation_p95_reduction.lt(comp.all75_translation_p95_reduction.median())
    material=comp.max_NL_rel_two_sigma.ge(.1)|comp.Huber_weight.le(.5)
    geometry=bool((mismatch&material).sum()>=2 and len(predictive[predictive.predictor.isin(['max_NL_rel_two_sigma','second_difference_curvature_proxy','Huber_weight'])&predictive.Spearman.abs().ge(.4)])>0)
    improved=set()
    for outcome in policy['outcomes']:
        base=float(predictive[predictive.predictor.eq('Stage16_DeltaI')&predictive.outcome.eq(outcome)].iloc[0].Spearman)
        other=predictive[predictive.outcome.eq(outcome)&~predictive.predictor.isin(['Stage16_DeltaI','projected_logdet_I_plus_C'])]
        if ((other.Spearman.abs()>=.4)&(other.Spearman.abs()-abs(base)>=.2)).any():improved.add(outcome)
    full_decision=projection and len(improved)>=2 and geometry and bool(improved&{'all75_translation_p95_reduction','all75_rotation_p95_reduction'})
    expected='RELATIVE_UNCERTAINTY_AND_NONLINEARITY_EXPLAIN_STAGE16_GAP' if full_decision else 'RELATIVE_UNCERTAINTY_AND_NONLINEARITY_PARTIALLY_EXPLAIN_STAGE16_GAP' if projection or geometry else 'NO_CLEAR_ADDITIONAL_EXPLANATION'
    check('frozen conservative decision reproduced',expected==decision['status'] and projection==decision['projection_material'] and geometry==decision['geometry_material'])
    for (fid,mode),group in curves.groupby(['frozen_loop_id','mode']):
        summary=pd.read_csv(OUT/'nonlinear_mode_summary.csv');summ=summary[summary.frozen_loop_id.eq(fid)&summary['mode'].eq(mode)].iloc[0]
        for radius,label in [(.5,'half'),(1.,'one'),(2.,'two')]:
            selected=group[group.alpha.abs().le(radius)&group.alpha.ne(0)]
            check(f'nonlinear aggregate#{fid}/{mode}/{radius}',np.isclose(selected.NL_abs.max(),summ[f'max_NL_abs_{label}_sigma'],atol=1e-12) and np.isclose(selected.NL_rel.max(),summ[f'max_NL_rel_{label}_sigma'],atol=1e-12))
    report=(OUT/'VALIDATION_REPORT.txt').read_text();check('validation reflects decision',decision['status'] in report and 'GT_USED: NO' in report and 'NUMERICAL_DIAGNOSTIC_LINE_CLOSED: YES' in report)
    artifacts=[dict(path=str(p.relative_to(ROOT)),sha256=sha(p),size_bytes=p.stat().st_size) for p in OUT.rglob('*') if p.is_file() and p.name not in ['independent_audit.json','stage17_artifact_manifest.json']]
    (OUT/'stage17_artifact_manifest.json').write_text(json.dumps(artifacts,indent=2)+'\n')
    (OUT/'independent_audit.json').write_text(json.dumps(dict(PASS=True,checks=len(CHECKS),details=CHECKS,GT_USED=False,optimization_runs=0),indent=2)+'\n')
    print(json.dumps(dict(PASS=True,checks=len(CHECKS),optimization_runs=0)))

if __name__=='__main__':main()
