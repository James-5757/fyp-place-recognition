#!/usr/bin/env python3
"""Independent saved-artifact audit; no optimizer, GT reader or retrieval."""
import hashlib
import json
from pathlib import Path
import sys
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/cumulti_v1/16_measurement_information'
OLD=ROOT/'outputs/cumulti_v1/15_informative_loop_selection'
CHECKS=[]

def check(name,ok):
    CHECKS.append(dict(name=name,PASS=bool(ok)))
    if not ok:raise AssertionError(name)

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    policy=json.loads((OUT/'stage16_experiment_policy.json').read_text())
    check('frozen policy unchanged',sha(OUT/'stage16_experiment_policy.json')==json.loads((OUT/'policy_freeze.json').read_text())['sha256'])
    check('no GT/new policy declared',policy['GT_used'] is False and policy['new_sparse_policy'] is False)
    manifest=json.loads((OUT/'stage16_input_manifest.json').read_text())
    for p,h in manifest['historical_before'].items():check('historical '+p,sha(p)==h)
    for entry in manifest['inputs']:check('input '+entry['path'],sha(entry['path'])==entry['sha256'])
    loops=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv')
    prov=json.loads((OUT/'loop_provenance_audit.json').read_text())
    check('120 / 75 / 45 exact provenance',(prov['historical_count'],prov['clean_count'],prov['excluded_count'])==(120,75,45))
    check('no excluded restored',not prov['excluded_restored'] and set(loops.frozen_loop_id)==set(prov['clean_ids']) and not set(prov['clean_ids'])&set(prov['excluded_ids']))
    sets=json.loads((OLD/'selected_loops_by_budget.json').read_text())
    table=pd.read_csv(OUT/'loop_information_gain.csv')
    check('five conditions exactly375 rows',len(table)==375 and set(table.condition)==set(policy['conditions']))
    for code,(selector,K,tag) in policy['conditions'].items():
        g=table[table.condition.eq(code)]
        check(code+' all clean IDs once',len(g)==75 and set(g.frozen_loop_id)==set(prov['clean_ids']) and not g.frozen_loop_id.duplicated().any())
        check(code+' selected exact',set(g[g.selected].frozen_loop_id)==set(sets[selector][str(K)]))
        arrays=np.load(OUT/f'workers/{code}_local_arrays.npz')
        poses=pd.read_csv(OLD/f'trajectories/{tag}.csv').set_index(['robot_id','keyframe_id'])
        for r in g.itertuples():
            C=arrays[f'C_{r.frozen_loop_id}'];J=arrays[f'J_{r.frozen_loop_id}']
            eig,V=np.linalg.eigh(C);sig=np.array(policy['noise']['sigmas']);R=np.diag(sig**2)
            check(f'{code}#{r.frozen_loop_id} covariance',C.shape==(12,12) and np.isfinite(C).all() and np.allclose(C,C.T,atol=1e-8) and eig.min()>=-1e-10*max(1,eig.max()))
            e=np.array(json.loads(r.logmap_residual));P=J@C@J.T;W=P/sig[:,None]/sig[None,:]
            sign,ld=np.linalg.slogdet(np.eye(6)+(W+W.T)/2)
            check(f'{code}#{r.frozen_loop_id} DeltaI',sign>0 and abs(.5*ld-r.DeltaI)<1e-7)
            nis=float(e@np.linalg.solve(P+R,e))
            check(f'{code}#{r.frozen_loop_id} NIS',np.isclose(nis,r.NIS,rtol=1e-8,atol=1e-8))
            H=(J/sig[:,None]).T@(J/sig[:,None]);support=sum(V[:,i]@H@V[:,i] for i in [-1,-2,-3])
            check(f'{code}#{r.frozen_loop_id} weak support',np.isclose(support,r.weak_support_top3,rtol=1e-7,atol=1e-9))
            check(f'{code}#{r.frozen_loop_id} finite difference gate',r.finite_difference_max_error<policy['numerical_gates']['FD_max_abs_lt'])
            loop=loops[loops.frozen_loop_id.eq(r.frozen_loop_id)].iloc[0]
            def T(row):
                mat=np.eye(4);mat[:3,:3]=Rotation.from_quat(row[['qx','qy','qz','qw']].to_numpy(dtype=float)).as_matrix();mat[:3,3]=row[['tx','ty','tz']].to_numpy(dtype=float);return mat
            Z=T(loop);q=T(poses.loc[('robot1',r.query_keyframe_id)]);c=T(poses.loc[('robot3',r.candidate_keyframe_id)])
            E=np.linalg.solve(Z,np.linalg.solve(q,c))
            check(f'{code}#{r.frozen_loop_id} physical residual',np.isclose(np.linalg.norm(E[:3,3]),r.translation_residual_m,atol=1e-8) and np.isclose(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude()),r.rotation_residual_deg,atol=1e-8))
    primary=table[table.condition.eq('S1')];om=primary[~primary.selected]
    selection=json.loads((OUT/'addback_selection.json').read_text());trials=selection['trials']
    expect=[]
    for asc in [False,True]:expect+=om.sort_values(['DeltaI','frozen_loop_id'],ascending=[asc,True]).head(5).frozen_loop_id.to_list()
    check('automatic TOP5 BOTTOM5',len(trials)==10 and [t['frozen_loop_id'] for t in trials]==expect and len(set(expect))==10)
    solver=pd.read_csv(OUT/'addback_solver_results.csv');effects=pd.read_csv(OUT/'addback_residual_effects.csv')
    check('ten independent K41 graphs',len(solver)==10 and solver.nodes.eq(5796).all() and solver.odom_factors.eq(5794).all() and solver.loop_factors.eq(41).all() and solver.prior.eq(1).all() and solver.final_error.lt(solver.initial_error).all())
    canonical=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_first_loop_trajectory.csv')
    for row in solver.itertuples():
        tr=pd.read_csv(OUT/f'trajectories/{row.condition}.csv')
        check(row.condition+' pose schema',len(tr)==5796 and not tr[['robot_id','keyframe_id']].duplicated().any() and np.isfinite(tr[['tx','ty','tz','qx','qy','qz','qw']]).all().all() and np.allclose(np.linalg.norm(tr[['qx','qy','qz','qw']],axis=1),1,atol=1e-9))
        check(row.condition+' original IDs timestamps',tr[['robot_id','keyframe_id','timestamp']].equals(canonical[['robot_id','keyframe_id','timestamp']]))
        check(row.condition+' remaining34',int(effects[effects.condition.eq(row.condition)].iloc[0].remaining34_count)==34)
        poses=tr.set_index(['robot_id','keyframe_id']);ts=[];rs=[]
        for loop in loops.itertuples():
            rr=pd.Series(loop._asdict());Z=T(rr);q=T(poses.loc[('robot1',loop.query_keyframe_id)]);c=T(poses.loc[('robot3',loop.candidate_keyframe_id)])
            E=np.linalg.solve(Z,np.linalg.solve(q,c));ts.append(np.linalg.norm(E[:3,3]));rs.append(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude()))
        er=effects[effects.condition.eq(row.condition)].iloc[0]
        for metric,values in [('translation',ts),('rotation',rs)]:
            unit='m' if metric=='translation' else 'deg'
            for name,computed in [('median',np.median(values)),('p90',np.percentile(values,90)),('p95',np.percentile(values,95)),('max',np.max(values))]:
                check(row.condition+' all75 '+metric+name,np.isclose(computed,er[f'all75_{metric}_{name}_{unit}'],atol=1e-8))
        anchor=tr[tr.robot_id.eq('robot1')&tr.keyframe_id.eq(150)].iloc[0]
        original=canonical[canonical.robot_id.eq('robot1')&canonical.keyframe_id.eq(150)].iloc[0]
        check(row.condition+' frozen anchor',np.max(abs(T(anchor)-T(original)))<1e-7)
        before=om[om.frozen_loop_id.eq(row.frozen_loop_id)].iloc[0]
        C=np.load(OUT/f'workers/{row.condition}_local_arrays.npz')[f'C_{row.frozen_loop_id}'];v=np.array(json.loads(before.weak_v1))
        expected=1-float(v@C@v)/before.max_covariance_eigenvalue
        check(row.condition+' original weak direction reduction',np.isclose(expected,effects[effects.condition.eq(row.condition)].iloc[0].original_weak_direction_fraction_reduction,atol=1e-8))
    statuses=[json.loads(p.read_text()) for p in (OUT/'workers').glob('*_status.json')]
    check('bounded workers fifteen successful',len(statuses)==15 and all(x['PASS'] and x['worker_wall_s']<120 for x in statuses))
    check('ten actual figures',len(list(OUT.glob('[0-9][0-9]_*.png')))==10)
    check('Stage15 remains negative',json.loads((OLD/'stage15_pre_gt_decision.json').read_text())['status']=='NO_INFORMATIVE_SPARSE_POLICY')
    result=dict(PASS=True,checks=len(CHECKS),details=CHECKS,GT_USED=False,optimization_rerun=False)
    (OUT/'independent_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(PASS=True,checks=len(CHECKS))))

if __name__=='__main__':main()
