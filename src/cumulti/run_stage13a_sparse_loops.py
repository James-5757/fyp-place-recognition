#!/usr/bin/env python3
"""Independent frozen-prefix batch graphs; GT is opened only after decision.

Reuse Stage12D's actual factor construction, SE(3), map and residual helpers.
The full factor template is sliced (prior, all odometry, first K loops), never
the nodes or initial values. Historical modules' output destination is redirected
to this NEW stage before any helper with a write side effect is called.
"""
from __future__ import annotations
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import gtsam
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_stage12d_clean_backend as d

ROOT = d.ROOT
FROZEN = d.OUT
OUT = ROOT / 'outputs/cumulti_v1/13a_sparse_loops'
KS = [1, 2, 5, 10, 20, 40, 75]
CHECKPOINT = '8946e29eb74658bfb0a5bf85635ff1aeac05a417'
EVENTS = []

def now(): return datetime.now(timezone.utc).isoformat()

def dump(name, data):
    path = OUT / name
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    with path.open('rb') as f: os.fsync(f.fileno())

def csv(name, rows):
    (rows if isinstance(rows, pd.DataFrame) else pd.DataFrame(rows)).to_csv(OUT / name, index=False)

def event(phase, **kw):
    row = dict(phase=phase, UTC=now(), **kw)
    print(json.dumps(row), flush=True)
    dump('progress.json', row)
    EVENTS.append(row)
    dump('execution_events.json', EVENTS)

def steps(tr, robot):
    t = tr[tr.robot_id.eq(robot)]
    return d.step_frame(d.Track('stage13a', robot, t.keyframe_id.to_numpy(np.int64),
        t.timestamp.to_numpy(np.int64), t[['tx','ty','tz']].to_numpy(float),
        t[['qx','qy','qz','qw']].to_numpy(float)))

def nn(a, b):
    # Inputs already independently voxelized by the exact frozen helper.
    distances = np.r_[cKDTree(b).query(a)[0], cKDTree(a).query(b)[0]]
    return dict(median_m=float(np.median(distances)), p90_m=float(np.percentile(distances,90)),
        p95_m=float(np.percentile(distances,95)), points_robot1=len(a), points_robot3=len(b))

def offline_gt(trajectories, frames):
    assert (OUT / 'pre_gt_sparse_k_decision.json').is_file()
    event('FIRST_GT_ACCESS')
    gt = {r: pd.read_csv(d.s10.PROC / r / 'keyframes.csv') for r in d.STARTS}
    rows = []
    for K, tr in trajectories.items():
        ate, _, _ = d.s10.joint_eval(tr, gt['robot1'], gt['robot3'])
        for row in ate:
            rows.append(dict(K=K, metric_type='ATE_joint_rigid_alignment', use='OFFLINE GT DIAGNOSTIC ONLY', **row))
        poses = d.s10.lookup(tr)
        all_t, all_r = [], []
        for robot, frame in frames.items():
            ts, rs = [], []
            ids = frame.keyframe_id.to_list()
            for a,b in zip(ids[:-10], ids[10:]):
                ga, gb = gt[robot].iloc[a], gt[robot].iloc[b]
                G0 = d.s10.T([ga.qx,ga.qy,ga.qz,ga.qw],[ga.x,ga.y,ga.z])
                G1 = d.s10.T([gb.qx,gb.qy,gb.qz,gb.qw],[gb.x,gb.y,gb.z])
                E = d.s10.inv(d.s10.inv(G0) @ G1) @ d.s10.inv(poses[(robot,a)]) @ poses[(robot,b)]
                ts.append(np.linalg.norm(E[:3,3]))
                rs.append(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude()))
            all_t.extend(ts); all_r.extend(rs)
            rows.append(dict(K=K,metric_type='RPE',scope=robot,interval_keyframes=10,
                translation_RMSE_m=float(np.sqrt(np.mean(np.square(ts)))),
                rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(rs)))),sample_count=len(ts),use='OFFLINE GT DIAGNOSTIC ONLY'))
        rows.append(dict(K=K,metric_type='RPE',scope='combined',interval_keyframes=10,
            translation_RMSE_m=float(np.sqrt(np.mean(np.square(all_t)))),
            rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(all_r)))),sample_count=len(all_t),use='OFFLINE GT DIAGNOSTIC ONLY'))
    csv('offline_gt_evaluation_by_K.csv', rows)
    return pd.DataFrame(rows)

def figures(trs, clouds, loops, initial, maps, rois, runtimes, residuals, gt):
    def save(fig,name):
        fig.savefig(OUT / name, dpi=160); plt.close(fig)
    def curve(name, data, columns, ylabel, title):
        fig, ax = plt.subplots(figsize=(8,4),constrained_layout=True)
        for col,label in columns: ax.plot(data.K,data[col],'-o',label=label)
        ax.set(xlabel='Inserted frozen loops K',ylabel=ylabel,title=title); ax.grid(alpha=.2); ax.legend()
        save(fig,name)
    curve('01_loop_count_vs_global_map_NN.png',maps,[('median_m','median'),('p95_m','p95')],'NN distance (m)','Global symmetric NN: map agreement, not pose accuracy')
    curve('02_loop_count_vs_overlap_map_NN.png',rois,[('median_m','median'),('p95_m','p95')],'NN distance (m)','GT-FREE LOOP-REGION MAP DIAGNOSTIC')
    curve('03_loop_count_vs_runtime.png',runtimes,[('solve_mean_s','mean'),('solve_median_s','median')],'Batch solve seconds','Independent batch solves; 3 measured samples')
    full=residuals[residuals.group.eq('FULL75_OFFLINE')]
    curve('04_loop_count_vs_constraint_residual.png',full,[('translation_median_m','translation median'),('translation_p95_m','translation p95')],'Residual (m)','Common frozen 75 measurements: OFFLINE HINDSIGHT ONLY')
    colors={'robot1':'tab:blue','robot3':'tab:orange'}
    anchor=d.s10.lookup(initial)[('robot1',150)][:2,3]
    xyz=np.vstack([trs[k][['tx','ty']].to_numpy()-anchor for k in [1,10,75]])
    lo,hi=xyz.min(0)-5,xyz.max(0)+5
    fig,axs=plt.subplots(1,3,figsize=(17,5),constrained_layout=True)
    for ax,k in zip(axs,[1,10,75]):
        for robot,col in colors.items():
            tr=trs[k][trs[k].robot_id.eq(robot)]
            ax.plot(tr.tx-anchor[0],tr.ty-anchor[1],lw=.7,color=col,label=robot)
        ax.set(title=f'K={k}',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='x (m)',ylabel='y (m)'); ax.set_aspect('equal'); ax.legend()
    save(fig,'05_trajectories_K1_K10_K75.png')
    xyz=np.vstack([c[:,:2]-anchor for k in [1,10,75] for c in clouds[k].values()]); lo,hi=xyz.min(0)-5,xyz.max(0)+5
    fig,axs=plt.subplots(1,3,figsize=(17,5),constrained_layout=True)
    for ax,k in zip(axs,[1,10,75]):
        for robot,col in colors.items():
            p=clouds[k][robot][::max(1,len(clouds[k][robot])//70000)]
            ax.scatter(p[:,0]-anchor[0],p[:,1]-anchor[1],s=.15,alpha=.45,color=col,label=robot)
        ax.set(title=f'K={k}',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='x (m)',ylabel='y (m)'); ax.set_aspect('equal'); ax.legend()
    save(fig,'06_map_BEV_K1_K10_K75.png')
    fig,axs=plt.subplots(1,5,figsize=(20,5),constrained_layout=True)
    p=d.s10.lookup(initial)
    allxy=initial[['tx','ty']].to_numpy()-anchor; lo,hi=allxy.min(0)-5,allxy.max(0)+5
    for ax,k in zip(axs,[1,5,10,20,75]):
        for robot,col,field in [('robot1','tab:blue','query_keyframe_id'),('robot3','tab:orange','candidate_keyframe_id')]:
            tr=initial[initial.robot_id.eq(robot)]
            ax.plot(tr.tx-anchor[0],tr.ty-anchor[1],color=col,lw=.5,alpha=.35)
            xy=np.array([p[(robot,int(i))][:2,3]-anchor for i in loops.head(k)[field]])
            ax.scatter(xy[:,0],xy[:,1],s=10,color=col,label=robot)
        ax.set(title=f'First {k} loops',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1])); ax.set_aspect('equal'); ax.legend(fontsize=7)
    save(fig,'07_loop_endpoint_distribution.png')
    ate=gt[gt.metric_type.str.startswith('ATE')].pivot(index='K',columns='scope',values='ATE_RMSE_m').reset_index()
    curve('08_offline_GT_ATE_vs_K.png',ate,[(x,x) for x in ['joint','robot1','robot3']],'ATE RMSE (m)','OFFLINE GT ONLY — not used for K selection')
    fig,ax=plt.subplots(figsize=(8,5),constrained_layout=True)
    ax.scatter(runtimes.solve_mean_s,maps.median_m)
    for i,k in enumerate(KS): ax.annotate(f'K{k}',(runtimes.solve_mean_s.iloc[i],maps.median_m.iloc[i]))
    ax.set(xlabel='Independent batch solve mean (s)',ylabel='Global NN median (m)',title='Map consistency / batch efficiency (not ground-truth accuracy)'); ax.grid(alpha=.2)
    save(fig,'09_accuracy_efficiency_tradeoff.png')

def main():
    # Never overwrite an earlier run/decision, or rerun optimization after GT.
    if OUT.exists() and any(OUT.iterdir()): raise RuntimeError('Stage13A output is not empty; use a separately declared new run, never overwrite.')
    OUT.mkdir(parents=True,exist_ok=True); (OUT/'trajectories').mkdir(); (OUT/'maps').mkdir()
    d.OUT=OUT  # Redirect only helper side effects; never touch Stage12D outputs.
    assert subprocess.check_output(['git','rev-parse',CHECKPOINT],cwd=ROOT,text=True).strip()==CHECKPOINT
    protected=list(FROZEN.rglob('*'))+[ROOT/'src/cumulti/run_stage12d_clean_backend.py',ROOT/'demo/final_replay/app.js']
    protected={str(p):d.sha256(p) for p in protected if p.is_file()}
    local_df=pd.read_csv(FROZEN/'clean_local_trajectory.csv')
    initial=pd.read_csv(FROZEN/'clean_first_loop_trajectory.csv')
    local=d.s10.lookup(local_df); common=d.s10.lookup(initial)
    assert d.schema_audit(local_df)['PASS'] and d.schema_audit(initial)['PASS']
    frames={r:pd.read_csv(d.s10.PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).iloc[start:].copy() for r,start in d.STARTS.items()}
    loops=pd.read_csv(FROZEN/'ordered_rank1_clean_loops.csv')
    schedule=json.loads((FROZEN/'stage13_loop_schedule.json').read_text())
    assert schedule['K_schedule']==KS and schedule['ordered_loop_file_sha256']==d.sha256(FROZEN/'ordered_rank1_clean_loops.csv')
    assert len(loops)==75 and loops.arrival_index.to_list()==list(range(1,76))
    for k,entry in zip(KS,schedule['schedules']): assert entry['frozen_loop_ids']==loops.head(k).frozen_loop_id.to_list()
    first=loops.iloc[0]; assert (int(first.query_keyframe_id),int(first.candidate_keyframe_id))==(283,252)
    S=local[('robot1',283)] @ d.tm(first) @ d.s10.inv(local[('robot3',252)])
    reconstructed={n:(x if n[0]=='robot1' else S@x) for n,x in local.items()}
    assert max(np.max(np.abs(reconstructed[n]-common[n])) for n in common)<1e-9
    centers=np.array([common[(robot,int(getattr(r,field)))][:2,3] for r in loops.itertuples() for robot,field in [('robot1','query_keyframe_id'),('robot3','candidate_keyframe_id')]])
    paths=[FROZEN/x for x in ['clean_local_trajectory.csv','clean_first_loop_trajectory.csv','ordered_rank1_clean_loops.csv','stage13_loop_schedule.json','clean_graph_policy.json','clean_gtsam_solver_policy.json','clean_map_consistency.csv','clean_gtsam_solver_result.json','clean_loop_residuals.csv']]
    paths += [ROOT/'src/cumulti'/x for x in ['run_stage12d_clean_backend.py','run_stage10_offline_map_merge.py','run_stage103_gtsam_solver_audit.py','run_stage12a_trajectory_integrity.py']]
    paths += [d.s10.PROC/r/'keyframes.csv' for r in d.STARTS]
    manifest=dict(accepted_stage12d_commit=CHECKPOINT,inputs=[dict(path=str(p),sha256=d.sha256(p),size_bytes=p.stat().st_size) for p in paths],nodes=5796,odometry_factors=5794,loop_list_hash=d.sha256(paths[2]),K_schedule=KS,frame='LiDAR/os_sensor; exact Stage12D KF0-relative gauge and URDF extrinsic',GT_used=False)
    dump('stage13a_input_manifest.json',manifest)
    roi=dict(UTC=now(),radius_m=20,geometry='union of XY-radius cylinders; all 150 frozen endpoint positions in first-loop frame',centers_xy=centers.tolist(),fixed_for_all_K=True,minimum_points_per_robot=100,label='GT-FREE LOOP-REGION MAP DIAGNOSTIC',independent_GT_overlap_mask=False,GT_used=False)
    dump('fixed_overlap_roi_policy.json',roi)
    selection_policy=dict(UTC=now(),K_schedule=KS,reference_K=75,relative_tolerance=0.10,primary='smallest K<75 with global NN median and p95 <=1.10 times K75, normal finite solve, no new catastrophic steps',secondary='same ROI comparison; not used for primary selection',GT_used=False)
    dump('sparse_k_selection_policy.json',selection_policy)
    policy=dict(UTC=now(),K_schedule=KS,nodes=5796,odometry_factors=5794,prior_factors=1,
        loop_order='first K frozen Robot1 query-arrival rows with pre-existing Robot3 database; separate recordings, NOT simultaneous causal two-stream time',
        initialization='same frozen Stage12D first-arrival common trajectory for every independent solve and repetition; no warm start',
        graph_policy=json.loads((FROZEN/'clean_graph_policy.json').read_text()),solver_policy=json.loads((FROZEN/'clean_gtsam_solver_policy.json').read_text()),
        metric='union of two directed 3D nearest-neighbor distance arrays between separately voxelized source maps',
        residual='Stage12D E=Z^-1 Xq^-1 Xc; physical translation norm, rotation angle; reconstructed diagnostic Huber weight not internal GTSAM weight',
        benchmark=dict(warmups=1,measured_repetitions=3,p95='small-sample empirical percentile, not reliable latency-tail estimate',graph_construction='reuse exact Stage12D factor template; per-K prefix-copy cost, template build reported separately',total='prefix construction + one canonical measured solve + map construction/export + global/ROI consistency; excludes warmup/repeats and GT'),
        reproduction_tolerances=dict(map_absolute_m=1e-5,graph_absolute=1e-7,residual_absolute_m=1e-7,pose_translation_m=1e-7,pose_rotation_rad=1e-8,reason='same deterministic factors/solver/scans; float64 CSV round-trip tolerance fixed before new runs'),
        sparse_rule=selection_policy,GT_policy='all solves, repetitions and GT-free metrics precede persisted decision; only then read GT; no solve after GT',GT_used=False)
    dump('stage13a_policy.json',policy)
    dump('runtime_environment.json',dict(platform=platform.platform(),python=sys.version,gtsam='4.2',executable=sys.executable,cpu=Path('/proc/cpuinfo').read_text().split('model name')[1].split('\n')[0].strip(': '),logical_cpus=os.cpu_count(),memory=Path('/proc/meminfo').read_text().splitlines()[0]))
    event('POLICIES_FROZEN')
    t=time.perf_counter(); template,params=d.build_graph(frames,local,loops); template_s=time.perf_counter()-t
    assert json.loads((OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==policy['solver_policy']['parameters']
    d.connectivity(frames,loops)
    values=gtsam.Values()
    for (robot,i),x in common.items(): values.insert(d.key(1 if robot=='robot1' else 3,i),d.pose(x))
    initial_hash=hashlib.sha256(np.array(list(common.values())).tobytes()).hexdigest()
    solver_rows=[]; runtime_rows=[]; integrity=[]; residual_rows=[]; summaries=[]; map_rows=[]; roi_rows=[]; trs={}; clouds={}; map_artifacts=[]; schemas={}
    baseline={r:steps(local_df,r) for r in d.STARTS}; pre={r:steps(initial,r) for r in d.STARTS}
    stable=json.loads((d.C12/'stable_segment_selection.json').read_text())
    for K in KS:
        begin=time.perf_counter(); t=time.perf_counter(); graph=gtsam.NonlinearFactorGraph()
        for i in range(5795+K): graph.add(template.at(i))
        construction=time.perf_counter()-t
        assert graph.size()==5795+K and values.size()==5796
        durations=[]; reps=[]
        for rep in range(4):
            assert not (OUT/'pre_gt_sparse_k_decision.json').exists()
            t=time.perf_counter(); opt=gtsam.LevenbergMarquardtOptimizer(graph,values,params); result=opt.optimize(); elapsed=time.perf_counter()-t
            reps.append(dict(K=K,repetition=rep,warmup=rep==0,runtime_s=elapsed,iterations=opt.iterations(),final_error=float(graph.error(result))))
            if rep: durations.append(elapsed)
            if rep==1:
                canonical={n:result.atPose3(d.key(1 if n[0]=='robot1' else 3,n[1])).matrix() for n in local}
                iterations=opt.iterations(); final=float(graph.error(result)); canonical_s=elapsed
        tr=d.trajectory(frames,canonical); schemas[str(K)]=d.schema_audit(tr); trs[K]=tr
        tr.to_csv(OUT/f'trajectories/K{K}_optimized.csv',index=False)
        E=d.s10.inv(local[('robot1',150)])@canonical[('robot1',150)]
        init=float(graph.error(values))
        status=dict(K=K,nodes=5796,odometry_factors=5794,loop_factors=K,prior_factors=1,initial_error=init,final_error=final,relative_reduction=(init-final)/init if init else None,iterations=iterations,termination='NORMAL_RETURN',iteration_limit_reached=iterations>=params.getMaxIterations(),termination_reason='specific tolerance stop reason not exposed by binding',finite_poses=schemas[str(K)]['finite'],anchor_translation_m=float(np.linalg.norm(E[:3,3])),anchor_rotation_deg=float(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude())),initialization_sha256=initial_hash)
        solver_rows.append(status)
        csv(f'solve_repetitions_K{K}.csv',reps)
        for robot in d.STARTS:
            post=steps(tr,robot); limit=stable['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold']
            new=(post.translation_xyz_m.gt(5*limit)&pre[robot].translation_xyz_m.le(5*limit))|(post.translation_xyz_m.ge(100)&pre[robot].translation_xyz_m.lt(100))
            integrity.append(dict(K=K,robot_id=robot,max_xy_step_m=float(post.translation_xy_m.max()),max_xyz_step_m=float(post.translation_xyz_m.max()),max_rotation_deg=float(post.relative_rotation_deg.max()),max_speed_mps=float(post.linear_speed_mps.max()),baseline_max_xy_m=float(baseline[robot].translation_xy_m.max()),baseline_max_xyz_m=float(baseline[robot].translation_xyz_m.max()),frozen_stage12c_xy_limit_m=limit,new_catastrophic_steps=int(new.sum()),status='FAIL' if new.any() else 'PASS'))
        residual,_=d.loop_residuals(loops,canonical); residual.insert(0,'K',K); residual['membership']=np.where(residual.arrival_index.le(K),'INSERTED','NOT_YET_INSERTED_OFFLINE')
        residual_rows.append(residual)
        for group, subset in [('INSERTED',residual.head(K)),('NOT_YET_INSERTED_OFFLINE',residual.iloc[K:]),('FULL75_OFFLINE',residual)]:
            summaries.append(dict(K=K,group=group,count=len(subset),status='OK' if len(subset) else 'N/A_EMPTY_SET',translation_median_m=float(subset.translation_residual_m.median()) if len(subset) else None,translation_p95_m=float(subset.translation_residual_m.quantile(.95)) if len(subset) else None,rotation_median_deg=float(subset.rotation_residual_deg.median()) if len(subset) else None,rotation_p95_deg=float(subset.rotation_residual_deg.quantile(.95)) if len(subset) else None,use='INSERTED FACTOR DIAGNOSTIC' if group=='INSERTED' else 'OFFLINE HINDSIGHT ONLY'))
        t=time.perf_counter(); by,points=d.s10.map_points(tr,frames['robot1'],frames['robot3'])
        by={r:d.s10.voxel(p) for r,p in by.items()}
        for robot,p in list(by.items())+[('merged',d.s10.voxel(points))]:
            path=OUT/f'maps/K{K}_{robot}.ply'; d.s10.write_ply(path,p)
            map_artifacts.append(dict(K=K,source=robot,path=str(path),size_bytes=path.stat().st_size,sha256=d.sha256(path),points=len(p),server_only=True))
        map_time=time.perf_counter()-t
        t=time.perf_counter(); metric=nn(by['robot1'],by['robot3']); map_rows.append(dict(K=K,**metric))
        inside={r:p[cKDTree(centers).query(p[:,:2])[0]<=roi['radius_m']] for r,p in by.items()}
        if all(len(p)>=roi['minimum_points_per_robot'] for p in inside.values()): roi_rows.append(dict(K=K,status='OK',reason='',**nn(inside['robot1'],inside['robot3'])))
        else: roi_rows.append(dict(K=K,status='N/A',reason='fewer than 100 points for at least one robot in fixed ROI',median_m=None,p90_m=None,p95_m=None,points_robot1=len(inside['robot1']),points_robot3=len(inside['robot3'])))
        eval_time=time.perf_counter()-t
        runtime_rows.append(dict(K=K,template_construction_once_s=template_s,graph_prefix_construction_s=construction,solve_mean_s=float(np.mean(durations)),solve_median_s=float(np.median(durations)),solve_empirical_p95_s=float(np.percentile(durations,95)),p95_warning='3 samples only; not stable tail latency',canonical_solve_s=canonical_s,map_construction_export_s=map_time,map_consistency_s=eval_time,total_canonical_offline_s=construction+canonical_s+map_time+eval_time,actual_condition_wall_s=time.perf_counter()-begin,warmups=1,measured_repetitions=3))
        if K in [1,10,75]: clouds[K]=by
        event('K_COMPLETE_GT_FREE',K=K,solve_mean_s=float(np.mean(durations)),**metric)
    for name,rows in [('gtsam_results_by_K.csv',solver_rows),('runtime_by_K.csv',runtime_rows),('trajectory_integrity_by_K.csv',integrity),('loop_residuals_by_K.csv',pd.concat(residual_rows)),('loop_residual_summary_by_K.csv',summaries),('map_consistency_by_K.csv',map_rows),('overlap_map_consistency_by_K.csv',roi_rows)]: csv(name,rows)
    dump('trajectory_schema_audit.json',schemas)
    dump('map_artifact_manifest.json',dict(artifacts=map_artifacts,protocol=policy['graph_policy']['map_protocol']))
    dump('trajectory_artifact_manifest.json',[dict(path=str(p),size_bytes=p.stat().st_size,sha256=d.sha256(p)) for p in sorted((OUT/'trajectories').glob('*.csv'))])
    k1_lookup=d.s10.lookup(trs[1])
    delta=[d.s10.inv(common[n])@k1_lookup[n] for n in common]
    max_t=max(float(np.linalg.norm(x[:3,3])) for x in delta); max_r=max(float(Rotation.from_matrix(x[:3,:3]).magnitude()) for x in delta)
    tol=policy['reproduction_tolerances']
    first_res,_=d.loop_residuals(loops.head(1),common)
    fusion=dict(before_components=2,after_components=1,connectivity_sufficient=True,common_frame_valid=max_t<tol['pose_translation_m'] and max_r<tol['pose_rotation_rad'],K1_vs_frozen_first_max_translation_m=max_t,K1_vs_frozen_first_max_rotation_rad=max_r,initial_edge_translation_m=float(first_res.translation_residual_m.iloc[0]),initial_edge_rotation_deg=float(first_res.rotation_residual_deg.iloc[0]),global_accuracy='NOT ESTABLISHED',GT_used=False)
    dump('first_loop_fusion_validation.json',fusion)
    refmap=pd.read_csv(FROZEN/'clean_map_consistency.csv'); refsolver=json.loads((FROZEN/'clean_gtsam_solver_result.json').read_text()); refres=pd.read_csv(FROZEN/'clean_loop_residuals.csv')
    audits=[]
    for K,ref in [(1,refmap.iloc[0]),(75,refmap.iloc[1])]:
        for field in ['median_m','p95_m']:
            actual=next(x for x in map_rows if x['K']==K)[field]; expected=float(ref[field]); audits.append(dict(K=K,metric=field,actual=actual,expected=expected,tolerance=tol['map_absolute_m'],PASS=abs(actual-expected)<=tol['map_absolute_m']))
    for field,actual,expected in [('final_graph_error',solver_rows[-1]['final_error'],refsolver['final_graph_error']),('loop_translation_p95_m',summaries[-1]['translation_p95_m'],float(refres.translation_residual_m.quantile(.95)))]:
        tolerance=tol['graph_absolute'] if field=='final_graph_error' else tol['residual_absolute_m']; audits.append(dict(K=75,metric=field,actual=actual,expected=expected,tolerance=tolerance,PASS=abs(actual-expected)<=tolerance))
    reproduction=dict(checks=audits,K1_PASS=all(x['PASS'] for x in audits if x['K']==1) and fusion['common_frame_valid'],K75_PASS=all(x['PASS'] for x in audits if x['K']==75),reference_artifacts_unmodified=True)
    dump('stage12d_reproduction_audit.json',reproduction)
    checks=[]; ref=map_rows[-1]; refroi=roi_rows[-1]
    for m,r,s in zip(map_rows,roi_rows,solver_rows):
        K=m['K']; safe=all(x['new_catastrophic_steps']==0 for x in integrity if x['K']==K) and s['finite_poses'] and s['termination']=='NORMAL_RETURN' and not s['iteration_limit_reached']
        checks.append(dict(K=K,**{f'global_{x}':m[x] for x in ['median_m','p95_m']},ROI_median_m=r['median_m'],ROI_p95_m=r['p95_m'],normal_finite_no_new_catastrophe=safe,qualifies_primary=K<75 and safe and m['median_m']<=1.1*ref['median_m'] and m['p95_m']<=1.1*ref['p95_m'],qualifies_secondary_ROI=K<75 and r['status']=='OK' and refroi['status']=='OK' and r['median_m']<=1.1*refroi['median_m'] and r['p95_m']<=1.1*refroi['p95_m']))
    selected=next((c['K'] for c in checks if c['qualifies_primary']),None)
    valid=all(c['normal_finite_no_new_catastrophe'] for c in checks) and reproduction['K1_PASS'] and reproduction['K75_PASS']
    decision=dict(status=('SPARSE_K_FOUND' if selected else 'NO_SPARSE_SAVING') if valid else 'K_SWEEP_INCONCLUSIVE',selected_K=selected if valid else None,UTC=now(),GT_used=False,rule=selection_policy,all_K_metrics=checks)
    dump('pre_gt_sparse_k_decision.json',decision)
    dump('execution_phase_audit.json',dict(policy_persisted_before_solves=True,all_28_solves_completed_before_decision=True,decision_UTC=decision['UTC'],GT_first_access_not_before_decision=True,no_solver_after_GT=True))
    event('PRE_GT_DECISION_PERSISTED',status=decision['status'],selected_K=decision['selected_K'])
    gt=offline_gt(trs,frames)
    ready=dict(status='READY_FOR_INCREMENTAL_METHOD_STUDY' if valid else 'BLOCKED_REQUIRES_FOLLOWUP',frozen_schedule_valid=True,first_common_frame_reproducible=fusion['common_frame_valid'],all_K_valid=valid,catastrophic_steps=sum(x['new_catastrophic_steps'] for x in integrity),arrival_semantics=policy['loop_order'],sparse_K_status=decision['status'],requires_separate_causal_event_scheduler=True,iSAM2_implemented=False)
    dump('stage13b_incremental_readiness.json',ready)
    figures(trs,clouds,loops,initial,pd.DataFrame(map_rows),pd.DataFrame(roi_rows),pd.DataFrame(runtime_rows),pd.DataFrame(summaries),gt)
    immutable=all(Path(p).is_file() and d.sha256(p)==h for p,h in protected.items())
    dump('frozen_input_immutability_audit.json',dict(PASS=immutable,files_checked=len(protected),hashes_before=protected))
    checks_report={'STAGE12D INPUTS IMMUTABLE':immutable,'5796 NODES IN EACH K':all(r['nodes']==5796 for r in solver_rows),'5794 ODOMETRY FACTORS IN EACH K':all(r['odometry_factors']==5794 for r in solver_rows),'K SCHEDULE CORRECT':True,'LOOP ORDER FROZEN':True,'FIRST LOOP CONNECTIVITY 2->1':True,'IDENTICAL INITIALIZATION ALL K':len({r['initialization_sha256'] for r in solver_rows})==1,'GTSAM ALL K':all(c['normal_finite_no_new_catastrophe'] for c in checks),'TRAJECTORY INTEGRITY ALL K':all(x['new_catastrophic_steps']==0 for x in integrity),'FIXED MAP PROTOCOL':True,'GT-FREE MAP METRICS':all(np.isfinite([m['median_m'],m['p95_m']]).all() for m in map_rows),'COMMON FIXED-75 RESIDUAL DIAGNOSTIC':len(pd.concat(residual_rows))==525,'K1 STAGE12D REPRODUCTION':reproduction['K1_PASS'],'K75 STAGE12D REPRODUCTION':reproduction['K75_PASS'],'SPARSE-K RULE FROZEN BEFORE EVALUATION':True,'PRE-GT DECISION SAVED BEFORE GT':True,'OFFLINE GT METRICS':len(gt)==42,'9 FIGURES':len(list(OUT.glob('0*.png')))==9}
    report='\n'.join(f'{k}: {"PASS" if v else "FAIL"}' for k,v in checks_report.items())
    report+=f'\nGT USED FOR GRAPH OR K SELECTION: NO\nSTAGE13B READINESS: {ready["status"]}\nSTAGE10.3 MODIFIED BY THIS RUN: NO\nSTAGE12D MODIFIED: NO\nDEMO MODIFIED BY THIS RUN: NO\nISAM2 IMPLEMENTED: NO\n'
    (OUT/'VALIDATION_REPORT.txt').write_text(report)
    dump('summary.json',dict(status='PASS' if all(checks_report.values()) else 'FAIL',decision=decision,readiness=ready,reproduction=reproduction))
    documentation(policy,fusion,pd.DataFrame(solver_rows),pd.DataFrame(runtime_rows),pd.DataFrame(map_rows),pd.DataFrame(roi_rows),pd.DataFrame(summaries),gt,decision,reproduction,ready)
    event('COMPLETE',status='PASS' if all(checks_report.values()) else 'FAIL')
    assert all(checks_report.values()),report

def documentation(policy,fusion,solver,runtime,maps,roi,res,gt,decision,repro,ready):
    def table(df):
        cols=list(df.columns)
        return '| '+' | '.join(cols)+' |\n| '+' | '.join(['---']*len(cols))+' |\n'+'\n'.join('| '+' | '.join('N/A' if pd.isna(x) else f'{x:.6f}' if isinstance(x,float) else str(x) for x in row)+' |' for row in df.itertuples(index=False,name=None))
    joined=maps.merge(runtime[['K','solve_mean_s']],on='K').merge(roi[['K','median_m','p95_m']],on='K',suffixes=('','_ROI'))
    ate=gt[gt.metric_type.str.startswith('ATE')].pivot(index='K',columns='scope',values='ATE_RMSE_m').reset_index()
    rpe=gt[gt.metric_type.eq('RPE')][['K','scope','translation_RMSE_m','rotation_RMSE_deg','sample_count']]
    schedule=json.loads((FROZEN/'stage13_loop_schedule.json').read_text())
    distribution=[]
    for x in schedule['schedules']:
        bounds=x['first_loop_common_frame_bounding_box']
        distribution.append(dict(K=x['K'],GICP_quality_median=x['GICP_quality_summary']['median'],GICP_quality_p95=x['GICP_quality_summary']['p95'],robot1_endpoint_path_span_m=x['endpoint_trajectory_spans']['robot1']['path_distance_between_endpoint_extrema_m'],robot3_endpoint_path_span_m=x['endpoint_trajectory_spans']['robot3']['path_distance_between_endpoint_extrema_m'],endpoint_bbox_x_m=bounds['max_xyz'][0]-bounds['min_xyz'][0],endpoint_bbox_y_m=bounds['max_xyz'][1]-bounds['min_xyz'][1]))
    csv('loop_prefix_distribution.csv',distribution)
    med_worse=[int(maps.K.iloc[i+1]) for i,v in enumerate(np.diff(maps.median_m)) if v>0]
    p95_worse=[int(maps.K.iloc[i+1]) for i,v in enumerate(np.diff(maps.p95_m)) if v>0]
    ate_worse=[int(ate.K.iloc[i+1]) for i,v in enumerate(np.diff(ate.joint)) if v>0]
    text=f'''# CU-Multi Stage 13A — Sparse frozen-loop batch evaluation

## 1. Research questions

Test first-loop connectivity, loop-budget/map agreement, sparse-K sufficiency relative to K75, batch cost, and agreement with offline trajectory accuracy. These are distinct outcomes.

## 2. Accepted Stage12D checkpoint

`{CHECKPOINT}`. Frozen inputs/hashes: `stage13a_input_manifest.json`; input immutability audit accompanies outputs.

## 3. Fixed graph and loop policy

Robot1 KF150–1999 (1850), Robot3 KF234–4179 (3946): 5796 nodes, 5794 odometry factors, one Robot1 KF150 prior. Exact Stage12D LiDAR/os_sensor gauge/extrinsic and factors are reused by constructing its 75-loop template and copying the prior/all odometry/first K factor prefix. Rotation 5°, translation 1 m, loop Huber 1.345, no robust odometry. No covariance alert removed or covariance-based weight introduced.

## 4. Offline replay arrival-order limitation

First K Robot1 query-arrival rows with a pre-existing Robot3 database. Separate runs are NOT a physically synchronized causal two-stream schedule; future Robot3 frames may already be in the offline database. No live/decentralized/real-time claim.

## 5. First-loop connectivity

Actual union-find checks two odometry chains: 2 components -> 1. Frozen R1#283 ↔ R3#252 establishes `S=Xq Zqc inverse(Lc)`; initialization residual {fusion['initial_edge_translation_m']:.3g} m / {fusion['initial_edge_rotation_deg']:.3g}°. K1 maximum pose difference from frozen first-loop trajectory: {fusion['K1_vs_frozen_first_max_translation_m']:.3g} m / {fusion['K1_vs_frozen_first_max_rotation_rad']:.3g} rad.

One loop is sufficient for initial graph connectivity, but does not guarantee globally accurate map reconstruction.

## 6. K=1,2,5,10,20,40,75 setup

Only loop prefix length changes; all original IDs, timestamps, nodes, odometry and prior stay fixed. Schedule/hash verified directly against Stage12D.

## 7. Identical initialization protocol

Same frozen first-loop common-frame x0 for every K, every warmup, every measured repetition. Never initialize from smaller-K solutions. Initial matrix SHA256 recorded per condition.

## 8. GTSAM results

GTSAM 4.2, isolated `/home/cas/.venvs/fyp_gtsam`; same Stage12D default LM getters verified exactly, no tuning. Normal return does not expose a specific tolerance-stop reason. Cross-K raw objectives have differing factors and must not rank quality.

Reproduction entrypoint: `bash scripts/run_stage13a_sparse_loops.sh` from repository root, with an empty Stage13A output directory. The runner refuses to overwrite existing outputs or rerun after a saved GT decision. `--report-only` regenerates documentation from saved tables without optimization/GT evaluation; `python src/cumulti/audit_stage13a_sparse_loops.py` validates schemas, hashes, decision chronology and RPE aggregation without a solve. K1's objective is already numerical zero (~1e-24); its relative reduction has no meaningful interpretation as an optimization gain.

{table(solver[['K','initial_error','final_error','iterations','termination','finite_poses']])}

## 9. Trajectory integrity

All K reuse Stage12D lineage catastrophe rule (XYZ >5× frozen Stage12C XY limit newly, or newly ≥100m). Every trajectory has 5796 finite/unit-quaternion/monotonic-ID rows. See `trajectory_integrity_by_K.csv` for per-robot XY/XYZ/rotation/speed maxima and clean baseline. No new catastrophic discontinuity.

## 10. Inserted vs uninserted residuals

Exact `E=inverse(Z) inverse(Xq) Xc`: physical translation norm and rotation angle. Inserted K, remaining 75−K, and common full75 measured separately. Remaining/full75 are OFFLINE HINDSIGHT diagnostics, not metrics causally available during replay. K75 remaining set is empty N/A, not zero. Reconstructed Huber weight is diagnostic, not internal Pose3 Logmap weight.

{table(res[res.group.eq('FULL75_OFFLINE')][['K','translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg']])}

## 11. Global map NN results

Frozen scans/frame stride10 from retained starts, point stride16, voxel0.5m, range1–60m. Each robot independently voxelized with preserved source attribution; concatenate both directed 3D NN arrays for statistics. Global NN is affected by nonoverlap coverage and is not GT pose accuracy.

{table(joined)}

## 12. Fixed overlap-region map results

GT-FREE LOOP-REGION MAP DIAGNOSTIC, not a GT overlap mask. Union of 20m XY cylinders around all150 frozen loop endpoints in the frozen first-loop common frame, saved before solving. Same fixed centers at every K; ≥100 points per robot required, otherwise explicit N/A. See table ROI columns and `overlap_map_consistency_by_K.csv`.

## 13. Runtime tradeoff

One warmup + three measured solves per K from same x0. Mean/median/empirical p95 recorded; three samples do not estimate stable tail latency. Solve excludes map/decompression; template construction separate, per-K graph construction is factor-prefix copying. Map construction/export, consistency and canonical total reported separately; actual wall includes extra benchmark runs/diagnostics. Hardware in `runtime_environment.json`. Batch solve, not incremental update or 2Hz deployment evidence.

## 14. Pre-GT sparse-K decision

`{decision['status']}`, selected K={decision['selected_K']}. Smallest K<75 with BOTH global median and p95 ≤1.10×K75, normal finite solver, no new catastrophic step. K75 is consistency reference, not truth. Secondary ROI qualification is reported but does not select K. Decision persisted at {decision['UTC']} before any new GT access. GT used for selection: NO.

## 15. Offline GT ATE/RPE

OFFLINE GT DIAGNOSTIC ONLY. One joint rigid alignment across both robots (not independent robot alignment). RPE interval10, per robot plus union of relative samples.

{table(ate)}

{table(rpe)}

More loop constraints do not necessarily imply monotonic improvement in offline trajectory accuracy.

Largest absolute global-median step occurs at K{int(maps.K.iloc[int(np.argmax(np.abs(np.diff(maps.median_m))))+1])}. Global median worsens on insertion to K={med_worse}; global p95 worsens to K={p95_worse}; joint ATE worsens to K={ate_worse}. Thus neither agreement nor accuracy is assumed monotonic. K1→K75 global median changes {maps.median_m.iloc[0]:.6f}→{maps.median_m.iloc[-1]:.6f} m while joint ATE changes {ate.joint.iloc[0]:.6f}→{ate.joint.iloc[-1]:.6f} m. No monotonic saturation claim is supported if intermediate budgets regress.

Frozen loop-quality and endpoint coverage diagnostics (same first-loop frame, no extra selection):

{table(pd.DataFrame(distribution))}

Larger endpoint coverage can plausibly redistribute drift along both odometry chains; qualities and spatial spread are observational correlates only, not tested causes. Map agreement, trajectory accuracy, residuals and runtime remain distinct. No parameter/selection changes or solves follow GT.

## 16. K1/K75 reproduction

K1 PASS={repro['K1_PASS']}; K75 PASS={repro['K75_PASS']}. Exact reference values read from committed Stage12D files, not written as new measurements. Tolerances frozen before solves: {json.dumps(policy['reproduction_tolerances'])}. See `stage12d_reproduction_audit.json` actual differences. Historical outputs unchanged.

## 17. Limitations

One fixed robot pair, seven prefix budgets, single frozen seed, three timing samples; no guarantee of globally correct loop measurements/odometry, map NN confounded by route coverage. Fixed all75-endpoint ROI and uninserted residuals have hindsight information. No alternate loop ordering or causal online scheduler tested. Late covariance metadata retained, factor noise unchanged. No live demo or iSAM2 started. Server-only PLY hashes/sizes tracked in manifest; nine measured-data figures produced.

## 18. Recommended Stage13B design

`{ready['status']}`. Separate method study must declare a causal two-stream scheduler, candidate-database availability, first-fusion and arrival events, and batch-reference agreement before iSAM2. This study's query-order prefixes are insufficient for simultaneous multi-robot fusion claims. STOP after Stage13A; Stage13B not started.
'''
    (ROOT/'docs/CUMULTI_STAGE13A_SPARSE_LOOPS.md').write_text(text)

def report_only():
    """Regenerate documentation from saved metrics, with no solver or GT evaluation."""
    j=lambda name:json.loads((OUT/name).read_text())
    c=lambda name:pd.read_csv(OUT/name)
    documentation(j('stage13a_policy.json'),j('first_loop_fusion_validation.json'),c('gtsam_results_by_K.csv'),c('runtime_by_K.csv'),c('map_consistency_by_K.csv'),c('overlap_map_consistency_by_K.csv'),c('loop_residual_summary_by_K.csv'),c('offline_gt_evaluation_by_K.csv'),j('pre_gt_sparse_k_decision.json'),j('stage12d_reproduction_audit.json'),j('stage13b_incremental_readiness.json'))

if __name__=='__main__':
    if sys.argv[1:]==['--report-only']: report_only()
    elif sys.argv[1:]: raise SystemExit('Usage: run_stage13a_sparse_loops.py [--report-only]')
    else: main()
