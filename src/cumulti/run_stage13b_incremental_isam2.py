#!/usr/bin/env python3
"""Prebuilt Robot3 + streamed Robot1; archived factors, genuine persistent ISAM2.

Stage12D factor objects are reused exactly, not reconstructed coordinates.
Primary policy: installed default iSAM2, one update per arriving event, no extra
iterations/relinearization/GT tuning to make disagreement disappear.
"""
from __future__ import annotations
import importlib.metadata
import json
import os
import platform
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
import run_stage13a_sparse_loops as a
d=a.d
ROOT=d.ROOT
D12=a.FROZEN
A13=a.OUT
OUT=ROOT/'outputs/cumulti_v1/13b_incremental_isam2'
CHECKPOINT='636d255d18202d2dae7aa4f2ea23827cbf08701c'
EVENTS=[]

def dump(name,data):
    p=OUT/name
    p.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    with p.open('rb') as f: os.fsync(f.fileno())

def csv(name,rows):
    (rows if isinstance(rows,pd.DataFrame) else pd.DataFrame(rows)).to_csv(OUT/name,index=False)

def event(phase,**kw):
    row=dict(phase=phase,UTC=a.now(),**kw); EVENTS.append(row)
    print(json.dumps(row),flush=True); dump('progress.json',row); dump('execution_events.json',EVENTS)

def numeric_key(node): return d.key(1 if node[0]=='robot1' else 3,node[1])

def to_values(matrices):
    v=gtsam.Values()
    for n,x in matrices.items(): v.insert(numeric_key(n),d.pose(x))
    return v

def matrices_from(v,nodes):
    return {n:v.atPose3(numeric_key(n)).matrix() for n in nodes}

def graph_subset(template,indices):
    g=gtsam.NonlinearFactorGraph()
    for i in indices: g.add(template.at(i))
    return g

def pose_difference(left,right):
    ts=[];rs=[]
    assert set(left)==set(right)
    for n in left:
        ts.append(np.linalg.norm(left[n][:3,3]-right[n][:3,3]))
        rs.append(np.rad2deg(Rotation.from_matrix(left[n][:3,:3].T@right[n][:3,:3]).magnitude()))
    return dict(translation_median_m=float(np.median(ts)),translation_p95_m=float(np.percentile(ts,95)),rotation_median_deg=float(np.median(rs)),rotation_p95_deg=float(np.percentile(rs,95)))

def schema(tr,frames):
    assert len(tr)==sum(len(f) for f in frames.values())
    assert not tr.duplicated(['robot_id','keyframe_id']).any()
    assert np.isfinite(tr[d.POSE_COLUMNS[3:]].to_numpy()).all()
    assert np.allclose(np.linalg.norm(tr[['qx','qy','qz','qw']],axis=1),1,atol=1e-8)
    for r,f in frames.items():
        t=tr[tr.robot_id.eq(r)]
        assert t.keyframe_id.to_list()==f.keyframe_id.to_list()
        assert np.all(np.diff(t.timestamp.to_numpy(np.int64))>0)
    return True

def integrity(tr,baseline,common,selection,label,checkpoint):
    rows=[]
    for robot in d.STARTS:
        post=a.steps(tr,robot); ids=post.keyframe_j.to_numpy()
        pre=a.steps(common[common.keyframe_id.le(int(tr[tr.robot_id.eq(robot)].keyframe_id.max()))|~common.robot_id.eq(robot)],robot)
        local=a.steps(baseline,robot).iloc[:len(post)]
        assert pre.keyframe_j.to_list()==post.keyframe_j.to_list()
        limit=selection['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold']
        new=(post.translation_xyz_m.gt(5*limit)&pre.translation_xyz_m.le(5*limit))|(post.translation_xyz_m.ge(100)&pre.translation_xyz_m.lt(100))
        rows.append(dict(checkpoint=checkpoint,solver=label,robot_id=robot,steps=len(post),max_xy_m=float(post.translation_xy_m.max()),max_xyz_m=float(post.translation_xyz_m.max()),max_rotation_deg=float(post.relative_rotation_deg.max()),max_speed_mps=float(post.linear_speed_mps.max()),baseline_max_xy_m=float(local.translation_xy_m.max()),baseline_max_xyz_m=float(local.translation_xyz_m.max()),new_catastrophic_steps=int(new.sum())))
    assert all(x['new_catastrophic_steps']==0 for x in rows),'STOP: new catastrophic trajectory jump'
    return rows

def make_map(tr,frames,name,scope,do_metric=True):
    begin=time.perf_counter(); by,allpoints=d.s10.map_points(tr,frames['robot1'],frames['robot3'])
    by={r:d.s10.voxel(p) for r,p in by.items()}
    artifacts=[]
    for robot,p in list(by.items())+([('merged',d.s10.voxel(allpoints))] if do_metric else []):
        path=OUT/f'snapshots/{name}_{robot}.ply'; d.s10.write_ply(path,p)
        artifacts.append(dict(path=str(path),sha256=d.sha256(path),size_bytes=path.stat().st_size,points=len(p),robot_source=robot,coordinate_frame='Robot1 KF150 gauge' if do_metric else robot+' independent local frame',server_only=True))
    publication_ms=(time.perf_counter()-begin)*1000
    t=time.perf_counter()
    metric=a.nn(by['robot1'],by['robot3']) if do_metric else dict(median_m=None,p90_m=None,p95_m=None,points_robot1=len(by['robot1']),points_robot3=len(by['robot3']))
    evaluation_ms=(time.perf_counter()-t)*1000
    manifest=dict(checkpoint=name,scope=scope,GT_used=False,robot1_last_keyframe=int(frames['robot1'].keyframe_id.max()),source_keyframes={r:f.iloc[::10].keyframe_id.astype(int).to_list() for r,f in frames.items()},protocol=dict(frame_stride=10,point_stride=16,voxel_m=.5,range_m=[1,60],stride_origin='retained start'),artifacts=artifacts,map_publication_ms=publication_ms,map_consistency_evaluation_ms=evaluation_ms,common_frame_metric_available=do_metric)
    dump(f'snapshots/{name}_map_manifest.json',manifest)
    return by,metric,publication_ms,evaluation_ms

def main():
    if OUT.exists() and any(p.name!='_failed_attempts' for p in OUT.iterdir()): raise RuntimeError('Refuse to overwrite Stage13B outputs or rerun after GT')
    OUT.mkdir(parents=True,exist_ok=True); (OUT/'snapshots').mkdir()
    d.OUT=OUT  # Historical helper writes redirected; no historical stage touched.
    subprocess.run(['git','merge-base','--is-ancestor',CHECKPOINT,'HEAD'],cwd=ROOT,check=True)
    protected=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
    protected=[ROOT/p for p in protected if p.startswith(('src/cumulti/','outputs/cumulti_v1/','demo/final_replay/')) and not p.startswith('outputs/cumulti_v1/13b_incremental_isam2/')]
    hashes={str(p):d.sha256(p) for p in protected if p.is_file()}
    # Verify frozen inputs, including all Stage13A reference manifests, before any run.
    for entry in json.loads((A13/'stage13a_input_manifest.json').read_text())['inputs']:
        assert d.sha256(entry['path'])==entry['sha256'],entry['path']
    for entry in json.loads((A13/'trajectory_artifact_manifest.json').read_text()): assert d.sha256(entry['path'])==entry['sha256']
    local_tr=pd.read_csv(D12/'clean_local_trajectory.csv'); local=d.s10.lookup(local_tr)
    common_tr=pd.read_csv(D12/'clean_first_loop_trajectory.csv'); common=d.s10.lookup(common_tr)
    frames={r:pd.read_csv(d.s10.PROC/r/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns','lidar_file']).iloc[start:].copy() for r,start in d.STARTS.items()}
    loops=pd.read_csv(D12/'ordered_rank1_clean_loops.csv')
    schedule=json.loads((D12/'stage13_loop_schedule.json').read_text())
    assert schedule['ordered_loop_file_sha256']==d.sha256(D12/'ordered_rank1_clean_loops.csv')
    assert len(loops)==75 and loops.arrival_index.to_list()==list(range(1,76))
    assert loops.query_robot.eq('robot1').all() and loops.candidate_robot.eq('robot3').all()
    assert loops.query_keyframe_id.is_monotonic_increasing
    assert set(loops.candidate_keyframe_id).issubset(set(frames['robot3'].keyframe_id))
    first=loops.iloc[0]; assert (int(first.query_keyframe_id),int(first.candidate_keyframe_id))==(283,252)
    S=local[('robot1',283)]@d.tm(first)@d.s10.inv(local[('robot3',252)])
    init_E=d.s10.inv(d.tm(first))@d.s10.inv(local[('robot1',283)])@(S@local[('robot3',252)])
    sanity=dict(translation_m=float(np.linalg.norm(init_E[:3,3])),rotation_deg=float(np.rad2deg(Rotation.from_matrix(init_E[:3,:3]).magnitude())))
    assert sanity['translation_m']<1e-8 and sanity['rotation_deg']<1e-8
    for n in local: assert np.max(abs(common[n]-(local[n] if n[0]=='robot1' else S@local[n])))<1e-9
    params=gtsam.ISAM2Params()
    attrs=['relinearizeSkip','enableRelinearization','evaluateNonlinearError','cacheLinearizedFactors','enableDetailedResults','enablePartialRelinearizationCheck','findUnusedFactorSlots']
    actual={x:getattr(params,x) for x in attrs if hasattr(params,x)}
    actual['factorization']=params.getFactorization()
    actual['relinearizationThreshold']='0.1 (verified installed parameter print; scalar getter unavailable)'
    actual['wildfireThreshold']=gtsam.ISAM2GaussNewtonParams().getWildfireThreshold()
    parameter_policy=dict(UTC=a.now(),defaults_unchanged=True,parameters=actual,parameter_text=str(params),available_api=[x for x in dir(params) if not x.startswith('_')],threshold_getter='UNAVAILABLE',optimization='installed default GaussNewton',one_update_per_event=True,extra_empty_updates=0,estimate_method='calculateEstimate (default wildfire behavior)',GT_used=False)
    dump('isam2_parameter_policy.json',parameter_policy)
    environment=dict(gtsam=importlib.metadata.version('gtsam'),python=sys.version,executable=sys.executable,platform=platform.platform(),CPU=Path('/proc/cpuinfo').read_text().split('model name')[1].split('\n')[0].strip(': '),logical_cpus=os.cpu_count(),ISAM2_available=hasattr(gtsam,'ISAM2'),required_API={x:hasattr(gtsam.ISAM2,x) for x in ['update','calculateEstimate','calculateEstimatePose3','getFactorsUnsafe']},parameter_API=parameter_policy['available_api'],ISAM2_API=[x for x in dir(gtsam.ISAM2) if not x.startswith('_')])
    assert environment['gtsam']=='4.2' and all(environment['required_API'].values())
    dump('isam2_environment_audit.json',environment)
    agree=dict(UTC=a.now(),translation_median_m=.10,translation_p95_m=.50,rotation_median_deg=.5,rotation_p95_deg=2.,same_gauge_no_alignment=True,not_GT_accuracy=True,no_threshold_changes=True,GT_used=False)
    dump('incremental_batch_agreement_policy.json',agree)
    policy=dict(UTC=a.now(),accepted_checkpoint=CHECKPOINT,reference_robot='robot3',streaming_robot='robot1',robot3_reference_start_kf=234,robot3_reference_end_kf=4179,robot1_stream_start_kf=150,robot1_stream_end_kf=1999,first_loop_query_kf=283,first_loop_candidate_kf=252,event_order='original Robot1 keyframes and actual timestamps, fast offline replay (no wall-clock sleeps)',loop_schedule=str(D12/'ordered_rank1_clean_loops.csv'),loop_sha256=d.sha256(D12/'ordered_rank1_clean_loops.csv'),loop_label='PRECOMPUTED FROZEN LOOP FACTORS; not causal online discovery',noise_policy=json.loads((D12/'clean_graph_policy.json').read_text()),prior='exactly one Robot1 KF150 prior after connected fusion; no joint ISAM2 before bridge',parameters=parameter_policy,initialization='first fusion S=Xq Zqc inverse(Lc); later guess=estimated preceding pose times new frozen relative odometry; no future Robot1 initialization',snapshots=['PRE_FIRST_LOOP','FIRST_LOOP','K2','K5','K10','K20','K40','K75','FINAL_STREAM_END'],benchmark='one primary replay; actual per-event measurements, single first-fusion sample; no replays/retuning after GT',timing='event processing excludes checkpoint batch-reference solve, NN evaluation, integrity/full-pose diagnostics and logging; checkpoint map publication included and separately reported; first-fusion construction separate',batch_initialization='same Stage12D first-loop x0 restricted to arrived Robot1 nodes and stored Robot3, not incremental warm start',GT_policy='all incremental/batch runs, snapshots, diagnostics and comparisons finished before persisted decision, then final-only GT evaluation',agreement=agree,GT_used=False)
    dump('incremental_policy.json',policy)
    dump('frozen_input_manifest.json',dict(accepted_checkpoint=CHECKPOINT,GT_used=False,inputs=[dict(path=p,sha256=h,size_bytes=Path(p).stat().st_size) for p,h in hashes.items()]))
    event('POLICY_FROZEN')
    t=time.perf_counter(); template,lmparams=d.build_graph(frames,local,loops); template_ms=(time.perf_counter()-t)*1000
    assert json.loads((OUT/'clean_gtsam_solver_policy.json').read_text())['parameters']==json.loads((D12/'clean_gtsam_solver_policy.json').read_text())['parameters']
    # Immutable factor identities indexed in the exact Stage12D construction order.
    odom_idx={('robot1',j):j-150 for j in range(151,2000)}
    odom_idx.update({('robot3',j):1850+(j-235) for j in range(235,4180)})
    loop_idx={int(r.arrival_index):5795+int(r.arrival_index)-1 for r in loops.itertuples()}
    loop_events={int(q):sub for q,sub in loops.groupby('query_keyframe_id',sort=False)}
    traces=[]; checkpoints=[]; comparisons=[]; mapmetrics=[]; batchmapmetrics=[]; integrity_rows=[]; checkpoint_clouds={}; checkpoint_trs={}; batch_trs={}
    registry_nodes=set(); registry_factors=set(); registry_loops=[]
    cumulative=gtsam.NonlinearFactorGraph(); isam=None; cumulative_loop_count=0
    selection=json.loads((d.C12/'stable_segment_selection.json').read_text())
    firstcost={}; final_values=None

    def snapshot(name,kf,estimate,indices,K,pre=False):
        available={'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']}
        nodes=[(r,int(i)) for r,f in available.items() for i in f.keyframe_id]
        matrices={n:local[n] for n in nodes} if pre else matrices_from(estimate,nodes)
        tr=d.trajectory(available,matrices); schema(tr,available)
        path=OUT/f'snapshots/{name}_trajectory.csv'; tr.to_csv(path,index=False)
        cloud,metric,pub_ms,nn_ms=make_map(tr,available,name,'INCREMENTAL',not pre)
        checkpoint_trs[name]=tr
        if name in ['FIRST_LOOP','K5','K20','K75']: checkpoint_clouds[name]=cloud
        checkpoints.append(dict(checkpoint=name,robot1_keyframe=kf,robot1_timestamp_ns=int(available['robot1'].lidar_timestamp_ns.iloc[-1]),robot1_nodes=len(available['robot1']),robot3_nodes=3946,graph_nodes=0 if pre else len(nodes),loop_factors=K,trajectory_path=str(path),trajectory_sha256=d.sha256(path),map_manifest=str(OUT/f'snapshots/{name}_map_manifest.json'),frame='SEPARATE_ROBOT_LOCAL_FRAMES' if pre else 'ROBOT1_KF150_GAUGE',map_publication_ms=pub_ms,map_evaluation_ms=nn_ms))
        mapmetrics.append(dict(checkpoint=name,robot1_keyframe=kf,robot1_nodes=len(available['robot1']),cumulative_loops=K,metric_status='NOT_APPLICABLE_SEPARATE_FRAMES' if pre else 'OK',**metric))
        if not pre:
            integrity_rows.extend(integrity(tr,local_tr,common_tr,selection,'ISAM2',name))
            graph=graph_subset(template,sorted(indices)); x0=to_values({n:common[n] for n in nodes})
            assert set(estimate.keys())==set(x0.keys())==set(graph.keyVector())
            start=time.perf_counter(); opt=gtsam.LevenbergMarquardtOptimizer(graph,x0,lmparams); result=opt.optimize(); batch_ms=(time.perf_counter()-start)*1000
            batch=matrices_from(result,nodes); btr=d.trajectory(available,batch); schema(btr,available)
            btr.to_csv(OUT/f'snapshots/{name}_batch_trajectory.csv',index=False); batch_trs[name]=btr
            diff=pose_difference(matrices,batch)
            passes=all(diff[x]<=agree[x] for x in diff)
            comparisons.append(dict(checkpoint=name,K=K,robot1_keyframe=kf,nodes=len(nodes),odometry_factors=len(nodes)-2,loop_factors=K,prior_factors=1,total_factors=graph.size(),initial_batch_error=float(graph.error(x0)),batch_final_error=float(graph.error(result)),incremental_final_error=float(graph.error(estimate)),graph_error_difference=float(graph.error(estimate)-graph.error(result)),batch_solve_ms=batch_ms,batch_iterations=opt.iterations(),batch_normal_return=True,identical_graph=True,agreement_status='INCREMENTAL_BATCH_AGREEMENT_PASS' if passes else 'ISAM2_BATCH_DIVERGENCE',**diff))
            _,bm,bpub,bnn=make_map(btr,available,name+'_BATCH','BATCH_IDENTICAL_GRAPH')
            batchmapmetrics.append(dict(checkpoint=name,robot1_keyframe=kf,robot1_nodes=len(available['robot1']),K=K,incremental_median_m=metric['median_m'],incremental_p95_m=metric['p95_m'],batch_median_m=bm['median_m'],batch_p95_m=bm['p95_m'],batch_map_publication_ms=bpub,batch_map_evaluation_ms=bnn,scans_identical=True))
            event('CHECKPOINT',name=name,kf=kf,K=K,agreement=comparisons[-1]['agreement_status'],translation_p95_m=diff['translation_p95_m'])
        csv('checkpoint_manifest.csv',checkpoints); csv('incremental_vs_batch_checkpoints.csv',comparisons); csv('incremental_map_consistency.csv',mapmetrics); csv('incremental_vs_batch_map_consistency.csv',batchmapmetrics); csv('incremental_trajectory_integrity.csv',integrity_rows)
        return pub_ms,nn_ms

    for event_index,r in enumerate(frames['robot1'].itertuples()):
        kf=int(r.keyframe_id); begin=time.perf_counter(); update_ms=extract_ms=pub_ms=construction_ms=0.; newodom=newloops=0
        if kf<283:
            event_type='LOCAL_ODOMETRY_ONLY'; newodom=int(kf>150)
            # Causal local odometry composition, no global solver or second prior.
            if kf==150: local_prefix={('robot1',150):local[('robot1',150)]}
            else:
                Z=template.at(odom_idx[('robot1',kf)]).measured().matrix()
                local_prefix[('robot1',kf)]=local_prefix[('robot1',kf-1)]@Z
                assert np.max(abs(local_prefix[('robot1',kf)]-local[('robot1',kf)]))<1e-8
            if kf==282:
                t=time.perf_counter(); pub_ms,_=snapshot('PRE_FIRST_LOOP',kf,None,[],0,True)
            total_ms=(time.perf_counter()-begin)*1000 if kf!=282 else pub_ms
            node_count=0; components=2
        else:
            new=gtsam.NonlinearFactorGraph(); newvalues=gtsam.Values()
            arrived=loop_events.get(kf)
            newloops=0 if arrived is None else len(arrived)
            assert newloops<=1,'Checkpoint prefix protocol requires at most one archived loop per query; inspect schedule before adapting'
            if kf==283:
                t=time.perf_counter()
                current_frames={'robot1':frames['robot1'][frames['robot1'].keyframe_id.le(kf)],'robot3':frames['robot3']}
                nodes=[(robot,int(i)) for robot,f in current_frames.items() for i in f.keyframe_id]
                indices=[0]+[odom_idx[('robot1',i)] for i in range(151,kf+1)]+[odom_idx[('robot3',i)] for i in range(235,4180)]+[loop_idx[1]]
                new=graph_subset(template,indices)
                newvalues=to_values({n:local[n] if n[0]=='robot1' else S@local[n] for n in nodes})
                registry_nodes.update(nodes); registry_factors.update(indices); registry_loops.append(1)
                construction_ms=(time.perf_counter()-t)*1000
                assert len(nodes)==4080 and len(indices)==4080 and newvalues.size()==4080
                dump('first_fusion_manifest.json',dict(robot1_nodes=134,robot3_nodes=3946,total_nodes=4080,robot1_odometry=133,robot3_odometry=3945,odometry_factors=4078,loop_factors=1,prior_factors=1,query_keyframe=283,candidate_keyframe=252,first_graph_construction_ms=construction_ms,factor_template_preparation_ms=template_ms,only_arrived_robot1_nodes=True,GT_used=False))
                # Actual graph traversal of odometry and bridge endpoints.
                parent={n:n for n in nodes}
                def find(n):
                    while parent[n]!=n: parent[n]=parent[parent[n]];n=parent[n]
                    return n
                def union(x,y): parent[find(x)]=find(y)
                for robot,f in current_frames.items():
                    ids=f.keyframe_id.to_list()
                    for x,y in zip(ids,ids[1:]): union((robot,x),(robot,y))
                before=len({find(n) for n in nodes}); union(('robot1',283),('robot3',252));after=len({find(n) for n in nodes})
                assert (before,after)==(2,1)
                dump('first_fusion_connectivity.json',dict(before=before,after=after,actual_union_find=True,no_joint_ISAM2_before_bridge=True,permanent_priors=1))
                isam=gtsam.ISAM2(params); isam_identity=id(isam)
                newodom=4078;event_type='FIRST_INTER_ROBOT_FUSION'
            else:
                assert id(isam)==isam_identity
                event_type='INTER_ROBOT_LOOP_UPDATE' if newloops else 'ODOMETRY_UPDATE'
                node=('robot1',kf); idx=odom_idx[node]; assert node not in registry_nodes and idx not in registry_factors
                t=time.perf_counter(); previous=isam.calculateEstimatePose3(numeric_key(('robot1',kf-1))); extract_ms+=(time.perf_counter()-t)*1000
                Z=template.at(idx).measured()
                newvalues.insert(numeric_key(node),previous.compose(Z)); new.add(template.at(idx)); registry_nodes.add(node); registry_factors.add(idx); indices=[idx]; newodom=1
                if arrived is not None:
                    for loop in arrived.itertuples():
                        li=int(loop.arrival_index); assert li not in registry_loops and li==len(registry_loops)+1 and int(loop.query_keyframe_id)==kf
                        ix=loop_idx[li]; assert ix not in registry_factors and ('robot3',int(loop.candidate_keyframe_id)) in registry_nodes
                        new.add(template.at(ix)); indices.append(ix); registry_loops.append(li); registry_factors.add(ix)
            assert not (OUT/'pre_gt_incremental_decision.json').exists()
            t=time.perf_counter(); result=isam.update(new,newvalues); update_ms=(time.perf_counter()-t)*1000
            for i in indices: cumulative.add(template.at(i))
            cumulative_loop_count=len(registry_loops)
            assert max(n[1] for n in registry_nodes if n[0]=='robot1')==kf
            assert isam.getFactorsUnsafe().size()==len(registry_factors)==cumulative.size()
            t=time.perf_counter(); estimate=isam.calculateEstimate(); extract_ms+=(time.perf_counter()-t)*1000
            assert estimate.size()==len(registry_nodes)
            # Finite check of every current estimate, every event (diagnostic cost separate).
            core_ms=(time.perf_counter()-begin)*1000
            current=matrices_from(estimate,registry_nodes)
            assert all(np.isfinite(x).all() for x in current.values()),'STOP: nonfinite estimate'
            # Monitor physical translation steps every update, before any future insertion.
            catastrophe=False
            for robot in d.STARTS:
                end=kf if robot=='robot1' else 4179; ids=list(range(d.STARTS[robot],end+1))
                xyz=np.array([current[(robot,i)][:3,3] for i in ids]); prexyz=np.array([common[(robot,i)][:3,3] for i in ids])
                poststep=np.linalg.norm(np.diff(xyz,axis=0),axis=1); prestep=np.linalg.norm(np.diff(prexyz,axis=0),axis=1)
                limit=selection['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold']
                catastrophe|=bool(((poststep>5*limit)&(prestep<=5*limit)|(poststep>=100)&(prestep<100)).any())
            assert not catastrophe,'STOP: new catastrophic position discontinuity'
            if kf==283 or cumulative_loop_count in a.KS and newloops:
                name='FIRST_LOOP' if cumulative_loop_count==1 else f'K{cumulative_loop_count}'
                pub_ms,_=snapshot(name,kf,estimate,registry_factors,cumulative_loop_count)
            total_ms=core_ms+pub_ms
            if kf==283:
                firstcost=dict(first_graph_construction_ms=construction_ms,factor_template_preparation_ms=template_ms,first_isam2_update_ms=update_ms,first_estimate_extraction_ms=extract_ms,first_global_map_publication_ms=pub_ms,first_fusion_total_ms=total_ms,one_sample_only=True,excludes_batch_reference_and_diagnostics=True)
                dump('first_fusion_latency.json',firstcost)
                dump('first_loop_incremental_fusion_result.json',dict(query=283,candidate=252,components_before=2,components_after=1,normal_ISAM2_init=True,initial_alignment=sanity,first_global_nodes=4080,common_frame=True,claim='One correct inter-robot loop establishes initial common-frame connectivity; it does not guarantee globally correct reconstruction.',**firstcost))
            final_values=estimate;node_count=len(registry_nodes);components=1
        traces.append(dict(event_index=event_index,event_type=event_type,robot1_keyframe=kf,robot1_timestamp_ns=int(r.lidar_timestamp_ns),current_robot1_node_count=kf-150+1,reference_robot3_node_count=3946,total_graph_nodes=node_count,new_odometry_factors=newodom,new_loop_factors=newloops,cumulative_loop_factors=cumulative_loop_count,connected_components=components,isam2_update_ms=update_ms,estimate_extraction_ms=extract_ms,map_publication_ms=pub_ms,total_event_ms=total_ms,processing_status='NORMAL_RETURN' if kf>=283 else 'LOCAL_ONLY_NO_GLOBAL_SOLVE',cumulative_global_factors=len(registry_factors),nodes_relinearized=None if kf<283 else int(result.getVariablesRelinearized()),variables_reeliminated=None if kf<283 else int(result.getVariablesReeliminated()),instance_id=None if kf<283 else isam_identity))
        if newloops or kf%100==0:
            csv('incremental_event_trace.csv',traces); event('STREAM_PROGRESS',kf=kf,nodes=node_count,loops=cumulative_loop_count)
    assert len(registry_nodes)==5796 and len(registry_factors)==5870 and registry_loops==list(range(1,76))
    # Structural equality is independent of insertion order. Exact original objects
    # and GTSAM equals include measured Pose3 and noise models, not just counts.
    actual=isam.getFactorsUnsafe()
    equality=all(actual.at(i).equals(cumulative.at(i),1e-12) for i in range(actual.size()))
    same_measurements=all(cumulative.at(i).equals(template.at(ix),1e-12) for i,ix in enumerate(list(range(0,134))+list(range(1850,5795))+[5795]+[ix for row in traces[134:] for ix in ([odom_idx[('robot1',int(row['robot1_keyframe']))]]+([loop_idx[int(row['cumulative_loop_factors'])]] if row['new_loop_factors'] else []))]))
    equivalent=dict(factor_count_equality=actual.size()==template.size()==5870,node_key_equality=set(final_values.keys())==set(template.keyVector()),loop_ID_equality=registry_loops==list(range(1,76)),odometry_factor_equality=len(registry_factors)-75-1==5794,measurement_noise_equality=bool(equality and same_measurements),all_template_factor_indices_once=registry_factors==set(range(template.size())),tolerance=1e-12,prior_count=1,GT_used=False)
    equivalent['PASS']=all(v for k,v in equivalent.items() if k not in ['tolerance','prior_count','GT_used'])
    dump('final_graph_equivalence.json',equivalent); assert equivalent['PASS'],'STOP: final graph not structurally identical'
    pub_ms,_=snapshot('FINAL_STREAM_END',1999,final_values,registry_factors,75)
    final_tr=checkpoint_trs['FINAL_STREAM_END']; final_tr.to_csv(OUT/'incremental_final_trajectory.csv',index=False)
    finalmat=d.s10.lookup(final_tr); oldbatch=d.s10.lookup(pd.read_csv(A13/'trajectories/K75_optimized.csv'))
    dump('final_vs_stage13a_K75.json',dict(identical_graph=True,**pose_difference(finalmat,oldbatch)))
    traces.append(dict(traces[-1],event_index=len(traces),event_type='FINAL_STATE',new_odometry_factors=0,new_loop_factors=0,isam2_update_ms=0.,estimate_extraction_ms=0.,map_publication_ms=pub_ms,total_event_ms=pub_ms))
    trace=pd.DataFrame(traces); csv('incremental_event_trace.csv',trace)
    latency=trace[trace.event_type.ne('FINAL_STATE')].copy()
    latency['next_interval_ms']=(latency.robot1_timestamp_ns.shift(-1)-latency.robot1_timestamp_ns)/1e6
    latency['update_deadline_met']=np.where(latency.next_interval_ms.notna(),latency.isam2_update_ms.lt(latency.next_interval_ms),None)
    latency['processing_deadline_met']=np.where(latency.next_interval_ms.notna(),latency.total_event_ms.lt(latency.next_interval_ms),None)
    csv('incremental_latency_by_event.csv',latency)
    timing=[]
    for kind,sub in latency.groupby('event_type',sort=False):
        for metric in ['isam2_update_ms','total_event_ms','map_publication_ms']:
            x=sub[metric].to_numpy(); valid=sub.next_interval_ms.notna()
            timing.append(dict(event_type=kind,metric=metric,count=len(x),mean=float(np.mean(x)),median=float(np.median(x)),p90=float(np.percentile(x,90)),p95=float(np.percentile(x,95)),p99=float(np.percentile(x,99)),max=float(np.max(x)),small_sample_warning='single replay; tail limited especially loop/first fusion',deadline_samples=int(valid.sum()),update_missed_deadlines=int((sub.loc[valid,'isam2_update_ms']>=sub.loc[valid,'next_interval_ms']).sum()),total_processing_missed_deadlines=int((sub.loc[valid,'total_event_ms']>=sub.loc[valid,'next_interval_ms']).sum()),update_deadline_met_fraction=float(sub.loc[valid,'isam2_update_ms'].lt(sub.loc[valid,'next_interval_ms']).mean()) if valid.any() else None))
    csv('incremental_latency_summary.csv',timing)
    residual,summary=d.loop_residuals(loops,finalmat); csv('incremental_final_loop_residuals.csv',residual); dump('incremental_final_loop_residual_summary.json',summary)
    valid_agreement=all(x['agreement_status']=='INCREMENTAL_BATCH_AGREEMENT_PASS' for x in comparisons)
    diagnosis=dict(status='INCREMENTAL_BATCH_AGREEMENT_PASS' if valid_agreement else 'ISAM2_BATCH_DIVERGENCE',fixed_defaults=actual is not None,parameters_changed=False,factor_equivalence=equivalent['PASS'],initialization='batch fixed first-loop x0 vs sequential causal ISAM2 guesses',relinearization_default=actual is not None,mechanisms_to_investigate=['single GaussNewton update vs converged LM; default skip10 relinearization / threshold0.1 / wildfire0.001','robust factor linearization caching and nonlinear loop arrivals','different nonlinear optimizer paths and possible local minima'],causality_established=False,no_parameter_changes_or_rerun=True)
    dump('incremental_batch_divergence_diagnosis.json',diagnosis)
    unchanged=all(d.sha256(p)==h for p,h in hashes.items())
    dump('frozen_input_immutability_audit.json',dict(PASS=unchanged,files_checked=len(hashes),before=hashes))
    ready=valid_agreement and unchanged and equivalent['PASS']
    decision=dict(decision='INCREMENTAL_BACKEND_READY' if ready else 'INCREMENTAL_BACKEND_NOT_READY',UTC=a.now(),GT_used=False,all_checkpoints_agree=valid_agreement,final_graph_equivalent=equivalent['PASS'],finite_and_no_catastrophic_jumps=True,parameters_frozen=True,reason='all frozen structural/numerical agreement gates pass' if ready else 'one or more default ISAM2 checkpoints diverge from identical-graph LM; no retuning',checkpoints=comparisons)
    dump('pre_gt_incremental_decision.json',decision); event('PRE_GT_DECISION',decision=decision['decision'])
    # Redirect existing Stage13A evaluation outputs to NEW stage only. The decision
    # existence alias expected by that helper is not used: use exact evaluation
    # kernel below, preserving the real Stage13B chronology.
    gt=offline_gt(final_tr,frames)
    assets=[dict(row) for row in trace.replace({np.nan:None}).to_dict('records')]
    dump('demo_event_manifest.json',dict(replay_model='PREBUILT ROBOT3 REFERENCE MAP + STREAMING ROBOT1',loop_label='PRECOMPUTED FROZEN LOOP FACTORS',online_SC_GICP=False,simultaneous_robots=False,events=assets,snapshots=checkpoints,historical_demo_modified=False))
    report=dict(stage13a_immutable=unchanged,api=True,reference_preloaded=True,stream_order_valid=trace.iloc[:-1].robot1_keyframe.to_list()==list(range(150,2000)),no_future_nodes=True,frozen_loops_unchanged=True,independent_pre_bridge=True,first_connected=True,first_initialization=True,ISAM2_init=True,persistent_later_updates=True,no_duplicates=True,final_nodes=True,final_odometry=True,final_loops=True,final_graph_equivalence=equivalent['PASS'],ISAM2_batch_agreement=valid_agreement,no_catastrophic=True,first_fusion_timing=True,steady_timing=True,decision_before_GT=True,offline_GT=True)
    dump('summary.json',dict(status='PASS' if ready else 'FAIL_NUMERICAL_AGREEMENT',decision=decision['decision'],first_fusion=firstcost,final_graph=equivalent,final_difference=comparisons[-1],latency_summary=timing,validation=report))
    validation_report(report)
    figures(trace,firstcost,pd.DataFrame(comparisons),pd.DataFrame(mapmetrics),checkpoint_trs,batch_trs,checkpoint_clouds,local_tr)
    document(policy,decision,firstcost,pd.DataFrame(comparisons),pd.DataFrame(mapmetrics),pd.DataFrame(timing),gt,diagnosis)
    event('COMPLETE',status='PASS' if ready else 'FAIL_NUMERICAL_AGREEMENT')

def offline_gt(tr,frames):
    assert (OUT/'pre_gt_incremental_decision.json').is_file(); event('FIRST_GT_ACCESS')
    gt={r:pd.read_csv(d.s10.PROC/r/'keyframes.csv') for r in d.STARTS}
    ate,_,_=d.s10.joint_eval(tr,gt['robot1'],gt['robot3']); rows=[]
    for x in ate: rows.append(dict(metric_type='ATE_joint_rigid_alignment',use='OFFLINE GT DIAGNOSTIC ONLY',**x))
    poses=d.s10.lookup(tr); all_t=[];all_r=[]
    for robot,f in frames.items():
        ts=[];rs=[];ids=f.keyframe_id.to_list()
        for u,v in zip(ids[:-10],ids[10:]):
            gu,gv=gt[robot].iloc[u],gt[robot].iloc[v]
            G0=d.s10.T([gu.qx,gu.qy,gu.qz,gu.qw],[gu.x,gu.y,gu.z]);G1=d.s10.T([gv.qx,gv.qy,gv.qz,gv.qw],[gv.x,gv.y,gv.z])
            E=d.s10.inv(d.s10.inv(G0)@G1)@d.s10.inv(poses[(robot,u)])@poses[(robot,v)]
            ts.append(np.linalg.norm(E[:3,3]));rs.append(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude()))
        all_t.extend(ts);all_r.extend(rs)
        rows.append(dict(metric_type='RPE',scope=robot,interval_keyframes=10,translation_RMSE_m=float(np.sqrt(np.mean(np.square(ts)))),rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(rs)))),sample_count=len(ts),use='OFFLINE GT DIAGNOSTIC ONLY'))
    rows.append(dict(metric_type='RPE',scope='combined',interval_keyframes=10,translation_RMSE_m=float(np.sqrt(np.mean(np.square(all_t)))),rotation_RMSE_deg=float(np.sqrt(np.mean(np.square(all_r)))),sample_count=len(all_t),use='OFFLINE GT DIAGNOSTIC ONLY'))
    csv('incremental_offline_gt_evaluation.csv',rows);return pd.DataFrame(rows)

def figures(trace,cost,comparison,maps,trs,btrs,clouds,local_tr):
    def save(fig,name):fig.savefig(OUT/name,dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,3),constrained_layout=True);q=trace[trace.new_loop_factors.gt(0)]
    ax.plot(trace.robot1_keyframe,trace.cumulative_loop_factors);ax.scatter(q.robot1_keyframe,q.cumulative_loop_factors,s=8);ax.set(xlabel='Robot1 keyframe',ylabel='Archived loops inserted',title='Reference Robot3 + Robot1 offline insertion timeline');save(fig,'01_incremental_timeline.png')
    colors={'robot1':'tab:blue','robot3':'tab:orange'}
    def lines(ax,tr,title):
        for r,c in colors.items():
            z=tr[tr.robot_id.eq(r)];ax.plot(z.tx,z.ty,lw=.7,c=c,label=r)
        ax.set(title=title,xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal',adjustable='box');ax.legend()
    fig,axs=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
    for ax,r in zip(axs[:2],colors):lines(ax,trs['PRE_FIRST_LOOP'][trs['PRE_FIRST_LOOP'].robot_id.eq(r)],r+' independent local frame')
    lines(axs[2],trs['FIRST_LOOP'],'First loop: connected common frame');save(fig,'02_first_loop_fusion.png')
    fig,ax=plt.subplots(figsize=(10,4),constrained_layout=True);ax.plot(trace.robot1_keyframe,trace.total_graph_nodes,label='global nodes');ax.plot(trace.robot1_keyframe,trace.cumulative_global_factors,label='global factors');ax.set(xlabel='Robot1 KF',ylabel='Count',title='Joint graph starts only at KF283');ax.legend();save(fig,'03_graph_growth.png')
    fig,ax=plt.subplots(figsize=(10,4),constrained_layout=True)
    for kind,c in [('ODOMETRY_UPDATE','tab:blue'),('INTER_ROBOT_LOOP_UPDATE','tab:orange'),('FIRST_INTER_ROBOT_FUSION','red')]:
        x=trace[trace.event_type.eq(kind)];ax.scatter(x.robot1_keyframe,x.isam2_update_ms,s=8,color=c,label=kind)
    ax.set(xlabel='Robot1 KF',ylabel='ISAM2.update ms',yscale='log',title='Actual update times; first fusion separate');ax.legend(fontsize=8);save(fig,'04_isam2_update_latency.png')
    fig,ax=plt.subplots(figsize=(8,4),constrained_layout=True);fields=['first_graph_construction_ms','first_isam2_update_ms','first_estimate_extraction_ms','first_global_map_publication_ms'];ax.bar(['Graph','ISAM2','Extract','Map'],[cost[f] for f in fields]);ax.set(ylabel='ms',title='First fusion: one sample; batch references excluded');save(fig,'05_first_fusion_cost_breakdown.png')
    fig,axs=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
    for ax,prefix,unit in zip(axs,['translation','rotation'],['m','deg']):
        ax.plot(comparison.checkpoint,comparison[f'{prefix}_median_{unit}'],'-o',label='median');ax.plot(comparison.checkpoint,comparison[f'{prefix}_p95_{unit}'],'-o',label='p95');ax.tick_params(axis='x',rotation=35);ax.set(ylabel=unit,title='Identical graph '+prefix+' differences');ax.legend()
    save(fig,'06_incremental_vs_batch_pose_error.png')
    allxy=np.vstack([p[:,:2] for by in clouds.values() for p in by.values()]);lo,hi=allxy.min(0)-5,allxy.max(0)+5
    fig,axs=plt.subplots(1,4,figsize=(18,5),constrained_layout=True)
    for ax,name in zip(axs,['FIRST_LOOP','K5','K20','K75']):
        for r,c in colors.items():
            p=clouds[name][r][::max(1,len(clouds[name][r])//60000)];ax.scatter(p[:,0],p[:,1],s=.15,c=c,alpha=.4,label=r)
        ax.set(title=name,xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]));ax.set_aspect('equal');ax.legend()
    save(fig,'07_incremental_map_snapshots.png')
    x=maps[maps.metric_status.eq('OK')]
    fig,axs=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
    axs[0].plot(x.robot1_keyframe,x.median_m,'-o',label='median');axs[0].plot(x.robot1_keyframe,x.p95_m,'-o',label='p95');axs[0].set(xlabel='Robot1 KF',ylabel='Symmetric NN (m)',title='Changing-coverage checkpoint maps');axs[0].legend()
    axs[1].plot(x.robot1_keyframe,x.points_robot1,'-o',label='Robot1 map points');axs[1].plot(x.robot1_keyframe,x.points_robot3,'-o',label='Robot3 map points');axs[1].set(xlabel='Robot1 KF',ylabel='Voxel points',title='Coverage explicitly changes');axs[1].legend();save(fig,'08_map_consistency_over_time.png')
    fig,axs=plt.subplots(1,2,figsize=(13,5),constrained_layout=True);both=pd.concat([trs['FINAL_STREAM_END'],btrs['FINAL_STREAM_END']])[['tx','ty']].to_numpy();lo,hi=both.min(0)-5,both.max(0)+5
    for ax,tr,title in zip(axs,[trs['FINAL_STREAM_END'],btrs['FINAL_STREAM_END']],['Final ISAM2 (default frozen policy)','Identical-final-graph batch LM']):lines(ax,tr,title);ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1])
    save(fig,'09_final_incremental_vs_batch.png')

def document(policy,decision,cost,comparison,maps,latency,gt,diagnosis):
    def table(df):
        return '| '+' | '.join(df.columns)+' |\n| '+' | '.join(['---']*len(df.columns))+' |\n'+'\n'.join('| '+' | '.join('N/A' if pd.isna(v) else f'{v:.6f}' if isinstance(v,float) else str(v) for v in row)+' |' for row in df.itertuples(index=False,name=None))
    text=f'''# Stage13B — Incremental multi-robot map fusion with ISAM2

## 1. Motivation

Test first-loop connectivity and repeated genuine incremental optimization against identical-graph batch LM. Accepted checkpoint `{CHECKPOINT}`; independent new outputs only.

Current method status: **{'PASS' if decision['decision']=='INCREMENTAL_BACKEND_READY' else 'FAIL_NUMERICAL_AGREEMENT'}**. Execution/provenance/integrity succeed, but readiness is **{decision['decision']}**; do not conflate successful execution with numerical readiness.

## 2. Stage13A limitations

Stage13A full-graph loop budgets include future Robot1 poses and are not causal stream checkpoints. It found NO_SPARSE_SAVING and non-monotonic map/GT outcomes. Nothing here rewrites those results.

## 3. Robot3 reference-map protocol

Robot3 KF234–4179 is a previously recorded complete 3946-pose local reference map, not a simultaneously running robot.

## 4. Robot1 streaming scheduler

Robot1 KF150–1999 arrives in original timestamp/keyframe order (nominal 2Hz), fast offline replay without sleep. Before bridge, local relative odometry is composed; after bridge initial guess = estimated previous pose × new odometry. No future Robot1 node, factor or scan is inserted.

## 5. Frozen loop arrival semantics

PRECOMPUTED FROZEN LOOP FACTORS: all75 frozen query-order factors inserted exactly once at query arrival. Candidate can come from any stored Robot3 frame. Offline sanitation may use later information; this tests causal insertion, NOT causal online discovery by SC/GICP.

## 6. Independent local frames

No joint unanchored solver before bridge, no fused map/NN before bridge, no pair of permanent hard priors. Pre-loop snapshot preserves two independent coordinate systems.

## 7. First-loop common-frame initialization

R1#283 ↔ R3#252, exact Stage12D LiDAR/os_sensor gauge and URDF extrinsic. `S=Xq Zqc inverse(Lc)`; numerical-zero first-loop residual recorded. One correct inter-robot loop establishes initial common-frame connectivity; it does not guarantee globally correct reconstruction.

## 8. First ISAM2 graph construction

4080 nodes (134 arrived R1 +3946 stored R3),4078 odometry,1 loop,1 prior on R1 KF150. Actual union-find verifies 2→1. ISAM2 initialized only on this connected graph. Factor objects from Stage12D construction reused exactly.

## 9. Sequential odometry updates

Same live ISAM2 instance thereafter. New Values contains exactly one arrived R1 pose; newFactors contains only its new odometry plus contemporaneous archived loops. Registries assert no repeats/future keys. Every estimate checked finite and every physical adjacent translation checked against frozen Stage12D catastrophe rule.

## 10. Inter-robot loop updates

Installed default GaussNewton ISAM2, one update per event, no hidden LM solve used to generate incremental poses, no extra empty updates. Relinearize skip10, threshold0.1, wildfire0.001, CHOLESKY, cache enabled; actual binding print/API in policy/audit. Parameters never changed after comparison. Batch runs are separate diagnostics. An initial development attempt stopped at checkpoint key validation because the installed `KeySet` is not iterable; after inspecting the API, validation switched to `keyVector()`. That attempt is preserved under `_failed_attempts/api_keyset/`, with identical parameters and no GT access; it is not pooled into primary latency measurements.

## 11. First-fusion cost

{table(pd.DataFrame([cost]))}

First-fusion graph construction/update/extraction/map publication measured separately, one sample. Prebuilding immutable factor template separately reported. Reference-map loading/publication and thousands of initial poses are not steady-state update cost.

## 12. Steady-state update latency

{table(latency[latency.metric.eq('isam2_update_ms')][['event_type','count','mean','median','p90','p95','p99','max','update_missed_deadlines','total_processing_missed_deadlines']])}

Actual next-keyframe intervals used for deadlines. Timing excludes frontend, network, batch diagnostics and integrity/full-matrix auditing; snapshot map publication is separately included in event processing and can miss deadlines. Small loop/first-fusion sample counts limit tails. No whole-system real-time claim. `total_graph_nodes` means instantiated joint-global graph nodes (zero before bridge), while both independent local maps have available pose counts separately recorded. Robot3 reference loading and the immutable full factor-template preparation are startup work, not hidden steady-state updates. Publication includes scan loading, transforms, voxelization, PLY writing and hashing; NN evaluation is separate.

## 13. Incremental vs batch comparison

Checkpoints FIRST_LOOP/K2/K5/K10/K20/K40/K75/FINAL_STREAM_END use identical arrived nodes/odom/loops/prior/noise, not full Stage13A K1. Batch x0 is the frozen first-loop frame restricted to available nodes; same Stage12D LM defaults. Same R1 KF150 gauge, NO alignment. Fixed limits 0.10/0.50m translation median/p95;0.5/2° orientation median/p95.

{table(comparison[['checkpoint','nodes','K','translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg','graph_error_difference','agreement_status']])}

Diagnosis: `{diagnosis['status']}`. Factors/anchor/noise/order provenance is independently recorded. One nonlinear ISAM2 update with skip10/wildfire and robust-factor linearization differs from converged LM and its initialization path; these are plausible mechanisms, not experimentally proven causes. No loosened thresholds or retuning/reruns.

Independent follow-up file `incremental_anchor_relinearization_audit.csv` verifies the same fixed anchor and records exposed relinearized/reeliminated counts at checkpoint arrival. `numerical_discrepancy_forensics.json` separates verified structural/gauge facts from unproven numerical mechanisms. Near-equal final objective does not guarantee the frozen pose-difference thresholds pass.

## 14. Final graph equivalence

5796 nodes,5794 odometry,75 loops,1 prior. All original template factors exactly once; GTSAM factor equality (1e-12) verifies measurements/noise despite insertion order. Final graph is structurally identical to Stage13A K75; final pose difference saved separately.

## 15. Map-consistency evolution

{table(maps[['checkpoint','robot1_keyframe','robot1_nodes','median_m','p95_m','points_robot1','points_robot3','metric_status']])}

Deterministic Stage12D frame10/point16/voxel0.5/range1–60, only arrived R1 scans. Source-specific maps and manifests exported. Different checkpoint coverage prevents treating this as the Stage13A full-trajectory sweep. Identical-scan batch metrics isolate solver effects at each checkpoint; between checkpoints acquisition and new factors both change, so their separate causal effects are NOT identifiable from raw NN curves alone. No common-frame metric pre-bridge.

## 16. Pre-GT decision

`{decision['decision']}`, persisted {decision['UTC']} after all incremental/batch runs and GT-free comparisons. Numerical readiness requires all fixed agreement gates; GT unused.

## 17. Offline GT diagnostic

One joint rigid alignment, per-robot/joint ATE and per-robot/combined10-KF RPE, only after saved decision. OFFLINE GT DIAGNOSTIC ONLY.

{table(gt)}

GT never changes readiness or parameters; no solve after GT.

## 18. Limitations

Reference Robot3 is stored; Robot1 alone streams; loops precomputed offline. No causal online SC/GICP, simultaneous two-robot acquisition, live input, decentralized optimization, network/transport/bandwidth/consensus benchmark. First-fusion cost != steady incremental update; batch latency != incremental latency. NN map agreement != GT trajectory accuracy. Single replay and default policy only; significant discrepancy cannot be silently tuned away.

## 19. Prepared demo assets

`demo_event_manifest.json` contains arrival/fusion/loop events, counts/connectivity/timings and snapshot paths; PLY source/frame/hash manifests are server-only. Historical `demo/final_replay/` untouched. No new demo started.

## 20. Next stage

Stop after Stage13B. If numerical disagreement persists, separately declare a GT-free relinearization/optimizer methodology audit before claiming backend readiness; do not change this frozen default run. Any Stage15 visualization is separately authorized. Do not implement distributed SLAM or select new loops.

Reproduce with `bash scripts/run_stage13b_incremental_isam2.sh` from root and an empty new-stage output directory; runner refuses overwrite. All historical stages immutable.
'''
    (ROOT/'docs/CUMULTI_STAGE13B_INCREMENTAL_ISAM2.md').write_text(text)

def validation_report(report):
    labels={'stage13a_immutable':'STAGE13A INPUT IMMUTABLE','api':'ISAM2 API VERIFIED','reference_preloaded':'ROBOT3 REFERENCE PRELOADED','stream_order_valid':'ROBOT1 STREAM ORDER VALID','no_future_nodes':'NO FUTURE ROBOT1 NODES','frozen_loops_unchanged':'FROZEN LOOP MEASUREMENTS UNCHANGED','independent_pre_bridge':'PRE-FIRST-LOOP INDEPENDENT MAPS','first_connected':'FIRST LOOP CONNECTIVITY 2->1','first_initialization':'FIRST LOOP INITIALIZATION VALID','ISAM2_init':'FIRST ISAM2 INIT','persistent_later_updates':'LATER ISAM2 UPDATES','no_duplicates':'NO DUPLICATE FACTORS','final_nodes':'FINAL NODE COUNT 5796','final_odometry':'FINAL ODOMETRY FACTORS 5794','final_loops':'FINAL LOOP FACTORS 75','final_graph_equivalence':'FINAL GRAPH EQUIVALENCE','ISAM2_batch_agreement':'ISAM2 VS BATCH AGREEMENT','no_catastrophic':'NO NEW CATASTROPHIC JUMPS','first_fusion_timing':'FIRST FUSION LATENCY MEASURED','steady_timing':'STEADY UPDATE LATENCY MEASURED','decision_before_GT':'PRE-GT DECISION SAVED BEFORE GT','offline_GT':'OFFLINE GT'}
    text='\n'.join(f'{labels[k]}: {"PASS" if v else "FAIL"}' for k,v in report.items())
    text+='\nFROZEN LOOP ORDER VALID: PASS\nGT USED FOR BACKEND OR POLICY: NO\nLIVE SENSOR INPUT: NO\nONLINE SC/GICP: NO\nSIMULTANEOUS TWO-ROBOT REPLAY: NO\nDECENTRALIZED OPTIMIZATION: NO\nHISTORICAL STAGES MODIFIED: NO\nFINAL DEMO MODIFIED: NO\n'
    (OUT/'VALIDATION_REPORT.txt').write_text(text)

def report_only():
    """Report formatting only: no solver, re-evaluation or GT access."""
    j=lambda n:json.loads((OUT/n).read_text())
    c=lambda n:pd.read_csv(OUT/n)
    document(j('incremental_policy.json'),j('pre_gt_incremental_decision.json'),j('first_fusion_latency.json'),c('incremental_vs_batch_checkpoints.csv'),c('incremental_map_consistency.csv'),c('incremental_latency_summary.csv'),c('incremental_offline_gt_evaluation.csv'),j('incremental_batch_divergence_diagnosis.json'))
    validation_report(j('summary.json')['validation'])

if __name__=='__main__':
    if sys.argv[1:]==['--report-only']: report_only()
    elif sys.argv[1:]: raise SystemExit('Usage: run_stage13b_incremental_isam2.py [--report-only]')
    else: main()
