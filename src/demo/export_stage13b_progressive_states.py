#!/usr/bin/env python3
"""Stage18 ONLY: exact Stage13B causal replay to export missing loop display states.
No batch optimization, GT, solver tuning, metrics experiment or frontend calls.
Saved checkpoints are reproduction gates, never parameter-selection targets.
"""
import json, sys, time
from pathlib import Path
import numpy as np
import pandas as pd
import gtsam
sys.path.insert(0,str(Path(__file__).resolve().parent))
import prepare_progressive_multi_robot_assets as a

def inv(T):
    B=np.eye(4);B[:3,:3]=T[:3,:3].T;B[:3,3]=-T[:3,:3].T@T[:3,3];return B
def key(robot,k):return gtsam.symbol('a' if robot=='robot1' else 'b',int(k))
def pose(T):return gtsam.Pose3(gtsam.Rot3(T[:3,:3]),gtsam.Point3(*T[:3,3]))

def main():
    target=a.OUT/'display_state_export_reproduction.json'
    assert not target.exists(),'Refuse completed per-loop-state overwrite'
    replay=json.loads((a.DATA/'replay.json').read_text())
    provenance=json.loads((a.OUT/'demo_data_provenance.json').read_text())
    a.SOURCES.update({x['path']:x for x in provenance['sources']})
    for fn,h in provenance['historical_demo_hashes'].items():assert a.sha(fn)==h
    policy=json.loads(a.source(a.B13/'isam2_parameter_policy.json','exact historical incremental default parameters').read_text())
    params=gtsam.ISAM2Params()
    assert params.relinearizeSkip==10 and gtsam.ISAM2GaussNewtonParams().getWildfireThreshold()==.001
    assert str(params)==policy['parameter_text'],'Exact installed historical parameter text must match'
    # Freeze an export protocol before any backend replay; no threshold search or policy choice.
    a.dump(a.OUT/'display_state_export_policy.json',dict(reference='Stage13B',source_sha256=a.sha(a.ROOT/'src/cumulti/run_stage13b_incremental_isam2.py'),parameters=policy,one_update_per_event=True,empty_updates=0,GT_loaded=False,SC_GICP_runs=0,batch_solves=0,checkpoint_max_matrix_difference_tolerance=1e-7,display_map_cap_per_robot=2500,display_trajectory_stride=5,full_original_graph_preserved=True))
    tr=pd.read_csv(a.source(a.D12/'clean_local_trajectory.csv','frozen local poses'),usecols=a.POSE)
    local={(r.robot_id,int(r.keyframe_id)):a.matrix(r) for r in tr.itertuples()}
    timestamps={(r.robot_id,int(r.keyframe_id)):str(r.timestamp) for r in tr.itertuples()}
    loops=pd.read_csv(a.source(a.D12/'ordered_rank1_clean_loops.csv','exact clean75 frozen arrival order'))
    S=local[('robot1',283)]@a.matrix(loops.iloc[0])@inv(local[('robot3',252)])
    sigmas=np.r_[np.full(3,np.deg2rad(5)),np.ones(3)]
    base=gtsam.noiseModel.Diagonal.Sigmas(sigmas)
    robust=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),base)
    template=gtsam.NonlinearFactorGraph()
    template.add(gtsam.PriorFactorPose3(key('robot1',150),pose(local[('robot1',150)]),gtsam.noiseModel.Diagonal.Sigmas(np.ones(6)*1e-6)))
    for robot,start,end in [('robot1',150,1999),('robot3',234,4179)]:
        for k in range(start+1,end+1):template.add(gtsam.BetweenFactorPose3(key(robot,k-1),key(robot,k),pose(inv(local[(robot,k-1)])@local[(robot,k)]),base))
    for l in loops.itertuples():template.add(gtsam.BetweenFactorPose3(key('robot1',l.query_keyframe_id),key('robot3',l.candidate_keyframe_id),pose(a.matrix(l)),robust))
    assert template.size()==5870
    # Immutable raw frame10/point16/range(1,60) scan cache for map reprojection only.
    clouds={}
    for robot,start,end in [('robot1',150,1999),('robot3',234,4179)]:
        proc=Path('/home/cas/CU-Multi/processed_v1/full_2hz')/robot
        k=pd.read_csv(a.source(proc/'keyframes.csv','LiDAR columns only, GT excluded'),usecols=['keyframe_id','lidar_file'])
        for r in k[k.keyframe_id.between(start,end)].iloc[::10].itertuples():
            fn=Path(r.lidar_file)
            if not fn.is_absolute():fn=proc/'lidar'/fn
            clouds[(robot,int(r.keyframe_id))]=a.cloud(fn)
    cps={int(s['kf']):s for s in replay['snapshots'] if s['K']}
    comparison=[]; additions=[]; isam=gtsam.ISAM2(params); updates=0; K=0
    event_by_k={int(l.query_keyframe_id):l for l in loops.itertuples()}
    for kf in range(283,2000):
        new=gtsam.NonlinearFactorGraph(); values=gtsam.Values()
        event=event_by_k.get(kf)
        if kf==283:
            indices=[0]+list(range(1,134))+list(range(1850,5795))+[5795]
            for ix in indices:new.add(template.at(ix))
            for robot,start,end in [('robot1',150,283),('robot3',234,4179)]:
                for k in range(start,end+1):values.insert(key(robot,k),pose(local[(robot,k)] if robot=='robot1' else S@local[(robot,k)]))
        else:
            idx=kf-150; Z=template.at(idx).measured()
            previous=isam.calculateEstimatePose3(key('robot1',kf-1))
            values.insert(key('robot1',kf),previous.compose(Z));new.add(template.at(idx))
            if event is not None:new.add(template.at(5795+int(event.arrival_index)-1))
        isam.update(new,values);updates+=1
        estimate=isam.calculateEstimate()  # same historical extraction after each update; no extra update
        if event is not None:K+=1
        assert estimate.size()==kf-149+3946 and isam.getFactorsUnsafe().size()==estimate.size()-2+K+1
        if event is None and kf!=1999:continue
        matrices={(robot,k):estimate.atPose3(key(robot,k)).matrix() for robot,start,end in [('robot1',150,kf),('robot3',234,4179)] for k in range(start,end+1)}
        assert all(np.isfinite(T).all() for T in matrices.values())
        if kf in cps:
            cp=cps[kf]; saved=pd.read_csv(a.source(a.B13/'snapshots'/f'{cp["name"]}_trajectory.csv','historical reproduction gate'),usecols=a.POSE)
            delta=max(np.max(abs(matrices[(r.robot_id,int(r.keyframe_id))]-a.matrix(r))) for r in saved.itertuples())
            assert delta<1e-7,f'Historical replay fidelity failed at {cp["name"]}: {delta}; no retuning permitted'
            comparison.append(dict(checkpoint=cp['name'],kf=kf,max_matrix_difference=float(delta),PASS=True))
            print('REPRODUCED',cp['name'],delta,flush=True)
            continue
        rows={}; maps={}
        for robot,start,end in [('robot1',150,kf),('robot3',234,4179)]:
            ids=set(range(start,end+1,5))|{start,end}
            ids.update(int(l.query_keyframe_id if robot=='robot1' else l.candidate_keyframe_id) for l in loops.iloc[:K].itertuples())
            rows[robot]=[[k,timestamps[(robot,k)],*matrices[(robot,k)][:3,3].tolist()] for k in sorted(ids)]
            acc=[]
            for (r,k),p in clouds.items():
                if r==robot and k<=end:
                    T=matrices[(r,k)];acc.append(p@T[:3,:3].T+T[:3,3])
            p=np.vstack(acc);_,ix=np.unique(np.floor(p/.5).astype(np.int64),axis=0,return_index=True)
            maps[robot]=a.sample(p[np.sort(ix)],2500)
        name=f'EVENT_{K:02d}';fn=name+'.json'
        state=dict(name=name,kf=kf,K=K,nodes=estimate.size(),frame='ROBOT1_KF150_GAUGE',trajectory=rows,maps=maps,origin='EXACT_STAGE13B_DISPLAY_REPLAY',render_trajectory_decimation_only=True)
        a.dump(a.DATA/fn,state); additions.append({x:state[x] for x in ['name','kf','K','nodes','frame']}|{'file':fn,'sha256':a.sha(a.DATA/fn)})
        print('DISPLAY STATE',K,kf,flush=True)
    assert K==75 and updates==1717 and len(comparison)==8
    assert all(a.sha(fn)==h for fn,h in provenance['historical_demo_hashes'].items())
    replay['snapshots']=sorted(replay['snapshots']+additions,key=lambda s:s['kf']);replay['per_event_backend_export']=True
    a.dump(a.DATA/'replay.json',replay)
    provenance['sources']=list(a.SOURCES.values());provenance['display_state_export']=dict(persistent_replay_count=1,updates=updates,extra_empty_updates=0,new_loop_display_states=len(additions),historical_checkpoints_verified=comparison,batch_solves=0,SC_GICP_runs=0,GT_loaded=False,parameter_changes=False,scientific_experiments=0)
    provenance['estimate_policy']='latest real causal post-loop estimate; between loop arrivals geometry held, no interpolation; existing golden Stage13B checkpoints reused'
    a.dump(a.OUT/'demo_data_provenance.json',provenance);a.dump(a.OUT/'display_state_export_reproduction.json',provenance['display_state_export'])
    print('PER-EVENT DISPLAY EXPORT COMPLETE',updates,'updates;',len(additions),'new display states',flush=True)
if __name__=='__main__':main()
