#!/usr/bin/env python3
"""One exact historical Stage13B reproduction: current R1 Pose3 only, no full-state saves."""
import sys,json,time
from pathlib import Path
import numpy as np,pandas as pd,gtsam
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent))
from export_stage13b_progressive_states import inv,key,pose,a
OUT=a.ROOT/'outputs/cumulti_v1/18_1_presentation_upgrade'
def main():
    OUT.mkdir(parents=True,exist_ok=True);target=a.DATA/'robot1_causal_pose_stream.json'
    assert not target.exists(),'Refuse pose stream overwrite'
    base=json.loads((a.OUT/'demo_data_provenance.json').read_text());inputs={p['path']:a.sha(p['path']) for p in base['sources'] if p['role'] in ['frozen local poses','exact clean75 frozen arrival order','exact historical incremental default parameters']}
    localtr=pd.read_csv(a.D12/'clean_local_trajectory.csv',usecols=a.POSE)
    local={(r.robot_id,int(r.keyframe_id)):a.matrix(r) for r in localtr.itertuples()}
    timestamp={int(r.keyframe_id):str(r.timestamp) for r in localtr[localtr.robot_id.eq('robot1')].itertuples()}
    loops=pd.read_csv(a.D12/'ordered_rank1_clean_loops.csv');bykf={int(r.query_keyframe_id):r for r in loops.itertuples()}
    S=local[('robot1',283)]@a.matrix(loops.iloc[0])@inv(local[('robot3',252)])
    params=gtsam.ISAM2Params();historical=json.loads((a.B13/'isam2_parameter_policy.json').read_text())
    assert str(params)==historical['parameter_text']
    a.dump(OUT/'pose_stream_extraction_policy.json',dict(accepted_base='0447a62210d46a685285b8f6d649d49fd880960d',frozen_parameters=historical,inputs=inputs,one_persistent_replay=True,one_update_per_query=True,extra_empty_updates=0,only_current_robot1_pose_saved=True,GT_used=False,checkpoint_max_matrix_difference=1e-7))
    template=gtsam.NonlinearFactorGraph();baseNoise=gtsam.noiseModel.Diagonal.Sigmas(np.r_[np.full(3,np.deg2rad(5)),np.ones(3)]);robust=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),baseNoise)
    template.add(gtsam.PriorFactorPose3(key('robot1',150),pose(local[('robot1',150)]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
    for robot,start,end in [('robot1',150,1999),('robot3',234,4179)]:
        for k in range(start+1,end+1):template.add(gtsam.BetweenFactorPose3(key(robot,k-1),key(robot,k),pose(inv(local[(robot,k-1)])@local[(robot,k)]),baseNoise))
    for r in loops.itertuples():template.add(gtsam.BetweenFactorPose3(key('robot1',r.query_keyframe_id),key('robot3',r.candidate_keyframe_id),pose(a.matrix(r)),robust))
    assert template.size()==5870;isam=gtsam.ISAM2(params);updates=0;K=0;rows=[];matches=[]
    cps={x['kf']:x for x in json.loads((a.DATA/'replay.json').read_text())['snapshots'] if x['name'] in ['FIRST_LOOP','K2','K5','K10','K20','K40','K75','FINAL_STREAM_END']}
    for kf in range(150,2000):
        if kf<283:T=local[('robot1',kf)]
        else:
            new=gtsam.NonlinearFactorGraph();v=gtsam.Values();event=bykf.get(kf)
            if kf==283:
                for ix in [0]+list(range(1,134))+list(range(1850,5795))+[5795]:new.add(template.at(ix))
                for robot,start,end in [('robot1',150,283),('robot3',234,4179)]:
                    for k in range(start,end+1):v.insert(key(robot,k),pose(local[(robot,k)] if robot=='robot1' else S@local[(robot,k)]))
            else:
                idx=kf-150;v.insert(key('robot1',kf),isam.calculateEstimatePose3(key('robot1',kf-1)).compose(template.at(idx).measured()));new.add(template.at(idx))
                if event is not None:new.add(template.at(5795+int(event.arrival_index)-1))
            isam.update(new,v);updates+=1;estimate=isam.calculateEstimate();T=estimate.atPose3(key('robot1',kf)).matrix()
            if event is not None:K+=1
            if kf in cps:
                name=cps[kf]['name'];saved=pd.read_csv(a.B13/'snapshots'/f'{name}_trajectory.csv',usecols=a.POSE)
                delta=max(np.max(abs(estimate.atPose3(key(r.robot_id,r.keyframe_id)).matrix()-a.matrix(r))) for r in saved.itertuples());assert delta<1e-7
                matches.append(dict(checkpoint=name,max_matrix_difference=float(delta),PASS=True))
        assert np.isfinite(T).all();q=Rotation.from_matrix(T[:3,:3]).as_quat();rows.append([kf,timestamp[kf],*T[:3,3].tolist(),*q.tolist()])
    assert updates==1717 and K==75 and len(rows)==1850 and len(matches)==8
    assert all(a.sha(fn)==h for fn,h in inputs.items())
    a.dump(target,dict(columns=['KF','timestamp_ns','tx','ty','tz','qx','qy','qz','qw'],rows=rows,source='one exact Stage13B causal display extraction',GT_used=False))
    a.dump(OUT/'pose_stream_validation.json',dict(PASS=True,rows=1850,updates=updates,loops=75,checkpoints=matches,stream_sha256=a.sha(target),new_scientific_experiments=0,GT_used=False))
    print('POSE STREAM PASS',len(rows),updates,max(x['max_matrix_difference'] for x in matches),flush=True)
if __name__=='__main__':main()
