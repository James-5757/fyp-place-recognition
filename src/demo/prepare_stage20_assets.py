#!/usr/bin/env python3
"""Package actual Stage20 prefix states; exact replay equivalence is a gate."""
import json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from export_stage19_rgb import ROOT,dump,digest
from prepare_progressive_multi_robot_assets import ply
OUT=ROOT/'outputs/cumulti_v1/20_dual_stream'
REPRO=OUT/'20b_display_reproduction'
SITE=ROOT/'demo/stage20_dual_stream'

def compact(p,x):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,separators=(',',':'),allow_nan=False)+'\n')

def main():
    assert json.loads((OUT/'stage20a_backend_decision.json').read_text())['status']=='DUAL_STREAM_BACKEND_READY'
    # Empty display-only comparison sets are NOT a numerical agreement PASS.
    # This is metadata cleanup only; primary experiment diagnostics stay untouched.
    if not (REPRO/'incremental_vs_batch_diagnostics.csv').read_text().strip():
        for name in ['stage20a_backend_decision.json','summary.json']:
            v=json.loads((REPRO/name).read_text());v['numerical_batch_agreement_all']=None
            v['numerical_diagnostic_status']='NOT_RUN_DISPLAY_EXTRACTION_ONLY';dump(REPRO/name,v)
        p=REPRO/'VALIDATION_REPORT.txt';p.write_text(p.read_text().replace('NUMERICAL AGREEMENT: True','NUMERICAL AGREEMENT: NOT_RUN_DISPLAY_EXTRACTION_ONLY'))
    assert json.loads((REPRO/'stage20a_backend_decision.json').read_text())['status']=='DUAL_STREAM_BACKEND_READY'
    primary=json.loads((OUT/'backend_state_manifest.json').read_text());states=json.loads((REPRO/'backend_state_manifest.json').read_text());byname={s['name']:s for s in states}
    maxpos=0.;maxrot=0.
    for s in primary:
        a=pd.read_csv(s['trajectory']);b=pd.read_csv(byname[s['name']]['trajectory'])
        assert a[['robot_id','keyframe_id','timestamp']].equals(b[['robot_id','keyframe_id','timestamp']])
        maxpos=max(maxpos,float(np.linalg.norm(a[['tx','ty','tz']].to_numpy()-b[['tx','ty','tz']].to_numpy(),axis=1).max()))
        diff=Rotation.from_quat(a[['qx','qy','qz','qw']].to_numpy()).inv()*Rotation.from_quat(b[['qx','qy','qz','qw']].to_numpy());maxrot=max(maxrot,float(diff.magnitude().max()))
    assert maxpos<1e-7 and maxrot<1e-7
    t=pd.read_csv(OUT/'central_incremental_event_trace.csv',keep_default_na=False);u=pd.read_csv(REPRO/'central_incremental_event_trace.csv',keep_default_na=False)
    fields=['robot_id','keyframe_id','timestamp_ns','normalized_ns','event_index','current_robot1_kf','current_robot3_kf','new_loop_ids','cumulative_loops','central_nodes','central_factors']
    assert t[fields].equals(u[fields])
    dump(OUT/'stage20b_display_reproduction_validation.json',dict(status='PASS',primary_checkpoints=len(primary),display_checkpoints=len(states),max_translation_difference_m=maxpos,max_rotation_difference_rad=maxrot,event_identity=True,extra_isam_updates=0,policy='same frozen policy and order; additional display snapshots only; primary timing/numerical result unchanged',primary_numerical_diagnostics='retained; not rerun in display extraction'))
    local=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv',usecols=['robot_id','keyframe_id','timestamp','tx','ty','tz','qx','qy','qz','qw'])
    old=json.loads((ROOT/'demo/stage19_rgb_three_map/local_maps.json').read_text());chunks=[];sources=[]
    key=pd.read_csv('/home/cas/CU-Multi/processed_v1/full_2hz/robot3/keyframes.csv',usecols=['keyframe_id','lidar_file']).set_index('keyframe_id');rt=local[local.robot_id.eq('robot3')].set_index('keyframe_id')
    for k in range(234,4180,10):
        p=Path(key.loc[k].lidar_file)
        if not p.is_absolute():p=Path('/home/cas/CU-Multi/processed_v1/full_2hz/robot3/lidar')/p
        cloud=np.asarray(np.load(p,mmap_mode='r')[::16,:3],float);norm=np.linalg.norm(cloud,axis=1);cloud=cloud[np.isfinite(cloud).all(axis=1)&(norm>1)&(norm<60)]
        pose=rt.loc[k];cloud=cloud@Rotation.from_quat(pose[['qx','qy','qz','qw']].to_numpy(float)).as_matrix().T+pose[['tx','ty','tz']].to_numpy(float)
        _,ix=np.unique(np.floor(cloud/.5).astype(np.int64),axis=0,return_index=True);cloud=cloud[np.sort(ix)];cloud=np.round(cloud[::max(1,math.ceil(len(cloud)/180))],3)
        chunks.append(dict(kf=k,points=cloud.tolist()));sources.append(dict(path=str(p),sha256=digest(p)))
    # Replace the old prebuilt R3 cloud with availability-gated local chunks.
    old.update(robot3_points=[],robot3_chunks=chunks,GT_used=False)
    compact(SITE/'local_maps.json',old)
    loops=pd.read_csv(OUT/'loop_eligibility_schedule.csv');events=[]
    for r in loops.itertuples():events.append(dict(index=int(r.eligibility_order),frozen_loop_id=int(r.frozen_loop_id),historical_order=int(r.historical_order),query=int(r.query_keyframe_id),candidate=int(r.candidate_keyframe_id),event_index=int(r.release_event_index),normalized_ns=str(r.eligibility_ns)))
    exported=[];bounds={}
    for s in states:
        tr=pd.read_csv(s['trajectory']);rows={};maps={}
        for robot,end in [('robot1',s['robot1_kf']),('robot3',s['robot3_kf'])]:
            a=tr[tr.robot_id.eq(robot)].sort_values('keyframe_id');required={e['query' if robot=='robot1' else 'candidate'] for e in events[:s['K']]}
            select=a.keyframe_id.isin(required|{int(a.keyframe_id.iloc[0]),int(a.keyframe_id.iloc[-1])})|(np.arange(len(a))%3==0);a=a[select]
            rows[robot]=[[int(r.keyframe_id),str(r.timestamp),r.tx,r.ty,r.tz,r.qx,r.qy,r.qz,r.qw] for r in a.itertuples()]
            # Independent local PLY must never appear in an uninitialized global panel.
            if s['global_initialized']:
                points=np.array(ply(next(v['path'] for v in s['maps'] if v['robot']==robot)))
                cap=10000 if s['name']=='FINAL' else 2500;points=points[::max(1,math.ceil(len(points)/cap))];maps[robot]=points.tolist()
            else:maps[robot]=[]
        xyz=np.array([v[2:5] for r in rows.values() for v in r]);b=[*xyz.min(axis=0).tolist(),*xyz.max(axis=0).tolist()]
        filename=s['name']+'.json';bounds[filename]=dict(global_=b);bounds[filename]['global']=bounds[filename].pop('global_')
        record=dict(name=s['name'],file=filename,kf=s['robot1_kf'],r3kf=s['robot3_kf'],K=s['K'],event_index=s['event_index'],normalized_ns=str(s['normalized_ns']),global_initialized=s['global_initialized'],trajectory=rows,maps=maps)
        compact(SITE/'states'/filename,record);exported.append({k:v for k,v in record.items() if k not in ['trajectory','maps']})
    compact(SITE/'bounds.json',bounds)
    schedule=[[int(r.event_index),str(r.normalized_ns),r.robot_id,int(r.keyframe_id),int(r.current_robot1_kf),int(r.current_robot3_kf),int(r.cumulative_loops),int(r.components),bool(r.global_initialized)] for r in t.itertuples()]
    jumps={s['name']:s['event_index'] for s in states if not s['name'].startswith(('LOOP_','PERIODIC_'))}
    origins={r:str(t[t.robot_id.eq(r)].timestamp_ns.iloc[0]) for r in ['robot1','robot3']}
    compact(SITE/'replay.json',dict(events=events,schedule=schedule,states=exported,jumps=jumps,origins=origins,base_compression=10,limits=dict(robot1=[150,1999],robot3=[234,4179]),GT_used=False,scientific='Hypothetical normalized two-stream offline replay; precomputed loop factors; centralized iSAM2',numerical_batch_agreement_all=False))
    dump(OUT/'demo_event_manifest.json',dict(jumps=jumps,states=exported,first10=[e['frozen_loop_id'] for e in events[:10]],base_compression=10,global_pose_policy='latest actual saved state at/before selected event; step-held and explicitly labelled, no interpolation',local_pose_policy='each own arrived immutable local prefix',sources=sources,GT_used=False))
    artifacts=[dict(path=str(p.relative_to(ROOT)),size=p.stat().st_size,sha256=digest(p)) for p in sorted(SITE.rglob('*')) if p.is_file() and p.suffix in ['.json','.jpg']]
    dump(OUT/'stage20b_asset_manifest.json',dict(artifacts=artifacts,source_state_directory=str(REPRO),GT_used=False))
    print('PACKAGED',len(states),'real states',len(chunks),'R3 local chunks',flush=True)

if __name__=='__main__':main()
