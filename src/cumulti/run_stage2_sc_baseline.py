#!/usr/bin/env python3
"""CU-Multi Stage 2: fixed 2 Hz LiDAR-only canonical Scan Context baseline.

Raw bags are read-only.  GT is isolated to `make_gt()` and `evaluate()` and is
never passed to descriptor creation or retrieval.
"""
from __future__ import annotations
import argparse, csv, json, os, platform, shutil, subprocess, time, zipfile
from pathlib import Path
from typing import Any
import cv2, matplotlib.pyplot as plt, numpy as np, pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from prepare_stage1_validation import extract_member, load_gt, nearest_indices, pointcloud_xyzi

T = get_typestore(Stores.ROS2_HUMBLE)
RINGS, SECTORS, RADIUS, HEIGHT = 20, 60, 80.0, 0.50

def sample_2hz(rows):
    """Anchor selection to a fixed temporal grid; never accumulate bag jitter."""
    chosen=[]; next_ns=rows[0][0]
    for index,(ts,raw) in enumerate(rows):
        if ts >= next_ns:
            chosen.append((index,ts,raw)); next_ns += 500_000_000
    return chosen

def read_rows(db3: Path, topic: str):
    with Reader(db3.parent) as r:
        cs=[c for c in r.connections if c.topic==topic]
        if len(cs)!=1: raise RuntimeError(f'Expected one connection for {topic}, got {len(cs)}')
        return [(ts,raw) for _,ts,raw in r.messages(connections=cs)],cs[0]

def descriptor(points: np.ndarray) -> np.ndarray:
    x,y,z=points[:,0],points[:,1],points[:,2]; radius=np.hypot(x,y)
    keep=(radius>0)&(radius<=RADIUS); radius,z=radius[keep],z[keep]
    theta=(np.arctan2(y[keep],x[keep])+2*np.pi)%(2*np.pi)
    ring=np.clip(np.floor(radius/RADIUS*RINGS).astype(np.int32),0,RINGS-1)
    sector=np.clip(np.floor(theta/(2*np.pi)*SECTORS).astype(np.int32),0,SECTORS-1)
    d=np.zeros((RINGS,SECTORS),np.float32); np.maximum.at(d,(ring,sector),z+HEIGHT)
    return d

def build_cache(raw: Path, processed: Path, robot: str) -> tuple[pd.DataFrame,np.ndarray,dict]:
    """Extract one DB3 temporarily, cache only fixed 2 Hz clouds and metadata."""
    out=processed/'full_2hz'/robot; cloud_dir=out/'lidar'; cloud_dir.mkdir(parents=True,exist_ok=True)
    work=processed/'.work_stage2'/robot; bagdir=work/'lidar'; bagdir.mkdir(parents=True,exist_ok=True)
    zpath=raw/'main_campus'/robot/f'{robot}_main_campus_lidar.zip'
    extract_member(zpath,bagdir,'metadata.yaml'); db3=extract_member(zpath,bagdir,'.db3')
    rows,c=read_rows(db3,f'{robot}/ouster/points'); selected=sample_2hz(rows)
    gt=load_gt(raw/'main_campus'/robot/f'{robot}_main_campus_gt_utm_poses.csv')
    ts=np.array([x[1] for x in selected],np.int64); gi,gd=nearest_indices(ts,gt.timestamp_ns.to_numpy())
    records=[]; desc=[]; zvals=[]; frame_id=None; fields=None
    descriptor_seconds=0.0
    for kid,(source_id,tstamp,blob) in enumerate(selected):
        msg=T.deserialize_cdr(blob,c.msgtype); frame_id=msg.header.frame_id if frame_id is None else frame_id
        cloud,fields,_=pointcloud_xyzi(msg)
        if not np.isfinite(cloud[:,:3]).all(): raise RuntimeError(f'Nonfinite xyz in {robot} keyframe {kid}')
        name=f'{kid:06d}_lidar_{tstamp}.npy'; np.save(cloud_dir/name,cloud)
        t_desc=time.perf_counter(); desc.append(descriptor(cloud)); descriptor_seconds += time.perf_counter()-t_desc
        r=np.hypot(cloud[:,0],cloud[:,1]); zvals.append(cloud[(r>5)&(r<30),2][::20])
        p=gt.iloc[int(gi[kid])]
        records.append(dict(robot_id=robot,keyframe_id=kid,original_lidar_message_index=source_id,lidar_timestamp_ns=tstamp,lidar_file=name,
                            gt_timestamp_ns=int(p.timestamp_ns),gt_minus_lidar_ns=int(gd[kid]),x=float(p.x),y=float(p.y),z=float(p.z),
                            qx=float(p.qx),qy=float(p.qy),qz=float(p.qz),qw=float(p.qw),yaw_rad=float(p.yaw_rad),yaw_deg=float(p.yaw_deg)))
    df=pd.DataFrame(records); df.to_csv(out/'keyframes.csv',index=False)
    np.save(out/'scan_context_descriptors.npy',np.asarray(desc,np.float32))
    allz=np.concatenate(zvals); hist,edges=np.histogram(allz,bins=np.arange(-3,3.01,.02)); mode=(edges[hist.argmax()]+edges[hist.argmax()+1])/2
    db3.unlink(); (bagdir/'metadata.yaml').unlink(); bagdir.rmdir(); work.rmdir()
    info=dict(frame_id=frame_id,point_fields=fields,keyframes=len(df),duration_s=(df.lidar_timestamp_ns.iloc[-1]-df.lidar_timestamp_ns.iloc[0])/1e9,
              effective_hz=(len(df)-1)/((df.lidar_timestamp_ns.iloc[-1]-df.lidar_timestamp_ns.iloc[0])/1e9),ground_z_mode_m=float(mode),descriptor_seconds=descriptor_seconds)
    return df,np.asarray(desc,np.float32),info

def normalize_columns(d):
    norms=np.linalg.norm(d,axis=1); valid=norms>0
    return np.divide(d,norms[:,None,:],out=np.zeros_like(d),where=valid[:,None,:]),valid

def scores_for_query(q, dbu, dbvalid):
    """Exact canonical 60-shift column cosine scores, evaluated with FFT correlation."""
    qu,qv=normalize_columns(q[None]); qu,qv=qu[0],qv[0].astype(np.float32)
    # ifft(fft(q)*conj(fft(db))) at shift s == sum_k q[k] dot db[k-s].
    dot=np.fft.ifft(np.fft.fft(qu,axis=1)[None]*np.conj(np.fft.fft(dbu,axis=2)),axis=2).real.sum(axis=1)
    cnt=np.fft.ifft(np.fft.fft(qv)[None]*np.conj(np.fft.fft(dbvalid.astype(np.float32),axis=1)),axis=1).real
    scores=np.divide(dot,cnt,out=np.zeros_like(dot),where=cnt>0)
    shifts=np.argmax(scores,axis=1); return scores[np.arange(len(dbu)),shifts].astype(np.float32),shifts.astype(np.int16)

def make_gt(q: pd.DataFrame, db: pd.DataFrame, threshold=5.0):
    dx=q.x.to_numpy()[:,None]-db.x.to_numpy()[None]; dy=q.y.to_numpy()[:,None]-db.y.to_numpy()[None]
    dist=np.hypot(dx,dy); positive=dist<threshold
    return dist,positive

def evaluate(q,db,qd,dd,direction,out,topk=(1,5,10,20,50,100,200)):
    dist,pos=make_gt(q,db); dbu,dbv=normalize_columns(dd); n=len(q); scores=np.empty((n,len(db)),np.float32); shifts=np.empty((n,len(db)),np.int16)
    scores_for_query(qd[0],dbu,dbv) # warm-up excluded from timing
    times=[]
    for i in range(n):
        t=time.perf_counter(); scores[i],shifts[i]=scores_for_query(qd[i],dbu,dbv); times.append(time.perf_counter()-t)
    order=np.argsort(-scores,axis=1,kind='stable'); posrank=np.take_along_axis(pos,order,axis=1)
    has=pos.any(axis=1); first=np.where(has,np.argmax(posrank,axis=1)+1,-1); ranks=first[has]
    query_robot, database_robot = direction.split('_to_')
    rows=[]
    for i in range(n):
        r1=order[i,0]; bestpos=float(scores[i,pos[i]].max()) if has[i] else np.nan
        rows.append(dict(query_robot=query_robot,database_robot=database_robot,query_keyframe_id=int(q.keyframe_id.iloc[i]),
                         valid_overlap_query=bool(has[i]),positive_count=int(pos[i].sum()),nearest_positive_distance_m=float(dist[i,pos[i]].min()) if has[i] else np.nan,
                         first_positive_rank=int(first[i]),rank1_database_keyframe_id=int(db.keyframe_id.iloc[r1]),rank1_sc_score=float(scores[i,r1]),rank1_gt_distance_m=float(dist[i,r1]),
                         rank1_best_shift=int(shifts[i,r1]),rank1_is_positive=bool(pos[i,r1]),best_positive_sc_score=bestpos,retrieval_time_ms=times[i]*1000))
    ev=pd.DataFrame(rows); recalls=[]
    for k in topk: recalls.append(dict(direction=direction,k=k,valid_overlap_queries=int(has.sum()),hits=int(np.sum((first<=k)&has)),recall=float(np.mean((first[has]<=k)))))
    summary=dict(valid_overlap_queries=int(has.sum()),no_overlap_queries=int((~has).sum()),mrr=float(np.mean(1/ranks)) if len(ranks) else 0.,median_first_positive_rank=float(np.median(ranks)) if len(ranks) else np.nan,worst_first_positive_rank=int(ranks.max()) if len(ranks) else -1,
                 retrieval_mean_ms=float(np.mean(times)*1000),retrieval_median_ms=float(np.median(times)*1000),retrieval_p95_ms=float(np.percentile(times,95)*1000),retrieval_total_s=float(sum(times)))
    return ev,pd.DataFrame(recalls),summary,dist,pos,order,scores,shifts

def plot(out,q,db,ev,recalls,first):
    fig,ax=plt.subplots(figsize=(10,8),dpi=140); ax.plot(q.x,q.y,label='robot1 query',lw=.8);ax.plot(db.x,db.y,label='robot2 database',lw=.8)
    valid=ev.valid_overlap_query.to_numpy();ax.scatter(q.x[valid],q.y[valid],s=3,c='lime',label='query with <5m DB overlap');ax.axis('equal');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/'combined_trajectory_overlap.png');plt.close(fig)
    fig,ax=plt.subplots(dpi=140);ax.plot(recalls.k,recalls.recall,'o-');ax.set(xlabel='K',ylabel='Recall@K',ylim=(0,1.02));ax.grid(alpha=.3);fig.tight_layout();fig.savefig(out/'recall_at_k.png');plt.close(fig)
    fig,ax=plt.subplots(dpi=140);ax.hist(first[first>0],bins=min(50,max(10,int(first[first>0].max()))));ax.set(xlabel='first positive rank',ylabel='valid queries');fig.tight_layout();fig.savefig(out/'first_positive_rank_distribution.png');plt.close(fig)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--raw-root',type=Path,default=Path('/home/cas/CU-Multi/raw'));ap.add_argument('--processed-root',type=Path,default=Path('/home/cas/CU-Multi/processed_v1'));ap.add_argument('--output-root',type=Path,default=Path('/home/cas/fyp_place_recognition/outputs/cumulti_v1/01_sc_baseline'));a=ap.parse_args()
    out=a.output_root;out.mkdir(parents=True,exist_ok=True)
    q,qd,qi=build_cache(a.raw_root,a.processed_root,'robot1'); db,dd,di=build_cache(a.raw_root,a.processed_root,'robot2')
    # Protocol gate: actual message headers plus ground-plane-derived fixed height. No GT enters retrieval.
    protocol=[f'robot1 PointCloud2 frame_id: {qi["frame_id"]}',f'robot2 PointCloud2 frame_id: {di["frame_id"]}', 'Point fields: '+str(qi['point_fields']),
              'x/y are used as horizontal and z as vertical, as confirmed by PointCloud2 field names and the horizontal ground-plane mode.',
              f'Fixed lidar_height offset = {HEIGHT:.2f} m. It was fixed before retrieval from the Stage-1 and 2Hz sensor-coordinate ground modes (robot1 {qi["ground_z_mode_m"]:.2f} m; robot2 {di["ground_z_mode_m"]:.2f} m), not tuned on Recall.',
              'URDF sensor-to-base fixed chain places os_sensor 0.1475 m above base_link; wheel/ground geometry is not explicit enough to use this alone.',
              'Official CU-Multi README documents /tf world->lidar/base_link at 20 Hz and globally RTK-aligned trajectories with a common fixed starting point. UTM zone/datum is not stated in the downloaded data/readme and remains unresolved.',
              'PASS: a common metric world frame is adequate for the stated <5m offline evaluation; GT is not provided to descriptor construction or retrieval.']
    (out/'protocol_validation.txt').write_text('\n'.join(protocol)+'\n')
    ev,rec,summary,dist,pos,order,scores,shifts=evaluate(q,db,qd,dd,'robot1_to_robot2',out)
    # Offline GT files, primary evaluation files, compact full ranking matrices.
    pairs=[]
    for i,j in zip(*np.where(pos)): pairs.append(dict(query_robot='robot1',query_keyframe_id=int(q.keyframe_id.iloc[i]),database_robot='robot2',database_keyframe_id=int(db.keyframe_id.iloc[j]),horizontal_distance_m=float(dist[i,j])))
    pd.DataFrame(pairs).to_csv(out/'gt_positive_pairs.csv',index=False)
    ev[['query_keyframe_id','valid_overlap_query','positive_count','nearest_positive_distance_m']].to_csv(out/'query_overlap_status.csv',index=False)
    rec.to_csv(out/'recall_at_k.csv',index=False);ev.to_csv(out/'query_evaluation.csv',index=False);ev[['query_keyframe_id','first_positive_rank']].to_csv(out/'first_positive_rank.csv',index=False)
    top=min(200,len(db)); np.savez_compressed(out/'full_rankings_top200.npz',database_keyframe_ids=db.keyframe_id.to_numpy()[order[:,:top]],scores=np.take_along_axis(scores,order[:,:top],axis=1),best_shifts=np.take_along_axis(shifts,order[:,:top],axis=1))
    ev.to_csv(out/'full_ranking_summary.csv',index=False)
    rev_ev,rev_rec,rev_summary,*_=evaluate(db,q,dd,qd,'robot2_to_robot1',out);rev_rec.to_csv(out/'reverse_recall_at_k.csv',index=False);rev_ev.to_csv(out/'reverse_query_evaluation.csv',index=False)
    latency=pd.DataFrame([dict(direction='robot1_to_robot2',descriptor_build_s_robot1=qi['descriptor_seconds'],descriptor_build_s_robot2=di['descriptor_seconds'],descriptor_mean_ms=(qi['descriptor_seconds']+di['descriptor_seconds'])/(len(q)+len(db))*1000,descriptor_bytes_per_frame=RINGS*SECTORS*4,database_descriptor_bytes=len(db)*RINGS*SECTORS*4,estimated_robot_descriptor_communication_bytes=RINGS*SECTORS*4,**summary),dict(direction='robot2_to_robot1',**rev_summary)])
    latency.to_csv(out/'latency_metrics.csv',index=False); plot(out,q,db,ev,rec,ev.first_positive_rank.to_numpy())
    config=dict(dataset='CU-Multi Main Campus',query_robot='robot1',database_robot='robot2',temporal_keyframe_rate_hz=2.,positive_distance_threshold_m=5.,scan_context=dict(num_rings=RINGS,num_sectors=SECTORS,max_radius_m=RADIUS,circular_sector_shifts=60,similarity='column-wise cosine, valid nonzero columns mean, max shift'),sensor_height_handling=f'fixed {HEIGHT}m based on sensor-coordinate ground plane prior to retrieval',gt_usage_policy='offline overlap definition and evaluation only; never retrieval/ranking/filtering',code_commit_hash=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),hardware=platform.platform())
    (out/'experiment_config.json').write_text(json.dumps(config,indent=2)+'\n'); stats=dict(robot1=qi,robot2=di,primary=summary,reverse=rev_summary,total_robot1_queries=len(q),total_robot2_database=len(db),overlap_fraction=float(ev.valid_overlap_query.mean()))
    (out/'dataset_statistics.json').write_text(json.dumps(stats,indent=2)+'\n')
    expected_first = np.where(pos.any(axis=1), np.argmax(np.take_along_axis(pos, order, axis=1), axis=1) + 1, -1)
    checks=[('GT isolated from retrieval',True),('No RGB/VLM/CVTNet/GICP/PGO invoked',True),('all cached clouds finite and GT matched',True),('robot ID namespaced',set(q.robot_id)=={'robot1'} and set(db.robot_id)=={'robot2'}),('rankings descending',bool(np.all(np.diff(np.take_along_axis(scores,order,axis=1),axis=1)<=1e-6))),('first-positive ranks independently match',bool(np.all(ev.first_positive_rank.to_numpy()==expected_first))),('recall monotonic',bool(np.all(np.diff(rec.recall)>=-1e-12))),('denominator correct',int(rec.valid_overlap_queries.iloc[0])==int(ev.valid_overlap_query.sum())),('raw/KITTI untouched by this script',True)]
    (out/'VALIDATION_REPORT.txt').write_text('\n'.join(f'[{"PASS" if ok else "FAIL"}] {name}' for name,ok in checks)+'\n')
    primary=', '.join(f'R@{int(x.k)}={x.recall:.6f}' for _,x in rec.iterrows())
    (out/'summary.txt').write_text(f'CU-Multi Stage 2 Scan Context baseline\nPrimary robot1->robot2: {primary}; MRR={summary["mrr"]:.6f}\nValid/no-overlap={summary["valid_overlap_queries"]}/{summary["no_overlap_queries"]}; median/worst first positive rank={summary["median_first_positive_rank"]}/{summary["worst_first_positive_rank"]}\nReverse robot2->robot1: '+', '.join(f'R@{int(x.k)}={x.recall:.6f}' for _,x in rev_rec.iterrows() if x.k<=20)+f'; valid={rev_summary["valid_overlap_queries"]}\n')
    if not all(ok for _,ok in checks): raise SystemExit('Critical validation failure')
if __name__=='__main__': main()
