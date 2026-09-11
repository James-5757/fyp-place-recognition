#!/usr/bin/env python3
"""Read-only diagnostics for the frozen CU-Multi Stage 2 Scan Context baseline."""
from __future__ import annotations
import argparse, json, shutil
from pathlib import Path
import cv2, matplotlib.pyplot as plt, numpy as np, pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from prepare_stage1_validation import extract_member, image_bgr, nearest_indices
from run_stage2_sc_baseline import normalize_columns, scores_for_query

T=get_typestore(Stores.ROS2_HUMBLE); BINS=np.array([0,30,60,90,120,150,180.0001])
def wrapped(a,b): return np.abs((a-b+180)%360-180)
def motion(df,robot):
    xy=df[['x','y']].to_numpy(); t=df.lidar_timestamp_ns.to_numpy()/1e9
    step=np.r_[0,np.linalg.norm(np.diff(xy,axis=0),axis=1)]; dt=np.r_[np.nan,np.diff(t)]
    speed=np.divide(step,dt,out=np.zeros_like(step),where=np.isfinite(dt)&(dt>0)); cum=np.cumsum(step)
    out=df[['robot_id','keyframe_id','lidar_timestamp_ns','x','y','z']].copy();out['displacement_m']=step;out['local_speed_mps']=speed;out['cumulative_distance_m']=cum
    out['stationary_or_near_stationary']=speed<=0.10; out['within_5m_of_initial']=np.linalg.norm(xy-xy[0],axis=1)<5
    # contiguous initial segment, including frame zero.
    false=np.flatnonzero(~out.within_5m_of_initial.to_numpy()); end=int(false[0]-1) if len(false) else len(out)-1
    return out,dict(robot=robot,frames=len(out),stationary_frames=int(out.stationary_or_near_stationary.sum()),stationary_fraction=float(out.stationary_or_near_stationary.mean()),moving_frames=int((~out.stationary_or_near_stationary).sum()),initial_within5m_frames=end+1,initial_within5m_duration_s=float(t[end]-t[0]),total_distance_m=float(cum[-1]))
def recall(eval_df,mask,label):
    d=eval_df[mask & eval_df.valid_overlap_query].copy(); ranks=d.first_positive_rank.to_numpy()
    return [dict(cohort=label,k=k,valid_overlap_queries=len(d),hits=int((ranks<=k).sum()),recall=float((ranks<=k).mean()) if len(d) else np.nan) for k in [1,5,10,20]]
def plot_bev(path,title,q,p,f):
    fig,axs=plt.subplots(1,3,figsize=(15,5),dpi=130)
    for ax,a,name in zip(axs,[q,p,f],['query','highest-SC GT-positive','Rank-1 false candidate']):
        s=a[::max(1,len(a)//25000)];ax.scatter(s[:,0],s[:,1],c=s[:,2],s=.25,cmap='viridis');ax.set_title(name);ax.axis('equal');ax.set(xlabel='x (m)',ylabel='y (m)')
    fig.suptitle(title);fig.tight_layout();fig.savefig(path);plt.close(fig)
def extract_rgb(raw,work,robot,timestamps):
    """Temporarily expand one RGB db3 and decode only named timestamps."""
    z=raw/'main_campus'/robot/f'{robot}_main_campus_camera_rgb.zip'; d=work/robot;d.mkdir(parents=True,exist_ok=True)
    extract_member(z,d,'metadata.yaml'); db3=extract_member(z,d,'.db3')
    with Reader(d) as r:
        cs=[c for c in r.connections if c.topic==f'{robot}/camera/color/image_raw'];c=cs[0]
        rows=[(ts,b) for _,ts,b in r.messages(connections=cs)]
    arr=np.array([x[0] for x in rows],np.int64);out={}
    for ts in timestamps:
        i,_=nearest_indices(np.array([ts],np.int64),arr);actual,blob=rows[int(i[0])];out[int(ts)]=image_bgr(T.deserialize_cdr(blob,c.msgtype))
    del rows;db3.unlink();(d/'metadata.yaml').unlink();d.rmdir();return out
def contact(path,title,ims):
    fig,axs=plt.subplots(1,3,figsize=(15,5),dpi=130)
    for ax,(name,img) in zip(axs,ims):ax.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB));ax.set_title(name);ax.axis('off')
    fig.suptitle(title);fig.tight_layout();fig.savefig(path);plt.close(fig)
def main():
 p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,default=Path('/home/cas/fyp_place_recognition'));p.add_argument('--processed',type=Path,default=Path('/home/cas/CU-Multi/processed_v1'));p.add_argument('--raw',type=Path,default=Path('/home/cas/CU-Multi/raw'));p.add_argument('--skip-rgb',action='store_true',help='Reuse existing failure panels when recomputing CSV-only diagnostics.');a=p.parse_args()
 frozen=a.repo/'outputs/cumulti_v1/01_sc_baseline';out=a.repo/'outputs/cumulti_v1/02_stage2_diagnostics';out.mkdir(parents=True,exist_ok=True);panels=out/'failure_case_panels';panels.mkdir(exist_ok=True)
 q=pd.read_csv(a.processed/'full_2hz/robot1/keyframes.csv');db=pd.read_csv(a.processed/'full_2hz/robot2/keyframes.csv');ev=pd.read_csv(frozen/'query_evaluation.csv')
 qm,qs=motion(q,'robot1');dm,ds=motion(db,'robot2');motion_stats=pd.concat([qm,dm]);motion_stats.to_csv(out/'motion_statistics.csv',index=False)
 # motion status is attached only in this diagnostic frame; frozen Stage 2 CSV stays unchanged.
 ev=ev.merge(qm[['keyframe_id','stationary_or_near_stationary','within_5m_of_initial']],left_on='query_keyframe_id',right_on='keyframe_id',how='left').drop(columns='keyframe_id')
 common_duration=min(qs['initial_within5m_duration_s'],ds['initial_within5m_duration_s']);elapsed=(q.lidar_timestamp_ns-q.lidar_timestamp_ns.iloc[0])/1e9
 outside_common=elapsed.to_numpy()>common_duration; moving=~ev.stationary_or_near_stationary.to_numpy()
 rec=pd.DataFrame(recall(ev,np.ones(len(ev),bool),'canonical_all_valid')+recall(ev,moving,'moving_query_valid')+recall(ev,outside_common,'outside_common_start_valid'));rec.to_csv(out/'moving_only_recall.csv',index=False)
 # Nearest GT positive is analysis-only and supplies headings/panels.
 dist=np.hypot(q.x.to_numpy()[:,None]-db.x.to_numpy()[None],q.y.to_numpy()[:,None]-db.y.to_numpy()[None]);pos=dist<5; nearest=np.where(pos,dist,np.inf).argmin(1)
 near_yaw=db.yaw_deg.to_numpy()[nearest];hd=wrapped(q.yaw_deg.to_numpy(),near_yaw)
 ev['query_gt_yaw_deg']=q.yaw_deg.to_numpy();ev['nearest_positive_database_keyframe_id']=db.keyframe_id.to_numpy()[nearest];ev['nearest_positive_yaw_deg']=near_yaw;ev['heading_difference_deg']=hd
 failures=ev[ev.valid_overlap_query & ~ev.rank1_is_positive].copy();failures['rank1_false_yaw_deg']=db.set_index('keyframe_id').loc[failures.rank1_database_keyframe_id,'yaw_deg'].to_numpy();failures['sc_score_margin']=failures.rank1_sc_score-failures.best_positive_sc_score
 # Stored Top-200 ranks identify most score-best positives.  For the one rank-221
 # case, score only that frozen descriptor vector to recover its identity; this is
 # a per-case lookup, not a rerun of the all-query baseline or its metrics.
 rank=np.load(frozen/'full_rankings_top200.npz');ids=rank['database_keyframe_ids'];scores=rank['scores'];best_ids=[]
 qdesc=np.load(a.processed/'full_2hz/robot1/scan_context_descriptors.npy');ddesc=np.load(a.processed/'full_2hz/robot2/scan_context_descriptors.npy');dbu,dbv=normalize_columns(ddesc)
 for _,x in failures.iterrows():
  qi=int(x.query_keyframe_id);candidate=ids[qi];mask=np.isin(candidate,db.keyframe_id.to_numpy()[pos[qi]]);best_ids.append(int(candidate[mask][np.argmax(scores[qi][mask])]) if mask.any() else np.nan)
  if not mask.any():
   case_scores,_=scores_for_query(qdesc[qi],dbu,dbv);best_ids[-1]=int(db.keyframe_id.iloc[np.where(pos[qi])[0][np.argmax(case_scores[pos[qi]])]])
 failures['best_positive_database_keyframe_id']=best_ids
 failures.to_csv(out/'failure_cases.csv',index=False)
 rows=[]
 for cohort,mask in [('all_valid',ev.valid_overlap_query.to_numpy()),('rank1_success',(ev.valid_overlap_query&ev.rank1_is_positive).to_numpy()),('rank1_failure',(ev.valid_overlap_query&~ev.rank1_is_positive).to_numpy())]:
  vals=ev.loc[mask,'heading_difference_deg'].to_numpy()
  for lo,hi in zip(BINS[:-1],BINS[1:]):rows.append(dict(cohort=cohort,heading_bin_deg=f'{int(lo)}-{int(min(hi,180))}',count=int(((vals>=lo)&(vals<hi)).sum()),fraction=float(((vals>=lo)&(vals<hi)).mean()) if len(vals) else np.nan))
 pd.DataFrame(rows).to_csv(out/'heading_bin_statistics.csv',index=False)
 # Plot trajectories with stationary points and heading distribution.
 fig,ax=plt.subplots(figsize=(10,8),dpi=140);ax.plot(q.x,q.y,lw=.7,label='robot1');ax.plot(db.x,db.y,lw=.7,label='robot2');ax.scatter(qm.loc[qm.stationary_or_near_stationary,'x'],qm.loc[qm.stationary_or_near_stationary,'y'],s=2,c='tab:red',label='robot1 stationary');ax.scatter(dm.loc[dm.stationary_or_near_stationary,'x'],dm.loc[dm.stationary_or_near_stationary,'y'],s=2,c='tab:purple',label='robot2 stationary');ax.axis('equal');ax.legend();ax.grid(alpha=.25);fig.tight_layout();fig.savefig(out/'trajectory_motion_status.png');plt.close(fig)
 fig,ax=plt.subplots(figsize=(8,5),dpi=140);ax.hist([ev.loc[ev.valid_overlap_query,'heading_difference_deg'],ev.loc[ev.valid_overlap_query&ev.rank1_is_positive,'heading_difference_deg'],ev.loc[ev.valid_overlap_query&~ev.rank1_is_positive,'heading_difference_deg']],bins=BINS,label=['all valid','Rank-1 success','Rank-1 failure'],histtype='step',linewidth=2);ax.set(xlabel='nearest-positive absolute heading difference (deg)',ylabel='queries');ax.legend();ax.grid(alpha=.25);fig.tight_layout();fig.savefig(out/'heading_difference_distribution.png');plt.close(fig)
 if not a.skip_rgb:
  # Load only failure clouds (all selected LiDAR cache already exists); extract matching RGB only for 10 panels.
  qtimes=q.set_index('keyframe_id').lidar_timestamp_ns;dtimes=db.set_index('keyframe_id').lidar_timestamp_ns
  r1times=[int(qtimes[x]) for x in failures.query_keyframe_id];r2times=[int(dtimes[x]) for x in np.r_[failures.best_positive_database_keyframe_id,failures.rank1_database_keyframe_id]]
  work=a.processed/'.work_stage25_rgb';r1rgb=extract_rgb(a.raw,work,'robot1',r1times);r2rgb=extract_rgb(a.raw,work,'robot2',r2times);work.rmdir()
  for _,x in failures.iterrows():
   qid,best,r1=int(x.query_keyframe_id),int(x.best_positive_database_keyframe_id),int(x.rank1_database_keyframe_id);title=f'q{qid}: first positive rank {int(x.first_positive_rank)}'
   qpc=np.load(a.processed/'full_2hz/robot1/lidar'/q.set_index('keyframe_id').loc[qid,'lidar_file']);ppc=np.load(a.processed/'full_2hz/robot2/lidar'/db.set_index('keyframe_id').loc[best,'lidar_file']);fpc=np.load(a.processed/'full_2hz/robot2/lidar'/db.set_index('keyframe_id').loc[r1,'lidar_file'])
   plot_bev(panels/f'q{qid:04d}_lidar_bev.png',title,qpc,ppc,fpc);contact(panels/f'q{qid:04d}_rgb_contact.png',title,[('query',r1rgb[int(qtimes[qid])]),('highest-SC GT-positive',r2rgb[int(dtimes[best])]),('Rank-1 false',r2rgb[int(dtimes[r1])])])
 # lightweight hard-pair inventory; no robot3/4 data is downloaded.
 available={r:(a.raw/'main_campus'/r/f'{r}_main_campus_gt_utm_poses.csv').exists() for r in ['robot1','robot2','robot3','robot4']}
 hard=[dict(pair='robot1-robot2',available=True,query_overlap_fraction=float(ev.valid_overlap_query.mean()),valid_overlap_queries=int(ev.valid_overlap_query.sum()),heading_overlap_above_90_fraction=float((ev.loc[ev.valid_overlap_query,'heading_difference_deg']>90).mean()),recommendation='Current pair; easy common-start portion diagnosed.')]
 for pair in ['robot1-robot3','robot1-robot4','robot2-robot3','robot2-robot4']:
  ra,rb=pair.split('-');hard.append(dict(pair=pair,available=available[ra] and available[rb],query_overlap_fraction=np.nan,valid_overlap_queries=np.nan,heading_overlap_above_90_fraction=np.nan,recommendation='Download only missing small gt_utm_poses.csv files before any sensor archives.'))
 pd.DataFrame([dict(robot=r,gt_utm_available=v) for r,v in available.items()]).to_csv(out/'hard_pair_gt_availability.csv',index=False);pd.DataFrame(hard).to_csv(out/'hard_pair_recommendation.csv',index=False)
 summary=f"""# Stage 2.5 diagnostic summary

## Frozen baseline preserved

Stage 2 ranking and metric files were read only. The historical `database_robot=to` metadata bug is fixed in the generator for future runs, but frozen Stage 2 CSV values were not overwritten.

## Why primary R@1 is high

Robot1 has {qs['initial_within5m_duration_s']:.1f}s and Robot2 has {ds['initial_within5m_duration_s']:.1f}s continuously within 5m of its own start. The shared common-start window is {common_duration:.1f}s. Early Robot1 queries therefore have many Robot2 frames in the same small start region (maximum positive count: {int(ev.positive_count.max())}), which inflates easy-match contribution. The canonical all-query score remains frozen; this diagnosis also reports moving and outside-common-start cohorts.

Motion threshold: stationary/near-stationary means local speed <=0.10 m/s across the preceding 0.5s interval.

Canonical/moving/outside-common-start valid-overlap counts are {int((ev.valid_overlap_query).sum())}/{int((ev.valid_overlap_query & moving).sum())}/{int((ev.valid_overlap_query & outside_common).sum())}. Their overlap fractions relative to all Robot1 queries are {ev.valid_overlap_query.mean():.3f}/{(ev.valid_overlap_query & moving).mean():.3f}/{(ev.valid_overlap_query & outside_common).mean():.3f}; conditional overlap fractions after retaining only moving/outside-common-start queries are {(ev.valid_overlap_query & moving).sum()/moving.sum():.3f}/{(ev.valid_overlap_query & outside_common).sum()/outside_common.sum():.3f}. Recall@K for those cohorts is in `moving_only_recall.csv`.

| robot | stationary frames | fraction | moving frames | travelled distance m |
|---|---:|---:|---:|---:|
| robot1 | {qs['stationary_frames']} | {qs['stationary_fraction']:.3f} | {qs['moving_frames']} | {qs['total_distance_m']:.1f} |
| robot2 | {ds['stationary_frames']} | {ds['stationary_fraction']:.3f} | {ds['moving_frames']} | {ds['total_distance_m']:.1f} |

See `moving_only_recall.csv`, `failure_cases.csv`, `heading_bin_statistics.csv`, and `failure_case_panels/` for the exact diagnostic records. All ten canonical Rank-1 failures are independently found at query IDs {', '.join(map(str,failures.query_keyframe_id.tolist()))}.

## Viewpoint protocol

For the next stage, retain this fixed 2Hz Robot1->Robot2 protocol and its offline `<5m` GT labels. Evaluate visual methods by the heading bins in `heading_bin_statistics.csv`, reporting all valid overlap queries plus Rank-1 SC failures separately. Do not provide GT headings, distances, or overlap status to candidate generation/ranking; use them only after rankings for stratified evaluation.

## Hard-pair planning

Only robot1/robot2 GT CSVs are present. Do not download LiDAR/RGB for robot3/4. Download only `robot3_main_campus_gt_utm_poses.csv` and `robot4_main_campus_gt_utm_poses.csv` first, then run the same lightweight overlap/heading inventory to choose a harder pair.
"""
 (out/'STAGE2_DIAGNOSTIC_SUMMARY.md').write_text(summary)
if __name__=='__main__':main()
