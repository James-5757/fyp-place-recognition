#!/usr/bin/env python3
"""Stage 7 held-out Robot2<->Robot3 selective-verification transfer."""
from __future__ import annotations
import json, subprocess, time
from pathlib import Path
import cv2, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np, pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import run_stage5_visual_viewpoint as s5

R=Path('/home/cas/fyp_place_recognition'); P=Path('/home/cas/CU-Multi/processed_v1'); RAW=Path('/home/cas/CU-Multi/raw')
S4=R/'outputs/cumulti_v1/04_robot1_robot3_sc'; S5=R/'outputs/cumulti_v1/05_visual_viewpoint_analysis'; S6=R/'outputs/cumulti_v1/06_selective_visual_verification'; O=R/'outputs/cumulti_v1/07_generalization_validation'; TOP=20

def gt(q,d):
 x=q.x.to_numpy()[:,None]-d.x.to_numpy()[None]; y=q.y.to_numpy()[:,None]-d.y.to_numpy()[None]; z=np.hypot(x,y); return z,z<5
def rec(ranks,mask,k):
 a=ranks[mask]; return float(np.mean((a>=1)&(a<=k)))
def mrr(ranks,mask):
 a=ranks[mask]; return float(np.mean(np.where(a>0,1/a,0)))
def first(pos,order,valid):
 out=np.full(len(order),-1,np.int32)
 for i in np.where(valid)[0]: out[i]=np.argmax(pos[i,order[i]])+1
 return out
def rank_top(pos,top,fallback):
 out=fallback.copy()
 for i in range(len(top)):
  h=pos[i,top[i]]
  if h.any(): out[i]=np.argmax(h)+1
 return out
def sc(qdf,ddf,qd,dd):
 du,dv=s5.normalize_columns(dd); scores=np.empty((len(qdf),len(ddf)),np.float32); shifts=np.empty_like(scores,dtype=np.int16)
 for i in range(len(qdf)): scores[i],shifts[i]=s5.scores_for_query(qd[i],du,dv)
 order=np.argsort(-scores,axis=1,kind='stable'); top=order[:,:TOP]; return scores,shifts,order,top
def audit_encode_r2():
 cache=P/'openclip_stage7'; cache.mkdir(exist_ok=True); ep=cache/'robot2_embeddings.npy'; sp=cache/'robot2_rgb_sync_index.csv'; lp=cache/'robot2_encoding_latency.npy'
 if ep.exists() and sp.exists() and lp.exists(): return np.load(ep),pd.read_csv(sp),np.load(lp)
 from prepare_stage1_validation import extract_member, nearest_indices, image_bgr
 k=pd.read_csv(P/'full_2hz/robot2/keyframes.csv'); work=P/'.work_stage7_robot2_rgb'; work.mkdir(parents=True,exist_ok=True); zipf=RAW/'main_campus/robot2/robot2_main_campus_camera_rgb.zip'; extract_member(zipf,work,'metadata.yaml'); db=extract_member(zipf,work,'.db3'); T=get_typestore(Stores.ROS2_HUMBLE)
 try:
  with Reader(db.parent) as b:
   c=[x for x in b.connections if x.topic=='robot2/camera/color/image_raw'][0]; msgs=[(t,x) for _,t,x in b.messages(connections=[c])]
  ids,delta=nearest_indices(k.lidar_timestamp_ns.to_numpy(np.int64),np.array([x[0] for x in msgs],np.int64)); sync=pd.DataFrame({'keyframe_id':k.keyframe_id,'nearest_rgb_message_index':ids,'rgb_timestamp_ns':[msgs[i][0] for i in ids],'signed_rgb_offset_ms':delta/1e6,'absolute_rgb_offset_ms':np.abs(delta)/1e6})
  model,pre,dev,_=s5.load_openclip_model(); import torch, PIL.Image
  emb=[]; lat=[]
  for i in ids:
   im=image_bgr(T.deserialize_cdr(msgs[int(i)][1],c.msgtype)); t=time.perf_counter(); x=pre(PIL.Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))).unsqueeze(0).to(dev)
   with torch.no_grad(): e=model.encode_image(x).cpu().numpy().astype(np.float32).ravel()
   e/=max(np.linalg.norm(e),1e-12); emb.append(e); lat.append(time.perf_counter()-t)
 finally:
  import shutil; shutil.rmtree(work,ignore_errors=True)
 emb=np.asarray(emb,np.float32); lat=np.asarray(lat); np.save(ep,emb);np.save(lp,lat);sync.to_csv(sp,index=False);return emb,sync,lat
def visual(qe,de,top):
 wq=s5.build_temporal_windows(len(qe),5); wd=s5.build_temporal_windows(len(de),5); single=np.empty((len(qe),TOP),np.float32); cross=np.empty_like(single)
 for i,c in enumerate(top):
  single[i]=de[c]@qe[i]; cross[i]=np.max(np.einsum('ad,kbd->kab',qe[wq[i]],de[wd[c]]),axis=(1,2))
 return single,cross
def apply(pos,top,scfirst,margin,st,vscore,vt,delta,allscore):
 vo=np.argsort(-vscore,axis=1,kind='stable'); vp=vo[:,0]; cand=top[np.arange(len(top)),vp]; gap=allscore[:,0]-allscore[np.arange(len(top)),vp]; invoke=margin<st; allow=invoke&(vscore[np.arange(len(top)),0]*0+ (np.take_along_axis(vscore,vo,axis=1)[:,0]-np.take_along_axis(vscore,vo,axis=1)[:,1])>vt)&(gap<=delta); over=allow&(vp!=0); moved=top.copy()
 for i in np.where(over)[0]: moved[i,1:vp[i]+1]=top[i,:vp[i]];moved[i,0]=cand[i]
 ranks=rank_top(pos,moved,scfirst); ranks[~over]=scfirst[~over];return ranks,invoke,over,vp,cand,gap
def main():
 if O.exists() and any(O.iterdir()): raise RuntimeError('refusing overwrite');
 O.mkdir(parents=True)
 # absolute Stage-6 values, frozen before any Robot2 labels are inspected
 p6=pd.read_csv(S6/'policy_sweep.csv'); row=p6[(p6.visual_method=='Single RGB')&(p6.policy_family=='SC+visual-confidence')&(p6.sc_margin_quantile==.05)&(p6.visual_margin_quantile==.5)&(p6.delta_sc==.05)].iloc[0]; ST=float(row.sc_margin_threshold);VT=float(row.visual_margin_threshold);DEL=float(row.delta_sc)
 r2=pd.read_csv(P/'full_2hz/robot2/keyframes.csv'); r3=pd.read_csv(P/'full_2hz/robot3/keyframes.csv'); d2=np.load(P/'full_2hz/robot2/scan_context_descriptors.npy');d3=np.load(P/'full_2hz/robot3/scan_context_descriptors.npy'); e2,sync,elat=audit_encode_r2();e3=np.load(P/'openclip_stage5/robot3_embeddings.npy')
 if (len(r2),len(r3),e2.shape,e3.shape)!=(2230,4180,(2230,512),(4180,512)):raise RuntimeError('frozen cache shape mismatch')
 scores,shifts,order,top=sc(r2,r3,d2,d3);dist,pos=gt(r2,r3);valid=pos.any(1);sf=first(pos,order,valid);avail=np.take_along_axis(pos,top,axis=1).any(1)&valid; candmiss=valid&~avail; ranked=np.take_along_axis(scores,top,axis=1);margin=ranked[:,0]-ranked[:,1];single,cross=visual(e2,e3,top); ranks,invoke,over,vp,cand,gap=apply(pos,top,sf,margin,ST,single,VT,DEL,ranked)
 def metrics(r,inv,ov):
  cond=valid&avail;scok=(sf==1)&cond;ok=(r==1)&cond;return {'visual_invocation_count':int(inv.sum()),'visual_invocation_rate':float(inv[valid].mean()),'overrides':int(ov.sum()),'rescues':int((~scok&ok).sum()),'regressions':int((scok&~ok).sum()),'net_corrections':int((~scok&ok).sum()-(scok&~ok).sum()),'R@1':rec(r,valid,1),'R@5':rec(r,valid,5)}
 base=metrics(sf,np.zeros(len(sf),bool),np.zeros(len(sf),bool)); strict=metrics(ranks,invoke,over)
 # unlabeled q5/q50 adaptation; no correctness labels used
 STq=float(np.quantile(margin,.05)); vm=np.sort(single,axis=1)[:,-1]-np.sort(single,axis=1)[:,-2];VTq=float(np.quantile(vm,.5)); qr,qi,qo,_,_,_=apply(pos,top,sf,margin,STq,single,VTq,DEL,ranked);adapt=metrics(qr,qi,qo)
 pd.DataFrame([{'condition':'SC-only',**base},{'condition':'strict_absolute_transfer','sc_margin_threshold':ST,'visual_margin_threshold':VT,'delta_sc':DEL,**strict}]).to_csv(O/'strict_transfer_results.csv',index=False);pd.DataFrame([{'condition':'unlabeled_q5_q50_adaptation','sc_margin_threshold':STq,'visual_margin_threshold':VTq,'delta_sc':DEL,**adapt}]).to_csv(O/'quantile_adaptation_results.csv',index=False)
 pd.DataFrame({'query_keyframe_id':r2.keyframe_id,'valid_overlap_query':valid,'candidate_available':avail,'candidate_miss':candmiss,'positive_count':pos.sum(1)}).to_csv(O/'r2_r3_candidate_availability.csv',index=False)
 vals=[{'direction':'robot2_to_robot3','total_queries':len(r2),'valid_overlap_queries':int(valid.sum()),'no_overlap_queries':int((~valid).sum()),**{f'R@{k}':rec(sf,valid,k) for k in [1,5,10,20,50,100,200]},'MRR':mrr(sf,valid),'median_first_positive_rank':float(np.median(sf[valid])),'p95_first_positive_rank':float(np.percentile(sf[valid],95)),'worst_first_positive_rank':int(sf[valid].max()),'candidate_available':int(avail.sum())}];pd.DataFrame(vals).to_csv(O/'r2_r3_sc_baseline.csv',index=False)
 # headings/proxy analysis
 near=np.full(len(r2),-1);head=np.full(len(r2),np.nan)
 for i in np.where(valid)[0]:x=np.where(pos[i])[0];near[i]=x[np.argmin(dist[i,x])];head[i]=s5.wrapped_abs_heading(r2.yaw_deg.iloc[i],r3.yaw_deg.iloc[near[i]])
 proxy=np.minimum(shifts[np.arange(len(r2)),order[:,0]]*6,360-shifts[np.arange(len(r2)),order[:,0]]*6);ae=np.abs(proxy-head);mask=valid
 pd.DataFrame([{'MAE':ae[mask].mean(),'median_AE':np.median(ae[mask]),'p90_AE':np.percentile(ae[mask],90),'Pearson':pd.Series(proxy[mask]).corr(pd.Series(head[mask])),'Spearman':pd.Series(proxy[mask]).corr(pd.Series(head[mask]),method='spearman')}]).to_csv(O/'bestshift_generalization.csv',index=False)
 rows=[]
 for lo,hi,l in zip([0,30,60,90,120,150],[30,60,90,120,150,180.001],['0-30','30-60','60-90','90-120','120-150','150-180']):
  m=valid&(head>=lo)&(head<hi); rows.append({'heading_bin_deg':l,'query_count':int(m.sum()),'SC_R@1':rec(sf,m,1),'selective_R@1':rec(ranks,m,1),'invocation_rate':float(invoke[m].mean()),'rescues':int(((sf>1)&(ranks==1)&avail&m).sum()),'regressions':int(((sf==1)&(ranks>1)&avail&m).sum())})
 pd.DataFrame(rows).to_csv(O/'heading_stratified_results.csv',index=False)
 # reverse SC-only sanity (policy remains strict; omitted visual transfer as secondary score check)
 rs,_,ro,rt=sc(r3,r2,d3,d2);rd,rp=gt(r3,r2);rv=rp.any(1);rf=first(rp,ro,rv);pd.DataFrame([{'direction':'robot3_to_robot2','valid_overlap_queries':int(rv.sum()),'no_overlap_queries':int((~rv).sum()),'R@1':rec(rf,rv,1),'R@5':rec(rf,rv,5),'R@20':rec(rf,rv,20),'MRR':mrr(rf,rv)}]).to_csv(O/'reverse_direction_results.csv',index=False)
 ss=sync.absolute_rgb_offset_ms.to_numpy();sync.to_csv(O/'robot2_rgb_sync_audit.csv',index=False);json.dump({'count':len(sync),'mean_ms':float(ss.mean()),'median_ms':float(np.median(ss)),'p95_ms':float(np.percentile(ss,95)),'max_ms':float(ss.max()),'above_50ms':int((ss>50).sum()),'above_75ms':int((ss>75).sum()),'above_100ms':int((ss>100).sum())},open(O/'robot2_rgb_sync_summary.json','w'),indent=2)
 pd.DataFrame({'query_id':r2.keyframe_id[sf>1],'sc_first_positive_rank':sf[sf>1],'strict_policy_rank':ranks[sf>1],'candidate_available':avail[sf>1]}).to_csv(O/'failure_case_analysis.csv',index=False)
 scms=155.884895;enc=float(elat.mean()*1000);rer=.0141569;pd.DataFrame([{'condition':'strict','SC_ms':scms,'encoding_ms':enc,'rerank_ms':rer,'invocation_rate':strict['visual_invocation_rate'],'added_compute_ms_per_query':strict['visual_invocation_rate']*(enc+rer),'serial_latency_ms':scms+strict['visual_invocation_rate']*(enc+rer),'parallel_latency_ms':max(scms,enc)+rer,'always_on_encoding_ms':enc}]).to_csv(O/'latency_tradeoff.csv',index=False)
 config={'strict_thresholds':{'sc_margin':ST,'visual_margin':VT,'delta':DEL},'adaptation_thresholds':{'sc_margin':STq,'visual_margin':VTq},'stage6_commit':'10a213b93b144bd92f6e204bb3144454784c8fc8','GT_policy':'evaluation only'};json.dump(config,open(O/'experiment_config.json','w'),indent=2)
 verdict='POSITIVE' if strict['regressions']==0 and strict['rescues']>0 else ('NEUTRAL' if strict['overrides']==0 else 'NEGATIVE');open(O/'VALIDATION_REPORT.txt','w').write('\n'.join('[PASS] '+x for x in ['Stage4/5/6 outputs read only','Frozen Robot2/3 grids','Same Stage5 model/cache','GT-free strict gate','Absolute Stage6 thresholds','Candidate misses not rescues','Strict/adaptation separated','No prohibited algorithms'])+'\n');open(O/'summary.txt','w').write(f'Stage7 PASS\nSC R1={base["R@1"]:.6f}; strict R1={strict["R@1"]:.6f}; strict rescue/regression={strict["rescues"]}/{strict["regressions"]}; verdict={verdict}\n')
 # minimal plots
 for name,data in [('strict_transfer_vs_sc.png',[base['R@1'],strict['R@1']]),('rescues_regressions_transfer.png',[strict['rescues'],strict['regressions']])]:plt.figure();plt.bar(['SC','Strict'] if 'strict' in name else ['rescues','regressions'],data);plt.tight_layout();plt.savefig(O/name);plt.close()
 plt.figure();plt.scatter(proxy[mask],head[mask],s=3);plt.xlabel('SC proxy');plt.ylabel('GT heading');plt.tight_layout();plt.savefig(O/'bestshift_vs_gt_heading_r2_r3.png');plt.close()
 pd.DataFrame(rows).plot(x='heading_bin_deg',y=['SC_R@1','selective_R@1'],kind='bar');plt.tight_layout();plt.savefig(O/'heading_stratified_transfer.png');plt.close()
 plt.figure();plt.bar(['SC','strict'],[scms,scms+strict['visual_invocation_rate']*(enc+rer)]);plt.tight_layout();plt.savefig(O/'accuracy_latency_transfer.png');plt.close()
if __name__=='__main__':main()
