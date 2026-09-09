import argparse, json
from pathlib import Path
import numpy as np, pandas as pd, torch
import torch.nn.functional as F
from PIL import Image
import open_clip
ROOT=Path.home()/"fyp_place_recognition"; OUT=ROOT/"outputs/canonical_v2"; RGB=ROOT/"data/kitti/dataset/sequences/00/image_2"; CKPT=ROOT/"models/openclip/vit_b32_laion400m_e32.pt"; OFF3=[-10,-5,0]; OFF5=[-20,-15,-10,-5,0]
def sig(p):s=p.stat();return f"{s.st_size}:{s.st_mtime_ns}"
def ev(df,col,name):
 R=[]
 for q,g in df[df.query_has_positive_in_B==1].groupby('query_frame'):
  b=g.sort_values('rank').iloc[0];x=g.sort_values([col,'rank'],ascending=[False,True]).reset_index(drop=True);pos=x.index[x.is_positive==1].tolist();R.append((int(b.is_positive),int(x.iloc[0].is_positive),1/(pos[0]+1) if pos else 0))
 a=pd.DataFrame(R,columns=['b','h','rr']);return {'method':name,'r_at_1':a.h.mean(),'mrr':a.rr.mean(),'top1_hits':int(a.h.sum()),'corrections':int(((a.b==0)&(a.h==1)).sum()),'regressions':int(((a.b==1)&(a.h==0)).sum()),'net_gain':int(((a.b==0)&(a.h==1)).sum()-((a.b==1)&(a.h==0)).sum())}
def main():
 pool=pd.read_csv(OUT/'01_scan_context/canonical_sc_top20_all_queries.csv');frames=sorted(set(pool.query_frame.astype(int))|set(pool.candidate_frame.astype(int)));need=sorted({f+o for f in frames for o in OFF5 if f+o>=0 and (RGB/f'{f+o:06d}.png').exists()});cache_path=OUT/'03_visual_baselines/cache/rgb_frame_embeddings.pt';cache_path.parent.mkdir(parents=True,exist_ok=True);cache={'entries':{}}
 old=ROOT/'outputs/temporal_rgb_reranking/cache/rgb_frame_embeddings.pt'
 if old.exists():cache=torch.load(old,map_location='cpu',weights_only=True); imported=len(cache['entries'])
 else:imported=0
 emb={};missing=[]
 for f in need:
  p=RGB/f'{f:06d}.png';e=cache['entries'].get(str(f))
  if e and e.get('signature')==sig(p):emb[f]=e['embedding'].float()
  else:missing.append((f,p))
 if missing:
  model,_,pre=open_clip.create_model_and_transforms('ViT-B-32-quickgelu',pretrained=str(CKPT));model.eval()
  for i in range(0,len(missing),16):
   batch=missing[i:i+16];t=[]
   for _,p in batch:
    with Image.open(p) as im:t.append(pre(im.convert('RGB')))
   with torch.inference_mode():v=F.normalize(model.encode_image(torch.stack(t)),p=2,dim=-1).cpu()
   for (f,p),e in zip(batch,v):cache['entries'][str(f)]={'signature':sig(p),'embedding':e};emb[f]=e
  torch.save(cache,cache_path)
 def win(f,off):return [f+o for o in off if f+o>=0 and f+o in emb]
 w3={f:win(f,OFF3) for f in frames};w5={f:win(f,OFF5) for f in frames};d3={f:F.normalize(torch.stack([emb[z] for z in w3[f]]).mean(0),p=2,dim=0) for f in frames};d5={f:F.normalize(torch.stack([emb[z] for z in w5[f]]).mean(0),p=2,dim=0) for f in frames};rows=[]
 for r in pool.itertuples(index=False):
  q,c=int(r.query_frame),int(r.candidate_frame);m=torch.stack([emb[z] for z in w5[q]]) @ torch.stack([emb[z] for z in w5[c]]).T;flat=m.flatten();idx=int(flat.argmax());qi,ci=divmod(idx,m.shape[1]);x=r._asdict();x.update({'single_rgb_similarity':float(emb[q]@emb[c]),'temporal_3_rgb_similarity':float(d3[q]@d3[c]),'temporal_5_rgb_similarity':float(d5[q]@d5[c]),'temporal_cross_max':float(flat.max()),'temporal_cross_top3':float(torch.topk(flat,k=min(3,len(flat))).values.mean()),'temporal_cross_symmetric':.5*(float(m.max(1).values.mean())+float(m.max(0).values.mean())),'best_query_temporal_frame':w5[q][qi],'best_candidate_temporal_frame':w5[c][ci]});rows.append(x)
 df=pd.DataFrame(rows);o=OUT/'03_visual_baselines';o.mkdir(exist_ok=True);df.to_csv(o/'canonical_visual_candidate_scores.csv',index=False);metrics=pd.DataFrame([ev(df,c,n) for n,c in [('canonical_sc','scan_context_score'),('single_rgb','single_rgb_similarity'),('temporal_rgb3','temporal_3_rgb_similarity'),('temporal_rgb5','temporal_5_rgb_similarity'),('cross_max','temporal_cross_max'),('cross_top3','temporal_cross_top3'),('cross_symmetric','temporal_cross_symmetric')]]);metrics.to_csv(o/'visual_baseline_metrics.csv',index=False);print(metrics.to_string(index=False))
if __name__=='__main__':main()
