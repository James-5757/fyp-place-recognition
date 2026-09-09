from pathlib import Path
import pandas as pd
from PIL import Image, ImageDraw, ImageFont
ROOT=Path('/home/cas/fyp_place_recognition');O=ROOT/'outputs/canonical_v2/06_vlm';RGB=ROOT/'data/kitti/dataset/sequences/00/image_2';OFF=[-10,-5,0]
def imgs(f):
 out=[]
 for o in OFF:
  p=RGB/f'{f+o:06d}.png'
  if f+o>=0 and p.exists():out.append((o,p))
 return out
def make(q,a,b,bq,bc,path):
 rows=[('QUERY',q),('CANDIDATE A',a),('CANDIDATE B',b)]; W,H=1241,376; canvas=Image.new('RGB',(3*W+170,3*H+120),'white');d=ImageDraw.Draw(canvas);font=ImageFont.load_default()
 for ri,(lab,f) in enumerate(rows):
  d.text((10,70+ri*H+H//2),lab,fill='black',font=font)
  for ci,(off,p) in enumerate(imgs(f)):
   im=Image.open(p).convert('RGB');x=170+ci*W;y=100+ri*H;canvas.paste(im,(x,y));d.text((x+5,y-18),'t' if off==0 else f't{off}',fill='black',font=font)
   if (ri==0 and f+off==bq) or (ri>0 and f+off==bc):d.rectangle((x,y,x+W-1,y+H-1),outline='gold',width=5)
 canvas.save(path)
def main():
 x=pd.read_csv(ROOT/'outputs/canonical_v2/04_crossmax/canonical_top20_crossmax_scores.csv');x['fusion_score']=.6*x.scan_context_score+.4*x.temporal_cross_max;fs=[];pool=[];dis=[];comps=[];ctrl=[]
 for q,g in x.groupby('query_frame'):
  f=g.sort_values(['fusion_score','rank'],ascending=[False,True]).head(5).copy();f['fusion_rank']=range(1,6);fs.append(f);anchor=g.sort_values('rank').iloc[0];a=int(anchor.candidate_frame);ft=f.iloc[0];ct=f.sort_values(['temporal_cross_max','rank'],ascending=[False,True]).iloc[0];non=f[f.candidate_frame.astype(int)!=a];fc=non.sort_values('fusion_rank').iloc[0];cc=non.sort_values(['temporal_cross_max','rank'],ascending=[False,True]).iloc[0];ids={int(fc.candidate_frame),int(cc.candidate_frame)};contains=bool(f.is_positive.astype(int).any());pool.append({'query_frame':int(q),'pool_status':'POOL_CONTAINS_POSITIVE' if contains else 'CANONICAL_TOP20_POOL_MISS','filtered_top5_contains_positive':contains})
  disagree=a!=int(ft.candidate_frame) or a!=int(ct.candidate_frame) or len(ids)==2
  if disagree:
   dis.append({'query_frame':int(q),'sc_rank1_frame':a,'fusion_rank1_frame':int(ft.candidate_frame),'crossmax_rank1_frame':int(ct.candidate_frame),'best_fusion_challenger':int(fc.candidate_frame),'best_crossmax_challenger':int(cc.candidate_frame)})
   for c in ids: comps.append((int(q),'DISAGREEMENT',a,c))
  if int(anchor.is_positive)==1 and a==int(ft.candidate_frame):
   ordered=g.sort_values('rank');ctrl.append({'query_frame':int(q),'margin':float(ordered.iloc[0].scan_context_score-ordered.iloc[1].scan_context_score)})
 filtered=pd.concat(fs,ignore_index=True);controls=pd.DataFrame(ctrl);low=controls.sort_values(['margin','query_frame']).head(10).assign(case_type='LOW_MARGIN_CONTROL');high=controls.sort_values(['margin','query_frame'],ascending=[False,True]).head(10).assign(case_type='HIGH_MARGIN_CONTROL');controls=pd.concat([low,high],ignore_index=True)
 for r in controls.itertuples(index=False):
  f=filtered[filtered.query_frame==r.query_frame];a=int(f.sort_values('rank').iloc[0].candidate_frame);c=int(f[f.candidate_frame.astype(int)!=a].sort_values('fusion_rank').iloc[0].candidate_frame);comps.append((int(r.query_frame),r.case_type,a,c))
 comps=sorted(set(comps),key=lambda z:(z[1],z[0],z[2],z[3]));paneldir=O/'prepared_panels_selective';paneldir.mkdir(parents=True,exist_ok=True);man=[]
 for q,typ,a,c in comps:
  g=x[x.query_frame==q];crow=g[g.candidate_frame.astype(int)==c].iloc[0];arow=g[g.candidate_frame.astype(int)==a].iloc[0];pst=next(z for z in pool if z['query_frame']==q)
  for order,A,B in [('AB',a,c),('BA',c,a)]:
   path=paneldir/f'q{q:06d}_{typ.lower()}_a{a:06d}_c{c:06d}_{order}.png';make(q,A,B,int(crow.best_query_temporal_frame),int(crow.best_candidate_temporal_frame),path);Arow=g[g.candidate_frame.astype(int)==A].iloc[0];Brow=g[g.candidate_frame.astype(int)==B].iloc[0];man.append({'query_frame':q,'case_type':typ,'anchor_frame':a,'challenger_frame':c,'order':order,'candidate_A_frame':A,'candidate_B_frame':B,'collage_path':str(path),'anchor_is_positive':int(arow.is_positive),'challenger_is_positive':int(crow.is_positive),'candidate_A_is_positive':int(Arow.is_positive),'candidate_B_is_positive':int(Brow.is_positive),'pool_contains_positive':pst['filtered_top5_contains_positive']})
 filtered.to_csv(O/'canonical_filtered_top5.csv',index=False);pd.DataFrame(pool).to_csv(O/'pool_status.csv',index=False);pd.DataFrame(dis).to_csv(O/'disagreement_queries.csv',index=False);controls.to_csv(O/'control_queries.csv',index=False);pd.DataFrame(man).to_csv(O/'case_manifest.csv',index=False);pd.DataFrame(man).to_csv(O/'pair_manifest.csv',index=False);pd.DataFrame(pool).to_csv(O/'query_manifest.csv',index=False);(O/'PENDING_VLM_EXECUTION.md').write_text(f'# Canonical VLM Pending\n\nSelective manifest: {len(man)} AB/BA requests. Old responses are not reused.\n');print('disagreements',len(dis),'controls',len(controls),'comparisons',len(comps),'requests',len(man))
main()
