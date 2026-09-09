import csv, json
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image
ROOT=Path('/home/cas/fyp_place_recognition');OUT=ROOT/'outputs/canonical_v2/06_vlm';RGB=ROOT/'data/kitti/dataset/sequences/00/image_2';OFF=[-10,-5,0]
def paths(f):return [(o,RGB/f'{f+o:06d}.png') for o in OFF if f+o>=0 and (RGB/f'{f+o:06d}.png').is_file()]
def panel(q,a,b,bq,bc,path):
 fig,ax=plt.subplots(3,3,figsize=(15,7),squeeze=False)
 for i,(label,f) in enumerate([('QUERY',q),('CANDIDATE A',a),('CANDIDATE B',b)]):
  for j,(o,p) in enumerate(paths(f)):
   with Image.open(p) as im:ax[i,j].imshow(im.convert('RGB'))
   ax[i,j].axis('off');
   if i==0:ax[i,j].set_title('t' if o==0 else f't{o}',fontsize=10)
   if j==0:ax[i,j].text(-.035,.5,label,transform=ax[i,j].transAxes,ha='right',va='center',fontsize=11)
   if (i==0 and f+o==bq) or (i>0 and f+o==bc):
    for s in ax[i,j].spines.values():s.set_visible(True);s.set_edgecolor('gold');s.set_linewidth(3)
 fig.suptitle('Canonical v2 Temporal RGB Place Verification',fontsize=14,fontweight='bold');fig.text(.5,.015,'Highlighted frame correspondence selected automatically; no scores or GT shown.',ha='center',fontsize=9);fig.tight_layout(rect=(.08,.04,1,.94));fig.savefig(path,dpi=150,bbox_inches='tight');plt.close(fig)
def main():
 scores=pd.read_csv(ROOT/'outputs/canonical_v2/04_crossmax/canonical_top20_crossmax_scores.csv');scores['fusion_score']=.6*scores.scan_context_score+.4*scores.temporal_cross_max
 filtered=[];allman=[];status=[]
 for q,g in scores.groupby('query_frame'):
  f=g.sort_values(['fusion_score','rank'],ascending=[False,True]).head(5).copy();f['fusion_rank']=range(1,6);filtered.append(f);contains=bool(f.is_positive.astype(int).any());status.append({'query_frame':int(q),'pool_status':'POOL_CONTAINS_POSITIVE' if contains else 'CANONICAL_TOP20_POOL_MISS','filtered_top5_contains_positive':contains})
  anchor=g.sort_values('rank').iloc[0];a=int(anchor.candidate_frame);ft=f.iloc[0];ct=f.sort_values(['temporal_cross_max','rank'],ascending=[False,True]).iloc[0];chs=set(f[f.candidate_frame.astype(int)!=a].candidate_frame.astype(int));fc=f[f.candidate_frame.astype(int)!=a].sort_values('fusion_rank').iloc[0];
  # Generate one challenger per policy: best fusion and best crossmax, deduplicated.
  cand_ids=set([int(fc.candidate_frame)])
  ctmp=f[f.candidate_frame.astype(int)!=a].sort_values(['temporal_cross_max','rank'],ascending=[False,True]);
  if len(ctmp):cand_ids.add(int(ctmp.iloc[0].candidate_frame))
  for c in cand_ids:
   row=f[f.candidate_frame.astype(int)==c].iloc[0];
   for order,A,B in [('AB',a,c),('BA',c,a)]:
    path=OUT/'prepared_panels'/f'q{q:06d}_a{a:06d}_c{c:06d}_{order}.png';path.parent.mkdir(parents=True,exist_ok=True);panel(int(q),A,B,int(row.best_query_temporal_frame),int(row.best_candidate_temporal_frame),path);ar=g[g.candidate_frame.astype(int)==A].iloc[0];br=g[g.candidate_frame.astype(int)==B].iloc[0];allman.append({'query_frame':int(q),'case_type':'DISAGREEMENT','anchor_frame':a,'challenger_frame':c,'order':order,'candidate_A_frame':A,'candidate_B_frame':B,'collage_path':str(path),'anchor_is_positive':int(anchor.is_positive),'challenger_is_positive':int(row.is_positive),'candidate_A_is_positive':int(ar.is_positive),'candidate_B_is_positive':int(br.is_positive),'pool_contains_positive':contains,'crossmax_score':float(row.temporal_cross_max)})
 pd.concat(filtered,ignore_index=True).to_csv(OUT/'canonical_filtered_top5.csv',index=False);pd.DataFrame(status).to_csv(OUT/'pool_status.csv',index=False);pd.DataFrame(allman).to_csv(OUT/'case_manifest.csv',index=False);pd.DataFrame(allman).to_csv(OUT/'pair_manifest.csv',index=False);pd.DataFrame(status).to_csv(OUT/'query_manifest.csv',index=False);print('canonical_vlm_pairs',len(allman),'queries',len(status),'pool_misses',sum(not x['filtered_top5_contains_positive'] for x in status))
main()
