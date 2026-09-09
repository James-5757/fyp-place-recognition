import numpy as np
import pandas as pd
from pathlib import Path
from scipy.spatial import cKDTree
from scan_context_canonical import load_kitti_bin, make_descriptor, aligned_similarity
ROOT=Path('/home/cas/fyp_place_recognition'); VELO=ROOT/'data/kitti/dataset/sequences/00/velodyne'; POSE=ROOT/'data/kitti/dataset/poses/00.txt'; OUT=ROOT/'outputs/canonical_v2/02_candidate_analysis'
pos=np.loadtxt(POSE).reshape(-1,3,4)[:,:,3]; sample=[int(p.stem) for p in sorted(VELO.glob('*.bin'))][::5]; A=[f for f in sample if f<=2400]; B=[f for f in sample if f>=2600]; tree=cKDTree(pos[B][:,[0,2]]); valid=[q for q in A if tree.query(pos[q][[0,2]],k=1)[0]<5]
def db(fs): return {f:make_descriptor(load_kitti_bin(VELO/f'{f:06d}.bin')) for f in fs}
da,dbb=db(A),db(B); rows=[]
for i,q in enumerate(valid,1):
 ranked=[]
 for c,d in dbb.items():
  s,sh=aligned_similarity(da[q],d); dist=float(np.hypot(pos[q,0]-pos[c,0],pos[q,2]-pos[c,2])); ranked.append((s,c,sh,dist,int(dist<5)))
 ranked.sort(key=lambda z:(-z[0],z[1])); first=next((x for x in ranked if x[4]),None); top=ranked[0]
 rows.append({'query_frame':q,'canonical_first_positive_rank':ranked.index(first)+1 if first else np.nan,'canonical_first_positive_frame':first[1] if first else np.nan,'canonical_first_positive_gt_distance':first[3] if first else np.nan,'canonical_first_positive_sc_score':first[0] if first else np.nan,'canonical_sc_rank1_frame':top[1],'canonical_sc_rank1_gt_distance':top[3],'canonical_sc_rank1_score':top[0],'canonical_sc_rank1_positive':top[4]})
 if i%25==0: print('ranked',i,'/',len(valid))
full=pd.DataFrame(rows);full.to_csv(OUT/'canonical_full_query_first_positive_rank.csv',index=False); ks=[1,5,10,20,30,50,100,200,389]; curve=[]
for k in ks:
 h=int((full.canonical_first_positive_rank<=k).sum());curve.append({'k':k,'hit_count':h,'miss_count':158-h,'recall':h/158})
pd.DataFrame(curve).to_csv(OUT/'canonical_extended_recall_at_k.csv',index=False); d=full[full.query_frame.isin([380,955])].copy();d['score_gap']=d.canonical_sc_rank1_score-d.canonical_first_positive_sc_score;d.to_csv(OUT/'canonical_380_955_full_ranking_diagnosis.csv',index=False);old=pd.read_csv(ROOT/'outputs/sc_candidate_recall_analysis/query_first_positive_rank.csv');m=old[['query_frame','first_positive_rank']].merge(full[['query_frame','canonical_first_positive_rank']],on='query_frame');m['rank_shift_old_minus_canonical']=m.first_positive_rank-m.canonical_first_positive_rank;m.to_csv(OUT/'old_vs_canonical_rank_shift.csv',index=False);print(pd.DataFrame(curve).to_string(index=False));print(d.to_string(index=False));print('rankshift mean',m.rank_shift_old_minus_canonical.mean(),'median',m.rank_shift_old_minus_canonical.median())
