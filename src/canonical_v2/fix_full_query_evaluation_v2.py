from pathlib import Path
import pandas as pd
O=Path('/home/cas/fyp_place_recognition/outputs/canonical_v2/06_vlm');R=Path('/home/cas/fyp_place_recognition/outputs/canonical_v2')
top=pd.read_csv(R/'04_crossmax/canonical_top20_crossmax_scores.csv');top['fusion_score']=.6*top.scan_context_score+.4*top.temporal_cross_max
old=pd.read_csv(O/'query_evaluation.csv').set_index('query_frame');rows=[]
for q,g in top.groupby('query_frame'):
 q=int(q);valid=int(g.query_has_positive_in_B.iloc[0])==1;top20=bool(g.is_positive.astype(int).any());f=bool(g.sort_values(['fusion_score','rank'],ascending=[False,True]).head(5).is_positive.astype(int).any());status='NO_OVERLAP_QUERY' if not valid else 'CANONICAL_TOP20_POOL_MISS' if not top20 else 'FILTERED_TOP5_MISS' if not f else 'POOL_CONTAINS_POSITIVE'
 row={'query_frame':q,'pool_status':status,'query_has_positive_in_B':valid,'canonical_top20_contains_positive':top20,'filtered_top5_contains_positive':f,'vlm_evaluated':q in old.index}
 if q in old.index:
  for k,v in old.loc[q].to_dict().items():
   if k not in {'pool_status','query_has_positive_in_B','canonical_top20_contains_positive','filtered_top5_contains_positive'}:row[k]=v
 rows.append(row)
full=pd.DataFrame(rows);full.to_csv(O/'query_evaluation.csv',index=False);counts=full.pool_status.value_counts().to_dict();ev=full[full.vlm_evaluated];c=int(ev.system_correction.sum());r=int(ev.system_regression.sum());system=149+c-r
lines=['# Canonical Selective VLM Evaluation','',f"No-overlap queries: {counts.get('NO_OVERLAP_QUERY',0)}",f"Canonical Top20 pool misses: {counts.get('CANONICAL_TOP20_POOL_MISS',0)}",f"Filtered Top5 misses excluding Top20 misses: {counts.get('FILTERED_TOP5_MISS',0)}",f"VLM evaluated queries: {len(ev)}",f"Corrections: {c}",f"Regressions: {r}",f"Net gain: {c-r}",f"System R@1: {system}/158 = {system/158:.6f}",f"VLM classification: {'VLM_USEFUL_BUT_CONFIDENCE_OR_STABILITY_LIMITED' if c>r else 'VLM_VERIFICATION_INTRODUCES_TOO_MANY_REGRESSIONS'}"]
(O/'summary.txt').write_text('\n'.join(lines)+'\n');print('\n'.join(lines));print(counts)
