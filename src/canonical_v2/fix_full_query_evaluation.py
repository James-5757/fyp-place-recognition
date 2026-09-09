from pathlib import Path
import pandas as pd
O=Path('/home/cas/fyp_place_recognition/outputs/canonical_v2/06_vlm');R=Path('/home/cas/fyp_place_recognition/outputs/canonical_v2')
top=pd.read_csv(R/'04_crossmax/canonical_top20_crossmax_scores.csv');top['fusion_score']=.6*top.scan_context_score+.4*top.temporal_cross_max
oldq=pd.read_csv(O/'query_evaluation.csv');oldq=oldq.set_index('query_frame')
rows=[]
for q,g in top.groupby('query_frame'):
 q=int(q); valid=int(g.query_has_positive_in_B.iloc[0])==1;f=g.sort_values(['fusion_score','rank'],ascending=[False,True]).head(5);top20_contains=bool(g.is_positive.astype(int).any());filtered_contains=bool(f.is_positive.astype(int).any())
 if not valid:status='NO_OVERLAP_QUERY'
 elif not top20_contains:status='CANONICAL_TOP20_POOL_MISS'
 elif not filtered_contains:status='FILTERED_TOP5_MISS'
 else:status='POOL_CONTAINS_POSITIVE'
 base={'query_frame':q,'pool_status':status,'query_has_positive_in_B':valid,'canonical_top20_contains_positive':top20_contains,'filtered_top5_contains_positive':filtered_contains,'vlm_evaluated':q in oldq.index,'anchor_is_positive':pd.NA,'raw_consistent_winner_frames':'[]','raw_positive_challengers':'[]','actionable_winner_frames':'[]','actionable_positive_challengers':'[]','deployment_actionable_challenger':pd.NA,'actionable_winner_is_positive':pd.NA,'threshold_abstained_correct':False,'action':'NO_VLM_EVALUATION','system_correction':False,'system_regression':False}
 if q in oldq.index:
  for k,v in oldq.loc[q].to_dict().items():base[k]=v
 rows.append(base)
full=pd.DataFrame(rows);full.to_csv(O/'query_evaluation.csv',index=False)
counts=full.pool_status.value_counts().to_dict();evaluated=full[full.vlm_evaluated];c=int(evaluated.system_correction.sum());r=int(evaluated.system_regression.sum());system=149+c-r
lines=['# Canonical Selective VLM Evaluation','',f"No-overlap queries: {counts.get('NO_OVERLAP_QUERY',0)}",f"Canonical Top20 pool misses: {counts.get('CANONICAL_TOP20_POOL_MISS',0)}",f"Filtered Top5 misses: {counts.get('FILTERED_TOP5_MISS',0)}",f"VLM evaluated queries: {len(evaluated)}",f"Corrections: {c}",f"Regressions: {r}",f"Net gain: {c-r}",f"System R@1: {system}/158 = {system/158:.6f}",f"VLM classification: {'VLM_USEFUL_BUT_CONFIDENCE_OR_STABILITY_LIMITED' if c>r else 'VLM_VERIFICATION_INTRODUCES_TOO_MANY_REGRESSIONS'}"]
(O/'summary.txt').write_text('\n'.join(lines)+'\n');print('\n'.join(lines));print('status_counts',counts)
