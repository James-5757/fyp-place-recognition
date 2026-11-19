#!/usr/bin/env python3
"""Generate Stage5B diagnostics and documentation from validated machine outputs."""
import argparse,json
from pathlib import Path
import numpy as np
import pandas as pd


def markdown(df,columns):
    rows=['| '+' | '.join(columns)+' |','|'+'|'.join(['---']*len(columns))+'|']
    for _,r in df.iterrows():
        rows.append('| '+' | '.join(f'{r[c]:.6f}' if isinstance(r[c],(float,np.floating)) else str(r[c]) for c in columns)+' |')
    return '\n'.join(rows)


def main():
    a=argparse.ArgumentParser();a.add_argument('--root',type=Path,required=True);a.add_argument('--out',type=Path,required=True);a.add_argument('--work',type=Path,required=True)
    z=a.parse_args();o=z.out;w=z.work;root=z.root
    assert json.loads((o/'independent_validation.json').read_text())['status']=='PASS'
    old=pd.read_csv(root/'outputs/cumulti_v1/04_robot1_robot3_sc/query_evaluation.csv');valid=old.valid_overlap_query.to_numpy();same=valid&(old.heading_difference_deg.to_numpy()<=10)
    pairs=pd.read_csv(root/'outputs/cumulti_v1/04_robot1_robot3_sc/gt_positive_pairs.csv');pos=np.zeros((2000,4180),bool);pos[pairs.query_keyframe_id,pairs.database_keyframe_id]=True
    sc=np.load(w/'sc_order.npy',mmap_mode='r');rows=[]
    for name,key in [('Single RGB','single'),('RGB5 Mean','mean5'),('Cross-Max','crossmax')]:
        s=np.load(w/f'{key}_scores.npy',mmap_mode='r');order=np.load(w/f'{key}_order.npy',mmap_mode='r')
        top=s[np.arange(2000),order[:,0]];ties=(s==top[:,None]).sum(1)
        # Same full visual scores, same SC pool; vary only exact-tie ordering.
        pool=np.sort(sc[:,:20],axis=1)
        selected=np.take_along_axis(pool,np.argsort(-np.take_along_axis(s,pool,axis=1),axis=1,kind='stable'),axis=1)
        pp=np.take_along_axis(pos,selected,axis=1);rr=np.where(pp.any(1),pp.argmax(1)+1,-1)
        for subset,mask in [('overall',valid),('same_view',same)]:
            r=rr[mask];rows.append(dict(method=name,subset=subset,query_count=int(mask.sum()),rank1_tied_queries=int((ties[mask]>1).sum()),median_rank1_tie_count=float(np.median(ties[mask])),max_rank1_tie_count=int(ties[mask].max()),pool_R1_with_ID_ties=float(((r>=1)&(r<=1)).mean()),pool_R5_with_ID_ties=float(((r>=1)&(r<=5)).mean())))
    pd.DataFrame(rows).to_csv(o/'tie_policy_diagnostic.csv',index=False,float_format='%.12g')
    allm=pd.read_csv(o/'metrics_overall.csv');same_m=pd.read_csv(o/'metrics_same_view.csv');pool=pd.read_csv(o/'historical_pool_comparison.csv')
    prov=json.loads((o/'run_provenance.json').read_text());cache=json.loads((o/'embedding_cache_audit.json').read_text())
    cfg=json.loads((o/'run_config.json').read_text());nextra=len(pd.read_csv(o/'rank1_positive_outside_sc_top20.csv'))
    merged=[]
    for subset,t in [('overall',allm),('same_view',same_m)]:
        for _,r in t.iterrows():merged.append(dict(subset=subset,method=r.method,scope='full_database',**{k:r[k] for k in ['query_count','R@1','R@5','R@10','R@20','MRR']}))
    for _,r in pool.iterrows():merged.append(dict(subset=r.subset,method=r.method,scope=r.scope,**{k:r[k] for k in ['query_count','R@1','R@5','R@10','R@20','MRR']}))
    pd.DataFrame(merged).to_csv(o/'comparison_all_scopes.csv',index=False,float_format='%.12g')
    doc=['# CU-Multi Stage5B — independent visual full-database retrieval','',
         'PASS. Historical audit found no validated independent all4180 R1-to-R3 visual benchmark in the accessible evidence. Stage5/7.5 and KITTI scripts used inherited SC candidate pools. Source/artifact evidence and availability limitations are recorded in HISTORICAL_AUDIT.md and historical_audit.csv. Existing RGB-D appearance-registration studies use a different dataset index/protocol and are not this benchmark.','',
         '## Protocol','',
         'Frozen Robot1 queries:2000; Robot3 database:4180; timestamp-sampled2Hz. All2000 queries are ranked against all4180 database keyframes before GT evaluation. Metrics use the exact1833 valid-overlap queries and the exact482 Stage7.5 same-view query IDs. The167 no-overlap queries are scored but excluded from recall/MRR denominators. Positive labels are strictly horizontal distance<5.0m and match every frozen Stage4 positive-pair CSV entry. Same-view means nearest-GT-positive wrapped heading<=10degrees; it selects the offline query stratum and never restricts database candidates.','',
         'The complete Stage5 normalized float32 embedding cache was reused: ViT-B-32-quickgelu / laion400m_e32 /512 dimensions; no training, image decoding or encoding. Both cache sidecars match every keyframe ID and synchronized RGB timestamp; no zero/nonfinite rows exist. Original checkpoint hash receipts are unavailable, so model provenance rests on recorded cache metadata, source configuration, complete identity checks and exact reproduction of historical pooled metrics. Current input hashes authenticate the reused cache for this experiment; they are not retroactive original encoding receipts.','',
         'Single RGB uses current-frame cosine. RGB5 Mean averages each causal five-keyframe window then normalizes before cosine. Cross-Max takes the maximum of all25 pairwise cosines between the causal windows. Left padding is exactly max(0,k-4+j). Efficient Cross-Max is a separable max over a shared2000x4180 cosine matrix; it does not allocate a query/database/window/feature tensor. No SC score enters visual scores or full visual ordering. All full methods use descending score and increasing database ID for exact ties.','',
         'Canonical SC scoring is imported unchanged from frozen Stage5 code because full score matrices were no longer archived. Reconstructed SC first-positive ranks and all2000 Rank1 IDs match Stage4 exactly. The canonical SC numerical metrics are unchanged.','',
         '## Overall:1833 valid queries','',markdown(allm,['method','R@1','R@5','R@10','R@20','MRR']),'',
         '## Same-view:exact482 queries','',markdown(same_m,['method','R@1','R@5','R@10','R@20','MRR']),'',
         '## Historical SC Top20 comparison','',
         'These are end-to-end denominators, including the three valid queries without a positive in SC Top20. They must not be mixed with the1830-query candidate-conditioned Stage5 cohort. Overall historical R@1 and all saved Stage7.5 pooled R@1/R@5/MRR were reproduced exactly from the cache and original scalar scorer. The historical Stage5 raw CSV directory is not retained in the current checkout/server snapshot; its corrected committed documentation is historical evidence. R@10/R@20 in this new comparison are newly recovered from the frozen protocol, not claimed as old saved metrics.','',
         markdown(pool,['subset','method','R@1','R@5','R@10','R@20','MRR']),'',
         'SC Top20 already supplies a geometry-filtered high-recall pool. A method described as RGB-only there still benefits from SC candidate generation. Independent retrieval faces all4180 candidates and must not inherit the pool-conditioned success rates.','',
         'Exact-tie caveat: historical Stage5 preserves SC candidate order when visual scores tie, whereas the full benchmark uses database-ID ties. This is particularly relevant to overlapping Cross-Max windows. tie_policy_diagnostic.csv includes same-score/SC-pool results with ID ties to separate tie policy from pool expansion; it does not tune or replace the declared result.','',
         markdown(pd.DataFrame(rows),['subset','method','rank1_tied_queries','median_rank1_tie_count','pool_R1_with_ID_ties','pool_R5_with_ID_ties']),'',
         f'There are {nextra} method/query Rank1-positive results whose winning database ID is outside the SC Top20. This local complementarity does not by itself establish an aggregate improvement or a validated fusion method.','',
         '## Validation and preservation','',
         'An independent validator checks every complete2000x4180 ordering as a permutation and deterministic sort, recomputes visual scores using a different temporal-max implementation, recomputes metrics from inverse permutations and frozen positive-pair labels, and checks all146640 Top20 rows. It independently rechecks all482 IDs and frozen-file hashes. No frozen Stage4/5/6/7/7.5 output was changed. Full score/rank arrays remain server-only with hashes and sizes in full_rankings_artifact_manifest.json.','',
         'All seven deterministic numeric/ID CSVs and all eight complete score/order arrays have identical hashes across two executions. The first attempt is retained server-side; the final source reads only keyframe IDs during pre-ranking cache checks.','',
         'Cross-Max has exactly five tied top-scoring candidates for every valid query in this run. Neighboring causal windows share the winning image pair; this is a score-definition ambiguity. The reported deterministic Rank1 is not a unique visual localization. R@5 and the tie-policy diagnostic provide additional context.','',
         '## Thesis interpretation and limits','',
         'On this frozen CU-Multi R1-to-R3 protocol, full-database Scan Context exceeds these three independent generic OpenCLIP retrieval methods, including the same-view control. The earlier visual results quantify reranking inside an SC pool and cannot support an independent visual retrieval claim. Do not claim that RGB improves place recognition from this experiment. These results do not establish universal LiDAR superiority over all dedicated visual place-recognition models or datasets.','',
         'The controlled subset is a query-heading stratum, not a heading-filtered database. Front RGB and360-degree LiDAR observe different context. This is offline static-database retrieval, not causal dual-stream discovery, mapping or online latency measurement. Cross-Max shared-window exact ties and static/common-start content remain relevant limitations. No threshold tuning, fusion, training or geometric verification is part of this stage.','',
         '## Reproduce','',
         'With the recorded processed caches and frozen project artifacts, run from repository root:',
         '```bash',
         'bash scripts/run_stage5b_independent_visual_retrieval.sh --out /home/cas/fyp_stage5b/reproduction',
         'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /home/cas/.venvs/fyp_stage24/bin/python src/cumulti/validate_stage5b_independent_visual_retrieval.py --repo . --processed /home/cas/CU-Multi/processed_v1 --work /home/cas/fyp_stage5b/work/full_rankings --out /home/cas/fyp_stage5b/reproduction',
         '```',
         'The runner refuses to overwrite completed outputs. Only its fingerprinted SC scratch matrix may be reused; all visual scores/orderings are regenerated. Set STAGE5B_WORK to a fresh scratch directory for a full SC reconstruction. Large embedding caches are not committed.']
    (root/'docs').mkdir(exist_ok=True)
    (root/'docs/CUMULTI_STAGE5B_INDEPENDENT_VISUAL_RETRIEVAL.md').write_text('\n'.join(doc)+'\n')
    (o/'summary.txt').write_text('PASS: independent full-database visual retrieval completed; full rankings validated.\n'+allm.to_string(index=False)+'\n\nSame-view\n'+same_m.to_string(index=False)+'\n')
    print('Report generated from validated outputs.')

if __name__=='__main__':main()
