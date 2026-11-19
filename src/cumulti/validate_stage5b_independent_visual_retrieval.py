#!/usr/bin/env python3
"""Independent Stage5B output validation using frozen positive-pair labels."""
import argparse,hashlib,json,os
from pathlib import Path
import numpy as np
import pandas as pd


def main():
    a=argparse.ArgumentParser();a.add_argument('--repo',type=Path,required=True);a.add_argument('--processed',type=Path,required=True);a.add_argument('--work',type=Path,required=True);a.add_argument('--out',type=Path,required=True);a.add_argument('--previous',type=Path)
    args=a.parse_args();o=args.out;p=args.processed;w=args.work;root=args.repo
    old=pd.read_csv(root/'outputs/cumulti_v1/04_robot1_robot3_sc/query_evaluation.csv')
    pairs=pd.read_csv(root/'outputs/cumulti_v1/04_robot1_robot3_sc/gt_positive_pairs.csv')
    labels=np.zeros((2000,4180),bool);labels[pairs.query_keyframe_id,pairs.database_keyframe_id]=True
    valid=labels.any(axis=1);same=valid&(old.heading_difference_deg.to_numpy()<=10)
    assert valid.sum()==1833 and same.sum()==482
    assert np.array_equal(np.where(same)[0],pd.read_csv(o/'same_view_query_ids.csv').query_id)
    q=np.load(p/'openclip_stage5/robot1_embeddings.npy');d=np.load(p/'openclip_stage5/robot3_embeddings.npy')
    # Independent max implementation: first max over query window then over candidate window.
    single=q@d.T
    qmax=np.full_like(single,-np.inf)
    for shift in range(5):qmax=np.maximum(qmax,single[np.maximum(np.arange(2000)-shift,0)])
    cross=np.full_like(single,-np.inf)
    for shift in range(5):cross=np.maximum(cross,qmax[:,np.maximum(np.arange(4180)-shift,0)])
    qi=np.maximum(np.arange(2000)[:,None]-np.arange(4,-1,-1),0)
    di=np.maximum(np.arange(4180)[:,None]-np.arange(4,-1,-1),0)
    mq=q[qi].mean(1);md=d[di].mean(1);mq=mq/np.sqrt((mq*mq).sum(1))[:,None];md=md/np.sqrt((md*md).sum(1))[:,None]
    independent=[single,mq@md.T,cross]
    score_checks=[];method_names=['SC','Single RGB','RGB5 Mean','Cross-Max'];file_names=['sc','single','mean5','crossmax']
    ranks={};orders={};scores={}
    for j,(method,key) in enumerate(zip(method_names,file_names)):
        s=np.load(w/f'{key}_scores.npy',mmap_mode='r');order=np.load(w/f'{key}_order.npy',mmap_mode='r')
        assert s.shape==order.shape==(2000,4180) and np.isfinite(s).all()
        assert np.array_equal(np.sort(order,axis=1),np.broadcast_to(np.arange(4180),order.shape))
        assert np.array_equal(order,np.argsort(-s,axis=1,kind='stable'))
        if j:
            delta=float(np.max(np.abs(s-independent[j-1])));assert delta<=1e-6
            # Rankings must also reproduce exactly despite a different mean reduction.
            assert np.array_equal(order,np.argsort(-independent[j-1],axis=1,kind='stable'))
            # Fixed samples check scalar dot / mean / all25 reference definition.
            for a0,b0 in [(0,0),(1,3),(4,8),(305,1241),(1849,2000),(1999,4179)]:
                if j==1:reference=np.dot(q[a0],d[b0])
                elif j==2:
                    qa=q[qi[a0]].mean(0);db=d[di[b0]].mean(0)
                    reference=np.dot(qa/np.linalg.norm(qa),db/np.linalg.norm(db))
                else:reference=np.max(q[qi[a0]]@d[di[b0]].T)
                assert abs(float(reference)-float(s[a0,b0]))<=1e-6
            score_checks.append(dict(method=method,max_absolute_score_difference=delta,all_full_rankings_reproduced=True))
        # Evaluate via the inverse permutation, distinct from the runner's gathered-label argmax.
        inv=np.empty_like(order);np.put_along_axis(inv,order,np.broadcast_to(np.arange(1,4181),order.shape),axis=1)
        r=np.where(labels,inv,4181).min(axis=1);r[~valid]=-1;ranks[method]=r;orders[method]=order;scores[method]=s
        if j==0:assert np.array_equal(r,old.first_positive_rank)
    metrics_checks=[]
    for subset,mask in [('overall',valid),('same_view',same)]:
        table=pd.read_csv(o/f'metrics_{subset}.csv');query=pd.read_csv(o/f'query_results_{subset}.csv')
        assert len(query)==int(mask.sum())*4
        for method in method_names:
            z=table[table.method==method].iloc[0];r=ranks[method][mask]
            assert int(z.query_count)==mask.sum() and int(z.database_count)==4180
            for k in (1,5,10,20):assert abs(float(z[f'R@{k}'])-np.count_nonzero((r>=1)&(r<=k))/len(r))<1e-10
            assert abs(float(z.MRR)-sum(1/float(x) for x in r if x>0)/len(r))<1e-10
            group=query[query.method==method].sort_values('query_id')
            assert np.array_equal(group.query_id,np.where(mask)[0]) and np.array_equal(group.first_positive_rank,r)
            metrics_checks.append(dict(method=method,subset=subset,status='PASS'))
    top=pd.read_csv(o/'top20_rankings.csv');assert len(top)==1833*4*20
    for method,g in top.groupby('method',sort=False):
        for i,z in g.groupby('query_id',sort=False):
            assert valid[i] and np.array_equal(z['rank'],np.arange(1,21))
            assert np.array_equal(z.database_id,orders[method][i,:20])
            assert np.array_equal(z.is_positive,labels[i,z.database_id])
            assert np.allclose(z.score,scores[method][i,z.database_id],rtol=1e-8,atol=1e-9)
    # Historical baseline preservation is independently recomputed from receipt.
    audit=json.loads((o/'preservation_audit.json').read_text())
    for rel,digest in audit['frozen_files'].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==digest
    checks=dict(status='PASS',all4180_candidates_eligible_per_query=True,all2000_order_rows_complete_permutations=True,
                GT_labels_from_frozen_positive_pair_CSV=True,exact482_same_view_queries=True,
                visual_retrieval_repeated_with_independent_temporal_max=True,score_checks=score_checks,metric_checks=metrics_checks,
                top20_rows_checked=len(top),frozen_file_hashes_rechecked=True,
                no_training_or_encoding=True,limitations='CPU replay arithmetic and generic OpenCLIP; not a dedicated VPR model or online latency benchmark')
    checks['validator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if args.previous:
        names=['metrics_overall.csv','metrics_same_view.csv','query_results_overall.csv','query_results_same_view.csv','top20_rankings.csv','same_view_query_ids.csv','historical_pool_comparison.csv']
        equal={n:hashlib.sha256((o/n).read_bytes()).hexdigest()==hashlib.sha256((args.previous/n).read_bytes()).hexdigest() for n in names}
        assert all(equal.values())
        current=json.loads((o/'full_rankings_artifact_manifest.json').read_text());previous=json.loads((args.previous/'full_rankings_artifact_manifest.json').read_text())
        assert {n:x['sha256'] for n,x in current.items()}=={n:x['sha256'] for n,x in previous.items()}
        (o/'reproducibility_check.json').write_text(json.dumps(dict(status='PASS',two_executions_identical_numeric_output_hashes=equal,all_full_score_and_order_array_hashes_identical=True,initial_attempt_preserved=str(args.previous)),indent=2)+'\n')
    (o/'independent_validation.json').write_text(json.dumps(checks,indent=2)+'\n')
    with (o/'VALIDATION_REPORT.txt').open('a') as f:f.write('[PASS] independent validator reproduced complete visual rankings, inverse-permutation metrics and all146640 top20 rows\n')
    print(json.dumps(checks,indent=2))

if __name__=='__main__':main()
