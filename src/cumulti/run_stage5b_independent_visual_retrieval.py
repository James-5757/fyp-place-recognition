#!/usr/bin/env python3
"""Independent all-database visual retrieval; no GT is accepted by score functions."""
from __future__ import annotations
import argparse, hashlib, json, os, platform, subprocess, time
from pathlib import Path
import numpy as np
import pandas as pd
import run_stage5_visual_viewpoint as frozen

METHODS=('SC','Single RGB','RGB5 Mean','Cross-Max')
FILES=('sc','single','mean5','crossmax')
ROOT=Path(__file__).resolve().parents[2]
DEFAULT_OUT=ROOT/'outputs/cumulti_v1/05b_independent_visual_retrieval'


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()


def dump(p,x):
    Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')


def windows(n):
    return np.maximum(np.arange(n)[:,None]+np.arange(-4,1)[None,:],0)


def visual_scores(q,d):
    """Score every row against every DB row. Inputs are embeddings only."""
    qw,dw=windows(len(q)),windows(len(d))
    single=q@d.T
    qm=q[qw].mean(axis=1);dm=d[dw].mean(axis=1)
    qm/=np.linalg.norm(qm,axis=1,keepdims=True)
    dm/=np.linalg.norm(dm,axis=1,keepdims=True)
    mean=qm@dm.T
    # Separable temporal maximum of the shared QxD cosine matrix = max of 25.
    # Memory is O(QD), not O(QD*25*512). Left padding repeats index zero.
    cross=np.full_like(single,-np.inf)
    for a in range(5):
        for b in range(5):
            np.maximum(cross,single[qw[:,a,None],dw[None,:,b]],out=cross)
    return single,mean,cross


def order_scores(scores):
    # DB rows are increasing keyframe ID, so stable sort implements score desc / ID asc.
    return np.argsort(-scores,axis=1,kind='stable').astype(np.int32)


def metric(ranks,mask):
    r=ranks[mask]
    return {'query_count':int(mask.sum()),'database_count':4180,
            **{f'R@{k}':float(np.mean((r>=1)&(r<=k))) for k in (1,5,10,20)},
            'MRR':float(np.mean(np.divide(1.,r,out=np.zeros(len(r)),where=r>0)))}


def rank_first(order,positive):
    has=positive.any(axis=1)
    return np.where(has,np.argmax(np.take_along_axis(positive,order,axis=1),axis=1)+1,-1)


def historical_files(root):
    paths=[]
    for dr in ('04_robot1_robot3_sc','05_visual_viewpoint_analysis','06_selective_visual','07_generalization_validation','07_5_supervisor_validation'):
        paths.extend(p for p in (root/'outputs/cumulti_v1'/dr).rglob('*') if p.is_file())
    paths += [root/'src/cumulti/run_stage5_visual_viewpoint.py',root/'src/cumulti/run_stage75_supervisor_validation.py',root/'docs/CUMULTI_STAGE5_VISUAL_VIEWPOINT_ANALYSIS.md']
    return {str(p.relative_to(root)):sha(p) for p in paths if p.exists()}


def main():
    a=argparse.ArgumentParser();a.add_argument('--processed',type=Path,default=Path('/home/cas/CU-Multi/processed_v1'))
    a.add_argument('--out',type=Path,default=DEFAULT_OUT);a.add_argument('--work',type=Path,required=True)
    args=a.parse_args();o=args.out;w=args.work;p=args.processed
    if (o/'run_provenance.json').exists():raise RuntimeError('Refusing to overwrite a completed experiment; use a new --out')
    o.mkdir(parents=True,exist_ok=True);w.mkdir(parents=True,exist_ok=True)
    before=historical_files(ROOT)
    input_paths=[p/'openclip_stage5/metadata.json']
    for robot in ('robot1','robot3'):
        input_paths.extend(p/x for x in (f'openclip_stage5/{robot}_embeddings.npy',f'openclip_stage5/{robot}_keyframe_ids.csv',f'full_2hz/{robot}/keyframes.csv',f'full_2hz/{robot}/rgb_sync_index.csv',f'full_2hz/{robot}/scan_context_descriptors.npy'))
    inputs={str(x):sha(x) for x in input_paths}
    meta=json.loads(input_paths[0].read_text())
    assert (meta['model_name'],meta['pretrained_tag'],meta['embedding_dim'])==('ViT-B-32-quickgelu','laion400m_e32',512)
    emb=[];audit=[]
    for robot,n in [('robot1',2000),('robot3',4180)]:
        e=np.load(p/f'openclip_stage5/{robot}_embeddings.npy',allow_pickle=False)
        ids=pd.read_csv(p/f'openclip_stage5/{robot}_keyframe_ids.csv')
        key=pd.read_csv(p/f'full_2hz/{robot}/keyframes.csv',usecols=['keyframe_id'])
        sync=pd.read_csv(p/f'full_2hz/{robot}/rgb_sync_index.csv')
        assert e.shape==(n,512) and e.dtype==np.float32 and np.isfinite(e).all()
        assert np.array_equal(ids.keyframe_id,np.arange(n)) and np.array_equal(key.keyframe_id,np.arange(n))
        assert np.array_equal(ids.keyframe_id,sync.keyframe_id) and np.array_equal(ids.rgb_timestamp_ns,sync.rgb_timestamp_ns)
        norm=np.linalg.norm(e,axis=1);assert np.max(np.abs(norm-1))<1e-5
        audit.append(dict(robot=robot,count=n,dimension=512,dtype=str(e.dtype),min_norm=float(norm.min()),max_norm=float(norm.max()),keyframe_and_rgb_timestamp_match=True,zero_rows=int((norm==0).sum())))
        emb.append(e)
    config={'dataset':'CU-Multi Main Campus','query_robot':'robot1','database_robot':'robot3',
            'all_queries_scored':2000,'database_eligible_per_query':4180,'valid_queries':1833,'same_view_queries':482,
            'positive_definition':'horizontal hypot(dx,dy) < 5.0 m','same_view':'exact Stage7.5 nearest-positive wrapped heading <= 10 degrees',
            'model':meta,'embedding_cache_reused':True,'images_reencoded':0,'window':5,'left_padding':'max(0,k-4+j), j=0..4',
            'sorting':'descending float32 score, ascending database keyframe ID for exact ties',
            'visual_scores':['current cosine','cosine of L2-normalized causal window means','maximum of all25 causal window pairwise cosines'],
            'GT_policy':'scores and all2000 full rankings saved before loading evaluation pose columns; GT offline only',
            'blas_threads':{k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
            'full_rankings_server_only':str(w),'query_order':'increasing original keyframe ID','no_overlap_queries':'scored but excluded from recall denominators'}
    dump(o/'run_config.json',config);dump(o/'embedding_cache_audit.json',audit)
    start=time.perf_counter();ss=visual_scores(*emb);times={'visual_all_methods_wall_s':time.perf_counter()-start}
    for name,s in zip(FILES[1:],ss):
        np.save(w/f'{name}_scores.npy',s);np.save(w/f'{name}_order.npy',order_scores(s))
    del ss
    print('All2000 x4180 visual rankings saved; no GT evaluation yet.',flush=True)
    # SC is regenerated only because full matrices/top20 are no longer archived.
    # Use frozen scoring verbatim, without calling its GT-bearing reconstruction wrapper.
    qdesc=np.load(p/'full_2hz/robot1/scan_context_descriptors.npy')
    ddesc=np.load(p/'full_2hz/robot3/scan_context_descriptors.npy')
    cache_key={str(x):inputs[str(x)] for x in input_paths if 'scan_context_descriptors' in str(x)}
    cache_key['frozen_sc_source_sha256']=sha(ROOT/'src/cumulti/run_stage5_visual_viewpoint.py')
    cp=w/'sc_cache_receipt.json'
    if cp.exists() and json.loads(cp.read_text())==cache_key and (w/'sc_scores.npy').exists():
        sc=np.load(w/'sc_scores.npy');times['sc_reconstruction']='reused fingerprinted Stage5B scratch matrix'
    else:
        dbu,dbv=frozen.normalize_columns(ddesc);sc=np.empty((2000,4180),np.float32);t=time.perf_counter()
        for i in range(2000):
            sc[i],_=frozen.scores_for_query(qdesc[i],dbu,dbv)
            if i%200==0:print('SC reconstruction',i,'/2000',flush=True)
        times['sc_reconstruction_s']=time.perf_counter()-t;np.save(w/'sc_scores.npy',sc);dump(cp,cache_key)
    sc_order=order_scores(sc);np.save(w/'sc_order.npy',sc_order)
    # Evaluation boundary: all four complete rankings already exist on disk.
    q=pd.read_csv(p/'full_2hz/robot1/keyframes.csv');d=pd.read_csv(p/'full_2hz/robot3/keyframes.csv')
    dist=np.hypot(q.x.to_numpy()[:,None]-d.x.to_numpy()[None,:],q.y.to_numpy()[:,None]-d.y.to_numpy()[None,:])
    pos=dist<5.;valid=pos.any(axis=1);assert int(valid.sum())==1833
    historical=pd.read_csv(ROOT/'outputs/cumulti_v1/04_robot1_robot3_sc/query_evaluation.csv')
    assert np.array_equal(valid,historical.valid_overlap_query.to_numpy())
    labels=pd.read_csv(ROOT/'outputs/cumulti_v1/04_robot1_robot3_sc/gt_positive_pairs.csv')
    label_matrix=np.zeros_like(pos);label_matrix[labels.query_keyframe_id,labels.database_keyframe_id]=True
    assert np.array_equal(label_matrix,pos)
    nearest=np.argmin(np.where(pos,dist,np.inf),axis=1)
    heading=np.abs((q.yaw_deg.to_numpy()-d.yaw_deg.to_numpy()[nearest]+180)%360-180)
    same=valid&(heading<=10)
    frozen_same=historical.valid_overlap_query.to_numpy()&(historical.heading_difference_deg.to_numpy()<=10)
    assert np.array_equal(same,frozen_same) and int(same.sum())==482
    pd.DataFrame({'query_id':np.where(same)[0]}).to_csv(o/'same_view_query_ids.csv',index=False)
    orders=[np.load(w/f'{x}_order.npy',mmap_mode='r') for x in FILES]
    ranks=[rank_first(order,pos) for order in orders]
    assert np.array_equal(ranks[0],historical.first_positive_rank.to_numpy())
    assert np.array_equal(sc_order[:,0],historical.rank1_database_keyframe_id.to_numpy())
    for subset,mask in [('overall',valid),('same_view',same)]:
        pd.DataFrame([dict(method=m,scope='independent_full_database',**metric(r,mask)) for m,r in zip(METHODS,ranks)]).to_csv(o/f'metrics_{subset}.csv',index=False,float_format='%.12g')
        rows=[]
        for m,r,order in zip(METHODS,ranks,orders):
            for i in np.where(mask)[0]:
                c=int(order[i,0]);rows.append(dict(method=m,query_id=int(i),first_positive_rank=int(r[i]),rank1_database_id=c,rank1_is_positive=bool(pos[i,c]),rank1_gt_distance_m=float(dist[i,c]),nearest_positive_database_id=int(nearest[i]),nearest_positive_distance_m=float(dist[i,nearest[i]]),nearest_positive_heading_deg=float(heading[i]),database_eligible_count=4180))
        pd.DataFrame(rows).to_csv(o/f'query_results_{subset}.csv',index=False,float_format='%.12g')
    # Save full-database top20, never use these as an input visual candidate pool.
    import csv
    with (o/'top20_rankings.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['method','query_id','rank','database_id','score','is_positive'])
        for method,name,order in zip(METHODS,FILES,orders):
            scores=np.load(w/f'{name}_scores.npy',mmap_mode='r')
            for i in np.where(valid)[0]:
                for k,c in enumerate(order[i,:20]):writer.writerow([method,i,k+1,int(c),format(float(scores[i,c]),'.9g'),int(pos[i,c])])
    # Historical-pool comparison uses the original scalar Stage5 scoring and tie order.
    qw,dw=windows(2000),windows(4180);poolranks={};reference=[]
    old_same=pd.read_csv(ROOT/'outputs/cumulti_v1/07_5_supervisor_validation/same_view_metrics.csv')
    expected_r1=[1287/1833,1251/1833,1310/1833]
    for j,(method,key) in enumerate(zip(METHODS[1:],('single','mean5','crossmax'))):
        rr=np.full(2000,-1)
        for i in np.where(valid)[0]:
            cand=sc_order[i,:20];oo,_=frozen.rerank_top20(emb[0][i],emb[1][cand],emb[0][qw[i]],emb[1][dw[cand]],key)
            pp=pos[i,cand[oo]]
            if pp.any():rr[i]=int(np.argmax(pp)+1)
        poolranks[method]=rr;assert abs(metric(rr,valid)['R@1']-expected_r1[j])<1e-12
        old=old_same[(old_same.subset=='0-10')&(old_same.method==method)].iloc[0]
        for field in ('R@1','R@5','MRR'):assert abs(metric(rr,same)[field]-float(old[field]))<1e-10
        for subset,mask in [('overall',valid),('same_view',same)]:
            reference.append(dict(method=method,subset=subset,scope='SC_top20_reranking_end_to_end',**metric(rr,mask)))
    pd.DataFrame(reference).to_csv(o/'historical_pool_comparison.csv',index=False,float_format='%.12g')
    count=int((np.take_along_axis(pos,sc_order[:,:20],axis=1).any(axis=1)&valid).sum());assert count==1830
    # A nonempty set of full-pool gains is permitted even though aggregate performance may drop.
    outside=[]
    for method,order in zip(METHODS[1:],orders[1:]):
        for i in np.where(valid)[0]:
            c=int(order[i,0])
            if pos[i,c] and c not in sc_order[i,:20]:outside.append(dict(method=method,query_id=int(i),database_id=c))
    pd.DataFrame(outside,columns=['method','query_id','database_id']).to_csv(o/'rank1_positive_outside_sc_top20.csv',index=False)
    after=historical_files(ROOT);assert before==after
    assert inputs=={str(x):sha(x) for x in input_paths}
    dump(o/'preservation_audit.json',dict(status='PASS',frozen_files=before,input_hashes_unchanged=True,raw_images_accessed=False))
    full_artifacts={x.name:{'path':str(x),'sha256':sha(x),'bytes':x.stat().st_size} for x in w.glob('*.npy')}
    dump(o/'full_rankings_artifact_manifest.json',full_artifacts)
    prov=dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip() if (ROOT/'.git').exists() else 'see input_bundle_commit.txt',
              input_bundle_commit=(ROOT/'input_bundle_commit.txt').read_text().strip() if (ROOT/'input_bundle_commit.txt').exists() else None,
              script_sha256=sha(Path(__file__)),frozen_sc_source_sha256=cache_key['frozen_sc_source_sha256'],input_sha256=inputs,
              numpy=np.__version__,pandas=pd.__version__,python=platform.python_version(),platform=platform.platform(),times=times,
              embedding_provenance='model metadata + full IDs/timestamps/norm checks + exact historical pooled metrics reproduction; no historical checkpoint hash receipt available',
              full_rankings_saved_before_GT_evaluation=True,SC_frozen_rank_and_rank1_reproduced=True,stage75_pooled_metrics_reproduced=True,
              same_view_ids_sha256=sha(o/'same_view_query_ids.csv'),all_rankings_shape=[2000,4180],evaluated_overall=1833,evaluated_same_view=482)
    dump(o/'run_provenance.json',prov)
    lines=['[PASS] all2000 queries scored against all4180 database rows before GT evaluation',
           '[PASS] no SC pool restrictions on any visual ranking','[PASS] three visual methods share complete cache/keyframe/timestamp coverage',
           '[PASS] original Stage5 left padding and normalized float32 embeddings',
           '[PASS] all1833 valid queries and exact482 frozen same-view IDs verified',
           '[PASS] strict d_xy <5m labels match every frozen Stage4 positive pair',
           '[PASS] frozen SC first-positive ranks and rank1 IDs reproduced exactly',
           '[PASS] historical Stage5 overall R@1 and Stage7.5 pooled R@1/R@5/MRR reproduced',
           '[PASS] frozen artifacts and input hashes unchanged','[PASS] no OpenCLIP encoding, fusion, training, GICP or graph optimization']
    (o/'VALIDATION_REPORT.txt').write_text('\n'.join(lines)+'\n')
    print((o/'metrics_overall.csv').read_text(),(o/'metrics_same_view.csv').read_text(),flush=True)

if __name__=='__main__':main()
