#!/usr/bin/env python3
"""Actual sequential Stage-9 retrieval->teammate-GICP latency harness."""
from __future__ import annotations
import argparse, importlib.util, json, sys, tempfile, time
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
import run_stage8_efficient_sc_retrieval as s8

REPO=Path('/home/cas/fyp_place_recognition'); PROCESSED=Path('/home/cas/CU-Multi/processed_v1')
OUT=REPO/'outputs/cumulti_v1/09_sc_gicp_integration'; TEAM=Path('/home/cas/fyp_robot13_geometry_20260915')
Q=0.6091; M=1000

def team_module():
    spec=importlib.util.spec_from_file_location('teammate_gicp', TEAM/'code/robot13_gicp.py'); module=importlib.util.module_from_spec(spec); assert spec.loader; spec.loader.exec_module(module); return module

def retrieve(qd, dbu, dbvalid, qring, tree):
    _,idx=tree.query(qring,k=M,workers=1); idx=np.asarray(idx,int); values,shifts=s8.score(qd,dbu[idx],dbvalid[idx]); local=np.argsort(-values,kind='stable')[:3]
    return idx[local],values[local],shifts[local]

def call(team, row, output):
    # The teammate CLI filters a one-pair manifest by ``rank <= top_k``.  This
    # backend-local rank is therefore 1; the enclosing harness retains SC rank.
    manifest=output.with_suffix('.manifest.csv'); pd.DataFrame([{'query_robot':'robot1','query_keyframe_id':int(row.qid),'database_robot':'robot3','database_keyframe_id':int(row.cid),'rank':1,'sc_score':float(row.score),'best_shift':int(row.shift)}]).to_csv(manifest,index=False)
    args=argparse.Namespace(manifest=manifest,processed_root=PROCESSED,cache_layout='teammate_stage4',output=output,top_k=1,direction='robot1_to_robot3',limit=None,pilot_stride=1,resume=False,hourly_rate=None,voxel_size=.75,min_range=2.,max_range=50.,min_height=-3.,max_height=5.,coarse_distance=3.,fine_distance=1.,iterations=30,cache_size=64)
    team.run(args); return pd.read_csv(output).iloc[0]

def stat(values):
    values=np.asarray(values,float)*1000
    return {'mean_ms':float(values.mean()),'median_ms':float(np.median(values)),'p95_ms':float(np.percentile(values,95)),'p99_ms':float(np.percentile(values,99)),'queries_per_sec':float(1000/values.mean()),'budget_percent_500ms':float(values.mean()/5)}

def main():
    r1=pd.read_csv(PROCESSED/'full_2hz/robot1/keyframes.csv'); d3=pd.read_csv(PROCESSED/'full_2hz/robot3/keyframes.csv'); qd=np.load(PROCESSED/'full_2hz/robot1/scan_context_descriptors.npy'); dd=np.load(PROCESSED/'full_2hz/robot3/scan_context_descriptors.npy'); dbu,dbvalid=s8.s5.normalize_columns(dd); tree=cKDTree(s8.ring_key(dd)); qring=s8.ring_key(qd)
    interface=pd.read_csv(OUT/'candidate_interface.csv'); sample=np.linspace(0,len(r1)-1,100,dtype=int); team=team_module(); rows=[]
    with tempfile.TemporaryDirectory(prefix='stage9_latency_') as temp:
      temp=Path(temp)
      for qid in sample:
        start=time.perf_counter(); ids,scores,shifts=retrieve(qd[qid],dbu,dbvalid,qring[qid],tree); retrieval_s=time.perf_counter()-start
        expected=interface[interface.query_keyframe_id==qid].sort_values('SC_rank').candidate_keyframe_id.to_numpy(); assert np.array_equal(ids,expected)
        first=type('R',(),{'qid':qid,'cid':ids[0],'rank':1,'score':scores[0],'shift':shifts[0]}); record=call(team,first,temp/f'r1_{qid}.csv'); rank1_s=time.perf_counter()-start
        calls=1; selected=record
        for rank in (2,3):
          if float(selected.gicp_quality)>=Q: break
          item=type('R',(),{'qid':qid,'cid':ids[rank-1],'rank':rank,'score':scores[rank-1],'shift':shifts[rank-1]}); selected=call(team,item,temp/f'r{rank}_{qid}.csv'); calls+=1
        rows.append({'query_keyframe_id':int(qid),'retrieval_s':retrieval_s,'rank1_pipeline_s':rank1_s,'top3_early_stop_pipeline_s':time.perf_counter()-start,'gicp_calls':calls,'rank1_quality':float(record.gicp_quality),'selected_quality':float(selected.gicp_quality)})
    frame=pd.DataFrame(rows); frame.to_csv(OUT/'integrated_latency.csv',index=False)
    summary={'scope':'100 deterministic Robot1 queries; each measurement is one sequential harness invocation of Stage-8 M1000 retrieval followed by direct teammate robot13_gicp.run() call(s), including cloud access.','rank1_pipeline':stat(frame.rank1_pipeline_s),'top3_early_stop_pipeline':stat(frame.top3_early_stop_pipeline_s),'mean_gicp_calls_per_query':float(frame.gicp_calls.mean()),'quality_threshold':Q}; (OUT/'integrated_latency_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if __name__=='__main__': main()
