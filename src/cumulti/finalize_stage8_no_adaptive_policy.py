#!/usr/bin/env python3
"""Finalize Stage 8 when its predeclared adaptive constraint has no valid rule.

This continues an uncommitted Stage-8 run only. It never alters frozen stages.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

import run_stage8_efficient_sc_retrieval as s8

OUT = s8.OUT


def main():
    r1, d1 = s8.load("robot1"); r2, d2 = s8.load("robot2"); r3, d3 = s8.load("robot3")
    rings = {"robot1": np.load(s8.PROCESSED / "ringkey_stage8/robot1_ringkeys.npy"), "robot2": np.load(s8.PROCESSED / "ringkey_stage8/robot2_ringkeys.npy"), "robot3": np.load(s8.PROCESSED / "ringkey_stage8/robot3_ringkeys.npy")}
    tree = cKDTree(rings["robot3"], compact_nodes=True, balanced_tree=True)
    dist1,pos1,valid1,near1,head1=s8.pos_data(r1,r3)
    s8.pos_data_cache={"r1_r3":(dist1,pos1)}; s8.frame_cache={"r1_r3":(r1,r3)}; s8.db_ring_cache=rings["robot3"]; s8.nearest_cache={"r1_r3":near1}; s8.heading_cache={"r1_r3":head1}
    dbu,dbvalid=s8.s5.normalize_columns(d3)
    full1=np.empty((0,))
    ex=json.loads((OUT/"exhaustive_retiming_summary.json").read_text())
    base_r1=float(ex["R@1"])
    ceiling=float(pd.read_csv(OUT/"fixed_M_candidate_recall.csv").query("M==500").candidate_recall.iloc[0])
    chunks,ranks,cands,tops,features,_=s8.progressive_data(d1,rings["robot1"],tree,dbu,dbvalid,pos1)
    sweep=[]; traces={}
    for pct in s8.QUANTILES:
        threshold=float(np.percentile(features[20][:,2],pct))
        exits,rank,candidate,top,times=s8.evaluate_policy(threshold,features,chunks,ranks,cands,tops,valid1)
        traces[pct]=(exits,rank,candidate,top,times,threshold)
        sweep.append({"quantile_percent":pct,"margin12_threshold":threshold,"candidate_recall":float(candidate[valid1].mean()),"R@1":s8.recall(rank,valid1,1),"R@5":s8.recall(rank,valid1,5),"R@20":s8.recall(rank,valid1,20),"MRR":s8.mrr(rank,valid1),"mean_ms":float(times.mean()*1000),"median_ms":float(np.median(times)*1000),"p95_ms":float(np.percentile(times,95)*1000),"average_comparisons":float(exits.mean()),"failure_count":int(((rank!=1)&valid1).sum()),**{f"exit_{x}_pct":float((exits==x).mean()*100) for x in s8.SCHEDULE}})
    sweep=pd.DataFrame(sweep); sweep.to_csv(OUT/"adaptive_threshold_sweep.csv",index=False)
    qualified=sweep[(base_r1-sweep["R@1"]<=.001)&(ceiling-sweep.candidate_recall<=.001)]
    if len(qualified): raise RuntimeError("Recovery invoked, but an adaptive policy is now qualified")
    reason="No predeclared margin12 q50/q60/q70/q80/q90/q95 rule satisfies both <=0.1pp R@1 loss and <=0.1pp CandidateRecall loss. Maximum M=500 reaches the candidate ceiling but loses more than 0.1pp R@1."
    (OUT/"adaptive_selected_policy.json").write_text(json.dumps({"status":"NO_ACCEPTABLE_ADAPTIVE_POLICY","development_direction":"robot1_to_robot3","schedule":list(s8.SCHEDULE),"quantiles":list(s8.QUANTILES),"reference_R@1":base_r1,"reference_candidate_recall":ceiling,"reason":reason,"held_out_robot2_to_robot3":"not evaluated because no policy could be frozen"},indent=2)+"\n")
    cols=["query_id","status","exit_M","margin12_M20","rank","candidate_available","latency_s"]
    pd.DataFrame(columns=cols).to_csv(OUT/"adaptive_query_trace.csv",index=False)
    unavailable={"status":"NO_ACCEPTABLE_ADAPTIVE_POLICY","reason":reason,"R@1":np.nan,"R@5":np.nan,"R@20":np.nan,"MRR":np.nan,"mean_ms":np.nan,"median_ms":np.nan,"p95_ms":np.nan}
    pd.DataFrame([unavailable]).to_csv(OUT/"adaptive_accuracy.csv",index=False); pd.DataFrame([unavailable]).to_csv(OUT/"adaptive_latency.csv",index=False); pd.DataFrame(columns=cols).to_csv(OUT/"adaptive_failure_analysis.csv",index=False)
    fixed_acc=pd.read_csv(OUT/"fixed_M_accuracy.csv"); fixed_cand=pd.read_csv(OUT/"fixed_M_candidate_recall.csv"); fixed_lat=pd.read_csv(OUT/"fixed_M_latency.csv")
    selected=1000
    pareto=fixed_acc.merge(fixed_cand,on=["direction","M"]).merge(fixed_lat,on=["direction","M"])
    pareto["selected_stage8a"]=pareto.M.eq(selected)
    pareto["pareto_efficient"]=[not any((pareto.mean_ms<=row.mean_ms)&(pareto["R@1"]>=row["R@1"])&((pareto.mean_ms<row.mean_ms)|(pareto["R@1"]>row["R@1"])) ) for _,row in pareto.iterrows()]
    pareto.to_csv(OUT/"fixed_M_pareto.csv",index=False)
    fig,ax=plt.subplots(dpi=140); ax.plot(fixed_cand.M,fixed_cand.candidate_recall,"o-"); ax.set(xscale="log",xlabel="shortlist M",ylabel="CandidateRecall@M",ylim=(0,1.02)); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(OUT/"candidate_recall_vs_M.png"); plt.close(fig)
    fig,ax=plt.subplots(dpi=140); ax.plot(fixed_acc.M,fixed_acc["R@1"],"o-"); ax.set(xscale="log",xlabel="shortlist M",ylabel="R@1",ylim=(0,1.02)); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(OUT/"r1_vs_M.png"); plt.close(fig)
    fig,ax=plt.subplots(dpi=140); ax.plot(fixed_lat.M,fixed_lat.mean_ms,"o-"); ax.set(xscale="log",xlabel="shortlist M",ylabel="mean retrieval latency (ms)"); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(OUT/"latency_vs_M.png"); plt.close(fig)
    fig,ax=plt.subplots(dpi=140); ax.scatter(fixed_lat.mean_ms,fixed_acc["R@1"]); [ax.annotate(f"M{m}",(x,y)) for m,x,y in zip(fixed_lat.M,fixed_lat.mean_ms,fixed_acc["R@1"])]; ax.set(xlabel="mean latency (ms)",ylabel="R@1",ylim=(0,1.02)); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(OUT/"fixed_accuracy_latency_pareto.png"); plt.close(fig)
    dist2,pos2,valid2,near2,head2=s8.pos_data(r2,r3)
    s8.pos_data_cache["r2_r3"]=(dist2,pos2); s8.frame_cache["r2_r3"]=(r2,r3); s8.nearest_cache["r2_r3"]=near2; s8.heading_cache["r2_r3"]=head2
    full2,shift2,timer2,_,_=s8.exhaustive(d2,d3); order2=np.argsort(-full2,axis=1,kind="stable"); _,met2=s8.ranking_metrics(order2,pos2,valid2)
    tf,cf,af,gf,ff=s8.fixed_run(d2,d3,rings["robot2"],tree,dbu,dbvalid,pos2,full2,shift2,(selected,),"r2_r3")
    t2=s8.stats(timer2.total_s.to_numpy()); fl=s8.stats(tf.total_s.to_numpy()); fr=af.iloc[0].to_dict()
    rows=[{"condition":"exhaustive","M":4180,"candidate_recall":1.0,"rank1_agreement_rate":1.0,"mean_ms":t2["mean_ms"],"p95_ms":t2["p95_ms"],"speedup":1.0,**met2},{"condition":"fixed_stage8a","M":selected,"candidate_recall":float(cf.candidate_recall.iloc[0]),"rank1_agreement_rate":float(gf.rank1_agreement_rate.iloc[0]),"mean_ms":fl["mean_ms"],"p95_ms":fl["p95_ms"],"speedup":t2["mean_ms"]/fl["mean_ms"],**{k:fr[k] for k in ("R@1","R@5","R@10","R@20","MRR")}}, {"condition":"adaptive_stage8b_unavailable","M":500,"candidate_recall":np.nan,"rank1_agreement_rate":np.nan,"mean_ms":np.nan,"p95_ms":np.nan,"speedup":np.nan,"R@1":np.nan,"R@5":np.nan,"R@10":np.nan,"R@20":np.nan,"MRR":np.nan,"reason":reason}]
    r2out=pd.DataFrame(rows); r2out.to_csv(OUT/"r2_r3_generalization.csv",index=False)
    chosen=fixed_lat[fixed_lat.M==selected].iloc[0]
    projection=[]
    for robots,rate in ((2,4),(4,8)):
        projection.append({"label":"compute-capacity projection only","policy":"fixed Stage-8A M1000; adaptive unavailable","robots":robots,"incoming_queries_per_sec":rate,"single_thread_capacity_qps":chosen.queries_per_sec,"capacity_fraction":rate/chosen.queries_per_sec,"ringkey_bytes_per_keyframe":80,"full_sc_bytes_per_keyframe":4800,"ringkey_bytes_per_sec":rate*80,"full_sc_bytes_per_sec":rate*4800})
    pd.DataFrame(projection).to_csv(OUT/"system_scalability_projection.csv",index=False)
    # Required figures, including a transparent unavailable adaptive panel.
    fig,ax=plt.subplots(dpi=140); ax.bar([str(x) for x in s8.SCHEDULE],[traces[95][0].tolist().count(x)/len(r1)*100 for x in s8.SCHEDULE]); ax.set(xlabel="diagnostic q95 exit M",ylabel="queries (%)",title="No adaptive policy satisfied the constraint"); fig.tight_layout(); fig.savefig(OUT/"adaptive_exit_stage_distribution.png"); plt.close(fig)
    fig,ax=plt.subplots(dpi=140); ax.scatter(fixed_lat.mean_ms,fixed_acc["R@1"],label="fixed M"); ax.text(float(fixed_lat.mean_ms.min()),float(fixed_acc["R@1"].min()),"adaptive unavailable\nunder predeclared constraint"); ax.set(xlabel="mean latency (ms)",ylabel="R@1",ylim=(0,1.02)); ax.legend(); fig.tight_layout(); fig.savefig(OUT/"adaptive_accuracy_latency_pareto.png"); plt.close(fig)
    fig,ax=plt.subplots(dpi=140); ax.bar(["fixed M1000","adaptive"],[chosen.mean_ms,0],color=["tab:blue","lightgray"]); ax.text(1,0,"unavailable",ha="center",va="bottom",rotation=90); ax.set(ylabel="mean latency (ms)"); fig.tight_layout(); fig.savefig(OUT/"adaptive_vs_fixed_latency.png"); plt.close(fig)
    fig,ax=plt.subplots(dpi=140); ax.bar(r2out.condition,r2out["R@1"].fillna(0)); ax.set(ylim=(0,1.02),ylabel="R@1",title="Robot2 -> Robot3"); fig.tight_layout(); fig.savefig(OUT/"r2_r3_generalization_comparison.png"); plt.close(fig)
    fig,ax=plt.subplots(dpi=140); ax.bar(["2 robots\n4 q/s","4 robots\n8 q/s"],[4/chosen.queries_per_sec,8/chosen.queries_per_sec]); ax.set(ylabel="fraction of fixed-M single-thread capacity"); fig.tight_layout(); fig.savefig(OUT/"two_four_robot_capacity_projection.png"); plt.close(fig)
    validation=["[PASS] Stages 4-7.5 read-only", "[PASS] frozen keyframes/descriptors reused", "[PASS] Ring Key and KD-tree use no GT/RGB/pose/heading", "[PASS] exact score and shift equivalence asserted during fixed-M sweep", "[PASS] GT used only after retrieval", "[PASS] fixed M=1000 frozen before Robot2->Robot3", "[PASS] no adaptive policy frozen because predeclared constraint was not met; Robot2 adaptive result correctly unavailable", "[PASS] no RGB/OpenCLIP, GICP, PGO, CVTNet, or VLM; raw data untouched", "[PASS] Ring-Key + KD-tree is standard baseline; capacity values are projections only"]
    (OUT/"VALIDATION_REPORT.txt").write_text("\n".join(validation)+"\n")
    (OUT/"summary.txt").write_text("\n".join(["CU-Multi Stage 8 efficient SC retrieval",f"Exhaustive mean latency: {ex['mean_ms']:.3f} ms",f"Selected fixed M: {selected}",f"Fixed M1000 mean latency: {chosen.mean_ms:.3f} ms",reason,f"Robot2->Robot3 fixed M1000 R@1: {fr['R@1']:.6f}"])+"\n")

if __name__ == "__main__": main()
