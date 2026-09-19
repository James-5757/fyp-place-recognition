#!/usr/bin/env python3
"""Render missing fixed-M Stage-8 figures from already validated CSV outputs."""
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

out=Path("/home/cas/fyp_place_recognition/outputs/cumulti_v1/08_efficient_sc_retrieval")
acc=pd.read_csv(out/"fixed_M_accuracy.csv"); cand=pd.read_csv(out/"fixed_M_candidate_recall.csv"); lat=pd.read_csv(out/"fixed_M_latency.csv")
fig,ax=plt.subplots(dpi=140); ax.plot(cand.M,cand.candidate_recall,"o-"); ax.set(xscale="log",xlabel="shortlist M",ylabel="CandidateRecall@M",ylim=(0,1.02)); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(out/"candidate_recall_vs_M.png"); plt.close(fig)
fig,ax=plt.subplots(dpi=140); ax.plot(acc.M,acc["R@1"],"o-"); ax.set(xscale="log",xlabel="shortlist M",ylabel="R@1",ylim=(0,1.02)); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(out/"r1_vs_M.png"); plt.close(fig)
fig,ax=plt.subplots(dpi=140); ax.plot(lat.M,lat.mean_ms,"o-"); ax.set(xscale="log",xlabel="shortlist M",ylabel="mean retrieval latency (ms)"); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(out/"latency_vs_M.png"); plt.close(fig)
fig,ax=plt.subplots(dpi=140); ax.scatter(lat.mean_ms,acc["R@1"]); [ax.annotate(f"M{m}",(x,y)) for m,x,y in zip(lat.M,lat.mean_ms,acc["R@1"])]; ax.set(xlabel="mean latency (ms)",ylabel="R@1",ylim=(0,1.02)); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(out/"fixed_accuracy_latency_pareto.png"); plt.close(fig)
(out/"experiment_config.json").write_text(json.dumps({"stage":"8","accepted_frozen_commit":"c899bfacda99431e03bd01b83de25525de39b83e","protocol":"frozen Robot1->Robot3 then held-out Robot2->Robot3","ring_key":"mean over 60 sectors for each of 20 rings","kdtree":"scipy cKDTree exact Euclidean","M_grid":[10,20,50,100,200,500,1000],"progressive_schedule":[20,50,100,200,500],"margin12_quantiles":[50,60,70,80,90,95],"selection_rule":"smallest fixed M with candidate recall loss <=0.1pp and R@1 loss <=0.1pp; adaptive minimizes latency under same constraints","GT":"offline evaluation only","prohibited":["RGB","OpenCLIP","fusion","GICP","PGO","CVTNet","VLM"]},indent=2)+"\n")
