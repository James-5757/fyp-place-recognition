import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = Path("outputs/report_figures")
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.titleweight": "bold",
    "axes.labelweight": "bold",
    "axes.edgecolor": "#C7D0D8",
    "axes.linewidth": 1.0,
    "figure.facecolor": "#F7F4EE",
    "axes.facecolor": "#F7F4EE",
    "savefig.facecolor": "#F7F4EE",
})

# Read final method metrics from existing experiment outputs.
bev = pd.read_csv("outputs/full_bev_reranking/metrics.csv").set_index("method")
rgb = pd.read_csv("outputs/full_rgb_reranking/metrics.csv").set_index("method")
temporal = pd.read_csv("outputs/temporal_rgb_reranking/metrics.csv").set_index("method")
cross = pd.read_csv("outputs/temporal_cross_frame_reranking/metrics.csv").set_index("method")

rows = [
    ("Scan Context", "scan_context_baseline", cross),
    ("BEV OpenCLIP", "aligned_bev_only", bev),
    ("Single-frame RGB", "single_frame_rgb_only", rgb),
    ("Temporal Mean-5", "temporal_5_rgb_only", temporal),
    ("Cross-Frame Max", "temporal_cross_max", cross),
    ("SC + Cross-Max (alpha=0.7)", "sc_cross_max_fusion_alpha_0.7", cross),
]
summary = []
for label, key, table in rows:
    r = table.loc[key]
    summary.append({
        "method": label,
        "r1_correct": int(r.top1_hits),
        "total_valid": int(r.queries),
        "r1": float(r.r_at_1),
        "mrr": float(r.mrr),
        "corrections": int(r.corrections),
        "regressions": int(r.regressions),
    })
summary = pd.DataFrame(summary)
summary.to_csv(OUT / "method_summary.csv", index=False)

# Figure 1: publication-style lollipop comparison.
colors = ["#284B63", "#8395A7", "#B24C63", "#C9923D", "#447A72", "#D45D3D"]
order = summary.iloc[::-1].reset_index(drop=True)
fig, ax = plt.subplots(figsize=(12, 6.4))
base = 80
for i, row in order.iterrows():
    x = row.r1 * 100
    ax.hlines(i, base, x, color=colors[len(order)-1-i], lw=4, alpha=.85)
    ax.scatter(x, i, s=165, color=colors[len(order)-1-i], edgecolor="#24313C", linewidth=1.2, zorder=3)
    ax.text(x + .28, i, f"{x:.2f}%  ({row.r1_correct}/{row.total_valid})", va="center", fontsize=11, color="#16232E")
ax.axvline(95.5696, color="#284B63", lw=1.4, ls="--", alpha=.75)
ax.annotate("+0.63 pp vs SC", xy=(96.2025, 0), xytext=(92.2, .65), arrowprops=dict(arrowstyle="->", color="#D45D3D", lw=1.4), color="#B54530", fontsize=11, fontweight="bold")
ax.set_yticks(range(len(order))); ax.set_yticklabels(order.method, fontsize=12)
ax.set_xlim(base, 100.3); ax.set_xlabel("R@1 (%)", fontsize=13)
ax.set_title("Place Recognition Performance Across Visual Verification Stages", loc="left", fontsize=17, pad=14)
ax.text(0, 1.02, "Same KITTI Sequence 00 formal split; final method is conservative SC + Cross-Max fusion", transform=ax.transAxes, fontsize=10, color="#5B6873")
ax.grid(axis="x", color="#D8D3CA", lw=.8, alpha=.8); ax.spines[["top", "right", "left"]].set_visible(False); ax.tick_params(axis="y", length=0)
fig.tight_layout(); fig.savefig(OUT / "method_r1_lollipop.png", dpi=220, bbox_inches="tight"); plt.close(fig)

# Figure 2: viewpoint motivation diagram.
fig, ax = plt.subplots(figsize=(12, 6.5)); ax.set_axis_off(); ax.set_xlim(0, 12); ax.set_ylim(0, 7)
ax.text(.4, 6.45, "Why Cross-Frame Matching Instead of Single-Frame RGB?", fontsize=19, fontweight="bold", color="#16232E")
ax.text(.4, 5.95, "Forward-facing cameras can observe the same place from near-opposite headings.", fontsize=11, color="#5B6873")
# Place / direction schematic
ax.add_patch(FancyBboxPatch((.7, 3.4), 3.1, 1.45, boxstyle="round,pad=.15", fc="#E7F0EE", ec="#447A72", lw=1.6))
ax.text(2.25, 4.38, "Same physical place", ha="center", fontsize=13, fontweight="bold", color="#285C55")
ax.text(2.25, 3.82, "spatial distance: 1.7–3.2 m", ha="center", fontsize=10)
ax.add_patch(FancyArrowPatch((1.0, 2.6), (3.1, 2.6), arrowstyle="->", mutation_scale=18, lw=3, color="#284B63"))
ax.add_patch(FancyArrowPatch((3.5, 2.05), (1.4, 2.05), arrowstyle="->", mutation_scale=18, lw=3, color="#B24C63"))
ax.text(2.05, 2.88, "Query camera", ha="center", fontsize=11, color="#284B63")
ax.text(2.45, 1.48, "Candidate camera", ha="center", fontsize=11, color="#B24C63")
ax.text(2.25, .85, "heading difference: 128°–172°", ha="center", fontsize=12, fontweight="bold", color="#B54530")
# Pipeline rationale
steps = [("Single RGB", "Highly viewpoint-sensitive", "#B24C63"), ("Temporal Mean Pooling", "May dilute distinctive observations", "#C9923D"), ("Cross-Frame Matching", "Searches compatible observations across time", "#447A72"), ("Cross-Max", "Lets strong local temporal evidence dominate", "#284B63")]
x = 5.1
for i, (head, sub, color) in enumerate(steps):
    y = 5.2 - i*1.35
    ax.add_patch(FancyBboxPatch((x, y), 6.0, .85, boxstyle="round,pad=.14", fc="#FFFFFF", ec=color, lw=1.6))
    ax.text(x+.25, y+.58, head, fontsize=12, fontweight="bold", color=color, va="center")
    ax.text(x+.25, y+.25, sub, fontsize=10.5, color="#3D4B55", va="center")
    if i < len(steps)-1: ax.add_patch(FancyArrowPatch((8.1,y-.08),(8.1,y-0.42),arrowstyle="->",mutation_scale=14,lw=1.4,color="#84919B"))
ax.text(.7, .15, "Spatial proximity does not guarantee visual overlap for forward-facing cameras.", fontsize=11, fontweight="bold", color="#16232E")
fig.tight_layout(); fig.savefig(OUT / "viewpoint_crossframe_motivation.png", dpi=220, bbox_inches="tight"); plt.close(fig)

# Figure 3: candidate recall ceiling / funnel.
recall = pd.read_csv("outputs/sc_candidate_recall_analysis/recall_curve.csv")
fig, ax = plt.subplots(figsize=(12, 7)); ax.set_axis_off(); ax.set_xlim(0, 12); ax.set_ylim(0, 8)
ax.text(.4, 7.45, "The New Bottleneck: Candidate Recall Ceiling", fontsize=19, fontweight="bold", color="#16232E")
levels = [("158 valid queries", "Formal evaluation set", 6.0, "#284B63"), ("SC Top-5 contains GT", "154 / 158  ·  97.47%", 4.45, "#447A72"), ("SC + Cross-Max correct", "152 / 158  ·  96.20%", 2.9, "#C9923D")]
for title, sub, y, color in levels:
    ax.add_patch(FancyBboxPatch((.85,y),5.3,1.0,boxstyle="round,pad=.18",fc="#FFFFFF",ec=color,lw=2))
    ax.text(3.5,y+.63,title,ha="center",fontsize=14,fontweight="bold",color="#16232E")
    ax.text(3.5,y+.26,sub,ha="center",fontsize=11,color="#5B6873")
    if y>3: ax.add_patch(FancyArrowPatch((3.5,y-.1),(3.5,y-.5),arrowstyle="->",mutation_scale=18,lw=2,color="#8395A7"))
ax.text(6.55,4.65,"4 retrieval misses\nGT absent from SC Top-5",fontsize=11,color="#B54530",va="center")
ax.text(6.55,3.07,"2 reranking misses\nGT available in Top-5",fontsize=11,color="#B54530",va="center")
ax.text(8.4,6.0,"Remaining errors = 6",fontsize=14,fontweight="bold",color="#16232E")
ax.barh(4.9,4, left=8.4, height=.6, color="#B24C63", label="Retrieval: 4")
ax.barh(4.9,2, left=12.4, height=.6, color="#C9923D", label="Reranking: 2")
ax.set_xlim(0,15.1)
ax.text(8.4,4.15,"Retrieval\n4 / 6 = 66.7%",ha="center",fontsize=11,color="#8F354A")
ax.text(13.35,4.15,"Reranking\n2 / 6 = 33.3%",ha="center",fontsize=11,color="#8A651F")
ax.text(8.4,2.3,"Recall@20 = 158 / 158 = 100%\nThe candidate ceiling can be removed by enlarging SC proposals to Top-20.",fontsize=12,color="#285C55",fontweight="bold")
fig.tight_layout(); fig.savefig(OUT / "candidate_recall_ceiling_funnel.png", dpi=220, bbox_inches="tight"); plt.close(fig)

# Figure 4: Recall@K curve.
fig, ax = plt.subplots(figsize=(10.8, 6.2))
ax.plot(recall.k, recall.recall_at_k*100, color="#284B63", lw=3, marker="o", ms=8, mfc="#F7F4EE", mec="#284B63", mew=2)
for row in recall.itertuples(index=False): ax.annotate(f"{row.recall_at_k*100:.2f}%", (row.k,row.recall_at_k*100), textcoords="offset points", xytext=(0,10), ha="center", fontsize=10, fontweight="bold")
ax.axhline(100,color="#447A72",lw=1.2,ls="--"); ax.axvline(20,color="#D45D3D",lw=1.4,ls="--")
ax.text(20.7,95.85,"Top-20 reaches\n100% candidate recall",color="#B54530",fontsize=11,fontweight="bold")
ax.set_xticks(recall.k); ax.set_ylim(94.8,100.35); ax.set_xlabel("SC Candidate Pool Size K",fontsize=13); ax.set_ylabel("Candidate Recall (%)",fontsize=13)
ax.set_title("Can Higher-Recall Retrieval Break the Ceiling?",loc="left",fontsize=17,pad=14)
ax.grid(True,color="#D8D3CA",lw=.8);ax.spines[["top","right"]].set_visible(False)
fig.tight_layout();fig.savefig(OUT / "candidate_recall_at_k.png",dpi=220,bbox_inches="tight");plt.close(fig)

print("Created", OUT)
