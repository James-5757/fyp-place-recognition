from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Rectangle
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

OUT = Path("outputs/report_figures")
OUT.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({"font.family":"DejaVu Sans","figure.facecolor":"#F7F4EE","axes.facecolor":"#F7F4EE","savefig.facecolor":"#F7F4EE","axes.titleweight":"bold"})

# 1. GT rank lollipop.
miss = pd.read_csv("outputs/sc_candidate_recall_analysis/top5_miss_analysis.csv").sort_values("first_positive_rank", ascending=False).reset_index(drop=True)
fig, ax = plt.subplots(figsize=(10.5, 5.8))
colors=["#D45D3D", "#C9923D", "#447A72", "#284B63"]
for i,row in miss.iterrows():
    ax.hlines(i,1,row.first_positive_rank,color=colors[i],lw=4,alpha=.85)
    ax.scatter(row.first_positive_rank,i,s=170,color=colors[i],edgecolor="#21313D",linewidth=1.2,zorder=3)
    ax.text(row.first_positive_rank+.38,i,f"Rank {int(row.first_positive_rank)}  ·  GT {row.first_positive_gt_distance:.2f} m",va="center",fontsize=11,color="#172631")
ax.set_yticks(range(len(miss)));ax.set_yticklabels([f"Query {int(q):04d}" for q in miss.query_frame],fontsize=12)
ax.set_xticks([1,5,10,15,20]);ax.set_xlim(.5,21.2);ax.set_ylim(-.7,len(miss)-.3)
ax.axvline(5,color="#B24C63",ls="--",lw=1.4);ax.axvline(10,color="#C9923D",ls="--",lw=1.4);ax.axvline(20,color="#447A72",ls="--",lw=1.4)
ax.text(5,.0,"Top-5",rotation=90,va="bottom",ha="right",color="#B24C63",fontsize=10)
ax.text(10,.0,"Top-10",rotation=90,va="bottom",ha="right",color="#9A6B1B",fontsize=10)
ax.text(20,.0,"Top-20",rotation=90,va="bottom",ha="right",color="#285C55",fontsize=10)
ax.set_xlabel("First Ground-Truth Positive Rank in Full Scan Context Ranking",fontsize=12,fontweight="bold")
ax.set_title("Where Do the Four SC Top-5 Misses First Recover?",loc="left",fontsize=17,pad=13)
ax.text(0,1.02,"All four positives appear by Rank 15; Top-20 achieves 100% candidate recall.",transform=ax.transAxes,fontsize=10,color="#5B6873")
ax.grid(axis="x",color="#D8D3CA",lw=.8);ax.spines[["top","right","left"]].set_visible(False);ax.tick_params(axis="y",length=0)
fig.tight_layout();fig.savefig(OUT/"top5_miss_gt_rank_lollipop.png",dpi=220,bbox_inches="tight");plt.close(fig)

# 2. Transition matrix, SC-only vs frozen Top20 fusion alpha .7.
scores=pd.read_csv("outputs/top20_visual_filtering/candidate_scores.csv")
trans=[]
for q,g in scores.groupby("query_frame"):
    sc=g.sort_values(["scan_context_score","sc_rank"],ascending=[False,True]).iloc[0]
    fu=g.sort_values(["sc_cross_max_alpha_0.7","sc_rank"],ascending=[False,True]).iloc[0]
    trans.append((int(sc.is_positive),int(fu.is_positive)))
counts={(a,b):trans.count((a,b)) for a in [1,0] for b in [1,0]}
mat=np.array([[counts[(1,1)],counts[(1,0)]],[counts[(0,1)],counts[(0,0)]]])
fig,ax=plt.subplots(figsize=(8.4,6.6));im=ax.imshow(mat,cmap="YlGnBu",vmin=0,vmax=151)
for i in range(2):
 for j in range(2):
  ax.text(j,i,str(mat[i,j]),ha="center",va="center",fontsize=27,fontweight="bold",color="white" if mat[i,j]>70 else "#172631")
ax.set_xticks([0,1]);ax.set_xticklabels(["Correct","Wrong"],fontsize=13)
ax.set_yticks([0,1]);ax.set_yticklabels(["Correct","Wrong"],fontsize=13)
ax.set_xlabel("SC + Cross-Max α=0.7",fontsize=13,fontweight="bold");ax.set_ylabel("Scan Context baseline",fontsize=13,fontweight="bold")
ax.set_title("Error Transition Matrix: SC vs Top-20 SC + Cross-Max",loc="left",fontsize=16,pad=14)
ax.text(.5,-.18,"2 corrections · 0 regressions · 153 / 158 correct",transform=ax.transAxes,ha="center",fontsize=12,fontweight="bold",color="#285C55")
fig.colorbar(im,ax=ax,fraction=.046,pad=.04,label="Queries")
fig.tight_layout();fig.savefig(OUT/"sc_to_crossmax_transition_matrix.png",dpi=220,bbox_inches="tight");plt.close(fig)

# 3. Four trajectory cases with global track + local inset.
poses=np.loadtxt("data/kitti/dataset/poses/00.txt").reshape(-1,3,4)[:,:,3]
traj=poses[:,[0,2]]
top20=pd.read_csv("outputs/top20_visual_filtering/sc_top20_candidates.csv")
fig,axes=plt.subplots(2,2,figsize=(13,11),squeeze=False)
for ax,row in zip(axes.flat,miss.sort_values("query_frame").itertuples(index=False)):
    q=int(row.query_frame);pos=int(row.first_positive_frame);false=int(row.sc_rank1_frame)
    top5=top20[(top20.query_frame==q)&(top20.sc_rank<=5)].candidate_frame.astype(int).tolist()
    ax.plot(traj[:,0],traj[:,1],color="#C9CED2",lw=1.1,zorder=1)
    ax.scatter(traj[top5,0],traj[top5,1],s=35,color="#9EA8B1",label="SC Top-5",zorder=3)
    ax.scatter(traj[q,0],traj[q,1],s=100,color="#284B63",edgecolor="white",linewidth=1.2,label="Query",zorder=5)
    ax.scatter(traj[pos,0],traj[pos,1],s=100,color="#447A72",marker="*",edgecolor="white",linewidth=1.0,label="First GT positive",zorder=5)
    ax.scatter(traj[false,0],traj[false,1],s=85,color="#D45D3D",marker="X",edgecolor="white",linewidth=1.0,label="SC Rank-1 false",zorder=5)
    ax.set_title(f"Query {q:04d}: GT rank {int(row.first_positive_rank)}",loc="left",fontsize=13,fontweight="bold")
    ax.set_xlabel("World X (m)");ax.set_ylabel("World Z (m)");ax.grid(color="#E1DED7",lw=.6);ax.set_aspect("equal",adjustable="datalim")
    pts=traj[[q,pos,false]+top5]
    pad=25;ax.set_xlim(pts[:,0].min()-pad,pts[:,0].max()+pad);ax.set_ylim(pts[:,1].min()-pad,pts[:,1].max()+pad)
    ax.legend(loc="best",fontsize=8,frameon=True,framealpha=.9)
fig.suptitle("Trajectory Context of the Four Scan Context Top-5 Retrieval Misses",fontsize=18,fontweight="bold",y=.98)
fig.text(.5,.015,"Blue = query · Green star = first GT positive · Red X = SC Rank-1 false candidate · Grey = SC Top-5",ha="center",fontsize=10)
fig.tight_layout(rect=(0,.04,1,.95));fig.savefig(OUT/"top5_miss_trajectory_cases.png",dpi=220,bbox_inches="tight");plt.close(fig)
print("created trajectory/report figures")
