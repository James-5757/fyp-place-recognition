from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

OUT = Path("outputs/report_figures")
BG = "#F7F4EE"
INK = "#172631"
BLUE = "#284B63"
TEAL = "#447A72"
RED = "#D45D3D"
GOLD = "#C9923D"
GREY = "#9EA8B1"
GRID = "#D8D3CA"
plt.rcParams.update({"font.family": "DejaVu Sans", "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG})

# Figure 1: use the frozen alpha=0.6 filtering result, not alpha=0.7 ranking ablation.
methods = pd.DataFrame([
    ("Scan Context", 151, 158),
    ("BEV OpenCLIP", 145, 158),
    ("Single-frame RGB", 141, 158),
    ("Temporal Mean-5", 135, 158),
    ("Cross-Frame Max", 147, 158),
    ("SC + Cross-Max (alpha=0.6)", 152, 158),
], columns=["method", "hits", "total"])
methods["r1"] = 100 * methods.hits / methods.total
colors = [BLUE, "#91A2B2", "#B94D69", GOLD, TEAL, RED]
fig, ax = plt.subplots(figsize=(12, 7.2))
y = np.arange(len(methods))
ax.hlines(y, 80, methods.r1, color=colors, lw=5, alpha=.9)
ax.scatter(methods.r1, y, s=260, color=colors, edgecolor=INK, lw=1.5, zorder=3)
for yi, row in zip(y, methods.itertuples()):
    ax.text(100.65, yi, f"{row.r1:.2f}%  ({row.hits}/{row.total})", ha="right", va="center", fontsize=12, color=INK)
ax.axvline(methods.iloc[0].r1, color=BLUE, ls="--", lw=1.5, alpha=.8)
ax.text(methods.iloc[0].r1 + .05, -.68, "SC baseline", color=BLUE, fontsize=10)
ax.annotate("Filtering result\n157/158 retained at Top-5", xy=(methods.iloc[5].r1, y[5]), xytext=(90.9, 4.45), arrowprops=dict(arrowstyle="->", lw=2, color=RED), color=RED, fontsize=11, fontweight="bold", ha="left")
ax.set_yticks(y, methods.method, fontsize=12)
ax.invert_yaxis(); ax.set_xlim(80, 100.8); ax.set_xticks([80,85,90,95,100])
ax.set_xlabel("R@1 (%)", fontsize=13, fontweight="bold")
ax.grid(axis="x", color=GRID, lw=.8)
ax.spines[["top", "right", "left"]].set_visible(False); ax.tick_params(axis="y", length=0)
fig.suptitle("Place Recognition Performance Across Visual Verification Stages", x=.08, y=.985, ha="left", fontsize=18, color=INK)
fig.text(.08, .945, "KITTI Sequence 00 · alpha=0.6 is frozen for high-retention Top-20 → Top-5 filtering", fontsize=10, color="#5B6873")
fig.subplots_adjust(left=.27, right=.95, top=.84, bottom=.14)
fig.savefig(OUT / "method_r1_lollipop.png", dpi=220, bbox_inches="tight", facecolor=BG)
plt.close(fig)

# Figure 2: explicit retrieval-set semantics. The GT-positive is deliberately outside SC Top-5.
miss = pd.read_csv("outputs/sc_candidate_recall_analysis/top5_miss_analysis.csv").sort_values("query_frame")
top20 = pd.read_csv("outputs/top20_visual_filtering/sc_top20_candidates.csv")
poses = np.loadtxt("data/kitti/dataset/poses/00.txt").reshape(-1, 3, 4)[:, :, 3]
traj = poses[:, [0, 2]]
fig, axes = plt.subplots(2, 2, figsize=(14, 11), squeeze=False)
legend_handles = [
    Line2D([0], [0], color="#C9CED2", lw=1.2, label="Sequence 00 trajectory"),
    Line2D([0], [0], marker="o", color="w", markerfacecolor=GREY, markersize=8, label="SC Top-5 retrieved candidates"),
    Line2D([0], [0], marker="o", color="w", markerfacecolor=BLUE, markeredgecolor="white", markersize=10, label="Query"),
    Line2D([0], [0], marker="*", color="w", markerfacecolor=TEAL, markeredgecolor=INK, markersize=13, label="First GT positive (outside SC Top-5)"),
    Line2D([0], [0], marker="X", color="w", markerfacecolor=RED, markeredgecolor="white", markersize=10, label="SC Rank-1 false candidate"),
]
for ax, row in zip(axes.flat, miss.itertuples(index=False)):
    q = int(row.query_frame); gt = int(row.first_positive_frame); false = int(row.sc_rank1_frame)
    top5 = top20[(top20.query_frame == q) & (top20.sc_rank <= 5)].candidate_frame.astype(int).tolist()
    assert gt not in top5, f"Query {q}: expected GT outside SC Top-5"
    ax.plot(traj[:,0], traj[:,1], color="#C9CED2", lw=1.0, zorder=1)
    ax.scatter(traj[top5,0], traj[top5,1], s=38, color=GREY, zorder=2)
    ax.scatter(*traj[q], s=110, color=BLUE, edgecolor="white", lw=1.1, zorder=5)
    ax.scatter(*traj[gt], s=160, color=TEAL, marker="*", edgecolor=INK, lw=.8, zorder=6)
    ax.scatter(*traj[false], s=100, color=RED, marker="X", edgecolor="white", lw=.8, zorder=6)
    ax.plot([traj[q,0], traj[gt,0]], [traj[q,1], traj[gt,1]], color=TEAL, ls="--", lw=1.2, alpha=.9, zorder=3)
    ax.plot([traj[q,0], traj[false,0]], [traj[q,1], traj[false,1]], color=RED, ls=":", lw=1.4, alpha=.9, zorder=3)
    points = traj[[q, gt, false] + top5]
    center = points.mean(axis=0)
    span = max(points[:, 0].max() - points[:, 0].min(), points[:, 1].max() - points[:, 1].min())
    half = max(span / 2 + 22, 45)
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="#E1DED7", lw=.6)
    ax.set_title(f"Query {q:04d} · GT first appears at SC Rank {int(row.first_positive_rank)}", loc="left", fontsize=12, fontweight="bold")
    ax.set_xlabel("World X (m)"); ax.set_ylabel("World Z (m)")
fig.legend(handles=legend_handles, loc="lower center", ncol=3, fontsize=9, frameon=True, bbox_to_anchor=(.5, .02))
fig.suptitle("Trajectory Context of Four SC Top-5 Retrieval Misses", fontsize=18, fontweight="bold", y=.985)
fig.text(.5, .945, "The GT positive is intentionally shown separately because it is absent from the original SC Top-5 set.", ha="center", fontsize=10, color="#5B6873")
fig.subplots_adjust(left=.07, right=.98, top=.89, bottom=.14, hspace=.34, wspace=.22)
fig.savefig(OUT / "top5_miss_trajectory_cases.png", dpi=220, bbox_inches="tight", facecolor=BG)
plt.close(fig)
print("repaired alpha and trajectory figures")
