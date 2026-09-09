import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
POSE_PATH = KITTI_ROOT / "poses/00.txt"
CANDIDATE_PATH = ROOT / "outputs/baseline_candidates.csv"
OUT_DIR = ROOT / "outputs/retrieval_cases"

OUT_DIR.mkdir(parents=True, exist_ok=True)

poses = np.loadtxt(POSE_PATH).reshape(-1, 3, 4)
positions = poses[:, :, 3]

df = pd.read_csv(CANDIDATE_PATH)

# 只看 Top-1
top1 = df[df["rank"] == 1].copy()

success_cases = top1[top1["is_positive"] == 1].head(5)
failure_cases = top1[top1["is_positive"] == 0].head(5)


def plot_case(row, out_path, title):
    q = int(row["query_frame"])
    c = int(row["candidate_frame"])
    dist = float(row["gt_distance"])
    score = float(row["scan_context_score"])

    x = positions[:, 0]
    z = positions[:, 2]

    q_pos = positions[q]
    c_pos = positions[c]

    plt.figure(figsize=(8, 8))
    plt.plot(x, z, linewidth=0.8, alpha=0.5, label="KITTI 00 trajectory")

    plt.scatter(q_pos[0], q_pos[2], s=80, marker="o", label=f"Query {q}")
    plt.scatter(c_pos[0], c_pos[2], s=80, marker="x", label=f"Candidate {c}")

    plt.plot([q_pos[0], c_pos[0]], [q_pos[2], c_pos[2]], linestyle="--", linewidth=1)

    plt.xlabel("x position")
    plt.ylabel("z position")
    plt.title(f"{title}\nscore={score:.3f}, gt_distance={dist:.2f}m")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()


for i, (_, row) in enumerate(success_cases.iterrows()):
    plot_case(row, OUT_DIR / f"success_top1_{i}.png", "Top-1 Success Case")

for i, (_, row) in enumerate(failure_cases.iterrows()):
    plot_case(row, OUT_DIR / f"failure_top1_{i}.png", "Top-1 Failure Case")

print(f"Saved cases to: {OUT_DIR}")
print("Success cases:", len(success_cases))
print("Failure cases:", len(failure_cases))
