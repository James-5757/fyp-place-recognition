import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

pose_path = Path.home() / "fyp_place_recognition/data/kitti/dataset/poses/00.txt"
out_path = Path.home() / "fyp_place_recognition/outputs/kitti00_trajectory.png"

poses = np.loadtxt(pose_path).reshape(-1, 3, 4)
positions = poses[:, :, 3]

x = positions[:, 0]
z = positions[:, 2]

plt.figure(figsize=(8, 8))
plt.plot(x, z, linewidth=1)
plt.scatter(x[0], z[0], marker="o", label="Start")
plt.scatter(x[-1], z[-1], marker="x", label="End")
plt.xlabel("x position")
plt.ylabel("z position")
plt.title("KITTI Odometry Sequence 00 Trajectory")
plt.axis("equal")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(out_path, dpi=200)

print(f"Saved trajectory plot to: {out_path}")
