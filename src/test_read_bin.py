import numpy as np
from pathlib import Path

bin_path = Path.home() / "fyp_place_recognition/data/kitti/dataset/sequences/00/velodyne/000000.bin"

points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)

print("Point cloud shape:", points.shape)
print("First 5 points:")
print(points[:5])

print("x range:", points[:, 0].min(), points[:, 0].max())
print("y range:", points[:, 1].min(), points[:, 1].max())
print("z range:", points[:, 2].min(), points[:, 2].max())
