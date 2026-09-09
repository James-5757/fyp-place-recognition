import numpy as np
from pathlib import Path

pose_path = Path.home() / "fyp_place_recognition/data/kitti/dataset/poses/00.txt"

poses = np.loadtxt(pose_path).reshape(-1, 3, 4)

print("Using pose file:", pose_path)
print("Pose shape:", poses.shape)
print("First pose:")
print(poses[0])

positions = poses[:, :, 3]
print("First 5 positions:")
print(positions[:5])
