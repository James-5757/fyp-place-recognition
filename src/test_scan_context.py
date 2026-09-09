import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from scan_context import load_kitti_bin, make_scan_context

root = Path.home() / "fyp_place_recognition"
bin_path = root / "data/kitti/dataset/sequences/00/velodyne/000000.bin"
out_path = root / "outputs/scan_context_000000.png"

points = load_kitti_bin(bin_path)
sc = make_scan_context(points)

print("Point cloud shape:", points.shape)
print("Scan Context shape:", sc.shape)
print("Scan Context min/max:", sc.min(), sc.max())

plt.figure(figsize=(10, 4))
plt.imshow(sc, aspect="auto", origin="lower")
plt.colorbar(label="Max height")
plt.xlabel("Sector")
plt.ylabel("Ring")
plt.title("Scan Context Descriptor - KITTI 00 Frame 000000")
plt.tight_layout()
plt.savefig(out_path, dpi=200)

print(f"Saved Scan Context image to: {out_path}")
