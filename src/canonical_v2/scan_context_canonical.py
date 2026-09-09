from pathlib import Path
import numpy as np

NUM_RINGS = 20
NUM_SECTORS = 60
MAX_RADIUS = 80.0
LIDAR_HEIGHT = 2.0


def load_kitti_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def make_descriptor(points, num_rings=NUM_RINGS, num_sectors=NUM_SECTORS, max_radius=MAX_RADIUS):
    xyz = points[:, :3]
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    radius = np.hypot(x, y)
    keep = (radius > 0.0) & (radius <= max_radius)
    radius, z = radius[keep], z[keep]
    theta = (np.arctan2(y[keep], x[keep]) + 2.0 * np.pi) % (2.0 * np.pi)
    ring = np.clip(np.floor(radius / max_radius * num_rings).astype(np.int32), 0, num_rings - 1)
    sector = np.clip(np.floor(theta / (2.0 * np.pi) * num_sectors).astype(np.int32), 0, num_sectors - 1)
    descriptor = np.zeros((num_rings, num_sectors), dtype=np.float32)
    np.maximum.at(descriptor, (ring, sector), z + LIDAR_HEIGHT)
    return descriptor


def aligned_similarity(query, candidate):
    best_score, best_shift = -1.0, 0
    q_norm = np.linalg.norm(query, axis=0)
    for shift in range(NUM_SECTORS):
        shifted = np.roll(candidate, shift=shift, axis=1)
        c_norm = np.linalg.norm(shifted, axis=0)
        valid = (q_norm > 0.0) & (c_norm > 0.0)
        if not np.any(valid):
            score = 0.0
        else:
            score = float(np.mean(np.sum(query[:, valid] * shifted[:, valid], axis=0) / (q_norm[valid] * c_norm[valid])))
        if score > best_score:
            best_score, best_shift = score, shift
    return best_score, best_shift
