import numpy as np


def load_kitti_bin(bin_path):
    """
    Load KITTI Velodyne .bin file.
    Each point is [x, y, z, intensity].
    """
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return points


def make_scan_context(points, num_rings=20, num_sectors=60, max_radius=80.0):
    """
    Convert a LiDAR point cloud into a Scan Context descriptor.

    rows    = rings, distance direction
    columns = sectors, angle direction
    value   = max height in each bin
    """
    xyz = points[:, :3]
    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    r = np.sqrt(x ** 2 + y ** 2)
    theta = np.arctan2(y, x)
    theta = (theta + 2 * np.pi) % (2 * np.pi)

    valid = (r > 0.1) & (r < max_radius)
    r = r[valid]
    theta = theta[valid]
    z = z[valid]

    ring_idx = np.floor(r / max_radius * num_rings).astype(np.int32)
    sector_idx = np.floor(theta / (2 * np.pi) * num_sectors).astype(np.int32)

    ring_idx = np.clip(ring_idx, 0, num_rings - 1)
    sector_idx = np.clip(sector_idx, 0, num_sectors - 1)

    height = z + 2.0
    sc = np.zeros((num_rings, num_sectors), dtype=np.float32)

    np.maximum.at(sc, (ring_idx, sector_idx), height)

    return sc


def cosine_similarity(a, b):
    a = a.reshape(-1)
    b = b.reshape(-1)

    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)

    if na < 1e-8 or nb < 1e-8:
        return 0.0

    return float(np.dot(a, b) / (na * nb))


def scan_context_similarity(sc1, sc2):
    """
    Compare two Scan Context descriptors with cyclic column shifts.
    Return:
        best_score: maximum cosine similarity
        best_shift: best sector shift
    """
    num_sectors = sc1.shape[1]
    best_score = -1.0
    best_shift = 0

    for shift in range(num_sectors):
        shifted_sc2 = np.roll(sc2, shift=shift, axis=1)
        score = cosine_similarity(sc1, shifted_sc2)
        if score > best_score:
            best_score = score
            best_shift = shift

    return best_score, best_shift
