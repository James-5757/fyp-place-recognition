import numpy as np
from pathlib import Path
from scipy.spatial import cKDTree

ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
VELODYNE_DIR = KITTI_ROOT / "sequences/00/velodyne"
POSE_PATH = KITTI_ROOT / "poses/00.txt"

FRAME_STEP = 5
POSITIVE_THRESHOLD = 5.0
BOUNDARY_GAP = 100

SPLIT_CANDIDATES = [1000, 1500, 2000, 2500, 3000, 3500]


def load_positions():
    poses = np.loadtxt(POSE_PATH).reshape(-1, 3, 4)
    positions = poses[:, :, 3]
    return positions


def get_sampled_frame_ids():
    files = sorted(VELODYNE_DIR.glob("*.bin"))
    frame_ids = [int(f.stem) for f in files]
    return frame_ids[::FRAME_STEP]


def main():
    positions = load_positions()
    sampled = get_sampled_frame_ids()

    print("Formal split overlap analysis")
    print(f"FRAME_STEP = {FRAME_STEP}")
    print(f"POSITIVE_THRESHOLD = {POSITIVE_THRESHOLD} m")
    print(f"BOUNDARY_GAP = {BOUNDARY_GAP} frames")
    print()

    print(
        f"{'split':>8} | {'A size':>6} | {'B size':>6} | "
        f"{'valid queries':>13} | {'coverage':>8} | {'mean nearest dist':>17}"
    )
    print("-" * 80)

    for split in SPLIT_CANDIDATES:
        robot_a = [f for f in sampled if f <= split - BOUNDARY_GAP]
        robot_b = [f for f in sampled if f >= split + BOUNDARY_GAP]

        if len(robot_a) == 0 or len(robot_b) == 0:
            continue

        a_pos = positions[robot_a][:, [0, 2]]
        b_pos = positions[robot_b][:, [0, 2]]

        tree = cKDTree(b_pos)
        nearest_dist, nearest_idx = tree.query(a_pos, k=1)

        valid = nearest_dist < POSITIVE_THRESHOLD
        valid_queries = int(valid.sum())
        coverage = valid_queries / len(robot_a)

        mean_nearest = float(nearest_dist.mean())

        print(
            f"{split:>8} | {len(robot_a):>6} | {len(robot_b):>6} | "
            f"{valid_queries:>13} | {coverage:>8.3f} | {mean_nearest:>17.2f}"
        )


if __name__ == "__main__":
    main()
