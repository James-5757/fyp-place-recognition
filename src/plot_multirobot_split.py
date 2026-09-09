import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree


ROOT = Path.home() / "fyp_place_recognition"
KITTI_ROOT = ROOT / "data/kitti/dataset"
VELODYNE_DIR = KITTI_ROOT / "sequences/00/velodyne"
POSE_PATH = KITTI_ROOT / "poses/00.txt"
OUT_DIR = ROOT / "outputs/figures"


def load_positions():
    poses = np.loadtxt(POSE_PATH).reshape(-1, 3, 4)
    return poses[:, :, 3]


def get_sampled_frame_ids(frame_step):
    files = sorted(VELODYNE_DIR.glob("*.bin"))
    frame_ids = [int(f.stem) for f in files]
    return frame_ids[::frame_step]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--split_frame", type=int, default=2500)
    parser.add_argument("--frame_step", type=int, default=5)
    parser.add_argument("--boundary_gap", type=int, default=100)
    parser.add_argument("--positive_threshold", type=float, default=5.0)
    parser.add_argument("--num_example_pairs", type=int, default=12)
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    positions = load_positions()
    sampled = get_sampled_frame_ids(args.frame_step)

    robot_a = [f for f in sampled if f <= args.split_frame - args.boundary_gap]
    gap = [f for f in sampled if args.split_frame - args.boundary_gap < f < args.split_frame + args.boundary_gap]
    robot_b = [f for f in sampled if f >= args.split_frame + args.boundary_gap]

    b_pos_xz = positions[robot_b][:, [0, 2]]
    tree = cKDTree(b_pos_xz)

    valid_pairs = []
    for q in robot_a:
        q_pos_xz = positions[q][[0, 2]]
        dist, idx = tree.query(q_pos_xz, k=1)
        if dist < args.positive_threshold:
            valid_pairs.append((q, robot_b[int(idx)], float(dist)))

    # Select several example loop closure pairs for visualization
    if len(valid_pairs) > args.num_example_pairs:
        indices = np.linspace(0, len(valid_pairs) - 1, args.num_example_pairs).astype(int)
        example_pairs = [valid_pairs[i] for i in indices]
    else:
        example_pairs = valid_pairs

    x_all = positions[:, 0]
    z_all = positions[:, 2]

    plt.figure(figsize=(9, 9))
    plt.plot(x_all, z_all, linewidth=0.8, alpha=0.35, label="Full KITTI 00 trajectory")

    if robot_a:
        plt.plot(positions[robot_a, 0], positions[robot_a, 2], linewidth=1.4, label="Robot A local trajectory")
    if gap:
        plt.plot(positions[gap, 0], positions[gap, 2], linewidth=2.0, alpha=0.6, label="Boundary gap")
    if robot_b:
        plt.plot(positions[robot_b, 0], positions[robot_b, 2], linewidth=1.4, label="Robot B local trajectory")

    if valid_pairs:
        valid_q = [p[0] for p in valid_pairs]
        plt.scatter(
            positions[valid_q, 0],
            positions[valid_q, 2],
            s=16,
            alpha=0.75,
            label="Robot A valid overlap queries"
        )

    for q, c, dist in example_pairs:
        q_pos = positions[q]
        c_pos = positions[c]
        plt.plot(
            [q_pos[0], c_pos[0]],
            [q_pos[2], c_pos[2]],
            linestyle="--",
            linewidth=0.8,
            alpha=0.7
        )

    plt.scatter(positions[0, 0], positions[0, 2], s=60, marker="o", label="Start")
    plt.scatter(positions[-1, 0], positions[-1, 2], s=60, marker="x", label="End")

    plt.xlabel("x position")
    plt.ylabel("z position")
    plt.title(
        f"Two-Robot Simulation from KITTI 00\n"
        f"split={args.split_frame}, gap={args.boundary_gap}, threshold={args.positive_threshold}m"
    )
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="best")
    plt.tight_layout()

    out_path = OUT_DIR / f"multirobot_split{args.split_frame}_gap{args.boundary_gap}_thr{args.positive_threshold:g}.png"
    plt.savefig(out_path, dpi=220)
    plt.close()

    summary_path = OUT_DIR / f"multirobot_split{args.split_frame}_summary.txt"
    with open(summary_path, "w") as f:
        f.write("Multi-Robot Split Summary\n")
        f.write(f"split_frame = {args.split_frame}\n")
        f.write(f"frame_step = {args.frame_step}\n")
        f.write(f"boundary_gap = {args.boundary_gap}\n")
        f.write(f"positive_threshold = {args.positive_threshold}\n")
        f.write(f"Robot A frames = {len(robot_a)}\n")
        f.write(f"Boundary gap frames = {len(gap)}\n")
        f.write(f"Robot B frames = {len(robot_b)}\n")
        f.write(f"Valid overlap queries = {len(valid_pairs)}\n")
        f.write(f"Overlap coverage = {len(valid_pairs) / len(robot_a):.4f}\n")

    print("Saved split figure to:", out_path)
    print("Saved summary to:", summary_path)
    print("Robot A frames:", len(robot_a))
    print("Robot B frames:", len(robot_b))
    print("Valid overlap queries:", len(valid_pairs))
    print("Overlap coverage:", f"{len(valid_pairs) / len(robot_a):.4f}")


if __name__ == "__main__":
    main()
