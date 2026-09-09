import os
import argparse
import numpy as np
from PIL import Image
from tqdm import tqdm


def load_kitti_bin(bin_path):
    """
    Load one KITTI Velodyne point cloud.

    Each point:
        x, y, z, intensity
    """
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return points


def pointcloud_to_bev(
    points,
    x_range=(-40.0, 40.0),
    y_range=(-40.0, 40.0),
    z_range=(-3.0, 2.0),
    resolution=0.2,
):
    """
    Convert KITTI LiDAR point cloud into a 3-channel BEV image.

    Channel 1: normalized maximum height
    Channel 2: point density
    Channel 3: maximum LiDAR intensity
    """

    x_min, x_max = x_range
    y_min, y_max = y_range
    z_min, z_max = z_range

    # ---------------------------------------------------------
    # 1. Keep only points inside the selected local area
    # ---------------------------------------------------------
    mask = (
        (points[:, 0] >= x_min)
        & (points[:, 0] < x_max)
        & (points[:, 1] >= y_min)
        & (points[:, 1] < y_max)
        & (points[:, 2] >= z_min)
        & (points[:, 2] < z_max)
    )

    points = points[mask]

    height = int((x_max - x_min) / resolution)
    width = int((y_max - y_min) / resolution)

    if len(points) == 0:
        return np.zeros((height, width, 3), dtype=np.uint8)

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    intensity = points[:, 3]

    # ---------------------------------------------------------
    # 2. Convert metric coordinates to image coordinates
    #
    # x = forward/backward
    # y = left/right
    #
    # Large x is placed toward the top of image.
    # ---------------------------------------------------------
    rows = ((x_max - x) / resolution).astype(np.int32)
    cols = ((y_max - y) / resolution).astype(np.int32)

    rows = np.clip(rows, 0, height - 1)
    cols = np.clip(cols, 0, width - 1)

    # ---------------------------------------------------------
    # 3. Height channel
    # ---------------------------------------------------------
    normalized_z = (z - z_min) / (z_max - z_min)
    normalized_z = np.clip(normalized_z, 0.0, 1.0)

    height_map = np.zeros((height, width), dtype=np.float32)

    # Store maximum height for each grid cell
    np.maximum.at(height_map, (rows, cols), normalized_z)

    # ---------------------------------------------------------
    # 4. Density channel
    # ---------------------------------------------------------
    density_map = np.zeros((height, width), dtype=np.float32)

    np.add.at(density_map, (rows, cols), 1)

    # Log normalization prevents dense cells from dominating
    density_map = np.minimum(
        1.0,
        np.log1p(density_map) / np.log(64.0)
    )

    # ---------------------------------------------------------
    # 5. Intensity channel
    # ---------------------------------------------------------
    intensity_map = np.zeros((height, width), dtype=np.float32)

    intensity = np.clip(intensity, 0.0, 1.0)

    np.maximum.at(
        intensity_map,
        (rows, cols),
        intensity
    )

    # ---------------------------------------------------------
    # 6. Combine into pseudo-RGB image
    #
    # R = height
    # G = density
    # B = intensity
    # ---------------------------------------------------------
    bev = np.stack(
        [
            height_map,
            density_map,
            intensity_map
        ],
        axis=-1
    )

    bev = (bev * 255).astype(np.uint8)

    return bev


def generate_single_frame(bin_path, output_path):
    points = load_kitti_bin(bin_path)

    bev = pointcloud_to_bev(points)

    image = Image.fromarray(bev, mode="RGB")
    image.save(output_path)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input_dir",
        default="data/kitti/dataset/sequences/00/velodyne",
        help="KITTI Velodyne directory"
    )

    parser.add_argument(
        "--output_dir",
        default="data/kitti_bev/00",
        help="Output BEV image directory"
    )

    parser.add_argument(
        "--frame_step",
        type=int,
        default=5,
        help="Generate every Nth frame"
    )

    parser.add_argument(
        "--single_frame",
        type=int,
        default=None,
        help="Generate only one frame, e.g. 1550"
    )

    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # ---------------------------------------------------------
    # Single-frame test
    # ---------------------------------------------------------
    if args.single_frame is not None:

        frame_name = f"{args.single_frame:06d}"

        bin_path = os.path.join(
            args.input_dir,
            frame_name + ".bin"
        )

        output_path = os.path.join(
            args.output_dir,
            frame_name + ".png"
        )

        if not os.path.exists(bin_path):
            raise FileNotFoundError(bin_path)

        generate_single_frame(
            bin_path,
            output_path
        )

        print(f"Saved: {output_path}")
        return

    # ---------------------------------------------------------
    # Batch generation
    # ---------------------------------------------------------
    files = sorted(
        f
        for f in os.listdir(args.input_dir)
        if f.endswith(".bin")
    )

    files = files[::args.frame_step]

    print(f"Number of frames to process: {len(files)}")

    for filename in tqdm(files):

        bin_path = os.path.join(
            args.input_dir,
            filename
        )

        output_name = filename.replace(
            ".bin",
            ".png"
        )

        output_path = os.path.join(
            args.output_dir,
            output_name
        )

        generate_single_frame(
            bin_path,
            output_path
        )

    print()
    print("Finished.")
    print(f"BEV images saved to: {args.output_dir}")


if __name__ == "__main__":
    main()