from pathlib import Path

import numpy as np
import pandas as pd


# ============================================================
# Configuration
# ============================================================

POSE_PATH = Path(
    "data/kitti/dataset/poses/00.txt"
)

CSV_PATH = Path(
    "outputs/formal_split2500_gap100_step5_thr5_candidates.csv"
)

OUTPUT_DIR = Path(
    "outputs/yaw_validation"
)

OUTPUT_DIR.mkdir(
    parents=True,
    exist_ok=True
)

NUM_SECTORS = 60

DEG_PER_SHIFT = 360.0 / NUM_SECTORS


# The six pairs we particularly care about
TARGET_PAIRS = [
    (1545, 4525),  # wrong
    (1545, 4540),  # correct

    (1550, 3555),  # wrong
    (1550, 4540),  # correct

    (1560, 3545),  # wrong
    (1560, 4535),  # correct
]


# ============================================================
# Angle helpers
# ============================================================

def wrap_deg(angle):
    """
    Wrap angle to [-180, 180).
    """
    return (angle + 180.0) % 360.0 - 180.0


def circular_error_deg(estimate, ground_truth):
    """
    Absolute circular angular error.
    """
    return abs(
        wrap_deg(
            estimate - ground_truth
        )
    )


# ============================================================
# Load KITTI poses
# ============================================================

def load_kitti_poses(path):

    poses = []

    with open(path, "r") as f:

        for line in f:

            values = np.fromstring(
                line.strip(),
                sep=" ",
                dtype=np.float64
            )

            if len(values) != 12:
                raise ValueError(
                    f"Expected 12 values per pose, "
                    f"got {len(values)}"
                )

            pose = values.reshape(
                3, 4
            )

            poses.append(pose)

    return poses


poses = load_kitti_poses(
    POSE_PATH
)

print(
    f"Loaded {len(poses)} KITTI poses."
)


# ============================================================
# Extract vehicle heading
# ============================================================

def get_heading_deg(frame_id):
    """
    Estimate heading from KITTI camera pose.

    KITTI camera forward direction is local +Z.
    We transform this direction into the world frame
    and project it onto the world X-Z plane.
    """

    pose = poses[int(frame_id)]

    R = pose[:, :3]

    # Local camera +Z = forward direction
    forward_world = R[:, 2]

    fx = forward_world[0]
    fz = forward_world[2]

    heading = np.degrees(
        np.arctan2(
            fx,
            fz
        )
    )

    return wrap_deg(heading)


def gt_relative_yaw(
    query_frame,
    candidate_frame
):
    """
    Candidate heading relative to query heading.
    """

    query_heading = get_heading_deg(
        query_frame
    )

    candidate_heading = get_heading_deg(
        candidate_frame
    )

    relative = wrap_deg(
        candidate_heading
        - query_heading
    )

    return relative


# ============================================================
# Load Scan Context candidates
# ============================================================

df = pd.read_csv(
    CSV_PATH
)

print(
    f"Loaded {len(df)} candidate rows."
)


# ============================================================
# Compute yaw estimates
# ============================================================

records = []

for _, row in df.iterrows():

    query_frame = int(
        row["query_frame"]
    )

    candidate_frame = int(
        row["candidate_frame"]
    )

    best_shift = int(
        row["best_shift"]
    )

    gt_yaw = gt_relative_yaw(
        query_frame,
        candidate_frame
    )

    raw_shift_deg = (
        best_shift
        * DEG_PER_SHIFT
    )

    # We do NOT assume the sign convention yet.
    sc_yaw_plus = wrap_deg(
        raw_shift_deg
    )

    sc_yaw_minus = wrap_deg(
        -raw_shift_deg
    )

    error_plus = circular_error_deg(
        sc_yaw_plus,
        gt_yaw
    )

    error_minus = circular_error_deg(
        sc_yaw_minus,
        gt_yaw
    )

    records.append(
        {
            "query_frame":
                query_frame,

            "candidate_frame":
                candidate_frame,

            "rank":
                int(row["rank"]),

            "is_positive":
                int(row["is_positive"]),

            "gt_distance":
                float(row["gt_distance"]),

            "scan_context_score":
                float(
                    row["scan_context_score"]
                ),

            "best_shift":
                best_shift,

            "raw_shift_deg":
                raw_shift_deg,

            "gt_relative_yaw_deg":
                gt_yaw,

            "sc_yaw_plus_deg":
                sc_yaw_plus,

            "sc_yaw_minus_deg":
                sc_yaw_minus,

            "error_plus_deg":
                error_plus,

            "error_minus_deg":
                error_minus,
        }
    )


result_df = pd.DataFrame(
    records
)


# ============================================================
# Determine GLOBAL sign convention
#
# IMPORTANT:
# Use positive place matches only.
# Do not choose sign independently for every pair.
# ============================================================

positive_df = result_df[
    result_df["is_positive"] == 1
].copy()

if len(positive_df) == 0:
    raise ValueError(
        "No positive candidate pairs available."
    )


median_plus = positive_df[
    "error_plus_deg"
].median()

median_minus = positive_df[
    "error_minus_deg"
].median()

mean_plus = positive_df[
    "error_plus_deg"
].mean()

mean_minus = positive_df[
    "error_minus_deg"
].mean()


print()
print("=" * 80)
print("GLOBAL SIGN CONVENTION TEST")
print("=" * 80)

print(
    f"Number of positive pairs: "
    f"{len(positive_df)}"
)

print()
print(
    f"+best_shift convention:"
)
print(
    f"  Median yaw error: "
    f"{median_plus:.3f}°"
)
print(
    f"  Mean yaw error:   "
    f"{mean_plus:.3f}°"
)

print()
print(
    f"-best_shift convention:"
)
print(
    f"  Median yaw error: "
    f"{median_minus:.3f}°"
)
print(
    f"  Mean yaw error:   "
    f"{mean_minus:.3f}°"
)


# Use median first because it is more robust
if median_plus < median_minus:

    chosen_sign = +1
    chosen_column = "sc_yaw_plus_deg"

elif median_minus < median_plus:

    chosen_sign = -1
    chosen_column = "sc_yaw_minus_deg"

else:

    # Tie-break using mean error
    if mean_plus <= mean_minus:
        chosen_sign = +1
        chosen_column = "sc_yaw_plus_deg"
    else:
        chosen_sign = -1
        chosen_column = "sc_yaw_minus_deg"


print()
print(
    f"Chosen GLOBAL convention: "
    f"{chosen_sign:+d} × best_shift × "
    f"{DEG_PER_SHIFT:.1f}°"
)


# ============================================================
# Final chosen yaw estimate
# ============================================================

result_df[
    "sc_yaw_estimate_deg"
] = result_df[
    chosen_column
]

result_df[
    "yaw_error_deg"
] = result_df.apply(
    lambda row:
        circular_error_deg(
            row[
                "sc_yaw_estimate_deg"
            ],
            row[
                "gt_relative_yaw_deg"
            ],
        ),
    axis=1,
)


positive_final = result_df[
    result_df["is_positive"] == 1
].copy()


# ============================================================
# Summary metrics for positive matches
# ============================================================

mean_error = positive_final[
    "yaw_error_deg"
].mean()

median_error = positive_final[
    "yaw_error_deg"
].median()

p90_error = positive_final[
    "yaw_error_deg"
].quantile(0.90)


within_6 = (
    positive_final[
        "yaw_error_deg"
    ] <= 6.0
).mean()

within_12 = (
    positive_final[
        "yaw_error_deg"
    ] <= 12.0
).mean()

within_18 = (
    positive_final[
        "yaw_error_deg"
    ] <= 18.0
).mean()

within_30 = (
    positive_final[
        "yaw_error_deg"
    ] <= 30.0
).mean()


print()
print("=" * 80)
print("POSITIVE-PAIR YAW VALIDATION")
print("=" * 80)

print(
    f"Mean yaw error:      "
    f"{mean_error:.3f}°"
)

print(
    f"Median yaw error:    "
    f"{median_error:.3f}°"
)

print(
    f"90th percentile:     "
    f"{p90_error:.3f}°"
)

print()

print(
    f"Within 6°:   "
    f"{within_6 * 100:.2f}%"
)

print(
    f"Within 12°:  "
    f"{within_12 * 100:.2f}%"
)

print(
    f"Within 18°:  "
    f"{within_18 * 100:.2f}%"
)

print(
    f"Within 30°:  "
    f"{within_30 * 100:.2f}%"
)


# ============================================================
# Print our six target pairs
# ============================================================

print()
print("=" * 80)
print("TARGET PAIRS")
print("=" * 80)


target_rows = []

for query_frame, candidate_frame in TARGET_PAIRS:

    rows = result_df[
        (
            result_df[
                "query_frame"
            ] == query_frame
        )
        &
        (
            result_df[
                "candidate_frame"
            ] == candidate_frame
        )
    ]

    if len(rows) == 0:

        print(
            f"\nWARNING: Pair "
            f"{query_frame} -> "
            f"{candidate_frame} "
            f"not found."
        )

        continue

    row = rows.iloc[0]

    target_rows.append(
        row
    )

    label = (
        "CORRECT"
        if int(
            row["is_positive"]
        ) == 1
        else "WRONG"
    )

    print()
    print(
        f"{query_frame} -> "
        f"{candidate_frame} "
        f"[{label}]"
    )

    print(
        f"  GT distance:        "
        f"{row['gt_distance']:.3f} m"
    )

    print(
        f"  Best shift:         "
        f"{int(row['best_shift'])}"
    )

    print(
        f"  Raw shift:          "
        f"{row['raw_shift_deg']:.1f}°"
    )

    print(
        f"  GT relative yaw:    "
        f"{row['gt_relative_yaw_deg']:+.2f}°"
    )

    print(
        f"  SC yaw estimate:    "
        f"{row['sc_yaw_estimate_deg']:+.2f}°"
    )

    print(
        f"  Circular yaw error: "
        f"{row['yaw_error_deg']:.2f}°"
    )


# ============================================================
# Save outputs
# ============================================================

all_output = (
    OUTPUT_DIR
    / "all_candidate_yaw_validation.csv"
)

result_df.to_csv(
    all_output,
    index=False
)


if target_rows:

    target_df = pd.DataFrame(
        target_rows
    )

    target_output = (
        OUTPUT_DIR
        / "target_pair_yaw_validation.csv"
    )

    target_df.to_csv(
        target_output,
        index=False
    )

else:

    target_output = None


print()
print("=" * 80)
print("OUTPUT")
print("=" * 80)

print(
    f"All results: "
    f"{all_output}"
)

if target_output is not None:

    print(
        f"Target pairs: "
        f"{target_output}"
    )

print("=" * 80)
