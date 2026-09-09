from pathlib import Path

import pandas as pd
import torch
import torch.nn.functional as F
import open_clip
from PIL import Image


# ============================================================
# Configuration
# ============================================================

MODEL_NAME = "ViT-B-32-quickgelu"

PRETRAINED = (
    "models/openclip/"
    "vit_b32_laion400m_e32.pt"
)

BEV_DIR = Path("data/kitti_bev/00")

CSV_PATH = (
    "outputs/"
    "formal_split2500_gap100_step5_thr5_candidates.csv"
)

OUTPUT_DIR = Path("outputs/exp3_sc_yaw")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

QUERY_FRAME = 1550
WRONG_FRAME = 3555
CORRECT_FRAME = 4540

NUM_SECTORS = 60
DEG_PER_SHIFT = 360.0 / NUM_SECTORS


# ============================================================
# Device
# ============================================================

device = "cuda" if torch.cuda.is_available() else "cpu"

print("=" * 72)
print("Experiment 3: Scan Context-guided BEV Yaw Alignment")
print("=" * 72)
print(f"Device: {device}")


# ============================================================
# Load OpenCLIP
# ============================================================

print("\nLoading OpenCLIP...")

model, _, preprocess = open_clip.create_model_and_transforms(
    MODEL_NAME,
    pretrained=PRETRAINED,
)

model = model.to(device)
model.eval()

print("OpenCLIP loaded.")


# ============================================================
# Helper functions
# ============================================================

def load_bev(frame_id):
    path = BEV_DIR / f"{frame_id:06d}.png"

    if not path.exists():
        raise FileNotFoundError(path)

    return Image.open(path).convert("RGB")


def encode_image(image):

    tensor = preprocess(image)
    tensor = tensor.unsqueeze(0).to(device)

    with torch.no_grad():
        embedding = model.encode_image(tensor)

    embedding = F.normalize(
        embedding,
        p=2,
        dim=-1,
    )

    return embedding


def cosine_similarity(a, b):

    return torch.sum(
        a * b,
        dim=-1,
    ).item()


def shift_to_pil_angle(best_shift):
    """
    Convert Scan Context sector shift to PIL rotation.

    Scan Context:
        +shift = positive sector roll

    BEV image:
        +x points upward
        +y points right

    Therefore the corresponding PIL rotation
    uses the opposite sign.
    """

    return -best_shift * DEG_PER_SHIFT


# ============================================================
# Read candidate CSV
# ============================================================

df = pd.read_csv(CSV_PATH)


def get_candidate_row(candidate_frame):

    rows = df[
        (df["query_frame"] == QUERY_FRAME)
        &
        (df["candidate_frame"] == candidate_frame)
    ]

    if len(rows) != 1:
        raise ValueError(
            f"Expected exactly one row for "
            f"{QUERY_FRAME} -> {candidate_frame}, "
            f"found {len(rows)}"
        )

    return rows.iloc[0]


wrong_row = get_candidate_row(WRONG_FRAME)
correct_row = get_candidate_row(CORRECT_FRAME)


# ============================================================
# Load images
# ============================================================

query_image = load_bev(QUERY_FRAME)
wrong_image = load_bev(WRONG_FRAME)
correct_image = load_bev(CORRECT_FRAME)


# ============================================================
# Query embedding
# ============================================================

query_embedding = encode_image(query_image)


# ============================================================
# Evaluate one candidate
# ============================================================

def evaluate_candidate(
    candidate_image,
    candidate_frame,
    row,
    label,
):

    best_shift = int(row["best_shift"])

    sc_score = float(
        row["scan_context_score"]
    )

    gt_distance = float(
        row["gt_distance"]
    )

    is_positive = int(
        row["is_positive"]
    )

    pil_angle = shift_to_pil_angle(
        best_shift
    )

    # Raw BEV
    raw_embedding = encode_image(
        candidate_image
    )

    raw_similarity = cosine_similarity(
        query_embedding,
        raw_embedding,
    )

    # Scan Context-guided yaw alignment
    aligned_image = candidate_image.rotate(
        pil_angle,
        resample=Image.Resampling.BILINEAR,
        expand=False,
        fillcolor=(0, 0, 0),
    )

    aligned_embedding = encode_image(
        aligned_image
    )

    aligned_similarity = cosine_similarity(
        query_embedding,
        aligned_embedding,
    )

    # Save aligned BEV for visual inspection
    output_path = (
        OUTPUT_DIR
        / f"{QUERY_FRAME:06d}_to_"
          f"{candidate_frame:06d}_"
          f"aligned.png"
    )

    aligned_image.save(output_path)

    result = {
        "label": label,
        "candidate_frame": candidate_frame,
        "rank": int(row["rank"]),
        "sc_score": sc_score,
        "best_shift": best_shift,
        "yaw_deg": best_shift * DEG_PER_SHIFT,
        "pil_angle": pil_angle,
        "gt_distance": gt_distance,
        "is_positive": is_positive,
        "raw_similarity": raw_similarity,
        "aligned_similarity": aligned_similarity,
        "improvement": (
            aligned_similarity
            - raw_similarity
        ),
    }

    return result


wrong = evaluate_candidate(
    wrong_image,
    WRONG_FRAME,
    wrong_row,
    "WRONG",
)

correct = evaluate_candidate(
    correct_image,
    CORRECT_FRAME,
    correct_row,
    "CORRECT",
)


# ============================================================
# Print results
# ============================================================

def print_result(r):

    print()
    print("-" * 72)
    print(r["label"])
    print("-" * 72)

    print(
        f"Candidate frame:     "
        f"{r['candidate_frame']:06d}"
    )

    print(
        f"Scan Context rank:   "
        f"{r['rank']}"
    )

    print(
        f"Scan Context score:  "
        f"{r['sc_score']:.6f}"
    )

    print(
        f"Best shift:          "
        f"{r['best_shift']}"
    )

    print(
        f"SC yaw magnitude:    "
        f"{r['yaw_deg']:.1f}°"
    )

    print(
        f"PIL rotation used:   "
        f"{r['pil_angle']:.1f}°"
    )

    print(
        f"GT distance:         "
        f"{r['gt_distance']:.3f} m"
    )

    print(
        f"Positive:            "
        f"{r['is_positive']}"
    )

    print(
        f"Raw CLIP similarity: "
        f"{r['raw_similarity']:.6f}"
    )

    print(
        f"Aligned similarity:  "
        f"{r['aligned_similarity']:.6f}"
    )

    print(
        f"Improvement:         "
        f"{r['improvement']:+.6f}"
    )


print_result(wrong)
print_result(correct)


# ============================================================
# Final comparison
# ============================================================

raw_gap = (
    correct["raw_similarity"]
    - wrong["raw_similarity"]
)

aligned_gap = (
    correct["aligned_similarity"]
    - wrong["aligned_similarity"]
)


print()
print("=" * 72)
print("FINAL COMPARISON")
print("=" * 72)

print(
    f"Raw correct - wrong:       "
    f"{raw_gap:+.6f}"
)

print(
    f"Aligned correct - wrong:   "
    f"{aligned_gap:+.6f}"
)

print()

if aligned_gap > 0:

    print(
        "RESULT: SC-GUIDED ALIGNMENT CORRECTS THE RANKING"
    )

elif aligned_gap == 0:

    print(
        "RESULT: SC-GUIDED ALIGNMENT PRODUCES A TIE"
    )

else:

    print(
        "RESULT: SC-GUIDED ALIGNMENT DOES NOT "
        "CORRECT THE RANKING"
    )

print("=" * 72)
