from pathlib import Path
import time

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

QUERY_FRAME = "001550"
WRONG_FRAME = "003555"
CORRECT_FRAME = "004540"

# Coarse rotation search
ANGLES = list(range(0, 360, 6))


# ============================================================
# Device
# ============================================================

device = "cuda" if torch.cuda.is_available() else "cpu"

print("=" * 70)
print("Experiment 2: Rotation-Robust BEV Similarity")
print("=" * 70)
print(f"Device: {device}")


# ============================================================
# Load model
# ============================================================

print("\nLoading OpenCLIP model...")

start = time.time()

model, _, preprocess = open_clip.create_model_and_transforms(
    MODEL_NAME,
    pretrained=PRETRAINED,
)

model = model.to(device)
model.eval()

print(f"Model loaded in {time.time() - start:.2f}s")
print(f"Model: {MODEL_NAME}")
print(f"Checkpoint: {PRETRAINED}")


# ============================================================
# Functions
# ============================================================

def load_bev(frame_id):
    path = BEV_DIR / f"{frame_id}.png"

    if not path.exists():
        raise FileNotFoundError(
            f"BEV image not found: {path}"
        )

    return Image.open(path).convert("RGB")


def encode_pil_image(image):
    """
    Encode one PIL image using OpenCLIP.
    """

    tensor = preprocess(image)
    tensor = tensor.unsqueeze(0).to(device)

    with torch.no_grad():
        embedding = model.encode_image(tensor)

    embedding = F.normalize(
        embedding,
        p=2,
        dim=-1
    )

    return embedding


def cosine_similarity(a, b):
    return torch.sum(a * b, dim=-1).item()


def rotation_search(
    query_embedding,
    candidate_image,
    candidate_frame,
):
    """
    Rotate the candidate BEV through all specified angles
    and return cosine similarity at each angle.
    """

    results = []

    print()
    print("-" * 70)
    print(f"Candidate: {candidate_frame}")
    print("-" * 70)

    for angle in ANGLES:

        # Keep original canvas size.
        # Black fill matches the natural background of our BEV.
        rotated = candidate_image.rotate(
            angle,
            resample=Image.Resampling.BILINEAR,
            expand=False,
            fillcolor=(0, 0, 0),
        )

        candidate_embedding = encode_pil_image(rotated)

        similarity = cosine_similarity(
            query_embedding,
            candidate_embedding,
        )

        results.append(
            {
                "angle": angle,
                "similarity": similarity,
            }
        )

        print(
            f"Angle {angle:3d}° "
            f"→ similarity = {similarity:.6f}"
        )

    best = max(
        results,
        key=lambda x: x["similarity"]
    )

    print()
    print(
        f"Best angle:      {best['angle']}°"
    )
    print(
        f"Best similarity: {best['similarity']:.6f}"
    )

    return results, best


# ============================================================
# Load images
# ============================================================

query_image = load_bev(QUERY_FRAME)
wrong_image = load_bev(WRONG_FRAME)
correct_image = load_bev(CORRECT_FRAME)


# ============================================================
# Query embedding
# ============================================================

print("\nEncoding query BEV...")

query_embedding = encode_pil_image(query_image)

print(
    f"Query {QUERY_FRAME} embedding shape: "
    f"{tuple(query_embedding.shape)}"
)


# ============================================================
# Raw similarities at 0 degrees
# ============================================================

wrong_raw_embedding = encode_pil_image(wrong_image)
correct_raw_embedding = encode_pil_image(correct_image)

raw_wrong = cosine_similarity(
    query_embedding,
    wrong_raw_embedding,
)

raw_correct = cosine_similarity(
    query_embedding,
    correct_raw_embedding,
)


print()
print("=" * 70)
print("Experiment 1 reference: Raw BEV")
print("=" * 70)

print(
    f"Query → Wrong   ({WRONG_FRAME}): "
    f"{raw_wrong:.6f}"
)

print(
    f"Query → Correct ({CORRECT_FRAME}): "
    f"{raw_correct:.6f}"
)

print(
    f"Correct - Wrong: "
    f"{raw_correct - raw_wrong:+.6f}"
)


# ============================================================
# Rotation search
# ============================================================

wrong_results, wrong_best = rotation_search(
    query_embedding,
    wrong_image,
    WRONG_FRAME,
)

correct_results, correct_best = rotation_search(
    query_embedding,
    correct_image,
    CORRECT_FRAME,
)


# ============================================================
# Final comparison
# ============================================================

best_wrong = wrong_best["similarity"]
best_correct = correct_best["similarity"]

difference = best_correct - best_wrong


print()
print("=" * 70)
print("ROTATION-ROBUST RESULT")
print("=" * 70)

print(f"Query frame: {QUERY_FRAME}")

print()
print("Wrong candidate:")
print(f"  Frame:           {WRONG_FRAME}")
print(f"  Raw similarity:  {raw_wrong:.6f}")
print(f"  Best angle:      {wrong_best['angle']}°")
print(f"  Best similarity: {best_wrong:.6f}")

print()
print("Correct candidate:")
print(f"  Frame:           {CORRECT_FRAME}")
print(f"  Raw similarity:  {raw_correct:.6f}")
print(f"  Best angle:      {correct_best['angle']}°")
print(f"  Best similarity: {best_correct:.6f}")

print()
print(
    f"Correct - Wrong after rotation search: "
    f"{difference:+.6f}"
)

print()

if best_correct > best_wrong:

    print("RESULT: CORRECTED BY ROTATION")

    print(
        "After rotation search, the correct candidate "
        "receives a higher BEV similarity."
    )

elif best_correct == best_wrong:

    print("RESULT: TIE")

else:

    print("RESULT: STILL NOT CORRECTED")

    print(
        "Even after rotation search, the wrong candidate "
        "still receives a higher BEV similarity."
    )

print("=" * 70)
