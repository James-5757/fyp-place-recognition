from pathlib import Path
import time

import torch
import open_clip
from PIL import Image
import torch.nn.functional as F


MODEL_NAME = "ViT-B-32-quickgelu"

PRETRAINED = "models/openclip/vit_b32_laion400m_e32.pt"

BEV_DIR = Path("data/kitti_bev/00")

QUERY_FRAME = "001550"
WRONG_FRAME = "003555"
CORRECT_FRAME = "004540"


device = "cuda" if torch.cuda.is_available() else "cpu"

print("=" * 60)
print(f"Device: {device}")
print("=" * 60)


print("Loading OpenCLIP model...")
start = time.time()

model, _, preprocess = open_clip.create_model_and_transforms(
    MODEL_NAME,
    pretrained=PRETRAINED
)

model = model.to(device)
model.eval()

print(f"Model loaded in {time.time() - start:.2f} s")
print(f"Model: {MODEL_NAME}")
print(f"Pretrained: {PRETRAINED}")


def encode_image(frame_id):
    image_path = BEV_DIR / f"{frame_id}.png"

    if not image_path.exists():
        raise FileNotFoundError(f"Missing image: {image_path}")

    image = Image.open(image_path).convert("RGB")
    image = preprocess(image).unsqueeze(0).to(device)

    start = time.time()

    with torch.no_grad():
        embedding = model.encode_image(image)
        embedding = F.normalize(
            embedding,
            p=2,
            dim=-1
        )

    elapsed = time.time() - start

    print(
        f"Encoded {frame_id}: "
        f"shape={tuple(embedding.shape)}, "
        f"time={elapsed:.2f}s"
    )

    return embedding


def cosine_similarity(a, b):
    return torch.sum(a * b, dim=-1).item()


print()
print("Encoding BEV images...")

query_embedding = encode_image(QUERY_FRAME)
wrong_embedding = encode_image(WRONG_FRAME)
correct_embedding = encode_image(CORRECT_FRAME)


sim_wrong = cosine_similarity(
    query_embedding,
    wrong_embedding
)

sim_correct = cosine_similarity(
    query_embedding,
    correct_embedding
)

difference = sim_correct - sim_wrong


print()
print("=" * 60)
print("BEV OpenCLIP sanity check")
print("=" * 60)

print(f"Query:             {QUERY_FRAME}")
print(f"Wrong candidate:   {WRONG_FRAME}")
print(f"Correct candidate: {CORRECT_FRAME}")

print()

print(
    f"Similarity query → wrong:   "
    f"{sim_wrong:.6f}"
)

print(
    f"Similarity query → correct: "
    f"{sim_correct:.6f}"
)

print(
    f"Correct - Wrong:            "
    f"{difference:+.6f}"
)

print()

if sim_correct > sim_wrong:
    print("RESULT: SUCCESS")
    print(
        "The correct candidate receives a higher "
        "BEV visual similarity."
    )
else:
    print("RESULT: NOT CORRECTED")
    print(
        "The wrong candidate still receives an equal "
        "or higher BEV visual similarity."
    )

print("=" * 60)
