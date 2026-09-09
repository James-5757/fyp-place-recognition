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

CSV_PATH = (
    "outputs/"
    "formal_split2500_gap100_step5_thr5_candidates.csv"
)

BEV_DIR = Path("data/kitti_bev/00")

OUTPUT_DIR = Path(
    "outputs/exp3c_corrected_yaw"
)

OUTPUT_DIR.mkdir(
    parents=True,
    exist_ok=True,
)

QUERY_FRAMES = [
    1545,
    1550,
    1560,
]

NUM_SECTORS = 60

DEG_PER_SHIFT = (
    360.0 / NUM_SECTORS
)


# ============================================================
# Device
# ============================================================

device = (
    "cuda"
    if torch.cuda.is_available()
    else "cpu"
)

print("=" * 80)
print(
    "Experiment 3B: "
    "SC-guided BEV Alignment "
    "for Three Reranking Cases"
)
print("=" * 80)

print(f"Device: {device}")


# ============================================================
# Load OpenCLIP
# ============================================================

print("\nLoading OpenCLIP...")

model, _, preprocess = (
    open_clip.create_model_and_transforms(
        MODEL_NAME,
        pretrained=PRETRAINED,
    )
)

model = model.to(device)
model.eval()

print("OpenCLIP loaded.")


# ============================================================
# Load candidate CSV
# ============================================================

df = pd.read_csv(CSV_PATH)


# ============================================================
# Helper functions
# ============================================================

def load_bev(frame_id):

    path = (
        BEV_DIR
        / f"{int(frame_id):06d}.png"
    )

    if not path.exists():
        raise FileNotFoundError(
            f"Missing BEV: {path}"
        )

    return Image.open(
        path
    ).convert("RGB")


def encode_image(image):

    tensor = preprocess(
        image
    ).unsqueeze(0).to(device)

    with torch.no_grad():

        embedding = (
            model.encode_image(
                tensor
            )
        )

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


def shift_to_pil_angle(
    best_shift
):

    return (
        float(best_shift)
        * DEG_PER_SHIFT
    )

def find_reranking_pair(
    query_frame
):

    query_rows = df[
        df["query_frame"]
        == query_frame
    ].copy()

    query_rows = (
        query_rows.sort_values(
            "rank"
        )
    )

    if len(query_rows) == 0:

        raise ValueError(
            f"No candidates found "
            f"for query {query_frame}"
        )

    # ----------------------------------------
    # Wrong candidate:
    # original Scan Context Rank-1
    # ----------------------------------------

    rank1_rows = query_rows[
        query_rows["rank"] == 1
    ]

    if len(rank1_rows) != 1:

        raise ValueError(
            f"Unexpected Rank-1 rows "
            f"for query {query_frame}"
        )

    wrong_row = (
        rank1_rows.iloc[0]
    )

    if int(
        wrong_row["is_positive"]
    ) != 0:

        raise ValueError(
            f"Query {query_frame}: "
            f"Rank-1 is already positive. "
            f"Not a reranking opportunity."
        )

    # ----------------------------------------
    # Correct candidate:
    # highest-ranked positive candidate
    # ----------------------------------------

    positive_rows = query_rows[
        query_rows["is_positive"]
        == 1
    ].sort_values("rank")

    if len(positive_rows) == 0:

        raise ValueError(
            f"Query {query_frame}: "
            f"no positive candidate found."
        )

    correct_row = (
        positive_rows.iloc[0]
    )

    return (
        wrong_row,
        correct_row,
    )


def evaluate_candidate(
    query_embedding,
    candidate_row,
    label,
    query_frame,
):

    candidate_frame = int(
        candidate_row[
            "candidate_frame"
        ]
    )

    candidate_image = load_bev(
        candidate_frame
    )

    # ----------------------------------------
    # Raw similarity
    # ----------------------------------------

    raw_embedding = encode_image(
        candidate_image
    )

    raw_similarity = (
        cosine_similarity(
            query_embedding,
            raw_embedding,
        )
    )

    # ----------------------------------------
    # Scan Context-guided yaw alignment
    # ----------------------------------------

    best_shift = int(
        candidate_row[
            "best_shift"
        ]
    )

    pil_angle = (
        shift_to_pil_angle(
            best_shift
        )
    )

    aligned_image = (
        candidate_image.rotate(
            pil_angle,
            resample=(
                Image.Resampling.BILINEAR
            ),
            expand=False,
            fillcolor=(0, 0, 0),
        )
    )

    aligned_embedding = (
        encode_image(
            aligned_image
        )
    )

    aligned_similarity = (
        cosine_similarity(
            query_embedding,
            aligned_embedding,
        )
    )

    # Save aligned image
    aligned_path = (
        OUTPUT_DIR
        / (
            f"q{query_frame:06d}_"
            f"{label.lower()}_"
            f"{candidate_frame:06d}_"
            f"aligned.png"
        )
    )

    aligned_image.save(
        aligned_path
    )

    return {
        "candidate_frame":
            candidate_frame,

        "rank":
            int(
                candidate_row[
                    "rank"
                ]
            ),

        "scan_context_score":
            float(
                candidate_row[
                    "scan_context_score"
                ]
            ),

        "best_shift":
            best_shift,

        "sc_yaw_deg":
            (
                best_shift
                * DEG_PER_SHIFT
            ),

        "pil_angle_deg":
            pil_angle,

        "gt_distance":
            float(
                candidate_row[
                    "gt_distance"
                ]
            ),

        "is_positive":
            int(
                candidate_row[
                    "is_positive"
                ]
            ),

        "raw_similarity":
            raw_similarity,

        "aligned_similarity":
            aligned_similarity,

        "similarity_change":
            (
                aligned_similarity
                - raw_similarity
            ),
    }


# ============================================================
# Main experiment
# ============================================================

summary_rows = []


for query_frame in QUERY_FRAMES:

    print()
    print("=" * 80)
    print(
        f"QUERY {query_frame}"
    )
    print("=" * 80)

    # ----------------------------------------
    # Find wrong + correct candidates
    # ----------------------------------------

    wrong_row, correct_row = (
        find_reranking_pair(
            query_frame
        )
    )

    wrong_frame = int(
        wrong_row[
            "candidate_frame"
        ]
    )

    correct_frame = int(
        correct_row[
            "candidate_frame"
        ]
    )

    print(
        f"Wrong Rank-1: "
        f"{wrong_frame}"
    )

    print(
        f"Correct candidate: "
        f"{correct_frame} "
        f"(Rank "
        f"{int(correct_row['rank'])})"
    )

    # ----------------------------------------
    # Query embedding
    # ----------------------------------------

    query_image = load_bev(
        query_frame
    )

    query_embedding = (
        encode_image(
            query_image
        )
    )

    # ----------------------------------------
    # Evaluate both candidates
    # ----------------------------------------

    wrong = evaluate_candidate(
        query_embedding,
        wrong_row,
        "WRONG",
        query_frame,
    )

    correct = evaluate_candidate(
        query_embedding,
        correct_row,
        "CORRECT",
        query_frame,
    )

    # ----------------------------------------
    # Margins
    # ----------------------------------------

    raw_gap = (
        correct[
            "raw_similarity"
        ]
        -
        wrong[
            "raw_similarity"
        ]
    )

    aligned_gap = (
        correct[
            "aligned_similarity"
        ]
        -
        wrong[
            "aligned_similarity"
        ]
    )

    margin_improvement = (
        aligned_gap
        - raw_gap
    )

    corrected = (
        aligned_gap > 0
    )

    # ----------------------------------------
    # Print detailed result
    # ----------------------------------------

    print()
    print("WRONG CANDIDATE")

    print(
        f"  Frame:              "
        f"{wrong['candidate_frame']}"
    )

    print(
        f"  SC Rank:            "
        f"{wrong['rank']}"
    )

    print(
        f"  SC score:           "
        f"{wrong['scan_context_score']:.6f}"
    )

    print(
        f"  Best shift:         "
        f"{wrong['best_shift']}"
    )

    print(
        f"  PIL rotation:       "
        f"{wrong['pil_angle_deg']:.1f}°"
    )

    print(
        f"  GT distance:        "
        f"{wrong['gt_distance']:.3f} m"
    )

    print(
        f"  Raw similarity:     "
        f"{wrong['raw_similarity']:.6f}"
    )

    print(
        f"  Aligned similarity: "
        f"{wrong['aligned_similarity']:.6f}"
    )

    print(
        f"  Change:             "
        f"{wrong['similarity_change']:+.6f}"
    )

    print()
    print("CORRECT CANDIDATE")

    print(
        f"  Frame:              "
        f"{correct['candidate_frame']}"
    )

    print(
        f"  SC Rank:            "
        f"{correct['rank']}"
    )

    print(
        f"  SC score:           "
        f"{correct['scan_context_score']:.6f}"
    )

    print(
        f"  Best shift:         "
        f"{correct['best_shift']}"
    )

    print(
        f"  PIL rotation:       "
        f"{correct['pil_angle_deg']:.1f}°"
    )

    print(
        f"  GT distance:        "
        f"{correct['gt_distance']:.3f} m"
    )

    print(
        f"  Raw similarity:     "
        f"{correct['raw_similarity']:.6f}"
    )

    print(
        f"  Aligned similarity: "
        f"{correct['aligned_similarity']:.6f}"
    )

    print(
        f"  Change:             "
        f"{correct['similarity_change']:+.6f}"
    )

    print()
    print("PAIR RESULT")

    print(
        f"  Raw correct-wrong gap:     "
        f"{raw_gap:+.6f}"
    )

    print(
        f"  Aligned correct-wrong gap: "
        f"{aligned_gap:+.6f}"
    )

    print(
        f"  Margin improvement:        "
        f"{margin_improvement:+.6f}"
    )

    print(
        f"  Corrected:                 "
        f"{corrected}"
    )

    # ----------------------------------------
    # Save summary row
    # ----------------------------------------

    summary_rows.append(
        {
            "query_frame":
                query_frame,

            "wrong_candidate":
                wrong[
                    "candidate_frame"
                ],

            "correct_candidate":
                correct[
                    "candidate_frame"
                ],

            "correct_original_rank":
                correct[
                    "rank"
                ],

            "wrong_sc_score":
                wrong[
                    "scan_context_score"
                ],

            "correct_sc_score":
                correct[
                    "scan_context_score"
                ],

            "wrong_best_shift":
                wrong[
                    "best_shift"
                ],

            "correct_best_shift":
                correct[
                    "best_shift"
                ],

            "wrong_raw_similarity":
                wrong[
                    "raw_similarity"
                ],

            "correct_raw_similarity":
                correct[
                    "raw_similarity"
                ],

            "wrong_aligned_similarity":
                wrong[
                    "aligned_similarity"
                ],

            "correct_aligned_similarity":
                correct[
                    "aligned_similarity"
                ],

            "raw_gap":
                raw_gap,

            "aligned_gap":
                aligned_gap,

            "margin_improvement":
                margin_improvement,

            "corrected":
                corrected,
        }
    )


# ============================================================
# Final summary
# ============================================================

summary_df = pd.DataFrame(
    summary_rows
)

summary_path = (
    OUTPUT_DIR
    / "three_case_summary.csv"
)

summary_df.to_csv(
    summary_path,
    index=False,
)


print()
print("=" * 80)
print("FINAL THREE-CASE SUMMARY")
print("=" * 80)

display_columns = [
    "query_frame",
    "wrong_candidate",
    "correct_candidate",
    "correct_original_rank",
    "raw_gap",
    "aligned_gap",
    "margin_improvement",
    "corrected",
]

print(
    summary_df[
        display_columns
    ].to_string(
        index=False
    )
)


num_corrected = int(
    summary_df[
        "corrected"
    ].sum()
)

num_improved = int(
    (
        summary_df[
            "margin_improvement"
        ]
        > 0
    ).sum()
)

print()
print(
    f"Cases with improved margin: "
    f"{num_improved}/"
    f"{len(summary_df)}"
)

print(
    f"Cases actually corrected:   "
    f"{num_corrected}/"
    f"{len(summary_df)}"
)

print()
print(
    f"Summary saved to: "
    f"{summary_path}"
)

print("=" * 80)
