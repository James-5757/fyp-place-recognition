# CU-Multi Stage 5 — Visual Viewpoint Analysis (corrected)

## Scope and safeguards

Stage 5 measures how reliable visual evidence is under different viewpoint / heading differences after Scan Context has already generated a high-recall candidate pool. It is NOT a fusion or improvement stage. The frozen Robot1-to-Robot3 protocol from Stage 4 is reused exactly. No SC+visual weighted fusion, VLM, CVTNet, GICP or PGO was run.

Ground truth is strictly evaluation-only: it defines offline `<5 m` positive labels, overlap, headings and evaluation metrics, but does not filter the database, select candidates, or rerank any candidate. RGB is used for visual encoding only and is not passed to Scan Context.

## Bug fixes (corrected from initial Stage 5 run)

1. **Rank -1 bug**: `compute_recall_at_k` previously used `ranks <= k`, which treated -1 (no GT-positive in SC Top-20) as a successful rank because -1 <= k. Fixed to require `(rank >= 1) AND (rank <= k)`. MRR assigns 0 contribution to rank <= 0.
2. **Rescue/regression bug**: Candidate-generation failures (no positive in SC Top-20) were incorrectly counted as visual rescues. Fixed: rescue/regression now computed only within `has_overlap & cand_avail` (candidate-conditioned cohort). The four counts (unchanged_correct + regression + rescue + unchanged_wrong) sum exactly to the cohort size.
3. **End-to-end <= candidate-conditioned**: Added automatic validation assertion that end_to_end R@K <= candidate_conditioned R@K for every method.
4. **Sync-clean dual-robot check**: Previously only checked Robot1 query sync. Fixed: sync-clean now requires both query AND candidate RGB observations to satisfy abs offset <= 75 ms. For Single RGB: query current frame + all 20 candidate current frames. For Mean5/Cross-Max: all 5 query window frames + all 5*20 candidate window frames.

## OpenCLIP configuration

- Model: `ViT-B-32-quickgelu`, pretrained tag: `laion400m_e32` (LAION-400M, epoch 32)
- Checkpoint: `models/openclip/vit_b32_laion400m_e32.pt`
- Embedding dimension: 512, L2-normalized float32
- No training or fine-tuning

## RGB synchronization audit

| Robot | Count | Mean (ms) | P95 (ms) | Max (ms) | >75ms | >100ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| robot1 | 2,000 | 25.1 | 47.2 | 69.1 | 0 | 0 |
| robot3 | 4,180 | 26.7 | 46.9 | 127.9 | 2 | 1 |

## Candidate pool

Frozen SC Top-20 reconstructed from frozen descriptors; Rank-1 verification PASSED. Stage-4 SC metrics reproduced unchanged (R@1=0.993453).

- Valid overlap: 1,833
- Candidate-available: 1,830
- Candidate-miss: 3

## Primary results (corrected)

### Candidate-conditioned (1,830 CANDIDATE_AVAILABLE queries)

| Method | R@1 | R@5 | MRR |
| --- | ---: | ---: | ---: |
| Frozen SC | 0.995082 | 0.997814 | 0.996281 |
| Single RGB | 0.703279 | 0.940984 | 0.803501 |
| RGB5 Mean | 0.683607 | 0.914208 | 0.777015 |
| Cross-Max | 0.715847 | 0.902732 | 0.793528 |

### End-to-end (ALL 1,833 valid queries)

| Method | R@1 | R@5 |
| --- | ---: | ---: |
| Frozen SC | 0.993453 | 0.996181 |
| Single RGB | 0.702128 | 0.939444 |
| RGB5 Mean | 0.682488 | 0.912711 |
| Cross-Max | 0.714675 | 0.901255 |

End-to-end R@K <= candidate-conditioned R@K for all methods (validated).

## Rescue / regression (corrected)

| Method | Unchanged Correct | Regression | Rescue | Unchanged Wrong |
| --- | ---: | ---: | ---: | ---: |
| Single RGB | 1,292 | 538 | 4 | (996) |
| RGB5 Mean | 1,258 | 572 | 2 | (1,030) |
| Cross-Max | 1,318 | 512 | 1 | (970) |

Four counts sum to 1,830 (candidate-conditioned cohort) for each method. Candidate-generation failures: 3, contributing zero visual rescues.

## Sync-quality sensitivity

| Method | All Valid R@1 | Sync-Clean R@1 | Clean Queries | Rule |
| --- | ---: | ---: | ---: | --- |
| Single RGB | 0.702128 | 0.702128 | 1,833 | query + all 20 candidate current frames <= 75ms |
| RGB5 Mean | 0.682488 | 0.682141 | 1,831 | all 5 query + all 5*20 candidate window frames <= 75ms |
| Cross-Max | 0.714675 | 0.714364 | 1,831 | all 5 query + all 5*20 candidate window frames <= 75ms |

## Stage-4 SC failure analysis

12 failures: 9 recoverable (positive in SC Top-20), 3 candidate-generation failures. Visual rescues: Single=3, Mean5=2, CrossMax=1 (all from recoverable failures only).

## Validation

All 21 checks PASS, including: no rank <= 0 counted as success, end-to-end <= candidate-conditioned, four-count sum equals cohort, candidate-generation failures contribute zero rescues, sync-clean checks both robots.
