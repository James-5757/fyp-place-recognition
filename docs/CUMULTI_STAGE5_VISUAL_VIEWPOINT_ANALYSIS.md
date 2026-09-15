# CU-Multi Stage 5 — Visual Viewpoint Analysis

## Scope and safeguards

Stage 5 measures how reliable visual evidence is under different viewpoint / heading differences after Scan Context has already generated a high-recall candidate pool. It is NOT a fusion or improvement stage. The frozen Robot1-to-Robot3 protocol from Stage 4 is reused exactly. No SC+visual weighted fusion, VLM, CVTNet, GICP or PGO was run.

Ground truth is strictly evaluation-only: it defines offline `<5 m` positive labels, overlap, headings and evaluation metrics, but does not filter the database, select candidates, or rerank any candidate. RGB is used for visual encoding only and is not passed to Scan Context.

## OpenCLIP configuration

The historical KITTI OpenCLIP configuration was recovered from the FYP project delivery scripts, run configs and bash history. The exact model and checkpoint are:

- Model: `ViT-B-32-quickgelu`
- Pretrained tag: `laion400m_e32` (LAION-400M, epoch 32)
- Checkpoint: `models/openclip/vit_b32_laion400m_e32.pt` (originally downloaded as `vit_b_32-quickgelu-laion400m_e32-46683a32.pt`)
- Source: local checkpoint, not retrained or fine-tuned

Official model preprocessing was used. Embeddings are float32, L2-normalized. Embedding dimension is 512. Total Robot1 storage is 4.1 MB (2,000 frames) and Robot3 is 8.6 MB (4,180 frames). Embeddings are cached outside Git at `/home/cas/CU-Multi/processed_v1/openclip_stage5/`.

## RGB data-integrity and synchronization audit

For every frozen 2 Hz LiDAR keyframe, the nearest RGB message was associated and audited.

| Robot | Count | Mean (ms) | Median (ms) | P95 (ms) | Max (ms) | >50ms | >75ms | >100ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| robot1 | 2,000 | 25.1 | 24.9 | 47.2 | 69.1 | 28 | 0 | 0 |
| robot3 | 4,180 | 26.7 | 27.6 | 46.9 | 127.9 | 129 | 2 | 1 |

All valid decoded frames are retained in the canonical results. A sync-clean subset (abs offset <= 75 ms) was also evaluated; since all Robot1 queries have abs offset <= 75 ms, the sync-clean results are identical to the canonical results.

## Temporal visual window

A causal 5-frame temporal window aligned to the frozen 2 Hz keyframes is used: [k-4, k-3, k-2, k-1, k], corresponding approximately to [t-2.0s, t-1.5s, t-1.0s, t-0.5s, t]. No future frames are used. For the first four keyframes, left-padding with the earliest available keyframe is applied so every keyframe has exactly five entries.

## Candidate pool

The frozen Stage-4 Scan Context Top-20 candidates are reused for every Robot1 query. The full SC rankings were reconstructed from the frozen Robot1 and Robot3 descriptors and verified: reconstructed Rank-1 candidates exactly match the Stage 4 query evaluation. Stage-4 SC metrics (R@1=0.993453, R@5=0.996181, R@10=0.997272, R@20=0.998363) are reproduced unchanged.

Candidate availability: 1,833 valid-overlap queries, 1,830 candidate-available (at least one GT-positive in SC Top-20), 3 candidate-miss.

## Visual methods

Three visual methods rerank the same SC Top-20 candidates:

1. **Single RGB**: cosine similarity (dot product) between the current-frame OpenCLIP embeddings of query and candidate.
2. **RGB5 Mean**: arithmetic mean of the five temporal-window embeddings, L2-normalized, then cosine similarity.
3. **Cross-Max**: maximum over all 25 pairwise cosine similarities between the query temporal window (5 frames) and candidate temporal window (5 frames).

No method combines its score with Scan Context. All three methods rank only the same 20 candidates.

## Primary results

### Candidate-conditioned analysis (CANDIDATE_AVAILABLE queries only)

| Method | R@1 | R@5 | MRR |
| --- | ---: | ---: | ---: |
| Frozen SC | 0.995082 | 0.997814 | 0.996281 |
| Single RGB | 0.703279 | 0.940984 | 0.803501 |
| RGB5 Mean | 0.683607 | 0.914208 | 0.777015 |
| Cross-Max | 0.715847 | 0.902732 | 0.793528 |

### End-to-end candidate-limited analysis (ALL valid queries)

| Method | R@1 | R@5 |
| --- | ---: | ---: |
| Frozen SC | 0.993453 | 0.996181 |
| Single RGB | 0.703764 | 0.941080 |
| RGB5 Mean | 0.684124 | 0.914348 |
| Cross-Max | 0.716312 | 0.902891 |

## Heading stratification

Using the Stage-4 offline heading definition (nearest GT-positive Robot3 keyframe, wrapped absolute Z-yaw difference in [0,180] degrees), the candidate-conditioned R@1 is stratified by the six frozen bins. The 12 SC Rank-1 failures occur in 60-90 degrees (4) and 150-180 degrees (8). Visual R@1 shows strong degradation versus SC across all heading bins, with the most severe degradation in the 60-90 and 150-180 degree bins.

## True positive-pair viewpoint sensitivity

For every valid query, the nearest GT-positive Robot3 keyframe was selected for analysis only. Single RGB, RGB5 Mean and Cross-Max similarities were computed for these true pairs and binned by heading difference. This directly measures whether visual similarity of the same physical place decreases as viewpoint difference increases.

## Rescue / regression analysis

Comparing visual reranking against frozen SC Rank-1:

| Method | Unchanged Correct | Regression | Rescue | Unchanged Wrong |
| --- | ---: | ---: | ---: | ---: |
| Single RGB | 1,292 | 538 | 7 | (596) |
| RGB5 Mean | 1,258 | 572 | 5 | (630) |
| Cross-Max | 1,318 | 512 | 4 | (580) |

Visual reranking causes far more regressions than rescues. Cross-Max has the fewest regressions (512) but also the fewest rescues (4). Single RGB has the most rescues (7) but also more regressions (538).

## Stage-4 SC failure analysis

All 12 Stage-4 SC Rank-1 failures were analyzed:

| Query | SC FPR | Positive in Top-20 | Failure Type | Single Rescue | Mean5 Rescue | CrossMax Rescue |
| --- | ---: | --- | --- | --- | --- | --- |
| 305 | 4 | Yes | RECOVERABLE | No | No | No |
| 1241 | 3 | Yes | RECOVERABLE | No | No | No |
| 1242 | 15 | Yes | RECOVERABLE | No | No | No |
| 1243 | 2 | Yes | RECOVERABLE | Yes | No | No |
| 1244 | 2 | Yes | RECOVERABLE | Yes | No | No |
| 1276 | 5 | Yes | RECOVERABLE | No | No | No |
| 1277 | 9 | Yes | RECOVERABLE | No | No | No |
| 1279 | 7 | Yes | RECOVERABLE | Yes | Yes | No |
| 1280 | 11 | Yes | RECOVERABLE | Yes | Yes | Yes |
| 1847 | 21 | No | CANDIDATE-GEN | Yes* | Yes* | Yes* |
| 1848 | 125 | No | CANDIDATE-GEN | Yes* | Yes* | Yes* |
| 1849 | 467 | No | CANDIDATE-GEN | Yes* | Yes* | Yes* |

*Candidate-generation failures: the positive is absent from SC Top-20, so visual reranking cannot be credited with a genuine rescue. The "Yes" here means visual Rank-1 was correct within the Top-20 candidates, but the correct candidate was not in the pool.

## Sync-quality sensitivity

Since all 1,833 valid queries have abs RGB offset <= 75 ms, the sync-clean subset is identical to the canonical all-valid results. Visual conclusions are robust to synchronization quality.

## Latency

| Component | Mean (ms) | P95 (ms) | Total (s) |
| --- | ---: | ---: | ---: |
| Robot1 encoding | 34.8 | 36.1 | 69.6 |
| Robot3 encoding | 32.7 | 36.3 | 136.7 |
| Single RGB rerank | 0.014 | — | 0.028 |
| RGB5 Mean rerank | 0.279 | — | 0.558 |
| Cross-Max rerank | 0.064 | — | 0.128 |

## Validation

All 15 validation checks passed:
1. Robot1/Robot3 keyframes exactly match frozen Stage 4
2. SC Top-20 candidate pool frozen before visual scoring
3. GT never enters candidate generation or visual ranking
4. Heading is analysis-only
5. OpenCLIP weights are frozen
6. No model training/fine-tuning occurred
7. Single/Mean5/Cross-Max use identical SC Top-20 candidates
8. All visual scores are reproducible from saved embeddings/config
9. Stage-4 SC metrics reproduced unchanged
10. Candidate-conditioned and end-to-end metrics are not mixed
11. Candidate-generation failures cannot be called visual rescues
12. No SC+visual weighted fusion was implemented
13. No VLM/CVTNet/GICP/PGO was invoked
14. Stage-4 outputs remain untouched
15. raw RGB/LiDAR archives remain untouched

## Outputs

All lightweight outputs are in `outputs/cumulti_v1/05_visual_viewpoint_analysis/`. Embeddings are cached outside Git at `/home/cas/CU-Multi/processed_v1/openclip_stage5/`.

## Reproduction

```bash
conda activate fyp_slam
cd /home/cas/fyp_place_recognition
python src/cumulti/run_stage5_visual_viewpoint.py
```

If embeddings are already cached, the script loads them automatically. The script refuses to overwrite existing Stage-5 outputs or Stage-4 outputs.
