# CU-Multi Stage5B — independent visual full-database retrieval

PASS. Historical audit found no validated independent all4180 R1-to-R3 visual benchmark in the accessible evidence. Stage5/7.5 and KITTI scripts used inherited SC candidate pools. Source/artifact evidence and availability limitations are recorded in HISTORICAL_AUDIT.md and historical_audit.csv. Existing RGB-D appearance-registration studies use a different dataset index/protocol and are not this benchmark.

## Protocol

Frozen Robot1 queries:2000; Robot3 database:4180; timestamp-sampled2Hz. All2000 queries are ranked against all4180 database keyframes before GT evaluation. Metrics use the exact1833 valid-overlap queries and the exact482 Stage7.5 same-view query IDs. The167 no-overlap queries are scored but excluded from recall/MRR denominators. Positive labels are strictly horizontal distance<5.0m and match every frozen Stage4 positive-pair CSV entry. Same-view means nearest-GT-positive wrapped heading<=10degrees; it selects the offline query stratum and never restricts database candidates.

The complete Stage5 normalized float32 embedding cache was reused: ViT-B-32-quickgelu / laion400m_e32 /512 dimensions; no training, image decoding or encoding. Both cache sidecars match every keyframe ID and synchronized RGB timestamp; no zero/nonfinite rows exist. Original checkpoint hash receipts are unavailable, so model provenance rests on recorded cache metadata, source configuration, complete identity checks and exact reproduction of historical pooled metrics. Current input hashes authenticate the reused cache for this experiment; they are not retroactive original encoding receipts.

Single RGB uses current-frame cosine. RGB5 Mean averages each causal five-keyframe window then normalizes before cosine. Cross-Max takes the maximum of all25 pairwise cosines between the causal windows. Left padding is exactly max(0,k-4+j). Efficient Cross-Max is a separable max over a shared2000x4180 cosine matrix; it does not allocate a query/database/window/feature tensor. No SC score enters visual scores or full visual ordering. All full methods use descending score and increasing database ID for exact ties.

Canonical SC scoring is imported unchanged from frozen Stage5 code because full score matrices were no longer archived. Reconstructed SC first-positive ranks and all2000 Rank1 IDs match Stage4 exactly. The canonical SC numerical metrics are unchanged.

## Overall:1833 valid queries

| method | R@1 | R@5 | R@10 | R@20 | MRR |
|---|---|---|---|---|---|
| SC | 0.993453 | 0.996181 | 0.997272 | 0.998363 | 0.994682 |
| Single RGB | 0.181669 | 0.256410 | 0.299509 | 0.345881 | 0.224026 |
| RGB5 Mean | 0.202400 | 0.236770 | 0.265685 | 0.310420 | 0.226750 |
| Cross-Max | 0.181124 | 0.196945 | 0.236770 | 0.282597 | 0.201424 |

## Same-view:exact482 queries

| method | R@1 | R@5 | R@10 | R@20 | MRR |
|---|---|---|---|---|---|
| SC | 1.000000 | 1.000000 | 1.000000 | 1.000000 | 1.000000 |
| Single RGB | 0.537344 | 0.676349 | 0.736515 | 0.775934 | 0.603940 |
| RGB5 Mean | 0.578838 | 0.645228 | 0.692946 | 0.746888 | 0.615009 |
| Cross-Max | 0.529046 | 0.553942 | 0.628631 | 0.692946 | 0.557298 |

## Historical SC Top20 comparison

These are end-to-end denominators, including the three valid queries without a positive in SC Top20. They must not be mixed with the1830-query candidate-conditioned Stage5 cohort. Overall historical R@1 and all saved Stage7.5 pooled R@1/R@5/MRR were reproduced exactly from the cache and original scalar scorer. The historical Stage5 raw CSV directory is not retained in the current checkout/server snapshot; its corrected committed documentation is historical evidence. R@10/R@20 in this new comparison are newly recovered from the frozen protocol, not claimed as old saved metrics.

| subset | method | R@1 | R@5 | R@10 | R@20 | MRR |
|---|---|---|---|---|---|---|
| overall | Single RGB | 0.702128 | 0.939444 | 0.992362 | 0.998363 | 0.802186 |
| same_view | Single RGB | 0.906639 | 0.981328 | 1.000000 | 1.000000 | 0.939558 |
| overall | RGB5 Mean | 0.682488 | 0.912711 | 0.989089 | 0.998363 | 0.775743 |
| same_view | RGB5 Mean | 0.883817 | 0.970954 | 0.995851 | 1.000000 | 0.919650 |
| overall | Cross-Max | 0.714675 | 0.901255 | 0.986907 | 0.998363 | 0.792230 |
| same_view | Cross-Max | 0.890041 | 0.962656 | 0.997925 | 1.000000 | 0.919292 |

SC Top20 already supplies a geometry-filtered high-recall pool. A method described as RGB-only there still benefits from SC candidate generation. Independent retrieval faces all4180 candidates and must not inherit the pool-conditioned success rates.

Exact-tie caveat: historical Stage5 preserves SC candidate order when visual scores tie, whereas the full benchmark uses database-ID ties. This is particularly relevant to overlapping Cross-Max windows. tie_policy_diagnostic.csv includes same-score/SC-pool results with ID ties to separate tie policy from pool expansion; it does not tune or replace the declared result.

| subset | method | rank1_tied_queries | median_rank1_tie_count | pool_R1_with_ID_ties | pool_R5_with_ID_ties |
|---|---|---|---|---|---|
| overall | Single RGB | 0 | 1.000000 | 0.702128 | 0.939444 |
| same_view | Single RGB | 0 | 1.000000 | 0.906639 | 0.981328 |
| overall | RGB5 Mean | 0 | 1.000000 | 0.682488 | 0.912711 |
| same_view | RGB5 Mean | 0 | 1.000000 | 0.883817 | 0.970954 |
| overall | Cross-Max | 1833 | 5.000000 | 0.659574 | 0.894163 |
| same_view | Cross-Max | 482 | 5.000000 | 0.869295 | 0.960581 |

There are 596 method/query Rank1-positive results whose winning database ID is outside the SC Top20. This local complementarity does not by itself establish an aggregate improvement or a validated fusion method.

## Validation and preservation

An independent validator checks every complete2000x4180 ordering as a permutation and deterministic sort, recomputes visual scores using a different temporal-max implementation, recomputes metrics from inverse permutations and frozen positive-pair labels, and checks all146640 Top20 rows. It independently rechecks all482 IDs and frozen-file hashes. No frozen Stage4/5/6/7/7.5 output was changed. Full score/rank arrays remain server-only with hashes and sizes in full_rankings_artifact_manifest.json.

All seven deterministic numeric/ID CSVs and all eight complete score/order arrays have identical hashes across two executions. The first attempt is retained server-side; the final source reads only keyframe IDs during pre-ranking cache checks.

Cross-Max has exactly five tied top-scoring candidates for every valid query in this run. Neighboring causal windows share the winning image pair; this is a score-definition ambiguity. The reported deterministic Rank1 is not a unique visual localization. R@5 and the tie-policy diagnostic provide additional context.

## Thesis interpretation and limits

On this frozen CU-Multi R1-to-R3 protocol, full-database Scan Context exceeds these three independent generic OpenCLIP retrieval methods, including the same-view control. The earlier visual results quantify reranking inside an SC pool and cannot support an independent visual retrieval claim. Do not claim that RGB improves place recognition from this experiment. These results do not establish universal LiDAR superiority over all dedicated visual place-recognition models or datasets.

The controlled subset is a query-heading stratum, not a heading-filtered database. Front RGB and360-degree LiDAR observe different context. This is offline static-database retrieval, not causal dual-stream discovery, mapping or online latency measurement. Cross-Max shared-window exact ties and static/common-start content remain relevant limitations. No threshold tuning, fusion, training or geometric verification is part of this stage.

## Reproduce

With the recorded processed caches and frozen project artifacts, run from repository root:
```bash
bash scripts/run_stage5b_independent_visual_retrieval.sh --out /home/cas/fyp_stage5b/reproduction
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /home/cas/.venvs/fyp_stage24/bin/python src/cumulti/validate_stage5b_independent_visual_retrieval.py --repo . --processed /home/cas/CU-Multi/processed_v1 --work /home/cas/fyp_stage5b/work/full_rankings --out /home/cas/fyp_stage5b/reproduction
```
The runner refuses to overwrite completed outputs. Only its fingerprinted SC scratch matrix may be reused; all visual scores/orderings are regenerated. Set STAGE5B_WORK to a fresh scratch directory for a full SC reconstruction. Large embedding caches are not committed.
