# CU-Multi Stage 7 — held-out generalization validation

## Protocol

Stage 7 tests the frozen Stage-6 selective visual policy on Robot2 -> Robot3. It
does not modify Stages 4--6. GT UTM poses are used only after retrieval is complete
for offline `d_xy < 5 m` labels and heading analysis, never for retrieval, candidate
selection, threshold selection, or reranking.

The exact frozen 2-Hz grids contain 2,230 Robot2 queries and 4,180 Robot3 database
frames. Scan Context is unchanged. The visual encoder is Stage-5 OpenCLIP
`ViT-B-32-quickgelu` / `laion400m_e32`; Robot2 512-D L2-normalized float32 RGB
embeddings were encoded once and cached outside Git. Only frozen SC Top-20 candidates
are eligible for visual verification.

## RGB audit

Nearest-RGB synchronization for all 2,230 Robot2 LiDAR keyframes has absolute
timestamp-difference mean 18.085 ms, median 16.137 ms, p95 33.612 ms, and maximum
67.869 ms. Four associations exceed 50 ms; none exceeds 75 or 100 ms. Details are
in `robot2_rgb_sync_audit.csv` and `robot2_rgb_sync_summary.json`.

## Primary Robot2 -> Robot3 result

There are 2,192 valid-overlap and 38 no-overlap queries. Whole-database frozen Scan
Context obtains R@1 **0.997263**, R@5/R@10/R@20 **1.000000**, MRR **0.998223**,
median/p95 first-positive rank 1, and worst rank 5. Every valid query has a positive
in the frozen SC Top-20.

Strict transfer is predeclared from Stage 6: SC-margin `< 0.0006740332464687`,
visual-margin `> 0.0062217712402343`, and SC-compatibility gap `<= 0.05`. It invokes
visual verification on 115/2,192 queries (4.973%), makes 14 overrides, and produces
**0 rescues / 0 regressions**. R@1 and R@5 remain **0.997263 / 1.000000**. This is
neutral held-out transfer: no harm, but no improvement over an already near-saturated
SC baseline.

As a separately labelled GT-free descriptive comparison, recalculated unlabeled
q5/q50 score thresholds of 0.000643527542706579 and 0.006049424409866333 invoke 112
queries (4.836%), make 14 overrides, and are also neutral. This is not the primary
transfer claim and was not selected using correctness labels.

## Heading, failures, reverse direction, and cost

Heading is offline-only. Strict transfer is identical to SC in every nearest-positive
GT-heading bin. Bin query counts from 0--30 through 150--180 degrees are 1,725, 53,
69, 7, 6 and 332. The 150--180-degree bin has SC R@1 0.984940; no bin has a rescue
or regression. The Scan Context best-shift proxy has MAE 2.673 degrees, median AE
1.490, p90 3.990, Pearson 0.991 and Spearman 0.900 against offline nearest-positive
heading; it is analysis only and not a policy input.

The six primary SC Rank-1 failures are IDs 413--417 and 1542, with first-positive
ranks 2, 2, 5, 3, 3 and 4. Each has a positive in Top-20, but strict verification
does not alter its rank. The secondary Robot3 -> Robot2 SC sanity direction has 1,879
valid-overlap and 2,301 no-overlap queries, with R@1 **0.996807**, R@5 **0.997339**,
R@20 **0.997871** and MRR **0.997102**.

Measured primary-direction costs are SC 155.885 ms/query, Robot2 RGB encoding 32.673
ms/frame and cached reranking 0.014 ms/query. At 4.973% invocation, added serial
compute is 1.625 ms/query; serial / speculative-parallel latency is 157.510 / 155.899
ms/query.

## Verdict

Strict Stage-6 policy transfer is **neutral**. It preserves the strong SC baseline
with zero observed regressions, but supplies no rescue. This held-out result does not
support a general visual-improvement or viewpoint-aware advantage claim. Cross-Max
was not run because an exact predeclared Robot2 temporal transfer configuration was
unavailable, avoiding an exploratory variant in held-out validation.

All artifacts are in `outputs/cumulti_v1/07_generalization_validation/`; Stages 4--6
remain unchanged.
