# CU-Multi Stage 8 — efficient hierarchical Scan Context retrieval

## Scope

Stage 8 leaves Stages 4--7.5 read-only. It uses frozen 2-Hz Robot1/Robot2/Robot3
keyframes and Scan Context descriptors only. No RGB, OpenCLIP, fusion, GICP, PGO,
CVTNet, VLM, raw-point-cloud transmission, or new descriptor is used. GT is only
applied after retrieval for offline labels and diagnostics.

Stage 8A is the **standard Scan Context Ring-Key plus KD-tree acceleration
baseline**, not a novel method. Stage 8B evaluates adaptive progressive retrieval
in this multi-robot pipeline; no novelty claim is made.

## Timing harness and Ring Key

All online timings preload descriptors and keys into memory, use the same process
and exact frozen circular-shift scoring function, and include a ten-query warm-up.
One-time KD-tree construction is excluded from online timing. The Stage-8 exhaustive
re-measurement is R@1/R@5/R@20 **0.993453/0.996181/0.998363**, MRR **0.994682**,
matching Stage 4. Mean/median/p95 latency is **106.473/106.078/109.183 ms** per
Robot1 query (9.392 q/s). The historical Stage-4 mean was 155.885 ms and is kept
only as historical context; the Stage-8 denominator is the new 106.473 ms value.

The canonical Ring Key is the 20-D float32 mean of each descriptor ring over its
60 sectors. It contains no pose, heading, GT, RGB, or visual data. Full SC is
4,800 B/frame while Ring Key is 80 B/frame. For Robot3, frozen SC cache storage is
20,064,000 B and Ring-Key storage is 334,400 B. The exact Euclidean `scipy.cKDTree`
over Robot3 keys builds offline in 0.00197 s; its Python object size is reported as
an incomplete practical memory indicator alongside the 334,400-B key array.

For every shortlisted candidate, the accelerated exact score and `best_shift` are
asserted equal to exhaustive scoring within `1e-6`. Thus hierarchy changes only
candidate selection, not Scan Context scoring.

## Stage 8A fixed shortlist sweep

| M | CandidateRecall | R@1 | R@5 | R@20 | Mean ms | p95 ms | Speedup | Rank-1 agreement |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | 0.936716 | 0.933988 | 0.936716 | 0.936716 | 0.256 | 0.298 | 415.31x | 0.725586 |
| 20 | 0.959629 | 0.955265 | 0.957992 | 0.959629 | 0.439 | 0.491 | 242.65x | 0.789416 |
| 50 | 0.974359 | 0.968903 | 0.971631 | 0.973813 | 0.958 | 0.995 | 111.14x | 0.875068 |
| 100 | 0.985270 | 0.978723 | 0.982542 | 0.984724 | 1.881 | 1.920 | 56.59x | 0.951991 |
| 200 | 0.993999 | 0.988543 | 0.990180 | 0.992908 | 3.762 | 3.814 | 28.30x | 0.974359 |
| 500 | 0.998363 | 0.991817 | 0.994544 | 0.997272 | 8.914 | 9.042 | 11.94x | 0.991271 |
| 1000 | 0.999454 | 0.992908 | 0.995636 | 0.998363 | 17.404 | 17.478 | 6.12x | 0.997818 |

The predeclared rule selects the smallest M with CandidateRecall loss <=0.1pp
relative to the exhaustive Top-20 ceiling and R@1 loss <=0.1pp relative to
exhaustive SC. **M=1000** is the only and thus selected fixed operating point.
It loses 0.0546pp R@1, while its candidate recall exceeds the Top-20 ceiling. It
uses 3.48% of the 500-ms 2-Hz budget. Candidate pruning changes rankings: at M1000,
one exhaustive-correct query becomes wrong, zero exhaustive-wrong queries become
correct, and mean Top-5 overlap is 0.997054. This is not presented as an accuracy
improvement.

## Stage 8B adaptive progressive retrieval

The predeclared schedule is 20->50->100->200->500. At each reached stage, exact
SC scores are reused and the GT-free `margin12` is recomputed. Threshold candidates
are q50/q60/q70/q80/q90/q95 of the Robot1->Robot3 M20 margin distribution. Every
candidate policy fails the joint predeclared constraint. The best high-confidence
diagnostic, q95, has CandidateRecall 0.997818 and R@1 0.991271; M500 reaches the
candidate ceiling but is still 0.2182pp below exhaustive R@1, exceeding the 0.1pp
allowance. Therefore **no adaptive policy is frozen** and no Robot2->Robot3
adaptive result is reported. This is a valid negative result, not a tuning failure.

## Held-out Robot2 -> Robot3

The frozen M=1000 point transfers without retuning. Exhaustive Robot2->Robot3 is
R@1/R@5/R@20 **0.997263/1.000000/1.000000**, MRR **0.998228**, mean/p95
**105.481/107.329 ms**. Fixed M1000 preserves these metrics exactly, has candidate
recall 1.0 and Rank-1 agreement 0.999088, at mean/p95 **19.767/20.073 ms** for a
**5.34x** measured speedup. Adaptive is explicitly unavailable because no
development-pair policy met the frozen constraint.

## System projection

This is a compute-capacity projection only, not a live multi-robot demonstration.
Using fixed M1000 throughput of 57.46 q/s, a centralized stream of 4 q/s from two
robots consumes 6.96% of one-thread retrieval capacity; 8 q/s from four robots
consumes 13.92%. Per-keyframe/per-second descriptor communication is 80 B/320 B/s
for Ring Keys versus 4,800 B/19,200 B/s for full SC at 4 q/s; at 8 q/s it is
640 B/s versus 38,400 B/s. This excludes networking, queueing, mapping, and all
other live-system work.

Outputs, candidate-generation misses, timing distributions, and figures are in
`outputs/cumulti_v1/08_efficient_sc_retrieval/`.
