# CU-Multi Stage 6 — selective visual verification

## Scope

Stage 6 asks when visual evidence may modify a strong LiDAR Scan Context ranking. Stages 4 and 5 are frozen, read-only inputs. The protocol keeps 2,000 Robot1 queries, 4,180 Robot3 database frames, 1,833 valid `<5 m` GT queries, and the frozen SC Top-20 pool. Stage-5 OpenCLIP embeddings are loaded from cache without image decoding or encoding.

The Stage-5 Top-20 score/order cache was not persisted. Stage 6 deterministically reconstructs it from frozen SC descriptors and requires exact Stage-4 Rank-1 agreement before analysis. GT is used only after deployable features are built, for offline labels, calibration and oracle analysis. No deployment policy uses GT heading, pose, overlap or positives.

## Ceiling and proxy calibration

Frozen SC is 1,821/1,833 R@1 = **0.993453**. SC Top-20 contains a positive for 1,830 valid queries, so the strict Top-20 ceiling is **1,830/1,833 = 0.998363**. Candidate-generation failures 1847, 1848 and 1849 cannot be rescued by any frozen-Top-20 reranker.

The deployable yaw proxy is `min(6*best_shift, 360-6*best_shift)`. It passes a predeclared geometric criterion (median absolute error <=45° and absolute Spearman >=0.20): overall MAE / median AE / p90 AE is **8.604 / 1.689 / 11.047°**, with Pearson/Spearman **0.923/0.886** against the offline nearest-positive heading. It is much less reliable on the 12 SC-wrong queries (MAE 48.332°), which remains explicitly reported.

## Selective policies

SC score, score-margin and Top-5 dispersion features, plus visual margin and SC-compatibility features, are all GT-free. Sweeps use predeclared SC-margin quantiles 1/2/5/10/20/30%, visual-margin quantiles 50/70/80/90%, compatibility gaps 0.02/0.05/0.10, and proxy ranges <=30/60/90/120/all degrees. Every row in `policy_sweep.csv` retains invocation rate, overrides, rescues, regressions, net correction, R@1 and R@5.

| Policy | R@1 | R@5 | Invocation on valid queries | Overrides | Rescue / regression | Net | Model-A added compute |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Frozen SC | 0.993453 | 0.996181 | 0.00% | 0 | 0 / 0 | 0 | 0.000 ms/query |
| Single RGB: SC q5% + visual q50% + delta 0.05 | **0.993999** | **0.996727** | 4.31% | 9 | 1 / 0 | +1 | 1.741 ms/query |

This Single-RGB point is exploratory and in-sample, not a globally optimal threshold. It is the only sweep region with a rescue and zero regressions. Cross-Max has no zero-regression rescue point. At the fixed 10% SC / 80% visual / 0.05 compatibility / <=90° ablation, viewpoint gating does not improve over confidence gating: Single RGB is 0 rescue / 0 regression and Cross-Max is 0 / 1. Thus the current data do not demonstrate incremental value from viewpoint gating beyond confidence.

## Cost and controls

Saved latency measurements are used: SC 155.885 ms/query, OpenCLIP query encoding 34.796 ms/frame, Single rerank 0.014 ms/query, Cross-Max rerank 0.064 ms/query. Model A charges visual work only when invoked. Model B reports speculative-parallel response latency; Cross-Max additionally records cached-temporal and lazy five-frame encoding costs separately.

Global fixed-weight fusion is a control only. At alpha_SC=0.50 it is harmful: Single RGB has 1 rescue/7 regressions and Cross-Max 0/41. High SC weights reproduce SC without rescue. `oracle_analysis.csv` is clearly labelled offline-only and is not a deployable result.

## Outputs and conclusion

`outputs/cumulti_v1/06_selective_visual_verification/` contains the frozen Stage-5 heading basis, calibration, SC/visual diagnostics, complete sweeps and Pareto set, ablation, fusion control, oracle analysis, failure table, latency trade-offs and figures. All validation checks pass.

Stage 6 identifies a low-invocation, zero-regression Single-RGB region, but does not establish a general threshold or a viewpoint-gating gain. The three candidate-generation misses remain outside the frozen reranking ceiling.
