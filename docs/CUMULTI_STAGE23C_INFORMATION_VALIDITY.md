# Stage23C — Nominal information versus realized mapping impact

Status: diagnostic artifact validation PASS. Scientific conclusion: **NOMINAL_INFORMATION_PROXY_NOT_SUPPORTED** for the original proxy on this exposed recording; **GROUP_INTERACTION_IMPORTANT** is an additional observation. This is RETROSPECTIVE_EXPLORATORY, not an independent predictive validation or a universal impossibility claim.

## 1. Why theoretical information increases
For an anchored, fixed-linearization Gaussian model, adding A-transpose A is PSD. The determinant lemma gives information gain 0.5 logdet(I + A C A-transpose), while modeled covariance cannot increase. Synthetic tests cover anchoring/gauge, sparse solves, Pose3 rotation-then-translation tangent ordering, exact nonzero-residual Jacobians and joint endpoint cross-correlations. This identity does not compare different nonlinear robust solutions.

Synthetic numerical evidence (generated from the artifact):
| fixed_PSD_information_gain_nats | covariance_trace_before | covariance_trace_after | biased_realized_error | Pose3_exact_Logmap_Jacobian_max_error |
| --- | --- | --- | --- | --- |
| 2.31249 | 1.02 | 0.0198039 | 3.96098 | 8.09161e-10 |

## 2. Why realized ATE may not improve
Expected covariance assumes the stated measurement model; realized trajectory error is a different quantity. The biased synthetic measurement decreases covariance and increases actual error. This is a mechanism demonstration, not evidence that real GICP receipts are biased. Drift, measurement quality, robust weighting, state-dependent Jacobians, redundancy and changed nonlinear solutions can all matter. Negative signed LOO effects remain negative.

## 3. Exact Stage23B proxy limitations
The original nominal score uses a causal P0 common-odometry prefix and predicted endpoint relation, not an optimized posterior and not the current GICP transform. Its state, covariance and factor set differ from the final Batch graph. Actual prior-event optimized P0/P5 snapshots are **OPTIMIZED_CAUSAL_STATE_NOT_AVAILABLE**. Historical final trajectories are retrospective only; Stage20 checkpoints belong to a different graph and cannot replace the missing states. First-bridge prefix residuals are explicitly unavailable.

The canonical union contains 141 P0-only, 84 P5-only and 147 common factors. Direct single-factor LOO exists for 141 P0-only and seven P5-only factors; the other 77 P5-only and 147 common factors have no individual outcome.

## 4. Measured GICP quality and residual findings
All union factors are linked to actual frozen JSON/CSV receipts, canonical transforms and original candidate direction. No registration was executed. The following descriptive comparison uses the eight already-known offline false factors; no online threshold was designed, and no factor was removed. A large residual may also reflect correct constraints against drifted odometry.
| offline_GT_positive | field | n | median | p95 |
| --- | --- | --- | --- | --- |
| False | GICP_quality | 8 | 0.622239 | 0.629005 |
| False | GICP_fitness | 8 | 0.882857 | 0.889497 |
| False | GICP_RMSE | 8 | 0.41694 | 0.426476 |
| False | pre_insertion_normalized_residual | 8 | 47.2879 | 47.8984 |
| False | full_P0_diagnostic_huber_weight | 8 | 1 | 1 |
| True | GICP_quality | 364 | 0.642854 | 0.697311 |
| True | GICP_fitness | 364 | 0.893686 | 0.949513 |
| True | GICP_RMSE | 364 | 0.384478 | 0.414575 |
| True | pre_insertion_normalized_residual | 363 | 51.6994 | 88.8389 |
| True | full_P0_diagnostic_huber_weight | 364 | 1 | 1 |

The false factors have plausible registration quality and their prefix residual median is not larger than that of correct factors in this cohort. Thus neither receipt acceptance nor a large odometry disagreement establishes correctness. The correlation table shows only a weak positive post-GICP quality association; it does not establish a deployable pre-verification predictor.

P0-only correlation table: target labels remain signed and offline-only. Top-K is descriptive overlap with largest signed LOO harm, not a trained selector. Random reference is K/n. Primary confidence intervals resample the seven frozen temporal clusters; IID intervals are provided only as dependence-ignoring sensitivity. Seven exposed-route clusters are not independent recordings. P5-only n=7 has no bootstrap confidence interval; multiple comparisons are uncorrected and exploratory.
| predictor | target | n | Spearman_rho | CI_95_low | CI_95_high | top_K_exact_overlap | random_overlap_reference |
| --- | --- | --- | --- | --- | --- | --- | --- |
| nominal_info_nats | delta_joint_ATE_RMSE_m | 141 | -0.244604 | -0.441428 | -0.202268 | 0.1 | 0.070922 |
| relative_uncertainty_trace | delta_joint_ATE_RMSE_m | 141 | -0.259472 | -0.450557 | -0.204594 | 0.1 | 0.070922 |
| SC_confidence_frozen | delta_joint_ATE_RMSE_m | 141 | 0.0955796 | -0.0489837 | 0.313767 | 0 | 0.070922 |
| GICP_quality | delta_joint_ATE_RMSE_m | 141 | 0.111426 | 0.056545 | 0.23594 | 0.1 | 0.070922 |
| pre_insertion_normalized_residual | delta_joint_ATE_RMSE_m | 141 | -0.130609 | -0.242997 | 0.0121589 | 0.3 | 0.070922 |
| pre_insertion_huber_if_added | delta_joint_ATE_RMSE_m | 141 | 0.130609 | -0.0121589 | 0.242997 | 0 | 0.070922 |
| coverage_gain | delta_regional_ATE_RMSE_m | 141 | -0.0829359 | -0.307748 | 0.18171 | 0.2 | 0.070922 |
| coverage_gain | delta_regional_ATE_p95_m | 141 | 0.00863051 | -0.0852043 | 0.171575 | 0.2 | 0.070922 |
| nominal_info_nats | delta_projected_cov_trace | 141 | 0.634561 | 0.432298 | 0.830927 | 0.5 | 0.070922 |
| full_P0_diagnostic_huber_weight | delta_joint_ATE_RMSE_m | 141 | unavailable | unavailable | unavailable | unavailable | 0.070922 |

## 5. Nonlinear and robust-weighting findings
Fourteen predeclared Stage23B representatives use sparse joint marginals. Full versus removed-factor covariance at identical frozen full-P0 values isolates the fixed Gaussian removal term. Then existing frozen LOO states change the state and robust weighting jointly; changing the endpoint projection Jacobian contributes a third term. The decomposition closes numerically, but it does not identify a unique percentage caused by robust weighting. Retained factors have actual before/after reconstructed weights. A removed factor has no actual after-removal weight; its LOO-state counterfactual weight is labeled separately. Weak-direction residual projections are diagnostic, not a selector. The endpoint projection uses the predicted-between tangent Jacobian; actual measurement information uses the frozen factor's Gaussian linearization, including its residual-dependent Jacobian and robust noise.
| case | fixed_factor_removal_cov_component | state_and_robust_reweight_cov_component | projection_J_cov_component | retained_factor_max_abs_weight_change |
| --- | --- | --- | --- | --- |
| LOO_P0_1051_3158 | 33.7396 | 20.3907 | -20.2832 | 0 |
| LOO_P0_1051_3673 | 29.8652 | 2.80142 | -2.82407 | 0 |
| LOO_P0_1120_2527 | 37.0313 | 124.695 | -124.778 | 0 |
| LOO_P0_1393_2846 | 21.3903 | 19.1793 | -19.1915 | 0 |
| LOO_P0_1411_2824 | 19.1634 | 167.985 | -168.046 | 0 |
| LOO_P0_1601_2657 | 20.1993 | 186.97 | -186.98 | 0 |
| LOO_P0_1684_3428 | 23.0096 | 90.7108 | -90.7208 | 0 |
| LOO_P0_559_2536 | 19.5085 | 9.53521 | -9.53084 | 0 |
| LOO_P0_575_2550 | 15.9812 | 4.18622 | -4.18428 | 0 |
| LOO_P0_747_3425 | 39.291 | 266.982 | -266.991 | 0 |
| LOO_P0_911_3293 | 18.3746 | 106.811 | -106.815 | 0 |
| LOO_P0_918_3790 | 17.7462 | 6.8358 | -6.84667 | 0 |
| LOO_P0_973_3748 | 45.2872 | 429.686 | -429.477 | 0 |
| LOO_P0_974_3246 | 22.9258 | 31.5647 | -31.5286 | 0 |

Observed weight evidence:

```json
{
  "representatives": 14,
  "representatives_with_retained_weight_change": 0,
  "maximum_retained_weight_change": 0.0,
  "maximum_covariance_decomposition_closure_error": 7.815970093361102e-14,
  "interpretation": "zero retained-weight change means this representative audit shows no robust-reweight mechanism; state/Jacobian variation remains distinct"
}
```

## 6. Group-level interactions
The bounded two-group pilot preceded all seven exact group calculations. Joint endpoint covariance includes cross-blocks. Exact group information and the sum of single-factor additions are evaluated against the SAME group-removed Gaussian base at the SAME full-P0 values. Their difference is Gaussian redundancy. Mean original prefix information uses another model and is not interchangeable with either. Group-removal offline ATE minus the sum of individual signed LOO effects reveals nonadditivity, not a unique causal decomposition. Regional masks differ between individual cases, so regional effects are not added. Single-LOO map effects were not measured; existing group regional map metrics are preserved explicitly.
| case | factors | exact_fixed_Gaussian_group_information_nats | sum_individual_addition_information_same_base_nats | mean_original_prefix_nominal_information | measured_group_delta_joint_ATE_m | sum_measured_single_LOO_delta_joint_ATE_m | nonadditive_joint_ATE_interaction_m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| GROUP_CLUSTER_3 | 2 | 17.083 | 22.1174 | 12.4296 | 0.0279475 | -0.00539017 | 0.0333377 |
| GROUP_CLUSTER_0 | 13 | 75.128 | 92.6456 | 10.6292 | -0.160453 | -0.163502 | 0.00304944 |
| GROUP_CLUSTER_1 | 11 | 69.4043 | 86.5532 | 13.107 | 0.168795 | -0.0548323 | 0.223628 |
| GROUP_CLUSTER_2 | 40 | 247.069 | 322.158 | 10.8276 | 0.297692 | 0.563662 | -0.265971 |
| GROUP_CLUSTER_4 | 48 | 301.045 | 372.529 | 11.1024 | -0.154806 | -0.156512 | 0.00170567 |
| GROUP_CLUSTER_5 | 2 | 12.6956 | 14.8495 | 9.35819 | -0.0123036 | -0.00267297 | -0.00963062 |
| GROUP_CLUSTER_6 | 25 | 151.031 | 215.181 | 9.93049 | -0.0337307 | -0.0518733 | 0.0181426 |

## 7. What is a causal pre-GICP feature
SC score/rank, frozen normalized confidence, prefix uncertainty, prior-event coverage and estimated transfer bytes are PRE_VERIFICATION. GICP fitness/RMSE/quality, measured transform and pre-insertion measurement residual/Huber weight are POST_VERIFICATION: the word pre-insertion does not mean pre-registration. Full-graph residuals and weights are RETROSPECTIVE_GRAPH_ANALYSIS. Known correctness and LOO ATE are OFFLINE_GT_EVALUATION. Post-verification scores cannot justify avoiding the exchange/registration that produced them. See the explicit feature inventory and immutable prefix provenance.

## 8. Does information gain deserve further use?
The original nominal proxy is not supported as an information-first ranking on this recording. Mathematical PSD behavior passes, but this does not validate its mapping utility. Nominal information is positively associated with the measured covariance-change target while negatively associated with signed offline ATE impact (see the generated table); modeled uncertainty and realized accuracy therefore disagree without violating the Gaussian identity. Receipt-quality associations are exploratory and post-verification; no optimized feature combination, cutoff, weight or model is chosen. No retained-factor weight change was observed in the representative audit, so robust reweighting is not demonstrated as the cause there. The covariance decomposition and group interactions explain distinctions and mechanisms, not a complete causal attribution of the negative correlation.

## 9. Recommended prospective P6 architecture
Prioritize communication-aware P5 improvements, cache reuse, explicit coverage constraints and budget scheduling. If information is revisited, predeclare its causal state/model and evaluate on genuinely unseen recordings with adequate bidirectional coverage. No P6, learned selector, demo change or distributed SLAM was implemented here.

## 10. Data coverage, resources and reproducibility
STAGE23_FINAL_INDEPENDENT_VALIDATION remains **NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE**. The 2112 unmeasured GICP candidates remain unknown. Exposed offline GT was used only through frozen Stage23B labels/evaluation results, never for retrieval, fitting or parameter selection. No independent final testing occurred.

Run from repository root: `scripts/run_stage23c_information_validity.sh`. The isolated GTSAM environment is unchanged; existing Stage23A/B inputs, root context, NEXT_STEPS and unrelated dirty files remain preserved. All input hashes and the predeclared policy are checked. No dense global information matrix, new nonlinear optimization or GICP execution is required. A provenance-only engineering repeat separated the immutable causal-prefix table from its later retrospective augmentation; both sparse calculation costs are retained in the recovery audit. Timing is machine-dependent. Lightweight results are sufficient; no raw dataset, bag, credential, model or hidden script is required beyond the documented frozen server inputs.

Recorded resource and validation summary:

```json
{
  "status": "PASS",
  "checks": {
    "frozen_input_hashes_match": true,
    "synthetic_information_sanity": true,
    "canonical_cohorts": true,
    "measured_outcomes_only": true,
    "actual_receipts_and_prior_events": true,
    "immutable_prefix_provenance": true,
    "no_optimized_causal_state_substitution": true,
    "current_measurement_features_are_post_verification": true,
    "fourteen_fixed_covariance_decompositions": true,
    "seven_exact_joint_group_calculations": true,
    "small_cohort_uncertainty_not_fabricated": true,
    "eight_false_factors_unfiltered": true,
    "signed_LOO_values_unchanged": true,
    "bounded_sparse_computation": true,
    "no_new_solver_GICP_or_selector": true,
    "independent_final_validation_not_executed": true,
    "required_outputs_exist": true
  },
  "scope": "artifact consistency validation, NOT independent scientific testing",
  "total_recorded_additional_calculation_seconds": 14.869919026270509,
  "peak_graph_RSS_MiB": 221.37109375,
  "engineering_repeat_included": true
}
```
