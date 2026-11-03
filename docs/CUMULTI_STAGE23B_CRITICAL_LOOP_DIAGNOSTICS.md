# Stage23B — retrospective critical-loop diagnostics

**STAGE23B_DIAGNOSTICS: COMPLETE**

**STAGE23_FINAL_INDEPENDENT_VALIDATION:
NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE**

These decisions are separate. This is an authorized retrospective diagnostic,
not independent final validation, an accepted selector, or deployable readiness.
No new GICP, unknown-outcome imputation, distributed SLAM or demo change.
The old dirty files and all Stage23A outputs remain byte-identical. No NEXT_STEPS
edit; this new document records the experiment without rewriting old results.

## 1. Exact P0/P5 identities and false factors

P0=288, P5=231, intersection147, P0-only141, P5-only84, net57.
The query-level geometric true-positive loss335 remains a different quantity.
All141 P0-only and84 P5-only factors are positive under offline d_xy<5m.
Common147 includes139 positives and8 negatives; false factors are explicitly
retained/labelled in the offline audit, never silently assumed correct or
removed from optimization. Do not use these labels to tune a future selector.
Canonical IDs, event, measured geometric quality, odometry bins, first-bridge
and nearest earlier admitted pair are recorded for both exclusive sets.

## 2. Seven temporal clusters

The fixed Stage23A30s-gap clusters are unchanged. Exact frozen Batch results:

| case | removed | joint_ATE_RMSE_m | delta_joint_ATE_RMSE_m | delta_regional_ATE_RMSE_m | delta_regional_ATE_p95_m |
| --- | --- | --- | --- | --- | --- |
| GROUP_CLUSTER_0 | 13 | 20.9991 | -0.160453 | -0.253685 | -0.369676 |
| GROUP_CLUSTER_1 | 11 | 21.3283 | 0.168795 | 0.233029 | -0.285078 |
| GROUP_CLUSTER_2 | 40 | 21.4572 | 0.297692 | -0.0175442 | -0.283248 |
| GROUP_CLUSTER_3 | 2 | 21.1875 | 0.0279475 | 0.167373 | 0.324506 |
| GROUP_CLUSTER_4 | 48 | 21.0047 | -0.154806 | -0.274033 | -0.0341513 |
| GROUP_CLUSTER_5 | 2 | 21.1472 | -0.0123036 | -0.00807157 | -0.00962694 |
| GROUP_CLUSTER_6 | 25 | 21.1258 | -0.0337307 | -0.351814 | -1.49458 |

Signed deltas matter: positive means removing the factors worsened the metric;
negative means removal improved it. True d_xy matches do not guarantee accurate
relative measurements, and extra factors need not monotonically improve GT
trajectory accuracy. The coarse timestamp clusters can span multiple spatial
regions. Maps are evaluated globally and in matched removed-endpoint regions,
plus all four uncovered bins; zero topological cut edges does not prove local
redundancy. All graphs remain connected via the unchanged odometry chains and
remaining cross-robot constraints; first bridge is never removed.

## 3. Four P5-uncovered route bins

| robot1_kf | robot3_kf | R1_bin50 | R3_bin50 | delta_joint_ATE_RMSE_m | delta_regional_ATE_RMSE_m | delta_regional_ATE_p95_m |
| --- | --- | --- | --- | --- | --- | --- |
| 1120 | 2527 | 12 | 30 | -0.157999 | -0.288629 | -0.321781 |
| 911 | 3293 | 9 | 42 | -0.000924888 | -0.00418598 | -0.00279552 |
| 747 | 3425 | 6 | 43 | -0.00217379 | -0.0048917 | -0.00926385 |
| 973 | 3748 | 9 | 48 | -0.0340613 | -0.149139 | -0.127716 |

Bins are50m frozen local-odometry arc-length pairs, not GT-derived geographic
regions. Region evaluation uses union +/-50m around the removed endpoints,
with the same membership for baseline and ablation. One JOINT global rigid
alignment is used per condition; no independent robot/region fit is performed.
Map NN uses frame10/point16/voxel0.5m/range1–60m. A positive/negative regional
map change can differ from global ATE; NN tails include non-overlap coverage
and are not calibrated absolute map accuracy.

## 4. Important individual missing factors

Largest observed harmful-removal effects, sorted by signed joint ATE delta:

| case | delta_joint_ATE_RMSE_m | delta_regional_ATE_RMSE_m | delta_regional_ATE_p95_m | delta_inter_robot_relative_translation_RMSE_m | delta_projected_cov_trace |
| --- | --- | --- | --- | --- | --- |
| LOO_P0_1410_2830 | 0.334929 | -0.0159898 | 0.213724 | 0.000695901 | 20.1937 |
| LOO_P0_1411_2824 | 0.249482 | 0.0959514 | 0.355222 | -0.00483877 | 19.1024 |
| LOO_P0_1349_2894 | 0.104107 | 0.152374 | 0.199125 | 0.00464334 | 37.8009 |
| LOO_P0_1390_2852 | 0.0647571 | 0.107858 | 0.182277 | 0.00128746 | 19.6663 |
| LOO_P0_1393_2846 | 0.0496678 | 0.076499 | 0.130633 | -0.00160499 | 21.3781 |
| LOO_P0_827_3360 | 0.044766 | 0.138908 | 0.196477 | 0.000405279 | 42.8653 |
| LOO_P0_538_3093 | 0.0406903 | -0.137436 | -0.0716751 | -0.00170461 | 31.5571 |
| LOO_P0_1445_2796 | 0.0316105 | -0.0391504 | -0.0309138 | 0.00120523 | 27.6694 |
| LOO_P0_968_3760 | 0.0240412 | 0.0841871 | 0.0777768 | 0.000441887 | 23.539 |
| LOO_P0_1495_2745 | 0.0192303 | -0.0114418 | -0.0396223 | -0.00175823 | 23.427 |

Across141 LOO cases, signed joint-ATE delta min/median/max is
-0.180257/
-0.000353877/
0.334929m. No materiality cutoff or online policy
was fitted from these GT outcomes. Single-factor effects do not sum to group
effects because optimization, robust reweighting and redundant support interact.
Every LOO retains5796 nodes/5794 odometry/one original prior, removes exactly
the named factor, uses the original chronological factor ordering and original
first-bridge initial estimates. Both full baselines reproduce frozen Batch
poses within1e-7; no tolerance/noise/Huber/solver change was used.
The ablated objective belongs to a different graph and is not directly an
accuracy comparison; full_reference_objective separately evaluates the original
policy graph at the ablated solution. Lower own-graph cost alone is not success.

## 5. Do P5-only factors provide alternative support?

| case | joint_ATE_RMSE_m | inter_robot_GT_relative_translation_RMSE_m |
| --- | --- | --- |
| BASE_P0 | 21.1595 | 3.75168 |
| BASE_P5 | 20.421 | 3.72596 |

| case | removed | joint_ATE_RMSE_m | delta_joint_ATE_RMSE_m | delta_regional_ATE_RMSE_m |
| --- | --- | --- | --- | --- |
| P0_COMMON_ONLY | 141 | 21.3628 | 0.203302 | -0.0200685 |
| P5_COMMON_ONLY | 84 | 21.3628 | 0.941877 | 0.946173 |

Removing all exclusive factors yields the SAME147-factor common-loop graph
from either policy; optimized matrix maximum difference is
0. This matched control
shows the two exclusive sets supply different alternative support. P5-only
group removal and the seven preselected P5 singleton removals are measured,
not inferred from net counts. It does not prove every P5-only edge substitutes
one particular missing P0 edge, nor isolate a single component cause. P5's
lower full-graph joint ATE here does not reverse Stage22B's geometric-quality
loss or incomplete independent-generalization result.

## 6. GT-free criticality vs offline effects

Features are frozen BEFORE any current-run GT evaluation. For all372 union
factors, reconstruct the strictly earlier P0 factor prefix at its logical
discovery event. Insert current arriving odometry, but not its current loop.
After first bridge, linearize at common-frame LOCAL ODOMETRY prefixes, NOT
future optimized poses. Preserve joint endpoint cross-covariance. Predicted
Z=current prefix between; nominal logdet/normalized relative trace/max-eigenvalue
use no actual current-GICP transform. Pre-bridge common covariance is explicitly
undefined. Coverage uses earlier-loop odometry path neighborhoods; connectivity
uses current global/region state. Marginal bytes come from the frozen P0 cache
state before that attempt; info/byte uses a documented denominator max(1,bytes)
regularizer for cached zero-transfer cases, not a fabricated transmitted byte.

| proxy | n | Spearman_rho | top_K | top_K_overlap_fraction | top_K_signed_harm_positive_fraction | status |
| --- | --- | --- | --- | --- | --- | --- |
| nominal_info_nats | 141 | -0.244604 | 10 | 0.1 | 0.1 | DEFINED |
| relative_uncertainty_trace | 141 | -0.259472 | 10 | 0.1 | 0.1 | DEFINED |
| relative_uncertainty_max_eigenvalue | 141 | -0.259681 | 10 | 0.1 | 0.1 | DEFINED |
| coverage_gain | 141 | -0.0981292 | 10 | 0.1 | 0.1 | DEFINED |
| region_connectivity_gain | 141 | 0.0635191 | 10 | 0 | 0.2 | DEFINED |
| global_connectivity_gain | 141 | nan | 10 | nan | nan | UNDEFINED_CONSTANT_OR_INSUFFICIENT |
| predicted_benefit_per_byte | 141 | -0.217608 | 10 | 0.1 | 0.5 | DEFINED |

The observed negative information/uncertainty rank correlations do NOT support
promoting information-first ranking on this route. Coverage also does not show
strong positive predictive evidence here. This negative finding is preserved,
not remedied by fitting weights to these GT-exposed outcomes.

Every information proxy is a local model, not calibrated physical uncertainty
or an optimized iSAM2 covariance. These are causal GT-free prefix-odometry
predictions tested against retrospective LOO effects; they are not a trained
online selector. Projected uncertainty and logdet are mathematically related,
not independent corroborating predictors. Global-connectivity proxy is constant
for the P0-only cohort and its correlation is undefined, not evidence of no
local importance. Group aggregation is mean individual prefix proxy, NOT a
joint group logdet. P5-only and7-group correlations are small-N exploratory;
P5 cases use a P0-prefix predictive context, not a claimed P5 online posterior.
Spearman p-values are uncorrected; Top-K exact overlap includes signed adverse
and beneficial removal outcomes, with random K/N reported. No physical accuracy
or generalization guarantee follows from these correlations.

Relative-pose GT error uses the FIXED union372 keyframe pairs and original
Stage22B reference CSV Pose3 convention. Translation/rotation residuals compare
estimated relative transforms with those references. No new extrinsic/frame
calibration was assumed; fixed reference/sensor-frame bias remains a limitation.
Measured-loop residuals are separately reported, not passed off as GT errors.
Covariance-change diagnostics preserve cross-correlations but compare each
optimized linearization/robust weighting, so are not pure fixed-Hessian removal
information. No dense global matrix was formed.

## 7. Cost, failures and reproducibility

Pilot14 solves actually took5.177s and projected
58.794s for all cases, BEFORE full LOO execution.
Budget900 solver seconds/2GiB was fixed in advance. Actual159 solver cases took
69.472s including optimization, pose extraction/checks and compressed
state export (not pure native-LM timing), peak180.230MiB;
0 failed/nonconverged. Offline evaluation/map elapsed
122.777s is separately measured. GT-free feature
construction took12.818s. State exports
and per-case receipts remain server-only with hashes. Required CSVs, policy,
pilot, freezes, independent validation and this report are versioned. A reporting
normalization makes the exact-set table one row per canonical identity; it does
not change solved graph/results. Run
`bash scripts/run_stage23b_critical_loop_diagnostics.sh`; existing frozen solves
are reused, never replaced. No existing Stage23A output is overwritten.

## 8. Recommendations for future P6 — not finalized

Keep the independent-data NO-GO unchanged. Treat coverage/connectivity as explicit
constraints rather than assuming logdet-per-byte alone protects useful regions.
Evaluate hybrid proxies prospectively on eligible independent data before any
promotion. Do not choose weights, graph thresholds, robust noise, materiality
gates or deployment claims from these GT-exposed retrospective results. Preserve
the distinction between model uncertainty, realized geometric quality, regional
coverage and actual physical accuracy. No selector was trained/finalized and
the2112 unmeasured GICP candidates remain unknown. Stop after Stage23B diagnostics.
