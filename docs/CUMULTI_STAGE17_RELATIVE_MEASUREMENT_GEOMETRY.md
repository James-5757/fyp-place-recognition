# Stage 17 — relative-measurement uncertainty and nonlinear geometry

Accepted checkpoint: `872a8d1a48f782afd0d48491e452f6b0c66dfe00`.
Execution and independent artifact audit: **PASS**.
Decision: **RELATIVE_UNCERTAINTY_AND_NONLINEARITY_PARTIALLY_EXPLAIN_STAGE16_GAP**.
**GT_USED: NO. NUMERICAL_DIAGNOSTIC_LINE_CLOSED: YES.**

This is the final numerical-diagnostic stage. It creates no sparse selector,
new K budget or accepted sparse policy. Stage15 remains
`NO_INFORMATIVE_SPARSE_POLICY`; Stage16 remains
`MEASUREMENT_INFORMATION_PARTIALLY_EXPLAINS_SPARSE_FAILURE`. No demo change,
distributed SLAM, SC/GICP rerun or graph optimization occurred.

## Frozen experiment and reproduction

`scripts/run_stage17_relative_measurement_geometry.sh` uses isolated GTSAM4.2
in `/home/cas/.venvs/fyp_gtsam`. The runner refuses an existing nonempty output
directory; `--report-only` regenerates reporting without computation/optimization,
and `--audit-only` independently checks saved artifacts. `--resume-workers` is a
guarded incomplete-run recovery mode and verifies the original policy/input hashes.
Outputs are in `outputs/cumulti_v1/17_relative_measurement_geometry/`.

Historical120 loops reproduce exactly clean75/excluded45, unchanged measurements,
endpoints, first bridge, odometry, noise/Huber and R1#150 gauge. Input SHA256
values match accepted Git blobs; historical non-GT files have before/after
immutability checks. No raw dataset, processed keyframe table with GT columns,
GT pose/evaluation file or map/scan is read. An input-access guard rejects these
dataset/GT paths. Graph frames are reconstructed from the frozen non-GT local
trajectory instead of accessing keyframe tables. Existing unrelated dirty demo
and Stage10.3 files remain unchanged and uncommitted.

Only these own frozen Stage15 optimized trajectories are used:

| Condition | Trajectory | Local covariance queries |
|---|---|---:|
| F0 Full75 | P0_K75.csv | same35 omitted-K40 pairs |
| S1 Coverage K40 | P2_K40.csv | omitted35 |
| S4 Coverage K20 | P2_K20.csv | omitted55 |

The same frozen full graph template supplies5796 nodes/5794 odometry/one prior,
with75/40/20 loops. Factors are checked against that template, not refitted.
Coverage selection is reproduced only for the relevant frozen K20/K40 sets.
There are **zero graph optimization runs** in Stage17.

Policy is fsynced before new metrics. Ten validation IDs remain exactly Stage16's
TOP5 `105,53,55,100,48` and BOTTOM5 `61,114,62,110,77`. Their original add-back
outcomes are read-only. No validation loop is selected using Stage17 scores.

## Eight distinct objects

1. Joint endpoint covariance: cross-correlated12x12 `Sigma_pair` from public
   GTSAM joint Marginals, explicitly assembled in query/candidate block order.
2. Exact residual: `E=Z^-1 Xq^-1 Xc`, `e=Pose3.Logmap(E)`; right tangent ordering
   `[rx,ry,rz,tx,ty,tz]`. Physical matrix translation remains separate from Logmap
   translation coordinates.
3. Exact measurement derivative: `J=LogmapDerivative(E) @ [between.Hq,between.Hc]`.
   The approximate GaussianFactor derivative is not substituted.
4. Measurement-visible covariance: `Sigma_rel=J Sigma_pair J.T`,6x6.
5. Dimensionless covariance: `C=D^-1 Sigma_rel D^-T`, with D from unchanged
   rotation sigma5deg/translation sigma1m.
6. Nonlinear whitened residual: `r=e/sigmas`, distinct from predicted uncertainty.
7. Local linear residual prediction: derivative-based first-order prediction
   under the declared temporary right perturbation.
8. Robust loss/weight and actual Stage16 add-back outcome: local public Huber
   diagnostics versus the already recorded independent full-graph response.

All125 pair/J queries reproduce the committed Stage16 local arrays; primary
joint covariance matrices match **bitwise**. Jacobian finite-difference maximum
error3.5863e-8 is below the unchanged1e-5 gate. Covariances are finite, symmetric
and PSD-valid, without clipping/jitter or an independent-block substitute. A
separately labelled **INVALID independent-block control** illustrates loss of
cross-correlation; it is never used for primary metrics.

### Ancillary query-check recovery

The first worker stopped at an ancillary entrywise comparison between separate
single-pose and joint-pair covariance queries. Recorded absolute differences
reach2.9763e-5, but block-scaled differences are at most3.4839e-10; small entries
make entrywise `allclose` unsuitable. The primary joint matrices match Stage16
bitwise, with endpoint ordering explicitly verified by public `JointMarginal.at`.
Recovery retains the initial failure, records per-endpoint differences and uses
the same1e-8 matrix-scaled numerical gate. No primary covariance/Jacobian,
information score, noise, research decision threshold or policy file changed.
The exact internal floating-point query paths were not instrumented, so this is
not a claim about a specific GTSAM implementation defect.

## Projected uncertainty: the common-mode interpretation is supported

| Median diagnostic | Full75, same35 | K40 omitted35 | K20 omitted55 |
|---|---:|---:|---:|
| Whitened relative covariance trace | 5.447383 | 94.503008 | 313.307588 |
| Largest whitened eigenvalue | 0.915137 | 18.571419 | 89.521080 |
| logdet(I+C) | 3.875983 | 16.854287 | 22.907263 |
| Entropy effective rank | 5.999658 | 5.930977 | 5.332235 |
| Whitened residual norm | 0.104922 | 1.971490 | 3.813129 |
| Huber weight | 1.000000 | 0.682225 | 0.352729 |

Comparing **identical35 pairs**, K40/Full75 ratios have medians:

- raw12D largest covariance eigenvalue: **1.000874**;
- measurement-visible whitened trace: **17.974028**;
- largest whitened relative eigenvalue: **20.893838**;
- logdet(I+C) difference: **12.970457**.

Thus Full75 substantially reduces measurement-visible uncertainty even though
the raw largest joint mode barely changes. These are local different-linearization
comparisons under the frozen numerical model, not physical localization accuracy.
K20 is more uncertain descriptively, but its55-loop omitted cohort differs from
K40's35; the distribution comparison is not a paired causal estimate.

For K40, the largest joint eigenvector's projection into the native-tangent
common-world-motion subspace has median fraction0.999999999643. That mode accounts
for only **1.4212e-6** of the whitened measurement covariance trace (top3 raw pair
modes contribute3.0159e-5 in median). This directly supports **common-mode
dominated under the tested relative-measurement projection**. It does not prove
a global graph nullspace or observability. Raw12D and6D eigenvalues have different
coordinate/normalization meanings; their simple ratio is not a variance-retention
percentage. The meaningful diagnostic uses modal contribution after J/D projection.

## DeltaI identity: projection is not a new information score

Stage16 already projected joint covariance through J. Consequently:

`logdet(I+C) = 2 * Stage16 DeltaI`.

Maximum discrepancy is1.0658e-14. Stage17 relative NIS reproduces Stage16 NIS
to8.8818e-16. Large endpoint common-motion modes therefore **did not inflate
Stage16 DeltaI merely by being large**: its formula already suppresses them.
Projection clarifies the misleading raw12D largest-eigenvalue interpretation,
but cannot by itself explain nominal DeltaI's limited add-back prediction.

For the frozen ten loops, projected trace and largest eigenvalue also preserve
DeltaI's rank ordering. Their Spearman add-back relationships are identical to
DeltaI, not improved. No independent predictive credit is assigned to2DeltaI.

## Residual direction and descriptive categories

Eigenvectors of C are sorted descending and signed by positive largest absolute
component. For `a=U.T r`, saved fractions are squared energy along top1/top2/top3;
angles are acute vector/subspace angles. Covariance-only amplitudes `a/sqrt(lambda)`
and noise-inclusive `a/sqrt(1+lambda)` are distinguished. Zero residual angles
would be undefined, not fabricated as perfect alignment.

K40 median top1 residual fraction is0.062888; top3 is0.552052. Effective rank
5.930977 indicates relatively broad, near-isotropic uncertainty: the first mode
is not a uniquely dominant physical direction. Across35 omitted loops, whitened
norm/top3 fraction Spearman is-0.022129. Physical translation/top3 fraction is
0.600840, whereas physical rotation/top3 is-0.509524. Alignment is thus modality/
metric-dependent, not universal alignment of all large residuals with one mode.

Predeclared median splits use logdet(I+C) and whitened nonlinear norm. The automatic
categories contain11 high-uncertainty/high-residual,7 high/low,7 low/high and10
low/low loops. These are relative numerical categories, not correct/incorrect
loop labels. Their IDs and full measurements are retained, without manual cases.

## Nonlinear sweeps: the local approximation remains accurate

For each frozen validation loop and its top3 C modes, amplitudes are exactly
`{-2,-1,-0.5,0,0.5,1,2}`:210 temporary factor evaluations, no graph perturbation
or optimization. C modes live in the **Logmap residual tangent**, not directly
the relative-pose right tangent. With B=`LogmapDerivative(E)`, use:

`direction = solve(B, D*u*sqrt(lambda))`

`Trel(alpha) = Trel Exp(alpha*direction)`

`e_lin(alpha) = e0 + alpha*B*direction`.

The derivative is finite-difference checked at alpha0. Omitting the B pullback
would mix coordinate conventions and exaggerate/misattribute linearization error.

After taking each loop's maximum across its fixed top3 modes:

| Tested extent | Median max NL_rel | Largest max NL_rel |
|---|---:|---:|
| ±0.5 sigma | 0.000395 | 0.001487 |
| ±1 sigma | 0.000994 | 0.003804 |
| ±2 sigma | 0.002018 | 0.007608 |

At±2sigma, the worst relative whitened prediction error is below0.8%; median
absolute whitened error0.011605, maximum0.112356. Curvature/asymmetry are recorded,
but these curves **do not support strong local nonlinearity as the primary cause**
of the Stage16 global-tail prediction gap. This test concerns ten local factor
curves, not the entire graph's nonlinear solve basin or other perturbations.

## Huber geometry and frozen add-back validation

GTSAM's public Huber.weight/loss methods are available. At the current poses,
public loss agrees with the unchanged actual robust factor error. The usual
block-norm Huber formula also agrees; labels do not imply that this local probe
recovers every internal optimization iteration's IRLS state. No optimizer weight
is changed. Nominal quadratic loss, public robust loss, regimes and weights are
saved at all210 sweep points.

Among omitted35 K40 loops,23 are in the Huber tail and8 have weight<0.5.
High-DeltaI validation loops105/100 have weights0.281309/0.359829. However they
are **not** the predeclared high-DeltaI/low-global-effect mismatch cases. Those
cases are48/55: both have weight1 and very small nonlinear errors
(max±2sigma relative errors0.000079/0.000671). Thus strong down-weighting or
large local nonlinearity does **not** explain these important mismatches.

The unchanged Stage16 outcomes are joined with11 predeclared predictors plus
DeltaI baseline. N=10, **EXPLORATORY**, shared variables, no independent-sample
significance, fitted composite, regression or optimized ranking function.

| Predictor / outcome | Spearman |
|---|---:|
| DeltaI / target translation correction | 0.539394 |
| Whitened residual norm / target translation correction | 0.818182 |
| DeltaI / target rotation correction | 0.551515 |
| Whitened residual norm / target rotation correction | 0.769697 |
| Curvature proxy / target rotation correction | 0.927273 |
| Curvature proxy / all75 translation-p95 correction | 0.684848 |
| Top3 residual alignment / all75 rotation-p95 correction | -0.575758 |

Stronger exploratory correlations do not establish a new predictive model.
Residual size is related to available correction headroom, while Huber weight
is mathematically tied to residual norm. Curvature correlations coexist with
tiny absolute nonlinear errors. Neither proves that robust/nonlinear effects
caused global-tail behavior. Nominal local information concerns uncertainty
reduction, not residual direction, correction headroom or the response of75
coupled measurements after a nonlinear graph solve. The score alone cannot
guarantee global p95 improvement.

## Conservative decision, resources and closure

Frozen projection-materiality gates pass. Multiple exploratory relationships
exceed the predeclared correlation comparison gate, but **zero** predefined
mismatch cases meet the material nonlinear/Huber gate. Therefore full explanation
fails and decision is **RELATIVE_UNCERTAINTY_AND_NONLINEARITY_PARTIALLY_EXPLAIN_STAGE16_GAP**.
It chiefly resolves raw-pair uncertainty interpretation; it does not establish
a complete explanation for nominal DeltaI's global-tail prediction gap.

Three bounded workers complete in2.451/1.289/1.411s, peakRSS199.539MiB.
Limits remain extra2GiB address space,90s CPU,120s wall, single-threaded BLAS.
Measured totals: Marginals125.985ms,125 joint queries158.257ms,125 exact Jacobian
calculations38.307ms,projection/alignment69.342ms,210 local sweeps2.232ms,
335 robust evaluations13.874ms. Reconstruction includes verification and loading;
parent worker wall overlaps worker phase costs. Diagnostic cost is not online
SLAM latency. No dense global Hessian is constructed.

The independent auditor checks accepted hashes, historical immutability, joint
covariance/J reproduction, projection/eigenspace/residual identities,210 exact
curves,robust losses,read-only outcomes,72 exploratory correlations,automatic
categories,decision logic and mode summaries (3,074 checks). Ten measured figures
and all required tables/JSONs are committed; local matrices remain bounded6x6/
6x12/12x12 artifacts, not dataset/model/cache files.

All covariances remain local numerical diagnostics under frozen factor/noise
and robust linearization conventions, not calibrated physical localization
uncertainty. The perturbation study uses relative-state tangent directions, not
global endpoint trajectories or a new optimizer schedule. Near-isotropic and
nearly repeated eigenvalues limit semantic interpretation of individual modes.
Different conditions have different local linearization points; small dependent
validation cohorts do not establish causality or generalization.

**Close numerical solver/sparse-loop diagnostic expansion after Stage17,
irrespective of this partial outcome.** Do not start Stage18 selectors, parameter
sweeps, more K values or covariance weighting. The next separately authorized
phase is final progressive multi-robot mapping demo/system integration and
thesis evaluation/writing. Preserve prior incremental-backend disagreement and
other limitations; clearly label archived cross-run replay versus causal/live
operation. No demo or distributed SLAM starts in this commit.
