# Stage 16 — measurement-information and weak-direction audit

Accepted Stage15 checkpoint: `cc32583794e3e076d19582311bb1bd53ae5cef74`.
Execution/provenance audit: **PASS**. Scientific decision:
**MEASUREMENT_INFORMATION_PARTIALLY_EXPLAINS_SPARSE_FAILURE**.
GT_USED: **NO**. No new sparse policy, retrieval/registration, demo or distributed
SLAM. Stage15 remains `NO_INFORMATIVE_SPARSE_POLICY`.

## Protocol and reproducibility

Run `scripts/run_stage16_measurement_information.sh` from repository root using
isolated GTSAM4.2 `/home/cas/.venvs/fyp_gtsam`; fyp_slam remains untouched.
The runner refuses a nonempty destination. `--report-only` rebuilds reporting
without optimization; `--audit-only` checks saved artifacts independently.
`--resume-addback` is guarded recovery for an incomplete run, not a new protocol.
All outputs are under `outputs/cumulti_v1/16_measurement_information/`.

Historical120 loops yield the identical75 retained/45 excluded set. Endpoint
IDs, transforms, quality, SC metadata, ordering, first bridge, odometry, factor
noise/kernel, stable ranges and anchor remain unchanged. Accepted input Git blobs
and historical before/after hashes are checked. The experiment policy was fsynced
before information calculations. Stage15 selections were regenerated exactly.

Exactly five own frozen Stage15 trajectories supply primary linearizations:

| Code | Condition | Pose artifact |
|---|---|---|
| F0 | Full75 | P0_K75.csv |
| S1 | Coverage K40 | P2_K40.csv |
| S2 | Quality K40 | P1_K40.csv |
| S3 | Quality+Coverage K40 | P3_K40.csv |
| S4 | Coverage K20 | P2_K20.csv |

None of these five conditions was reoptimized. Exact graph reconstruction uses
5,796 nodes, 5,794 odometry factors, one unchanged R1#150 prior and the specified
loop subset. Full75 is a clean numerical/measurement reference, **not GT**.

## Measurement and API audit

Residual `E = Z^-1 Xq^-1 Xc`, `e = Pose3.Logmap(E)`, tangent ordering
`[rx,ry,rz,tx,ty,tz]`. Physical matrix translation and rotation angle are reported
separately from Logmap translation coordinates. Sigmas remain `[5deg]*3+[1m]*3`;
loop robustification remains Huber1.345, odometry nominal Gaussian.

Public Gaussian-factor `.linearize().jacobian()` works. Its approximate local
chart derivative is **not assumed** to be the exact Logmap residual derivative:
the largest recorded difference is 13.576258. Measurement Jacobians use public
`Pose3.LogmapDerivative(E)` times the endpoint `between` Jacobians. Central
right-retraction finite differences at epsilon1e-6 check every measurement;
maximum error across five conditions is 4.010e-8, below frozen1e-5 gate.
The actual graph's historical GTSAM linearization/noise is not modified.
This distinction is retained in per-loop diagnostic columns.

`Marginals.jointMarginalCovariance(KeyVector)` and `JointMarginal.at` work.
12x12 covariance is assembled explicitly in query/candidate order from all four
6x6 blocks, preserving cross-correlation. A toy graph verifies fullMatrix and
single-marginal agreement and nonzero cross-blocks. No block-diagonal substitute
or global dense6N Hessian is constructed. Every queried pair is finite,
symmetric and PSD-valid; no jitter or eigenvalue clipping forces a PASS.

For nominal local `H=J.T R^-1 J`, information proxy is the Cholesky-evaluated
`0.5 logdet(I + D^-1 J Sigma J.T D^-1)`, D=diag(sigmas), in nats.
Predicted covariance `S=J Sigma J.T+R`; NIS=`e.T solve(S,e)` is diagnostic,
not a chi-square test. Reconstructed Huber weight is
`min(1,1.345/norm(e/sigma))`, explicitly a **diagnostic**, not an asserted exact
internal robust weight. Nominal and weight-scaled information are both retained.

Weak support uses largest/top3 joint covariance eigenvectors, Euclidean-normalized
in native right-retraction rad/metre coordinates, and `v.T H v`. These mixed-unit
directions are coordinate-dependent endpoint-pair diagnostics, not global Hessian
nullspace directions. Marginals under this frozen model are **not calibrated
physical trajectory uncertainty**. Comparing different optimized linearization
points also mixes information and nonlinear representation changes.

## Primary Coverage K40 contrast

Frozen map median/p95 is 3.098681/119.062359m versus Full75
3.030506/119.277542m. The35 omitted loops nevertheless retain physical
translation/rotation p95 **3.956524m/14.071211deg**. All75 K40 p95 is
2.786279m/11.297416deg; do not confuse this with the omitted-only cohort.

Omitted DeltaI ranges4.047331–10.950431, median8.427143 nats; selected40
in-graph median2.034651. The latter is **IN_GRAPH_LOCAL_INFORMATION_DIAGNOSTIC**,
not pre-insertion gain, removal contribution or proof of redundancy. The
selected/omitted difference partially follows from conditioning on factors
already present and cannot establish causal necessity.

Among omitted35, Spearman correlations with DeltaI are:

| Diagnostic | Spearman |
|---|---:|
| GICP quality | 0.164706 |
| Frozen structural novelty | 0.683754 |
| Nearest selected normalized pair distance | 0.933053 |
| Translation residual | 0.343417 |
| Rotation residual | 0.264986 |
| Weak support top3 | 0.460504 |
| Weak support1 | 0.694958 |
| NIS | -0.031092 |

Thus quality alone is weakly associated with information. Structural distance
does carry information within the omitted cohort, but excellent global pair
coverage does not bound every missing factor's local information. Across all75,
novelty/DeltaI Spearman is -0.617468 and quality/DeltaI0.046629; this mixes
selected in-graph and omitted proxies and is not an equivalent-gain comparison.

NIS omitted median0.212571, p950.831832, max2.107871. Larger residuals are not
necessarily higher normalized surprise when graph-predicted uncertainty is
large; NIS cannot identify physically noisy/incorrect loops without an external
model validation. No causal interpretation or independent-loop p-values is used.

Median omitted top3 weak support is1.159e-6 versus selected1.574e-8 in the
declared mixed-unit convention. Absolute values are small: the largest endpoint
covariance modes can be common-motion modes poorly observed by relative factors.
Do not call correlation alone strong global observability evidence.

## Frozen regions and representative cases

Before scores, Stage14 regions were frozen to R3KF1208–1408,1535–1735, and
matched control windows452–652,2403–2603,451–651. No clean-loop endpoints occur
in either high-disagreement window or the controls around551/552. This is an
important limitation, not evidence of absent weak uncertainty. The2503 control
window has four loops, one omitted under K40 (DeltaI8.572613); its sparse/full
largest-eigenvalue ratio is1.004753. Per-pose marginals at all six fixed centers
are retained for five references and ten trials, regardless of endpoint coverage.

Representative cases use predeclared all75 median splits (novelty0.019115,
DeltaI2.061500) and lowest frozen ID per supported category. They are descriptive,
not hand-picked winners. All metrics and separate DeltaI/NIS/weak-support rankings
are in the machine-readable case tables.

## Ten independent add-backs

TOP5 IDs: **105,53,55,100,48**. BOTTOM5: **61,114,62,110,77**.
They were selected automatically by frozen DeltaI direction, then lower ID ties.
Each graph adds exactly one omitted factor to the original K40, from identical
canonical first-bridge x0; no K40 warm start or cumulative addition. Each has
5,796 nodes/5,794 odometry/41 loops/one prior, seven LM iterations, finite poses
and frozen anchor. Solve runtimes are376.584–402.472ms; ten-sample diagnostic
costs are not streaming iSAM2 latency.

| Actual effect (group median) | TOP5 | BOTTOM5 |
|---|---:|---:|
| Added loop translation reduction (m) | 1.304242 | 0.535291 |
| Added loop rotation reduction (deg) | 6.113094 | 3.449651 |
| All75 translation p95 reduction (m) | 0.000294 | 0.000188 |
| All75 rotation p95 reduction (deg) | -0.000170 | 0.110934 |
| Trajectory translation p95 change (m) | 0.136572 | 0.194718 |
| Original weak-direction variance fraction reduction | 0.006471 | 0.001758 |

Spearman DeltaI vs added-loop translation/rotation reduction:
0.539394/0.551515; all75 translation/rotation p95 reduction:
0.466667/-0.224242; trajectory displacement0.212121; original weak-direction
variance fraction reduction0.745455. These n10 associations are mixed and
depend on shared variables; n5 group medians do not imply significance.

Loop105 improves all75 translation p95 from2.786279 to2.437064m, but the
TOP5 median global-tail gain is tiny and rotation-tail improvement is not
consistently better. Add-back maps remain near the control (medians
3.087282–3.100006m, p95119.018479–119.116535m). Remaining34 omitted and
all75 median/p90/p95/max residuals are retained separately per trial.

A failed first A0 **reporting** attempt was archived (helper returned a tuple).
The interface was corrected; recovery repeated that one identical trial and
then completed all ten under unchanged policy. Five information references
were not recomputed. Canonical recorded timings are the recovery trials, not
chosen fastest runs. Report-only refinements do not alter experiment metrics.

## Full75 covariance reference and explanation

Across identical35 pairs, sparse/full joint max-eigenvalue ratio median1.000874;
translation-trace ratio1.001249, rotation-trace1.017592. Joint logdet sparse-minus-
full median17.252976. Thus dominant common-mode uncertainty changes little while
other relative modes/information volume can change substantially. Full75 does
not calibrate physical uncertainty. Both joint cross-correlation and measurement
projection matter; independent endpoint blocks would misrepresent this contrast.

Global map NN summarizes sampled surface agreement, not consistency against
every omitted six-DOF measurement. Spatially distributed selected constraints
can preserve aggregate map NN while particular relative configurations remain
weak/inconsistent. Information heterogeneity and target-loop add-back effects
make information loss a plausible **partial** explanation, but not a sufficient
explanation of global residual tails. No nonlinear entropy/global observability
proof, no physically necessary-loop claim and no sparse policy PASS follows.

The predeclared conservative decision requires multiple material outcome
metrics for full explanation. Only target translation met the TOP-vs-BOTTOM
materiality gate; global-tail and covariance effects were mixed/small. Therefore
the decision is **MEASUREMENT_INFORMATION_PARTIALLY_EXPLAINS_SPARSE_FAILURE**.
Stage15's negative selection and supplementary rotation-screen caveats persist.

## Resources, artifacts and stop point

Fifteen bounded workers succeeded; peakRSS435.652MiB. Each had an extra2GiB
address-space limit,90s CPU and120s wall cap, single-threaded BLAS. Fifteen
Marginals constructions total615.719ms;385 joint queries598.590ms;385 measurement
linearizations151.809ms; ten LM solves3.953s, map NN26.973s and map transform/
voxel15.512s. Worker totals overlap phase costs; never sum both as independent
costs. Factor-reconstruction timing includes read/verification/pose loading.
Reporting, resource statuses and any recovery are separately recorded.

Ten required figures, small endpoint arrays (6x12/12x12), ten trajectories,
policy/provenance/decision, individual and group add-back results are retained.
No dataset/model/cache is committed. Independent auditor checks historical
hashes, analytic metrics from saved arrays, physical transforms, automatic
top/bottom selection, K41 residuals/schema/anchor and resources (3,632 checks).

Stage16 is a **local linearized information audit**, not a global observability
proof. A possible next separately predeclared study may test measurement-space
relative uncertainty and robust/nonlinear residual geometry; it must not
automatically create an information-greedy selector, retune thresholds or alter
the demo. Stop after this audit.
