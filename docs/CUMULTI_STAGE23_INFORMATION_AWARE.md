# Stage23 — Information-aware, Coverage-preserving Loop Selection

## Phase0 / Stage23A decision: NO-GO for the full experiment

This separately authorized **preflight is complete**, not a selector result.
Unchanged references: Stage22B `bd3a67859c9412c8aab8ed8da66d1cf96f77101e` and
Stage22A `c929fc55a9f42c149ce571205618da73db7d4bc5`.
No full GICP replay, nonlinear graph optimization, new selector, sensor download,
large archive extraction, demo modification or historical rewrite was performed.
NEXT_STEPS.md is unchanged. Existing two tracked modifications and untracked
benchmark_before directory were reported and preserved. Branch remains main;
the requested branch creation was conditional on a clean worktree, which was
not satisfied. Nothing is staged or committed by this Phase0 runner.

## A. Dataset and route eligibility

Targets are unchanged: >=100 causal positive queries per direction overall,
>=50 per direction while both robots progress, and independent development/test
recordings. Positive definition remains **d_xy<5m**, GT eligibility analysis only.
All available raw recording inventory is MainCampus robots1/2/3 sensors and
robot4 GT-only; no second geographic route was found locally.

| recording | keyframes | duration_s | point_cloud_files | GT_sync_abs_max_ms | RGB_sync_available |
| --- | --- | --- | --- | --- | --- |
| robot1 | 2000 | 999.535027661 | 2000 | 24.845815 | True |
| robot2 | 2230 | 1114.544274123 | 2230 | 24.720846 | False |
| robot3 | 4180 | 2089.511053803 | 4180 | 24.795275 | True |
| robot4 | 5544 | 2771.544088832 | 0 | nan | False |

These are different robot recordings of the same campus, not physically
simultaneous streams. Each exact original LiDAR timestamp subtracts its own
first retained timestamp; frozen Stage20 tie ordering applies. Both-progressing
is the inclusive prefix through the first robot's final sensor event; later
queries are classified after either finishes. No arbitrary start offsets were
optimized to improve eligibility. Full all-database overlap is also reported
but must not be substituted for causal availability.

Frozen stable R1/R3 counts reproduce Stage22A/22B:

| query_robot | queries | positive_overlap_queries | both_progressing_positive_queries | after_either_finishes_positive_queries | synchronized_header_valid_cloud_pairs |
| --- | --- | --- | --- | --- | --- |
| robot1 | 1850 | 59 | 59 | 0 | 901 |
| robot3 | 3946 | 1411 | 0 | 1411 | 25643 |

R1 overall59 fails100; R3 dual-progress0 fails50. R3's1411 positives are all
after R1 finishes. Dividing this known route into new time windows does not
create independent final-test data. Full-recording common initialization
positives are diagnostic only: they cannot rehabilitate previously excluded
unstable EKF portions or manufacture a qualified split.

All six unordered / twelve directed robot pairings are in
`bidirectional_overlap_coverage.csv`, for both full-recording diagnostics and
the frozen R1/R3 retained starts. Robot2 has raw sensors and2230 header-valid
clouds but no audited stable local-odometry cache. Robot4 has5544 sampled GT
poses and **zero local sensor clouds**. R2/R4 is recording-disjoint from R1/R3;
its full-recording causal positive counts are316/1596, dual-progress316/77.
It is a **conditional acquisition candidate**, not an eligible benchmark:
R4 real sensor timestamps/odometry are missing and neither new stable prefix is
validated. A different geographic route is preferred for stronger transfer,
but geographic novelty is not silently added as a new mandatory numeric target.
Other pairings share an exposed R1/R3 recording with current development.
Disjoint robot IDs/log files do not themselves prove independent acquisition
sessions or missions; that provenance is unresolved for R2/R4. The raw-file
inventory includes auxiliary depth/label archives where locally present.

GT-only robot4 sampling uses exact Decimal seconds-to-ns conversion and a fixed
0.5s grid (first GT at/after grid); this is not a fabricated LiDAR sequence.
Valid-pair counts mean causal GT-positive associations with both NPY headers
valid float32 Nx4 and existing nearest-GT associations. They do NOT imply
physical simultaneity or a new full point-finiteness scan. No new sync tolerance
was fitted. Original maximum GT offsets and RGB sync distributions are recorded.
ROS2 topic names/message counts come from actual ZIP metadata without extracting
bags; URDF joints/extrinsics and frozen LiDAR/os_sensor graph convention are in
`dataset_sensor_metadata.json` / `coordinate_frame_audit.json`. UTM zone/EPSG and
R2/R4 raw message-frame/stability details remain unresolved where not evidenced.

Eligibility conclusion: **NO-GO**. No currently available pair of independent,
sensor-complete, stable and bidirectionally qualified dev/final-test recordings
has been established. Do not weaken100/50 or rebalance exposed data after GT.

## B. Critical-loop retrospective identity audit

The335 fewer geometrically true-positive **query events** are1056−721.
The57 fewer admitted true **factors** are280−223. They are not the same unit.
Canonical identity is(R1 original KF,R3 original KF), with measured transforms
preserved. Exact sets reveal:

- P0-only factors:141, all GT-positive under d_xy<5m.
- Common factors:147 (139 GT-positive and8 GT-negative).
- P5-only factors:84, all GT-positive.
- Therefore net true factor loss is **141−84=57**, not57 identifiable missing
  P0 factors. Causal duplicate representative changes explain why set churn
  must be analyzed separately from the net count.

The141 missing P0 true factors form7 time clusters at a diagnostic30s maximum
inter-factor gap. Largest cluster48; another has40. All occur after R1 finishes
(first1146.50s, last1823.01s in normalized time). They span multiple R1 revisit
regions rather than a single localized section. Under fixed50m frozen-local-
odometry pair cells,8 are singleton P0 region constraints;4 lie in cells with
no P5 factor. This is coarse path-coverage evidence, not proof of physical
region uniqueness or causal necessity. No missing factor is the first bridge.
With two full odometry chains and many cross-robot edges, no individual missing
edge is a global graph-theoretic cut edge; regional connectivity is different.
The8 false admitted factors are retained in common: this negative control must
not disappear behind a favorable call-count saving.

Cheap retrospective information: reconstruct the **unchanged P5 graph**, use
its saved final batch poses and GTSAM sparse Marginals; no optimization.
For141 P0-only true measured factors, retain the joint12x12 endpoint covariance
including cross blocks. With E=Z^-1 Xq^-1 Xc and exact Pose3 Logmap Jacobian J,
nominal diagnostic DeltaI=0.5logdet(I+R^-1/2 J C J^T R^-1/2).
A separately labelled reconstructed Huber-weighted version is also supplied.
Both covariance and Z are **offline retrospective evidence**, not features
available to a pre-GICP selector; values are not calibrated physical uncertainty.
This is hypothetical factor ADDITION to the saved P5 linearization, not a
measured nonlinear effect of REMOVING the factor from P0. Only the separately
planned matched-graph ablation can test that effect.
Sparse diagnostic elapsed0.230s,
peak process RSS164.12MiB.
No dense global Hessian was formed.

## C. Candidate measurement registry

17379 causal event/rank candidates;15267 have actual
historical Stage22A measurements. Stage22B reexecuted313 unique candidate
identities;14954 have only their original Stage22A receipt available.
**2112 were never measured**, principally ranks skipped by earlier acceptance.
Every existing receipt and raw result CSV hash is verified. There are zero new
Stage23 registrations. A reordered policy could require up to2112 previously
unknown pair outcomes; the actual subset cannot be known until a separately
frozen policy chooses them. No SC score, neighboring edge, GT or archive
acceptance is allowed to stand in for real GICP. Same-pair reuse requires exact
input clouds/shift/config/direction hashes and distinct timing-evidence labels.

## D. Draft causal policies — not implemented

P0/P3/P5 remain the exact frozen full Top3, SC-confidence and C75/B80 references.
All proposed features are computed before current GICP: current arrived scans,
SC/ring-key values, source odometry, own cache/request history and prior admitted
graph factors only. No future state, future result or GT label is online input.

- **P6-INFO:** predicted geometric-acceptance probability times nominal
  measurement-projected information gain per exact marginal byte. Post-bridge
  joint covariance comes from the current causal prefix, with the arriving
  pose propagated from local odometry. Predict a measurement/Jacobian from
  causal poses/SC yaw, not the unavailable actual current GICP transform.
- **P6-COVER:** odometry arc-length endpoint facility-location coverage.
  For currently arrived anchors u, Fcov(S)=sum_u w_u max_e kernel(|s_u−s_e|).
  Freeze anchor spacing/kernel scales on independent development without GT.
  No future trajectory is used to define the anchor universe.
- **P6-HYBRID:** nonnegative combination of predicted information, marginal
  coverage and current regional/component connectivity, under both call/byte
  budgets. Freeze weights/ties using development only, never GT performance.
- **Before first bridge:** two local gauges have no valid common-frame joint
  covariance. Use an explicitly frozen connectivity/SC-confidence bridge mode;
  do not invent a joint covariance or use a future optimized alignment.
- **P7-TIAN:** draft a resource-aware budgeted submodular greedy baseline with
  candidate edges as verifications and observation vertices as shared uploads.
  Use a frozen arrived-only snapshot, fixed nominal PSD contributions and
  facility-location utility; feasibility checks must charge exact marginal
  shared cloud/cache bytes and verification count. This is a Tian-inspired
  streaming adaptation, not yet a theorem-certified reproduction.

Tian, Khosoussi and How formulate monotone-submodular loop selection under
communication/computation budgets ([primary paper](https://arxiv.org/abs/1907.04904),
[DOI](https://doi.org/10.1177/0278364920948594)). Full original pseudocode and
guarantee conditions must be reviewed before implementation. A changing causal
graph, state-dependent robust weights, adaptive outcome feedback and finite
cache eviction do not automatically inherit the original guarantee. The exact
paper baseline and any adapted heuristic must be separately named. A predicted
quality-acceptance probability is not GT correctness or an imputed GICP result.

## E. Planned critical-factor ablation and resource estimate

`leave_one_out_plan.csv` contains141 exact P0 true-factor removals, not57.
One matched full-P0 reference plus141 Batch cases means142 solves. Rebuild the
same full node/odometry/prior/noise graph, changing exactly one loop per case.
Use common retained initializer R1KF250↔R3KF235 for every case; it is never a
removed factor. Freeze same-gauge diagnostics, finite/anchor/jump checks,
measured-loop residual tails, fixed-protocol map NN and regional effects first.
Only after GT-free outcomes/decisions are saved, report offline joint-aligned
ATE and separate/union interval10 RPE. Do not label a loop critical from GT alone.
Rank correlations of retrospective DeltaI and ablation effect are exploratory,
not an accepted selector or proof of physical observability.

Historical measured P0 Batch LM=380.077ms.
**Modelled**, not measured:142 solves approximately
53.97–539.71s
under1–10x historical solve cost. This excludes map projection/NN, metrics, I/O
and convergence variation. Proposed future serial cap1800s/RSS2GiB is a draft,
not authorization to launch. A global dense34776-square float64 Hessian would
require9.675GB even before factorization;
use sparse elimination and small joint blocks. Future PLY storage should be
bounded/manifests-only, not one huge cached raw map per case. No LOO was run.

## Recommended next action and unresolved risks

Acquire/qualify an independent recording pair with useful causal overlap in
both directions while both streams progress, plus odometry/calibration/sync
evidence. R2/R4 could be audited further with small odometry/topic inventory
first, but R4 sensors and valid stable starts remain prerequisites; no download
was started. Prefer another route/recording for confirmatory generalization.
R1/R3 is already exposed and quantitatively fails the eligibility targets.

Risks: unstable initialization can inflate full-route overlap; GT-only sampling
does not prove sensor availability; source odometry coverage drifts; expected
information can favor high uncertainty without reliable matching; P5 removes
true factors while retaining false ones; missing measurements prevent a fair
reordered counterfactual; and no theoretical or empirical information-aware
success has yet been demonstrated. Historical Stage15 negative sparse-policy
and Stage22B incomplete-generalization outcomes remain unchanged.

Independent artifact consistency checks PASS. SHA256 input manifests and
before/after digests over64530 historical files agree. Run
`bash scripts/run_stage23_information_aware.sh` only against a fresh Stage23
output directory; it refuses to overwrite an existing audit. Stop after Phase0.
