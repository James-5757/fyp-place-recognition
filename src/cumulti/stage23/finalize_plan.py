"""Finish the Phase0 planning document and independent artifact consistency audit."""
import preflight as p
from preflight import *

def main():
    report=read(OUT/'stage23_preflight_report.json');manifest=read(OUT/'stage23_input_manifest.json')
    inventory=pd.read_csv(OUT/'dataset_route_inventory.csv');coverage=pd.read_csv(OUT/'bidirectional_overlap_coverage.csv');audit=pd.read_csv(OUT/'stage22b_missing_loop_audit.csv');info=pd.read_csv(OUT/'missing_loop_linearized_information.csv')
    # Explicitly repair metadata schema interpretation, not any historical file.
    r3=pd.read_csv(PROC/'full_2hz/robot3/keyframes.csv',usecols=['gt_sync_error_ns'])
    inventory.loc[inventory.recording.eq('robot3'),'GT_sync_abs_max_ms']=float(abs(r3.gt_sync_error_ns).max()/1e6);csv('dataset_route_inventory.csv',inventory)
    report['reason']='Frozen stable R1/R3 fails the bidirectional/dual-progress targets. R2/R4 is recording-disjoint but R4 has GT only, and stable odometry starts for R2/R4 are not validated. No qualified independent development/final-test split is available; targets are unchanged.'
    report['candidate_final_test']='Conditional R2/R4 recording-disjoint candidate: full-recording GT coverage passes, but actual R4 sensors/odometry and stable starts are missing or unaudited. Same-campus recordings do not demonstrate cross-route generalization.'
    report['independence_definition']='Source-disjoint per-robot logs, independent missions/sessions and geographic routes are distinct. R2/R4 has disjoint robot logs, but independent acquisition-session provenance is NOT established. A different geographic route is preferred, not silently added as a mandatory numerical eligibility target.'
    raw_entries=[dict(path=str(x),bytes=x.stat().st_size,mtime_ns=x.stat().st_mtime_ns,integrity_scope='presence/size metadata only; not a full archive CRC or raw-data hash rescan') for x in sorted(RAW.rglob('*')) if x.is_file()]
    csv('raw_file_inventory.csv',raw_entries)
    manifest['raw_files_metadata_only']=raw_entries
    manifest['calibration_archive_sha256']=sha(RAW/'calib/robot_description.zip')
    for n in [P0,P5]:
        policy=B/'policies'/n/'online_backend_policy.json';manifest['inputs'][str(policy)]=dict(bytes=policy.stat().st_size,sha256=sha(policy))
    dump('stage23_input_manifest.json',manifest)
    policy=dict(status='DRAFT_ONLY_NO_SELECTOR_IMPLEMENTED',online_feature_time='after current sensor/odometry arrival, before any current GICP outcome; state from past admitted factors only',scheduler='frozen timestamp-normalized deterministic events; no future descriptor/state/outcome; freeze eligibility and splits before selector work',P0='unchanged frozen full rank-ordered Top3 reference',P3='unchanged Stage22B SC confidence baseline; original normalization and threshold',P5='unchanged Stage22B C75/B80 reference; no hindsight re-selection',P6_INFO='p_hat_GICP * 0.5logdet(I6 + R_nom^-1/2 J_pred C_pair_past J_pred.T R_nom^-1/2) divided by exact marginal request bytes (minimum positive framing cost); include compute cap separately',P6_COVER='causal odometry arc-length endpoint facility-location marginal coverage per marginal byte; no GT or future trajectory',P6_HYBRID='fixed nonnegative combination of predicted information, odometry coverage and regional graph connectivity; predeclared development-only calibration; shared resource caps',P7_TIAN='Tian-inspired resource-aware budgeted submodular greedy comparison over a frozen arrived-only snapshot, verification-edge count and shared observation-upload bytes; not a claimed exact theorem-certified reproduction of the original algorithm',p_hat='optional calibrated geometric-quality acceptance probability trained only on genuinely independent development actual GICP outcomes; not GT physical correctness and not unmeasured outcome imputation',pre_bridge='cross-component connectivity mode; no invertible joint common-frame covariance is claimed before a bridge. Use a separately frozen SC-confidence/cost bridge rule, never a future optimized transform.',post_bridge='current causal ISAM2 prefix pose/covariance plus propagated arriving odometry node. Predict Z from causal poses and frozen SC yaw hypothesis; nominal noise fixed. Never insert measured current-GICP Z into pre-GICP features.',coverage_formula='F_cov(S)=sum over currently arrived robot/path anchors u of w_u * max_{e in S} exp(-abs(s_u-s_endpoint(e))/ell_robot); s from nonGT local odometry; anchor spacing/kernel scales frozen on development without GT',connectivity='component reduction in currently available robot/odometry-region graph; region coverage distinct from global graph connectivity',cost='same serializer and cache semantics as baseline; exact marginal cloud-request union including framing/cache eviction; RingKey bytes alone are not total bytes',Tian_reference='https://arxiv.org/abs/1907.04904',Tian_DOI='10.1177/0278364920948594',theorem_caveat='monotone logdet of fixed PSD contributions and fixed-universe facility location can be submodular. Evolving posterior covariance, relinearization, robust weights, finite-cache costs and streaming deadlines do not automatically preserve a cited approximation guarantee. Review original pseudocode and prove equivalence before using guarantee language.',selection_tie_break='deterministic event/rank/original IDs, frozen before future execution',unknown_rank_rule='new real GICP required when selected; 2112 unknown ranks are never inferred from SC/GT/neighbors',stage23B_authorized=False)
    dump('stage23_protocol_draft.json',policy)
    missing=report['missing_loop_diagnosis'];registry=report['candidate_registry'];cost=report['leave_one_out_cost_estimate']
    stable=coverage[(coverage.variant=='FROZEN_STARTS_R1_R3_OTHERS_UNAUDITED')&(coverage.route_pair=='robot1-robot3')]
    r1=stable[stable.query_robot=='robot1'].iloc[0];r3row=stable[stable.query_robot=='robot3'].iloc[0]
    assert [int(r1.positive_overlap_queries),int(r3row.positive_overlap_queries)]==[59,1411]
    assert [int(r1.both_progressing_positive_queries),int(r3row.both_progressing_positive_queries)]==[59,0]
    assert (audit.membership.eq('P0_ONLY')&audit.GT_positive).sum()==missing['P0_true_factors_absent_P5']
    assert missing['P0_true_factors_absent_P5']-missing['P5_only_true_factors']==57
    assert len(info)==missing['P0_true_factors_absent_P5'] and np.isfinite(info[['nominal_DeltaI_nats','diagnostic_robust_DeltaI_nats']]).all().all()
    registry_table=pd.read_csv(OUT/'candidate_measurement_coverage.csv');assert len(registry_table)==17379 and registry_table.never_measured.sum()==2112
    assert not report['selector_implemented'] and report['full_GICP_runs']==report['full_pose_graph_optimization_runs']==0
    assert historical_digest()==manifest['historical_before']
    for path,entry in manifest['inputs'].items():assert sha(path)==entry['sha256']
    # The declared dirty state must remain exactly the same among tracked files.
    changed=subprocess.check_output(['git','diff','--name-only'],cwd=ROOT).decode().splitlines()
    assert set(changed)=={'demo/final_replay/app.js','outputs/cumulti_v1/10_3_solver_audit/nontrivial_small_graph_sanity.json'}
    assert subprocess.check_output(['git','branch','--show-current'],cwd=ROOT).decode().strip()==manifest['branch']
    report['independent_validation']='PASS: frozen dataset/input hashes, both-direction counts, identity-set arithmetic, unknown-rank accounting, finite sparse information and historical digest all agree'
    report['linearized_information_summary']={k:dict(median=float(info[k].median()),p95=float(info[k].quantile(.95)),max=float(info[k].max())) for k in ['nominal_DeltaI_nats','diagnostic_robust_DeltaI_nats']}
    dump('stage23_preflight_report.json',report)
    def table(df,cols):
        rows=['| '+' | '.join(cols)+' |','| '+' | '.join(['---']*len(cols))+' |']
        return '\n'.join(rows+['| '+' | '.join(str(row[c]) for c in cols)+' |' for row in df.to_dict('records')])
    doc=f'''# Stage23 — Information-aware, Coverage-preserving Loop Selection

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

{table(inventory,['recording','keyframes','duration_s','point_cloud_files','GT_sync_abs_max_ms','RGB_sync_available'])}

These are different robot recordings of the same campus, not physically
simultaneous streams. Each exact original LiDAR timestamp subtracts its own
first retained timestamp; frozen Stage20 tie ordering applies. Both-progressing
is the inclusive prefix through the first robot's final sensor event; later
queries are classified after either finishes. No arbitrary start offsets were
optimized to improve eligibility. Full all-database overlap is also reported
but must not be substituted for causal availability.

Frozen stable R1/R3 counts reproduce Stage22A/22B:

{table(stable,['query_robot','queries','positive_overlap_queries','both_progressing_positive_queries','after_either_finishes_positive_queries','synchronized_header_valid_cloud_pairs'])}

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
Sparse diagnostic elapsed{report['linearized_diagnostic']['elapsed_s']:.3f}s,
peak process RSS{report['linearized_diagnostic']['peak_RSS_mib']:.2f}MiB.
No dense global Hessian was formed.

## C. Candidate measurement registry

{registry['total_candidates']} causal event/rank candidates;15267 have actual
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

Historical measured P0 Batch LM={cost['reference_measured_batch_ms']:.3f}ms.
**Modelled**, not measured:142 solves approximately
{cost['solver_only_serial_seconds_1x']:.2f}–{cost['solver_only_serial_seconds_10x']:.2f}s
under1–10x historical solve cost. This excludes map projection/NN, metrics, I/O
and convergence variation. Proposed future serial cap1800s/RSS2GiB is a draft,
not authorization to launch. A global dense34776-square float64 Hessian would
require{cost['dense_Hessian_bytes_to_avoid']/1e9:.3f}GB even before factorization;
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
'''
    (ROOT/'docs/CUMULTI_STAGE23_INFORMATION_AWARE.md').write_text(doc)
    artifacts=[dict(path=str(x.relative_to(ROOT)),bytes=x.stat().st_size,sha256=sha(x)) for x in sorted(OUT.iterdir()) if x.is_file() and x.name!='stage23_artifact_manifest.json']
    sources=[dict(path=str(x.relative_to(ROOT)),sha256=sha(x)) for x in (SRC/'stage23').glob('*.py')]
    dump('stage23_artifact_manifest.json',dict(outputs=artifacts,sources=sources,documentation_sha256=sha(ROOT/'docs/CUMULTI_STAGE23_INFORMATION_AWARE.md'),launcher_sha256=sha(ROOT/'scripts/run_stage23_information_aware.sh')))
    print('STAGE23_PHASE0_COMPLETE: NO_GO; no selector/GICP/optimization/commit',flush=True)
if __name__=='__main__':main()
