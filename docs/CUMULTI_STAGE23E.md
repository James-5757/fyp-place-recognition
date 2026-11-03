# Stage23E — P6 graph reconstruction and mapping quality

## Separate final statuses

```json
{
  "P6_GRAPH_RECONSTRUCTION": "PASS",
  "P6_INCREMENTAL_BATCH_AGREEMENT": "FAIL",
  "P6_RETROSPECTIVE_MAPPING_QUALITY": "NO_POLICY_PASSED_ALL_ORIGINAL_GATES",
  "P6_COMMUNICATION_EFFICIENCY": "ALL_PASS_P0_COST_GATES_BUT_ALL_COST_MORE_THAN_P5",
  "P6_INDEPENDENT_VALIDATION": "NOT_RUN",
  "STAGE23_FINAL_INDEPENDENT_VALIDATION": "NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE",
  "label": "RETROSPECTIVE_GT_EXPOSED_RECORDING",
  "new_GICP_calls": 0,
  "GT_based_retuning": false,
  "policy_promoted": false,
  "matched_budget_experiment": "NOT_RUN",
  "matched_budget_followup_candidates": [],
  "candidate_caveat": "lower incremental ATE on this exposed route only, not a selected policy, universal mapping improvement or matched-budget result",
  "baseline_metrics": "copied frozen P0/P5 tables; no re-evaluation",
  "stopping_point": "Stage23E complete; no automatic matched-budget or independent experiment"
}
```

This is **RETROSPECTIVE_GT_EXPOSED_RECORDING**. Independent geographical validation is NOT_RUN and eligibility remains NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE. Artifact correctness, numerical agreement, empirical quality and communication savings are separate decisions. No P6 policy is promoted, no matched-budget experiment is run, and no GICP is executed.

## Frozen inputs and experiment record
Reference Stage23D: `2fb55ad08f56cd74198c175d7116462b84a89427`. The server-only `p6_verified_factors.csv` exists and matches the specified440808bytes and SHA256 `476a490b787a2912e61b9f1e46424990b0786008780c9f7eaca6713799d30bab`, as well as its original artifact manifest. No scheduler rerun was needed. Queue/budget archives, source files, policies, odometry, raw scan hashes and original metric definitions are frozen in `23e_p6_graphs/stage23e_input_manifest.json` with a policy lock. All Stage18–23D scientific artifacts and original dirty work remain untouched; this new report records the experiment without rewriting historical documents.

Two pre-optimization engineering failures are retained: the GT-file guard initially rejected a Python source filename containing offline_, then metadata construction duplicated an already-frozen keyword. The guard now permits only reading source .py/.pyc files under src/cumulti while still rejecting GT data. The duplicate metadata keyword was removed. Neither failure called an optimizer or GICP. The original input manifest is preserved; narrowly allowlisted hashes of the two new Stage23E source repairs are recorded separately. Scientific parameters and original sources did not change. The pre-optimization partial directory is preserved, not relabeled a solve. A final metadata compatibility fix permits a clean replay with no repair journal (SHA=null). It changes no metric formula or accepted result; the original execution hash stays in the GT-entry file and the source amendment is recorded separately. No optimizer or GT evaluator was rerun for this fix.

## Reconstruction and insertion causality
All four isolated runners validate canonical IDs, proper SE(3), receipt SHA256 and CSV digest, exact canonical transform equality, endpoint arrival by discovery, and discovery<=insertion. Factors sort by insertion_event, discovery_event, rank, Robot1KF, Robot3KF. **Only insertion_event controls backend insertion.** Discovery remains query identity and controls the original dev/embargo/heldout classification. Deferred measurements are never inserted retroactively. Same-event processing order is the explicitly requested deterministic order, not an undocumented priority-queue order.

The first causally inserted frozen factor initializes the common frame using S=Xq Zqc inverse(Lc); no GT or highest-quality fallback chooses it. Prior: Robot1KF150, sigma1e-6. Every graph retains5796 nodes and5794 original odometry edges. Rotation/translation sigmas are5degrees/1m, tangent order rotation then translation, and loop Huber1.345. Odometry is unrobustified diagonal noise. One persistent default iSAM2, identical frozen parameter text, one update per connected arrival and zero extra empty updates. Batch LM uses the existing Stage22A/B–Stage12D defaults unchanged. The brief's Stage22D-compatible wording refers to these existing parameter files; no new Stage22D solver was invented.

Graph and solver results:

| policy | status | nodes | odometry | loops | factors | first_bridge_discovery_event | first_bridge_insertion_event | new_catastrophic_jumps | agreement_status | agreement_incremental_error | agreement_batch_error | agreement_batch_iterations | agreement_batch_runtime_ms | runtime_including_audit_state_export_maps_s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P6_D0 | PASS | 5796 | 5794 | 246 | 6041 | 203 | 203 | 0 | FAIL | 57.6681 | 57.6677 | 7 | 361.099 | 131.715 |
| P6_D1 | PASS | 5796 | 5794 | 223 | 6018 | 200 | 250 | 0 | PASS | 57.444 | 57.4432 | 7 | 367.922 | 130.694 |
| P6_D2 | PASS | 5796 | 5794 | 215 | 6010 | 204 | 204 | 0 | PASS | 52.2128 | 52.2125 | 7 | 361.521 | 131.349 |
| P6_D3 | PASS | 5796 | 5794 | 211 | 6006 | 204 | 204 | 0 | PASS | 52.5932 | 52.593 | 7 | 366.77 | 130.751 |

Incremental/Batch pose agreement, same graph and gauge; original limits0.10m median,0.50m p95,0.5degrees median,2degrees p95 are not relaxed:

| policy | agreement_status | agreement_translation_median_m | agreement_translation_p95_m | agreement_rotation_median_deg | agreement_rotation_p95_deg |
| --- | --- | --- | --- | --- | --- |
| P6_D0 | FAIL | 0.141895 | 0.274319 | 0.0485245 | 0.140114 |
| P6_D1 | PASS | 0.0708076 | 0.248363 | 0.0262008 | 0.13602 |
| P6_D2 | PASS | 0.0520946 | 0.260172 | 0.0179043 | 0.140175 |
| P6_D3 | PASS | 0.0654151 | 0.255947 | 0.0243865 | 0.136754 |

A structural PASS does not imply numerical agreement. Similar objective values do not establish pose equality. Objectives across DIFFERENT loop graphs are not direct mapping-accuracy measures. Batch runtime is native optimize time only; whole replay runtime additionally includes event extraction, integrity auditing, archive compression and map generation. Exact latency distributions are in `p6_backend_latency_summary.csv`, not an end-to-end real-time claim.

## Every-event trajectories and maps
Every one of5796 events per successful policy has a real float64 Nx4x4 state in the server-only ZIP, with entry shape/hash and prefix last IDs in the event trace. Rows are Robot1 ascending then Robot3 ascending. Before the first bridge the archive is explicitly LOCAL_ONLY: two independent local frames, not an overlaid common graph. After the bridge snapshots are the actual current iSAM2 estimates, without interpolation, future-state morphing or extra updates. CRC of all ZIP entries is checked; the independent auditor additionally inspects all insertion-event matrices, pre/at-bridge states, periodic500 states and FINAL. Backend runtime checks proper SE(3) at every event.

Actual incremental AND Batch maps use the same580 original scans, frame stride10, point stride16, range1–60m and per-source voxel0.5m. Each maps/ directory contains incremental_robot1.ply, incremental_robot3.ply, incremental_merged.ply, batch_robot1.ply, batch_robot3.ply and batch_merged.ply, hashed in online_map_manifest.json. Original metric XYZ and gauge remain intact: no independent best fit, GT transform, Z flattening, scaling or cosmetic adjustment. Separate per-source clouds preserve the blue/orange rendering provenance convention; merged XYZ is an additional union voxel map. No frozen demo was edited.

## Offline evaluation ordering and definitions
Only after all four backend results, factor schedules, trajectories, event archives and maps were hashed in `all_graphs_pre_gt_freeze.json` was `offline_evaluation_entry.json` written and GT opened. There is no optimization after seeing GT. Frozen P0/P5 values are COPIED from Stage22B tables, never re-evaluated or overwritten.

Query precision/recall and admitted precision/admission recall use the original discovery event and the frozen causal available-positive d_xy<5m denominator. Original receipt correctness is independently checked against those GT pairs; query-level success includes geometrically accepted duplicates, while admission is reported separately. Counts are not collapsed by insertion event. Original split: dev0–999, embargo1000–1099, heldout1100–5795.

Factors crossing query-discovery and insertion split boundaries (descriptive only):

```json
{}
```

ATE uses precisely the Stage22B one joint global rigid alignment for each final system trajectory. Robot/scope/region subsets reuse that same alignment; there is no independent robot, region or heldout fit. Heldout/region accuracy describes subsets of the FINAL frozen estimate, not the causal estimate available at query time. Frozen Stage22B does not provide heldout graph ATE: no new baseline value is fabricated. The original gate uses full incremental joint ATE ratio, as in Stage22B; additional heldout graph values are diagnostic.

RPE interval10 uses the same SE(3) relative-error definition; samples are calculated separately for each robot and combined by union, not by copying a global value into robot rows. Both interval endpoints must lie within the requested scope/region. Empty samples are unavailable, not zero. The eleven pre-existing regions are the seven temporal groups and four non-GT local-odometry bin pairs. Region definitions are verified through the frozen Stage23D asset manifest. Existing sensor/reference-frame limitations are retained; no new extrinsic or GT convention correction is introduced.

Full query geometric metrics:

| policy | queries | positive_queries | geometric_accepted | geometric_true | false_geometric | precision | recall |
| --- | --- | --- | --- | --- | --- | --- | --- |
| P6_D0 | 5796 | 1470 | 795 | 766 | 29 | 0.963522 | 0.521088 |
| P6_D1 | 5796 | 1470 | 790 | 789 | 1 | 0.998734 | 0.536735 |
| P6_D2 | 5796 | 1470 | 779 | 777 | 2 | 0.997433 | 0.528571 |
| P6_D3 | 5796 | 1470 | 787 | 785 | 2 | 0.997459 | 0.534014 |

Heldout query geometric metrics:

| policy | queries | positive_queries | geometric_accepted | geometric_true | precision | recall |
| --- | --- | --- | --- | --- | --- | --- |
| P6_D0 | 4696 | 1411 | 751 | 723 | 0.962716 | 0.512403 |
| P6_D1 | 4696 | 1411 | 733 | 733 | 1 | 0.51949 |
| P6_D2 | 4696 | 1411 | 746 | 745 | 0.99866 | 0.527994 |
| P6_D3 | 4696 | 1411 | 751 | 750 | 0.998668 | 0.531538 |

Full admitted-factor metrics:

| policy | admitted | admitted_true | false_admitted | admitted_precision | admission_recall |
| --- | --- | --- | --- | --- | --- |
| P6_D0 | 246 | 239 | 7 | 0.971545 | 0.162585 |
| P6_D1 | 223 | 222 | 1 | 0.995516 | 0.15102 |
| P6_D2 | 215 | 213 | 2 | 0.990698 | 0.144898 |
| P6_D3 | 211 | 209 | 2 | 0.990521 | 0.142177 |

Incremental final-trajectory accuracy, full and heldout subsets (joint RPE means union of robot samples):

| policy | scope | robot_id | nodes | ATE_RMSE_m | RPE_samples | RPE_translation_RMSE_m | RPE_rotation_RMSE_deg |
| --- | --- | --- | --- | --- | --- | --- | --- |
| P6_D0 | full | joint | 5796 | 20.7094 | 5776 | 10.5101 | 6.93569 |
| P6_D0 | full | robot1 | 1850 | 16.389 | 1840 | 9.44542 | 5.39279 |
| P6_D0 | full | robot3 | 3946 | 22.4505 | 3936 | 10.9724 | 7.54957 |
| P6_D0 | heldout | joint | 4696 | 20.4276 | 4676 | 10.7387 | 7.0001 |
| P6_D0 | heldout | robot1 | 1300 | 16.6779 | 1290 | 9.43975 | 5.69433 |
| P6_D0 | heldout | robot3 | 3396 | 21.6921 | 3386 | 11.194 | 7.43751 |
| P6_D1 | full | joint | 5796 | 20.9084 | 5776 | 10.5143 | 7.02281 |
| P6_D1 | full | robot1 | 1850 | 16.3217 | 1840 | 9.45583 | 5.61122 |
| P6_D1 | full | robot3 | 3946 | 22.7424 | 3936 | 10.9742 | 7.59322 |
| P6_D1 | heldout | joint | 4696 | 20.6652 | 4676 | 10.7412 | 7.1042 |
| P6_D1 | heldout | robot1 | 1300 | 16.6663 | 1290 | 9.4457 | 5.98985 |
| P6_D1 | heldout | robot3 | 3396 | 22.0045 | 3386 | 11.1955 | 7.48523 |
| P6_D2 | full | joint | 5796 | 20.9114 | 5776 | 10.5082 | 7.03826 |
| P6_D2 | full | robot1 | 1850 | 16.5771 | 1840 | 9.44289 | 5.49071 |
| P6_D2 | full | robot3 | 3946 | 22.6598 | 3936 | 10.9708 | 7.65513 |
| P6_D2 | heldout | joint | 4696 | 20.6574 | 4676 | 10.7363 | 7.12642 |
| P6_D2 | heldout | robot1 | 1300 | 17.0485 | 1290 | 9.43713 | 5.83374 |
| P6_D2 | heldout | robot3 | 3396 | 21.8819 | 3386 | 11.1917 | 7.56098 |
| P6_D3 | full | joint | 5796 | 21.0919 | 5776 | 10.5091 | 6.98789 |
| P6_D3 | full | robot1 | 1850 | 16.8241 | 1840 | 9.44888 | 5.38946 |
| P6_D3 | full | robot3 | 3946 | 22.8196 | 3936 | 10.9697 | 7.62098 |
| P6_D3 | heldout | joint | 4696 | 20.8358 | 4676 | 10.736 | 7.06336 |
| P6_D3 | heldout | robot1 | 1300 | 17.2073 | 1290 | 9.44016 | 5.69108 |
| P6_D3 | heldout | robot3 | 3396 | 22.0675 | 3386 | 11.1903 | 7.52059 |

Per-region and Batch metrics remain fully available in the required CSVs. Loop residual summaries are GT-free measurement residuals, not GT pose errors:

| policy | condition | translation_median_m | translation_p95_m | rotation_median_deg | rotation_p95_deg | minimum_diagnostic_huber_weight |
| --- | --- | --- | --- | --- | --- | --- |
| P6_D0 | batch | 0.0850751 | 0.270955 | 0.343453 | 1.26524 | 1 |
| P6_D0 | incremental | 0.0849557 | 0.270903 | 0.34364 | 1.26603 | 1 |
| P6_D1 | batch | 0.092291 | 0.331035 | 0.375975 | 1.32744 | 1 |
| P6_D1 | incremental | 0.0928047 | 0.331091 | 0.374352 | 1.32739 | 1 |
| P6_D2 | batch | 0.0834545 | 0.237857 | 0.336074 | 1.21629 | 1 |
| P6_D2 | incremental | 0.0834604 | 0.237805 | 0.336635 | 1.21622 | 1 |
| P6_D3 | batch | 0.0862342 | 0.239337 | 0.327254 | 1.26445 | 1 |
| P6_D3 | incremental | 0.0862237 | 0.239337 | 0.327265 | 1.26432 | 1 |

## Resource/quality comparison and unchanged gates
Attempt savings and application-byte savings are relative to frozen P0. Extra bytes and ATE differences are relative to frozen P5. These are selected historical receipt workloads and protocol-simulated application bytes, not new GICP executions, physical networking or a measured complete frontend replay runtime.

| policy | GICP_receipt_attempts | admitted_factors | application_bytes | application_byte_reduction_vs_P0 | additional_bytes_vs_P5 | recall | precision | joint_ATE_RMSE_m | ATE_difference_vs_P5_m | map_NN_median_m | map_NN_p95_m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P0 | 15267 | 288 | 6107977604 | 0 | 1928538552 | 0.718367 | 0.970588 | 21.1593 | 0.732506 | 2.89125 | 124.6 |
| P5 | 8647 | 231 | 4179439052 | 0.315741 | 0 | 0.490476 | 0.957503 | 20.4268 | 0 | 2.87782 | 124.808 |
| P6_D0 | 10386 | 246 | 4823332921 | 0.210322 | 643893869 | 0.521088 | 0.963522 | 20.7094 | 0.282619 | 2.89912 | 124.766 |
| P6_D1 | 10911 | 223 | 4822278513 | 0.210495 | 642839461 | 0.536735 | 0.998734 | 20.9084 | 0.481578 | 3.07919 | 125.001 |
| P6_D2 | 10752 | 215 | 4827519926 | 0.209637 | 648080874 | 0.528571 | 0.997433 | 20.9114 | 0.484617 | 2.87512 | 124.651 |
| P6_D3 | 11102 | 211 | 4826470249 | 0.209809 | 647031197 | 0.534014 | 0.997459 | 21.0919 | 0.665106 | 2.9203 | 124.8 |

Observed tradeoff, generated directly from the comparison table:

```json
{
  "all_P6_full_query_recall_higher_than_P5": true,
  "all_P6_full_query_precision_higher_than_P5": true,
  "any_P6_full_incremental_joint_ATE_lower_than_P5": false,
  "all_P6_use_more_bytes_than_P5": true,
  "interpretation": "query-verification gains relative to P5 do not demonstrate better global map accuracy; individual regional/RPE/NN changes remain separate"
}
```

The query-level improvements relative to P5 must not be conflated with joint ATE improvement. Regional/RPE/NN diagnostics can move differently, and point-cloud overlap alone is not accuracy. D0's numerical agreement failure additionally limits interpretation of its incremental estimate. The unchanged primary noninferiority reference is P0, not a retrospectively substituted P5 target.

The original gate values are retained: attempt reduction>=25%, byte reduction>=20%, recall drop<=0.02 and precision drop<=0.01 in BOTH full and heldout scopes, full incremental joint ATE ratio<=1.05, no new catastrophic jump, proper factor convention, and original same-graph agreement. This is an empirical exposed-recording check, not a powered statistical noninferiority result.

| policy | attempt_reduction | application_byte_reduction | recall | precision | joint_ATE | no_new_catastrophic_jump | proper_factor_convention | incremental_batch_agreement | all_original_gates_pass | ATE_ratio_vs_P0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P6_D0 | True | True | False | True | True | True | True | False | False | 0.978738 |
| P6_D1 | True | True | False | True | True | True | True | True | False | 0.988141 |
| P6_D2 | True | True | False | True | True | True | True | True | False | 0.988285 |
| P6_D3 | True | True | False | True | True | True | True | True | False | 0.996815 |

The four Pareto plots show bytes/recall, bytes/ATE, receipt attempts/admitted factors, and bytes/map-NN. Readable `presentation_*.png` copies use separated callouts for nearly coincident points; the original evaluation plots and all numerical outputs remain unchanged. Pareto labels are descriptive only, not a selection/promotion rule. Global symmetric NN median/p95 includes non-overlap and cannot be read as literal absolute map alignment error. More factors or coverage bins alone do not demonstrate better maps. A lower ATE accompanied by higher cost than P5 is only a possible later MATCHED-BUDGET hypothesis; numerical and geometric gate failures must still be read. No such experiment starts here.

## Validation and limitations
Independent artifact audit status: PASS. It rebuilds final graphs without solving, checks chronological factor ledger equality, exact-once counts/connectivity, receipt/measurement convention, source timestamps and final pose schemas, recomputed objectives, same-graph pose statistics, all archive CRCs, representative real states and map artifact hashes. Original catastrophic rules are evaluated during EVERY backend event. Failures remain failures; there is no noise, solver, retrieval, GICP, threshold or GT-based retuning.

This is one previously exposed pair of separate recordings with deficient positive heldout dual-progress/direction coverage. Native sparse replay performance is not universal solver superiority or real-time SLAM. Offline metrics are diagnostic; graph structural validity and protocol savings do not establish accuracy, independent generalization, physical network efficiency or deployability. Stage23D and prior scientific artifacts remain frozen. No live/distributed demo or matched-budget study is started.

## Reproduction and exact locations

```bash
bash scripts/run_stage23e_p6_backend.sh
bash scripts/run_stage23e_evaluation.sh
```

The backend script resumes only verified completed per-policy results and creates the all-four pre-GT freeze. The evaluation script requires that freeze and preserves an existing completed evaluation. The isolated environment remains `/home/cas/.venvs/fyp_gtsam`; scientific helper files are imported, their entrypoints are NOT invoked against historical output paths.

Graph root: `/home/cas/fyp_place_recognition/outputs/cumulti_v1/23e_p6_graphs/`, children P6_D0/P6_D1/P6_D2/P6_D3. Evaluation root: `/home/cas/fyp_place_recognition/outputs/cumulti_v1/23e_p6_evaluation/`. Large full event ZIPs and PLY maps stay server-only with SHA256/size manifests; lightweight scientific CSVs, policy/freeze/validation reports and figures are versioned. This report is the new experiment-log entry; frozen history/context files are preserved.

STOP after Stage23E. Any next investigation requires separate authorization and must preserve the original gates. Independent recording eligibility remains NO_GO; independent validation is NOT_RUN.
