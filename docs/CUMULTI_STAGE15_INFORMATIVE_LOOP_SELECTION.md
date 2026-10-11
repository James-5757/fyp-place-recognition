# Stage15 — GT-free informative inter-robot loop selection

Accepted checkpoint `2c169e573401e65ea6bf9c91d7840075867e377a`. Final decision **NO_INFORMATIVE_SPARSE_POLICY**.

## Protocol and provenance

Historical120 -> stable endpoints R1KF>=150/R3KF>=234 -> exact frozen75; excluded45 never restored. All endpoint IDs, timestamps, transform measurements, GICP quality and SC metadata match Stage12D/14. Every subset includes frozenID45, R1#283↔R3#252. No GT column decoded, optional GT evaluation NOT_RUN. Input hashes and full historical before/after hashes accompany results. Existing user-dirty demo/10.3 files preserved.

## Four deterministic nested selectors

P0 exact arrival prefix; P1 decreasing frozen GICP quality; P2 maximin in normalized R1/R3 endpoint progress; P3 geometric mean of quality/novelty ranks recomputed among remaining candidates. Equal values use average ranks/n; no alpha/threshold sweep. Higher quality, earlier query timestamp, lower frozenID tie breaks. One ranking per selector, nested K1/2/5/10/20/40/75. All remaining-candidate scores are saved in selector_trace.csv; exact sets in selected_loops_by_budget.json. Endpoint progress is structural, not physical-map distance or observability.

## Fixed graph/solver/map

5796nodes,5794 frozen odometry,one R1KF150 prior,Kloops=5795+Kfactors. Identical first-bridge common x0, frozen5deg/1m noise,Huber1.345,URDF/gauge/measurement direction and12D default LM. No warm starts. Selected factors inserted in original arrival order; omitted factors never optimize or initialize. Selected,held-out andall75 diagnostics use physical E=Z^-1 Xq^-1 Xc translation/rotation, not tangent translation. Cross-K raw objectives are not quality scores.

Source-separated maps use retained-start frame stride10,point stride16,range1–60m,voxel0.5m; union of both directed3D NN samples. Cache holds only the sampled raw points and preserves exact historical operation order. All seven P0 outputs reproduce Stage13A maps,ROI,objective/poses within its frozen tolerances. Fixed20m ROI centers/radius unchanged, diagnostic only; all75 endpoint ROI is hindsight, not GT overlap. Map NN includes nonoverlap and is not global trajectory accuracy. Float64 source-map NPZ files are server-only with hashes/manifests.

## Frozen qualification and results

Smallest qualifying K<75 with normal finite/integrity return, global map median/p95<=1.10×full75, held-out translation/rotationp95<=1.50×full75all-loop. Worst normalized primary ratio, mean solve runtime,exact P3/P2/P1/P0 tie order follow. Full75 batch is a numerical/consistency reference, never truth. Pose agreement is reported but excluded from selection. Policy fsynced before any new solves/maps; decision fsynced after all GT-free metrics and before conditional incremental replay. No policy changes afterwards.

| selector | K | map_median_m | map_p95_m | heldout_translation_p95_m | heldout_rotation_p95_deg | pair_distance_p95 | quality_median | robot3_p95_nearest_endpoint_KF | worst_normalized_primary_metric | qualifies | batch_runtime_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ARRIVAL_PREFIX | 1 | 10.865166 | 144.832800 | 56.821569 | 68.773960 | 1.125503 | 0.749343 | 3521.400000 | 193.071508 | False | 85.584736 |
| ARRIVAL_PREFIX | 2 | 8.102668 | 129.738145 | 48.540015 | 90.578312 | 0.759213 | 0.711856 | 843.400000 | 164.931978 | False | 368.457419 |
| ARRIVAL_PREFIX | 5 | 4.288245 | 135.556565 | 28.986928 | 51.524026 | 0.682743 | 0.694030 | 701.400000 | 98.493405 | False | 376.450252 |
| ARRIVAL_PREFIX | 10 | 4.228913 | 115.804903 | 21.230601 | 77.661850 | 0.611960 | 0.687402 | 192.000000 | 72.138524 | False | 427.581107 |
| ARRIVAL_PREFIX | 20 | 4.099509 | 117.668069 | 19.072383 | 73.937763 | 0.597465 | 0.678106 | 192.000000 | 64.805211 | False | 379.002964 |
| ARRIVAL_PREFIX | 40 | 6.188868 | 115.251448 | 52.218379 | 66.340395 | 0.409362 | 0.659350 | 81.000000 | 177.430531 | False | 380.928774 |
| ARRIVAL_PREFIX | 75 | 3.030506 | 119.277542 | N/A | N/A | 0.000000 | 0.642445 | 0.000000 | N/A | False | 397.271253 |
| QUALITY_GREEDY | 1 | 10.865166 | 144.832800 | 56.821569 | 68.773960 | 1.125503 | 0.749343 | 3521.400000 | 193.071508 | False | 85.584736 |
| QUALITY_GREEDY | 2 | 4.669906 | 127.322104 | 25.521357 | 69.916166 | 0.676915 | 0.740729 | 1223.400000 | 86.717895 | False | 389.745978 |
| QUALITY_GREEDY | 5 | 4.292447 | 129.093196 | 36.427911 | 57.052371 | 0.479725 | 0.716523 | 378.700000 | 123.776794 | False | 389.256560 |
| QUALITY_GREEDY | 10 | 4.004545 | 127.855227 | 35.483988 | 60.570795 | 0.226183 | 0.711151 | 378.700000 | 120.569479 | False | 389.175414 |
| QUALITY_GREEDY | 20 | 4.177128 | 128.300861 | 32.932049 | 55.066079 | 0.136276 | 0.695344 | 179.900000 | 111.898358 | False | 390.194320 |
| QUALITY_GREEDY | 40 | 3.317613 | 122.088428 | 8.702660 | 33.547006 | 0.035750 | 0.671387 | 37.600000 | 29.570385 | False | 392.203171 |
| QUALITY_GREEDY | 75 | 3.030506 | 119.277542 | N/A | N/A | 0.000000 | 0.642445 | 0.000000 | N/A | False | 397.271253 |
| COVERAGE_GREEDY | 1 | 10.865166 | 144.832800 | 56.821569 | 68.773960 | 1.125503 | 0.749343 | 3521.400000 | 193.071508 | False | 85.584736 |
| COVERAGE_GREEDY | 2 | 5.600993 | 143.441988 | 32.356750 | 54.378487 | 0.734214 | 0.679228 | 1580.500000 | 109.943575 | False | 407.237617 |
| COVERAGE_GREEDY | 5 | 4.732567 | 126.261195 | 47.070281 | 85.570094 | 0.239286 | 0.703581 | 338.500000 | 159.938035 | False | 394.105656 |
| COVERAGE_GREEDY | 10 | 3.535191 | 119.822356 | 17.572537 | 48.867032 | 0.135293 | 0.647034 | 191.900000 | 59.708949 | False | 448.091333 |
| COVERAGE_GREEDY | 20 | 3.266874 | 116.969383 | 14.193187 | 50.230805 | 0.037226 | 0.653631 | 55.100000 | 48.226404 | False | 445.805567 |
| COVERAGE_GREEDY | 40 | 3.098681 | 119.062359 | 3.956524 | 14.071211 | 0.013981 | 0.649574 | 22.600000 | 13.443700 | False | 399.199894 |
| COVERAGE_GREEDY | 75 | 3.030506 | 119.277542 | N/A | N/A | 0.000000 | 0.642445 | 0.000000 | N/A | False | 397.271253 |
| QUALITY_COVERAGE_GREEDY | 1 | 10.865166 | 144.832800 | 56.821569 | 68.773960 | 1.125503 | 0.749343 | 3521.400000 | 193.071508 | False | 85.584736 |
| QUALITY_COVERAGE_GREEDY | 2 | 3.910955 | 141.507625 | 14.005087 | 56.009392 | 0.681698 | 0.722046 | 1267.500000 | 47.587268 | False | 405.864195 |
| QUALITY_COVERAGE_GREEDY | 5 | 4.433801 | 140.373364 | 32.016430 | 59.936101 | 0.161029 | 0.711625 | 416.500000 | 108.787216 | False | 395.342973 |
| QUALITY_COVERAGE_GREEDY | 10 | 3.864392 | 139.313342 | 32.626762 | 48.891144 | 0.109661 | 0.699780 | 120.200000 | 110.861038 | False | 393.390397 |
| QUALITY_COVERAGE_GREEDY | 20 | 3.489841 | 123.091997 | 20.410010 | 45.035544 | 0.046454 | 0.690213 | 54.800000 | 69.350273 | False | 390.777364 |
| QUALITY_COVERAGE_GREEDY | 40 | 3.279994 | 121.305450 | 4.026596 | 17.795745 | 0.020589 | 0.659755 | 22.600000 | 13.681795 | False | 392.931700 |
| QUALITY_COVERAGE_GREEDY | 75 | 3.030506 | 119.277542 | N/A | N/A | 0.000000 | 0.642445 | 0.000000 | N/A | False | 397.271253 |

Selected policy: `null`. Exact selected IDs: `[]`. Selection thresholds: `{"map_median_m": 3.3335569069226363, "map_p95_m": 131.20529673911, "heldout_translation_p95_m": 0.2943032355810134, "heldout_rotation_p95_deg": 2.0759022608008286}`.

## Runtime and reuse

One warmup,three measured LM solves per unique subset; primary coordinates from first measured solve, not fastest. 22 unique subsets cover28conditions; identicalK1/K75 sets and any other exact duplicates explicitly reuse their source_condition, not pretend independent repeats. Solve timing excludes factor construction,map transform/voxel,NN/ROI and diagnostics, recorded separately. Three samples are insufficient for reliable timing tails. No bandwidth measurement; 1-K/75 is only loop-factor/communication-event reduction proxy.

## Scientific questions A–J: same-K evidence

The following table lists strictly better budgets, excluding identicalK1/K75 controls. A lower map NN/residual/pair-coverage value means improvement in that diagnostic only, not higher global physical accuracy.

| selector | budgets_tested | map_median_better_K | map_p95_better_K | heldout_translation_better_K | heldout_rotation_better_K | pair_coverage_better_K |
| --- | --- | --- | --- | --- | --- | --- |
| QUALITY_GREEDY | 5 | 2,10,40 | 2,5 | 2,40 | 2,10,20,40 | 2,5,10,20,40 |
| COVERAGE_GREEDY | 5 | 2,10,20,40 | 5,20 | 2,10,20,40 | 2,10,20,40 | 2,5,10,20,40 |
| QUALITY_COVERAGE_GREEDY | 5 | 2,10,20,40 |  | 2,40 | 2,10,20,40 | 2,5,10,20,40 |

A/B: quality-only and coverage-only improve some metrics/budgets, not all. C: the combined selector does not consistently dominate either; compare the measured same-K curves rather than assume its geometric mean is optimal. D: candidates meeting BOTH original10% map inequalities (without ignoring or replacing the other criteria) are:

| selector | K |
| --- | --- |
| QUALITY_GREEDY | 40 |
| COVERAGE_GREEDY | 20 |
| COVERAGE_GREEDY | 40 |
| QUALITY_COVERAGE_GREEDY | 40 |

E: every K<75 violates the held-out translationp95 envelope; even the smallest observed sparse value 3.956524m exceeds 0.294303m. Held-out loops never optimized. Thus no subset qualifies even if the supplementary rotation screen is set aside **for explanation only**, not by altering the frozen decision.

F: endpoint coverage loss is quantified in structural_coverage.csv; maximin greatly lowers pair-distancep95 but does not prove physical observability. G: improvements are nonmonotonic in several curves, no universal diminishing-return cutoff declared. H: quality-only retains high median registration quality; combined selection trades this against novelty; coverage selection can retain low-quality distant endpoints. Full distributions atK10/K40 are plotted. I: no selected policy means no validated removed messages/event workload. K40's hypothetical35/75=46.67% factor/event reduction is NOT an accepted saving or measured bandwidth reduction. J: conditional standard iSAM2 replay NOT_APPLICABLE, no optimizer parameter sweep or post-result promotion.

## Integrity screening: retained negative findings

All28conditions return normally with finite valid poses, original IDs/timestamps and fixed anchor. There are zero new frozen12D translation-catastrophe flags. However, ten informative-selector conditions trigger one newly crossed frozen12C rotation-step threshold each. The stricter supplementary lineage screen was included in the policy before any solve and remains FAIL; no threshold is relaxed, and these candidates are ineligible. A robust rotation threshold crossing is not by itself proof of a physical catastrophic jump. The independent audit verifies these failure flags rather than pretending all trajectories pass. Detailed values are in rotation_threshold_crossing_details.csv when the independent auditor is run.

| selector | K | robot_id | new_rotation_threshold_crossings | new_catastrophic_steps |
| --- | --- | --- | --- | --- |
| QUALITY_GREEDY | 5 | robot3 | 1 | 0 |
| QUALITY_GREEDY | 10 | robot3 | 1 | 0 |
| QUALITY_GREEDY | 20 | robot3 | 1 | 0 |
| QUALITY_GREEDY | 40 | robot1 | 1 | 0 |
| COVERAGE_GREEDY | 20 | robot3 | 1 | 0 |
| COVERAGE_GREEDY | 40 | robot1 | 1 | 0 |
| QUALITY_COVERAGE_GREEDY | 5 | robot3 | 1 | 0 |
| QUALITY_COVERAGE_GREEDY | 10 | robot3 | 1 | 0 |
| QUALITY_COVERAGE_GREEDY | 20 | robot3 | 1 | 0 |
| QUALITY_COVERAGE_GREEDY | 40 | robot1 | 1 | 0 |

## Exact nested selected loop IDs (all budgets)

These are candidate subsets, not an accepted sparse winner. Factor insertion within each set follows original arrival order, while the list preserves selector ranking.

| selector | K | frozen_loop_ids |
| --- | --- | --- |
| ARRIVAL_PREFIX | 1 | 45 |
| ARRIVAL_PREFIX | 2 | 45,46 |
| ARRIVAL_PREFIX | 5 | 45,46,47,48,49 |
| ARRIVAL_PREFIX | 10 | 45,46,47,48,49,50,51,52,53,54 |
| ARRIVAL_PREFIX | 20 | 45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64 |
| ARRIVAL_PREFIX | 40 | 45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84 |
| ARRIVAL_PREFIX | 75 | 45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119 |
| QUALITY_GREEDY | 1 | 45 |
| QUALITY_GREEDY | 2 | 45,51 |
| QUALITY_GREEDY | 5 | 45,51,74,53,47 |
| QUALITY_GREEDY | 10 | 45,51,74,53,47,55,59,103,101,99 |
| QUALITY_GREEDY | 20 | 45,51,74,53,47,55,59,103,101,99,118,49,57,58,98,54,100,60,46,62 |
| QUALITY_GREEDY | 40 | 45,51,74,53,47,55,59,103,101,99,118,49,57,58,98,54,100,60,46,62,56,76,52,88,70,106,77,73,111,64,78,75,82,68,81,112,91,90,87,102 |
| QUALITY_GREEDY | 75 | 45,51,74,53,47,55,59,103,101,99,118,49,57,58,98,54,100,60,46,62,56,76,52,88,70,106,77,73,111,64,78,75,82,68,81,112,91,90,87,102,65,83,66,96,61,107,67,80,79,63,85,117,95,92,115,50,71,97,105,110,108,84,109,113,104,116,94,72,114,48,86,93,89,69,119 |
| COVERAGE_GREEDY | 1 | 45 |
| COVERAGE_GREEDY | 2 | 45,119 |
| COVERAGE_GREEDY | 5 | 45,119,50,101,51 |
| COVERAGE_GREEDY | 10 | 45,119,50,101,51,88,113,89,69,46 |
| COVERAGE_GREEDY | 20 | 45,119,50,101,51,88,113,89,69,46,95,106,118,81,74,49,107,64,104,63 |
| COVERAGE_GREEDY | 40 | 45,119,50,101,51,88,113,89,69,46,95,106,118,81,74,49,107,64,104,63,87,66,99,82,47,98,73,94,102,111,93,117,75,78,65,109,92,52,68,83 |
| COVERAGE_GREEDY | 75 | 45,119,50,101,51,88,113,89,69,46,95,106,118,81,74,49,107,64,104,63,87,66,99,82,47,98,73,94,102,111,93,117,75,78,65,109,92,52,68,83,48,53,105,71,97,112,100,103,115,79,67,86,90,72,84,58,80,116,59,76,70,96,60,91,108,54,85,77,110,55,62,114,56,57,61 |
| QUALITY_COVERAGE_GREEDY | 1 | 45 |
| QUALITY_COVERAGE_GREEDY | 2 | 45,118 |
| QUALITY_COVERAGE_GREEDY | 5 | 45,118,47,98,74 |
| QUALITY_COVERAGE_GREEDY | 10 | 45,118,47,98,74,54,103,59,77,111 |
| QUALITY_COVERAGE_GREEDY | 20 | 45,118,47,98,74,54,103,59,77,111,99,88,106,70,51,101,46,49,90,91 |
| QUALITY_COVERAGE_GREEDY | 40 | 45,118,47,98,74,54,103,59,77,111,99,88,106,70,51,101,46,49,90,91,82,66,73,83,78,107,96,87,65,68,62,100,81,117,112,102,50,92,79,67 |
| QUALITY_COVERAGE_GREEDY | 75 | 45,118,47,98,74,54,103,59,77,111,99,88,106,70,51,101,46,49,90,91,82,66,73,83,78,107,96,87,65,68,62,100,81,117,112,102,50,92,79,67,63,53,110,115,97,105,76,71,80,60,104,58,113,84,52,94,108,95,116,55,72,48,75,85,86,64,109,56,89,57,93,114,69,61,119 |

## Incremental validation

`{"status": "NOT_APPLICABLE", "reason": "No sparse candidate met frozen GT-free criteria"}`. Only if a sparse policy exists: one standard persistent instance,skip10/threshold0.1/wildfire0.001,no empty calls. PrebuiltR3,streamingR1 at original query arrivals,precomputed selected loops,not simultaneous/causal online discovery. First bridge,each selected event,FINAL compared against same arrived sparse factor graph LM from common x0. No extra alignment; historical pose gates are diagnostics,never alter selected policy. Final map publication is in-memory transform/voxel only,separately timed. Backend deadlines do not establish end-to-end realtime.

## Interpretation and limitations

Same-K metric ratios/quality/coverage/residual curves support comparisons, not universal selector superiority. No monotonicity or optimal-set claim assumed; omitted75-K loops are held-out frozen measurements with hindsight availability. The full75 reference retains Stage14 numerical path-dependence limitations. One robot pair,one fixed seed,several prescribed budgets,possibly weak/nonuniform geometry,globally unmasked NN influenced by nonoverlap. Historical objectives across different factor counts are not equivalent scores. No GT accuracy inference,distributed SLAM or demo change. See independent auditor for actual reproducibility/integrity status. Stop after Stage15.

Next-stage recommendation: separately predeclare a GT-free support/weak-direction or measurement-information study if justified; neither rank quality nor timeline maximin alone establishes sparse sufficiency. Preserve the75-loop full-clean reference, original residual envelope, Stage14 caveats and excluded45. Do not automatically expand budgets/selectors, tune thresholds, change noise, load GT or start deployment/demo.

## Reproduction

From repository root: `bash scripts/run_stage15_informative_loop_selection.sh` (empty new Stage15 directory required; refuses overwrites). `--report-only` regenerates saved-data report/plots, no solver/selection/GT. `python src/cumulti/audit_stage15_informative_loop_selection.py` independently checks rankings,sets,graphs,residuals,qualification,history,chronology and schemas without optimization.
