# Stage14 — final-state consistency and incremental path-dependence audit

Accepted `292f8c22ef62b51dace4ac17b0df633170322fbb`. Decision **RESIDUAL_NUMERICAL_DISCREPANCY_UNRESOLVED**. Independent numerical diagnostic, not a deployable policy; batch LM is not ground truth.

## Frozen inputs / loop provenance

Historical Rank1 sanitized120 -> frozen12C endpoints R1>=150/R3>=234 -> retained75, unchanged measurements and query order. No excluded45 restored. Same frozen odometry, prior atR1KF150, sigma/Huber/extrinsic. FINAL5796nodes/5794odom/75loops/1prior=5870factors. K2/K75 only arrived R1 nodes; stored Robot3 complete. Accepted blobs and actualfactor.equals verified. All CSVs use explicit non-GT column allowlists; raw datasets/GT/maps/frontend/demo untouched.

## Design and API

A0: read-only13D D0normal/D2extra5. A1 freshD2 canonical common x0; A2 freshD2 D0coordinates; A3 freshD2 D2coordinates. Complete graph inserted into nine NEW ISAM2 instances, exactly5 empty calls each. skip1/threshold0.01/wildfire0.001. No force API assumed/used, no batch coordinates initialize ISAM2. Invalid counters retain raw strings, null/INVALID_COUNTER; in-range is not independent proof of semantics. Six LM probes B0canonical/B1D0/B2D2 atK75/FINAL use frozen12D defaults. Same gauge, NO alignment. Gate0.1/0.5m and0.5/2deg unchanged; no-change<1e-7m/<1e-7deg/<1e-9objective twice, not proof of mathematical convergence. Policy fsynced before runs.

## Historical persistent and fresh final states

| checkpoint | solver_state | translation_median_m | translation_p95_m | rotation_median_deg | rotation_p95_deg | nonlinear_error | objective_gap | extra_updates | PASS |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| K2 | D0_PERSISTENT | 3.382671272149987 | 12.286617828357349 | 1.3420693539494963 | 2.216088598381601 | 0.7683611578192504 | 0.6899491274819463 | historical0 | False |
| K2 | D2_PERSISTENT | 0.0090606945068178 | 0.0551091750914391 | 0.0065322566086014 | 0.0209447931430039 | 0.07841228299261 | 2.526553058712633e-07 | historical5 | True |
| K2 | A1_FRESH_CANONICAL | 0.0090606943929872 | 0.0551091750215972 | 0.0065322566042772 | 0.0209447931247639 | 0.0784122829926116 | 2.526553074255755e-07 | 5 | True |
| K2 | A2_FRESH_D0 | 0.0090782111973817 | 0.0550593637294727 | 0.0065290155822224 | 0.0209340860831184 | 0.078412282524985 | 2.5218768083534737e-07 | 5 | True |
| K2 | A3_FRESH_D2 | 0.0099640493566032 | 0.0541878586121197 | 0.0059738038829576 | 0.0199595509485906 | 0.0784119962383658 | 3.4098938381244004e-08 | 5 | True |
| K75 | D0_PERSISTENT | 0.1231858831222212 | 0.7445213459990673 | 0.054755177707472 | 0.7459608778990042 | 25.060652355652454 | 1.220268700152836 | historical0 | False |
| K75 | D2_PERSISTENT | 0.1066188502568129 | 0.5723921583406475 | 0.0365465963786922 | 0.3103017837844535 | 23.84038893787938 | 5.282379756721412e-06 | historical5 | False |
| K75 | A1_FRESH_CANONICAL | 0.129682754558028 | 0.5605187625402571 | 0.0453487704273695 | 0.3017619163650349 | 23.84033237309773 | 5.128240188767563e-05 | 5 | False |
| K75 | A2_FRESH_D0 | 0.1124889110556834 | 0.5684375887779403 | 0.0385996879906058 | 0.3083008118441689 | 23.84037347868634 | 1.0176813280082795e-05 | 5 | False |
| K75 | A3_FRESH_D2 | 0.1107844044932024 | 0.5686403879748465 | 0.0374116811988892 | 0.3083052029110385 | 23.840395065619287 | 1.1410119665100638e-05 | 5 | False |
| FINAL_STREAM_END | D0_PERSISTENT | 0.1244306586651536 | 0.5639854280275804 | 0.0553191878789764 | 0.3015371230925491 | 23.840417589290723 | 3.394381223031928e-05 | historical0 | False |
| FINAL_STREAM_END | D2_PERSISTENT | 0.1081791554434834 | 0.5747419196126624 | 0.036756271876577 | 0.3117222336594787 | 23.840388868345496 | 5.222867002885323e-06 | historical5 | False |
| FINAL_STREAM_END | A1_FRESH_CANONICAL | 0.1305053760927055 | 0.5636221503836432 | 0.0455649176176768 | 0.3031870627582599 | 23.840332373097677 | 5.1272380819256114e-05 | 5 | False |
| FINAL_STREAM_END | A2_FRESH_D0 | 0.1139900795990718 | 0.5707845035632929 | 0.038881867343243 | 0.3097522194793471 | 23.84037347972239 | 1.0165756101798706e-05 | 5 | False |
| FINAL_STREAM_END | A3_FRESH_D2 | 0.112041507538517 | 0.5709946548560623 | 0.037598428279768 | 0.3097569228365038 | 23.84039506564653 | 1.1420168036124778e-05 | 5 | False |

## LM initialization probes

| checkpoint | solver_state | initial_error | final_error | iterations | runtime_ms | translation_p95_m | from_init_translation_p95_m | PASS | iteration_limit_reached |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| K75 | B0_LM_CANONICAL | 3843.986005704936 | 23.840383655499632 | 7 | 399.473330937326 | 2.009718347115232e-14 | 110.89903025789486 | True | False |
| K75 | B1_LM_D0 | 25.060652355652454 | 23.84038044294856 | 3 | 179.56734634935856 | 0.5702328270170784 | 0.1936897726463054 | False | False |
| K75 | B2_LM_D2 | 23.84038893787938 | 23.84037743332096 | 1 | 62.58265720680356 | 0.572389596771516 | 0.0012135516636373 | False | False |
| FINAL_STREAM_END | B0_LM_CANONICAL | 3843.986005704936 | 23.840383645478425 | 7 | 418.73261006549 | 2.009718347115232e-14 | 110.848432089282 | True | False |
| FINAL_STREAM_END | B1_LM_D0 | 23.840417589290723 | 23.84036735794681 | 1 | 64.49490785598755 | 0.5640320986726389 | 0.0059174001329183 | False | False |
| FINAL_STREAM_END | B2_LM_D2 | 23.840388868345496 | 23.840377433087973 | 1 | 65.01536443829536 | 0.5747391179957249 | 0.0011620954031657 | False | False |

Pairwise same-gauge comparisons are in pairwise_solver_agreement.csv: original-gate consistency and stricter descriptive essentially-same0.001m/0.001deg reported separately. Fixed LM stopping can leave nearby coordinates, not proof of distinct global minima or a uniquely converged optimum. A2 changes historicalD0 settings toD2, so it confounds settings with rebuild; A3 preserves historicalD2 settings but still changes full insertion/linearization path, not only the Bayes tree in isolation.

| checkpoint | left | right | translation_median_m | translation_p95_m | near_objective | original_gate_consistent | essentially_same_coordinates |
| --- | --- | --- | --- | --- | --- | --- | --- |
| K75 | B0_LM_CANONICAL | B1_LM_D0 | 0.1342948756575216 | 0.5702328270170784 | True | False | False |
| K75 | B0_LM_CANONICAL | B2_LM_D2 | 0.1062213154865359 | 0.572389596771516 | True | False | False |
| K75 | B1_LM_D0 | B2_LM_D2 | 0.0218960019449587 | 0.0722984407938426 | True | True | False |
| K75 | A1_FRESH_CANONICAL | A2_FRESH_D0 | 0.0205299895734666 | 0.051960496089528 | True | True | False |
| K75 | A1_FRESH_CANONICAL | A3_FRESH_D2 | 0.0227609298363825 | 0.0579715010381348 | True | True | False |
| K75 | A2_FRESH_D0 | A3_FRESH_D2 | 0.0023876537528712 | 0.0065350368815979 | True | True | False |
| FINAL_STREAM_END | B0_LM_CANONICAL | B1_LM_D0 | 0.1240882283853884 | 0.5640320986726389 | True | False | False |
| FINAL_STREAM_END | B0_LM_CANONICAL | B2_LM_D2 | 0.107943530684959 | 0.5747391179957249 | True | False | False |
| FINAL_STREAM_END | B1_LM_D0 | B2_LM_D2 | 0.0198870245009012 | 0.0503772247138358 | True | True | False |
| FINAL_STREAM_END | A1_FRESH_CANONICAL | A2_FRESH_D0 | 0.0207689011277019 | 0.0538233626784534 | True | True | False |
| FINAL_STREAM_END | A1_FRESH_CANONICAL | A3_FRESH_D2 | 0.0230012265229481 | 0.0596537707491565 | True | True | False |
| FINAL_STREAM_END | A2_FRESH_D0 | A3_FRESH_D2 | 0.002399803719935 | 0.0062395356338468 | True | True | False |

## Scientific interpretation of measured contrasts

Fresh reconstruction from identical persistent coordinates DOES change poses measurably: FINAL A2/D0 p95 movement is approximately0.0304m; A3/D2 approximately0.0209m. However, FINAL disagreement to frozenbatch remains approximately0.57m and every fresh final condition fails. K75 D0 improves but K75D2/FINAL do not satisfy the predeclared clear-toward-reference rule. Rebuilding alone is not sufficient to explain/resolve the residual under this protocol.

Fresh canonical/D0/D2 initializations produce centimeter-scale different coordinates, but their pairwise differences stay within original gates. LM canonical probes reproduce the frozenreference, while LM from persistent states stop at measurably different coordinates despite near-equal objectives; these do NOT constitute one essentially identical LM pose solution. This supports initialization/termination sensitivity as an observation, not proof of multiple global minima. The conservative frozen overall decision requires additional contrasts, and remains unresolved rather than being relaxed after results.

FINAL discrepancy principally affects Robot3 (persistent p95 approximately0.65–0.67m versus Robot1 approximately0.16–0.19m). Both persistent profiles classify LONG_CHAIN_SMOOTH_DEFORMATION, with adjacent translation-delta p95 approximately0.0037–0.0038m. PCA is descriptive, not an information-nullspace result. Nearest-loop-index distance has positive Pearson/Spearman association with error; local endpoint density is negatively associated. Temporal autocorrelation and shared trajectory structure preclude causal inference from these correlations.

## Localization, deformation and structural support

Perrobot physical translation/rotation, worst20, contiguous p95 regions and nodewise Logmap are saved. Physical world xyz deltas differ from body-frame Logmap translation tangent coordinates; Logmap ordering rot then trans, finite flag stored. Rotation vector profiles are world-frame relative rotations. Descriptive classifications/PCA and structural correlations:

```json
{
  "D0_PERSISTENT": {
    "classification": "LONG_CHAIN_SMOOTH_DEFORMATION",
    "high_error_fraction": 0.10973137354282818,
    "longest_gt_free_high_error_run_kf": 312,
    "adjacent_delta_median_m": 0.0007092536646800867,
    "adjacent_delta_p95_m": 0.003739558662157912,
    "adjacent_delta_max_m": 0.005569963914709782,
    "adjacent_rotation_vector_p95_rad": 1.9482707671206876e-05,
    "PCA_variance_ratios": [
      0.8351562999546448,
      0.15790507738396262,
      0.00693862266139251
    ],
    "PCA_axes": [
      [
        0.6616824554152999,
        -0.6686691374238635,
        -0.3392018762513056
      ],
      [
        0.14987800370469492,
        0.5612264156612994,
        -0.8139788046193003
      ],
      [
        0.7346515583602031,
        0.4877565940310773,
        0.4715724682147254
      ]
    ],
    "mean_delta_world_m": [
      0.009890350655193177,
      -0.019171097660635506,
      0.07941528990928509
    ],
    "left_transform_translation_variation_p95_m": 0.4654320844747768,
    "left_transform_rotation_variation_p95_deg": 0.3223237255646155,
    "not_Hessian_nullspace_evidence": true,
    "support_correlations": [
      {
        "proxy": "nearest_loop_endpoint_kf_distance",
        "Pearson_r": 0.887963678744964,
        "Spearman_rho": 0.7291059441423249,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_count_w50",
        "Pearson_r": -0.3656172589055731,
        "Spearman_rho": -0.5089332895656278,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_density_w50",
        "Pearson_r": -0.3680143727752093,
        "Spearman_rho": -0.5093457151667604,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_count_w100",
        "Pearson_r": -0.3969529540918112,
        "Spearman_rho": -0.537419686051347,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_density_w100",
        "Pearson_r": -0.3998790726313024,
        "Spearman_rho": -0.5389327945062735,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_count_w250",
        "Pearson_r": -0.45091396271655204,
        "Spearman_rho": -0.6082425790343501,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_density_w250",
        "Pearson_r": -0.4547343740873869,
        "Spearman_rho": -0.6058432606762655,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      }
    ]
  },
  "D2_PERSISTENT": {
    "classification": "LONG_CHAIN_SMOOTH_DEFORMATION",
    "high_error_fraction": 0.11505321844906234,
    "longest_gt_free_high_error_run_kf": 324,
    "adjacent_delta_median_m": 0.0005139382919279671,
    "adjacent_delta_p95_m": 0.003814463606783285,
    "adjacent_delta_max_m": 0.005626810746184283,
    "adjacent_rotation_vector_p95_rad": 1.1853220215848413e-05,
    "PCA_variance_ratios": [
      0.8107248893859813,
      0.1807762777552586,
      0.008498832858760172
    ],
    "PCA_axes": [
      [
        0.6637230286141008,
        -0.664069415891711,
        -0.3442143985433601
      ],
      [
        0.21073538502450298,
        0.6075706202769365,
        -0.765799281061867
      ],
      [
        0.7176784368962883,
        0.4357404643288706,
        0.5432015362280928
      ]
    ],
    "mean_delta_world_m": [
      0.002550347753626991,
      -0.006284469164723557,
      0.08140666820722967
    ],
    "left_transform_translation_variation_p95_m": 0.46635190755784267,
    "left_transform_rotation_variation_p95_deg": 0.33159769079907453,
    "not_Hessian_nullspace_evidence": true,
    "support_correlations": [
      {
        "proxy": "nearest_loop_endpoint_kf_distance",
        "Pearson_r": 0.8996212399266601,
        "Spearman_rho": 0.7226680075781102,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_count_w50",
        "Pearson_r": -0.3690639124575775,
        "Spearman_rho": -0.4930735029110125,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_density_w50",
        "Pearson_r": -0.3714265185502015,
        "Spearman_rho": -0.49346674423406667,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_count_w100",
        "Pearson_r": -0.4005728113417446,
        "Spearman_rho": -0.5253331982012114,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_density_w100",
        "Pearson_r": -0.4034585836061448,
        "Spearman_rho": -0.526710800345316,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_count_w250",
        "Pearson_r": -0.45743425789330394,
        "Spearman_rho": -0.6083630411101081,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      },
      {
        "proxy": "endpoint_density_w250",
        "Pearson_r": -0.4615503822890451,
        "Spearman_rho": -0.6059790050474922,
        "p_values_not_interpreted_due_to_chain_autocorrelation": true
      }
    ]
  }
}
```

Windows±50/100/250KF frozen before results; loop-factor endpoint counts (duplicates count factors), boundary-adjusted density, nearest path-order distance. Correlations are serially dependent descriptive associations, not causal proof or unobservability. Common-transform test compares per-pose left transforms, never aligns trajectories or modifies gate coordinates. Smooth disagreement/PCA does not prove a Hessian nullspace.

## Bounded conditioning / weak directions

Status **PASS**. Predeclared9pose set: three historicalD0 worst R3 poses separated>=200KF, three nearest-index lower-quartile R3controls (matching gaps recorded), R1controls150/1075/1999. Two FINAL linearizations: D0 and frozenbatch. Sparse GTSAM Marginals in isolated worker,1GiB extraaddress-space/30CPU/45wall budget; only6x6 matrices, no global denseHessian. Covariance is local robust-factor linearization/IRLS information, not calibrated physical accuracy; condition proxy mixes rad/m tangent units, not global Hessiancondition. PSD/symmetry checks and exact pose selection/runtimes are in conditioning_method.json and weak_direction_diagnostic.csv. No nullspace claim.

Group median local diagnostics (NOT actual trajectory accuracy):

```csv
linearization,group,translation_covariance_trace_m2,rotation_covariance_trace_rad2,condition_proxy
D0_PERSISTENT,LOW_ERROR_CONTROL,26048.120442312684,7.991729821279897,17053.415328555035
D0_PERSISTENT,ROBOT1_CONTROL,66869.70582909805,5.731257020347619,215200.68102524005
D0_PERSISTENT,WORST,209209.85375205337,14.01033843669418,155104.5143479655
FROZEN_BATCH,LOW_ERROR_CONTROL,26074.27617628818,7.993462956591149,17080.560919996165
FROZEN_BATCH,ROBOT1_CONTROL,66892.40773122528,5.731800809554192,215235.4062568032
FROZEN_BATCH,WORST,208426.9994229062,14.012877654740269,154682.2584187287

```

All18 blocks are valid in this run. Large translation covariance traces occur at both Robot3 worst regions and the late Robot1 control, indicating nonuniform/weak local information under these fixed factor weights, not an exclusively Robot3 information issue. The matched low-error R3 controls are557–868KF away from their worst partners: imperfect matching limits causal comparisons. Selected marginal observations do not prove a global Hessian nullspace or isolate one causal explanation.

## Runtime categories

| checkpoint | operation | construction_ms | full_insertion_ms | empty_update_total_ms | extraction_ms | objective_ms | file_localization_ms |
| --- | --- | --- | --- | --- | --- | --- | --- |
| FINAL_STREAM_END | A1_FRESH_CANONICAL | 0.0737342052161693 | 51.54887400567532 | 241.9954976066947 | 108.22071926668286 | 337.85125566646457 | 631.6191656515002 |
| FINAL_STREAM_END | A2_FRESH_D0 | 0.1472779549658298 | 45.69573700428009 | 72.43734272196889 | 90.4573849402368 | 337.9363985732198 | 629.0685320273042 |
| FINAL_STREAM_END | A3_FRESH_D2 | 0.1478507183492183 | 46.854383777827024 | 12.753319460898638 | 79.48117563501 | 330.8316650800407 | 621.0690611042082 |
| K2 | A1_FRESH_CANONICAL | 0.0782008282840251 | 30.29606211930513 | 143.54222500696778 | 69.91423014551401 | 233.84337360039353 | 457.45732681825757 |
| K2 | A2_FRESH_D0 | 0.1094252802431583 | 28.398679103702307 | 116.14367365837097 | 67.66103580594063 | 233.1028850749135 | 452.3788574151695 |
| K2 | A3_FRESH_D2 | 0.1167221926152706 | 29.49794987216592 | 28.436266817152504 | 56.24826205894351 | 232.925727032125 | 454.14426596835256 |
| K75 | A1_FRESH_CANONICAL | 0.0632312148809433 | 43.22865698486567 | 229.20937556773424 | 104.83874240890145 | 330.42497048154473 | 622.4390254355967 |
| K75 | A2_FRESH_D0 | 0.1054871827363967 | 43.350504245609045 | 67.82084656879306 | 88.11766421422362 | 331.98335906490684 | 620.1082658953965 |
| K75 | A3_FRESH_D2 | 0.1118090003728866 | 45.13654997572303 | 10.379609186202288 | 76.05242822319269 | 324.0356151945889 | 616.8473712168634 |

Historical incremental per-frame timing is a separate read-only reference in historical_incremental_runtime.csv; never equate it to full fresh insertion. LM runtimes above and marginal construction/query/wall cost separate. File/localization overhead separate; single trial percondition. No end-to-end real-time claim.

## Decision and unresolved questions

```json
{
  "UTC": "2026-10-10T11:06:50.988768+00:00",
  "decision": "RESIDUAL_NUMERICAL_DISCREPANCY_UNRESOLVED",
  "rebuild_toward_reference_evidence": [
    {
      "checkpoint": "K75",
      "starting_state": "D0_PERSISTENT",
      "fresh_state": "A2_FRESH_D0",
      "reduction_m": 0.176083757221127,
      "D0_parameter_change_confound": true
    }
  ],
  "LM_near_objective_gate_inconsistent_checkpoints": [
    "FINAL_STREAM_END",
    "K75"
  ],
  "fresh_initialization_gate_inconsistent_checkpoints": [],
  "GT_used": false,
  "production_ready": false,
  "not_multiple_global_minima_proof": true,
  "limitations": "A2 changes defaultD0 parameters toD2; A3 keepsD2 parameters but freshfull insertion/relinearization path changes. Fixed LM termination is not proof of global convergence. Conditioning/association alone not cause."
}
```

Objective/pose table and figure show near-equal objective versus original gates; objective agreement never substitutes for pose agreement. No retuning, no GT or newloops. Rebuild/init sensitivity, structural associations and covariance observations must not be collapsed into one unproven causal mechanism.

## Reproduction and next step

Run `bash scripts/run_stage14_final_consistency.sh` from root with a new empty Stage14 directory. Refuses overwrite. `--report-only` reads saved data and never optimizes or rewrites policy/decision; independent auditor checks artifacts. Stop after14. Any solver-method or informative-loop-selection follow-up needs a separate protocol; no automatic newloops/demo/live stage. Historical13B/C/D status stays unchanged.

Implementation recovery is explicit: an initial prefix-schema preflight failed before new solves; the first full experimental attempt later reached all solver states but a Robot3-only reporting helper failed before some timers/iterations were saved. Both attempts were archived server-side; one necessary exact-policy recovery supplied mandatory runtime records. recovery_reproduction_audit.csv compares all66 estimate files and confirms identical numerical coordinates; archived-file hashes are in failed_attempt_artifact_manifest.json. Only the successful recovery timing is primary, never the better of two timings. Policy bytes are identical before/after recovery. No optimization follows the final decision; report-only finalization records source/policy/decision hashes.
