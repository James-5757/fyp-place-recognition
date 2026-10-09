# Stage13C — ISAM2 numerical convergence and relinearization

## 1. Motivation

Independent GT-free numerical convergence study, not trajectory-accuracy or map-consistency evaluation.

## 2. Stage13B numerical disagreement

Frozen default result remains FAIL_NUMERICAL_AGREEMENT / INCREMENTAL_BACKEND_NOT_READY. Final objectives nearly agree without passing pose gates.

## 3. Fixed graph and event schedule

Prebuilt Robot3 KF234–4179 plus streamed Robot1 KF150–1999. First graph4080 nodes at KF283; archived loops, not causal online SC/GICP. Same actual Stage12D factors/anchor/noise/frames, causal previous-estimate initial guesses.

## 4. Reproducibility and environment

Accepted `b20f214e3eac8436f32bb3f9f6250c75b30c04bb`; isolated GTSAM4.2/Python3.10. Recorded API/configuration/input SHA. Run `bash scripts/run_stage13c_isam2_convergence.sh` with empty output directory; never invokes historical mains, offline GT, map builders or LM optimizer.

## 5. Frozen ISAM2 parameter variants

| name | skip | threshold | extra |
| --- | --- | --- | --- |
| V0_BASELINE_REPLICATION | 10 | 0.100000 | 0 |
| V1_EVERY_EVENT_RELINEARIZATION_CHECK | 1 | 0.100000 | 0 |
| V2_LOWER_RELINEARIZATION_THRESHOLD | 1 | 0.010000 | 0 |
| V3_STRICT_RELINEARIZATION_THRESHOLD | 1 | 0.001000 | 0 |
| V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES | 10 | 0.100000 | 3 |

## 6. Baseline replication

V0 reproduced original eight checkpoint pose statistics: True. See baseline_reproduction_audit.csv; runtime not required to match another host load/time.

## 7. Relinearization frequency ablation

V0 vs V1 changes only skip10→1; compare measured K2 and checkpoint differences below. More frequent CHECKS do not guarantee every variable is relinearized.

## 8. Threshold ablation

V1→V2→V3 threshold0.1→0.01→0.001, skip1 fixed, wildfire0.001 unchanged. No expanded grid.

## 9. Extra-update diagnostic

V4 defaults +exactly3 supported no-argument empty updates after each of75 loops, including first. Ordinary insertion time separate. Estimates0/1/2/3 stored server-only NPZ with hashes and checkpoint CSVs. Empty calls can be no-ops; exposed counters and changes recorded; V4 never eligible for standard selection.

## 10. Numerical agreement across checkpoints

| variant | checkpoint | translation_median_m | translation_p95_m | rotation_median_deg | rotation_p95_deg | PASS |
| --- | --- | --- | --- | --- | --- | --- |
| V0 | FIRST_LOOP | 0.000000 | 0.000000 | 0.000000 | 0.000000 | True |
| V0 | K2 | 3.382671 | 12.286618 | 1.342069 | 2.216089 | False |
| V0 | K5 | 0.039307 | 0.116133 | 0.023190 | 0.025991 | True |
| V0 | K10 | 1.005827 | 2.456849 | 0.601675 | 2.549446 | False |
| V0 | K20 | 0.074729 | 0.374318 | 0.035544 | 0.156282 | True |
| V0 | K40 | 0.211279 | 0.611506 | 0.160136 | 0.319450 | False |
| V0 | K75 | 0.123186 | 0.744521 | 0.054755 | 0.745961 | False |
| V0 | FINAL_STREAM_END | 0.124431 | 0.563985 | 0.055319 | 0.301537 | False |
| V1 | FIRST_LOOP | 0.000000 | 0.000000 | 0.000000 | 0.000000 | True |
| V1 | K2 | 3.382671 | 12.286618 | 1.342069 | 2.216089 | False |
| V1 | K5 | 0.039675 | 0.119524 | 0.023184 | 0.026005 | True |
| V1 | K10 | 0.037875 | 0.135119 | 0.021904 | 0.046144 | True |
| V1 | K20 | 0.071380 | 0.350590 | 0.025785 | 0.140472 | True |
| V1 | K40 | 0.210125 | 0.606550 | 0.160539 | 0.318259 | False |
| V1 | K75 | 0.122814 | 0.746215 | 0.055427 | 0.745035 | False |
| V1 | FINAL_STREAM_END | 0.124359 | 0.566468 | 0.055960 | 0.302735 | False |
| V2 | FIRST_LOOP | 0.000000 | 0.000000 | 0.000000 | 0.000000 | True |
| V2 | K2 | 3.382671 | 12.286618 | 1.342069 | 2.216089 | False |
| V2 | K5 | 0.011965 | 0.054942 | 0.005388 | 0.021448 | True |
| V2 | K10 | 0.036051 | 0.123531 | 0.019082 | 0.053691 | True |
| V2 | K20 | 0.060983 | 0.330064 | 0.022047 | 0.136461 | True |
| V2 | K40 | 0.213820 | 0.592589 | 0.160801 | 0.314250 | False |
| V2 | K75 | 0.106619 | 0.748666 | 0.036852 | 0.744008 | False |
| V2 | FINAL_STREAM_END | 0.108179 | 0.574742 | 0.036756 | 0.311722 | False |
| V3 | FIRST_LOOP | 0.000000 | 0.000000 | 0.000000 | 0.000000 | True |
| V3 | K2 | 3.382671 | 12.286618 | 1.342069 | 2.216089 | False |
| V3 | K5 | 0.009822 | 0.054061 | 0.005534 | 0.021055 | True |
| V3 | K10 | 0.039951 | 0.121750 | 0.019475 | 0.055046 | True |
| V3 | K20 | 0.061803 | 0.326725 | 0.021041 | 0.134461 | True |
| V3 | K40 | 0.209849 | 0.591039 | 0.159134 | 0.313269 | False |
| V3 | K75 | 0.109834 | 0.742261 | 0.037673 | 0.744023 | False |
| V3 | FINAL_STREAM_END | 0.111753 | 0.571735 | 0.037626 | 0.310047 | False |
| V4 | FIRST_LOOP | 0.000000 | 0.000000 | 0.000000 | 0.000000 | True |
| V4 | K2 | 0.469389 | 1.679341 | 0.398842 | 0.892435 | False |
| V4 | K5 | 0.039321 | 0.116849 | 0.023187 | 0.026013 | True |
| V4 | K10 | 0.183343 | 0.829108 | 0.195484 | 0.717421 | False |
| V4 | K20 | 0.064327 | 0.380850 | 0.031834 | 0.156830 | True |
| V4 | K40 | 0.089643 | 0.339738 | 0.042939 | 0.171297 | True |
| V4 | K75 | 0.122012 | 0.561976 | 0.055470 | 0.300404 | False |
| V4 | FINAL_STREAM_END | 0.123634 | 0.564448 | 0.055880 | 0.301812 | False |

Original gates0.10/0.50m translation and0.5/2deg rotation; all four at all eight checkpoints. Same gauge, no extra alignment. Objective agreement is not pose agreement.

## 11. K2/K10 discrepancy localization

Per-robot metrics and worst20 translation +worst20 rotation keys, anchor distance/latest-endpoint KF distance saved. <=50KF is a frozen descriptive near-endpoint rule, not a qualification gate. See localization.csv and figures; far-chain errors must not be described as exclusively endpoint-local.

## 12. Relinearization counts and limitations

Per-event counters measured from binding, NOT_EXPOSED if unavailable. Zero at a checkpoint does not imply zero in preceding events. Variants each use new ISAM2 instance; no warm-start from other trials.

## 13. Runtime tradeoff

| variant | status | checkpoints_passed | post_update_median_ms | post_update_mean_ms | post_update_p95_ms |
| --- | --- | --- | --- | --- | --- |
| V0_BASELINE_REPLICATION | SOME_CHECKPOINTS_FAIL | 3 | 0.046673 | 1.958639 | 16.544552 |
| V1_EVERY_EVENT_RELINEARIZATION_CHECK | SOME_CHECKPOINTS_FAIL | 4 | 0.214636 | 3.041299 | 25.865481 |
| V2_LOWER_RELINEARIZATION_THRESHOLD | SOME_CHECKPOINTS_FAIL | 4 | 0.224007 | 5.183803 | 37.908802 |
| V3_STRICT_RELINEARIZATION_THRESHOLD | SOME_CHECKPOINTS_FAIL | 4 | 0.231470 | 8.254052 | 40.190104 |
| V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES | SOME_CHECKPOINTS_FAIL | 4 | 0.046455 | 1.784194 | 12.628920 |

One replay each; first fusion one sample. Update/construction/extraction/validation/checkpoint/V4 diagnostic costs separately saved; no batch solves/maps/GT in timings.

## 14. 2Hz engineering diagnostic

| variant | events_with_next_interval | update_missed_deadlines | full_incremental_solver_processing_missed_deadlines | update_under_fraction | full_processing_under_fraction | V4_includes_extra_update_cost |
| --- | --- | --- | --- | --- | --- | --- |
| V0_BASELINE_REPLICATION | 1716 | 0 | 0 | 1.000000 | 1.000000 | False |
| V1_EVERY_EVENT_RELINEARIZATION_CHECK | 1716 | 0 | 0 | 1.000000 | 1.000000 | False |
| V2_LOWER_RELINEARIZATION_THRESHOLD | 1716 | 0 | 0 | 1.000000 | 1.000000 | False |
| V3_STRICT_RELINEARIZATION_THRESHOLD | 1716 | 0 | 0 | 1.000000 | 1.000000 | False |
| V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES | 1716 | 0 | 0 | 1.000000 | 1.000000 | True |

Backend-only, no sleep. ExtraV4 calls included in its solver/update-processing budget, never hidden in ordinary update. Full-system real-time capability NOT established.

## 15. Final graph equivalence

Every completed variant:5796 nodes/5794 odometry/75 loops/1 prior=5870 factors; GTSAM factor equals checks actual insertion ordering against immutable template, keys/registry/timestamps unchanged. Finite/unit quaternion/IDs and frozen catastrophic-step protocol checked.

## 16. GT-free selection decision

`NO_STANDARD_CONFIG_MET_GATES`; selected `None` at 2026-10-09T03:02:06.044076+00:00. Only V0–V3 all-checkpoint qualifiers; lowest median ordinary post-fusion update, p95 tie-break, then fewer parameter changes within predeclared1% timing indistinguishability. V4 excluded. GT_used_for_selection=false.

## 17. Evidence-supported interpretation

Controlled frequency/threshold changes support sensitivity statements only where measured contrasts demonstrate it. Caching, wildfire stopping, weak directions, different nonlinear paths remain alternative hypotheses; no claim that relinearization is the sole cause.

Measured controlled contrasts:

| variant | checkpoint | translation_median_m | translation_p95_m | absolute_gap |
| --- | --- | --- | --- | --- |
| V0 | K2 | 3.382671 | 12.286618 | 0.689949 |
| V0 | K10 | 1.005827 | 2.456849 | 4.574689 |
| V1 | K2 | 3.382671 | 12.286618 | 0.689949 |
| V1 | K10 | 0.037875 | 0.135119 | 0.002927 |
| V2 | K2 | 3.382671 | 12.286618 | 0.689949 |
| V2 | K10 | 0.036051 | 0.123531 | 0.003219 |
| V3 | K2 | 3.382671 | 12.286618 | 0.689949 |
| V3 | K10 | 0.039951 | 0.121750 | 0.003218 |
| V4 | K2 | 0.469389 | 1.679341 | 0.000524 |
| V4 | K10 | 0.183343 | 0.829108 | 0.123313 |

V0→V1 (skip10→1, threshold0.1 fixed) changes K10 translation p95 from 2.456849 to 0.135119m, but K2 stays unchanged. V1→V2→V3 threshold reduction gives modest further K10 improvement, not monotonic final improvement. Checkpoints passed: {'V0': 3, 'V1': 4, 'V2': 4, 'V3': 4, 'V4': 4}. NO CONFIGURATION PASSED ALL EIGHT CHECKPOINTS; no standard selection is justified.

V4 records 225 empty calls; 25 materially change the recorded pose estimate, 25 expose positive relinearized counts, and 225 expose positive re-eliminated counts. Many calls are pose-negligible despite re-elimination. Empty calls advance the skip counter as well as adding computation; this diagnostic does not isolate nonlinear iteration count alone.

Localization of failing K2/K10 checkpoints:

| variant | checkpoint | metric | largest_robot_p95 | top20_far_from_latest_endpoint | top20_max_kf_distance | extent |
| --- | --- | --- | --- | --- | --- | --- |
| V0_BASELINE_REPLICATION | K2 | translation | robot3 | 20 | 410 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V0_BASELINE_REPLICATION | K2 | rotation | robot3 | 20 | 218 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V0_BASELINE_REPLICATION | K10 | translation | robot3 | 20 | 174 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V0_BASELINE_REPLICATION | K10 | rotation | robot3 | 0 | 14 | MOSTLY_ENDPOINT_NEAR_OR_MIXED |
| V1_EVERY_EVENT_RELINEARIZATION_CHECK | K2 | translation | robot3 | 20 | 410 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V1_EVERY_EVENT_RELINEARIZATION_CHECK | K2 | rotation | robot3 | 20 | 218 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V2_LOWER_RELINEARIZATION_THRESHOLD | K2 | translation | robot3 | 20 | 410 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V2_LOWER_RELINEARIZATION_THRESHOLD | K2 | rotation | robot3 | 20 | 218 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V3_STRICT_RELINEARIZATION_THRESHOLD | K2 | translation | robot3 | 20 | 410 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V3_STRICT_RELINEARIZATION_THRESHOLD | K2 | rotation | robot3 | 20 | 218 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES | K2 | translation | robot3 | 20 | 1623 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES | K2 | rotation | robot3 | 20 | 1128 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES | K10 | translation | robot3 | 20 | 177 | LONG_CHAIN_NOT_ENDPOINT_ONLY |
| V4_DEFAULT_WITH_EXTRA_DIAGNOSTIC_UPDATES | K10 | rotation | robot3 | 0 | 16 | MOSTLY_ENDPOINT_NEAR_OR_MIXED |

Translation discrepancy is principally Robot3 and distributed beyond the recent endpoint neighborhood in this replay, not just a new Robot1 pose issue. Long-chain/weak-direction/path-dependence explanations remain hypotheses; no Hessian conditioning study or alternate optimizer path was performed. No GT trajectory accuracy or map consistency conclusion follows.


## 18. Unresolved numerical issues

Numerical agreement, backend runtime, trajectory accuracy, map consistency and full-system real-time capability are distinct. No GT or map inference here. Any unsupported/failed variant remains recorded, not silently repaired with another policy.

## 19. Next-stage recommendation

Stop after13C. If a primary config meets all gates, independently validate its robustness and engineering budget before demo use. If none passes, separately declare a further GT-free solver-method audit; do not choose near-pass, change loops/demo or tune from GT.
