# Stage13D — targeted ISAM2 convergence and diagnostic validity audit

Accepted checkpoint `49a53e89e8651ebfb31d2865d6dc9c3b895b2165`. Independent runner, historical B/C/demo unchanged.
Decision: **NUMERICAL_DISCREPANCY_PARTIALLY_EXPLAINED**. Qualifying diagnostic schedules: []. Not a production policy.

## Frozen design and graph provenance

Six checkpoints FIRST_LOOP/K2/K10/K40/K75/FINAL, three fresh persistent instances. D0 default skip10/threshold0.1, D1 same +exactly5 empty calls at these checkpoints, D2 skip1/threshold0.01 +same5 calls. Wildfire0.001, noise, Huber, measurements and KF150 anchor unchanged. Original four gates unchanged. No GT, scans, maps, online frontend or simultaneous robots. Robot3 stored; Robot1 alone streams. Extra calls change schedule and cannot be called one-update-per-event timing. Empirical no-change <1e-7m/<1e-7deg/<1e-9 objective on2 consecutive calls, still record all5; not proof of mathematical convergence.

## Counter validity correction

Invalid records: 494 (historical derived tables duplicate some returns; this is not a unique-call count). Examples: ['101444947035872', '18446744073709551488', '99275457627108', '99275457822918']. Raw values preserved as exact strings; out-of-range gets INVALID_COUNTER and null diagnostic value, never clipping or zeroing. Physical range0..nodes does not prove API semantics. Tiny independent Pose3 test covers insertions/empty/repeated empty calls, default detailed-results flag recorded. Its initial source-unknown annotation records knowledge at test time; subsequent verified official-tag findings are in counter_source_audit.json.

Official GTSAM4.2 [ISAM2Result.h](https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/nonlinear/ISAM2Result.h) shows that its constructor does not initialize the scalar counts. [ISAM2.cpp](https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/nonlinear/ISAM2.cpp) explicitly zeros the relinearization count but assigns the re-elimination count only inside conditional recalculation branches. Empty no-work paths can therefore expose uninitialized re-elimination memory. Tiny-graph and real-call out-of-range returns are consistent with this source mechanism. Exact wheel build identity and individual C++ branch execution were not instrumented. Even an in-range no-work return may be coincidental garbage. The relinearization getter is marked-set bookkeeping, not proof that every counted pose changed coordinates.

The Stage13C claim of225 positive re-elimination returns does **not** prove225 valid re-elimination events. This new interpretation does not modify historical CSVs or their numerical pose metrics.

## Microtrace/reference conventions

BEFORE_EVENT uses the preceding actual graph (KF-1, before grouped odometry/loop insertion), AFTER_INSERTION uses current KF. They are different graphs; each is compared only to its OWN identical-graph batch reference. FIRST_LOOP before-state has disconnected local frames, no common-frame objective/pose comparison. FINAL has no new loop. Six after-references reused from13B after accepted-blob/keys/objective checks. Five missing pre-event references solved ONCE for evaluation with frozen12D LM settings and common x0, never warm-started from ISAM2 or fed to ISAM2. No additional rigid alignment.

## Numerical results

| variant | checkpoint | normal_translation_p95_m | final_translation_p95_m | normal_PASS | final_PASS | extra_updates_attempted | first_agreement_step | initial_objective | final_objective | objective_gap | classification | counter_diagnostic |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| D0 | FIRST_LOOP | 1.3781204854500086e-12 | 1.3781204854500086e-12 | True | True | 0 | 0.0 | 1.1018653835900711e-24 | 1.1018653835900711e-24 | 1.0342291908113011e-24 | AGREEMENT_REACHED | IN_RANGE_SEMANTICS_UNVERIFIED |
| D0 | K2 | 12.286617828357349 | 12.286617828357349 | False | False | 0 | nan | 0.7683611578192497 | 0.7683611578192497 | 0.6899491274819456 | STILL_DIVERGING | IN_RANGE_SEMANTICS_UNVERIFIED |
| D0 | K10 | 2.456849108557954 | 2.456849108557954 | False | False | 0 | nan | 6.298025206191291 | 6.298025206191291 | 4.574689192096527 | STILL_DIVERGING | IN_RANGE_SEMANTICS_UNVERIFIED |
| D0 | K40 | 0.611506028555215 | 0.611506028555215 | False | False | 0 | nan | 15.809222758665936 | 15.809222758665936 | 0.0382486148457204 | STILL_DIVERGING | IN_RANGE_SEMANTICS_UNVERIFIED |
| D0 | K75 | 0.7445213459990747 | 0.7445213459990747 | False | False | 0 | nan | 25.06065235565471 | 25.06065235565471 | 1.2202687001550885 | STILL_DIVERGING | IN_RANGE_SEMANTICS_UNVERIFIED |
| D0 | FINAL_STREAM_END | 0.5639854280275796 | 0.5639854280275796 | False | False | 0 | nan | 23.840417589292866 | 23.840417589292866 | 3.394381436905292e-05 | OBJECTIVE_CONVERGED_POSE_GATE_FAILED | IN_RANGE_SEMANTICS_UNVERIFIED |
| D1 | FIRST_LOOP | 1.3781204854500086e-12 | 1.3781204854500086e-12 | True | True | 5 | 0.0 | 1.1018653835900711e-24 | 1.1018653835900711e-24 | 1.0342291908113011e-24 | AGREEMENT_REACHED | INVALID_COUNTER_DIAGNOSTIC |
| D1 | K2 | 12.286617828357349 | 12.286617828357349 | False | False | 5 | nan | 0.7683611578192497 | 0.7683611578192497 | 0.6899491274819456 | NO_MATERIAL_CHANGE | INVALID_COUNTER_DIAGNOSTIC |
| D1 | K10 | 2.456849108557954 | 0.7750400472327883 | False | False | 5 | nan | 6.298025206191291 | 1.7629882001393886 | 0.0396521860446252 | STILL_DIVERGING | INVALID_COUNTER_DIAGNOSTIC |
| D1 | K40 | 0.6259784923560783 | 0.3565405578822783 | False | True | 5 | 3.0 | 15.809210840879176 | 15.771002013085411 | 2.786926519604549e-05 | AGREEMENT_REACHED | INVALID_COUNTER_DIAGNOSTIC |
| D1 | K75 | 0.744828101239411 | 0.5614442325081597 | False | False | 5 | nan | 25.06065236234009 | 23.84068033123419 | 0.0002966757345674 | OBJECTIVE_CONVERGED_POSE_GATE_FAILED | INVALID_COUNTER_DIAGNOSTIC |
| D1 | FINAL_STREAM_END | 0.5638894177319658 | 0.5638894177319658 | False | False | 5 | nan | 23.840417603129826 | 23.840417603129826 | 3.395765133262785e-05 | OBJECTIVE_CONVERGED_POSE_GATE_FAILED | INVALID_COUNTER_DIAGNOSTIC |
| D2 | FIRST_LOOP | 1.3781204854500086e-12 | 1.3781204854500086e-12 | True | True | 5 | 0.0 | 1.1018653835900711e-24 | 1.1018653835900711e-24 | 1.0342291908113011e-24 | AGREEMENT_REACHED | INVALID_COUNTER_DIAGNOSTIC |
| D2 | K2 | 12.286617828357349 | 0.055109175091443 | False | True | 5 | 2.0 | 0.7683611578192497 | 0.0784122829926101 | 2.526553059684078e-07 | AGREEMENT_REACHED | IN_RANGE_SEMANTICS_UNVERIFIED |
| D2 | K10 | 0.1235313625043528 | 0.1268702281355858 | True | True | 5 | 0.0 | 1.7265550215473384 | 1.723335860075143 | 1.5401962039085507e-07 | AGREEMENT_REACHED | IN_RANGE_SEMANTICS_UNVERIFIED |
| D2 | K40 | 0.5925888309517651 | 0.309202380781279 | False | True | 5 | 1.0 | 15.807421151999494 | 15.771004787177242 | 3.0643357026960416e-05 | AGREEMENT_REACHED | IN_RANGE_SEMANTICS_UNVERIFIED |
| D2 | K75 | 0.7486658125347173 | 0.5723921583406472 | False | False | 5 | nan | 25.05874362576005 | 23.84038893788152 | 5.28238189900776e-06 | OBJECTIVE_CONVERGED_POSE_GATE_FAILED | IN_RANGE_SEMANTICS_UNVERIFIED |
| D2 | FINAL_STREAM_END | 0.5747419196126619 | 0.5747419196126619 | False | False | 5 | nan | 23.84038886834764 | 23.84038886834764 | 5.222869145171671e-06 | OBJECTIVE_CONVERGED_POSE_GATE_FAILED | INVALID_COUNTER_DIAGNOSTIC |

Full step0..5 graph objective, same-gauge median/p95 translation/rotation, counters and cost are in checkpoint_microtrace.csv. Objective plateau classification is descriptive absolute batch gap<=1e-3, not a relaxed pose gate or mathematical convergence test. Near-equal objective is not numerical pose agreement.

K2 complete after-insertion/extra trace:

| variant | extra_step | translation_median_m | translation_p95_m | nonlinear_error | objective_gap | PASS |
| --- | --- | --- | --- | --- | --- | --- |
| D0 | 0 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D1 | 0 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D1 | 1 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D1 | 2 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D1 | 3 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D1 | 4 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D1 | 5 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D2 | 0 | 3.382671272149987 | 12.286617828357349 | 0.7683611578192497 | 0.6899491274819456 | False |
| D2 | 1 | 0.469748973570691 | 1.6798334974271805 | 0.0789360911693129 | 0.0005240608320087 | False |
| D2 | 2 | 0.0196313674891488 | 0.1063971976072673 | 0.0784127251030737 | 6.947657695960396e-07 | True |
| D2 | 3 | 0.0074692409803362 | 0.0455015614113887 | 0.078412016737881 | 1.3599423143162426e-08 | True |
| D2 | 4 | 0.0092680879562641 | 0.055087022092037 | 0.0784120036060311 | 2.6731273070068617e-08 | True |
| D2 | 5 | 0.0090606945068178 | 0.055109175091443 | 0.0784122829926101 | 2.526553059684078e-07 | True |

## Measured schedule/convergence interpretation

Official [ISAM2-impl.h](https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/nonlinear/ISAM2-impl.h) uses a modulo skip check; each empty call advances the counter. D1's K2 normal call is ordinal101 after its five FIRST_LOOP calls, and extras102–106 reach no skip10 boundary. All five return zero relinearization bookkeeping and leave objective/poses unchanged. Thus five empty calls do not automatically mean five nonlinear relinearization iterations. This explains the apparent contrast with Stage13C V4's different all-loop schedule, without modifying it. See update_schedule_analysis.csv.

D2 checks every call. At K2, extra1 sharply reduces objective and pose disagreement; additional calls improve it enough to meet all four original gates, but later steps are not strictly monotonic. At K10 the normal D2 state already passes; K40 extra updates improve agreement. K75 and FINAL remain above the original pose gates even when objectives nearly match batch. Objective convergence and pose agreement do not coincide generally; no condition passes all six selected checkpoints.

## Localization and residuals

Per-scope errors/worst20 each robot and full nodewise differences retained for every state. Loop physical metric E=Z^-1 Xi^-1 Xj, translation norm and rotation angle; only already inserted loops. No future-loop residual evaluation. K2 Robot3 chain distribution is plotted. Hessian conditioning: NOT_TESTED; no dense6N matrix or nullspace claim. Long-chain deformation/path dependence are hypotheses unless separately measured.

K2 per-robot disagreement before/after diagnostics:

| variant | extra_step | scope | translation_median_m | translation_p95_m | rotation_p95_deg |
| --- | --- | --- | --- | --- | --- |
| D0 | 0 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D0 | 0 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D1 | 0 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D1 | 0 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D1 | 1 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D1 | 1 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D1 | 2 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D1 | 2 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D1 | 3 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D1 | 3 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D1 | 4 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D1 | 4 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D1 | 5 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D1 | 5 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D2 | 0 | robot1 | 2.175734784748796e-05 | 0.0347555743550594 | 0.3822408359005045 |
| D2 | 0 | robot3 | 3.544860561163233 | 12.3391559842754 | 2.2502333023924708 |
| D2 | 1 | robot1 | 2.1757699432671464e-05 | 0.0117675541288543 | 0.1365753630482198 |
| D2 | 1 | robot3 | 0.5121704610616449 | 1.6950855563319145 | 0.8931331491006949 |
| D2 | 2 | robot1 | 2.150679317082824e-05 | 0.0007583628234505 | 0.0042056409338153 |
| D2 | 2 | robot3 | 0.0213169083412706 | 0.1089929444912425 | 0.0634275698845089 |
| D2 | 3 | robot1 | 2.05701669545972e-05 | 0.0001957055855791 | 0.0014151238326884 |
| D2 | 3 | robot3 | 0.0081238603240718 | 0.046441217706864 | 0.0156887631013964 |
| D2 | 4 | robot1 | 2.1757831145430185e-05 | 0.0002289141786005 | 0.002095100728401 |
| D2 | 4 | robot3 | 0.0098346650299008 | 0.0564142400183504 | 0.0213096414596556 |
| D2 | 5 | robot1 | 2.1757831145430185e-05 | 0.0002289141786005 | 0.002095100728401 |
| D2 | 5 | robot3 | 0.0094747901447022 | 0.0564411871943564 | 0.0210274803493076 |

## Timing

| variant | kind | count | normal_update_median_ms | normal_update_p95_ms | extra_update_total_ms | extraction_total_ms | backend_next_interval_misses |
| --- | --- | --- | --- | --- | --- | --- | --- |
| D0 | FIRST_FUSION | 1 | 24.112654384225607 | 24.112654384225607 | 0.0 | 3.558829892426729 | 0 |
| D0 | ODOMETRY | 1642 | 0.0517335720360279 | 16.736079193651516 | 0.0 | 4413.393540307879 | 0 |
| D0 | LOOP | 74 | 0.5106555763632059 | 21.28581162542097 | 0.0 | 418.5086856596172 | 0 |
| D1 | FIRST_FUSION | 1 | 28.660347685217857 | 28.660347685217857 | 0.0610011629760265 | 11.885665822774172 | 0 |
| D1 | ODOMETRY | 1642 | 0.05179783329367635 | 16.893963469192194 | 0.0808299519121646 | 4501.1669574305415 | 0 |
| D1 | LOOP | 74 | 0.49410248175263405 | 7.95051262248307 | 79.01472365483642 | 476.5597069635987 | 0 |
| D2 | FIRST_FUSION | 1 | 31.823312863707542 | 31.823312863707542 | 0.6917314603924751 | 12.242590077221394 | 0 |
| D2 | ODOMETRY | 1642 | 0.23380503989756105 | 37.95545543543994 | 1.0235952213406565 | 5144.459572155029 | 0 |
| D2 | LOOP | 74 | 0.8234567940235138 | 37.6983358990401 | 475.47739185392857 | 540.6556511297822 | 0 |

Ordinary and extra update/extraction/construction times distinct. Objective computation, batch comparison, full diagnostics, validation and reference-LM costs recorded separately; never included as ordinary update. Backend next-interval comparison uses actual timestamps and includes extra-call processing, excludes offline diagnostics and whole-system frontend/network/map work. Single replay/first fusion one sample. See diagnostic_update_timing.csv, checkpoint_cost_breakdown.csv and batch_reference_provenance.csv.

| variant | replay_elapsed_UTC_ms | backend_processing_total_ms | non_backend_diagnostic_logging_overhead_ms | extra_update_total_ms | clock_limitation |
| --- | --- | --- | --- | --- | --- |
| D0 | 45274.39 | 8481.784985400736 | 36792.60501459926 | 0.0 | derived from recorded UTC start/end, not primary monotonic latency benchmark; excludes startup/reference LM |
| D1 | 65811.598 | 8677.752127870917 | 57133.84587212908 | 79.15655476972461 | derived from recorded UTC start/end, not primary monotonic latency benchmark; excludes startup/reference LM |
| D2 | 72852.653 | 15094.097010791302 | 57758.555989208704 | 477.1927185356617 | derived from recorded UTC start/end, not primary monotonic latency benchmark; excludes startup/reference LM |

Replay elapsed/aggregate residual diagnostic overhead above is derived from captured UTC event timestamps, not substituted for monotonic per-call timings. It includes integrity checks, counter collection, microtrace diagnostics, logging and serialization; startup/template and the five reference LM solves are separate. No end-to-end real-time claim.

## Reproduction and limitations

Run `bash scripts/run_stage13d_solver_diagnostics.sh` from root with a new empty stage directory. Runner refuses overwrite. `--report-only` regenerates figures/doc from saved data, without optimizer or decision rewrite. Frozen D0 comparisons, exact final factor identity, no future R1 nodes, unit-quaternion schema, finite poses and frozen catastrophic-step tests are recorded. Current summary: Counter initialization source unverified; no reliable Hessian/conditioning measurement. No claim of sole cause or production readiness.

## Next-stage recommendation

Stop after13D. If a diagnostic schedule reaches all selected gates, validate it under a separately predeclared all-checkpoint/held-out protocol before any deployment. Otherwise further GT-free stopping/linearization/weak-direction study is needed; do not loosen gates, choose loops, change demo or tune from GT. Historical13B/C remain negative results regardless of this diagnostic.
