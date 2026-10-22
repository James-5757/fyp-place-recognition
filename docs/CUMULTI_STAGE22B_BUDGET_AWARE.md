# Stage22B — budget-aware inter-robot geometric verification

Decision: **EXPERIMENT_INCOMPLETE**; within the observed scope the empirical resource/quality outcome is **NO_POLICY_MET_NONINFERIORITY**. This is not a live-robot or wireless demonstration. The heldout generalization/promotion phase is stopped because positive direction/phase coverage fails; already frozen diagnostic policies are retained, not retuned.

## 1. Hypothesis and contribution
An interpretable causal utility might avoid low-value GICP and raw-cloud requests. The contribution here is a reproducible controlled experiment with immutable outcome provenance, hard budgets, actual reduced-workload benchmarking, and independently rebuilt graphs; a negative result is retained.

## 2. Reference and limitations
Accepted source `c929fc55a9f42c149ce571205618da73db7d4bc5`. Exactly reproduced5796 events/15267 calls/1088 geometrically accepted queries/288 admitted loops/6107977604 application bytes. Hypothetical normalized synchronized replays of separately recorded R1/R3 runs, centralized backend, no real wireless measurement. Neither archived75 loops nor GT choose candidates.

## 3. Causal features
Current SC score/rank/margin, arrived opposite DB size/direction; own past admitted endpoint neighborhoods and distances; own cache residency; exact serialized request-cost catalog for available cloud identities; direction/rank CPU medians from development; remaining global/rolling budgets and past admitted count. Future GICP quality, GT distance, future frames and optimized poses are excluded. Features are recorded before each current rank decision; earlier ranks of the same query may have been observed. `latest_state_event` is only a prior-event watermark, not a complete cache freshness timestamp; the independent auditor reconstructs cache state rank by rank.

## 4. Method and controls
P0 exact Top3 early-stop; P1 Top1; P2 Top2; P3 top1 confidence>=development20th-percentile; P4 skip rank1 in an already admitted delta3/delta5 neighborhood. P5 utility = normalized confidence +0.1 normalized margin +0.1 novelty -0.1 redundancy -0.05 normalized predicted CPU -0.05 normalized requested bytes. Prefix order and first geometric acceptance stopping are fixed. First denied rank stops a query; no future-event postponement.

## 5. Development and independent holdout
Events0–999 development;1000–1099 embargo;1100–5795 heldout, continuing causal state without resetting cache/admissions. Development grid0/.1/.2/.3/.4/.5 retains>=98% measured geometric acceptance before minimizing calls/bytes; no GT correctness labels. Frozen threshold=0.5. Predeclared main finalist C75/B80, not chosen from heldout quality. Post-freeze GT audit finds ZERO positive-overlap queries among2599 heldout both-progressing events; all1411 heldout positive queries lie after R1 finishes. The holdout therefore cannot independently test positive R1-direction/early dual-stream discovery. Gate F is COMPLETED_WITH_COVERAGE_LIMITATION, not a scientific PASS. No label-based rebalance is performed. Early pre-cut opportunities remain development-only. Full coverage is in `offline_split_coverage.csv`. Near-duplicate correlations and one route limit external generalization; empirical gates are not a powered statistical noninferiority trial.

## 6. Budgets and normalization
Calls100/75/50/25% and communication100/80/60/40% of the same Stage22A full workload:16 P5 combinations. Hard cumulative floor caps;100-event rolling call cap floor(300*fraction), communication cap floor(100*baseline_bytes/5796*fraction), and at most3 calls/query. Fixed descriptor/candidate/odometry publication is reserved and counted. A249-byte potential loop is reserved before verifying; unused reservation is released. No result exceeds a declared cap. Normalization and cost medians are frozen in the policy JSON. Budget levels constrain consumption, not guaranteed utility or savings.

## 7. Measured outcome provenance
Every historical artifact, receipt-directory digest, raw cloud and pair result was hash-verified. 2112 candidate pairs unmeasured due to historical early stopping remain unknown. All selected calls are prefixes of measured attempted sequences, never inferred by SC/GT. Duplicate suppression is reconstructed from each policy's own discoveries; it is not filtering the288 historical loops.

## 8. Communication and cache sensitivity
B1 full-SC-first, fixed Stage21 binary framing, exact lossless XYZI, query-then-candidate requests. Fixed messages reuse same-event measured buffers; all variable buffers for the six primary policies are newly encoded/decoded/zlib1 compressed. `communication_by_policy.csv` distinguishes these from catalog-based budget sweeps and null compression values. Sends are PROTOCOL_SIMULATED, not network throughput/latency/energy. Primary central cache is unbounded; LRU1000/LRU128 are separate sensitivity conditions, not fair algorithm-only cross-cache comparisons. Physical GICP cache remains unchanged fresh64 per teammate call. Top1 can save calls while requesting almost identical clouds, so computation savings do not imply byte savings.

## 9. Actual GICP benchmark and repairs
Frozen120 query IDs, P0/P3/P5; same original geometry environment and one thread, same unmodified teammate implementation, no index acceleration. 762 comparable actual calls completed, plus 1 confirmed actual call whose outer timing report failed and whose raw result is preserved separately. Initial native-environment preflight made zero calls. Recovery preserves identical query/pair identifiers and scientific parameters; no unknown call is silently repeated. The extra known execution is counted, not relabeled a warmup. Some first-failure outer/CPU timings are NOT_MEASURED. Comparable sample timing separates cloud loading/preprocessing/coarse/fine/index/CSV and outer invocation costs; peakRSS retained. Full-stream runtime and wireless operation are NOT_MEASURED. OS filesystem-cache state is uncontrolled; fresh teammate cache is not a cold-OS-cache guarantee.

## 10. New graph reconstruction
All six primary full streams use only their own causally admitted factors, first discovered bridge initialization, original5796 nodes/5794 odometry/one prior and frozen noise/default persistent iSAM2. Expected factors5795+L. Independent same-graph LM and original0.10/0.50m,0.5/2deg agreement limits are retained. Failures are not repaired with archived factors, extra updates or relaxed thresholds. Per-policy structural/numerical outcomes remain separate.

## 11. Quality and empirical acceptance
Primary query recall uses the same causal available-positive (<5m) denominator before duplicate suppression; admitted precision/recall and false factors remain separate. GT is read only after decisions and finalists freeze, never for reoptimization. ATE uses one joint rigid alignment; per-robot ATE and interval10 RPE are reported. Mapping consistency is GT-free NN under the frozen map protocol. Every empirical threshold appears separately in `acceptance_gates_by_policy.csv`; no test-based winner selection.

## 12. Primary results

|Policy|Calls|Attempt saving|Application-byte saving|Full geometric P/R|Admitted / false|Joint ATE m|Same-graph numeric|
|---|---:|---:|---:|---|---|---:|---|
|P0_FULL_TOP3|15267|0.00%|0.00%|0.970588/0.718367|288/8|21.159290|PASS|
|P1_FIXED_TOP1|5795|62.04%|0.00%|0.972196/0.689796|280/5|20.328554|FAIL|
|P2_FIXED_TOP2|10545|30.93%|0.00%|0.971001/0.706122|285/7|20.438367|FAIL|
|P3_SC_CONFIDENCE_GATE|14079|7.78%|3.76%|0.970588/0.718367|288/8|21.159290|PASS|
|P4_CAUSAL_REDUNDANCY_GATE|14323|6.18%|13.58%|0.975945/0.193197|286/7|21.165165|PASS|
|P5_BUDGET_AWARE_C75_B80|8647|43.36%|31.57%|0.957503/0.490476|231/8|20.426784|PASS|

## 13. Ablations, failure cases and Pareto interpretation
Six fixed baselines/main conditions,16 budget levels,three cache capacities for P0/P5, and dev threshold grid remain separate. Test Pareto visualization is descriptive only; it does not authorize choosing another P5 budget as an independent successful heldout test. The three main observed limitations are: call savings need not remove unique raw-cloud transfers; neighborhood suppression loses query recall even if graph duplicates are redundant; a development-stationary region/confidence distribution need not transfer to later geometric opportunities, and hard rolling transfer caps may skip valuable late queries. See per-budget quality and numerical tables rather than assuming savings preserve maps.

## 14. Threats to validity
One pair of separate recordings and one chronological split; correlated neighbors and different route phases; development geometric acceptance is not true-place correctness; known historical aggregate GT context; host/OS cache timing; simulated central communication; index/CSV overhead in original harness; sparse actual replay slice; numerical sensitivity and nonmonotonic map accuracy. Neither artifact PASS nor point-estimate tolerances establish deployable SLAM, statistical equivalence or a publishable algorithmic gain.

## 15. Recommendation and stopping point
Preserve Stage22A as reference. No overall READY promotion is supported by this split. Use the observed-scope result to distinguish compute-only and negative evidence, without claiming full bidirectional generalization. A separately authorized next study first needs an independent recording/route with genuine positive opportunities in both directions/phases, and a preregistered split that does not rebalance this now-exposed heldout GT. Only then investigate incremental information/coverage and rolling-budget adaptation. Revisit transport/cloud representation only as a separately validated protocol change. STOP AFTER STAGE22B: no visual SLAM, distributed optimization, third robot or demo change.

## Reproduction and artifacts
Run `scripts/run_stage22b_budget_aware.sh` from the repository. Existing frozen phases resume only from verified results; uncertain actual intents block. Heavy raw scans/full traces/maps stay server-only and are hashed in the result manifest. Lightweight CSV summaries, figures, policies, tests and documentation are versioned. `stage22b_independent_validation.json` audits causal/byte/factor provenance, not universal algorithm success.
