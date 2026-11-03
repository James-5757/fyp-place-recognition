"""Independent artifact checks and mechanically generated Stage23C report."""
from common import *

def table(df, columns):
    def fmt(x):
        if pd.isna(x): return 'unavailable'
        return format(x,'.6g') if isinstance(x,(float,np.floating)) else str(x)
    return '| '+' | '.join(columns)+' |\n| '+' | '.join(['---']*len(columns))+' |\n'+'\n'.join('| '+' | '.join(fmt(r[c]) for c in columns)+' |' for r in df.to_dict('records'))+'\n'

def main():
    verify();checks={}
    s=read(OUT/'criticality_mechanism_summary.json');syn=read(OUT/'synthetic_information_sanity.json')
    f=pd.read_csv(OUT/'information_vs_mapping_impact.csv');q=pd.read_csv(OUT/'causal_prefix_measurement_residuals.csv');lin=pd.read_csv(OUT/'information_linearization_audit.csv');g=pd.read_csv(OUT/'group_information_interaction.csv');corr=pd.read_csv(OUT/'spearman_bootstrap_correlations.csv');comp=read(OUT/'stage23c_graph_computation_audit.json')
    checks['frozen_input_hashes_match']=True
    checks['synthetic_information_sanity']=syn['status']=='PASS'
    checks['canonical_cohorts']=len(f)==372 and f.membership.value_counts().to_dict()=={'COMMON':147,'P0_ONLY':141,'P5_ONLY':84} and not f.duplicated(['robot1_kf','robot3_kf']).any()
    checks['measured_outcomes_only']=f.groupby('membership').direct_measured_single_LOO.sum().to_dict()=={'COMMON':0,'P0_ONLY':141,'P5_ONLY':7}
    checks['actual_receipts_and_prior_events']=len(q)==372 and q.receipt_verified.all() and (q.max_prior_factor_event<q.discovery_event).all()
    provenance=read(OUT/'causal_state_availability.json')
    checks['immutable_prefix_provenance']=sha(OUT/provenance['prefix_quality_file'])==provenance['prefix_quality_sha256'] and not provenance['final_states_used_as_causal']
    checks['no_optimized_causal_state_substitution']=(q.optimized_causal_state=='OPTIMIZED_CAUSAL_STATE_NOT_AVAILABLE').all() and not q.prefix_is_optimized.any()
    checks['current_measurement_features_are_post_verification']=(q.current_measurement_features_availability=='POST_VERIFICATION_NOT_PRE_VERIFICATION').all()
    checks['fourteen_fixed_covariance_decompositions']=len(lin)==14 and (lin.fixed_factor_removal_cov_component>=-1e-5).all() and abs(lin.decomposition_closure_error).max()<1e-5 and lin.actual_removed_factor_weight_after.isna().all()
    checks['seven_exact_joint_group_calculations']=len(g)==7 and (g.exact_fixed_Gaussian_group_information_nats>=0).all() and (g.information_redundancy_nats>=-1e-5).all() and not g.global_dense_information_matrix.any()
    checks['small_cohort_uncertainty_not_fabricated']=corr[corr.n<20].CI_95_low.isna().all() and corr[corr.n<20].CI_95_high.isna().all()
    checks['eight_false_factors_unfiltered']=(~f.offline_correct_d_xy_lt5).sum()==8
    plans={c['name']:c for c in read(OLD/'stage23b_policy.json')['cases']}
    source=pd.read_csv(OLD/'critical_loop_loo.csv')
    expected={tuple(plans[r['case']]['remove'][0]):r['delta_joint_ATE_RMSE_m'] for r in source.to_dict('records')}
    checks['signed_LOO_values_unchanged']=all(np.isclose(r.delta_joint_ATE_RMSE_m,expected[(r.robot1_kf,r.robot3_kf)],rtol=0,atol=1e-12) for r in f[f.direct_measured_single_LOO].itertuples())
    recovery=read(OUT/'prefix_provenance_engineering_recovery.json') if (OUT/'prefix_provenance_engineering_recovery.json').exists() else {'previous_calculation':{'elapsed_s':0,'peak_RSS_mib':0}}
    total=comp['elapsed_s']+recovery['previous_calculation']['elapsed_s']+s['analysis_elapsed_s']
    peak=max(comp['peak_RSS_mib'],recovery['previous_calculation']['peak_RSS_mib'])
    checks['bounded_sparse_computation']=comp['status']=='COMPLETE' and total<900 and peak<2048 and not comp['dense_global_matrix']
    checks['no_new_solver_GICP_or_selector']=s['new_GICP_calls']==s['new_nonlinear_solver_calls']==0 and not s['regression_or_selector_fitted']
    checks['independent_final_validation_not_executed']=s['final_independent_validation']=='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE' and s['unknown_GICP_candidates_unchanged']==2112
    required=['stage23c_input_manifest.json','synthetic_information_sanity.json','information_linearization_audit.csv','gicp_measurement_quality_audit.csv','pre_post_verification_feature_inventory.csv','information_vs_mapping_impact.csv','group_information_interaction.csv','criticality_mechanism_summary.json']
    checks['required_outputs_exist']=all((OUT/p).is_file() for p in required)
    validation=dict(status='PASS' if all(checks.values()) else 'FAIL',checks={k:bool(v) for k,v in checks.items()},scope='artifact consistency validation, NOT independent scientific testing',total_recorded_additional_calculation_seconds=total,peak_graph_RSS_MiB=peak,engineering_repeat_included=True)
    dump('stage23c_independent_artifact_validation.json',validation)
    assert all(checks.values()),validation
    primary=corr[(corr.cohort=='P0_ONLY')&(corr.CI_method=='CLUSTER')&(corr.top_K==10)]
    descriptive=pd.read_csv(OUT/'false_true_measurement_descriptives.csv')
    weight_evidence=dict(representatives=len(lin),representatives_with_retained_weight_change=int((lin.retained_factor_max_abs_weight_change>1e-12).sum()),maximum_retained_weight_change=float(lin.retained_factor_max_abs_weight_change.max()),maximum_covariance_decomposition_closure_error=float(abs(lin.decomposition_closure_error).max()),interpretation='zero retained-weight change means this representative audit shows no robust-reweight mechanism; state/Jacobian variation remains distinct')
    text='''# Stage23C — Nominal information versus realized mapping impact

Status: diagnostic artifact validation PASS. Scientific conclusion: **NOMINAL_INFORMATION_PROXY_NOT_SUPPORTED** for the original proxy on this exposed recording; **GROUP_INTERACTION_IMPORTANT** is an additional observation. This is RETROSPECTIVE_EXPLORATORY, not an independent predictive validation or a universal impossibility claim.

## 1. Why theoretical information increases
For an anchored, fixed-linearization Gaussian model, adding A-transpose A is PSD. The determinant lemma gives information gain 0.5 logdet(I + A C A-transpose), while modeled covariance cannot increase. Synthetic tests cover anchoring/gauge, sparse solves, Pose3 rotation-then-translation tangent ordering, exact nonzero-residual Jacobians and joint endpoint cross-correlations. This identity does not compare different nonlinear robust solutions.

Synthetic numerical evidence (generated from the artifact):
'''+table(pd.DataFrame([syn]),['fixed_PSD_information_gain_nats','covariance_trace_before','covariance_trace_after','biased_realized_error','Pose3_exact_Logmap_Jacobian_max_error'])+'''
## 2. Why realized ATE may not improve
Expected covariance assumes the stated measurement model; realized trajectory error is a different quantity. The biased synthetic measurement decreases covariance and increases actual error. This is a mechanism demonstration, not evidence that real GICP receipts are biased. Drift, measurement quality, robust weighting, state-dependent Jacobians, redundancy and changed nonlinear solutions can all matter. Negative signed LOO effects remain negative.

## 3. Exact Stage23B proxy limitations
The original nominal score uses a causal P0 common-odometry prefix and predicted endpoint relation, not an optimized posterior and not the current GICP transform. Its state, covariance and factor set differ from the final Batch graph. Actual prior-event optimized P0/P5 snapshots are **OPTIMIZED_CAUSAL_STATE_NOT_AVAILABLE**. Historical final trajectories are retrospective only; Stage20 checkpoints belong to a different graph and cannot replace the missing states. First-bridge prefix residuals are explicitly unavailable.

The canonical union contains 141 P0-only, 84 P5-only and 147 common factors. Direct single-factor LOO exists for 141 P0-only and seven P5-only factors; the other 77 P5-only and 147 common factors have no individual outcome.

## 4. Measured GICP quality and residual findings
All union factors are linked to actual frozen JSON/CSV receipts, canonical transforms and original candidate direction. No registration was executed. The following descriptive comparison uses the eight already-known offline false factors; no online threshold was designed, and no factor was removed. A large residual may also reflect correct constraints against drifted odometry.
'''+table(descriptive,['offline_GT_positive','field','n','median','p95'])+'''
The false factors have plausible registration quality and their prefix residual median is not larger than that of correct factors in this cohort. Thus neither receipt acceptance nor a large odometry disagreement establishes correctness. The correlation table shows only a weak positive post-GICP quality association; it does not establish a deployable pre-verification predictor.

P0-only correlation table: target labels remain signed and offline-only. Top-K is descriptive overlap with largest signed LOO harm, not a trained selector. Random reference is K/n. Primary confidence intervals resample the seven frozen temporal clusters; IID intervals are provided only as dependence-ignoring sensitivity. Seven exposed-route clusters are not independent recordings. P5-only n=7 has no bootstrap confidence interval; multiple comparisons are uncorrected and exploratory.
'''+table(primary,['predictor','target','n','Spearman_rho','CI_95_low','CI_95_high','top_K_exact_overlap','random_overlap_reference'])+'''
## 5. Nonlinear and robust-weighting findings
Fourteen predeclared Stage23B representatives use sparse joint marginals. Full versus removed-factor covariance at identical frozen full-P0 values isolates the fixed Gaussian removal term. Then existing frozen LOO states change the state and robust weighting jointly; changing the endpoint projection Jacobian contributes a third term. The decomposition closes numerically, but it does not identify a unique percentage caused by robust weighting. Retained factors have actual before/after reconstructed weights. A removed factor has no actual after-removal weight; its LOO-state counterfactual weight is labeled separately. Weak-direction residual projections are diagnostic, not a selector. The endpoint projection uses the predicted-between tangent Jacobian; actual measurement information uses the frozen factor's Gaussian linearization, including its residual-dependent Jacobian and robust noise.
'''+table(lin,['case','fixed_factor_removal_cov_component','state_and_robust_reweight_cov_component','projection_J_cov_component','retained_factor_max_abs_weight_change'])+'\nObserved weight evidence:\n\n```json\n'+json.dumps(weight_evidence,indent=2)+'\n```\n'+'''
## 6. Group-level interactions
The bounded two-group pilot preceded all seven exact group calculations. Joint endpoint covariance includes cross-blocks. Exact group information and the sum of single-factor additions are evaluated against the SAME group-removed Gaussian base at the SAME full-P0 values. Their difference is Gaussian redundancy. Mean original prefix information uses another model and is not interchangeable with either. Group-removal offline ATE minus the sum of individual signed LOO effects reveals nonadditivity, not a unique causal decomposition. Regional masks differ between individual cases, so regional effects are not added. Single-LOO map effects were not measured; existing group regional map metrics are preserved explicitly.
'''+table(g,['case','factors','exact_fixed_Gaussian_group_information_nats','sum_individual_addition_information_same_base_nats','mean_original_prefix_nominal_information','measured_group_delta_joint_ATE_m','sum_measured_single_LOO_delta_joint_ATE_m','nonadditive_joint_ATE_interaction_m'])+'''
## 7. What is a causal pre-GICP feature
SC score/rank, frozen normalized confidence, prefix uncertainty, prior-event coverage and estimated transfer bytes are PRE_VERIFICATION. GICP fitness/RMSE/quality, measured transform and pre-insertion measurement residual/Huber weight are POST_VERIFICATION: the word pre-insertion does not mean pre-registration. Full-graph residuals and weights are RETROSPECTIVE_GRAPH_ANALYSIS. Known correctness and LOO ATE are OFFLINE_GT_EVALUATION. Post-verification scores cannot justify avoiding the exchange/registration that produced them. See the explicit feature inventory and immutable prefix provenance.

## 8. Does information gain deserve further use?
The original nominal proxy is not supported as an information-first ranking on this recording. Mathematical PSD behavior passes, but this does not validate its mapping utility. Nominal information is positively associated with the measured covariance-change target while negatively associated with signed offline ATE impact (see the generated table); modeled uncertainty and realized accuracy therefore disagree without violating the Gaussian identity. Receipt-quality associations are exploratory and post-verification; no optimized feature combination, cutoff, weight or model is chosen. No retained-factor weight change was observed in the representative audit, so robust reweighting is not demonstrated as the cause there. The covariance decomposition and group interactions explain distinctions and mechanisms, not a complete causal attribution of the negative correlation.

## 9. Recommended prospective P6 architecture
Prioritize communication-aware P5 improvements, cache reuse, explicit coverage constraints and budget scheduling. If information is revisited, predeclare its causal state/model and evaluate on genuinely unseen recordings with adequate bidirectional coverage. No P6, learned selector, demo change or distributed SLAM was implemented here.

## 10. Data coverage, resources and reproducibility
STAGE23_FINAL_INDEPENDENT_VALIDATION remains **NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE**. The 2112 unmeasured GICP candidates remain unknown. Exposed offline GT was used only through frozen Stage23B labels/evaluation results, never for retrieval, fitting or parameter selection. No independent final testing occurred.

Run from repository root: `scripts/run_stage23c_information_validity.sh`. The isolated GTSAM environment is unchanged; existing Stage23A/B inputs, root context, NEXT_STEPS and unrelated dirty files remain preserved. All input hashes and the predeclared policy are checked. No dense global information matrix, new nonlinear optimization or GICP execution is required. A provenance-only engineering repeat separated the immutable causal-prefix table from its later retrospective augmentation; both sparse calculation costs are retained in the recovery audit. Timing is machine-dependent. Lightweight results are sufficient; no raw dataset, bag, credential, model or hidden script is required beyond the documented frozen server inputs.
'''
    text+='\nRecorded resource and validation summary:\n\n```json\n'+json.dumps(validation,indent=2)+'\n```\n'
    (ROOT/'docs/CUMULTI_STAGE23C_INFORMATION_VALIDITY.md').write_text(text)
    dump('stage23c_artifact_manifest.json',{p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in OUT.iterdir() if p.is_file() and p.name!='stage23c_artifact_manifest.json'})
    print(json.dumps(validation))

if __name__=='__main__':main()
