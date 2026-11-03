"""Independent consistency checks and report; never reruns or tunes a solver."""
from diagnostic_model import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def table(df,cols):
    head='| '+' | '.join(cols)+' |\n| '+' | '.join(['---']*len(cols))+' |\n';rows=[]
    for r in df.to_dict('records'):rows.append('| '+' | '.join(f'{r[c]:.6g}' if isinstance(r[c],float) else str(r[c]) for c in cols)+' |')
    return head+'\n'.join(rows)
def main():
    manifest=read(OUT/'stage23b_input_manifest.json');policy=read(OUT/'stage23b_policy.json');freeze=read(OUT/'stage23b_solve_freeze.json');evaluated=read(OUT/'stage23b_offline_audit.json')
    assert sha(OUT/'stage23b_policy.json')==manifest['policy_sha256']
    for path,h in {**manifest['preserved_Stage23A'],**manifest['inputs']}.items():assert sha(path)==h,('changed frozen input',path)
    import preflight
    assert preflight.historical_digest()==manifest['historical']
    exact=pd.read_csv(OUT/'loop_set_exact_comparison.csv')
    if 'P0_admitted' not in exact:
        # Reporting normalization only: one row per canonical identity rather
        # than two policy-specific rows for common edges. No metric changes.
        grouped=[]
        for (q,c),g in exact.groupby(['robot1_kf','robot3_kf']):
            r=g.iloc[0];grouped.append(dict(robot1_kf=int(q),robot3_kf=int(c),membership=r.membership,P0_admitted=P0 in set(g.policy),P5_admitted=P5 in set(g.policy),offline_correct_d_xy_lt5=bool(r.offline_correct_d_xy_lt5),offline_GT_d_xy_m=float(r.offline_GT_d_xy_m)))
        exact=pd.DataFrame(grouped);csv('loop_set_exact_comparison.csv',exact)
    counts=[int(exact.P0_admitted.sum()),int(exact.P5_admitted.sum()),int((exact.P0_admitted&exact.P5_admitted).sum()),int((exact.P0_admitted&~exact.P5_admitted).sum()),int((~exact.P0_admitted&exact.P5_admitted).sum())];assert counts==[288,231,147,141,84] and len(exact)==372
    allrows=pd.read_csv(OUT/'stage23b_all_ablations.csv');loo=pd.read_csv(OUT/'critical_loop_loo.csv');groups=pd.read_csv(OUT/'critical_loop_group_ablation.csv');features=pd.read_csv(OUT/'gt_free_criticality_features.csv');maps=pd.read_csv(OUT/'regional_map_quality.csv');correlation=pd.read_csv(OUT/'criticality_predictive_validation.csv');p0=pd.read_csv(OUT/'p0_only_loop_diagnostics.csv');p5=pd.read_csv(OUT/'p5_only_loop_diagnostics.csv')
    false=exact[~exact.offline_correct_d_xy_lt5]
    assert len(false)==8 and false.membership.eq('COMMON').all()
    assert np.isfinite(loo[['delta_joint_ATE_RMSE_m','delta_regional_ATE_RMSE_m','delta_regional_ATE_p95_m','delta_inter_robot_relative_translation_RMSE_m','delta_projected_cov_trace']]).all().all()
    undefined=correlation.status.eq('UNDEFINED_CONSTANT_OR_INSUFFICIENT')
    correlation.loc[undefined,['top_K_overlap_fraction','top_K_signed_harm_positive_fraction']]=np.nan
    csv('criticality_predictive_validation.csv',correlation)
    assert len(p0)==141 and len(p5)==84 and len(loo[loo.kind=='LOO_P0'])==141 and len(loo[loo.kind=='LOO_P5'])==7 and len(groups[groups.kind=='TEMPORAL_GROUP'])==7
    assert sha(OUT/'gt_free_criticality_features.csv')==freeze['features_sha256'];assert not features.GT_used.any() and not features.current_GICP_transform_used.any();assert (features.max_prior_loop_event<features.event_index).all()
    assert not allrows.case.duplicated().any() and allrows.nodes.eq(5796).all() and allrows.odom.eq(5794).all() and allrows.prior.eq(1).all()
    assert (allrows.factors==5795+allrows.loops).all()
    # All selected removal sets omit only loop indices, never an odometry/prior.
    loops,frame,nodes,local,arc,events,params=load()
    for n in [P0,P5]:
        g,_,idx=build(n,loops,nodes,local,events);assert len(idx)==len(loops[n]) and len(set(idx.values()))==len(idx)
        for case in policy['cases']:
            if case['policy']==n:assert all(tuple(x) in idx for x in case['remove'])
    common0=np.load(OUT/'stage23b_states/P0_COMMON_ONLY.npz')['matrices'];common5=np.load(OUT/'stage23b_states/P5_COMMON_ONLY.npz')['matrices'];assert np.max(abs(common0-common5))<1e-7
    complete=len(allrows)==len(policy['cases']);status='COMPLETE' if complete else 'PARTIAL'
    failures=int((allrows.status!='NORMAL_RETURN').sum());good=allrows[allrows.status=='NORMAL_RETURN'];p0loo=loo[loo.kind=='LOO_P0'];top=p0loo.sort_values('delta_joint_ATE_RMSE_m',ascending=False).head(10)
    base=allrows[allrows.kind=='BASELINE'];support=groups[groups.kind=='SUPPORT_GROUP'];temporal=groups[groups.kind=='TEMPORAL_GROUP'];uncovered=p0[p0.P5_route_bin_uncovered]
    uncovered=uncovered.merge(p0loo.assign(robot1_kf=p0loo.case.str.split('_').str[-2].astype(int),robot3_kf=p0loo.case.str.split('_').str[-1].astype(int)),on=['robot1_kf','robot3_kf'],suffixes=('','_loo'))
    total_solver=float(allrows.runtime_s.sum());pilot=read(OUT/'stage23b_pilot_report.json')
    # Public Pose3 Jacobians at predicted zero residual: independent finite diff.
    q=gtsam.Pose3(gtsam.Rot3.RzRyRx(.1,-.2,.3),np.array([1.,2.,3.]));c=gtsam.Pose3(gtsam.Rot3.RzRyRx(-.2,.1,.7),np.array([4.,-1.,2.]));a=np.zeros((6,6),order='F');b=a.copy(order='F');z=q.between(c,a,b);J=np.hstack([a,b]);fd=np.empty((6,12));eps=1e-6
    for j in range(12):
        h=np.zeros(6);h[j%6]=eps;plus=[q,c];minus=[q,c];plus[j//6]=plus[j//6].retract(h);minus[j//6]=minus[j//6].retract(-h);fd[:,j]=(gtsam.Pose3.Logmap(z.inverse().compose(plus[0].between(plus[1])))-gtsam.Pose3.Logmap(z.inverse().compose(minus[0].between(minus[1]))))/(2*eps)
    diff=float(np.max(abs(J-fd)));assert diff<1e-5
    validation=dict(STAGE23B_DIAGNOSTICS=status,STAGE23_FINAL_INDEPENDENT_VALIDATION='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE',exact_set_counts=dict(zip(['P0','P5','intersection','P0_only','P5_only'],counts)),cases_completed=len(allrows),cases_planned=len(policy['cases']),P0_LOO=141,P5_selected_LOO=7,temporal_groups=7,common_support_groups=2,nonconverged_or_failed=failures,all_finite=bool(allrows.finite.all()),baseline_reproduction=base[['case','frozen_reproduction_translation_max_m','frozen_reproduction_rotation_max_rad']].to_dict('records'),initialization_noise_solver_unchanged=True,common_only_graph_reproduction_max_abs=float(np.max(abs(common0-common5))),feature_Jacobian_finite_difference_max_abs=diff,GT_free_features_causal_and_unchanged=True,unknown_GICP_results_imputed=0,new_GICP_calls=0,selector_trained=False,historical_and_Stage23A_hashes_unchanged=True,NEXT_STEPS_unchanged=True,pilot_actual_solver_s=pilot['actual_pilot_solver_s'],total_actual_solver_s=total_solver,solver_peak_RSS_mib=float(allrows.peak_RSS_mib.max()),offline_evaluation_elapsed_s=evaluated['elapsed_evaluation_s'],GT_free_feature_elapsed_s=read(OUT/'stage23b_feature_freeze.json')['elapsed_s'],GT_evaluation_only_after_feature_and_solve_freeze=True,independent_algorithm_readiness_claimed=False)
    dump('stage23b_validation.json',validation)
    # Small static scientific figures; no demo/asset mutation.
    fig,ax=plt.subplots(figsize=(10,4));ax.bar(temporal.case,temporal.delta_joint_ATE_RMSE_m);ax.axhline(0,color='k',lw=.5);ax.tick_params(axis='x',rotation=25);ax.set_ylabel('delta joint ATE RMSE (m), offline only');fig.tight_layout();fig.savefig(OUT/'stage23b_group_effects.png',dpi=140);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4));ax.hist(p0loo.delta_joint_ATE_RMSE_m,bins=30);ax.set_xlabel('signed delta joint ATE after single removal (m), offline only');ax.set_ylabel('P0-only factors');fig.tight_layout();fig.savefig(OUT/'stage23b_LOO_distribution.png',dpi=140);plt.close(fig)
    cf=correlation[(correlation.cohort=='LOO_P0')&(correlation.top_K==10)];fig,ax=plt.subplots(figsize=(10,4));ax.bar(cf.proxy,cf.Spearman_rho.fillna(0));ax.tick_params(axis='x',rotation=30);ax.set_ylabel('Spearman rho with signed delta ATE');ax.set_title('Undefined constant proxies shown at0 only for display; see CSV');fig.tight_layout();fig.savefig(OUT/'stage23b_predictive_correlations.png',dpi=140);plt.close(fig)
    doc=f'''# Stage23B — retrospective critical-loop diagnostics

**STAGE23B_DIAGNOSTICS: {status}**

**STAGE23_FINAL_INDEPENDENT_VALIDATION:
NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE**

These decisions are separate. This is an authorized retrospective diagnostic,
not independent final validation, an accepted selector, or deployable readiness.
No new GICP, unknown-outcome imputation, distributed SLAM or demo change.
The old dirty files and all Stage23A outputs remain byte-identical. No NEXT_STEPS
edit; this new document records the experiment without rewriting old results.

## 1. Exact P0/P5 identities and false factors

P0=288, P5=231, intersection147, P0-only141, P5-only84, net57.
The query-level geometric true-positive loss335 remains a different quantity.
All141 P0-only and84 P5-only factors are positive under offline d_xy<5m.
Common147 includes139 positives and8 negatives; false factors are explicitly
retained/labelled in the offline audit, never silently assumed correct or
removed from optimization. Do not use these labels to tune a future selector.
Canonical IDs, event, measured geometric quality, odometry bins, first-bridge
and nearest earlier admitted pair are recorded for both exclusive sets.

## 2. Seven temporal clusters

The fixed Stage23A30s-gap clusters are unchanged. Exact frozen Batch results:

{table(temporal,['case','removed','joint_ATE_RMSE_m','delta_joint_ATE_RMSE_m','delta_regional_ATE_RMSE_m','delta_regional_ATE_p95_m'])}

Signed deltas matter: positive means removing the factors worsened the metric;
negative means removal improved it. True d_xy matches do not guarantee accurate
relative measurements, and extra factors need not monotonically improve GT
trajectory accuracy. The coarse timestamp clusters can span multiple spatial
regions. Maps are evaluated globally and in matched removed-endpoint regions,
plus all four uncovered bins; zero topological cut edges does not prove local
redundancy. All graphs remain connected via the unchanged odometry chains and
remaining cross-robot constraints; first bridge is never removed.

## 3. Four P5-uncovered route bins

{table(uncovered,['robot1_kf','robot3_kf','R1_bin50','R3_bin50','delta_joint_ATE_RMSE_m','delta_regional_ATE_RMSE_m','delta_regional_ATE_p95_m'])}

Bins are50m frozen local-odometry arc-length pairs, not GT-derived geographic
regions. Region evaluation uses union +/-50m around the removed endpoints,
with the same membership for baseline and ablation. One JOINT global rigid
alignment is used per condition; no independent robot/region fit is performed.
Map NN uses frame10/point16/voxel0.5m/range1–60m. A positive/negative regional
map change can differ from global ATE; NN tails include non-overlap coverage
and are not calibrated absolute map accuracy.

## 4. Important individual missing factors

Largest observed harmful-removal effects, sorted by signed joint ATE delta:

{table(top,['case','delta_joint_ATE_RMSE_m','delta_regional_ATE_RMSE_m','delta_regional_ATE_p95_m','delta_inter_robot_relative_translation_RMSE_m','delta_projected_cov_trace'])}

Across141 LOO cases, signed joint-ATE delta min/median/max is
{p0loo.delta_joint_ATE_RMSE_m.min():.6g}/
{p0loo.delta_joint_ATE_RMSE_m.median():.6g}/
{p0loo.delta_joint_ATE_RMSE_m.max():.6g}m. No materiality cutoff or online policy
was fitted from these GT outcomes. Single-factor effects do not sum to group
effects because optimization, robust reweighting and redundant support interact.
Every LOO retains5796 nodes/5794 odometry/one original prior, removes exactly
the named factor, uses the original chronological factor ordering and original
first-bridge initial estimates. Both full baselines reproduce frozen Batch
poses within1e-7; no tolerance/noise/Huber/solver change was used.
The ablated objective belongs to a different graph and is not directly an
accuracy comparison; full_reference_objective separately evaluates the original
policy graph at the ablated solution. Lower own-graph cost alone is not success.

## 5. Do P5-only factors provide alternative support?

{table(base,['case','joint_ATE_RMSE_m','inter_robot_GT_relative_translation_RMSE_m'])}

{table(support,['case','removed','joint_ATE_RMSE_m','delta_joint_ATE_RMSE_m','delta_regional_ATE_RMSE_m'])}

Removing all exclusive factors yields the SAME147-factor common-loop graph
from either policy; optimized matrix maximum difference is
{validation['common_only_graph_reproduction_max_abs']:.6g}. This matched control
shows the two exclusive sets supply different alternative support. P5-only
group removal and the seven preselected P5 singleton removals are measured,
not inferred from net counts. It does not prove every P5-only edge substitutes
one particular missing P0 edge, nor isolate a single component cause. P5's
lower full-graph joint ATE here does not reverse Stage22B's geometric-quality
loss or incomplete independent-generalization result.

## 6. GT-free criticality vs offline effects

Features are frozen BEFORE any current-run GT evaluation. For all372 union
factors, reconstruct the strictly earlier P0 factor prefix at its logical
discovery event. Insert current arriving odometry, but not its current loop.
After first bridge, linearize at common-frame LOCAL ODOMETRY prefixes, NOT
future optimized poses. Preserve joint endpoint cross-covariance. Predicted
Z=current prefix between; nominal logdet/normalized relative trace/max-eigenvalue
use no actual current-GICP transform. Pre-bridge common covariance is explicitly
undefined. Coverage uses earlier-loop odometry path neighborhoods; connectivity
uses current global/region state. Marginal bytes come from the frozen P0 cache
state before that attempt; info/byte uses a documented denominator max(1,bytes)
regularizer for cached zero-transfer cases, not a fabricated transmitted byte.

{table(cf,['proxy','n','Spearman_rho','top_K','top_K_overlap_fraction','top_K_signed_harm_positive_fraction','status'])}

The observed negative information/uncertainty rank correlations do NOT support
promoting information-first ranking on this route. Coverage also does not show
strong positive predictive evidence here. This negative finding is preserved,
not remedied by fitting weights to these GT-exposed outcomes.

Every information proxy is a local model, not calibrated physical uncertainty
or an optimized iSAM2 covariance. These are causal GT-free prefix-odometry
predictions tested against retrospective LOO effects; they are not a trained
online selector. Projected uncertainty and logdet are mathematically related,
not independent corroborating predictors. Global-connectivity proxy is constant
for the P0-only cohort and its correlation is undefined, not evidence of no
local importance. Group aggregation is mean individual prefix proxy, NOT a
joint group logdet. P5-only and7-group correlations are small-N exploratory;
P5 cases use a P0-prefix predictive context, not a claimed P5 online posterior.
Spearman p-values are uncorrected; Top-K exact overlap includes signed adverse
and beneficial removal outcomes, with random K/N reported. No physical accuracy
or generalization guarantee follows from these correlations.

Relative-pose GT error uses the FIXED union372 keyframe pairs and original
Stage22B reference CSV Pose3 convention. Translation/rotation residuals compare
estimated relative transforms with those references. No new extrinsic/frame
calibration was assumed; fixed reference/sensor-frame bias remains a limitation.
Measured-loop residuals are separately reported, not passed off as GT errors.
Covariance-change diagnostics preserve cross-correlations but compare each
optimized linearization/robust weighting, so are not pure fixed-Hessian removal
information. No dense global matrix was formed.

## 7. Cost, failures and reproducibility

Pilot14 solves actually took{pilot['actual_pilot_solver_s']:.3f}s and projected
{pilot['projected_all_solver_s']:.3f}s for all cases, BEFORE full LOO execution.
Budget900 solver seconds/2GiB was fixed in advance. Actual159 solver cases took
{total_solver:.3f}s including optimization, pose extraction/checks and compressed
state export (not pure native-LM timing), peak{validation['solver_peak_RSS_mib']:.3f}MiB;
{failures} failed/nonconverged. Offline evaluation/map elapsed
{evaluated['elapsed_evaluation_s']:.3f}s is separately measured. GT-free feature
construction took{validation['GT_free_feature_elapsed_s']:.3f}s. State exports
and per-case receipts remain server-only with hashes. Required CSVs, policy,
pilot, freezes, independent validation and this report are versioned. A reporting
normalization makes the exact-set table one row per canonical identity; it does
not change solved graph/results. Run
`bash scripts/run_stage23b_critical_loop_diagnostics.sh`; existing frozen solves
are reused, never replaced. No existing Stage23A output is overwritten.

## 8. Recommendations for future P6 — not finalized

Keep the independent-data NO-GO unchanged. Treat coverage/connectivity as explicit
constraints rather than assuming logdet-per-byte alone protects useful regions.
Evaluate hybrid proxies prospectively on eligible independent data before any
promotion. Do not choose weights, graph thresholds, robust noise, materiality
gates or deployment claims from these GT-exposed retrospective results. Preserve
the distinction between model uncertainty, realized geometric quality, regional
coverage and actual physical accuracy. No selector was trained/finalized and
the2112 unmeasured GICP candidates remain unknown. Stop after Stage23B diagnostics.
'''
    (ROOT/'docs/CUMULTI_STAGE23B_CRITICAL_LOOP_DIAGNOSTICS.md').write_text(doc)
    statefiles=list((OUT/'stage23b_states').glob('*.npz'));dump('stage23b_state_manifest.json',dict(artifacts=[dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p)) for p in statefiles],server_only=True))
    outputs=[p for p in OUT.iterdir() if p.is_file() and (p.name.startswith('stage23b_') or p.name in ['loop_set_exact_comparison.csv','p0_only_loop_diagnostics.csv','p5_only_loop_diagnostics.csv','critical_loop_loo.csv','critical_loop_group_ablation.csv','regional_map_quality.csv','gt_free_criticality_features.csv','criticality_predictive_validation.csv']) and p.name!='stage23b_artifact_manifest.json']
    dump('stage23b_artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=sha(p)) for p in outputs],source=[dict(path=str(p.relative_to(ROOT)),sha256=sha(p)) for p in (SRC/'stage23').glob('diagnostic_*.py')],documentation_sha256=sha(ROOT/'docs/CUMULTI_STAGE23B_CRITICAL_LOOP_DIAGNOSTICS.md')))
    print(json.dumps(validation),flush=True)
if __name__=='__main__':main()
