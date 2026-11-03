"""Retrospective/exploratory associations; no fitting, new solver or GICP."""
from common import *
from scipy.stats import spearmanr

def bootstrap(x,y,clusters,kind,seed,nboot):
    rng=np.random.default_rng(seed);out=[];unique=np.unique(clusters)
    for _ in range(nboot):
        if kind=='CLUSTER':
            chosen=rng.choice(unique,len(unique),replace=True);ix=np.concatenate([np.flatnonzero(clusters==c) for c in chosen])
        else:ix=rng.integers(0,len(x),len(x))
        xx,yy=x[ix],y[ix]
        if len(xx)>=3 and np.ptp(xx)>0 and np.ptp(yy)>0:out.append(float(spearmanr(xx,yy).statistic))
    return (float(np.percentile(out,2.5)),float(np.percentile(out,97.5)),len(out)) if out else (None,None,0)

def main():
    verify();assert read(OUT/'synthetic_information_sanity.json')['status']=='PASS';start=time.monotonic()
    feat=pd.read_csv(OLD/'gt_free_criticality_features.csv');quality=pd.read_csv(OUT/'gicp_measurement_quality_audit.csv');labels=pd.read_csv(OLD/'loop_set_exact_comparison.csv');loo=pd.read_csv(OLD/'critical_loop_loo.csv');groups=pd.read_csv(OLD/'critical_loop_group_ablation.csv');policy=read(OLD/'stage23b_policy.json');plans={c['name']:c for c in policy['cases']}
    frame=feat.merge(quality,on=['robot1_kf','robot3_kf'],suffixes=('','_quality'),validate='one_to_one').merge(labels[['robot1_kf','robot3_kf','membership','offline_correct_d_xy_lt5']],on=['robot1_kf','robot3_kf'],validate='one_to_one')
    effects=[]
    for r in loo.to_dict('records'):
        q,c=plans[r['case']]['remove'][0];effects.append(dict(robot1_kf=q,robot3_kf=c,LOO_case=r['case'],**{k:v for k,v in r.items() if k.startswith('delta_')}))
    frame=frame.merge(pd.DataFrame(effects),on=['robot1_kf','robot3_kf'],how='left',validate='one_to_one');frame['direct_measured_single_LOO']=frame.LOO_case.notna();frame['interpretation']='RETROSPECTIVE_EXPLORATORY'
    clusters={tuple(qc):c['cluster'] for c in policy['cases'] if c['kind']=='TEMPORAL_GROUP' for qc in c['remove']};frame['temporal_cluster']=[clusters.get((int(r.robot1_kf),int(r.robot3_kf)),np.nan) for r in frame.itertuples()]
    assert len(frame)==372 and frame.membership.value_counts().to_dict()=={'COMMON':147,'P0_ONLY':141,'P5_ONLY':84}
    assert frame.groupby('membership').direct_measured_single_LOO.sum().to_dict()=={'COMMON':0,'P0_ONLY':141,'P5_ONLY':7}
    csv('information_vs_mapping_impact.csv',frame);csv('cohort_outcome_availability.csv',frame[['robot1_kf','robot3_kf','membership','direct_measured_single_LOO','LOO_case']])
    csv('eight_false_factor_audit.csv',frame[~frame.offline_correct_d_xy_lt5]);assert (~frame.offline_correct_d_xy_lt5).sum()==8
    configuration=read(OUT/'stage23c_policy.json');correlations=[]
    comparisons=[('nominal_info_nats','delta_joint_ATE_RMSE_m','PRE_VERIFICATION'),('relative_uncertainty_trace','delta_joint_ATE_RMSE_m','PRE_VERIFICATION'),('SC_confidence_frozen','delta_joint_ATE_RMSE_m','PRE_VERIFICATION'),('GICP_quality','delta_joint_ATE_RMSE_m','POST_VERIFICATION'),('pre_insertion_normalized_residual','delta_joint_ATE_RMSE_m','POST_VERIFICATION'),('pre_insertion_huber_if_added','delta_joint_ATE_RMSE_m','POST_VERIFICATION'),('coverage_gain','delta_regional_ATE_RMSE_m','PRE_VERIFICATION'),('coverage_gain','delta_regional_ATE_p95_m','PRE_VERIFICATION'),('nominal_info_nats','delta_projected_cov_trace','PRE_VERIFICATION'),('full_P0_diagnostic_huber_weight','delta_joint_ATE_RMSE_m','RETROSPECTIVE_GRAPH_ANALYSIS')]
    for cohort in ['P0_ONLY','P5_ONLY']:
        for predictor,target,availability in comparisons:
            sub=frame[(frame.membership==cohort)&frame.direct_measured_single_LOO].dropna(subset=[predictor,target]);x=sub[predictor].to_numpy(float);y=sub[target].to_numpy(float);n=len(x);valid=n>=3 and np.ptp(x)>0 and np.ptp(y)>0
            rho,pvalue=spearmanr(x,y) if valid else (None,None)
            for kind in (['CLUSTER','IID_SENSITIVITY'] if n>=20 else ['NO_CI_SMALL_N']):
                lo=hi=None;rep=0
                if valid and n>=20:lo,hi,rep=bootstrap(x,y,sub.temporal_cluster.fillna(-1).to_numpy(int),kind,configuration['bootstrap_seed'],configuration['bootstrap_replicates'])
                for K in ([10,20] if n>=20 else [3]):
                    k=min(n,K);pred=np.argsort(-x,kind='stable')[:k];actual=np.argsort(-y,kind='stable')[:k]
                    correlations.append(dict(cohort=cohort,predictor=predictor,target=target,availability=availability,n=n,Spearman_rho=None if rho is None else float(rho),uncorrected_p=None if pvalue is None else float(pvalue),CI_method=kind,CI_95_low=lo,CI_95_high=hi,bootstrap_valid_replicates=rep,top_K=k,top_K_exact_overlap=float(len(set(pred)&set(actual))/k) if valid and k else None,random_overlap_reference=k/n if n else None,status='UNDEFINED_CONSTANT' if not valid else 'EXPLORATORY_SMALL_N' if n<20 else 'RETROSPECTIVE_EXPLORATORY',CI_caveat='7 exposed-route clusters are NOT independent recording replication; IID ignores dependence; no feature/model selection'))
            budget(start)
    csv('spearman_bootstrap_correlations.csv',correlations)
    # Fixed Top10 rank-disagreement cohort, not a new feature cutoff/selector.
    sub=frame[frame.membership=='P0_ONLY'].copy();sub['nominal_info_rank']=sub.nominal_info_nats.rank(ascending=False,method='first');sub['measured_harm_rank']=sub.delta_joint_ATE_RMSE_m.rank(ascending=False,method='first')
    csv('rank_disagreement_cases.csv',sub[(sub.nominal_info_rank<=10)|(sub.measured_harm_rank<=10)])
    exact=pd.read_csv(OUT/'group_information_interaction.csv');interactions=[]
    for row in exact.to_dict('records'):
        plan=plans[row['case']];member=set(map(tuple,plan['remove']));singles=frame[[ (int(r.robot1_kf),int(r.robot3_kf)) in member for r in frame.itertuples() ]];assert len(singles)==len(member) and singles.direct_measured_single_LOO.all()
        observed=groups[groups.case==row['case']].iloc[0];individual_sum=float(singles.delta_joint_ATE_RMSE_m.sum());row.update(measured_group_delta_joint_ATE_m=float(observed.delta_joint_ATE_RMSE_m),sum_measured_single_LOO_delta_joint_ATE_m=individual_sum,nonadditive_joint_ATE_interaction_m=float(observed.delta_joint_ATE_RMSE_m)-individual_sum,max_abs_single_LOO_delta_joint_ATE_m=float(singles.delta_joint_ATE_RMSE_m.abs().max()),group_delta_regional_ATE_m=float(observed.delta_regional_ATE_RMSE_m),sum_individual_regional_ATE='NOT_COMPARABLE_DIFFERENT_REGIONAL_MASKS',sum_individual_map_effects='NOT_AVAILABLE_NO_SINGLE_LOO_MAPS',interpretation='nonadditivity combines redundancy, nonlinear solution and robust effects; not a unique causal decomposition')
        mapdf=pd.read_csv(OLD/'regional_map_quality.csv');mp=mapdf[(mapdf.case==row['case'])&(mapdf.region=='REMOVED_ENDPOINT_NEIGHBORHOODS')]
        if len(mp):row.update(measured_group_delta_regional_map_median_m=float(mp.iloc[0].delta_median_m),measured_group_delta_regional_map_p95_m=float(mp.iloc[0].delta_p95_m))
        interactions.append(row)
    csv('group_information_interaction.csv',interactions)
    # Inspect false measurements descriptively. No threshold is proposed/fitted.
    summaries=[]
    for label,group in frame.groupby('offline_correct_d_xy_lt5'):
        for field in ['GICP_quality','GICP_fitness','GICP_RMSE','pre_insertion_normalized_residual','full_P0_diagnostic_huber_weight']:
            a=group[field].dropna();summaries.append(dict(offline_GT_positive=bool(label),field=field,n=len(a),median=float(a.median()) if len(a) else None,p95=float(a.quantile(.95)) if len(a) else None,threshold_fitted=False))
    csv('false_true_measurement_descriptives.csv',summaries)
    linear=pd.read_csv(OUT/'information_linearization_audit.csv');primary=next(r for r in correlations if r['cohort']=='P0_ONLY' and r['predictor']=='nominal_info_nats' and r['target']=='delta_joint_ATE_RMSE_m' and r['CI_method']=='CLUSTER' and r['top_K']==10)
    observations=['GROUP_INTERACTION_IMPORTANT'] if interactions and max(abs(r['nonadditive_joint_ATE_interaction_m']) for r in interactions)>1e-8 else []
    summary=dict(primary_conclusion='NOMINAL_INFORMATION_PROXY_NOT_SUPPORTED' if primary['Spearman_rho'] is None or primary['Spearman_rho']<=0 else 'INCONCLUSIVE',additional_observations=observations,scope='RETROSPECTIVE_EXPLORATORY only; no optimized feature combination, cutoff, weights or model fitted',original_nominal_correlation=primary,cohorts=dict(P0_only=141,P5_only=84,common=147,total=372),direct_LOO_outcomes=dict(P0_only=141,P5_only=7,P5_only_missing=77,common_missing=147),existing_temporal_groups=len(interactions),false_factor_count=8,unknown_GICP_candidates_unchanged=2112,actual_optimized_causal_state='OPTIMIZED_CAUSAL_STATE_NOT_AVAILABLE',new_GICP_calls=0,new_nonlinear_solver_calls=0,regression_or_selector_fitted=False,final_independent_validation='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE',fixed_removal_cov_components_nonnegative=bool((linear.fixed_factor_removal_cov_component>=-1e-5).all()),max_cov_decomposition_closure_error=float(abs(linear.decomposition_closure_error).max()),robust_state_attribution='combined observational decomposition only; no unique percent contribution or controlled robust-weight intervention',maximum_group_nonadditivity_m=max(abs(r['nonadditive_joint_ATE_interaction_m']) for r in interactions) if interactions else None,analysis_elapsed_s=time.monotonic()-start,recommended_next_P6='prospective communication-aware P5 improvements, cache reuse, explicit coverage constraints and budget scheduling; do not promote information-first ranking from these GT-exposed data')
    dump('criticality_mechanism_summary.json',summary);print(json.dumps(summary),flush=True)
if __name__=='__main__':main()
