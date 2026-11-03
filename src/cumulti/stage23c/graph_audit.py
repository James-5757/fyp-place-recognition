"""Frozen receipts, prefix residuals and retrospective sparse information audits."""
from common import *

def prefix_quality():
    m.guard();loops,frame,nodes,local,arc,events,params=m.load();index={n:{m.identity(r):r for r in f.itertuples()} for n,f in loops.items()};union={**index[P5],**index[P0]};targets={}
    for qc,r in union.items():targets.setdefault(int(r.discovery_event),[]).append(qc)
    features=pd.read_csv(OLD/'gt_free_criticality_features.csv');feature={(int(r.robot1_kf),int(r.robot3_kf)):r for r in features.itertuples()};norm=read(B/'stage22b_experiment_policy.json')['normalization'];S=None;past=[];rows=[];p0events={int(r.discovery_event):r for r in loops[P0].itertuples()}
    for e in events:
        i=e['event_index']
        for qc in targets.get(i,[]):
            q,c=qc;r=union[qc];p=A/'attempts'/f'e{int(r.event_index):04d}_r{int(r.candidate_rank)}.json';receipt=read(p)['measurement'];assert sha(p.with_suffix('.csv'))==receipt['result_csv_sha256'];Z=np.asarray(json.loads(r.canonical_transform));np.testing.assert_allclose(Z,receipt['canonical_transform'],atol=0,rtol=0);assert abs(float(r.gicp_quality)-receipt['gicp_quality'])<1e-12
            f=feature[qc];row=dict(robot1_kf=q,robot3_kf=c,cohort='COMMON' if qc in index[P0] and qc in index[P5] else 'P0_ONLY' if qc in index[P0] else 'P5_ONLY',discovery_event=i,query_robot=r.query_robot,candidate_robot=r.candidate_robot,SC_score=float(r.SC_score),SC_rank=int(r.candidate_rank),SC_confidence_frozen=float(np.clip((r.SC_score-norm['SC_q05'])/(norm['SC_q95']-norm['SC_q05']),0,1)),GICP_fitness=float(r.gicp_fitness),GICP_RMSE=float(r.gicp_RMSE),GICP_quality=float(r.gicp_quality),canonical_transform=json.dumps(Z.tolist()),estimated_transfer_bytes=int(f.estimated_marginal_bytes),coverage_gain=float(f.coverage_gain),prior_loop_count=len(past),max_prior_factor_event=max(past,default=-1),pre_insertion_normalized_residual=None,pre_insertion_huber_if_added=None,pre_insertion_translation_residual_m=None,prefix_is_optimized=False,optimized_causal_state='OPTIMIZED_CAUSAL_STATE_NOT_AVAILABLE',current_measurement_features_availability='POST_VERIFICATION_NOT_PRE_VERIFICATION',receipt_verified=True)
            if S is not None:
                Tq=local['robot1',q];Tc=S@local['robot3',c];E=m.inv(Z)@m.inv(Tq)@Tc;err=gtsam.Pose3.Logmap(gtsam.Pose3(E));z=float(np.linalg.norm(err/SIG));row.update(pre_insertion_normalized_residual=z,pre_insertion_huber_if_added=min(1.,1.345/z) if z else 1.,pre_insertion_translation_residual_m=float(np.linalg.norm(E[:3,3])))
            assert row['max_prior_factor_event']<i;rows.append(row)
        if i in p0events:
            r=p0events[i]
            if S is None:
                q,c=m.identity(r);S=local['robot1',q]@np.asarray(json.loads(r.canonical_transform))@m.inv(local['robot3',c])
            past.append(i)
    assert len(rows)==372;csv('gicp_measurement_quality_audit.csv',rows);csv('causal_prefix_measurement_residuals.csv',rows)
    inventory=[dict(feature=n,availability=category,GT_used=category=='OFFLINE_GT_EVALUATION',caveat=caveat) for names,category,caveat in [(['SC_score','SC_rank','SC_confidence_frozen','estimated_transfer_bytes','coverage_gain','original_prefix_nominal_information','original_prefix_uncertainty'],'PRE_VERIFICATION','frozen earlier reference prefix/cache; common-odometry proxy, not an optimized posterior'),(['GICP_fitness','GICP_RMSE','GICP_quality','canonical_transform','pre_insertion_residual','normalized_residual','pre_insertion_huber_if_added'],'POST_VERIFICATION','pre-insertion means AFTER current registration and BEFORE factor insertion; never a pre-exchange selector input'),(['full_batch_robust_weight','fixed_full_graph_information','nonlinear_LOO_covariance_change','exact_group_information'],'RETROSPECTIVE_GRAPH_ANALYSIS','complete frozen graph/states, never substitutes for causal prefix'),(['signed_delta_ATE','regional_GT_error','offline_false_label'],'OFFLINE_GT_EVALUATION','not a selector feature or independently validated training target')] for n in names]
    csv('pre_post_verification_feature_inventory.csv',inventory)
    dump('causal_state_availability.json',dict(status='OPTIMIZED_CAUSAL_STATE_NOT_AVAILABLE',P0_P5_available_states='final full trajectories only; Stage23B ablation states all final full5796-node states',Stage20_snapshots='actual earlier states exist but belong to archived75 graph, NOT P0/P5; rejected as replacements',final_states_used_as_causal=False,prefix_model='unchanged Stage23B common-odometry prefix; earlier first-bridge initialization only',quality_rows=len(rows),prefix_quality_file='causal_prefix_measurement_residuals.csv',prefix_quality_sha256=sha(OUT/'causal_prefix_measurement_residuals.csv')))

def fixed_audit():
    m.guard();start=time.monotonic();loops,frame,nodes,local,arc,events,params=m.load();full,_,idx=m.build(P0,loops,nodes,local,events);mat=np.load(OLD/'stage23b_states/BASE_P0.npz')['matrices'];v0=m.to_values(mat,nodes);marg0=gtsam.Marginals(full,v0);rows=[];features=pd.read_csv(OLD/'gt_free_criticality_features.csv').set_index(['robot1_kf','robot3_kf']);pol=read(OLD/'stage23b_policy.json');cases={c['name']:c for c in pol['cases']};p0={m.identity(r):r for r in loops[P0].itertuples()}
    weights0={qc:residual(v0,*qc,np.asarray(json.loads(r.canonical_transform)))[2] for qc,r in p0.items()}
    quality=pd.read_csv(OUT/'gicp_measurement_quality_audit.csv')
    for i,r in quality.iterrows():
        q,c=int(r.robot1_kf),int(r.robot3_kf);Z=np.asarray(json.loads(r.canonical_transform));err,n,w=residual(v0,q,c,Z);present=(q,c) in p0
        quality.loc[i,'full_P0_factor_present']=present;quality.loc[i,'full_P0_normalized_residual']=n;quality.loc[i,'full_P0_diagnostic_huber_weight']=w;quality.loc[i,'actual_full_P0_in_graph_reconstructed_weight']=w if present else np.nan
    csv('gicp_measurement_quality_audit.csv',quality)
    for name in pol['pilot_names']:
        budget(start);case=cases[name];q,c=case['remove'][0];ks=[m.key(('robot1',q)),m.key(('robot3',c))];without=graph_without(full,idx,[(q,c)]);mg0=gtsam.Marginals(without,v0);C0=Cpair(marg0,ks);Cm=Cpair(mg0,ks);W0=Jbetween(v0,ks);A0=gaussian_A(full.at(idx[q,c]),v0,ks)
        after=np.load(OLD/'stage23b_states'/(name+'.npz'))['matrices'];v1=m.to_values(after,nodes);mg1=gtsam.Marginals(without,v1);C1=Cpair(mg1,ks);W1=Jbetween(v1,ks)
        tr=lambda W,C:float(np.trace(W@C@W.T));before=tr(W0,C0);fixed=tr(W0,Cm);changed=tr(W0,C1);actual=tr(W1,C1);pure=fixed-before;state=changed-fixed;projection=actual-changed;assert pure>-1e-5;assert abs((actual-before)-(pure+state+projection))<1e-5
        Z=np.asarray(json.loads(p0[q,c].canonical_transform));e0,z0,w0=residual(v0,q,c,Z);e1,z1,w1=residual(v1,q,c,Z)
        R=W0@C0@W0.T;ev,vec=np.linalg.eigh((R+R.T)/2);normalized=e0/SIG;align=float(abs(vec[:,-1]@normalized)/np.linalg.norm(normalized)) if np.linalg.norm(normalized)>0 else None
        weight_changes=[abs(residual(v1,a,b,np.asarray(json.loads(r.canonical_transform)))[2]-weights0[a,b]) for (a,b),r in p0.items() if (a,b)!=(q,c)]
        rows.append(dict(case=name,robot1_kf=q,robot3_kf=c,original_prefix_nominal_info=float(features.loc[q,c].nominal_info_nats),fixed_full_P0_robust_Gaussian_gain_nats=gain(A0,Cm),full_projected_cov_trace=before,removed_same_linearization_cov_trace=fixed,removed_relinearized_cov_trace=actual,fixed_factor_removal_cov_component=pure,state_and_robust_reweight_cov_component=state,projection_J_cov_component=projection,total_cov_trace_change=actual-before,decomposition_closure_error=(actual-before)-(pure+state+projection),weak_relative_eigenvalue=float(ev[-1]),actual_residual_alignment_to_weak_direction=align,actual_removed_factor_weight_before=w0,actual_removed_factor_weight_after=None,removed_factor_counterfactual_weight_at_LOO_state=w1,retained_factor_mean_abs_weight_change=float(np.mean(weight_changes)),retained_factor_max_abs_weight_change=float(np.max(weight_changes)),actual_causal_optimized_state='OPTIMIZED_CAUSAL_STATE_NOT_AVAILABLE',full_states_scope='RETROSPECTIVE_ONLY_NEVER_PRE_GICP',component_caveat='state+robust effect jointly observed, NOT a controlled unique robust-weight causal attribution'))
    csv('information_linearization_audit.csv',rows)
    groupcases=[c for c in pol['cases'] if c['kind']=='TEMPORAL_GROUP'];groupby={c['name']:c for c in groupcases};plan=read(OUT/'stage23c_policy.json');order=plan['pilot_groups']+[c['name'] for c in groupcases if c['name'] not in plan['pilot_groups']];out=[];pilot=[];partial=False
    try:
        for name in order:
            budget(start);t=time.monotonic();case=groupby[name];pairs=[tuple(p) for p in case['remove']];keys=sorted({m.key((r,k)) for q,c in pairs for r,k in [('robot1',q),('robot3',c)]});without=graph_without(full,idx,pairs);mg=gtsam.Marginals(without,v0);C=Cpair(mg,keys);blocks=[gaussian_A(full.at(idx[p]),v0,keys) for p in pairs];joint_gain=gain(np.vstack(blocks),C);individual=sum(gain(block,C) for block in blocks);assert individual>=joint_gain-1e-5
            row=dict(case=name,factors=len(pairs),joint_endpoint_dimensions=len(keys)*6,measurement_dimensions=len(pairs)*6,exact_fixed_Gaussian_group_information_nats=joint_gain,sum_individual_addition_information_same_base_nats=individual,information_redundancy_nats=individual-joint_gain,mean_original_prefix_nominal_information=float(np.mean([features.loc[p].nominal_info_nats for p in pairs])),runtime_s=time.monotonic()-t,linearization='frozen full P0 Batch, RETROSPECTIVE_ONLY',Gaussian_weights='actual frozen robust factor linearizations at the SAME values; no across-solution monotonicity claim',global_dense_information_matrix=False);out.append(row);print('EXACT_GROUP_INFORMATION',name,flush=True)
            if name in plan['pilot_groups']:pilot.append(row)
            if len(pilot)==2 and not (OUT/'group_information_pilot.json').exists():
                a,b=pilot;slope=max(0,(b['runtime_s']-a['runtime_s'])/(b['factors']**3-a['factors']**3));constant=max(0,a['runtime_s']-slope*a['factors']**3);projection=sum(constant+slope*len(c['remove'])**3 for c in groupcases);rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
                dump('group_information_pilot.json',dict(cases=pilot,projection_model='nonnegative constant+cubic fitted to TIMING ONLY, not GT outcomes',modelled_all_group_seconds=projection,actual_pilot_seconds=sum(r['runtime_s'] for r in pilot),peak_RSS_mib=rss,frozen_cap_s=900,feasible=projection<900 and rss<2048))
                if projection>=900 or rss>=2048:partial=True;break
    except TimeoutError:partial=True
    csv('group_information_interaction.csv',out);dump('stage23c_graph_computation_audit.json',dict(status='PARTIAL' if partial else 'COMPLETE',representatives=len(rows),exact_groups=len(out),new_nonlinear_optimization_calls=0,new_GICP_calls=0,elapsed_s=time.monotonic()-start,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,dense_global_matrix=False,GT_used=False))
if __name__=='__main__':
    verify()
    if (OUT/'stage23c_graph_computation_audit.json').exists() and not (OUT/'prefix_provenance_engineering_recovery.json').exists():
        dump('prefix_provenance_engineering_recovery.json',dict(reason='separate immutable causal-prefix artifact from subsequently augmented retrospective quality table; no math/input/threshold change',previous_calculation=read(OUT/'stage23c_graph_computation_audit.json'),new_nonlinear_or_GICP_calls=0))
    prefix_quality();fixed_audit()
