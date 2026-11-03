"""Offline Stage23E quality. ALL four graph artifacts must freeze before GT opens."""
from stage23e_common import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LABEL='RETROSPECTIVE_GT_EXPOSED_RECORDING'
def boolean(v):return str(v).lower()=='true'
def split(i):return 'dev' if i<1000 else 'embargo' if i<1100 else 'heldout'
def rmse(v):return float(np.sqrt(np.mean(np.square(v)))) if len(v) else None
def scope_ids(scope):return range(5796) if scope=='full' else range(1000) if scope=='dev' else range(1000,1100) if scope=='embargo' else range(1100,5796)
def verify_graph_freeze():
    lock_verify();frozen=read(G/'all_graphs_pre_gt_freeze.json')
    assert frozen['status']=='ALL_FOUR_BACKEND_RESULTS_FROZEN' and not frozen['GT_read']
    for name,v in frozen['files'].items():assert sha(name)==v['sha256'],name
    return frozen
def main():
    frozen=verify_graph_freeze()
    if (E/'stage23e_decision.json').exists():print('EXISTING_STAGE23E_EVALUATION_RETAINED',flush=True);return
    # Verify frozen acceptance gates and region definitions by their pre-existing
    # Stage23D manifest chains, never by refitting from current GT outcomes.
    historical=json.loads(gzip.decompress((MAP/'historical_integrity_inventory.json.gz').read_bytes()))
    gates_path=B/'stage22b_predeclared_acceptance.json';assert sha(gates_path)==historical[str(gates_path)]['sha256'];limits=read(gates_path)
    asset_manifest=MAP/'demo_asset_manifest.json';old=read(D/'stage23d_artifact_manifest.json');assert sha(asset_manifest)==old[str(asset_manifest.relative_to(ROOT))]['sha256'];assets=read(asset_manifest);demo_data=ROOT/'demo/stage23d_policy_map_compare/data.json';assert sha(demo_data)==assets[str(demo_data.relative_to(ROOT))]['sha256']
    dump(E/'offline_evaluation_entry.json',dict(label=LABEL,all_four_graphs_frozen_before_GT=True,pre_gt_freeze_sha256=sha(G/'all_graphs_pre_gt_freeze.json'),frozen_unix_ns=frozen['frozen_unix_ns'],GT_entry_unix_ns=time.time_ns(),evaluator_sha256=sha(Path(__file__)),backend_source_sha256=sha(ROOT/'src/cumulti/stage23d/run_stage23e_backend.py'),engineering_repair_sha256=sha(G/'stage23e_engineering_repair.json') if (G/'stage23e_engineering_repair.json').exists() else None,gates_sha256=sha(gates_path),region_assets_sha256=sha(demo_data),no_GT_retuning=True))
    # First GT data read occurs below this persisted entry.
    gt={r:pd.read_csv(PROC/r/'keyframes.csv').set_index('keyframe_id',drop=False) for r in ['robot1','robot3']}
    reference=pd.read_csv(A/'offline_causal_query_evaluation.csv').set_index('event_index');original_positive=pd.read_csv(A/'offline_geometrically_accepted_including_duplicates_labels.csv').set_index('discovery_event')
    schedule=pd.read_csv(A/'dual_stream_event_schedule.csv');arrival={(r.robot_id,int(r.keyframe_id)):int(r.event_index) for r in schedule.itertuples()};region_states=read(demo_data)['states']['P0_incremental']['regions']
    traces=pd.read_csv(D/'p6_queue_event_trace.csv.gz');traces=traces[traces.action=='VERIFIED'];all_factors=pd.read_csv(D/'p6_verified_factors.csv');resources=pd.read_csv(D/'resource_comparison_summary.csv').set_index('policy')
    sys.path.insert(0,str(ROOT/'src/cumulti'));import run_stage10_offline_map_merge as s10
    query_rows=[];factor_rows=[];factor_labels=[];accuracy=[];regional=[];residual_rows=[];comparisons=[]
    def positive(q,c):
        a,b=gt['robot1'].loc[int(q)],gt['robot3'].loc[int(c)];distance=float(np.linalg.norm([a.x-b.x,a.y-b.y]));return distance<5,distance
    for short,name in POLICIES.items():
        directory=G/short;graph=read(directory/'graph_integrity.json');attempt=traces[traces.policy==name];accepted=attempt[attempt.geometrically_accepted.map(boolean)];factors=all_factors[all_factors.policy==name];assert not accepted.discovery_event.duplicated().any()
        accepted_ids=set(accepted.discovery_event.astype(int));admitted_ids=set(factors.discovery_event.astype(int));labels={}
        for r in accepted.itertuples():
            yes,dist=positive(r.robot1_kf,r.robot3_kf);event=int(r.discovery_event);labels[event]=yes;assert event in original_positive.index;assert yes==bool(original_positive.loc[event,'positive_d_xy_lt5']);assert int(original_positive.loc[event,'robot1_kf'])==int(r.robot1_kf) and int(original_positive.loc[event,'robot3_kf'])==int(r.robot3_kf)
        for r in factors.itertuples():
            yes,dist=positive(r.robot1_kf,r.robot3_kf);factor_labels.append(dict(policy=short,discovery_event=int(r.discovery_event),insertion_event=int(r.insertion_event),rank=int(r.rank),robot1_kf=int(r.robot1_kf),robot3_kf=int(r.robot3_kf),offline_d_xy_m=dist,offline_positive=yes,query_split=split(int(r.discovery_event)),insertion_split=split(int(r.insertion_event)),label=LABEL));assert int(r.discovery_event) in labels
        for scope in ['full','heldout','dev','embargo']:
            frame=reference.loc[list(scope_ids(scope))]
            for direction in ['ALL','robot1','robot3']:
                subset=frame if direction=='ALL' else frame[frame.query_robot==direction];indices=set(subset.index);valid=set(subset[subset.valid_overlap].index);geo=accepted_ids&indices;adm=admitted_ids&indices;tp=sum(labels[i] for i in geo);atp=sum(labels[i] for i in adm)
                query_rows.append(dict(policy=short,scope=scope,direction=direction,queries=len(subset),positive_queries=len(valid),geometric_accepted=len(geo),geometric_true=tp,false_geometric=len(geo)-tp,precision=tp/len(geo) if geo else None,recall=tp/len(valid) if valid else None,query_clock='discovery_event',label=LABEL))
                factor_rows.append(dict(policy=short,scope=scope,direction=direction,admitted=len(adm),admitted_true=atp,false_admitted=len(adm)-atp,admitted_precision=atp/len(adm) if adm else None,admission_recall=atp/len(valid) if valid else None,query_clock='discovery_event',backend_clock='insertion_event',label=LABEL))
        if graph['status']=='PASS':
            for condition in ['incremental','batch']:
                trajectory=pd.read_csv(directory/f'online_{condition}_trajectory.csv');ate_full,aligned,errors=s10.joint_eval(trajectory,gt['robot1'],gt['robot3']);lookup=s10.lookup(trajectory);node_keys=[(r.robot_id,int(r.keyframe_id)) for r in trajectory.itertuples()];rpe={}
                for robot in ['robot1','robot3']:
                    ids=trajectory[trajectory.robot_id==robot].keyframe_id.astype(int).tolist();samples=[]
                    for k in ids[:-10]:
                        a,b=gt[robot].loc[k],gt[robot].loc[k+10];G0=s10.T([a.qx,a.qy,a.qz,a.qw],[a.x,a.y,a.z]);G1=s10.T([b.qx,b.qy,b.qz,b.qw],[b.x,b.y,b.z]);err=inv(inv(G0)@G1)@inv(lookup[robot,k])@lookup[robot,k+10];samples.append((k,k+10,float(np.linalg.norm(err[:3,3])),float(np.rad2deg(Rotation.from_matrix(err[:3,:3]).magnitude()))))
                    rpe[robot]=samples
                def quality(mask,robot_filter=None):
                    selected={n for n,yes in zip(node_keys,mask) if yes};samples=[x for r,items in rpe.items() if robot_filter is None or robot_filter==r for x in items if (r,x[0]) in selected and (r,x[1]) in selected];er=errors[mask]
                    return dict(nodes=int(mask.sum()),ATE_RMSE_m=rmse(er),ATE_p95_m=float(np.percentile(er,95)) if len(er) else None,RPE_interval=10,RPE_samples=len(samples),RPE_translation_RMSE_m=rmse([x[2] for x in samples]),RPE_rotation_RMSE_deg=rmse([x[3] for x in samples]),alignment='ONE joint full-system rigid alignment; no per-robot/region/scope refit',label=LABEL)
                for scope in ['full','heldout','dev','embargo']:
                    events=set(scope_ids(scope));base=np.array([arrival[n] in events for n in node_keys])
                    for robot in ['joint','robot1','robot3']:
                        mask=base if robot=='joint' else base & np.array([n[0]==robot for n in node_keys]);row=dict(policy=short,condition=condition,scope=scope,robot_id=robot,**quality(mask,None if robot=='joint' else robot));accuracy.append(row)
                        if scope=='full':assert np.isclose(row['ATE_RMSE_m'],next(x['ATE_RMSE_m'] for x in ate_full if x['scope']==robot),atol=1e-12,rtol=0)
                for region,members in region_states.items():
                    nodes={(r,int(k)) for r,ids in members.items() for k in ids};base=np.array([n in nodes for n in node_keys])
                    for robot in ['joint','robot1','robot3']:
                        mask=base if robot=='joint' else base & np.array([n[0]==robot for n in node_keys]);regional.append(dict(policy=short,condition=condition,region=region,robot_id=robot,**quality(mask,None if robot=='joint' else robot)))
            residual=pd.read_csv(directory/'online_loop_residuals.csv')
            for condition,f in residual.groupby('condition'):
                residual_rows.append(dict(policy=short,condition=condition,translation_median_m=float(f.translation_m.median()),translation_p95_m=float(f.translation_m.quantile(.95)),rotation_median_deg=float(f.rotation_deg.median()),rotation_p95_deg=float(f.rotation_deg.quantile(.95)),minimum_diagnostic_huber_weight=float(f.diagnostic_reconstructed_huber_weight.min())))
        cost=resources.loc[name];q=next(r for r in query_rows if r['policy']==short and r['scope']=='full' and r['direction']=='ALL');f=next(r for r in factor_rows if r['policy']==short and r['scope']=='full' and r['direction']=='ALL');a=next((r for r in accuracy if r['policy']==short and r['condition']=='incremental' and r['scope']=='full' and r['robot_id']=='joint'),{});maps=pd.read_csv(directory/'online_map_consistency.csv') if (directory/'online_map_consistency.csv').exists() else pd.DataFrame();metric=maps[maps.condition=='incremental'].iloc[0] if len(maps) else {}
        comparisons.append(dict(policy=short,application_bytes=int(cost.application_bytes),GICP_receipt_attempts=int(cost.completed_receipt_decisions),admitted_factors=len(factors),precision=q['precision'],recall=q['recall'],admitted_precision=f['admitted_precision'],joint_ATE_RMSE_m=a.get('ATE_RMSE_m'),RPE_translation_RMSE_m=a.get('RPE_translation_RMSE_m'),RPE_rotation_RMSE_deg=a.get('RPE_rotation_RMSE_deg'),map_NN_median_m=metric.get('median_m'),map_NN_p95_m=metric.get('p95_m'),graph_status=graph['status'],label=LABEL))
        print('STAGE23E_OFFLINE_POLICY',short,flush=True)
    # Baselines are COPIED from frozen tables; no P0/P5 trajectory or GT evaluator.
    old_accuracy=pd.read_csv(B/'offline_map_accuracy_by_policy.csv');old_quality=pd.read_csv(B/'policy_comparison_full.csv');old_cost=pd.read_csv(B/'policy_results_by_budget.csv');old_maps=pd.read_csv(B/'gt_free_map_consistency_by_policy.csv')
    baseline=[]
    for name in [P0,P5]:
        c=old_cost[old_cost.policy==name].iloc[0];q=old_quality[(old_quality.policy==name)&(old_quality.direction=='ALL')].iloc[0];a=old_accuracy[(old_accuracy.policy==name)&(old_accuracy.condition=='incremental')];ate=a[(a.metric=='ATE_joint_rigid')&(a.scope=='joint')].iloc[0];rpe=a[(a.metric=='RPE_interval10')&(a.scope=='combined')].iloc[0];m=old_maps[old_maps.policy==name].iloc[0]
        baseline.append(dict(policy='P0' if name==P0 else 'P5',application_bytes=int(c.application_bytes),GICP_receipt_attempts=int(c.attempts),admitted_factors=int(c.admitted),precision=float(q.precision),recall=float(q.recall),admitted_precision=float(q.admitted_precision),joint_ATE_RMSE_m=float(ate.ATE_RMSE_m),RPE_translation_RMSE_m=float(rpe.translation_RMSE_m),RPE_rotation_RMSE_deg=float(rpe.rotation_RMSE_deg),map_NN_median_m=float(m.median_m),map_NN_p95_m=float(m.p95_m),graph_status=read(B/'policies'/name/'online_graph_integrity.json')['status'],label='COPIED_FROZEN_STAGE22B'))
    comparison=pd.DataFrame(baseline+comparisons);P0ref=comparison[comparison.policy=='P0'].iloc[0];P5ref=comparison[comparison.policy=='P5'].iloc[0];comparison['application_byte_reduction_vs_P0']=1-comparison.application_bytes/P0ref.application_bytes;comparison['attempt_reduction_vs_P0']=1-comparison.GICP_receipt_attempts/P0ref.GICP_receipt_attempts;comparison['additional_bytes_vs_P5']=comparison.application_bytes-P5ref.application_bytes;comparison['ATE_difference_vs_P5_m']=comparison.joint_ATE_RMSE_m-P5ref.joint_ATE_RMSE_m
    gate_rows=[];full_quality=pd.DataFrame(query_rows);summary=pd.DataFrame(accuracy)
    for short in POLICIES:
        row=comparison[comparison.policy==short].iloc[0];integrity=read(G/short/'graph_integrity.json');agreement=pd.read_csv(G/short/'online_incremental_vs_batch.csv') if (G/short/'online_incremental_vs_batch.csv').exists() else pd.DataFrame();scopes={}
        for scope,file in [('full','policy_comparison_full.csv'),('heldout','policy_comparison_holdout.csv')]:
            old=pd.read_csv(B/file);reference_row=old[(old.policy==P0)&(old.direction=='ALL')].iloc[0];new=full_quality[(full_quality.policy==short)&(full_quality.scope==scope)&(full_quality.direction=='ALL')].iloc[0];scopes[scope]=dict(recall=float(new.recall),precision=float(new.precision),recall_drop=float(reference_row.recall-new.recall),precision_drop=float(reference_row.precision-new.precision),recall_gate=bool(new.recall>=reference_row.recall-limits['recall_drop_max']),precision_gate=bool(new.precision>=reference_row.precision-limits['precision_drop_max']))
        checks=dict(attempt_reduction=bool(row.attempt_reduction_vs_P0>=limits['attempt_reduction_min']-1e-12),application_byte_reduction=bool(row.application_byte_reduction_vs_P0>=limits['application_byte_reduction_min']-1e-12),recall=all(x['recall_gate'] for x in scopes.values()),precision=all(x['precision_gate'] for x in scopes.values()),joint_ATE=bool(pd.notna(row.joint_ATE_RMSE_m) and row.joint_ATE_RMSE_m<=P0ref.joint_ATE_RMSE_m*limits['joint_ATE_ratio_max']),no_new_catastrophic_jump=integrity['new_catastrophic_jumps']==0,proper_factor_convention=integrity['status']=='PASS',incremental_batch_agreement=bool(len(agreement) and agreement.iloc[0].status=='PASS'))
        gate_rows.append(dict(policy=short,checks=checks,all_original_gates_pass=all(checks.values()),scope_quality=scopes,ATE_ratio_vs_P0=float(row.joint_ATE_RMSE_m/P0ref.joint_ATE_RMSE_m) if pd.notna(row.joint_ATE_RMSE_m) else None,ATE_improves_vs_P5=bool(pd.notna(row.joint_ATE_RMSE_m) and row.joint_ATE_RMSE_m<P5ref.joint_ATE_RMSE_m),extra_bytes_vs_P5=int(row.additional_bytes_vs_P5),interpretation=LABEL))
    # Descriptive Pareto flags, never policy selection/promotion on exposed GT.
    for target,name in [('recall','pareto_bytes_recall'),('joint_ATE_RMSE_m','pareto_bytes_ATE')]:
        flags=[]
        for a in comparison.itertuples():
            better=(lambda x,y:x>=y) if target=='recall' else (lambda x,y:x<=y);dominated=any(b.policy!=a.policy and b.application_bytes<=a.application_bytes and better(getattr(b,target),getattr(a,target)) and (b.application_bytes<a.application_bytes or getattr(b,target)!=getattr(a,target)) for b in comparison.itertuples());flags.append(not dominated)
        comparison[name]=flags
    csv(E/'p6_query_quality.csv',query_rows);csv(E/'p6_factor_quality.csv',factor_rows);csv(E/'p6_factor_labels.csv',factor_labels);csv(E/'p6_graph_accuracy.csv',accuracy);csv(E/'p6_region_accuracy.csv',regional);csv(E/'p6_loop_residual_summary.csv',residual_rows);csv(E/'p6_pareto_comparison.csv',comparison);dump(E/'p6_gate_results.json',dict(limits=limits,original_gate_source_sha256=sha(gates_path),assessment='same original full/heldout query gates and FULL incremental joint ATE ratio; heldout graph ATE additionally reported but no invented baseline',policies=gate_rows,label=LABEL))
    plots=[('application_bytes','recall','application_bytes_vs_recall.png','Application GB (protocol simulated)','Geometric query recall · offline GT'),('application_bytes','joint_ATE_RMSE_m','application_bytes_vs_ATE.png','Application GB (protocol simulated)','Joint rigid ATE RMSE · m · offline GT'),('GICP_receipt_attempts','admitted_factors','attempts_vs_admitted_factors.png','Selected historical GICP receipts','Admitted graph factors'),('application_bytes','map_NN_median_m','communication_vs_map_consistency.png','Application GB (protocol simulated)','GT-free symmetric map NN median · m')]
    for x,y,file,xlabel,ylabel in plots:
        fig,ax=plt.subplots(figsize=(8,5));xx=comparison[x]/1e9 if x=='application_bytes' else comparison[x];ax.scatter(xx,comparison[y],s=65,c=['#455a64','#de8e35','#2073b5','#40a997','#9966bd','#cd5272']);
        for j,r in comparison.iterrows():ax.annotate(r.policy,(xx.iloc[j],r[y]),xytext=(5,6+j%2*7),textcoords='offset points',fontsize=9)
        ax.set(xlabel=xlabel,ylabel=ylabel,title=LABEL);ax.grid(alpha=.2);fig.tight_layout();fig.savefig(E/file,dpi=160);plt.close(fig)
    decision=dict(P6_GRAPH_RECONSTRUCTION='PASS' if all(r['graph_status']=='PASS' for r in comparisons) else 'FAIL',P6_INCREMENTAL_BATCH_AGREEMENT='PASS' if all(r['checks']['incremental_batch_agreement'] for r in gate_rows) else 'FAIL',P6_RETROSPECTIVE_MAPPING_QUALITY='SOME_POLICIES_PASS_ALL_ORIGINAL_GATES' if any(r['all_original_gates_pass'] for r in gate_rows) else 'NO_POLICY_PASSED_ALL_ORIGINAL_GATES',P6_COMMUNICATION_EFFICIENCY='ALL_PASS_P0_COST_GATES_BUT_ALL_COST_MORE_THAN_P5' if all(r['checks']['attempt_reduction'] and r['checks']['application_byte_reduction'] and r['extra_bytes_vs_P5']>0 for r in gate_rows) else 'MIXED_SEE_EXACT_GATES',P6_INDEPENDENT_VALIDATION='NOT_RUN',STAGE23_FINAL_INDEPENDENT_VALIDATION='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE',label=LABEL,new_GICP_calls=0,GT_based_retuning=False,policy_promoted=False,matched_budget_experiment='NOT_RUN',matched_budget_followup_candidates=[r['policy'] for r in gate_rows if r['ATE_improves_vs_P5'] and r['extra_bytes_vs_P5']>0],candidate_caveat='lower incremental ATE on this exposed route only, not a selected policy, universal mapping improvement or matched-budget result',baseline_metrics='copied frozen P0/P5 tables; no re-evaluation',stopping_point='Stage23E complete; no automatic matched-budget or independent experiment')
    dump(E/'stage23e_decision.json',decision);dump(E/'stage23e_evaluation_artifact_manifest.json',{p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in E.iterdir() if p.is_file() and p.name!='stage23e_evaluation_artifact_manifest.json'});print(json.dumps(decision),flush=True)
if __name__=='__main__':main()
