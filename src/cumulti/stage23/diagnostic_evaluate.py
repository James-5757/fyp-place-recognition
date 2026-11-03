"""Offline-only ablation evaluation after GT-free features and solves are frozen."""
from diagnostic_model import *
from scipy.stats import spearmanr
from scipy.spatial import cKDTree

def region_mask(pairs,nodes,arc):
    mask=np.zeros(len(nodes),bool)
    for q,c in pairs:
        for i,n in enumerate(nodes):
            center=arc['robot1',q] if n[0]=='robot1' else arc['robot3',c]
            if abs(arc[n]-center)<=50:mask[i]=True
    return mask if pairs else np.ones(len(nodes),bool)
def alignment_error(mat,G):
    p=mat[:,:3,3];g=G[:,:3,3];cp,cg=p.mean(0),g.mean(0);u,_,v=np.linalg.svd((p-cp).T@(g-cg));R=v.T@u.T
    if np.linalg.det(R)<0:v[-1]*=-1;R=v.T@u.T
    return np.linalg.norm((p-cp)@R.T+cg-g,axis=1)
def relative_errors(mat,qs,cs,Z):
    Rq=np.swapaxes(mat[qs,:3,:3],1,2);Rp=Rq@mat[cs,:3,:3];tp=(Rq@(mat[cs,:3,3]-mat[qs,:3,3])[:,:,None])[:,:,0];Zr=np.swapaxes(Z[:,:3,:3],1,2)
    te=(Zr@(tp-Z[:,:3,3])[:,:,None])[:,:,0];re=Zr@Rp
    return np.linalg.norm(te,axis=1),np.rad2deg(Rotation.from_matrix(re).magnitude())
def projected(marg,v,q,c):
    ks=[key(('robot1',q)),key(('robot3',c))];joint=marg.jointMarginalCovariance(gtsam.KeyVector(ks));C=np.block([[joint.at(a,b) for b in ks] for a in ks]);a=np.zeros((6,6),order='F');b=np.zeros((6,6),order='F');v.atPose3(ks[0]).between(v.atPose3(ks[1]),a,b);J=np.hstack([a,b])/SIG[:,None];return float(np.trace(J@C@J.T))
def main():
    freeze=read(OUT/'stage23b_solve_freeze.json');assert sha(OUT/'gt_free_criticality_features.csv')==freeze['features_sha256'];assert sha(OUT/'stage23b_solver_results.csv')==freeze['solver_results_sha256']
    dump('stage23b_offline_entry.json',dict(features_and_solve_plan_frozen_before_GT=True,feature_sha256=freeze['features_sha256'],solve_sha256=freeze['solver_results_sha256'],unix_ns=time.time_ns(),selector_process_has_no_GT=True))
    loops,frame,nodes,local,arc,events,params=load();policy=read(OUT/'stage23b_policy.json');nodeidx={n:i for i,n in enumerate(nodes)};cases={r['name']:r for r in policy['cases']};results=pd.read_csv(OUT/'stage23b_solver_results.csv')
    # First GT read in this evaluator. No solver/selector is run after it.
    gt={r:pd.read_csv(Path('/home/cas/CU-Multi/processed_v1/full_2hz')/r/'keyframes.csv').set_index('keyframe_id') for r in ['robot1','robot3']}
    G=np.array([np.block([[Rotation.from_quat(gt[r].loc[k,['qx','qy','qz','qw']].to_numpy(float)).as_matrix(),gt[r].loc[k,['x','y','z']].to_numpy(float)[:,None]],[np.zeros((1,3)),np.ones((1,1))]]) for r,k in nodes])
    from diagnostic_execute import metadata
    meta,index,sets=metadata(loops,arc);union={**index[P5],**index[P0]};pairs=sorted(union);qs=np.array([nodeidx['robot1',q] for q,c in pairs]);cs=np.array([nodeidx['robot3',c] for q,c in pairs]);Zgt=np.array([inv(G[a])@G[b] for a,b in zip(qs,cs)]);Zmeasured=np.array([np.asarray(json.loads(union[p].canonical_transform)) for p in pairs])
    for row in meta:
        a,b=nodeidx['robot1',row['robot1_kf']],nodeidx['robot3',row['robot3_kf']];d=float(np.linalg.norm(G[a,:2,3]-G[b,:2,3]));row.update(offline_GT_d_xy_m=d,offline_correct_d_xy_lt5=d<5)
    oldmeta=pd.read_csv(OUT/'stage23b_GT_free_loop_metadata.csv').set_index(['policy','robot1_kf','robot3_kf'])
    for row in meta:
        old=oldmeta.loc[row['policy'],row['robot1_kf'],row['robot3_kf']];row['temporal_cluster']=None if pd.isna(old.get('temporal_cluster')) else int(old.temporal_cluster);row['P5_route_bin_uncovered']=bool(old.P5_region_uncovered)
    exact=[]
    for q,c in pairs:
        records=[r for r in meta if (r['robot1_kf'],r['robot3_kf'])==(q,c)];r=records[0]
        exact.append(dict(robot1_kf=q,robot3_kf=c,membership=r['membership'],P0_admitted=(q,c) in sets[P0],P5_admitted=(q,c) in sets[P5],offline_correct_d_xy_lt5=r['offline_correct_d_xy_lt5'],offline_GT_d_xy_m=r['offline_GT_d_xy_m']))
    csv('loop_set_exact_comparison.csv',exact);csv('p0_only_loop_diagnostics.csv',[r for r in meta if r['membership']=='P0_ONLY']);csv('p5_only_loop_diagnostics.csv',[r for r in meta if r['membership']=='P5_ONLY']);csv('stage23b_false_factor_audit.csv',[r for r in meta if not r['offline_correct_d_xy_lt5']])
    cache={n:build(n,loops,nodes,local,events) for n in [P0,P5]};basem={};basev={};basemarg={};baseerrors={};baseRPE={}
    for n,name in [(P0,'BASE_P0'),(P5,'BASE_P5')]:
        mat=np.load(OUT/'stage23b_states'/(name+'.npz'))['matrices'];basem[n]=mat;basev[n]=to_values(mat,nodes);basemarg[n]=gtsam.Marginals(cache[n][0],basev[n]);baseerrors[n]=alignment_error(mat,G);baseRPE[n]=relative_errors(mat,qs,cs,Zgt)
    output=[];started=time.monotonic()
    for result in results.to_dict('records'):
        case=cases[result['case']];record=dict(result);n=case['policy'];removed=[tuple(x) for x in case['remove']]
        if result['status'] not in ['NORMAL_RETURN','ITERATION_CAP']:output.append(record);continue
        mat=np.load(OUT/'stage23b_states'/(case['name']+'.npz'))['matrices'];err=alignment_error(mat,G);mask=region_mask(case['remove'],nodes,arc);base=baseerrors[n];t,r=relative_errors(mat,qs,cs,Zgt);bt,br=baseRPE[n];mt,mr=relative_errors(mat,qs,cs,Zmeasured)
        rmse=lambda x:float(np.sqrt(np.mean(x*x)))
        record.update(joint_ATE_RMSE_m=rmse(err),delta_joint_ATE_RMSE_m=rmse(err)-rmse(base),regional_nodes=int(mask.sum()),regional_ATE_RMSE_m=rmse(err[mask]),delta_regional_ATE_RMSE_m=rmse(err[mask])-rmse(base[mask]),regional_ATE_p95_m=float(np.percentile(err[mask],95)),delta_regional_ATE_p95_m=float(np.percentile(err[mask],95)-np.percentile(base[mask],95)),inter_robot_GT_relative_translation_RMSE_m=rmse(t),delta_inter_robot_relative_translation_RMSE_m=rmse(t)-rmse(bt),inter_robot_GT_relative_rotation_RMSE_deg=rmse(r),delta_inter_robot_relative_rotation_RMSE_deg=rmse(r)-rmse(br),fixed_measurement_relative_translation_RMSE_m=rmse(mt),fixed_measurement_relative_rotation_RMSE_deg=rmse(mr),connected_components=1,change_in_connectivity=0,joint_alignment_only=True,offline_GT_only=True)
        if removed:
            full,_,idx=cache[n];g=gtsam.NonlinearFactorGraph();removeidx={idx[x] for x in removed}
            for j in range(full.size()):
                if j not in removeidx:g.add(full.at(j))
            v=to_values(mat,nodes);marg=gtsam.Marginals(g,v);before=np.mean([projected(basemarg[n],basev[n],q,c) for q,c in removed]);after=np.mean([projected(marg,v,q,c) for q,c in removed]);record.update(projected_cov_trace_before=float(before),projected_cov_trace_after=float(after),delta_projected_cov_trace=float(after-before),covariance_note='cross-correlated normalized relative covariance; condition-specific optimized linearization/robust weights, not pure fixed-linearization information loss')
        output.append(record)
    csv('critical_loop_loo.csv',[r for r in output if r['kind'] in ['LOO_P0','LOO_P5']]);csv('critical_loop_group_ablation.csv',[r for r in output if r['kind'] in ['TEMPORAL_GROUP','SUPPORT_GROUP']]);csv('stage23b_all_ablations.csv',output)
    # Fixed map protocol; one cached sensor subset, reprojected through REAL case poses.
    mapframes=[]
    for robot,start in [('robot1',150),('robot3',234)]:
        f=gt[robot].loc[start:]
        for k,row in f.iloc[::10].iterrows():
            p=np.load(Path('/home/cas/CU-Multi/processed_v1/full_2hz')/robot/'lidar'/row.lidar_file,allow_pickle=False)[::16,:3];p=p[np.isfinite(p).all(1)&(np.linalg.norm(p,axis=1)>1)&(np.linalg.norm(p,axis=1)<60)];mapframes.append((robot,nodeidx[robot,int(k)],p))
    import run_stage10_offline_map_merge as s10
    uncovered=[r for r in meta if r['membership']=='P0_ONLY' and r['P5_route_bin_uncovered']];regions={'ALL':np.ones(len(nodes),bool)}
    for r in uncovered:regions[f"UNCOVERED_{r['R1_bin50']}_{r['R3_bin50']}"]=region_mask([[r['robot1_kf'],r['robot3_kf']]],nodes,arc)
    maprows=[];baseline_maps={}
    mapcases=[c for c in policy['cases'] if c['kind']!='LOO_P0'];successful={r['case'] for r in output if r.get('finite',False)}
    for case in mapcases:
        if case['name'] not in successful:continue
        mat=np.load(OUT/'stage23b_states'/(case['name']+'.npz'))['matrices'];points={}
        for robot,ix,cloud in mapframes:points[robot,ix]=cloud@mat[ix,:3,:3].T+mat[ix,:3,3]
        masks=dict(regions)
        if case['remove']:masks['REMOVED_ENDPOINT_NEIGHBORHOODS']=region_mask(case['remove'],nodes,arc)
        for label,mask in masks.items():
            by={r:[p for (rr,i),p in points.items() if rr==r and mask[i]] for r in ['robot1','robot3']}
            if not all(by.values()):maprows.append(dict(case=case['name'],region=label,status='INSUFFICIENT_BOTH_ROBOT_POINTS'));continue
            stats=s10.consistency(np.vstack(by['robot1']),np.vstack(by['robot3']));entry=dict(case=case['name'],policy=case['policy'],region=label,status='MEASURED_GT_FREE_MAP_NN',**stats)
            if case['kind']=='BASELINE':baseline_maps[case['policy'],label]=stats
            ref=baseline_maps.get((case['policy'],label))
            if ref:entry.update(delta_median_m=stats['median_m']-ref['median_m'],delta_p95_m=stats['p95_m']-ref['p95_m'])
            # Endpoint masks vary per case: compute the MATCHED baseline mask,
            # not a differently aligned or differently covered regional map.
            if label=='REMOVED_ENDPOINT_NEIGHBORHOODS':
                bm=basem[case['policy']];bp={r:[] for r in ['robot1','robot3']}
                for robot,ix,cloud in mapframes:
                    if mask[ix]:bp[robot].append(cloud@bm[ix,:3,:3].T+bm[ix,:3,3])
                ref=s10.consistency(np.vstack(bp['robot1']),np.vstack(bp['robot3']));entry.update(delta_median_m=stats['median_m']-ref['median_m'],delta_p95_m=stats['p95_m']-ref['p95_m'])
            maprows.append(entry)
        print('MAP_DIAGNOSTIC',case['name'],flush=True)
    csv('regional_map_quality.csv',maprows)
    features=pd.read_csv(OUT/'gt_free_criticality_features.csv');ft={(int(r.robot1_kf),int(r.robot3_kf)):r._asdict() for r in features.itertuples()};proxies=['nominal_info_nats','relative_uncertainty_trace','relative_uncertainty_max_eigenvalue','coverage_gain','region_connectivity_gain','global_connectivity_gain','predicted_benefit_per_byte'];validations=[]
    for cohort in ['LOO_P0','LOO_P5','TEMPORAL_GROUP']:
        cohortrows=[r for r in output if r['kind']==cohort and 'delta_joint_ATE_RMSE_m' in r]
        for proxy in proxies:
            x=[];y=[]
            for r in cohortrows:
                removed=cases[r['case']]['remove'];v=[ft[tuple(qc)][proxy] for qc in removed];v=[a for a in v if pd.notna(a)]
                if not v:continue
                x.append(float(np.mean(v)));y.append(float(r['delta_joint_ATE_RMSE_m']))
            x,y=np.asarray(x),np.asarray(y);ok=len(x)>=3 and np.ptp(x)>0 and np.ptp(y)>0;rho,pvalue=spearmanr(x,y) if ok else (None,None)
            for K in ([10,20] if cohort=='LOO_P0' else [3]):
                k=min(K,len(x));pred=np.argsort(-x,kind='stable')[:k];truth=np.argsort(-y,kind='stable')[:k];validations.append(dict(cohort=cohort,proxy=proxy,n=len(x),Spearman_rho=None if rho is None else float(rho),uncorrected_p=None if pvalue is None else float(pvalue),top_K=k,top_K_overlap_fraction=float(len(set(pred)&set(truth))/k) if k and ok else None,top_K_signed_harm_positive_fraction=float((y[pred]>0).mean()) if k and ok else None,random_overlap_reference=k/len(x) if len(x) else None,status='EXPLORATORY_SMALL_N' if len(x)<20 and ok else 'DEFINED' if ok else 'UNDEFINED_CONSTANT_OR_INSUFFICIENT',target='signed delta joint ATE; negative values retained; no materiality threshold fitted',group_aggregation='mean individual prefix proxy, NOT exact joint group information',GT_used_for_proxy=False))
    csv('criticality_predictive_validation.csv',validations)
    assert sha(OUT/'gt_free_criticality_features.csv')==freeze['features_sha256'];dump('stage23b_offline_audit.json',dict(status='PASS',feature_file_unchanged=True,GT_after_feature_and_solver_freeze=True,new_optimization_after_GT=False,case_metrics=len(output),map_rows=len(maprows),elapsed_evaluation_s=time.monotonic()-started,GT_pose_convention='existing Stage22B reference CSV Pose3; fixed sensor-frame bias not newly calibrated'))
    print('STAGE23B_OFFLINE_EVALUATION_COMPLETE',flush=True)
if __name__=='__main__':main()
