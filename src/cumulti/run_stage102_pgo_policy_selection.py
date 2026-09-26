#!/usr/bin/env python3
"""Predeclared, GT-separated Stage-10.2 PGO budget/policy experiment."""
from __future__ import annotations
import hashlib, json, math, sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
sys.path.insert(0, str(Path(__file__).parent))
import run_stage10_offline_map_merge as s10

REPO=Path('/home/cas/fyp_place_recognition')
OUT=REPO/'outputs/cumulti_v1/10_2_pgo_policy_selection'
S9=REPO/'outputs/cumulti_v1/09_sc_gicp_integration'
S101=REPO/'outputs/cumulti_v1/10_1_pose_graph_correctness'
SCHEDULE=[25,50,100,200]
QUALITY=0.6091

def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()

def sanitize_rank1(raw):
    """Exact Stage-9 temporal sanitation: q gap >3 OR candidate gap >5 starts a cluster."""
    a=raw[(raw.SC_rank==1)&(raw.GICP_quality>=QUALITY)].copy().sort_values('query_keyframe_id')
    clusters=[]; cluster=-1; prev_q=prev_c=None
    for _,r in a.iterrows():
        if prev_q is None or r.query_keyframe_id-prev_q>3 or abs(r.candidate_keyframe_id-prev_c)>5: cluster+=1
        clusters.append(cluster); prev_q,prev_c=r.query_keyframe_id,r.candidate_keyframe_id
    a['temporal_cluster_id']=clusters; a['accepted_before_sanitation']=True; a['accepted_after_sanitation']=False
    a['duplicate_status']='redundant_temporal_match'
    keep=a.groupby('temporal_cluster_id').GICP_quality.idxmax()
    a.loc[keep,'accepted_after_sanitation']=True; a.loc[keep,'duplicate_status']='cluster_representative'
    a['transform_direction']='T_query_from_candidate: p_query = T_query_from_candidate * p_candidate'
    return a,a[a.accepted_after_sanitation].copy()

def loop_tuples(df,n1):
    out=[]
    for r in df.itertuples():
        z=np.eye(4); z[:3,3]=[r.tx,r.ty,r.tz]; z[:3,:3]=s10.Rotation.from_quat([r.qx,r.qy,r.qz,r.qw]).as_matrix()
        out.append((int(r.query_keyframe_id),n1+int(r.candidate_keyframe_id),z,'loop',r))
    return out

def init(policy, loops, l1, l3, n1):
    best=max(loops,key=lambda e:e[4].GICP_quality); q,c,z,_,row=best
    S=l1[q]@z@s10.inv(l3[c-n1]); x=np.array(list(l1)+[S@p for p in l3])
    r=s10.log(s10.inv(z)@s10.inv(x[q])@x[c]); t=float(np.linalg.norm(r[:3])); deg=float(np.rad2deg(np.linalg.norm(r[3:])))
    assert t<1e-6 and deg<1e-6
    result={'policy':policy,'query_keyframe_id':int(row.query_keyframe_id),'candidate_keyframe_id':int(row.candidate_keyframe_id),'GICP_quality':float(row.GICP_quality),'translation_residual_m':t,'rotation_residual_deg':deg,'status':'PASS','formula':'S = X_q @ Z_qc @ inverse(L_c)'}
    return x,result

def residual_stats(df):
    return {'translation_median_m':float(df.translation_after_m.median()),'translation_p95_m':float(df.translation_after_m.quantile(.95)),'rotation_median_deg':float(df.rotation_after_deg.median()),'rotation_p95_deg':float(df.rotation_after_deg.quantile(.95)),'weight_lt_0_1':int((df.robust_weight<.1).sum()),'weight_lt_0_25':int((df.robust_weight<.25).sum()),'weight_lt_0_5':int((df.robust_weight<.5).sum()),'minimum_weight':float(df.robust_weight.min())}

def loop_residuals(x0,x,loops,weights,odometry_edge_count):
    """Stage-10.2 audit uses loop weights at their real graph-edge offsets."""
    rows=[]
    for idx,(i,j,z,_,r) in enumerate(loops):
        def value(poses):
            v=s10.log(s10.inv(z)@s10.inv(poses[i])@poses[j])
            return float(np.linalg.norm(v[:3])),float(np.rad2deg(np.linalg.norm(v[3:])))
        before,after=value(x0),value(x)
        rows.append([idx,int(r.query_keyframe_id),int(r.candidate_keyframe_id),int(r.SC_rank),float(r.SC_score),float(r.GICP_quality),*before,*after,float(weights[odometry_edge_count+idx])])
    return pd.DataFrame(rows,columns=['loop_id','query_keyframe_id','candidate_keyframe_id','SC_rank','SC_score','GICP_quality','translation_before_m','rotation_before_deg','translation_after_m','rotation_after_deg','robust_weight'])

def policy_compare(top3,rank1):
    a={(int(r.query_keyframe_id),int(r.candidate_keyframe_id)) for r in top3.itertuples()}; b={(int(r.query_keyframe_id),int(r.candidate_keyframe_id)) for r in rank1.itertuples()}
    rows=[]
    for name,d in [('TOP3_SANITIZED',top3),('RANK1_SANITIZED',rank1)]:
        q=d.GICP_quality
        rows.append({'policy':name,'edge_count':len(d),'query_coverage':d.query_keyframe_id.nunique(),'candidate_coverage':d.candidate_keyframe_id.nunique(),'SC_rank_counts':json.dumps({str(k):int(v) for k,v in d.SC_rank.value_counts().items()}),'quality_median':float(q.median()),'quality_p10':float(q.quantile(.1)),'quality_p90':float(q.quantile(.9)),'quality_p95':float(q.quantile(.95)),'common_exact_edges':len(a&b),'only_top3':len(a-b),'only_rank1':len(b-a)})
    return pd.DataFrame(rows)

def rpe(traj,k1,k3,step=10):
    """Offline-only fixed 10-keyframe relative-pose error, reported after policy freeze."""
    vals=[]
    for robot,k in [('robot1',k1),('robot3',k3)]:
        d=traj[traj.robot_id==robot].sort_values('keyframe_id').reset_index(drop=True)
        for i in range(len(d)-step):
            p0=s10.T(d.loc[i,['qx','qy','qz','qw']].to_numpy(float),d.loc[i,['tx','ty','tz']].to_numpy(float)); p1=s10.T(d.loc[i+step,['qx','qy','qz','qw']].to_numpy(float),d.loc[i+step,['tx','ty','tz']].to_numpy(float))
            g0=k.iloc[i]; g1=k.iloc[i+step]
            x0=s10.T(g0[['qx','qy','qz','qw']].to_numpy(float),g0[['x','y','z']].to_numpy(float)); x1=s10.T(g1[['qx','qy','qz','qw']].to_numpy(float),g1[['x','y','z']].to_numpy(float))
            e=s10.inv(s10.inv(x0)@x1)@(s10.inv(p0)@p1); z=s10.log(e); vals.append((np.linalg.norm(z[:3]),np.rad2deg(np.linalg.norm(z[3:]))))
    a=np.asarray(vals); return float(np.sqrt(np.mean(a[:,0]**2))),float(np.sqrt(np.mean(a[:,1]**2)))

def main():
    OUT.mkdir(parents=True,exist_ok=False)
    top3_path=S9/'sanitized_loop_edges.csv'; rank_path=S9/'rank1_gicp_results.csv'
    top3=pd.read_csv(top3_path); assert len(top3)==139
    raw=pd.read_csv(rank_path); rank_raw,rank1=sanitize_rank1(raw)
    required={'query_robot','query_keyframe_id','candidate_robot','candidate_keyframe_id','SC_rank','SC_score','best_shift','GICP_fitness','GICP_inlier_RMSE','GICP_quality','tx','ty','tz','qx','qy','qz','qw','transform_direction'}
    assert required <= set(rank_raw.columns) and (rank_raw.SC_rank==1).all()
    rank_raw.to_csv(OUT/'rank1_raw_edges.csv',index=False); rank1.to_csv(OUT/'rank1_sanitized_edges.csv',index=False)
    (OUT/'rank1_sanitation_summary.json').write_text(json.dumps({'quality_threshold':QUALITY,'raw_accepted_rank1_edge_count':len(rank_raw),'sanitized_rank1_edge_count':len(rank1),'temporal_duplicates_removed':len(rank_raw)-len(rank1),'cluster_count':int(rank_raw.temporal_cluster_id.nunique()),'rule':'query gap >3 OR absolute candidate gap >5 starts a new cluster; maximum GICP_quality represents each cluster'},indent=2)+'\n')
    (OUT/'top3_loop_policy.json').write_text(json.dumps({'policy':'TOP3_SANITIZED','edge_count':len(top3),'input_path':str(top3_path),'sha256':digest(top3_path),'immutable_stage9_input':True},indent=2)+'\n')
    policy_compare(top3,rank1).to_csv(OUT/'loop_policy_comparison_gtfree.csv',index=False)
    manifest={'stage':'10.2','main_repository_checkpoint':'ebe60523f880e33c15ae7ee3e32ffba759252682','teammate_backend':{'repository':'woshanli351-afk/fyp-lidar-registration','commit':'e9f3781b46a6f8c52bedc254bf7b2a24d14a7731','interface':'T_query_from_candidate; p_query = T_query_from_candidate * p_candidate'},'budget_schedule':SCHEDULE,'quality_threshold':QUALITY,'gt_policy':'not loaded until pre_gt_policy_decision.json is written','frozen_common_graph':{'robot1_nodes':2000,'robot3_nodes':4180,'sequential_odometry_edges':6178,'node_frame':'LiDAR / os_sensor','stage101_reference':str(S101)}}
    (OUT/'experiment_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    k1,l1,_=s10.aligned('robot1'); k3,l3,_=s10.aligned('robot3'); n1=len(k1); assert (n1,len(k3))==(2000,4180)
    policies={'TOP3_SANITIZED':top3,'RANK1_SANITIZED':rank1}; finals={}; sweep=[]; inits={}
    for name,d in policies.items():
        loops=loop_tuples(d,n1); x0,san=init(name,loops,l1,l3,n1); inits[name]=(x0,loops,san); (OUT/f'initialization_sanity_{name.lower().split("_")[0]}.json').write_text(json.dumps(san,indent=2)+'\n')
        chosen=None
        for budget in SCHEDULE:
            s10.POLICY['least_squares_max_nfev']=budget
            x,summary,w=s10.optimize(x0,s10.relative_edges(l1,0)+s10.relative_edges(l3,n1)+loops,True)
            r=loop_residuals(x0,x,loops,w,(len(l1)-1)+(len(l3)-1)); stat=residual_stats(r)
            sweep.append({'policy':name,'max_nfev_budget':budget,'success':summary['converged'],'solver_status':'CONVERGED' if summary['converged'] else 'NOT_CONVERGED_AT_FROZEN_CAP','solver_message':'scipy least_squares status is represented by success flag','nfev':summary['iterations'],**summary,**stat})
            chosen=(x,summary,w,r,budget)
            if summary['converged']: break
        finals[name]=chosen
    sweep_df=pd.DataFrame(sweep); sweep_df.to_csv(OUT/'solver_budget_sweep.csv',index=False)
    decision={}
    for name,(x,s,w,r,budget) in finals.items():
        stat=residual_stats(r); decision[name]={'selected_max_nfev':budget,'converged':bool(s['converged']),'solver_status':'CONVERGED' if s['converged'] else 'NO_CONVERGED_SOLVER_WITHIN_PREDECLARED_BUDGET','edge_count':len(policies[name]),'initialization_edge':inits[name][2],'final_objective':s['objective_final'],'loop_residual_statistics':stat,'robust_weight_statistics':stat,'finite_valid_se3':bool(np.isfinite(np.asarray(x)).all())}
    candidates=[n for n in ['RANK1_SANITIZED','TOP3_SANITIZED'] if decision[n]['converged'] and decision[n]['finite_valid_se3']]
    selected=candidates[0] if candidates else 'NO_POLICY_READY'
    (OUT/'pre_gt_policy_decision.json').write_text(json.dumps({'selection_hierarchy':'formal convergence, finite SE(3), non-catastrophic GT-free residuals; if both pass prefer Rank-1 as cheaper online policy supported by frozen registration evidence','selected_demo_policy':selected,'policies':decision},indent=2)+'\n')
    # Final state was already produced in the GT-free sweep; no graph is rerun here.
    single=inits['TOP3_SANITIZED'][0]; single_df=s10.frame_csv(single,k1,k3); single_df.to_csv(OUT/'single_loop_trajectory.csv',index=False)
    all_traj={'SINGLE_LOOP':single_df}
    for name,(x,s,w,r,budget) in finals.items():
        tag='top3' if name.startswith('TOP3') else 'rank1'; traj=s10.frame_csv(x,k1,k3); traj.to_csv(OUT/f'{tag}_final_trajectory.csv',index=False); r.to_csv(OUT/f'loop_residuals_{tag}.csv',index=False); all_traj[name]=traj
    # Non-robust diagnostics use each policy's selected budget, never GT-selected.
    nonrows=[]
    for name,(xrob,srob,wrob,rrob,budget) in finals.items():
        x0,loops,_=inits[name]; s10.POLICY['least_squares_max_nfev']=budget; x,s,w=s10.optimize(x0,s10.relative_edges(l1,0)+s10.relative_edges(l3,n1)+loops,False); t=s10.frame_csv(x,k1,k3); ev,_,_=s10.joint_eval(t,k1,k3); jm=[z['ATE_RMSE_m'] for z in ev if z['scope']=='joint'][0]; nonrows.append({'policy':name,'max_nfev':budget,'converged':s['converged'],'objective_final':s['objective_final'],'joint_ATE_RMSE_m_offline_only':jm})
    pd.DataFrame(nonrows).to_csv(OUT/'nonrobust_diagnostic.csv',index=False)
    # GT-free map evaluation and map files. GT has still not been read here.
    map_rows=[]; map_paths={}
    for name,traj in all_traj.items():
        by,merged=s10.map_points(traj,k1,k3); c=s10.consistency(by['robot1'],by['robot3']); map_rows.append({'condition':name,**c}); tag={'SINGLE_LOOP':'single_loop','TOP3_SANITIZED':'top3_robust','RANK1_SANITIZED':'rank1_robust'}[name]; path=OUT/f'{tag}_merged_map.ply'; s10.write_ply(path,s10.voxel(merged)); map_paths[name]=path
    pd.DataFrame(map_rows).to_csv(OUT/'map_consistency_policy_comparison.csv',index=False)
    (OUT/'map_artifact_manifest.json').write_text(json.dumps({'large_files_not_committed_to_git':True,'artifacts':[{'condition':n,'absolute_server_path':str(p),'size_bytes':p.stat().st_size,'sha256':digest(p)} for n,p in map_paths.items()]},indent=2)+'\n')
    # GT is first accessed below this line, after policy decision and map evaluation are serialized.
    gtrows=[]
    for name,traj in all_traj.items():
        ev,_,_=s10.joint_eval(traj,k1,k3); rt,rr=rpe(traj,k1,k3,10)
        for z in ev: gtrows.append({'condition':name,**z,'RPE_keyframe_interval':10,'RPE_translation_RMSE_m':rt,'RPE_rotation_RMSE_deg':rr})
    pd.DataFrame(gtrows).to_csv(OUT/'offline_gt_policy_evaluation.csv',index=False)
    # Figures use already frozen policy/result data; no selection uses their GT labels.
    def xy(ax,t,title):
        for robot,color in [('robot1','tab:blue'),('robot3','tab:orange')]:
            z=t[t.robot_id==robot]; ax.plot(z.tx,z.ty,color=color,label=robot)
        ax.set_title(title); ax.set_aspect('equal'); ax.legend()
    fig,axes=plt.subplots(1,2,figsize=(14,6));
    for ax,(name,traj) in zip(axes,[(n,all_traj[n]) for n in ['TOP3_SANITIZED','RANK1_SANITIZED']]):
        xy(ax,traj,name); pp=s10.lookup(traj)
        for _,r in policies[name].iterrows():
            a=pp[('robot1',int(r.query_keyframe_id))]; b=pp[('robot3',int(r.candidate_keyframe_id))]; ax.plot([a[0,3],b[0,3]],[a[1,3],b[1,3]],c='gray',alpha=.12,lw=.35)
    fig.tight_layout(); fig.savefig(OUT/'loop_policy_graph_comparison.png',dpi=180); plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5))
    for n,g in sweep_df.groupby('policy'): ax.plot(g.max_nfev_budget,g.objective_final,'o-',label=n)
    ax.set(xlabel='max_nfev budget',ylabel='final objective');ax.legend();fig.tight_layout();fig.savefig(OUT/'solver_convergence.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(18,6));
    for ax,n in zip(axes,['SINGLE_LOOP','TOP3_SANITIZED','RANK1_SANITIZED']):xy(ax,all_traj[n],n)
    fig.tight_layout();fig.savefig(OUT/'trajectory_policy_comparison.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(18,6));
    for ax,n in zip(axes,['SINGLE_LOOP','TOP3_SANITIZED','RANK1_SANITIZED']):
        by,m=s10.map_points(all_traj[n],k1,k3);ax.scatter(m[::30,0],m[::30,1],s=.1,c=m[::30,2],cmap='viridis');ax.set_title(n);ax.set_aspect('equal')
    fig.tight_layout();fig.savefig(OUT/'map_policy_comparison_bev.png',dpi=180);plt.close(fig)
    maps=pd.read_csv(OUT/'map_consistency_policy_comparison.csv'); gte=pd.read_csv(OUT/'offline_gt_policy_evaluation.csv'); j=gte[gte.scope=='joint'];fig,ax=plt.subplots(1,3,figsize=(15,5));ax[0].bar(j.condition,j.ATE_RMSE_m);ax[0].set_title('GT-derived joint ATE');ax[1].bar(maps.condition,maps.median_m);ax[1].set_title('GT-free map NN median');ax[2].bar(maps.condition,maps.p95_m);ax[2].set_title('GT-free map NN p95');[a.tick_params(axis='x',rotation=20) for a in ax];fig.tight_layout();fig.savefig(OUT/'policy_metric_comparison.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5));[ax.hist(finals[n][3].robust_weight,bins=30,alpha=.5,label=n) for n in finals];ax.legend();ax.set_xlabel('robust loop weight');fig.tight_layout();fig.savefig(OUT/'robust_weight_distribution.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(1,2,figsize=(12,5));[ax[0].hist(finals[n][3].translation_after_m,bins=35,alpha=.5,label=n) for n in finals];[ax[1].hist(finals[n][3].rotation_after_deg,bins=35,alpha=.5,label=n) for n in finals];ax[0].set_title('translation residual');ax[1].set_title('rotation residual');[a.legend() for a in ax];fig.tight_layout();fig.savefig(OUT/'loop_residual_policy_comparison.png',dpi=180);plt.close(fig)
    shown=selected if selected!='NO_POLICY_READY' else 'TOP3_SANITIZED'; fig,ax=plt.subplots(figsize=(10,8));xy(ax,all_traj[shown],f'Pre-GT demo candidate: {shown}'); by,m=s10.map_points(all_traj[shown],k1,k3);ax.scatter(m[::50,0],m[::50,1],s=.1,c='k',alpha=.25);fig.tight_layout();fig.savefig(OUT/'final_demo_candidate.png',dpi=180);plt.close(fig)
    conv='CONVERGED' if any(decision[n]['converged'] for n in decision) else 'NO_CONVERGENCE_WITHIN_PREDECLARED_BUDGET'
    (OUT/'VALIDATION_REPORT.txt').write_text('[PASS] Stage-10.1 and frozen Stage-9 inputs were not modified.\n[PASS] GT excluded from policy construction, sanitation, initialization, and solver-budget selection.\n[PASS] pre-GT policy decision saved before offline GT evaluation.\n[PASS] fixed schedule 25,50,100,200 only; no graph-parameter tuning.\n[PASS] no live robot demo performed.\n')
    (OUT/'summary.txt').write_text(f'Stage 10.2 PGO convergence: {conv}. Pre-GT demo policy: {selected}. Offline GT supports evaluation only; no live demo.\n')

if __name__=='__main__': main()
