"""Gate22A-3: new causal measurements only, one persistent frozen-default ISAM2."""
import resource,time
import gtsam
from scipy.spatial.transform import Rotation
import run_stage13b_incremental_isam2 as b
from stage22a_common import *
d=b.d

def transform(row):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([row.qx,row.qy,row.qz,row.qw]).as_matrix();T[:3,3]=[row.tx,row.ty,row.tz];return T
def key(n):return b.numeric_key(n)
def values(mat):return b.to_values(mat)
def trajectory(matrices,f):
    return d.trajectory(f,matrices)

def main():
    verify_freeze();assert read('stage22a2_decision.json')['status']=='ONLINE_GICP_VERIFICATION_READY';assert not (OUT/'stage22a3_decision.json').exists()
    online_guard();f=frames();events=events_for(f);loops=pd.read_csv(OUT/'online_verified_loops.csv')
    localrows=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv',usecols=d.POSE_COLUMNS)
    local={(r.robot_id,int(r.keyframe_id)):transform(r) for r in localrows.itertuples()}
    params=gtsam.ISAM2Params();assert str(params)==json.loads((b.OUT/'isam2_parameter_policy.json').read_text())['parameter_text']
    noise,robust=d.build_noise_models();lmparams=gtsam.LevenbergMarquardtParams()
    oldlm=json.loads((ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_gtsam_solver_policy.json').read_text())['parameters']
    assert all(getattr(lmparams,k)()==v for k,v in oldlm.items())
    dump('online_backend_policy.json',dict(isam2_parameter_text=str(params),LM=oldlm,extra_empty_updates=0,initialization='first newly discovered causal admissible loop; only arrived prefixes',noise_sigmas=[float(np.deg2rad(5))]*3+[1.]*3,Huber=1.345,prior_sigma=1e-6,anchor='robot1 KF150',GT_used=False))
    if loops.empty:
        no=dict(status='CAUSAL_ONLINE_GRAPH_NOT_READY',reason='NO_FIRST_BRIDGE_FOUND',nodes=5796,local_components=2)
        dump('online_graph_integrity.json',no);dump('stage22a3_decision.json',no);csv('online_incremental_vs_batch.csv',[dict(status='NOT_RUN',reason='NO_FIRST_BRIDGE_FOUND')]);return
    groups={int(e):x.sort_values('loop_id').to_dict('records') for e,x in loops.groupby('discovery_event')}
    first=loops.iloc[0];first_event=int(first.discovery_event);Z=np.asarray(json.loads(first.canonical_transform));qn=('robot1',int(first.canonical_robot1_kf));cn=('robot3',int(first.canonical_robot3_kf))
    S=local[qn]@Z@inv(local[cn]);common=lambda n:local[n] if n[0]=='robot1' else S@local[n]
    initial=inv(Z)@inv(common(qn))@common(cn);assert np.linalg.norm(initial[:3,3])<1e-8 and Rotation.from_matrix(initial[:3,:3]).magnitude()<1e-8
    dump('first_online_bridge.json',dict(discovery_event=first_event,normalized_time_ns=int(first.discovery_normalized_ns),query_robot=first.query_robot,query_keyframe_id=int(first.query_keyframe_id),candidate_robot=first.candidate_robot,candidate_keyframe_id=int(first.candidate_keyframe_id),canonical_robot1_kf=qn[1],canonical_robot3_kf=cn[1],translation_residual_m=float(np.linalg.norm(initial[:3,3])),rotation_residual_rad=float(Rotation.from_matrix(initial[:3,:3]).magnitude()),common_frame_transform=S.tolist(),archived_id_forced=False))
    selection=json.loads((ROOT/'outputs/cumulti_v1/12c_dataset_stability/stable_segment_selection.json').read_text())
    graph=gtsam.NonlinearFactorGraph();seen=set();odom=set();inserted=set();trace=[];loop_res=[];isam=None;estimate=None;first_matrices=None;failed=None;instance=None;last={r:STARTS[r]-1 for r in STARTS};start=time.monotonic()
    def odom_factor(n):
        prev=(n[0],n[1]-1);return gtsam.BetweenFactorPose3(key(prev),key(n),d.pose(inv(local[prev])@local[n]),noise)
    def loop_factor(row):
        q=('robot1',int(row['canonical_robot1_kf']));c=('robot3',int(row['canonical_robot3_kf']));assert q in seen and c in seen
        return gtsam.BetweenFactorPose3(key(q),key(c),d.pose(np.asarray(json.loads(row['canonical_transform']))),robust)
    try:
        for e in events:
            i=e['event_index'];n=(e['robot_id'],e['keyframe_id']);assert n not in seen;assert n[1]==last[n[0]]+1;seen.add(n);last[n[0]]=n[1];arrivals=groups.get(i,[]);update=extract=0.;new=gtsam.NonlinearFactorGraph();v=gtsam.Values()
            if i==first_event:
                isam=gtsam.ISAM2(params);instance=id(isam)
                new.add(gtsam.PriorFactorPose3(key(('robot1',150)),d.pose(local[('robot1',150)]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
                for x in sorted(seen):
                    v.insert(key(x),d.pose(common(x)))
                    if x[1]>STARTS[x[0]]:new.add(odom_factor(x));odom.add(x)
                first_matrices={x:common(x) for x in seen}
            elif isam is not None:
                assert id(isam)==instance and n[1]>STARTS[n[0]]
                factor=odom_factor(n);new.add(factor);assert n not in odom;odom.add(n)
                previous=isam.calculateEstimatePose3(key((n[0],n[1]-1)));v.insert(key(n),previous.compose(factor.measured()))
            if isam is not None:
                for row in arrivals:
                    lid=int(row['loop_id']);assert lid not in inserted;inserted.add(lid);new.add(loop_factor(row))
                for j in range(new.size()):graph.add(new.at(j))
                _,update=timed(lambda:isam.update(new,v));estimate,extract=timed(lambda:isam.calculateEstimate());assert estimate.size()==len(seen)
                assert isam.getFactorsUnsafe().size()==graph.size()==1+len(odom)+len(inserted)
                matrices=b.matrices_from(estimate,seen);assert all(np.isfinite(T).all() for T in matrices.values())
                jumps=0;maxstep=0.
                for robot in STARTS:
                    ids=range(STARTS[robot],last[robot]+1);post=np.array([matrices[(robot,k)][:3,3] for k in ids]);pre=np.array([common((robot,k))[:3,3] for k in ids])
                    ds=np.linalg.norm(np.diff(post,axis=0),axis=1);ps=np.linalg.norm(np.diff(pre,axis=0),axis=1);limit=selection['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold']
                    jumps+=int((((ds>5*limit)&(ps<=5*limit))|((ds>=100)&(ps<100))).sum());maxstep=max(maxstep,float(ds.max()) if len(ds) else 0.)
                if jumps:raise RuntimeError('frozen catastrophic new jump rule failed: '+str(jumps))
            trace.append(dict(event_index=i,normalized_time_ns=e['normalized_ns'],nodes=len(seen),odometry=len(odom),loops=len(inserted),factors=graph.size(),isam2_update_ms=update,estimate_extract_ms=extract,status='LOCAL_ONLY' if isam is None else 'FINITE_CENTRAL',new_loops=len(arrivals)))
            if i%500==0:print('BACKEND',i,'loops',len(inserted),flush=True)
        assert len(seen)==5796 and len(odom)==5794 and len(inserted)==len(loops) and graph.size()==5795+len(loops)
    except Exception as exc:failed=type(exc).__name__+': '+str(exc)
    csv('online_backend_event_trace.csv',trace)
    structural=failed is None
    if structural:
        final=b.matrices_from(estimate,seen);tr=trajectory(final,f);b.schema(tr,f);csv('online_incremental_trajectory.csv',tr)
        x0=values({n:common(n) for n in seen});opt=gtsam.LevenbergMarquardtOptimizer(graph,x0,lmparams);batch,ms=timed(lambda:opt.optimize());bm=b.matrices_from(batch,seen);bt=trajectory(bm,f);b.schema(bt,f);csv('online_batch_trajectory.csv',bt)
        diff=b.pose_difference(final,bm);gates=dict(translation_median_m=.10,translation_p95_m=.50,rotation_median_deg=.5,rotation_p95_deg=2.)
        numerical=all(diff[k]<=v for k,v in gates.items());csv('online_incremental_vs_batch.csv',[dict(status='PASS' if numerical else 'FAIL',same_new_graph=True,same_gauge=True,incremental_error=float(graph.error(estimate)),batch_error=float(graph.error(batch)),batch_runtime_ms=ms,batch_iterations=opt.iterations(),**diff)])
        residual=[]
        for row in loops.to_dict('records'):
            q=('robot1',int(row['canonical_robot1_kf']));c=('robot3',int(row['canonical_robot3_kf']));E=inv(np.asarray(json.loads(row['canonical_transform'])))@inv(final[q])@final[c]
            residual.append(dict(loop_id=int(row['loop_id']),translation_m=float(np.linalg.norm(E[:3,3])),rotation_deg=float(np.rad2deg(Rotation.from_matrix(E[:3,:3]).magnitude()))))
        csv('online_loop_residuals.csv',residual)
        # Frozen map protocol, no GT: physical matrix translation, not tangent v.
        by,allpoints=d.s10.map_points(tr,f['robot1'],f['robot3']);metric=d.s10.consistency(by['robot1'],by['robot3']);csv('online_map_consistency.csv',[dict(status='MEASURED_GT_FREE',**metric)])
        (OUT/'maps').mkdir(exist_ok=True);art=[]
        for robot,p in by.items():
            target=OUT/'maps'/f'online_{robot}.ply';d.s10.write_ply(target,d.s10.voxel(p));art.append(dict(path=str(target),bytes=target.stat().st_size,sha256=sha(target)))
        dump('online_map_manifest.json',dict(artifacts=art,frame_stride=10,point_stride=16,voxel_m=.5,GT_used=False))
    else:
        numerical=False;csv('online_incremental_vs_batch.csv',[dict(status='NOT_RUN',reason=failed)]);csv('online_map_consistency.csv',[dict(status='NOT_RUN',reason=failed)])
    status='CAUSAL_ONLINE_GRAPH_READY' if structural and numerical else 'CAUSAL_ONLINE_GRAPH_NOT_READY'
    integrity=dict(status='PASS' if structural else 'FAIL',reason=failed,nodes=len(seen),odometry=len(odom),loops=len(inserted),prior=1 if isam else 0,factors=graph.size(),expected_final_nodes=5796,expected_final_odom=5794,expected_final_factors=5795+len(loops),independent_discovered_loops=len(loops),exact_once=structural,persistent_ISAM2=True,elapsed_s=time.monotonic()-start,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    dump('online_graph_integrity.json',integrity)
    dump('stage22a3_decision.json',dict(status=status,structural_status=integrity['status'],numerical_status='PASS' if numerical else 'FAIL' if structural else 'NOT_RUN',reason=failed or ('original numerical agreement gates passed' if numerical else 'original numerical agreement gates failed; no retuning'),new_graph_only=True,GT_used=False))
    dump('pre_gt_online_decision.json',dict(frontend=read('stage22a2_decision.json')['status'],backend=read('stage22a3_decision.json'),frozen_result_hashes={p.name:sha(p) for p in OUT.glob('online_*.csv')},GT_read_yet=False))
    print(status,flush=True)
if __name__=='__main__':main()
