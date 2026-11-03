"""Independent P6 graphs: delayed constraints enter at INSERTION, never discovery."""
from stage23e_common import *
import io,zipfile,argparse,traceback
import gtsam
sys.path.insert(0,str(ROOT/'src/cumulti'))

def timed(f):
    t=time.perf_counter();v=f();return v,(time.perf_counter()-t)*1000
def write_trajectory(path,matrices,frame):
    a=np.array([matrices[r.robot_id,int(r.keyframe_id)] for r in frame.itertuples()]);q=Rotation.from_matrix(a[:,:3,:3]).as_quat();df=frame[['robot_id','keyframe_id','timestamp']].copy()
    for j,n in enumerate(['tx','ty','tz']):df[n]=a[:,j,3]
    for j,n in enumerate(['qx','qy','qz','qw']):df[n]=q[:,j]
    assert len(df)==5796 and not df.duplicated(['robot_id','keyframe_id']).any();proper(a)
    assert np.max(abs(np.linalg.norm(q,axis=1)-1))<1e-8
    for _,f in df.groupby('robot_id'):assert np.all(np.diff(f.timestamp.to_numpy(np.int64))>0)
    csv(path,df);return df

def export_maps(directory,conditions,frame):
    source=read(MAP/'comparison_input_manifest.json');scans=source['selected_scans'];assert len(scans)==580
    start=time.monotonic();raw={}
    for identity,name in scans.items():
        r,k=identity.split(':');p=np.load(name)[::16,:3];p=p[np.isfinite(p).all(1)&(np.linalg.norm(p,axis=1)>1)&(np.linalg.norm(p,axis=1)<60)];raw[r,int(k)]=p
    (directory/'maps').mkdir(exist_ok=True);artifacts=[];metrics=[]
    for condition,matrices in conditions.items():
        by={}
        for robot in ['robot1','robot3']:
            points=[p@matrices[r,k][:3,:3].T+matrices[r,k][:3,3] for (r,k),p in raw.items() if r==robot];v=d.s10.voxel(np.vstack(points));by[robot]=v;path=directory/'maps'/f'{condition}_{robot}.ply';d.s10.write_ply(path,v);artifacts.append(dict(condition=condition,robot=robot,path=str(path),points=len(v),bytes=path.stat().st_size,sha256=sha(path)))
        path=directory/'maps'/f'{condition}_merged.ply';v=d.s10.voxel(np.vstack(list(by.values())));d.s10.write_ply(path,v);artifacts.append(dict(condition=condition,robot='merged',path=str(path),points=len(v),bytes=path.stat().st_size,sha256=sha(path)));metrics.append(dict(condition=condition,status='MEASURED_GT_FREE',**d.s10.consistency(by['robot1'],by['robot3'])))
    csv(directory/'online_map_consistency.csv',metrics)
    dump(directory/'online_map_manifest.json',dict(artifacts=artifacts,frame_stride=10,point_stride=16,range_m=[1,60],voxel_m=.5,source_scan_count=580,source_scan_inventory_sha256=sha(MAP/'comparison_input_manifest.json'),GT_used=False,elapsed_s=time.monotonic()-start))

def run(short):
    global historical,d,key
    lock_verify();gt_free_guard()
    import run_stage13b_incremental_isam2 as historical
    d=historical.d;key=historical.numeric_key;directory=G/short
    if (directory/'graph_integrity.json').exists():print('EXISTING_STAGE23E_GRAPH',short,flush=True);return
    assert not directory.exists(),'Incomplete prior execution retained; do not overwrite'
    directory.mkdir();loops,frame,local,events=load_factors(short);csv(directory/'factor_insertion_schedule.csv',loops)
    params=gtsam.ISAM2Params();frozen=read(B/'policies'/P0/'online_backend_policy.json');assert str(params)==frozen['isam2_parameter_text'];lm=gtsam.LevenbergMarquardtParams();assert all(getattr(lm,k)()==v for k,v in frozen['LM'].items())
    base,robust=d.build_noise_models();np.testing.assert_array_equal(base.sigmas(),frozen['noise_sigmas']);assert frozen['Huber']==1.345 and frozen['prior_sigma']==1e-6 and d.s10.POLICY['map_voxel_m']==.5
    dump(directory/'online_backend_policy.json',dict(**frozen,source='unchanged Stage22A/B scientific helpers; isolated output runner',loop_insertion='insertion_event',factor_order=['insertion_event','discovery_event','rank','robot1_kf','robot3_kf']))
    groups={int(i):f.to_dict('records') for i,f in loops.groupby('insertion_event')};first=loops.iloc[0];first_i=int(first.insertion_event);q,c=('robot1',int(first.robot1_kf)),('robot3',int(first.robot3_kf));Z=np.array(json.loads(first.canonical_transform));S=local[q]@Z@inv(local[c]);common=lambda n:local[n] if n[0]=='robot1' else S@local[n]
    edge_error=inv(Z)@inv(common(q))@common(c);assert np.linalg.norm(edge_error[:3,3])<1e-8 and Rotation.from_matrix(edge_error[:3,:3]).magnitude()<1e-8
    dump(directory/'first_online_bridge.json',dict(discovery_event=int(first.discovery_event),insertion_event=first_i,robot1_kf=q[1],robot3_kf=c[1],translation_residual_m=float(np.linalg.norm(edge_error[:3,3])),rotation_residual_rad=float(Rotation.from_matrix(edge_error[:3,:3]).magnitude()),common_frame_transform=S.tolist(),selection='first in frozen insertion/discovery/rank/KF order, no GT or quality selection'))
    thresholds=read(BASE/'12c_dataset_stability/stable_segment_selection.json');graph=gtsam.NonlinearFactorGraph();seen=set();odom=set();inserted=set();trace=[];factor_rows=[];state_index=[];last={'robot1':149,'robot3':233};isam=None;instance=None;estimate=None;failed=None;total_jumps=0;baseline={};archive=directory/'event_trajectories_float64.zip';start=time.monotonic()
    def odometry(n):
        prev=(n[0],n[1]-1);return gtsam.BetweenFactorPose3(key(prev),key(n),gtsam.Pose3(inv(local[prev])@local[n]),base)
    def remember(factor,kind,i,identity):
        graph.add(factor);factor_rows.append(dict(factor_index=graph.size()-1,kind=kind,insertion_event=i,keys=json.dumps(list(factor.keys())),identity=identity))
    try:
        with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as states:
            for event in events:
                i=int(event['event_index']);n=(event['robot_id'],int(event['keyframe_id']));assert n not in seen and n[1]==last[n[0]]+1;seen.add(n);last[n[0]]=n[1];baseline[n]=common(n);arrivals=groups.get(i,[]);new=gtsam.NonlinearFactorGraph();values=gtsam.Values();update=extract=0.
                if i==first_i:
                    isam=gtsam.ISAM2(params);instance=id(isam);prior=gtsam.PriorFactorPose3(key(('robot1',150)),gtsam.Pose3(local['robot1',150]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6)));new.add(prior);remember(prior,'PRIOR',i,'robot1:150')
                    for node in sorted(seen):
                        values.insert(key(node),gtsam.Pose3(common(node)))
                        if node[1]>(150 if node[0]=='robot1' else 234):f=odometry(node);new.add(f);remember(f,'ODOMETRY',i,str(node));odom.add(node)
                elif isam is not None:
                    assert id(isam)==instance;f=odometry(n);new.add(f);remember(f,'ODOMETRY',i,str(n));assert n not in odom;odom.add(n);previous=isam.calculateEstimatePose3(key((n[0],n[1]-1)));values.insert(key(n),previous.compose(f.measured()))
                if isam is not None:
                    for row in arrivals:
                        identity=(int(row['discovery_event']),int(row['rank']),int(row['robot1_kf']),int(row['robot3_kf']));assert identity not in inserted and int(row['insertion_event'])==i and identity[0]<=i
                        a,b=('robot1',identity[2]),('robot3',identity[3]);assert a in seen and b in seen;z=np.array(json.loads(row['canonical_transform']));f=gtsam.BetweenFactorPose3(key(a),key(b),gtsam.Pose3(z),robust);np.testing.assert_allclose(f.measured().matrix(),z,atol=1e-12,rtol=0);new.add(f);remember(f,'LOOP',i,json.dumps(identity));inserted.add(identity)
                    _,update=timed(lambda:isam.update(new,values));estimate,extract=timed(lambda:isam.calculateEstimate());assert estimate.size()==len(seen) and isam.getFactorsUnsafe().size()==graph.size()==1+len(odom)+len(inserted)
                    matrices=historical.matrices_from(estimate,seen)
                else:matrices={node:local[node] for node in seen}
                audit_start=time.perf_counter();ordered=sorted(seen);array=np.array([matrices[node] for node in ordered]);proper(array);jumps=0;maxstep=0.
                if isam is not None:
                    for robot in last:
                        ids=[(robot,k) for k in range(150 if robot=='robot1' else 234,last[robot]+1)];post=np.array([matrices[node][:3,3] for node in ids]);pre=np.array([baseline[node][:3,3] for node in ids]);ds=np.linalg.norm(np.diff(post,axis=0),axis=1);ps=np.linalg.norm(np.diff(pre,axis=0),axis=1);lim=thresholds['robots'][robot]['frozen_thresholds']['translation_step_xy_m']['threshold'];jumps+=int((((ds>5*lim)&(ps<=5*lim))|((ds>=100)&(ps<100))).sum());maxstep=max(maxstep,float(ds.max()) if len(ds) else 0.)
                total_jumps+=jumps
                if jumps:raise RuntimeError('unchanged catastrophic new-jump gate failed: '+str(jumps))
                audit_ms=(time.perf_counter()-audit_start)*1000;name=f'event_{i:06d}.npy';buffer=io.BytesIO();np.save(buffer,array,allow_pickle=False);t=time.perf_counter();states.writestr(name,buffer.getvalue());archive_ms=(time.perf_counter()-t)*1000
                row=dict(event_index=i,normalized_ns=int(event['normalized_ns']),nodes=len(seen),odometry=len(odom),loops=len(inserted),factors=graph.size(),robot1_last_KF=last['robot1'],robot3_last_KF=last['robot3'],new_loops=len(arrivals),isam2_update_ms=update,estimate_extract_ms=extract,integrity_audit_ms=audit_ms,state_archive_ms=archive_ms,objective=float(graph.error(estimate)) if isam is not None else None,catastrophic_new_jumps=jumps,max_translation_step_m=maxstep,status='FINITE_CENTRAL' if isam is not None else 'LOCAL_ONLY',state_file=name)
                trace.append(row);state_index.append(dict(event_index=i,state_file=name,shape=list(array.shape),bytes=len(buffer.getvalue()),sha256=hashlib.sha256(buffer.getvalue()).hexdigest(),frame_scope=row['status']))
                if i%500==0:csv(directory/'online_backend_event_trace.csv',trace);dump(directory/'progress.json',dict(policy=short,event_index=i,loops=len(inserted),elapsed_s=time.monotonic()-start));print('STAGE23E',short,i,len(inserted),time.monotonic()-start,flush=True)
        assert len(seen)==5796 and len(odom)==5794 and len(inserted)==COUNTS[short] and graph.size()==5795+COUNTS[short]
        for j in range(graph.size()):assert graph.at(j).equals(isam.getFactorsUnsafe().at(j),1e-12)
        final=historical.matrices_from(estimate,seen);write_trajectory(directory/'online_incremental_trajectory.csv',final,frame);initial=historical.to_values({node:common(node) for node in seen});opt=gtsam.LevenbergMarquardtOptimizer(graph,initial,lm);batch,ms=timed(lambda:opt.optimize());bm=historical.matrices_from(batch,seen);proper(np.array(list(bm.values())));write_trajectory(directory/'online_batch_trajectory.csv',bm,frame)
        diff=historical.pose_difference(final,bm);gates=read(B/'stage22b_predeclared_acceptance.json')['same_graph_agreement'];agreement=all(diff[k]<=v for k,v in gates.items());csv(directory/'online_incremental_vs_batch.csv',[dict(status='PASS' if agreement else 'FAIL',same_graph=True,same_gauge=True,incremental_error=float(graph.error(estimate)),batch_initial_error=float(graph.error(initial)),batch_error=float(graph.error(batch)),batch_runtime_ms=ms,batch_iterations=opt.iterations(),max_iterations_reached=opt.iterations()>=lm.getMaxIterations(),termination='native optimize returned normally; no parameter retuning',**diff)])
        residuals=[]
        for condition,lookup in [('incremental',final),('batch',bm)]:
            for row in loops.itertuples():
                z=np.array(json.loads(row.canonical_transform));err=inv(z)@inv(lookup['robot1',int(row.robot1_kf)])@lookup['robot3',int(row.robot3_kf)];e=gtsam.Pose3.Logmap(gtsam.Pose3(err));norm=float(np.linalg.norm(e/np.array(frozen['noise_sigmas'])));residuals.append(dict(condition=condition,discovery_event=int(row.discovery_event),insertion_event=int(row.insertion_event),robot1_kf=int(row.robot1_kf),robot3_kf=int(row.robot3_kf),translation_m=float(np.linalg.norm(err[:3,3])),rotation_deg=float(np.rad2deg(Rotation.from_matrix(err[:3,:3]).magnitude())),normalized_Logmap_norm=norm,diagnostic_reconstructed_huber_weight=min(1.,1.345/norm) if norm else 1.))
        csv(directory/'online_loop_residuals.csv',residuals);export_maps(directory,dict(incremental=final,batch=bm),frame)
    except Exception as ex:
        failed=type(ex).__name__+': '+str(ex);(directory/'failure_traceback.txt').write_text(traceback.format_exc());print('STAGE23E_FAILURE',short,failed,flush=True)
    csv(directory/'online_backend_event_trace.csv',trace);csv(directory/'graph_factor_ledger.csv',factor_rows);dump(directory/'event_trajectory_manifest.json',dict(path=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),states=len(state_index),state_index=state_index,precision='float64 matrices; row order robot1 ascending then robot3 ascending; endpoints in event trace',before_bridge='independent local frames; NOT a common graph'))
    integrity=dict(status='PASS' if failed is None else 'FAIL',reason=failed,nodes=len(seen),odometry=len(odom),loops=len(inserted),prior=1 if isam else 0,factors=graph.size(),expected_factors=5795+COUNTS[short],exact_once=failed is None,persistent_ISAM2=True,extra_empty_updates=0,components=1 if isam is not None else 2,new_catastrophic_jumps=total_jumps,finite_proper_SE3=failed is None,event_states=len(state_index),first_bridge_insertion_event=first_i,first_bridge_discovery_event=int(first.discovery_event),runtime_including_audit_state_export_maps_s=time.monotonic()-start,peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,GT_used=False,new_GICP_calls=0)
    dump(directory/'graph_integrity.json',integrity);dump(directory/'pre_gt_policy_artifacts.json',dict(policy=short,GT_used=False,files={str(p.relative_to(directory)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in directory.rglob('*') if p.is_file() and p.name!='pre_gt_policy_artifacts.json'}));print('STAGE23E_DONE',short,json.dumps(integrity),flush=True)
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('policy',choices=POLICIES);args=ap.parse_args();run(args.policy)
