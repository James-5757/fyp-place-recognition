"""Stage23 Phase0 ONLY: inventory, retrospective structure/local information, plan.

No selector, registration, optimization, dataset extraction or historical writes.
GT labels are read only for eligibility/retrospective diagnosis, not online use.
"""
import hashlib,itertools,json,os,resource,subprocess,sys,time,zipfile
import xml.etree.ElementTree as ET
from decimal import Decimal
from pathlib import Path
import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation

ROOT=Path('/home/cas/fyp_place_recognition');SRC=ROOT/'src/cumulti'
sys.path.insert(0,str(SRC))
OUT=ROOT/'outputs/cumulti_v1/23_information_aware'
PROC=Path('/home/cas/CU-Multi/processed_v1');RAW=Path('/home/cas/CU-Multi/raw')
A=ROOT/'outputs/cumulti_v1/22a_online_frontend';B=ROOT/'outputs/cumulti_v1/22b_budget_aware'
P0='P0_FULL_TOP3';P5='P5_BUDGET_AWARE_C75_B80'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for x in iter(lambda:f.read(4*1024**2),b''):h.update(x)
    return h.hexdigest()
def dump(name,obj):
    p=OUT/name;p.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def csv(name,rows):
    (rows if isinstance(rows,pd.DataFrame) else pd.DataFrame(rows)).to_csv(OUT/name,index=False,lineterminator='\n')
def read(p):return json.loads(Path(p).read_text())
def historical_digest():
    h=hashlib.sha256();count=0;size=0
    paths=[]
    for directory in [A,B,SRC/'stage22b']:
        paths.extend(p for p in directory.rglob('*') if p.is_file() and '__pycache__' not in str(p) and not p.name.endswith('.lock'))
    paths.extend(p for p in SRC.glob('*stage22a*.py') if p.is_file())
    for p in sorted(set(paths)):
        h.update((str(p.relative_to(ROOT))+' '+sha(p)+'\n').encode());size+=p.stat().st_size;count+=1
    return dict(files=count,bytes=size,sorted_path_SHA256_digest=h.hexdigest(),bytecode_caches_excluded=True)
def pose(r):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([r.qx,r.qy,r.qz,r.qw]).as_matrix();T[:3,3]=[r.tx,r.ty,r.tz];return T
def inverse(T):
    x=np.eye(4);x[:3,:3]=T[:3,:3].T;x[:3,3]=-x[:3,:3]@T[:3,3];return x
def archive_metadata(p):
    if not p.exists():return dict(status='MISSING',path=str(p))
    with zipfile.ZipFile(p) as z:
        metadata=[]
        for name in z.namelist():
            if name.endswith('metadata.yaml'):
                m=yaml.safe_load(z.read(name));info=m.get('rosbag2_bagfile_information',m)
                metadata.append(dict(member=name,topics=info.get('topics_with_message_count'),start=info.get('starting_time'),duration=info.get('duration')))
    return dict(path=str(p),bytes=p.stat().st_size,status='CENTRAL_DIRECTORY_AND_METADATA_READ',metadata=metadata)

def dataset():
    inventory=[];tables={};metadata={}
    for r in ['robot1','robot2','robot3','robot4']:
        gtpath=RAW/'main_campus'/r/(r+'_main_campus_gt_utm_poses.csv');cache=PROC/'full_2hz'/r/'keyframes.csv'
        if cache.exists():
            f=pd.read_csv(cache);f['sample_timestamp_ns']=f.lidar_timestamp_ns.astype('int64');f['cloud_exists']=False;f['cloud_header_valid']=False
            for ix,row in f.iterrows():
                p=cache.parent/'lidar'/row.lidar_file;ok=p.is_file();f.loc[ix,'cloud_exists']=ok
                if ok:
                    a=np.load(p,mmap_mode='r',allow_pickle=False);f.loc[ix,'cloud_header_valid']=bool(a.ndim==2 and a.shape[1]==4 and a.dtype==np.float32)
            sync_field=next((x for x in ['gt_minus_lidar_ns','gt_sync_error_ns'] if x in f),None)
            sampling='EXISTING_2HZ_LIDAR_KEYFRAMES';sync=float(abs(f[sync_field]).max()/1e6) if sync_field else None
        else:
            names=['timestamp','x','y','z','qx','qy','qz','qw'];g=pd.read_csv(gtpath,comment='#',header=None,names=names,dtype={'timestamp':str})
            ns=np.array([int(Decimal(t)*1_000_000_000) for t in g.timestamp],dtype=np.int64)
            targets=np.arange(ns[0],ns[-1]+1,500_000_000,dtype=np.int64);idx=np.unique(np.searchsorted(ns,targets));idx=idx[idx<len(g)]
            f=g.iloc[idx].copy().reset_index(drop=True);f['keyframe_id']=np.arange(len(f));f['sample_timestamp_ns']=ns[idx];f['cloud_exists']=False;f['cloud_header_valid']=False
            sampling='GT_ONLY_0.5S_GRID_FIRST_AT_OR_AFTER_EXACT_DECIMAL_NS';sync=None
        tables[r]=f
        rgb=cache.parent/'rgb_sync_index.csv';rgbstats=None
        if rgb.exists():
            x=pd.read_csv(rgb);rgbstats=dict(rows=len(x),absolute_offset_ms_median=float(x.absolute_rgb_offset_ms.median()),absolute_offset_ms_p95=float(x.absolute_rgb_offset_ms.quantile(.95)))
        archives={kind:archive_metadata(RAW/'main_campus'/r/(r+'_main_campus_'+kind+'.zip')) for kind in ['lidar','camera_rgb','gt_rel_poses','imu_gps']}
        metadata[r]=dict(archives=archives,RGB_sync=rgbstats,GT_pose_fields=['timestamp','x','y','z','qx','qy','qz','qw'],GT_unit='decimal Unix seconds; UTM reference solution',GT_source=str(gtpath))
        inventory.append(dict(route='main_campus/'+r,recording=r,keyframes=len(f),duration_s=(int(f.sample_timestamp_ns.iloc[-1])-int(f.sample_timestamp_ns.iloc[0]))/1e9,sampling=sampling,point_cloud_files=int(f.cloud_exists.sum()),header_valid_point_clouds=int(f.cloud_header_valid.sum()),cloud_finite_points_rescan='NOT_RUN_METADATA_ONLY',GT_sync_abs_max_ms=sync,RGB_sync_available=rgb.exists(),local_odometry='frozen12D retained nonGT local poses available' if r in ['robot1','robot3'] else 'raw ROS2 odometry metadata only; stable local pose cache NOT_AUDITED',initialization='frozen stable start KF150' if r=='robot1' else 'frozen stable start KF234' if r=='robot3' else 'NO_FROZEN_STABILITY_ELIGIBILITY',genuinely_separate_recording=True,independent_geographic_route=False,independent_of_exposed_R1_R3='different robot recording; same MainCampus campaign; not a fresh independent route' if r in ['robot2','robot4'] else 'NO already exposed development/test recording'))
    csv('dataset_route_inventory.csv',inventory);dump('dataset_sensor_metadata.json',metadata)
    urdfs=[]
    with zipfile.ZipFile(RAW/'calib/robot_description.zip') as z:
        for name in z.namelist():
            if not name.endswith('.urdf'):continue
            root=ET.fromstring(z.read(name));joints=[]
            for j in root.findall('joint'):
                parent,child,origin=j.find('parent'),j.find('child'),j.find('origin')
                joints.append(dict(name=j.get('name'),parent=None if parent is None else parent.get('link'),child=None if child is None else child.get('link'),origin=None if origin is None else origin.attrib))
            urdfs.append(dict(member=name,robot_name=root.get('name'),joints=joints))
    dump('coordinate_frame_audit.json',dict(URDF_source=str(RAW/'calib/robot_description.zip'),URDFs=urdfs,frozen_R1_R3_convention=read(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_graph_policy.json'),GT_world='UTM reference coordinates shared numerical domain; exact EPSG/zone not independently declared here',unknowns=['R2 EKF stability/local frame needs original odometry audit','R4 sensor timestamps/extrinsics need sensor data; GT alone cannot validate them'],ROS_message_frame_evidence='Stage12C R1/R3 EKF parent robot{1,3}_map / imu_link; zip metadata alone does not expose per-message frame_id'))
    coverage=[]
    variants=[('FULL_RECORDINGS_DIAGNOSTIC',tables)]
    retained=dict(tables);retained['robot1']=tables['robot1'].query('keyframe_id>=150').copy();retained['robot3']=tables['robot3'].query('keyframe_id>=234').copy()
    variants.append(('FROZEN_STARTS_R1_R3_OTHERS_UNAUDITED',retained))
    for variant,t in variants:
        for a,b in itertools.combinations(t,2):
            fs={a:t[a].reset_index(drop=True),b:t[b].reset_index(drop=True)};events=[]
            for robot,f in fs.items():
                ts=f.sample_timestamp_ns.to_numpy(np.int64);origin=int(ts[0]);events.extend((int(ns)-origin,robot,int(k),i) for i,(ns,k) in enumerate(zip(ts,f.keyframe_id)))
            events.sort();last={r:max(i for i,e in enumerate(events) if e[1]==r) for r in fs};seen={a:[],b:[]}
            stats={r:dict(queries=0,positive_overlap_queries=0,all_database_positive_queries=0,both_progressing_positive_queries=0,after_either_finishes_positive_queries=0,positive_pair_associations=0,synchronized_header_valid_cloud_pairs=0) for r in fs}
            for event,(ns,r,k,ix) in enumerate(events):
                other=b if r==a else a;f=fs[r];db=fs[other];xy=f.loc[ix,['x','y']].to_numpy(float);s=stats[r];s['queries']+=1
                full=np.linalg.norm(db[['x','y']].to_numpy(float)-xy,axis=1)<5;s['all_database_positive_queries']+=int(full.any())
                ids=seen[other];positive=full[ids] if ids else np.zeros(0,bool);n=int(positive.sum());s['positive_pair_associations']+=n
                if n:
                    s['positive_overlap_queries']+=1;period='both_progressing_positive_queries' if event<=min(last.values()) else 'after_either_finishes_positive_queries';s[period]+=1
                    if bool(f.loc[ix,'cloud_header_valid']):s['synchronized_header_valid_cloud_pairs']+=int((positive&db.loc[ids,'cloud_header_valid'].to_numpy(bool)).sum())
                seen[r].append(ix)
            for r,s in stats.items():
                other=b if r==a else a;spatial=s['positive_overlap_queries']>=100 and s['both_progressing_positive_queries']>=50
                coverage.append(dict(variant=variant,route_pair=a+'-'+b,query_robot=r,database_robot=other,**s,target_100_overall_PASS=s['positive_overlap_queries']>=100,target_50_both_progressing_PASS=s['both_progressing_positive_queries']>=50,spatial_targets_PASS=spatial,sensor_complete=bool(fs[r].cloud_header_valid.all() and fs[other].cloud_header_valid.all()),independent_of_R1_R3_recordings=not ({a,b}&{'robot1','robot3'}),genuinely_new_geographic_route=False,clock='own first timestamp normalization; deterministic robot/KF ties; not physical synchronization',pair_validity='both header-valid clouds, existing nearest-GT associations; no new sync-error cutoff; no full point-finiteness rescan'))
    csv('bidirectional_overlap_coverage.csv',coverage)
    return inventory,coverage

def loops_audit():
    loops={n:pd.read_csv(B/'policies'/n/'online_verified_loops.csv') for n in [P0,P5]}
    loc=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv');local={(r.robot_id,int(r.keyframe_id)):pose(r) for r in loc.itertuples()}
    distances={}
    for robot,g in loc.groupby('robot_id'):
        p=g[['tx','ty','tz']].to_numpy();s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))];distances.update({(robot,int(k)):float(v) for k,v in zip(g.keyframe_id,s)})
    gt={r:pd.read_csv(PROC/'full_2hz'/r/'keyframes.csv').set_index('keyframe_id') for r in ['robot1','robot3']}
    def pair(row):return int(row.canonical_robot1_kf),int(row.canonical_robot3_kf)
    index={n:{pair(r):r for r in f.itertuples()} for n,f in loops.items()};sets={n:set(x) for n,x in index.items()}
    regions={n:{} for n in sets}
    def region(q,c):return (int(distances['robot1',q]//50),int(distances['robot3',c]//50))
    for n,s in sets.items():
        for q,c in s:regions[n][region(q,c)]=regions[n].get(region(q,c),0)+1
    rows=[]
    for q,c in sorted(sets[P0]|sets[P5]):
        p=index[P0].get((q,c));v=index[P5].get((q,c));ref=p or v;key=region(q,c)
        dxy=float(np.linalg.norm(gt['robot1'].loc[q,['x','y']].to_numpy(float)-gt['robot3'].loc[c,['x','y']].to_numpy(float)))
        rows.append(dict(robot1_kf=q,robot3_kf=c,membership='COMMON' if p is not None and v is not None else 'P0_ONLY' if p is not None else 'P5_ONLY',GT_positive=dxy<5,GT_d_xy_m=dxy,P0_loop_id=None if p is None else int(p.loop_id),P5_loop_id=None if v is None else int(v.loop_id),discovery_event=int(ref.discovery_event),normalized_time_s=int(ref.discovery_normalized_ns)/1e9,R1_local_path_m=distances['robot1',q],R3_local_path_m=distances['robot3',c],R1_path_bin50m=key[0],R3_path_bin50m=key[1],P0_region_factors=regions[P0].get(key,0),P5_region_factors=regions[P5].get(key,0),unique_P0_region=regions[P0].get(key,0)==1,P5_region_uncovered=regions[P5].get(key,0)==0,P0_first_bridge=p is not None and int(p.loop_id)==1,P5_first_bridge=v is not None and int(v.loop_id)==1,global_graph_bridge=False,region_semantics='50m frozen-local-odometry pair cells; structural proxy, not proven physical or unique topological necessity'))
    csv('stage22b_missing_loop_audit.csv',rows)
    compare=pd.read_csv(B/'policy_comparison_full.csv');r0=compare.query('policy==@P0 and direction=="ALL"').iloc[0];r5=compare.query('policy==@P5 and direction=="ALL"').iloc[0]
    missing=[r for r in rows if r['membership']=='P0_ONLY' and r['GT_positive']];only5=[r for r in rows if r['membership']=='P5_ONLY' and r['GT_positive']]
    assert int(r0.geometric_true-r5.geometric_true)==335 and int(r0.admitted_true-r5.admitted_true)==57
    assert len(missing)-len(only5)==57
    common=sets[P0]&sets[P5];seed=min((index[P0][x] for x in common),key=lambda r:(r.discovery_event,r.loop_id))
    clusters=[];previous=None
    for r in sorted(missing,key=lambda x:x['normalized_time_s']):
        if previous is None or r['normalized_time_s']-previous>30:clusters.append([])
        r['missing_time_cluster']=len(clusters)-1;clusters[-1].append(r);previous=r['normalized_time_s']
    csv('stage22b_missing_loop_audit.csv',rows)
    csv('missing_loop_temporal_clusters.csv',[dict(cluster=i,factors=len(g),start_s=min(r['normalized_time_s'] for r in g),end_s=max(r['normalized_time_s'] for r in g),R1_path_min_m=min(r['R1_local_path_m'] for r in g),R1_path_max_m=max(r['R1_local_path_m'] for r in g),R3_path_min_m=min(r['R3_local_path_m'] for r in g),R3_path_max_m=max(r['R3_local_path_m'] for r in g),definition='gap<=30s between missing factor events; retrospective descriptor, no policy fitting') for i,g in enumerate(clusters)])
    summary=dict(geometrically_true_query_loss=335,net_admitted_true_factor_loss=57,P0_true_factors_absent_P5=len(missing),P5_only_true_factors=len(only5),common_factors=len(common),P0_only_factors=len(sets[P0]-sets[P5]),P5_only_factors=len(sets[P5]-sets[P0]),missing_first_bridge=sum(r['P0_first_bridge'] for r in missing),missing_unique_region=sum(r['unique_P0_region'] for r in missing),missing_P5_uncovered_region=sum(r['P5_region_uncovered'] for r in missing),missing_time_clusters=len(clusters),largest_missing_time_cluster=max(map(len,clusters)) if clusters else 0,shared_initializer_pair=list(pair(seed)),initializer_not_removed_in_any_planned_case=True,GT_role='retrospective cohort label only, not online feature or future policy selection')
    dump('missing_loop_summary.json',summary)
    plan=[dict(case=i+1,remove_robot1_kf=r['robot1_kf'],remove_robot3_kf=r['robot3_kf'],P0_loop_id=r['P0_loop_id'],baseline='full frozen P0 with same shared-edge initialization',same_graph_except_single_factor=True,initializer=list(pair(seed)),noise_and_prior='unchanged frozen policy',primary_GT_free_outcomes='heldout measured-loop residuals; fixed-map NN; same-gauge pose change; finite/anchor/jump checks',GT_outcomes='offline-only joint ATE and per-robot/union RPE after decisions; not critical-loop selection rule',status='PLANNED_NOT_RUN') for i,r in enumerate(missing)]
    csv('leave_one_out_plan.csv',plan)
    return rows,summary,local,loops

def linearized(rows,local,loops):
    import gtsam
    start=time.monotonic();key=lambda r,k:gtsam.symbol('a' if r=='robot1' else 'b',int(k));sig=np.array([np.deg2rad(5)]*3+[1.]*3)
    noise=gtsam.noiseModel.Diagonal.Sigmas(sig);robust=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),noise)
    graph=gtsam.NonlinearFactorGraph();graph.add(gtsam.PriorFactorPose3(key('robot1',150),gtsam.Pose3(local['robot1',150]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
    for r,start_id,end in [('robot1',150,1999),('robot3',234,4179)]:
        for k in range(start_id+1,end+1):graph.add(gtsam.BetweenFactorPose3(key(r,k-1),key(r,k),gtsam.Pose3(inverse(local[r,k-1])@local[r,k]),noise))
    for row in loops[P5].itertuples():graph.add(gtsam.BetweenFactorPose3(key('robot1',row.canonical_robot1_kf),key('robot3',row.canonical_robot3_kf),gtsam.Pose3(np.asarray(json.loads(row.canonical_transform))),robust))
    saved=pd.read_csv(B/'policies'/P5/'online_batch_trajectory.csv');values=gtsam.Values()
    for r in saved.itertuples():values.insert(key(r.robot_id,r.keyframe_id),gtsam.Pose3(pose(r)))
    assert values.size()==5796 and graph.size()==5795+len(loops[P5])
    marg=gtsam.Marginals(graph,values);p0={(int(r.canonical_robot1_kf),int(r.canonical_robot3_kf)):r for r in loops[P0].itertuples()};diagnostics=[]
    for row in rows:
        if row['membership']!='P0_ONLY' or not row['GT_positive']:continue
        if time.monotonic()-start>180:raise TimeoutError('180s sparse diagnostic safety bound; no parameter retuning')
        q,c=row['robot1_kf'],row['robot3_kf'];ks=[key('robot1',q),key('robot3',c)];joint=marg.jointMarginalCovariance(gtsam.KeyVector(ks));C=np.block([[joint.at(i,j) for j in ks] for i in ks])
        Hq=np.zeros((6,6),order='F');Hc=np.zeros((6,6),order='F');pred=values.atPose3(ks[0]).between(values.atPose3(ks[1]),Hq,Hc);Z=gtsam.Pose3(np.asarray(json.loads(p0[q,c].canonical_transform)));E=Z.inverse().compose(pred);e=gtsam.Pose3.Logmap(E);J=gtsam.Pose3.LogmapDerivative(E)@np.hstack([Hq,Hc]);W=J/sig[:,None];V=W@C@W.T;V=(V+V.T)/2;eig=np.linalg.eigvalsh(V);assert eig.min()>-1e-6
        eig=np.maximum(eig,0);z=float(np.linalg.norm(e/sig));w=min(1.,1.345/z) if z else 1.
        diagnostics.append(dict(robot1_kf=q,robot3_kf=c,nominal_DeltaI_nats=float(.5*np.log1p(eig).sum()),diagnostic_robust_DeltaI_nats=float(.5*np.log1p(w*eig).sum()),diagnostic_Huber_weight=w,whitened_residual_norm=z,cross_correlation_preserved=True,linearization='frozen P5 final batch poses, OFFLINE ONLY',measurement='already measured P0 factor; unavailable before new GICP, NOT an online predictor',physical_uncertainty_calibrated=False))
    csv('missing_loop_linearized_information.csv',diagnostics)
    return dict(status='COMPLETED_SPARSE_LINEARIZATION_ONLY',factors=graph.size(),nodes=values.size(),diagnosed_missing_true_factors=len(diagnostics),elapsed_s=time.monotonic()-start,peak_RSS_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,optimization_calls=0,full_dense_Hessian_created=False,online_selector_implemented=False)

def candidates():
    trace=pd.read_csv(A/'online_candidate_trace.csv');attempts=pd.read_csv(A/'online_gicp_attempts.csv');measured={(int(r.event_index),int(r.candidate_rank)):r for r in attempts.itertuples()}
    for r in attempts.itertuples():
        path=A/'attempts'/f'e{int(r.event_index):04d}_r{int(r.candidate_rank)}.json';receipt=read(path)
        assert receipt['measurement']['result_csv_sha256']==r.result_csv_sha256==sha(path.with_suffix('.csv'))
    actual=pd.read_csv(B/'gicp_actual_replay_timings.csv');reexecuted={(int(r.event_index),int(r.candidate_rank)) for r in actual.itertuples()};assert reexecuted<=set(measured);rows=[]
    for row in trace.itertuples():
        ident=(int(row.event_index),int(row.candidate_rank));m=measured.get(ident)
        rows.append(dict(event_index=ident[0],rank=ident[1],query_robot=row.query_robot,query_keyframe_id=int(row.query_keyframe_id),candidate_robot=row.candidate_robot,candidate_keyframe_id=int(row.candidate_keyframe_id),best_shift=int(row.best_shift),actual_Stage22A_receipt=m is not None,Stage22B_actual_reexecution=ident in reexecuted,only_original_historical_receipt_available=m is not None and ident not in reexecuted,never_measured=m is None,may_need_new_execution_if_Stage23_reorders=bool(m is None),Stage23_actual_execution=False,outcome_status='MEASURED_HISTORICAL' if m is not None else 'UNKNOWN_NOT_IMPUTED',receipt_path=str(A/'attempts'/f'e{ident[0]:04d}_r{ident[1]}.json') if m is not None else None))
    csv('candidate_measurement_coverage.csv',rows)
    return dict(total_candidates=len(rows),actual_historical_measured=len(measured),Stage22B_actual_reexecuted_unique=len(reexecuted),historical_receipt_only=sum(r['only_original_historical_receipt_available'] for r in rows),never_measured=sum(r['never_measured'] for r in rows),potential_new_execution_upper_bound=sum(r['never_measured'] for r in rows),Stage23_actual_registrations=0,not_inferred_from_SC_GT_or_neighbors=True)

def main():
    assert not OUT.exists(),'Do not overwrite any preflight output directory'
    OUT.mkdir(parents=True);t=time.monotonic();before=historical_digest();gitstatus=subprocess.check_output(['git','status','--short'],cwd=ROOT).decode()
    for rev in ['bd3a67859c9412c8aab8ed8da66d1cf96f77101e','c929fc55a9f42c149ce571205618da73db7d4bc5']:subprocess.run(['git','cat-file','-e',rev+'^{commit}'],cwd=ROOT,check=True)
    inputs=[A/'online_candidate_trace.csv',A/'online_gicp_attempts.csv',B/'receipt_input_index.csv',B/'gicp_actual_replay_timings.csv',B/'policy_comparison_full.csv',ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv']
    inputs += [B/'policies'/p/n for p in [P0,P5] for n in ['online_verified_loops.csv','online_batch_trajectory.csv','online_incremental_vs_batch.csv','online_map_manifest.json']]
    inputs += list((PROC/'full_2hz').glob('*/keyframes.csv'))+list(RAW.glob('main_campus/*/*gt_utm_poses.csv'))
    dump('stage23_input_manifest.json',dict(frozen_references={'Stage22A':'c929fc55a9f42c149ce571205618da73db7d4bc5','Stage22B':'bd3a67859c9412c8aab8ed8da66d1cf96f77101e'},inputs={str(p):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in inputs},historical_before=before,existing_git_status=gitstatus,branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT).decode().strip(),new_branch_created=False,reason='worktree dirty; preserve existing changes and do not activate conditional clean-worktree branch creation',allowed_new_paths=['src/cumulti/stage23/','outputs/cumulti_v1/23_information_aware/','docs/CUMULTI_STAGE23_INFORMATION_AWARE.md','scripts/run_stage23_information_aware.sh'],GT_role='offline eligibility and retrospective labels only',no_expensive_experiments=True))
    inventory,coverage=dataset();rows,missing,local,loops=loops_audit();registry=candidates();information=linearized(rows,local,loops)
    measured=pd.read_csv(B/'policies'/P0/'online_incremental_vs_batch.csv').iloc[0];N=missing['P0_true_factors_absent_P5']+1
    estimates=dict(status='MODELLED_PROJECTION_NOT_EXECUTED',solves=N,reference_measured_batch_ms=float(measured.batch_runtime_ms),solver_only_serial_seconds_1x=N*float(measured.batch_runtime_ms)/1000,solver_only_serial_seconds_10x=N*float(measured.batch_runtime_ms)/100,measured_sparse_preflight_peak_RSS_mib=information['peak_RSS_mib'],proposed_future_serial_RSS_cap_GiB=2,dense_Hessian_bytes_to_avoid=(5796*6)**2*8,model_excludes='map reprojection/NN/evaluation/I/O and nonlinear convergence variation; not a guaranteed runtime',proposed_compute_cap_seconds=1800,proposed_workers=1,authorization_status='PLANNED_ONLY_NOT_RUN')
    after=historical_digest();assert before==after,'Historical source/output mutation detected'
    report=dict(status='PHASE0_PREFLIGHT_COMPLETE',dataset_eligibility='NO_GO',eligibility_targets=dict(positive_queries_each_direction=100,both_progressing_positive_queries_each_direction=50,independent_development_and_final_test_required=True,targets_weakened=False),reason='Frozen stable R1/R3 causal direction/dual-progress coverage fails. R2/R4 is a recording-disjoint candidate but R4 has GT only and neither new stable-odometry prefix is validated. Same-campaign full-recording initialization positives do not establish usable independent development/test data.',candidate_development='existing exposed R1/R3 for exploratory retrospective diagnostics only; coverage fails for a confirmatory dev/test benchmark',candidate_final_test='R2/R4 is recording-disjoint but sensor/stability-unqualified; no qualified final test currently; a new independent route/recording with adequate causal overlap is preferred',missing_loop_diagnosis=missing,candidate_registry=registry,linearized_diagnostic=information,leave_one_out_cost_estimate=estimates,historical_stages_unchanged=before==after,historical_after=after,full_GICP_runs=0,full_pose_graph_optimization_runs=0,selector_implemented=False,NEXT_STEPS_modified=False,branch_created=False,elapsed_s=time.monotonic()-t,recommended_next_action='Stop algorithm promotion. Obtain independent multi-robot recording(s), audit eligibility with unchanged targets, then separately authorize protocol freeze/selector implementation and prospective development/test experiment.',technical_risks=['causal overlap depends on exact start normalization; not physically simultaneous recordings','GT exposure prevents relabeling old data as blind heldout','regional odometry bins depend on gauge/path drift; not physical coverage guarantee','actual current-loop transform/outcome unavailable before GICP; retrospective DeltaI is not a causal feature','unknown early-stopped ranks require new real GICP, not imputation','submodular guarantees do not automatically transfer to changing causal graphs, robust state-dependent weights and shared cloud/cache costs','R2 stable odometry and R4 sensor availability unresolved','P0/P5 regional/factor losses are net changes, not equivalent to query recall loss'])
    dump('stage23_preflight_report.json',report);print(json.dumps(report),flush=True)
if __name__=='__main__':main()
