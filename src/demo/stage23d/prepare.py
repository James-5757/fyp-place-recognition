"""Real frozen P0/P5 geometry; no best-fit transform or graph optimization."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'cumulti/stage23d'))
from common import *
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

def poses(df):
    a=np.tile(np.eye(4),(len(df),1,1));a[:,:3,:3]=Rotation.from_quat(df[['qx','qy','qz','qw']].to_numpy()).as_matrix();a[:,:3,3]=df[['tx','ty','tz']].to_numpy();return a
def voxel(p):
    _,ix=np.unique(np.floor(p/.5).astype(np.int64),axis=0,return_index=True);return p[np.sort(ix)]
def stat(x):return dict(n=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(np.max(x))) if len(x) else dict(n=0,median=None,p95=None,max=None)
def nn(a,b):
    if not len(a) or not len(b):return dict(status='UNAVAILABLE_EMPTY_REGION')
    d=np.r_[cKDTree(b).query(a)[0],cKDTree(a).query(b)[0]];overlap=d[d<5]
    return dict(status='GT_FREE_DIAGNOSTIC',all_points=stat(d),within5m=stat(overlap),within5m_fraction=float(len(overlap)/len(d)),interpretation='NN includes non-overlap; not literal alignment error or accuracy')
def ply(p):
    with p.open('rb') as h:
        header=b''
        while not header.endswith(b'end_header\n'):header+=h.readline()
        assert b'format binary_little_endian 1.0' in header
        return np.frombuffer(h.read(),dtype='<f4').reshape(-1,3)

def main():
    protect_verify();start=time.monotonic();manifest=read(MAP/'comparison_input_manifest.json');local=pd.read_csv(BASE/'12d_clean_backend/clean_local_trajectory.csv');local=local.sort_values(['robot_id','keyframe_id']);arc={}
    for r,f in local.groupby('robot_id'):
        xyz=f[['tx','ty','tz']].to_numpy();arc[r]={int(k):v for k,v in zip(f.keyframe_id,np.r_[0,np.linalg.norm(np.diff(xyz,axis=0),axis=1).cumsum()])}
    plans=read(OLD/'stage23b_policy.json')['cases'];regions=[]
    for c in plans:
        if c['kind']=='TEMPORAL_GROUP':regions.append(dict(name=c['name'],kind='TEMPORAL_CLUSTER',pairs=c['remove']))
    meta=pd.read_csv(OLD/'stage23b_GT_free_loop_metadata.csv')
    bins=meta[(meta.policy==P0)&meta.P5_region_uncovered][['R1_bin50','R3_bin50']].drop_duplicates().astype(int)
    assert len(regions)==7 and len(bins)==4
    for a,b in bins.itertuples(index=False,name=None):regions.append(dict(name=f'ROUTE_BIN_R1_{a}_R3_{b}',kind='ODOMETRY_ROUTE_BIN_PAIR_50M',bins=[a,b]))
    def membership(region,r,k):
        x=arc[r][int(k)];i=0 if r=='robot1' else 1
        return any(abs(x-arc[r][int(pair[i])])<=50 for pair in region['pairs']) if region['kind']=='TEMPORAL_CLUSTER' else int(x//50)==region['bins'][i]
    sources={};raw={}
    for identity,name in manifest['selected_scans'].items():
        r,k=identity.split(':');p=np.load(name)[::16,:3];p=p[np.isfinite(p).all(1)&(np.linalg.norm(p,axis=1)>1)&(np.linalg.norm(p,axis=1)<60)];raw[r,int(k)]=p
    assets=DEMO/'assets';assets.mkdir(exist_ok=True);states={};maps={};checks={};paired=[];regional=[];source_matched=[]
    loops={p:pd.read_csv(B/'policies'/p/'online_verified_loops.csv') for p in [P0,P5]}
    identities={p:set(zip(f.canonical_robot1_kf.astype(int),f.canonical_robot3_kf.astype(int))) for p,f in loops.items()}
    assert [len(identities[P0]),len(identities[P5]),len(identities[P0]&identities[P5]),len(identities[P0]-identities[P5]),len(identities[P5]-identities[P0])]==[288,231,147,141,84]
    for mode in ['incremental','batch']:
        trajectories={};transforms={}
        for label,p in [('P0',P0),('P5',P5)]:
            directory=B/'policies'/p;df=pd.read_csv(directory/f'online_{mode}_trajectory.csv',dtype={'timestamp':str}).sort_values(['robot_id','keyframe_id']).reset_index(drop=True);T=poses(df)
            assert len(df)==5796 and not df.duplicated(['robot_id','keyframe_id']).any() and np.isfinite(T).all()
            assert np.max(abs(np.linalg.norm(df[['qx','qy','qz','qw']].to_numpy(),axis=1)-1))<1e-8
            for r,f in df.groupby('robot_id'):assert np.all(np.diff(f.timestamp.map(int).to_numpy())>0)
            integ=read(directory/'online_graph_integrity.json');assert integ['nodes']==5796 and integ['odometry']==5794
            reference=poses(local[(local.robot_id=='robot1')&(local.keyframe_id==150)])[0]
            assert np.max(abs(T[0]-reference))<1e-6,'Gauge anchor differs; no alignment allowed'
            key=label+'_'+mode;checks[key]=dict(finite_proper_SE3=True,nodes=len(df),odometry=5794,anchor_matrix_max_abs_error=float(np.max(abs(T[0]-reference))),gauge='original robot1 KF150 LiDAR prior; metric XYZ, not ENU')
            trajectories[label]=df;transforms[label]=T;lookup={(r.robot_id,int(r.keyframe_id)):t for r,t in zip(df.itertuples(),T)}
            data=dict(trajectory={r:[[int(z.keyframe_id),float(z.tx),float(z.ty),float(z.tz)] for z in f.itertuples()] for r,f in df.groupby('robot_id')},regions={reg['name']:{r:[int(k) for k in f.keyframe_id if membership(reg,r,k)] for r,f in df.groupby('robot_id')} for reg in regions},loops=[[int(q),int(c),'COMMON' if (q,c) in identities[P0]&identities[P5] else label+'_ONLY'] for q,c in sorted(identities[p])],maps={})
            for r in ['robot1','robot3']:
                transformed={k:point@lookup[r,k][:3,:3].T+lookup[r,k][:3,3] for (robot,k),point in raw.items() if robot==r};v=voxel(np.vstack(list(transformed.values())));maps[key,r]=dict(all=v,regions={reg['name']:voxel(np.vstack([p for k,p in transformed.items() if membership(reg,r,k)])) if any(membership(reg,r,k) for k in transformed) else np.empty((0,3)) for reg in regions})
                if mode=='incremental':
                    expected=next(x for x in read(directory/'online_map_manifest.json')['artifacts'] if x['path'].endswith(r+'.ply'));assert sha(expected['path'])==expected['sha256'];old=ply(Path(expected['path']));assert old.shape==v.shape;assert np.allclose(old,v.astype('<f4'),atol=2e-5,rtol=0),'Raw scan reconstruction does not reproduce source PLY'
                chosen=np.linspace(0,len(v)-1,min(25000,len(v)),dtype=int);path=assets/(key+'_'+r+'.f32');v[chosen].astype('<f4').tofile(path);data['maps'][r]=dict(file='assets/'+path.name,points=len(chosen),full_voxel_points=len(v),sha256=sha(path),bytes=path.stat().st_size)
            states[key]=data
        assert trajectories['P0'][['robot_id','keyframe_id','timestamp']].equals(trajectories['P5'][['robot_id','keyframe_id','timestamp']])
        delta=np.linalg.norm(transforms['P0'][:,:3,3]-transforms['P5'][:,:3,3],axis=1);rot=np.rad2deg(Rotation.from_matrix(np.einsum('nji,njk->nik',transforms['P0'][:,:3,:3],transforms['P5'][:,:3,:3])).magnitude())
        for row,d,a in zip(trajectories['P0'].itertuples(),delta,rot):paired.append(dict(condition=mode,robot_id=row.robot_id,keyframe_id=int(row.keyframe_id),timestamp=row.timestamp,translation_difference_m=d,rotation_difference_deg=a,meaning='paired policy difference, NOT GT error'))
        for reg in [dict(name='ALL',kind='ALL')]+regions:
            for r in ['robot1','robot3']:
                f=trajectories['P0'];mask=(f.robot_id==r).to_numpy()
                if reg['name']!='ALL':mask &= np.array([membership(reg,z.robot_id,z.keyframe_id) for z in f.itertuples()])
                a=maps['P0_'+mode,r]['all'] if reg['name']=='ALL' else maps['P0_'+mode,r]['regions'][reg['name']];b=maps['P5_'+mode,r]['all'] if reg['name']=='ALL' else maps['P5_'+mode,r]['regions'][reg['name']]
                point_delta=[]
                for (robot,k),p in raw.items():
                    if robot!=r or (reg['name']!='ALL' and not membership(reg,r,k)):continue
                    ix=int(f.index[(f.robot_id==r)&(f.keyframe_id==k)][0]);t0,t5=transforms['P0'][ix],transforms['P5'][ix];point_delta.extend(np.linalg.norm(p@(t0[:3,:3]-t5[:3,:3]).T+t0[:3,3]-t5[:3,3],axis=1))
                regional.append(dict(condition=mode,region=reg['name'],region_kind=reg['kind'],robot_id=r,**{'pose_translation_'+k:v for k,v in stat(delta[mask]).items()},**{'pose_rotation_'+k:v for k,v in stat(rot[mask]).items()},**{'matched_raw_point_displacement_'+k:v for k,v in stat(np.array(point_delta)).items()},map_NN=json.dumps(nn(a,b))))
    consistency=[]
    for key in states:
        for reg in ['ALL']+[r['name'] for r in regions]:
            a=maps[key,'robot1']['all'] if reg=='ALL' else maps[key,'robot1']['regions'][reg];b=maps[key,'robot3']['all'] if reg=='ALL' else maps[key,'robot3']['regions'][reg];consistency.append(dict(state=key,region=reg,metric=nn(a,b)))
    offline=pd.read_csv(B/'offline_map_accuracy_by_policy.csv');quality=pd.read_csv(B/'policy_comparison_full.csv');cost=pd.read_csv(B/'policy_results_by_budget.csv');metrics={}
    for label,p in [('P0',P0),('P5',P5)]:
        c=cost[cost.policy==p].iloc[0];qr=quality[(quality.policy==p)&(quality.direction=='ALL')].iloc[0]
        metrics[label]=dict(loops=int(c.admitted),GICP_calls=int(c.attempts),application_bytes=int(c.application_bytes),geometric_precision=float(qr.precision),geometric_recall=float(qr.recall),offline=[{k:(None if pd.isna(v) else v) for k,v in row.items()} for row in offline[offline.policy==p].to_dict('records')],map_consistency=[x for x in consistency if x['state'].startswith(label+'_') and x['region']=='ALL'])
    allxyz=np.vstack([np.array(t)[:,1:4] for s in states.values() for t in s['trajectory'].values()]);bounds=[allxyz.min(0).tolist(),allxyz.max(0).tolist()]
    dump(DEMO/'data.json',dict(states=states,metrics=metrics,regions=regions,bounds=bounds,progressive_status='PROGRESSIVE_STATE_EXPORT_REQUIRED',notes=['separate-recording hypothetical dual-stream replay','no live network measurement; application bytes are protocol-simulated','original gauge XYZ in metres; not geographic ENU','policy pose differences are not GT errors','final states only; no progressive morph or artificial Z flattening']))
    csv(MAP/'paired_trajectory_difference.csv',paired);csv(MAP/'regional_comparison.csv',regional);dump(MAP/'source_matched_map_consistency.json',consistency)
    dump(MAP/'map_comparison_validation.json',dict(status='PASS',checks=checks,raw_LiDAR_reconstruction_matches_both_incremental_PLYs=True,shared_keyframe_ids_timestamps_raw_points=True,same_frame_stride_point_stride_voxel=True,independent_best_fit_or_GT_transform=False,XYZ_rescaled_or_flattened=False,loop_counts=[288,231,147,141,84],progressive='PROGRESSIVE_STATE_EXPORT_REQUIRED',display_cap_per_source=25000,complete_geometry_scope='full per-source voxel maps reconstructed on server; browser deterministic capped subsets',elapsed_s=time.monotonic()-start,peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024))
    dump(MAP/'demo_asset_manifest.json',{str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in DEMO.rglob('*') if p.is_file()})
    print('MAP_EXPORT_COMPLETE',time.monotonic()-start,flush=True)
if __name__=='__main__':main()
