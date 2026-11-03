"""Independent factor/event/objective/archive verification, never reruns solvers."""
from stage23e_common import *
import zipfile,io
import gtsam

def main():
    lock_verify();frozen=read(G/'all_graphs_pre_gt_freeze.json');entry=read(E/'offline_evaluation_entry.json');assert entry['pre_gt_freeze_sha256']==sha(G/'all_graphs_pre_gt_freeze.json') and frozen['frozen_unix_ns']<entry['GT_entry_unix_ns']
    assert entry['backend_source_sha256']==sha(ROOT/'src/cumulti/stage23d/run_stage23e_backend.py')
    if entry['engineering_repair_sha256'] is not None:assert entry['engineering_repair_sha256']==sha(G/'stage23e_engineering_repair.json')
    for name,v in frozen['files'].items():assert sha(name)==v['sha256'],name
    for name,v in read(E/'stage23e_evaluation_artifact_manifest.json').items():assert sha(E/name)==v['sha256'],name
    gt_free_guard();results=[];sigmas=np.array([np.deg2rad(5)]*3+[1.]*3);base=gtsam.noiseModel.Diagonal.Sigmas(sigmas);robust=gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),base);key=lambda n:gtsam.symbol('a' if n[0]=='robot1' else 'b',int(n[1]))
    for short in POLICIES:
        directory=G/short;record=read(directory/'graph_integrity.json');loops,frame,local,events=load_factors(short)
        if record['status']!='PASS':results.append(dict(policy=short,status='FAILED_GRAPH_RETAINED',reason=record['reason']));continue
        trace=pd.read_csv(directory/'online_backend_event_trace.csv');ledger=pd.read_csv(directory/'graph_factor_ledger.csv');assert len(trace)==5796 and trace.event_index.tolist()==list(range(5796))
        first=loops.iloc[0];first_i=int(first.insertion_event);groups={int(i):f for i,f in loops.groupby('insertion_event')};arrived=set();graph=gtsam.NonlinearFactorGraph();expected=[];inserted=0
        def add(f,kind,i,identity):graph.add(f);expected.append((kind,i,json.dumps(list(f.keys())),identity))
        def odom(n):return gtsam.BetweenFactorPose3(key((n[0],n[1]-1)),key(n),gtsam.Pose3(inv(local[n[0],n[1]-1])@local[n]),base)
        for event in events:
            i=int(event['event_index']);node=(event['robot_id'],int(event['keyframe_id']));arrived.add(node)
            if i==first_i:
                add(gtsam.PriorFactorPose3(key(('robot1',150)),gtsam.Pose3(local['robot1',150]),gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))),'PRIOR',i,'robot1:150')
                for n in sorted(arrived):
                    if n[1]>(150 if n[0]=='robot1' else 234):add(odom(n),'ODOMETRY',i,str(n))
            elif i>first_i:add(odom(node),'ODOMETRY',i,str(node))
            for row in groups.get(i,pd.DataFrame()).itertuples():
                q,c=('robot1',int(row.robot1_kf)),('robot3',int(row.robot3_kf));assert q in arrived and c in arrived;identity=(int(row.discovery_event),int(row.rank),q[1],c[1]);add(gtsam.BetweenFactorPose3(key(q),key(c),gtsam.Pose3(np.array(json.loads(row.canonical_transform))),robust),'LOOP',i,json.dumps(identity));inserted+=1
            rec=trace.iloc[i];assert rec.nodes==i+1 and rec.loops==int((loops.insertion_event<=i).sum()) and rec.factors==(i+inserted if i>=first_i else 0)
        assert graph.size()==5795+COUNTS[short] and inserted==COUNTS[short]
        observed=list(zip(ledger.kind,ledger.insertion_event,ledger['keys'],ledger.identity));assert observed==expected
        keys={int(k) for j in range(graph.size()) for k in graph.at(j).keys()};parent={k:k for k in keys}
        def root(x):
            while parent[x]!=x:parent[x]=parent[parent[x]];x=parent[x]
            return x
        for j in range(graph.size()):
            ks=list(graph.at(j).keys())
            if len(ks)==2:parent[root(ks[0])]=root(ks[1])
        assert len(parent)==5796 and len({root(k) for k in parent})==1
        trajectories={};values={};agreement=pd.read_csv(directory/'online_incremental_vs_batch.csv').iloc[0]
        for condition in ['incremental','batch']:
            tr=pd.read_csv(directory/f'online_{condition}_trajectory.csv');assert len(tr)==5796 and tr[['robot_id','keyframe_id','timestamp']].equals(frame[['robot_id','keyframe_id','timestamp']]);matrices={(r.robot_id,int(r.keyframe_id)):matrix(r) for r in tr.itertuples()};proper(np.array(list(matrices.values())));v=gtsam.Values()
            for n,t in matrices.items():v.insert(key(n),gtsam.Pose3(t))
            val=float(graph.error(v));original=float(agreement.incremental_error if condition=='incremental' else agreement.batch_error);assert np.isclose(val,original,atol=1e-7,rtol=1e-12),(short,condition,val,original);trajectories[condition]=matrices;values[condition]=val
        inc,bat=trajectories['incremental'],trajectories['batch'];ds=np.array([np.linalg.norm(inc[n][:3,3]-bat[n][:3,3]) for n in inc]);rs=np.array([np.rad2deg(Rotation.from_matrix(inc[n][:3,:3].T@bat[n][:3,:3]).magnitude()) for n in inc]);diff=dict(translation_median_m=float(np.median(ds)),translation_p95_m=float(np.percentile(ds,95)),rotation_median_deg=float(np.median(rs)),rotation_p95_deg=float(np.percentile(rs,95)))
        for k,v in diff.items():assert np.isclose(v,float(agreement[k]),atol=1e-7,rtol=1e-10)
        gates=read(B/'stage22b_predeclared_acceptance.json')['same_graph_agreement'];assert (agreement.status=='PASS')==all(diff[k]<=v for k,v in gates.items())
        archive=read(directory/'event_trajectory_manifest.json');index={x['event_index']:x for x in archive['state_index']};assert len(index)==5796;sample=sorted({0,first_i-1,first_i,5795,*range(0,5796,500),*loops.insertion_event.astype(int).tolist()})
        with zipfile.ZipFile(directory/'event_trajectories_float64.zip') as z:
            assert z.testzip() is None and len(z.namelist())==5796
            for i in sample:
                payload=z.read(f'event_{i:06d}.npy');assert hashlib.sha256(payload).hexdigest()==index[i]['sha256'];a=np.load(io.BytesIO(payload),allow_pickle=False);proper(a);nodes=sorted((e['robot_id'],int(e['keyframe_id'])) for e in events[:i+1]);assert a.shape==(i+1,4,4)
                if i<first_i:np.testing.assert_allclose(a,np.array([local[n] for n in nodes]),atol=1e-12,rtol=0)
                else:assert np.max(abs(a[0]-local['robot1',150]))<1e-6
                if i==5795:np.testing.assert_allclose(a,np.array([inc[n] for n in nodes]),atol=1e-7,rtol=0)
        maps=read(directory/'online_map_manifest.json');assert maps['source_scan_count']==580 and maps['frame_stride']==10 and maps['point_stride']==16 and maps['voxel_m']==.5 and not maps['GT_used'];assert len(maps['artifacts'])==6
        for item in maps['artifacts']:assert sha(item['path'])==item['sha256'] and Path(item['path']).stat().st_size==item['bytes']
        results.append(dict(policy=short,status='PASS',nodes=5796,odometry=5794,loops=COUNTS[short],factors=graph.size(),components=1,ledger_exact=True,delayed_factors=int((loops.insertion_event>loops.discovery_event).sum()),insertion_event_backdating_absent=True,objectives_recomputed_without_solving=values,archive_crc_all_events=True,archive_states=5796,independently_inspected_state_matrices=len(sample),archive_matrix_inspection='all insertion events plus before/at bridge, periodic500 and FINAL; backend runtime checks all events',maps_hashes_match=True))
    status='PASS' if all(r['status']=='PASS' for r in results) else 'PARTIAL_FAILED_GRAPHS_RETAINED'
    dump(E/'stage23e_independent_artifact_validation.json',dict(status=status,policies=results,all_graphs_frozen_before_GT=True,new_GICP_calls=0,no_optimizer_reruns=True,independent_scientific_validation='NOT_RUN',scope='artifact and mathematics audit on exposed historical recording, not geographic generalization'))
    print('STAGE23E_INDEPENDENT_ARTIFACT_AUDIT',status,flush=True)
if __name__=='__main__':main()
