"""Stage23E isolated ledger, unchanged Stage22A mathematics and GT separation."""
from common import *
import gzip
from scipy.spatial.transform import Rotation
G=BASE/'23e_p6_graphs';E=BASE/'23e_p6_evaluation';D=BASE/'23d_budget_scheduler'
POLICIES={'P6_D0':'P6_D0_IMMEDIATE_CONTROL','P6_D1':'P6_D1_DEFERRED_ONLY','P6_D2':'P6_D2_DEFERRED_COVERAGE','P6_D3':'P6_D3_DEFERRED_COVERAGE_DIRECTION'}
COUNTS={'P6_D0':246,'P6_D1':223,'P6_D2':215,'P6_D3':211}
EXPECTED='476a490b787a2912e61b9f1e46424990b0786008780c9f7eaca6713799d30bab'
def inv(T):
    a=np.eye(4);a[:3,:3]=T[:3,:3].T;a[:3,3]=-a[:3,:3]@T[:3,3];return a
def matrix(r):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([r.qx,r.qy,r.qz,r.qw]).as_matrix();T[:3,3]=[r.tx,r.ty,r.tz];return T
def proper(T):
    a=np.asarray(T);assert a.shape[-2:]==(4,4) and np.isfinite(a).all()
    assert np.max(abs(a[...,3,:]-[0,0,0,1]))<1e-8
    R=a[...,:3,:3];assert np.max(abs(np.swapaxes(R,-1,-2)@R-np.eye(3)))<1e-6 and np.max(abs(np.linalg.det(R)-1))<1e-6
def gt_free_guard():
    def hook(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            name=os.fsdecode(args[0]).lower()
            if '/src/cumulti/' in name and name.endswith(('.py','.pyc')):return
            if any(s in name for s in ['gt_utm','gt_rel','offline_','keyframes.csv','p6_graph_accuracy','p6_query_quality','p6_region_accuracy','stage23b_gt_free_loop_metadata','loop_set_exact_comparison']):raise PermissionError('GT unavailable in backend: '+name)
    sys.addaudithook(hook)
def verify():
    lock=read(G/'stage23e_input_manifest.json')
    repair=read(G/'stage23e_engineering_repair.json') if (G/'stage23e_engineering_repair.json').exists() else dict(source_repairs={})
    for path,v in lock['inputs'].items():
        expected=v['sha256']
        if path in repair['source_repairs']:
            assert path in [str(ROOT/'src/cumulti/stage23d'/n) for n in ['stage23e_common.py','run_stage23e_backend.py']]
            update=repair['source_repairs'][path];assert update['original_sha256']==expected;expected=update['corrected_sha256']
        assert sha(path)==expected,path
    assert sha(D/'p6_verified_factors.csv')==EXPECTED
def freeze():
    assert not G.exists() and not E.exists(),'Preserve prior Stage23E outputs'
    protect_verify()
    factors=D/'p6_verified_factors.csv';assert sha(factors)==EXPECTED and factors.stat().st_size==440808
    manifest=read(D/'stage23d_artifact_manifest.json');assert manifest[str(factors.relative_to(ROOT))]['sha256']==EXPECTED
    G.mkdir();E.mkdir()
    paths=set()
    for name,v in read(MAP/'comparison_input_manifest.json')['inputs'].items():paths.add(Path(name))
    paths.update(p for p in D.iterdir() if p.is_file());paths.update([MAP/'comparison_input_manifest.json',MAP/'historical_integrity_inventory.json.gz'])
    paths.update((ROOT/'src/cumulti').glob('*.py'));paths.update((ROOT/'src/cumulti/stage22b').glob('*.py'));paths.update((ROOT/'src/cumulti/stage23d').glob('*.py'))
    paths.update([ROOT/'outputs/cumulti_v1/12c_dataset_stability/stable_segment_selection.json',BASE/'12d_clean_backend/clean_gtsam_solver_policy.json',BASE/'13b_incremental_isam2/isam2_parameter_policy.json'])
    paths.update(ROOT/p for p in ['scripts/run_stage23e_p6_backend.sh','scripts/run_stage23e_evaluation.sh','PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md'])
    paths.update([A/'offline_causal_query_evaluation.csv',A/'offline_geometrically_accepted_including_duplicates_labels.csv',PROC/'robot1/keyframes.csv',PROC/'robot3/keyframes.csv'])
    for p in (B/'policies'/P0).glob('*'):
        if p.is_file():paths.add(p)
    for p in (B/'policies'/P5).glob('*'):
        if p.is_file():paths.add(p)
    for row in pd.read_csv(factors).itertuples():paths.add(Path(row.receipt));paths.add(Path(row.receipt).with_suffix('.csv'))
    inputs={str(p):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(paths) if p.is_file()}
    dump(G/'stage23e_input_manifest.json',dict(reference='2fb55ad08f56cd74198c175d7116462b84a89427',inputs=inputs,existing_dirty=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),factor_source_expected_sha256=EXPECTED,all_GT_files_only_hashed_opaquely_at_freeze=True))
    dump(G/'stage23e_policy.json',dict(order=['insertion_event','discovery_event','rank','robot1_kf','robot3_kf'],insertion_clock='insertion_event ONLY; discovery_event remains query identity',nodes=5796,odometry=5794,loops=COUNTS,factors={k:5795+v for k,v in COUNTS.items()},noise_sigmas=[float(np.deg2rad(5))]*3+[1.]*3,Huber=1.345,prior_sigma=1e-6,anchor='robot1 KF150',ISAM2='exact frozen Stage22A defaults; persistent; no empty updates',LM='exact frozen Stage22A/22B and Stage12D settings; Stage22D-compatible in brief refers to these existing files (no Stage22D solver exists)',new_GICP_calls=0,GT_before_all_four_graph_freeze=False,event_trajectory_format='server-only ZIP of float64 Nx4x4 NPY states for EVERY event; rows robot1 ascending then robot3 ascending; prefix IDs in event trace; pre-bridge states independently LOCAL_ONLY',map_protocol=read(MAP/'comparison_input_manifest.json')['map_protocol'],evaluation_label='RETROSPECTIVE_GT_EXPOSED_RECORDING',split=dict(dev=[0,999],embargo=[1000,1099],heldout=[1100,5795]),heldout_graph_metric_rule='same ONE joint full-trajectory alignment; subset only arrived event IDs; no region/robot/scope refitting',RPE_subset_rule='both interval10 endpoints inside the same requested scope/region; combined is union of robot samples',baseline_evaluation='READ FROZEN P0/P5 METRICS ONLY; unavailable scoped baseline metrics remain unavailable'))
    dump(G/'stage23e_policy_lock.json',dict(input_manifest_sha256=sha(G/'stage23e_input_manifest.json'),policy_sha256=sha(G/'stage23e_policy.json')))
    print('STAGE23E_INPUTS_FROZEN',len(inputs),flush=True)
def lock_verify():
    v=read(G/'stage23e_policy_lock.json');assert sha(G/'stage23e_policy.json')==v['policy_sha256'] and sha(G/'stage23e_input_manifest.json')==v['input_manifest_sha256'];verify()
def load_factors(short):
    events=pd.read_csv(A/'dual_stream_event_schedule.csv').to_dict('records');arrival={(r['robot_id'],int(r['keyframe_id'])):int(r['event_index']) for r in events};frame=pd.read_csv(BASE/'12d_clean_backend/clean_local_trajectory.csv').sort_values(['robot_id','keyframe_id']).reset_index(drop=True);local={(r.robot_id,int(r.keyframe_id)):matrix(r) for r in frame.itertuples()}
    factors=pd.read_csv(D/'p6_verified_factors.csv');factors=factors[factors.policy==POLICIES[short]].sort_values(['insertion_event','discovery_event','rank','robot1_kf','robot3_kf']).reset_index(drop=True)
    assert len(factors)==COUNTS[short] and not factors.duplicated(['robot1_kf','robot3_kf']).any()
    for r in factors.itertuples():
        q,c=('robot1',int(r.robot1_kf)),('robot3',int(r.robot3_kf));assert q in local and c in local;assert max(arrival[q],arrival[c])<=r.discovery_event<=r.insertion_event
        assert sha(r.receipt)==r.receipt_sha256;receipt=read(r.receipt)['measurement'];assert sha(Path(r.receipt).with_suffix('.csv'))==receipt['result_csv_sha256'];Z=np.asarray(json.loads(r.canonical_transform));proper(Z);np.testing.assert_array_equal(Z,np.asarray(receipt['canonical_transform']));assert receipt['geometrically_accepted'] and int(receipt['event_index'])==int(r.discovery_event) and int(receipt['candidate_rank'])==int(r.rank)
        assert int(receipt['canonical_robot1_kf'])==r.robot1_kf and int(receipt['canonical_robot3_kf'])==r.robot3_kf
    return factors,frame,local,events
if __name__=='__main__':freeze()
