"""Gate A. Audit source bytes first; write-once predeclarations before policy evaluation."""
import time
from common import *


def main():
    subprocess.run(['git','merge-base','--is-ancestor',BASE,'HEAD'],cwd=ROOT,check=True)
    # Reproduction after the Stage22B commit is allowed; scientific baseline code is not.
    frozen_sources=subprocess.check_output(['git','ls-tree','-r','--name-only',BASE,'src/cumulti'],cwd=ROOT,text=True).splitlines()
    for relative in frozen_sources:
        path=ROOT/relative
        if path.is_file():
            assert sha(path)==hashlib.sha256(subprocess.check_output(['git','show',BASE+':'+relative],cwd=ROOT)).hexdigest(),relative
    assert not OUT.exists(),'Never overwrite a Stage22B experiment'
    OUT.mkdir(parents=True)
    manifest=read(A/'stage22a_artifact_manifest.json')
    inputs={}
    for item in manifest['artifacts']:
        p=Path(item['path']);assert p.stat().st_size==item['bytes'] and sha(p)==item['sha256'],str(p)
        inputs[str(p)]=item['sha256']
    # Exact raw-file hashes, not just filenames or approximate wire sizes.
    lidar=pd.read_csv(A/'lidar_input_manifest.csv')
    for index,row in enumerate(lidar.itertuples()):
        assert sha(row.path)==row.sha256
        if index%1000==0:print('RAW_HASH',index,flush=True)
    old=read(A/'stage22a_input_manifest.json')
    for path,digest in old['inputs'].items():assert sha(path)==digest;inputs[path]=digest
    inputs[str(A/'stage22a_artifact_manifest.json')]=sha(A/'stage22a_artifact_manifest.json')
    for name in ['stage21_wire.py','stage22a_common.py','run_stage22a_verification.py','run_stage22a_backend.py']:
        p=SRC/name
        assert hashlib.sha256(subprocess.check_output(['git','show',BASE+':'+str(p.relative_to(ROOT))],cwd=ROOT)).hexdigest()==sha(p)
        inputs[str(p)]=sha(p)
    tracked=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().strip('\0').split('\0')
    protected={p:sha(ROOT/p) for p in tracked if (ROOT/p).is_file() and p.startswith(('src/','scripts/','outputs/','demo/'))}
    events,candidates,attempts=input_tables()
    assert len(events)==5796 and len(attempts)==15267 and int(attempts.geometrically_accepted.sum())==1088
    keys={(int(r.event_index),int(r.candidate_rank)) for r in attempts.itertuples()}
    raw_lookup={(r.robot,int(r.keyframe)):r.sha256 for r in lidar.itertuples()}
    receipt_index=[]
    directory_hash=hashlib.sha256()
    directory=Path(manifest['attempt_directory']['path'])
    for p in sorted(directory.iterdir()):
        if p.is_file():directory_hash.update((p.name+' '+sha(p)+'\n').encode())
    assert directory_hash.hexdigest()==manifest['attempt_directory']['sha256_of_sorted_filename_space_sha256_LF']
    for row in attempts.to_dict('records'):
        ident=f"e{int(row['event_index']):04d}_r{int(row['candidate_rank'])}"
        receipt=read(directory/(ident+'.json'));m=receipt['measurement']
        assert m['result_csv_sha256']==sha(directory/(ident+'.csv'))
        for name in ['event_index','candidate_rank','query_robot','query_keyframe_id','candidate_robot','candidate_keyframe_id','best_shift']:
            assert m[name]==row[name]
        assert abs(m['gicp_quality']-row['gicp_quality'])<1e-12
        assert np.max(abs(np.array(m['verified_transform'])-np.array(json.loads(row['verified_transform']))))<1e-12
        receipt_index.append(dict(event_index=row['event_index'],rank=row['candidate_rank'],receipt_sha256=sha(directory/(ident+'.json')),
            query_cloud_sha256=raw_lookup[(row['query_robot'],int(row['query_keyframe_id']))],
            candidate_cloud_sha256=raw_lookup[(row['candidate_robot'],int(row['candidate_keyframe_id']))]))
    csv(OUT/'receipt_input_index.csv',receipt_index)
    dev=candidates[candidates.event_index<1000]
    q05,q95=np.quantile(dev.SC_score,[.05,.95]).tolist()
    times=attempts[attempts.event_index<1000].groupby(['query_robot','candidate_rank']).call_cpu_ms.median()
    costs={r+':'+str(k):float(times.loc[r,k]) for r in ['robot1','robot3'] for k in [1,2,3]}
    write_json(OUT/FROZEN[0],dict(baseline_commit=BASE,freeze_unix_ns=time.time_ns(),inputs=inputs,
        raw_clouds_verified=len(lidar),raw_cloud_catalog_sha256=sha(A/'lidar_input_manifest.csv'),
        receipt_directory_verified=manifest['attempt_directory'],receipt_input_index_sha256=sha(OUT/'receipt_input_index.csv'),
        protected_worktree=protected,preexisting_dirty=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),
        source_commits={p:subprocess.check_output(['git','log','-1','--format=%H',BASE,'--',str(Path(p).relative_to(ROOT))],cwd=ROOT,text=True).strip()
                        for p in inputs if Path(p).is_relative_to(ROOT) and subprocess.run(['git','ls-files','--error-unmatch',str(Path(p).relative_to(ROOT))],cwd=ROOT,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0},
        scientist_has_historical_baseline_context=True,GT_read_for_policy=False))
    write_json(OUT/FROZEN[1],dict(
        policies=['P0_FULL_TOP3','P1_FIXED_TOP1','P2_FIXED_TOP2','P3_SC_CONFIDENCE_GATE','P4_CAUSAL_REDUNDANCY_GATE','P5_BUDGET_AWARE'],
        order='prefix of original attempted rank sequence; stop at first geometric acceptance or first denial',
        duplicate='Stage22A first admitted canonical pair within deltaR1<=3 AND deltaR3<=5; reconstruct for every policy',
        requests='B1 descriptor and candidate publication fixed; cloud requests only for actually selected pairs, query then candidate',
        cache='unbounded central logical point-cloud cache, independent of fresh teammate64 per-call physical cache',
        normalization=dict(SC_q05=q05,SC_q95=q95,clip=[0,1],margin_scale=.1,novelty_scale_KF=[30,50],
            transfer_scale_bytes=2*(1048632+49),cpu_scale_ms=float(np.median(list(costs.values())))),
        predicted_cpu_ms=costs,
        P3_top1_threshold=float(dev[dev.candidate_rank==1].SC_score.quantile(.20)),
        P4='skip entire query when rank1 canonical pair lies inside past admitted delta3/delta5 neighborhood',
        P5_weights=dict(confidence=1.0,margin=.1,novelty=.1,redundancy=.1,cpu=.05,transfer=.05),
        utility_threshold_grid=[0,.1,.2,.3,.4,.5],
        dev_selection='Only events0:1000 and measured geometric acceptance, no GT; retain >=98% of P0 geometrically accepted queries, then minimum calls then bytes then threshold. If none feasible select threshold0 and explicitly report failure.',
        main_candidate_compute_fraction=.75,main_candidate_communication_fraction=.8,
        finalist_rule='P0 and P1/P2/P3/P4 controls plus P5 at .75/.8; no heldout reselection',
        actual_reexecution_query_ids=np.linspace(0,5795,120,dtype=int).tolist(),
        actual_reexecution_policies=['P0_FULL_TOP3','P3_SC_CONFIDENCE_GATE','P5_BUDGET_AWARE_C75_B80'],
        benchmark_transform_tolerance=1e-6,benchmark_quality_tolerance=1e-9,
        benchmark_limit_seconds=1800,threads=1,physical_cache='fresh teammate64 per actual_call; no index optimization',
        feature_provenance='SC/rank/margin and opposite DB from current causal trace; cache from own past requests; novelty/redundancy/count from own earlier admitted loops; cost from development only; remaining budgets from own past decisions',
        no_future_postponement=True,cache_sensitivity=[None,1000,128],
        compression_sensitivity='Exact raw encoded lengths catalog; compressed bytes only for physically re-encoded messages. No header-invariant compression assumption.',
        evidence_type=REPLAY))
    write_json(OUT/FROZEN[2],dict(compute_fractions=[1,.75,.5,.25],communication_fractions=[1,.8,.6,.4],
        reference_calls=15267,reference_application_bytes=6107977604,
        cumulative='floor(fraction*same full Stage22A workload); calls exact cap, application bytes include fixed baseline publication',
        rolling='last100 events, floor(300*compute_fraction) calls; fixed publication plus cloud/loop bytes <= floor(100*baseline_bytes_per_event*communication_fraction)',
        per_query_calls=3,estimated_loop_reserve_bytes=249,
        predicted_cpu='reported separately, primary hard constraint uses calls not realized historical runtime',
        fixed_descriptor_publication=True,do_not_silently_exceed=True))
    write_json(OUT/FROZEN[3],dict(development=list(range(1000)),embargo=list(range(1000,1100)),heldout=list(range(1100,5796)),
        rule='single chronological cut,100-event embargo; state continues causally through all events; no random pair split; no label-based rebalance',
        limitation='Same route replay, not independent geography; early loop opportunities before cut are dev-only. Report heldout overlap coverage after frozen decisions.'))
    write_json(OUT/FROZEN[4],dict(attempt_reduction_min=.25,application_byte_reduction_min=.20,
        recall_drop_max=.02,precision_drop_max=.01,joint_ATE_ratio_max=1.05,
        no_new_catastrophic_jump=True,proper_factor_convention=True,
        same_graph_agreement=dict(translation_median_m=.10,translation_p95_m=.50,rotation_median_deg=.5,rotation_p95_deg=2.),
        assessed_scopes=['full','heldout'],quality_recall_denominator='causal queries with at least one already-arrived opposite GT frame within d_xy<5m',
        no_posthoc_relaxation=True,overall_ready_requires_all_gates=True))
    write_json(OUT/'policy_lock.json',dict(files={n:sha(OUT/n) for n in FROZEN},frozen_unix_ns=time.time_ns()))
    write_json(OUT/'counterfactual_receipt_coverage.json',dict(evidence_type=REPLAY,candidate_pairs=len(candidates),measured_pairs=len(keys),
        unknown_pairs=len(candidates)-len(keys),unknown_reason='Historical early stopping, never imputed',
        receipt_identity='event/rank, query/candidate IDs, direction, exact raw cloud hashes, frozen GICP source/config and inverse SE3 convention',
        all_actual_receipts_hash_verified=True,all_raw_clouds_hash_verified=True))
    write_json(OUT/'gate_A.json',dict(status='PASS',events=len(events),measured_pairs=len(keys),GT_used=False))
    print('GATE_A_PASS',flush=True)

if __name__=='__main__':main()
