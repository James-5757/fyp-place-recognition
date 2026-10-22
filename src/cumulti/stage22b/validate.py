"""Independent saved-artifact audit: no policy selection, no parameter fitting."""
from collections import OrderedDict,deque
from common import *


def main(causal_only=False):
    verify_lock();assert_source_unchanged()
    assert read(OUT/'stage21_schema_catalog_audit.json')['status']=='PASS'
    unit=read(OUT/'synthetic_unit_tests.json')
    assert unit['status']=='PASS' and unit['source_sha256']==sha(Path(__file__).parent/'test_stage22b.py')
    lock=read(OUT/'pre_gt_decision_lock.json');budget=read(OUT/'stage22b_budget_policy.json')
    original=pd.read_csv(A/'online_gicp_attempts.csv')
    reference={(int(r.event_index),int(r.candidate_rank)):r for r in original.itertuples()}
    groups={int(i):list(g.candidate_rank.astype(int)) for i,g in original.groupby('event_index')}
    events=pd.read_csv(A/'dual_stream_event_schedule.csv')
    arrival={(r.robot_id,int(r.keyframe_id)):int(r.event_index) for r in events.itertuples()}
    source_message=pd.read_csv(A/'online_protocol_message_trace.csv')
    fixed=source_message[~source_message.message_type.isin(['POINT_CLOUD_REQUEST','POINT_CLOUD_RESPONSE','LOOP_CONSTRAINT'])]
    fixed_total=int(fixed.serialized_total_bytes.sum());fixed_event=fixed.groupby('event_index').serialized_total_bytes.sum().to_dict()
    checks=0;audits=[]
    for name,contract in lock['decisions'].items():
        directory=OUT/'policies'/name
        assert sha(directory/'candidate_decision_trace.csv')==contract['decisions_sha256']
        assert sha(directory/'online_verified_loops.csv')==contract['loops_sha256']
        decisions=pd.read_csv(directory/'candidate_decision_trace.csv')
        selected=decisions[decisions.execute]
        attempts=pd.read_csv(directory/'selected_attempts.csv')
        assert list(zip(selected.event_index,selected.candidate_rank))==list(zip(attempts.event_index,attempts.candidate_rank))
        for event,group in attempts.groupby('event_index'):
            ranks=group.candidate_rank.astype(int).tolist()
            assert ranks==groups[int(event)][:len(ranks)],'Not measured original prefix'
            assert not group.geometrically_accepted.iloc[:-1].any(),'Continued after acceptance'
            checks+=1
        cache=OrderedDict();admitted=[];new_admitted=[];var=0;ledger=deque();count=0
        trace=selected.set_index(['event_index','candidate_rank'])
        features=pd.read_csv(directory/'causal_feature_table.csv').set_index(['event_index','candidate_rank'])
        for row in attempts.itertuples():
            e,k=int(row.event_index),int(row.candidate_rank);original_row=reference[e,k]
            assert row.query_robot==original_row.query_robot and row.candidate_robot==original_row.candidate_robot
            assert int(row.query_keyframe_id)==int(original_row.query_keyframe_id) and int(row.candidate_keyframe_id)==int(original_row.candidate_keyframe_id)
            assert row.geometrically_accepted==original_row.geometrically_accepted
            assert np.max(abs(np.array(json.loads(row.verified_transform))-np.array(json.loads(original_row.verified_transform))))<1e-12
            keys=[(row.query_robot,int(row.query_keyframe_id)),(row.candidate_robot,int(row.candidate_keyframe_id))]
            assert all(arrival[key]<=e for key in keys)
            feature=features.loc[e,k]
            assert bool(feature.query_cached)==(keys[0] in cache) and bool(feature.candidate_cached)==(keys[1] in cache)
            assert int(feature.accepted_loop_count)==len(admitted)
            assert bool(feature.feature_before_current_GICP)
            transfer=0;miss=0
            for key in keys:
                if key not in cache:transfer+=1048632+49;miss+=1;cache[key]=None
                else:cache.move_to_end(key)
            assert transfer==int(feature.estimated_transfer_bytes)==int(trace.loc[e,k].cloud_misses)*(1048632+49)
            assert miss==int(trace.loc[e,k].cloud_misses)
            q,c=int(row.canonical_robot1_kf),int(row.canonical_robot3_kf)
            expected_q,expected_c=(int(row.query_keyframe_id),int(row.candidate_keyframe_id)) if row.query_robot=='robot1' else (int(row.candidate_keyframe_id),int(row.query_keyframe_id))
            assert (q,c)==(expected_q,expected_c)
            actual=np.array(json.loads(row.verified_transform));canonical=np.array(json.loads(row.canonical_transform))
            assert np.max(abs(canonical-(actual if row.query_robot=='robot1' else inv(actual))))<1e-10
            dup=any(abs(q-a)<=3 and abs(c-b)<=5 for a,b in admitted)
            assert bool(feature.redundancy)==dup
            extra=transfer
            if row.geometrically_accepted and not dup:
                admitted.append((q,c));new_admitted.append((e,q,c));extra+=249
            assert int(trace.loc[e,k].variable_bytes)==extra
            var+=extra;count+=1
            while ledger and ledger[0][0]<=e-100:ledger.popleft()
            ledger.append((e,1,extra))
            if name.startswith('P5'):
                result=contract['result'];compute=result['compute_fraction'];comm=result['communication_fraction']
                assert count<=int(np.floor(15267*compute))
                assert fixed_total+var<=int(np.floor(6107977604*comm))
                assert sum(x[1] for x in ledger)<=int(np.floor(300*compute))
                assert sum(x[2] for x in ledger)+sum(fixed_event.get(i,0) for i in range(max(0,e-99),e+1))<=int(np.floor(100*6107977604/5796*comm))
            checks+=12
        loops=pd.read_csv(directory/'online_verified_loops.csv')
        assert new_admitted==[(int(r.discovery_event),int(r.canonical_robot1_kf),int(r.canonical_robot3_kf)) for r in loops.itertuples()]
        assert var+fixed_total==contract['result']['application_bytes']
        assert count==contract['result']['attempts']
        if (directory/'online_graph_integrity.json').exists():
            graph=read(directory/'online_graph_integrity.json')
            assert graph['status']=='PASS'
            assert graph['nodes']==5796 and graph['odometry']==5794 and graph['prior']==1
            assert graph['loops']==len(loops) and graph['factors']==5795+len(loops)
            assert graph['exact_once'] and graph['persistent_ISAM2']
            backend=pd.read_csv(directory/'online_backend_event_trace.csv')
            assert backend.event_index.tolist()==list(range(5796)) and backend.nodes.tolist()==list(range(1,5797))
            assert int(backend.new_loops.sum())==len(loops)
            cfg=read(directory/'online_backend_policy.json')
            assert cfg['extra_empty_updates']==0 and cfg['Huber']==1.345 and cfg['prior_sigma']==1e-6
            np.testing.assert_allclose(cfg['noise_sigmas'],[np.deg2rad(5)]*3+[1.]*3,atol=1e-15)
            numeric=pd.read_csv(directory/'online_incremental_vs_batch.csv').iloc[0]
            gates=read(OUT/'stage22b_predeclared_acceptance.json')['same_graph_agreement']
            expected='PASS' if all(numeric[k]<=v for k,v in gates.items()) else 'FAIL'
            assert numeric.status==expected and numeric.same_new_graph and numeric.same_gauge
            assert np.isfinite(numeric.incremental_error) and np.isfinite(numeric.batch_error)
            checks+=25
        audits.append(dict(policy=name,status='PASS',calls=count,loops=len(loops),measured_prefix=True,
            causal_cache_and_admission=True,budgets_verified=name.startswith('P5')))
    sensitivity=pd.read_csv(OUT/'cache_sensitivity.csv')
    for item in read(OUT/'cache_sensitivity_selected_pairs.json').values():
        cache=OrderedDict();misses=0;hits=0;admitted=[];variable=0;ledger=deque();capacity=item['capacity']
        report=sensitivity[(sensitivity.policy==item['policy'])&(sensitivity.cache_capacity.astype(str)==('unbounded' if capacity is None else str(capacity)))].iloc[0]
        ranks_by_event={}
        for count,(e,k) in enumerate(item['pairs'],start=1):
            r=reference[e,k];extra=0;ranks_by_event.setdefault(e,[]).append(k)
            for key in [(r.query_robot,int(r.query_keyframe_id)),(r.candidate_robot,int(r.candidate_keyframe_id))]:
                assert arrival[key]<=e
                if key in cache:hits+=1;cache.move_to_end(key)
                else:
                    misses+=1;extra+=1048681;cache[key]=None
                    if capacity is not None and len(cache)>capacity:cache.popitem(last=False)
            q,c=int(r.canonical_robot1_kf),int(r.canonical_robot3_kf)
            if r.geometrically_accepted and not any(abs(q-a)<=3 and abs(c-b)<=5 for a,b in admitted):
                admitted.append((q,c));extra+=249
            variable+=extra
            while ledger and ledger[0][0]<=e-100:ledger.popleft()
            ledger.append((e,extra))
            if item['policy'].startswith('P5'):
                assert count<=int(np.floor(15267*.75)) and fixed_total+variable<=int(np.floor(6107977604*.8))
                assert len(ledger)<=225
                assert sum(x[1] for x in ledger)+sum(fixed_event.get(i,0) for i in range(max(0,e-99),e+1))<=int(np.floor(100*6107977604/5796*.8))
        assert all(ranks==groups[e][:len(ranks)] for e,ranks in ranks_by_event.items())
        assert len(item['pairs'])==int(report.attempts) and misses==int(report.cloud_cache_misses) and hits==int(report.cloud_cache_hits)
        assert len(admitted)==int(report.admitted) and fixed_total+variable==int(report.application_bytes)
        checks+=len(item['pairs'])*4
    actual=pd.read_csv(OUT/'gicp_actual_replay_timings.csv')
    assert actual.acceptance_equal.all() and actual.transform_max_abs_error.max()<=1e-6 and actual.quality_abs_error.max()<=1e-9
    assert not actual.duplicated(['policy','event_index','candidate_rank']).any()
    if not causal_only:assert read(OUT/'communication_validation.json')['status']=='PASS'
    # Validate exact P0 numerical reproduction, not just factors or similar ATE.
    baseline=OUT/'policies/P0_FULL_TOP3'
    old=pd.read_csv(A/'online_incremental_trajectory.csv')
    new=pd.read_csv(baseline/'online_incremental_trajectory.csv')
    assert list(old.columns)==list(new.columns) and len(old)==len(new)==5796
    numeric=old.select_dtypes(include='number').columns
    assert np.max(abs(old[numeric].to_numpy()-new[numeric].to_numpy()))<1e-7
    receipt_dir=read(OUT/'stage22b_input_manifest.json')['receipt_directory_verified']
    digest=hashlib.sha256()
    for path in sorted(Path(receipt_dir['path']).iterdir()):
        if path.is_file():digest.update((path.name+' '+sha(path)+'\n').encode())
    assert digest.hexdigest()==receipt_dir['sha256_of_sorted_filename_space_sha256_LF']
    raw=pd.read_csv(A/'lidar_input_manifest.csv')
    for row in raw.itertuples():assert sha(row.path)==row.sha256
    checks+=len(raw)+receipt_dir['files']
    audit=dict(status='PASS',checks=checks,policies=len(audits),baseline_reproduction='EXACT_RECEIPTS_AND_NUMERICAL_TRAJECTORY',
        actual_benchmark_equivalence=True,historical_inputs_unchanged=True,GT_not_used_for_selection=True,
        no_unknown_outcome_imputation=True,posthoc_policy_changes=False,policy_audits=audits,
        feature_metadata_caveat='latest_state_event is prior-event watermark, not a full state freshness timestamp; cache state may include earlier verified ranks of the same event. Exact per-rank cache reconstruction independently passes.',
        physical_network='NOT_MEASURED',scope='Artifact/causality correctness, NOT a blanket quality or resource-goal PASS')
    if causal_only:
        audit['scope']='Causal/factor/cache audit only; full communication validation remains pending'
    write_json(OUT/('stage22b_causal_validation.json' if causal_only else 'stage22b_independent_validation.json'),audit)
    print('INDEPENDENT_AUDIT_PASS',checks,flush=True)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--causal-only',action='store_true')
    main(parser.parse_args().causal_only)
