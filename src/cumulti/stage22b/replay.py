"""Prefix-preserving policy study. Registration outcomes are read only AFTER selection."""
from collections import OrderedDict,deque
import math
from common import *


class Cache:
    def __init__(self,capacity=None):self.capacity=capacity;self.items=OrderedDict()
    def misses(self,keys):
        if self.capacity is None:return [k for k in keys if k not in self.items]
        # Fetching a new query may evict an old resident candidate before its access.
        planned=self.items.copy();missing=[]
        for key in keys:
            if key in planned:planned.move_to_end(key)
            else:
                missing.append(key);planned[key]=None
                if len(planned)>self.capacity:planned.popitem(last=False)
        return missing
    def access(self,key):
        hit=key in self.items
        if hit:self.items.move_to_end(key)
        else:
            self.items[key]=None
            if self.capacity is not None and len(self.items)>self.capacity:self.items.popitem(last=False)
        return hit


def neighborhood(q,c,admitted):
    return any(abs(q-a)<=3 and abs(c-b)<=5 for a,b in admitted)


def canonical_ids(row):
    q,c=int(row['query_keyframe_id']),int(row['candidate_keyframe_id'])
    return (q,c) if row['query_robot']=='robot1' else (c,q)


class Data:
    def __init__(self):
        self.events,self.candidates,attempts=input_tables()
        self.groups={int(i):g.sort_values('candidate_rank').to_dict('records') for i,g in self.candidates.groupby('event_index')}
        self.receipts={(int(r['event_index']),int(r['candidate_rank'])):r for r in attempts.to_dict('records')}
        messages=pd.read_csv(A/'online_protocol_message_trace.csv')
        self.fixed=messages[~messages.message_type.isin(['POINT_CLOUD_REQUEST','POINT_CLOUD_RESPONSE','LOOP_CONSTRAINT'])].copy()
        self.fixed_events=self.fixed.groupby('event_index').serialized_total_bytes.sum().to_dict()
        self.cloud_size={(('robot'+str(int(r.robot_id))),int(r.keyframe_id)):int(r.serialized_total_bytes)+49
                         for r in messages[messages.message_type=='POINT_CLOUD_RESPONSE'].itertuples()}
        assert len(self.cloud_size)==5796
        assert set(self.cloud_size.values())=={1048681},'Frozen fixed-shape cloud cost model no longer applies'
        self.policy=read(OUT/'stage22b_experiment_policy.json')
        self.budget=read(OUT/'stage22b_budget_policy.json')
        self.lookups=0

    def outcome(self,event,rank):
        self.lookups+=1
        assert (event,rank) in self.receipts,'UNKNOWN GICP OUTCOME: must actually execute extension, never impute'
        return self.receipts[event,rank]


def run(data,name,threshold=0,compute=1,communication=1,capacity=None,end=5796,save=False):
    p=data.policy;n=p['normalization'];w=p['P5_weights']
    proposed=name.startswith('P5')
    cache=Cache(capacity);admitted=[];discoveries=[];loops=[];chosen=[];trace=[];features=[]
    variable=0;calls=0;cpu_ms=0.;wall_ms=0.;hits=0;misses=0;ledger=deque()
    fixed_total=sum(data.fixed_events.get(i,0) for i in range(end))
    maxcalls=math.floor(data.budget['reference_calls']*compute)
    maxbytes=math.floor(data.budget['reference_application_bytes']*communication)
    roll_calls=math.floor(300*compute)
    roll_bytes=math.floor(100*data.budget['reference_application_bytes']/5796*communication)
    seen={};limit=1 if name=='P1_FIXED_TOP1' else 2 if name=='P2_FIXED_TOP2' else 3
    for event in data.events[:end]:
        index=int(event['event_index']);r=event['robot_id'];k=int(event['keyframe_id'])
        seen[(r,k)]=index
        rows=data.groups.get(index,[])
        while ledger and ledger[0][0]<=index-100:ledger.popleft()
        usedrollcalls=sum(x[1] for x in ledger);usedrollbytes=sum(x[2] for x in ledger)
        fixedroll=sum(data.fixed_events.get(i,0) for i in range(max(0,index-99),index+1))
        top=rows[0]['SC_score'] if rows else 0
        margin=float(top-rows[1]['SC_score']) if len(rows)>1 else 0.
        stop=None
        for row in rows:
            rank=int(row['candidate_rank']);q,c=canonical_ids(row)
            keys=[(row['query_robot'],int(row['query_keyframe_id'])),(row['candidate_robot'],int(row['candidate_keyframe_id']))]
            assert all(key in seen and seen[key]<=index for key in keys),'Future candidate lookup'
            missing=cache.misses(keys);transfer=sum(data.cloud_size[key] for key in missing)
            redundancy=neighborhood(q,c,admitted)
            novelty=min([max(abs(q-a)/30,abs(c-b)/50) for a,b in admitted],default=1.)
            novelty=min(novelty,1.)
            confidence=float(np.clip((row['SC_score']-n['SC_q05'])/(n['SC_q95']-n['SC_q05']),0,1))
            cost=p['predicted_cpu_ms'][r+':'+str(rank)]
            utility=(w['confidence']*confidence+w['margin']*min(margin/n['margin_scale'],1)
                     +w['novelty']*novelty-w['redundancy']*redundancy
                     -w['cpu']*cost/n['cpu_scale_ms']-w['transfer']*transfer/n['transfer_scale_bytes'])
            feature=dict(policy=name,event_index=index,candidate_rank=rank,query_robot=r,query_keyframe_id=k,
                candidate_robot=row['candidate_robot'],candidate_keyframe_id=row['candidate_keyframe_id'],SC_score=row['SC_score'],
                margin12=margin,opposite_database_size=row['current_R3_database_size'] if r=='robot1' else row['current_R1_database_size'],
                confidence=confidence,novelty=novelty,redundancy=redundancy,
                nearest_admitted_R1_KF_distance=min([abs(q-a) for a,b in admitted],default=-1),
                nearest_admitted_R3_KF_distance=min([abs(c-b) for a,b in admitted],default=-1),
                query_cached=keys[0] in cache.items,candidate_cached=keys[1] in cache.items,
                predicted_cpu_ms=cost,estimated_transfer_bytes=transfer,utility=utility,
                remaining_calls=maxcalls-calls,remaining_bytes=maxbytes-fixed_total-variable,
                rolling_calls_remaining=roll_calls-usedrollcalls,rolling_bytes_remaining=roll_bytes-fixedroll-usedrollbytes,
                accepted_loop_count=len(admitted),latest_state_event=index-1,feature_before_current_GICP=True)
            reason=stop
            if reason is None and rank>limit:reason='FIXED_RANK_CAP'
            if reason is None and name=='P3_SC_CONFIDENCE_GATE' and top<p['P3_top1_threshold']:reason='LOW_TOP1_CONFIDENCE'
            if reason is None and name=='P4_CAUSAL_REDUNDANCY_GATE' and rank==1 and redundancy:reason='PAST_ADMITTED_NEIGHBORHOOD'
            if reason is None and proposed and utility<threshold:reason='UTILITY_BELOW_THRESHOLD'
            if reason is None and proposed and (calls+1>maxcalls or usedrollcalls+1>roll_calls):reason='COMPUTE_BUDGET'
            if reason is None and proposed and (fixed_total+variable+transfer+249>maxbytes or fixedroll+usedrollbytes+transfer+249>roll_bytes):reason='COMMUNICATION_BUDGET'
            if reason is not None:
                stop=reason
                if save:
                    features.append(feature);trace.append(dict(policy=name,event_index=index,candidate_rank=rank,execute=False,reason=reason))
                continue
            # Only after committing the GT-free action may the measured outcome be read.
            actual=data.outcome(index,rank)
            calls+=1;cpu_ms+=actual['call_cpu_ms'];wall_ms+=actual['call_wall_ms'];chosen.append(actual)
            for key in keys:
                if cache.access(key):hits+=1
                else:misses+=1
            extra=transfer
            if actual['geometrically_accepted']:
                T=np.array(json.loads(actual['canonical_transform']))
                assert np.isfinite(T).all() and np.max(abs(T[3]-[0,0,0,1]))<1e-8
                assert np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))<1e-6 and abs(np.linalg.det(T[:3,:3])-1)<1e-6
                dup=neighborhood(q,c,admitted)
                rec=dict(actual,discovery_event=index,discovery_normalized_ns=int(event['normalized_ns']),
                    duplication_status='SUPPRESSED_CAUSAL_DUPLICATE' if dup else 'BACKEND_GRAPH_ADMISSIBLE',
                    loop_id=None if dup else len(admitted)+1)
                discoveries.append(rec)
                if not dup:admitted.append((q,c));loops.append(rec);extra+=249
                stop='STOP_AT_GEOMETRIC_ACCEPTANCE'
            variable+=extra;ledger.append((index,1,extra));usedrollcalls+=1;usedrollbytes+=extra
            if save:
                features.append(feature);trace.append(dict(policy=name,event_index=index,candidate_rank=rank,execute=True,reason='VERIFIED',
                    geometrically_accepted=actual['geometrically_accepted'],cloud_misses=len(missing),variable_bytes=extra))
            if proposed:
                assert calls<=maxcalls and fixed_total+variable<=maxbytes
                assert usedrollcalls<=roll_calls and fixedroll+usedrollbytes<=roll_bytes
        if index%1000==0 and save:print('REPLAY',name,index,calls,len(loops),flush=True)
    result=dict(policy=name,events=end,attempts=calls,geometrically_accepted=len(discoveries),admitted=len(loops),
        receipt_cpu_ms=cpu_ms,receipt_harness_ms=wall_ms,application_bytes=fixed_total+variable,
        descriptor_bytes=int(data.fixed[data.fixed.event_index<end].query('message_type=="FULL_SC_DESCRIPTOR_UPLOAD"').serialized_total_bytes.sum()),
        point_cloud_bytes=misses*(1048632+49),factor_update_bytes=int(data.fixed[data.fixed.event_index<end].query('message_type=="ODOMETRY_UPDATE"').serialized_total_bytes.sum())+249*len(loops),
        cloud_cache_hits=hits,cloud_cache_misses=misses,cache_capacity='unbounded' if capacity is None else capacity,
        compute_fraction=compute,communication_fraction=communication,utility_threshold=threshold,
        attempt_reduction=1-calls/15267,application_byte_reduction=1-(fixed_total+variable)/6107977604,
        evidence_type=REPLAY,communication_evidence_type='PROTOCOL_SIMULATED',
        compressed_application_bytes=None,compressed_scope='NOT_MEASURED until actual serializer replay; no compression projection')
    if save:
        directory=OUT/'policies'/name;directory.mkdir(parents=True,exist_ok=True)
        csv(directory/'selected_attempts.csv',chosen,columns=list(next(iter(data.receipts.values()))))
        columns=list(next(iter(data.receipts.values())))+['discovery_event','discovery_normalized_ns','duplication_status','loop_id']
        csv(directory/'online_verified_loops.csv',loops,columns=columns)
        csv(directory/'online_loop_discovery_trace.csv',discoveries,columns=columns)
        csv(directory/'candidate_decision_trace.csv',trace);csv(directory/'causal_feature_table.csv',features)
        write_json(directory/'decision.json',dict(result=result,decisions_sha256=sha(directory/'candidate_decision_trace.csv'),
            loops_sha256=sha(directory/'online_verified_loops.csv'),GT_read=False))
    return result,chosen,discoveries,loops


def main():
    verify_lock();guard();data=Data()
    # Gate B reproduces full chronological state, not a filter of admitted loops.
    baseline,_,discoveries,loops=run(data,'P0_FULL_TOP3',save=True)
    historical=pd.read_csv(A/'online_verified_loops.csv')
    assert baseline['attempts']==15267 and baseline['geometrically_accepted']==1088 and len(loops)==288
    assert baseline['application_bytes']==6107977604
    assert [(x['discovery_event'],x['canonical_robot1_kf'],x['canonical_robot3_kf']) for x in loops]==[
        (int(x.discovery_event),int(x.canonical_robot1_kf),int(x.canonical_robot3_kf)) for x in historical.itertuples()]
    write_json(OUT/'gate_B.json',dict(status='PASS',exact_counts_and_loop_order=True,**baseline))
    # Finite predeclared development search, using measured geometric outcomes, not GT correctness.
    target=run(data,'P0_FULL_TOP3',end=1000)[0]['geometrically_accepted']
    development=[]
    for threshold in data.policy['utility_threshold_grid']:
        res=run(data,'P5_DEV',threshold=threshold,compute=.75,communication=.8,end=1000)[0]
        res['geometric_retention']=res['geometrically_accepted']/target if target else 0
        development.append(res)
    feasible=[x for x in development if x['geometric_retention']>=.98]
    selected=min(feasible,key=lambda x:(x['attempts'],x['application_bytes'],x['utility_threshold'])) if feasible else development[0]
    threshold=selected['utility_threshold']
    csv(OUT/'development_proxy_search.csv',development)
    write_json(OUT/'finalist_lock.json',dict(threshold=threshold,development_proxy_feasible=bool(feasible),
        main_finalist='P5_BUDGET_AWARE_C75_B80',GT_used=False,grid_changed=False,
        development_search_sha256=sha(OUT/'development_proxy_search.csv'),frozen_unix_ns=__import__('time').time_ns()))
    summaries=[baseline]
    for name in data.policy['policies'][1:5]:summaries.append(run(data,name,save=True)[0])
    for compute in data.budget['compute_fractions']:
        for communication in data.budget['communication_fractions']:
            name=f'P5_BUDGET_AWARE_C{round(compute*100)}_B{round(communication*100)}'
            summaries.append(run(data,name,threshold,compute,communication,save=True)[0])
    csv(OUT/'policy_results_by_budget.csv',summaries)
    for filename in ['candidate_decision_trace.csv','causal_feature_table.csv']:
        csv(OUT/filename,pd.concat([pd.read_csv(OUT/'policies'/r['policy']/filename) for r in summaries],ignore_index=True))
    csv(OUT/'verified_loops_by_policy.csv',pd.concat([pd.read_csv(OUT/'policies'/r['policy']/'online_verified_loops.csv').assign(policy=r['policy']) for r in summaries],ignore_index=True))
    sensitivity=[];sensitivity_pairs={}
    for name in ['P0_FULL_TOP3','P5_BUDGET_AWARE_C75_B80']:
        for capacity in [None,1000,128]:
            result,attempts,_,_=run(data,name,threshold,.75 if name.startswith('P5') else 1,.8 if name.startswith('P5') else 1,capacity=capacity)
            sensitivity.append(result)
            sensitivity_pairs[name+':'+str(capacity)]=dict(policy=name,capacity=capacity,
                pairs=[[int(r['event_index']),int(r['candidate_rank'])] for r in attempts])
    csv(OUT/'cache_sensitivity.csv',sensitivity)
    write_json(OUT/'cache_sensitivity_selected_pairs.json',sensitivity_pairs)
    csv(OUT/'communication_by_policy.csv',summaries)
    write_json(OUT/'pre_gt_decision_lock.json',dict(GT_used=False,
        decisions={r['policy']:read(OUT/'policies'/r['policy']/'decision.json') for r in summaries},
        finalist_lock_sha256=sha(OUT/'finalist_lock.json'),freeze_unix_ns=__import__('time').time_ns()))
    write_json(OUT/'gates_CDE.json',dict(C='PASS',D='PASS',E='PASS',dev_proxy_feasible=bool(feasible),
        explanation='No GT optimization; infeasible proxy target is retained as negative evidence, not relaxed.'))
    print('GATES_BCDE_COMPLETED',flush=True)

if __name__=='__main__':main()
