"""Causal, bounded scheduler. Receipt access occurs after an action is committed."""
from collections import deque,Counter
from common import *

def ids(r):
    q,c=int(r['query_keyframe_id']),int(r['candidate_keyframe_id'])
    return (q,c) if r['query_robot']=='robot1' else (c,q)
def keys(r):return [(r['query_robot'],int(r['query_keyframe_id'])),(r['candidate_robot'],int(r['candidate_keyframe_id']))]
def duplicate(q,c,past):return any(abs(q-a)<=3 and abs(c-b)<=5 for a,b in past)
def limits():
    b=read(B/'stage22b_budget_policy.json');p=read(B/'stage22b_experiment_policy.json')
    return dict(calls=int(b['reference_calls']*.75),bytes=int(b['reference_application_bytes']*.8),roll_calls=int(300*.75),roll_bytes=int(100*b['reference_application_bytes']/5796*.8),max_cpu=max(p['predicted_cpu_ms'].values()))
def protocol():
    messages=pd.read_csv(A/'online_protocol_message_trace.csv',usecols=['event_index','message_type','serialized_total_bytes','robot_id','keyframe_id'])
    fixed=messages[~messages.message_type.isin(['POINT_CLOUD_REQUEST','POINT_CLOUD_RESPONSE','LOOP_CONSTRAINT'])].groupby('event_index').serialized_total_bytes.sum().reindex(range(5796),fill_value=0).to_numpy(np.int64)
    cloud=messages[messages.message_type=='POINT_CLOUD_RESPONSE'];assert len(cloud)==5796 and set(cloud.serialized_total_bytes+49)=={1048681}
    return fixed

def audit_p5(fixed):
    p=read(B/'stage22b_experiment_policy.json');n=p['normalization'];w=p['P5_weights'];directory=B/'policies'/P5
    assert w==dict(confidence=1.,margin=.1,novelty=.1,redundancy=.1,cpu=.05,transfer=.05)
    assert read(B/'finalist_lock.json')['threshold']==.5
    f=pd.read_csv(directory/'causal_feature_table.csv');tr=pd.read_csv(directory/'candidate_decision_trace.csv');sel=pd.read_csv(directory/'selected_attempts.csv');cap=limits()
    components=pd.DataFrame(dict(confidence=f.confidence,margin=.1*np.minimum(f.margin12/.1,1),novelty=.1*f.novelty,redundancy=-.1*f.redundancy.astype(int),CPU=-.05*f.predicted_cpu_ms/n['cpu_scale_ms'],transfer=-.05*f.estimated_transfer_bytes/n['transfer_scale_bytes']))
    error=float(np.max(abs(components.sum(axis=1)-f.utility)));assert error<1e-12
    flips={c:int(((components.sum(axis=1)>=.5)!=((components.sum(axis=1)-components[c])>=.5)).sum()) for c in components}
    executed=tr[tr.execute];ev=np.zeros(5796,dtype=int);byte=np.zeros(5796,dtype=np.int64);cpu=np.zeros(5796)
    for r in executed.itertuples():ev[int(r.event_index)]+=1;byte[int(r.event_index)]+=int(r.variable_bytes)
    for r in sel.itertuples():cpu[int(r.event_index)]+=p['predicted_cpu_ms'][r.query_robot+':'+str(r.candidate_rank)]
    roll=lambda x:pd.Series(x).rolling(100,min_periods=1).sum().to_numpy()
    timeline=pd.DataFrame(dict(event_index=np.arange(5796),calls=ev,cumulative_calls=ev.cumsum(),cumulative_application_bytes=np.cumsum(fixed+byte),reserved_total_application_bytes=fixed.sum()+byte.cumsum(),rolling_calls_100=roll(ev),rolling_bytes_100=roll(fixed+byte),rolling_predicted_CPU_ms_100=roll(cpu)))
    timeline['rolling_byte_capacity_remaining']=cap['roll_bytes']-timeline.rolling_bytes_100;timeline['rolling_call_capacity_remaining']=cap['roll_calls']-timeline.rolling_calls_100;timeline['global_byte_capacity_remaining']=cap['bytes']-timeline.reserved_total_application_bytes
    assert int(ev.sum())==8647 and int((fixed+byte).sum())==4179439052
    csv(OUT/'p5_rolling_budget_timeline.csv',timeline)
    rejections=[]
    joined=f.merge(tr[['event_index','candidate_rank','execute','reason']],on=['event_index','candidate_rank'],validate='one_to_one')
    for r in joined[~joined.execute].itertuples():
        i=int(r.event_index);cost=int(r.estimated_transfer_bytes)+249;future=timeline.iloc[i+1:min(i+101,5796)];fits=future[(future.rolling_byte_capacity_remaining>=cost)&(future.rolling_call_capacity_remaining>=1)&(future.global_byte_capacity_remaining>=cost)]
        rejections.append(dict(event_index=i,candidate_rank=int(r.candidate_rank),direction=r.query_robot,SC_score=r.SC_score,confidence=r.confidence,utility=r.utility,reason=r.reason,estimated_transfer_bytes=int(r.estimated_transfer_bytes),rolling_bytes_remaining_before=r.rolling_bytes_remaining,rolling_calls_remaining_before=r.rolling_calls_remaining,blocked_during_busy_window=bool(r.rolling_bytes_remaining<cost or r.rolling_calls_remaining<1),first_later_frozen_capacity_event=int(fits.iloc[0].event_index) if len(fits) else None,capacity_note='descriptive frozen timeline AFTER its actions; not a rescheduled counterfactual',measured_outcome_status='not consulted by rejection analysis'))
    csv(OUT/'p5_rejection_analysis.csv',rejections)
    result=read(directory/'decision.json')['result'];csv(OUT/'p5_utility_components.csv',pd.concat([f[['event_index','candidate_rank','utility']],components],axis=1))
    audit=dict(status='COMPLETE',formula='confidence +0.1 margin +0.1 novelty -0.1 redundancy -0.05 predicted_cpu -0.05 predicted_transfer',normalizations=n,weights=w,frozen_threshold=.5,max_utility_reconstruction_error=error,weights_origin='heuristic predeclared scalarization; no theoretical mapping-utility derivation or calibration; threshold development-tested events0:999 with measured acceptance, no GT',absolute_contribution_medians={c:float(abs(components[c]).median()) for c in components},drop_one_term_threshold_flip_counts_fixed_state=flips,sensitivity_scope='descriptive frozen states only; not rerun policies, GT optimization or independent ablations',decision_rows=len(tr),rejected_candidate_rows=len(rejections),rejected_queries=int(joined[~joined.execute].event_index.nunique()),rejection_reasons=joined[~joined.execute].reason.value_counts().astype(int).to_dict(),busy_rejection_rows=sum(r['blocked_during_busy_window'] for r in rejections),unused_rolling_capacity_events=int(((timeline.rolling_byte_capacity_remaining>0)&(timeline.rolling_call_capacity_remaining>0)).sum()),remaining_global_bytes=int(cap['bytes']-result['application_bytes']),budget_rejections_by_direction=joined[(~joined.execute)&joined.reason.isin(['COMPUTE_BUDGET','COMMUNICATION_BUDGET'])].groupby('query_robot').size().astype(int).to_dict(),known_original_GICP_attempts=15267,unknown_original_candidate_combinations=2112,GT_used=False,reference=result)
    dump(OUT/'p5_utility_audit.json',audit);return result

class Engine:
    def __init__(self,fixed):
        self.fixed=fixed;self.cap=limits();self.p=read(B/'stage22b_experiment_policy.json');self.plan=read(OUT/'scheduler_predeclared_policy.json')
        self.events=pd.read_csv(A/'dual_stream_event_schedule.csv').to_dict('records');c=pd.read_csv(A/'online_candidate_trace.csv');self.groups={int(i):g.sort_values('candidate_rank').to_dict('records') for i,g in c.groupby('event_index')}
        local=pd.read_csv(BASE/'12d_clean_backend/clean_local_trajectory.csv');self.bins={}
        for r,f in local.sort_values('keyframe_id').groupby('robot_id'):
            x=f[['tx','ty','tz']].to_numpy();arc=np.r_[0,np.linalg.norm(np.diff(x,axis=0),axis=1).cumsum()];self.bins.update({(r,int(k)):int(a//50) for k,a in zip(f.keyframe_id,arc)})
    def run(self,name,variant,cache_contexts=True):
        start=time.monotonic();cap=self.cap;cache=set();seen={};pending={};admitted=[];covered=set();direction_calls=Counter();arrivals=Counter();calls=0;variable=0;pred_cpu=0.;actual_cpu=0.;ledger=deque();tr=[];unknown=[];factors=[];snapshots=[];first_bridge=None;maxqueue=0;checks=0;halt=False
        def record(action,i,item,**extra):
            row=item['rows'][item['next']];q,c=ids(row)
            tr.append(dict(policy=name,action=action,event_index=i,discovery_event=item['event'],candidate_rank=int(row['candidate_rank']),robot1_kf=q,robot3_kf=c,direction=row['query_robot'],queue_age=i-item['event'],expiry_event=item['event']+self.plan['max_age_events'],**extra))
        def context(item,i):
            row=item['rows'][item['next']];ks=keys(row);q,c=ids(row);transfer=1048681*sum(k not in cache for k in ks);binpair=(self.bins['robot1',q],self.bins['robot3',c]);pred=self.p['predicted_cpu_ms'][row['query_robot']+':'+str(row['candidate_rank'])]
            cls=0 if not admitted else 1 if binpair not in covered else 2 if transfer<2097362 else 3
            if variant<2:priority=(item['event'],int(row['candidate_rank']))
            else:priority=(cls,direction_calls[row['query_robot']]/max(1,arrivals[row['query_robot']]) if variant==3 else 0,transfer,int(row['candidate_rank']),-float(row['SC_score']),item['event'],q,c)
            used_calls=sum(x[1] for x in ledger);used_bytes=sum(x[2] for x in ledger);used_cpu=sum(x[3] for x in ledger);fixedroll=int(self.fixed[max(0,i-99):i+1].sum())
            reason='GLOBAL_CALL_CAP' if calls+1>cap['calls'] else 'GLOBAL_BYTE_CAP' if self.fixed.sum()+variable+transfer+249>cap['bytes'] else 'ROLLING_CALL_CAP' if used_calls+1>cap['roll_calls'] else 'ROLLING_BYTE_CAP' if fixedroll+used_bytes+transfer+249>cap['roll_bytes'] else 'ROLLING_PREDICTED_CPU_CAP' if used_cpu+pred>cap['roll_calls']*cap['max_cpu']+1e-8 else 'GLOBAL_PREDICTED_CPU_CAP' if pred_cpu+pred>cap['calls']*cap['max_cpu']+1e-8 else None
            return row,ks,transfer,binpair,pred,priority,reason
        for event in self.events:
            i=int(event['event_index']);seen[event['robot_id'],int(event['keyframe_id'])]=i;arrivals[event['robot_id']]+=1
            while ledger and ledger[0][0]<=i-100:ledger.popleft()
            for eid,item in list(pending.items()):
                if i-item['event']>self.plan['max_age_events']:record('EXPIRED',i,item,reason='MAX_AGE');del pending[eid]
            rows=self.groups.get(i,[])
            if rows:
                item=dict(event=i,rows=rows,next=0,last_block=None)
                assert all(np.isfinite(float(r['SC_score'])) and r['query_robot']!=r['candidate_robot'] for r in rows),'Invalid scientific candidate input'
                assert all(k in seen and seen[k]<=i for r in rows for k in keys(r))
                if len(pending)>=self.plan['max_queue_queries']:record('DROPPED',i,item,reason='QUEUE_FULL')
                elif any(ids(rows[0])==ids(x['rows'][x['next']]) for x in pending.values()):record('DROPPED',i,item,reason='DUPLICATE_PENDING_CANONICAL')
                else:pending[i]=item;record('ENQUEUED',i,item,reason='DISCOVERED_ORIGINAL_QUERY')
            maxqueue=max(maxqueue,len(pending));blocked=set()
            choices_cache=None
            while pending:
                if not cache_contexts:choices_cache=None
                if choices_cache is None:
                    choices_cache=[]
                    for eid,item in pending.items():
                        if eid in blocked:continue
                        ctx=context(item,i);choices_cache.append((ctx[5],eid,ctx))
                    choices_cache.sort(key=lambda x:(x[0],x[1]),reverse=True)
                choices=choices_cache
                if not choices:break
                _,eid,ctx=choices.pop();row,ks,transfer,binpair,pred,priority,reason=ctx;item=pending[eid];checks+=1
                if reason:
                    if item['last_block']!=reason:record('DEFERRED',i,item,reason=reason,estimated_bytes=transfer+249,predicted_CPU_ms=pred,priority_class=priority[0]);item['last_block']=reason
                    blocked.add(eid);continue
                assert all(seen[k]<=i for k in ks)
                # Selection is irrevocably logged BEFORE even testing receipt existence.
                record('SELECTED',i,item,estimated_bytes=transfer+249,predicted_CPU_ms=pred,prior_rank_failures=item['next'],endpoint_max_arrival_event=max(seen[k] for k in ks),priority_class=priority[0])
                path=A/'attempts'/f"e{eid:04d}_r{int(row['candidate_rank'])}.json"
                if not path.exists():
                    record('UNKNOWN_REQUIRES_ACTUAL_GICP',i,item,reason='NO_MEASURED_RECEIPT_AFTER_CAUSAL_SELECTION');q,c=ids(row);unknown.append(dict(policy=name,event_index=i,discovery_event=eid,rank=int(row['candidate_rank']),robot1_kf=q,robot3_kf=c,reason='selected causal query prefix lacks actual GICP; halt policy before outcome-dependent state changes'));halt=True;break
                actual=read(path)['measurement'];assert sha(path.with_suffix('.csv'))==actual['result_csv_sha256'];assert keys(actual)==ks
                accepted=bool(actual['geometrically_accepted']);q,c=ids(row);admit=accepted and not duplicate(q,c,admitted);extra=transfer+249*admit
                calls+=1;variable+=extra;pred_cpu+=pred;actual_cpu+=float(actual['call_cpu_ms']);direction_calls[row['query_robot']]+=1;cache.update(ks);ledger.append((i,1,extra,pred))
                if admit:
                    Z=np.array(json.loads(actual['canonical_transform']) if isinstance(actual['canonical_transform'],str) else actual['canonical_transform']);assert np.isfinite(Z).all();admitted.append((q,c));covered.add(binpair);factors.append(dict(policy=name,insertion_event=i,discovery_event=eid,rank=int(row['candidate_rank']),robot1_kf=q,robot3_kf=c,canonical_transform=json.dumps(Z.tolist()),receipt=str(path),receipt_sha256=sha(path)))
                    if first_bridge is None:first_bridge=i
                record('VERIFIED',i,item,variable_bytes=extra,transfer_bytes=transfer,predicted_CPU_ms=pred,actual_receipt_CPU_ms=float(actual['call_cpu_ms']),geometrically_accepted=accepted,graph_admitted=admit,cumulative_calls=calls,cumulative_application_bytes=int(self.fixed.sum()+variable))
                assert calls<=cap['calls'] and self.fixed.sum()+variable<=cap['bytes']
                assert sum(x[1] for x in ledger)<=cap['roll_calls'] and self.fixed[max(0,i-99):i+1].sum()+sum(x[2] for x in ledger)<=cap['roll_bytes']
                if accepted or item['next']+1==len(item['rows']):del pending[eid]
                else:item['next']+=1;item['last_block']=None
                # A denial changes no cache, ledger, priorities or admissions.
                # Recompute contexts only after an executed verification.
                choices_cache=None
                # A successful lower rank closes the entire query even when its
                # factor is suppressed. No higher rank is inferred or unlocked.
            snapshots.append(dict(policy=name,event_index=i,queue_queries=len(pending),calls=calls,variable_bytes=variable,rolling_calls=sum(x[1] for x in ledger),rolling_variable_bytes=sum(x[2] for x in ledger),rolling_predicted_CPU_ms=sum(x[3] for x in ledger)))
            if halt:break
            if variant==0:
                for item in pending.values():record('DROPPED',i,item,reason='IMMEDIATE_CONTROL_NO_DEFERRAL')
                pending.clear()
        if not halt:
            for item in pending.values():record('EXPIRED',5795,item,reason='END_OF_STREAM_NO_FUTURE_EVENTS')
        actions=Counter(x['action'] for x in tr);delayed=[x for x in tr if x['action']=='VERIFIED' and x['queue_age']>0]
        result=dict(policy=name,status='PARTIAL' if halt else 'PASS',completed_events=len(snapshots),selected_verification_attempts=actions['SELECTED'],completed_receipt_decisions=calls,new_actual_GICP_executions=0,delayed_verifications=len(delayed),queue_expirations=actions['EXPIRED'],queue_drops=actions['DROPPED'],first_bridge_event=first_bridge,admitted_factors=len(admitted),covered_odometry_bin_pairs=len(covered),direction_attempts=dict(direction_calls),unknown_outcomes=len(unknown),application_bytes=int(self.fixed[:len(snapshots)].sum()+variable),global_fixed_reserved_plus_variable_bytes=int(self.fixed.sum()+variable),byte_cap=cap['bytes'],call_cap=cap['calls'],byte_utilization=float((self.fixed.sum()+variable)/cap['bytes']),call_utilization=calls/cap['calls'],peak_queue_queries=maxqueue,decision_checks=checks,runtime_s=time.monotonic()-start,peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,accuracy='NOT_RUN_NO_NEW_POSE_GRAPH; no complete Recall/ATE claim',communication='PROTOCOL_SIMULATED; exact frozen serializer catalog, not physical network traffic',cache='unbounded same as P5')
        return result,tr,unknown,factors,snapshots

def main():
    protect_verify();guard();fixed=protocol();reference=audit_p5(fixed);engine=Engine(fixed);results=[dict(policy='P5_FROZEN',status='FROZEN_REFERENCE',completed_receipt_decisions=int(reference['attempts']),admitted_factors=int(reference['admitted']),application_bytes=int(reference['application_bytes']),new_actual_GICP_executions=0,accuracy='read frozen Stage22B only')];traces=[];unknown=[];allfactors=[];allbudgets=[]
    for variant,name in enumerate(['P6_D0_IMMEDIATE_CONTROL','P6_D1_DEFERRED_ONLY','P6_D2_DEFERRED_COVERAGE','P6_D3_DEFERRED_COVERAGE_DIRECTION']):
        result,tr,u,f,b=engine.run(name,variant);result['bytes_saved_vs_frozen_P5']=reference['application_bytes']-result['application_bytes'];results.append(result);traces.extend(tr);unknown.extend(u);allfactors.extend(f);allbudgets.extend(b);print(json.dumps(result),flush=True)
    csv(OUT/'p6_ablation_comparison.csv',results);csv(OUT/'p6_queue_event_trace.csv',traces);csv(OUT/'unknown_gicp_extension_plan.csv',unknown,columns=['policy','event_index','discovery_event','rank','robot1_kf','robot3_kf','reason']);csv(OUT/'p6_verified_factors.csv',allfactors);csv(OUT/'p6_budget_event_trace.csv',allbudgets)
    dump(OUT/'scheduler_trace_freeze.json',dict(GT_read=False,sha256={p.name:sha(p) for p in OUT.glob('*.csv')},new_GICP_calls=0,new_backend_solves=0,final_independent_validation='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE'))
if __name__=='__main__':main()
