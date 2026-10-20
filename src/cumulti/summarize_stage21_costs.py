#!/usr/bin/env python3
"""Evidence-separated summaries/figures, no new algorithm or optimization."""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np,pandas as pd
from prepare_stage21_policy import ROOT,OUT,PROC,dump

def stats(x):
    a=np.asarray(x,float);a=a[np.isfinite(a)]
    return dict(sample_count=len(a),mean=float(a.mean()),median=float(np.median(a)),p90=float(np.percentile(a,90)),p95=float(np.percentile(a,95)),maximum=float(a.max()))

def main():
    ops=pd.read_csv(OUT/'frontend_operation_timings.csv');pairs=pd.read_csv(OUT/'frontend_pair_operations.csv');q=pd.read_csv(OUT/'frontend_benchmark_queries.csv');backend=pd.read_csv(ROOT/'outputs/cumulti_v1/20_dual_stream/central_incremental_event_trace.csv').fillna({'new_loop_ids':''});batch=pd.read_csv(ROOT/'outputs/cumulti_v1/20_dual_stream/incremental_vs_batch_diagnostics.csv').drop_duplicates('checkpoint');policy=json.loads((OUT/'stage21_measurement_policy.json').read_text())
    front=[]
    def fr(name,x,source,scope,cpu=None):
        s=stats(x);s.update(operation=name,source=source,scope=scope,unit='ms',cpu_mean_ms=None if cpu is None else float(np.mean(cpu)));front.append(s)
    for (name,robot),a in ops.groupby(['operation','robot_id']):fr(name+' / '+robot,a.wall_ms,'NEW_ACTUAL_TIMING','fixed100 queries x2; R3 descriptor extraction uses20 fixed samples',a.cpu_ms)
    for name,col in [('cloud_load_decode','cloud_load_decode_ms'),('exclusive_cloud_preprocessing','cloud_preprocess_ms'),('GICP_coarse','coarse_ms'),('GICP_fine','fine_ms'),('GICP_registration_per_attempt','registration_ms'),('backend_manifest_index_csv_other','manifest_index_CSV_and_other_ms'),('backend_call_per_attempt','call_wall_ms'),('verification_decision_arithmetic_micro','decision_arithmetic_micro_ms')]:fr(name,pairs[col],'NEW_ACTUAL_TIMING','same frozen173 attempts per100-query repeat; fresh per-call LRU64; OS cache unknown')
    for name,col in [('M1000_retrieval_per_query','retrieval_ms'),('rank1_sequential_harness','rank1_actual_harness_ms'),('frozen_top3_trace_sequential_harness','frozen_top3_trace_harness_ms')]:fr(name,q[col],'NEW_ACTUAL_TIMING','actual same-invocation measurements, not sum of independent averages',q.process_cpu_ms if name.startswith('frozen') else None)
    hist=pd.read_csv(ROOT/'outputs/cumulti_v1/09_sc_gicp_integration/integrated_latency.csv')
    for name,col in [('Stage9_historical_retrieval','retrieval_s'),('Stage9_historical_rank1_integrated','rank1_pipeline_s'),('Stage9_historical_top3_integrated','top3_early_stop_pipeline_s')]:fr(name,hist[col]*1000,'HISTORICAL_IMPORTED','original100 prebuilt-database sequential harness; original thread/hardware context retained')
    s8=pd.read_csv(ROOT/'outputs/cumulti_v1/08_efficient_sc_retrieval/exhaustive_retiming.csv');fr('Stage8_exhaustive',s8.total_s*1000,'HISTORICAL_IMPORTED','all2000 prebuilt R3 database queries')
    pd.DataFrame(front).to_csv(OUT/'frontend_benchmark_summary.csv',index=False)
    back=[]
    subsets={'Robot1_local_processing':backend[backend.robot_id.eq('robot1')],'Robot3_local_processing':backend[backend.robot_id.eq('robot3')],'central_iSAM2_odometry_R1':backend[backend.event_type.eq('ODOMETRY_UPDATE')&backend.robot_id.eq('robot1')],'central_iSAM2_odometry_R3':backend[backend.event_type.eq('ODOMETRY_UPDATE')&backend.robot_id.eq('robot3')],'central_iSAM2_loop_events':backend[backend.new_loop_ids.ne('')],'first_fusion':backend[backend.event_type.eq('FIRST_BRIDGE')],'estimate_extraction':backend[backend.global_initialized],'map_publication_nonzero_events':backend[backend.map_publication_ms.gt(0)],'finite_integrity_analysis_only':backend[backend.finite_integrity_audit_ms.gt(0)]}
    for label,a in subsets.items():
        columns=['local_processing_ms'] if label.endswith('local_processing') else ['map_publication_ms'] if label.startswith('map_') else ['estimate_extraction_ms'] if label=='estimate_extraction' else ['finite_integrity_audit_ms'] if label.startswith('finite_') else ['graph_construction_ms','isam2_update_ms','estimate_extraction_ms','map_publication_ms','measured_service_ms'] if label=='first_fusion' else ['isam2_update_ms']
        for c in columns:back.append(dict(category=label,metric=c,source='HISTORICAL_IMPORTED_STAGE20',scope='DUAL_STREAM_ARCHIVED_FACTOR_REPLAY',cpu_process_time='NOT_MEASURED',peak_RSS_mib=float(a.peak_rss_mib.max()),**stats(a[c])))
    back.append(dict(category='offline_batch_LM_DIAGNOSTIC_ONLY',metric='batch_runtime_ms',source='HISTORICAL_IMPORTED_STAGE20',scope='analysis-only, not an online pipeline component; one row per checkpoint',cpu_process_time='NOT_MEASURED',peak_RSS_mib=None,**stats(batch.batch_runtime_ms)))
    pd.DataFrame(back).to_csv(OUT/'backend_cost_summary.csv',index=False)
    # Exact publication-rate denominators are sensor-grid spans, not wireless rates.
    schedule=[];frames={}
    for robot in ['robot1','robot3']:
        f=pd.read_csv(PROC/robot/'keyframes.csv',usecols=['keyframe_id','lidar_timestamp_ns']);frames[robot]=f.set_index('keyframe_id');ts=f.lidar_timestamp_ns.to_numpy(np.int64);duration=(int(ts[-1])-int(ts[0]))/1e9;schedule.append(dict(robot_id=robot,keyframes=len(f),recorded_span_s=duration,keyframe_interval_median_ms=float(np.median(np.diff(ts)))/1e6,recorded_grid_hz=(len(f)-1)/duration,ring_key_array_bytes=80,ring_key_application_message_bytes=128,full_SC_array_bytes=4800,full_SC_application_message_bytes=4848,modelled_full_grid_ring_key_bytes_per_second=len(f)*128/duration,modelled_full_grid_fullSC_bytes_per_second=len(f)*4848/duration,status='MODELLED_PUBLICATION_RATE_FROM_REAL_TIMESTAMPS_NOT_NETWORK_MEASUREMENT'))
    pd.DataFrame(schedule).to_csv(OUT/'descriptor_publication_schedule.csv',index=False)
    interface=pd.read_csv(ROOT/'outputs/cumulti_v1/09_sc_gicp_integration/candidate_interface.csv');clean=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv');avail=[]
    for source,a in [('historical_top3_candidates',interface),('archived_clean75',clean)]:
        for row in a.itertuples():
            qi=int(row.query_keyframe_id);ci=int(row.candidate_keyframe_id);qt=int(frames['robot1'].loc[qi,'lidar_timestamp_ns']);ct=int(frames['robot3'].loc[ci,'lidar_timestamp_ns']);qn=qt-int(frames['robot1'].loc[150,'lidar_timestamp_ns']);cn=ct-int(frames['robot3'].loc[234,'lidar_timestamp_ns']);status='OUTSIDE_RETAINED_QUERY' if qi<150 else 'OUTSIDE_RETAINED_CANDIDATE' if ci<234 else 'FUTURE_CANDIDATE_AT_QUERY_TIME' if (cn,'robot3',ci)>(qn,'robot1',qi) else 'CANDIDATE_AVAILABLE_AT_QUERY_TIME';avail.append(dict(source_scope=source,query_keyframe_id=qi,candidate_keyframe_id=ci,SC_rank=int(row.SC_rank) if hasattr(row,'SC_rank') else 1,query_timestamp_ns=str(qt),candidate_timestamp_ns=str(ct),query_normalized_ns=str(qn),candidate_normalized_ns=str(cn),status=status,not_online_SC_GICP=True))
    av=pd.DataFrame(avail);av.to_csv(OUT/'candidate_availability_audit.csv',index=False)
    # Existing offline labels only summarize a sample already frozen without GT.
    labels=pd.read_csv(ROOT/'outputs/cumulti_v1/09_sc_gicp_integration/rank1_gicp_results.csv',usecols=['query_keyframe_id','offline_true_pair']);labels=labels[labels.query_keyframe_id.isin(policy['queries'])]
    pc=pd.read_csv(OUT/'pointcloud_payload_metrics.csv');dump('frontend_sample_coverage.json',dict(queries=policy['queries'],historical_rank1_offline_true_pair_counts={str(k):int(v) for k,v in labels.offline_true_pair.value_counts().items()},sampling_used_GT=False,labels_used_only_after_sample_freeze=True,attempted_rank_counts= pairs[pairs.repeat.eq(0)].SC_rank.value_counts().sort_index().to_dict(),accept_reject_counts=pairs[pairs.repeat.eq(0)].quality_accepted.value_counts().to_dict(),raw_point_count_range=[int(pc.point_count.min()),int(pc.point_count.max())],raw_cloud_size_variation='NOT_AVAILABLE: all sampled native Ouster frames have65536 points; no synthetic cropping to fabricate diversity',trajectory_query_id_range=[min(policy['queries']),max(policy['queries'])],scope='both repeated runs use identical IDs; uncertainty is descriptive across this fixed sample, no population CI or full-run measured-time extrapolation'))
    comm=pd.read_csv(OUT/'communication_summary_by_mode.csv');cat=pd.read_csv(OUT/'message_category_summary.csv')
    def catbytes(mode,kinds):return int(cat[cat['mode'].eq(mode)&cat.message_type.isin(kinds)].serialized_bytes.sum())
    for i,row in comm.iterrows():
        m=row['mode'];comm.loc[i,'descriptor_message_bytes']=catbytes(m,['RING_KEY_UPLOAD','FULL_SC_DESCRIPTOR_UPLOAD','FULL_SC_DESCRIPTOR_RESPONSE']);comm.loc[i,'point_cloud_message_bytes']=catbytes(m,['POINT_CLOUD_RESPONSE']);comm.loc[i,'loop_factor_message_bytes']=catbytes(m,['LOOP_CONSTRAINT']);comm.loc[i,'correction_update_message_bytes']=catbytes(m,['ODOMETRY_UPDATE','GLOBAL_POSE_CORRECTION']);comm.loc[i,'control_request_message_bytes']=catbytes(m,['CANDIDATE_REQUEST','FULL_SC_DESCRIPTOR_REQUEST','POINT_CLOUD_REQUEST','ACK_OR_CONTROL'])
    comm.to_csv(OUT/'communication_summary_by_mode.csv',index=False)
    table=[];split=[];newf=pd.DataFrame(front);fetch=lambda name:float(newf[(newf.operation==name)&(newf.source=='NEW_ACTUAL_TIMING')]['mean'].iloc[0])
    for row in comm[comm.primary_mode].to_dict('records'):
        oracle=row['mode'].startswith('B3');row.update(communication_evidence='actual encoded-buffer lengths summed over PROTOCOL_SIMULATED_SEND_EVENTS; not network',SC_retrieval_mean_ms=None if oracle else fetch('M1000_retrieval_per_query'),GICP_per_attempt_mean_ms=None if oracle else fetch('GICP_registration_per_attempt'),iSAM2_evidence='Stage20 actual event trace imported separately' if oracle else 'NOT_APPLICABLE_DIFFERENT_PROTOCOL',quality_equivalence='oracle excludes discovery, not an operational frontend' if oracle else 'lossless consumed XYZ/fullSC; cache does not prune candidates; no causal online-quality proof',network_latency='NOT_MEASURED',missing='physical robot CPU; transport/packet overhead; map/correction publication policy; causal online frontend',Pareto_status='NO_JOINT_ONLINE_PARETO_POINT; separated workload/evidence scopes')
        table.append(row)
        if not oracle:
            for operation,location,measurement in [('SC_descriptor_extraction','central' if row['mode'].startswith('B0') else 'robot-local proxy on measured server','SC_descriptor_extraction / robot1'),('Ring_Key_construction','central' if row['mode'].startswith('B0') else 'robot-local proxy on measured server','Ring_Key_construction / robot1'),('M1000_retrieval','central','M1000_retrieval_per_query'),('GICP_registration','central','GICP_registration_per_attempt')]:split.append(dict(mode=row['mode'],operation=operation,logical_location=location,physical_measurement_host='Linux server, NOT onboard robot CPU',mean_wall_ms=fetch(measurement),evidence='NEW_ACTUAL_TIMING_FIXED_SAMPLE',scope='historical prebuilt frontend only; full DB descriptor extraction cost not measured'))
    for a in back:split.append(dict(mode='B3_LOOP_FACTOR_ONLY_ORACLE',operation=a['category']+' / '+a['metric'],logical_location='robot-local frozen-chain update (not raw EKF cost)' if a['category'].startswith('Robot') else 'central',physical_measurement_host='historical Stage20 server',mean_wall_ms=a['mean'],evidence=a['source'],scope=a['scope']))
    pd.DataFrame(table).to_csv(OUT/'cost_comparison_table.csv',index=False);pd.DataFrame(table).to_csv(OUT/'stage22_pareto_input_SEPARATE_SCOPES.csv',index=False);pd.DataFrame(split).to_csv(OUT/'architecture_compute_split.csv',index=False)
    # All figure labels carry evidence scope; B3 never shares a comparable
    # frontend accuracy/compute coordinate with B0/B1/B2.
    def save(n,title,fn,size=(9,4)):
        fig,ax=plt.subplots(figsize=size,dpi=140);fn(ax);ax.set_title(title,fontsize=10);ax.grid(axis='y',alpha=.2);fig.tight_layout();fig.savefig(OUT/n);plt.close(fig)
    dm=pd.read_csv(OUT/'descriptor_payload_metrics.csv').groupby('representation').serialized_total_bytes.mean()
    save('01_descriptor_payload_comparison.png','MEASURED application buffers (48B framing included)',lambda ax:(ax.bar(dm.index,dm.values),ax.set_ylabel('bytes / message')))
    save('02_pointcloud_payload_distribution.png','MEASURED native XYZ / XYZI serialized buffers; no display caps',lambda ax:(ax.boxplot([pc[pc.representation.eq(k)].serialized_total_bytes/1024 for k in ['XYZ','XYZI']],tick_labels=['XYZ consumed fields','XYZI all stored fields']),ax.set_ylabel('KiB / application message')))
    primary=comm[comm.primary_mode].copy();primary['short']=primary['mode'].str[:2]
    save('03_communication_bytes_by_mode.png','SIMULATED protocol totals from ACTUAL encoded buffers; B3 ORACLE different scope',lambda ax:(ax.bar(primary.short,primary.total_serialized_bytes/1024**2,color=['#5082ba','#5082ba','#5082ba','#999999']),ax.set_yscale('log'),ax.set_ylabel('MiB (log); application only')))
    def cats(ax):
        base=np.zeros(4)
        for c,label in [('descriptor_message_bytes','descriptors'),('point_cloud_message_bytes','clouds'),('loop_factor_message_bytes','loop factors'),('correction_update_message_bytes','odom updates'),('control_request_message_bytes','requests')]:v=100*primary[c].to_numpy(float)/primary.total_serialized_bytes.to_numpy(float);ax.bar(primary.short,v,bottom=base,label=label);base+=v
        ax.set_ylabel("Share of each mode's own serialized bytes (%)");ax.set_ylim(0,100);ax.legend(fontsize=7)
    save('04_message_category_breakdown.png','SIMULATED sends: MEASURED byte-category shares; B3 separate oracle scope',cats)
    names=['SC_descriptor_extraction / robot1','M1000_retrieval_per_query','GICP_registration_per_attempt','backend_manifest_index_csv_other'];values=[fetch(n) for n in names]
    save('05_frontend_compute_breakdown.png','NEW MEASURED SAMPLE; per-frame/query/attempt scopes explicitly differ',lambda ax:(ax.barh(['SC extraction / frame','SC retrieval / query','GICP / attempt','manifest/index/CSV / attempt'],values),ax.set_xlabel('mean wall ms; NOT an end-to-end sum')))
    chosen=[a for a in back if a['metric'] in ['local_processing_ms','isam2_update_ms','map_publication_ms'] and a['category']!='first_fusion']
    save('06_backend_compute_breakdown.png','HISTORICAL IMPORT Stage20; conditional per-event distributions',lambda ax:(ax.barh([a['category'] for a in chosen],[a['median'] for a in chosen]),ax.set_xscale('log'),ax.set_xlabel('median wall ms (log); map shown for nonzero publication events')),size=(11,5))
    save('07_sc_gicp_latency_distribution.png','NEW ACTUAL TIMING: M1000 retrieval / query and GICP / attempted pair',lambda ax:(ax.hist(q.retrieval_ms,bins=25,alpha=.7,label='SC retrieval/query'),ax.hist(pairs.registration_ms,bins=35,alpha=.5,label='GICP/attempt'),ax.legend(),ax.set_xlabel('wall ms')))
    cache=pd.read_csv(OUT/'cache_sensitivity_summary.csv');ca=cache[cache['mode'].str.startswith('B2')]
    save('08_cache_sensitivity.png','PROTOCOL-SIMULATED cache; same measured100x1000 full-SC access trace',lambda ax:(ax.bar(ca.capacity.astype(str),ca.total_serialized_bytes/1024**2),ax.set_ylabel('actual encoded bytes summed / MiB')))
    def mixed(ax):
        common=primary[~primary['mode'].str.startswith('B3')];ax.scatter(common.total_serialized_bytes/1024**2,[float(q.frozen_top3_trace_harness_ms.mean())]*len(common))
        for r in common.itertuples():ax.annotate(r.short,(r.total_serialized_bytes/1024**2,float(q.frozen_top3_trace_harness_ms.mean())),xytext=(4,-16 if r.short=='B2' else 9),textcoords='offset points')
        ax.set_xlabel('SIMULATED common-workload application MiB');ax.set_ylabel('NEW MEASURED shared query-harness mean ms');ax.text(.02,.04,'Not a joint online Pareto frontier.\nPhysical placement, DB setup and transport not timed.\nB3 excluded: different oracle scope.',transform=ax.transAxes,fontsize=8)
    save('09_communication_vs_compute_baseline.png','Separated communication/compute evidence — no architecture speedup claim',mixed)
    counts=av.groupby(['source_scope','status']).size().unstack(fill_value=0)
    save('10_causal_candidate_availability.png','OFFLINE SCHEDULE AUDIT: historical candidates vs Stage20 endpoint availability',lambda ax:(counts.plot.bar(stacked=True,ax=ax),ax.set_ylabel('candidate pairs'),ax.tick_params(axis='x',rotation=0),ax.legend(fontsize=6)),size=(11,5))
    print('SUMMARIES_AND_10_FIGURES_WRITTEN',flush=True)
if __name__=='__main__':main()
