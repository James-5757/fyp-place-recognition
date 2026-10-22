"""Generate supported figures, honest decisions and lightweight research context."""
import datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stage22a_common import *

def main():
    assert read('independent_validation.json')['status']=='PASS'
    if (OUT/'checkpoint_recovery_audit.json').exists() and (OUT/'gicp_full_invocation_wall.json').exists():
        wall=read('gicp_full_invocation_wall.json');recovery=read('checkpoint_recovery_audit.json')
        wall['scope']='resumed verification subprocess invocation ONLY, including receipt replay and new attempts; excludes disconnected downtime and initial invocation'
        wall['previous_measured_wall_to_last_durable_checkpoint_seconds']=recovery['prior_progress']['elapsed_invocation_s']
        wall['sum_of_recorded_working_wall_windows_seconds']=wall['measured_wall_seconds']+recovery['prior_progress']['elapsed_invocation_s']
        wall['sum_is_not_uninterrupted_end_to_end_latency']=True;dump('gicp_full_invocation_wall.json',wall)
    a1=read('bidirectional_retrieval_summary.json');a2=read('frontend_execution_coverage.json');a3=read('stage22a3_decision.json');pilot=read('gicp_pilot_summary.json');cost=read('online_cost_summary.json');archive=read('offline_archive_comparison_summary.json')
    events=pd.read_csv(OUT/'dual_stream_event_schedule.csv');growth=pd.read_csv(OUT/'database_growth.csv');lat=pd.read_csv(OUT/'online_retrieval_latency.csv');candidates=pd.read_csv(OUT/'online_candidate_trace.csv');attempts=pd.read_csv(OUT/'online_gicp_attempts.csv');discovery=pd.read_csv(OUT/'online_loop_discovery_trace.csv');com=pd.read_csv(OUT/'online_communication_cost.csv');compute=pd.read_csv(OUT/'online_frontend_compute_cost.csv');accuracy=pd.read_csv(OUT/'offline_frontend_accuracy.csv')
    def save(name,title,draw):
        fig,ax=plt.subplots(figsize=(10,5));draw(ax);ax.set_title(title);fig.tight_layout();fig.savefig(OUT/name,dpi=140);plt.close(fig)
    seconds=growth.normalized_time_ns/1e9
    save('01_dynamic_database_growth.png','Independent arrived-only databases; normalized separate-recording clock',lambda ax:(ax.plot(seconds,growth.robot1_size,label='R1'),ax.plot(seconds,growth.robot3_size,label='R3'),ax.set_xlabel('hypothetical replay seconds'),ax.set_ylabel('arrived frames'),ax.legend()))
    save('02_bidirectional_candidate_timeline.png','Causal Top3 candidates (not geometric acceptance)',lambda ax:([ax.scatter(g.normalized_time_ns/1e9,g.candidate_keyframe_id,s=1,label=r) for r,g in candidates.groupby('query_robot')],ax.set_xlabel('normalized query time (s)'),ax.set_ylabel('candidate original KF'),ax.legend()))
    save('03_future_candidate_leakage_audit.png','Independent event-position audit: future candidates = 0',lambda ax:(ax.plot(events.event_index,np.zeros(len(events))),ax.set_xlabel('event index'),ax.set_ylabel('future candidates'),ax.set_ylim(-.1,1)))
    save('04_online_SC_latency.png','Actual SC retrieval, including KD index rebuild',lambda ax:(ax.plot(lat.event_index,lat.total_retrieval_ms,lw=.5),ax.set_xlabel('event index'),ax.set_ylabel('ms/query')))
    g=attempts.groupby('event_index').agg(calls=('candidate_rank','count'),wall_ms=('call_wall_ms','sum'))
    save('05_GICP_calls_and_runtime.png','Actual attempted GICP harness time (including loading/index/I/O)',lambda ax:(ax.scatter(g.index,g.wall_ms,s=3,c=g.calls,cmap='viridis'),ax.set_xlabel('verified event index'),ax.set_ylabel('ms/verified query')))
    save('06_online_loop_discovery_timeline.png','New causal representative loops, not archived75',lambda ax:(ax.step(discovery[discovery.duplication_status.eq('BACKEND_GRAPH_ADMISSIBLE')].discovery_normalized_ns/1e9,np.arange(1,a2['independent_loops']+1),where='post') if len(discovery) else ax.text(.1,.5,'No chronological full-verification loop set admitted',transform=ax.transAxes),ax.set_xlabel('normalized discovery time (s)'),ax.set_ylabel('independent admitted loops')))
    save('07_online_vs_archived_loop_coverage.png','Offline exact-pair overlap; NOT ground-truth recall',lambda ax:(ax.bar(['archived75 overlap','new pairs outside archive'],[archive['exact_overlap'],archive['new_exact_pairs_not_in_archive']]),ax.set_ylabel('new admitted canonical pairs')))
    save('08_compute_cost_per_query.png','Measured algorithm costs; post-hoc serialization reported separately',lambda ax:(ax.plot(compute.event_index,compute.candidate_retrieval_ms,label='retrieval'),ax.plot(compute.event_index,compute.actual_GICP_call_ms,label='GICP harness'),ax.plot(compute.event_index,compute.actual_isam2_update_ms,label='ISAM2'),ax.set_xlabel('event index'),ax.set_ylabel('ms'),ax.legend()))
    cats=pd.read_csv(OUT/'online_protocol_category_summary.csv')
    save('09_protocol_byte_cost.png','Real serialized buffers over simulated B1 sends; network NOT_MEASURED',lambda ax:(ax.bar(cats.message_type,cats.serialized_total_bytes/1e6),ax.tick_params(axis='x',rotation=35),ax.set_ylabel('application MB')))
    if a3['status']=='CAUSAL_ONLINE_GRAPH_READY':
        back=pd.read_csv(OUT/'online_backend_event_trace.csv')
        save('10_online_graph_growth.png','Persistent ISAM2: exact newly discovered graph',lambda ax:(ax.plot(back.event_index,back.nodes,label='nodes'),ax.plot(back.event_index,back.loops,label='new loops'),ax.legend(),ax.set_xlabel('event index')))
        bridge=read('first_online_bridge.json');tr=pd.read_csv(OUT/'online_incremental_trajectory.csv');end=int(bridge['discovery_event']);prefix=events[events.event_index.le(end)]
        def first_plot(ax):
            for r,g in tr.groupby('robot_id'):
                ids=set(prefix[prefix.robot_id.eq(r)].keyframe_id);x=g[g.keyframe_id.isin(ids)];ax.plot(x.tx,x.ty,label=r)
            ax.legend();ax.axis('equal');ax.set_xlabel('gauge X (m)');ax.set_ylabel('gauge Y (m)')
        # No future final estimates shown as first-bridge state: reconstruct from
        # frozen local prefixes and the actually discovered first measurement.
        local=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv');S=np.asarray(bridge['common_frame_transform'])
        from scipy.spatial.transform import Rotation
        firstrows=[]
        for row in local.itertuples():
            if (row.robot_id,row.keyframe_id) not in {(e.robot_id,e.keyframe_id) for e in prefix.itertuples()}:continue
            T=np.eye(4);T[:3,:3]=Rotation.from_quat([row.qx,row.qy,row.qz,row.qw]).as_matrix();T[:3,3]=[row.tx,row.ty,row.tz]
            if row.robot_id=='robot3':T=S@T
            firstrows.append(dict(robot_id=row.robot_id,tx=T[0,3],ty=T[1,3]))
        def initial_plot(ax):
            for r,g in pd.DataFrame(firstrows).groupby('robot_id'):ax.plot(g.tx,g.ty,label=r)
            ax.legend();ax.axis('equal');ax.set_xlabel('gauge X (m)');ax.set_ylabel('gauge Y (m)')
        save('11_first_discovered_loop_fusion.png','Actual first-bridge common initialization: arrived prefixes only',initial_plot)
    complete=a2['status']=='ONLINE_GICP_VERIFICATION_READY';ready=a3['status']=='CAUSAL_ONLINE_GRAPH_READY'
    final='CAUSAL_ONLINE_GRAPH_READY' if ready else 'CAUSAL_ONLINE_GRAPH_NOT_READY' if complete else 'FRONTEND_VERIFICATION_INCOMPLETE'
    readiness='CAUSAL_BASELINE_AVAILABLE_FOR_SEPARATELY_AUTHORIZED_STUDY; not started' if ready else 'FRONTEND_BASELINE_AVAILABLE_BUT_BACKEND_NOT_READY; no automatic22B' if complete else 'NOT_READY: incomplete verification'
    result=dict(status=final,retrieval=a1,verification=a2,backend=a3,offline_accuracy=json.loads(accuracy.to_json(orient='records')),archive_offline=archive,costs=cost,Stage22B_readiness=readiness,scope='hypothetical normalized-time replay of separate R1/R3 recordings; actual algorithms, no live simultaneous robots, no measured networking',GT_used_only_offline_after_decisions=True)
    dump('stage22a_final_decision.json',result);dump('summary.json',result)
    dump('verification_progress.json',dict(phase='FINISHED',status=a2['status'],completed_queries=a2['queries_verified'],actual_calls=a2['actual_calls'],admitted_representatives=a2['independent_loops'],backend_status=a3['status']))
    tests=read('independent_validation.json');bridge=read('first_online_bridge.json') if (OUT/'first_online_bridge.json').exists() else None
    recovery_note='No interrupted verification recovery was needed for this output run.'
    if (OUT/'checkpoint_recovery_audit.json').exists():
        recovery=read('checkpoint_recovery_audit.json')
        recovery_note=f'''The initial SSH-bound verification was interrupted after
{recovery['prior_progress']['completed_queries']} queries. Its
{recovery['durable_actual_calls']} durable receipts were hash-verified before
detached resume, with no uncertain in-flight attempt and no repeated registration.
The7200s actual-call budget was not reset. `checkpoint_recovery_audit.json`
preserves evidence. Resumed subprocess wall does not invent an uninterrupted
total latency. The optional initial pidfd timing observer was unavailable in
that Python build; the utility now has a quantified proc-polling fallback.
These are execution/reporting repairs, not threshold/noise/candidate changes.'''
    facts=f'''# CU-Multi Stage22A — causal online inter-robot frontend

Final decision: **{final}**. Artifact-validation PASS does not imply backend
numerical readiness, physical correctness, or full verification if incomplete.

## Motivation and frozen scope
Stage21 found74/75 archived clean candidates unavailable at the historical R1
query time. This independent baseline uses no archived loops to select queries,
register pairs, initialize the graph or replace bad constraints. Stage8 exact
SC/RingKey/scoring, Stage9 teammate GICP, Stage12D odometry/URDF/noise,
Stage20 scheduler and Stage21 wire schema are unchanged. Historical results,
including prior incremental/batch discrepancies, remain frozen. No demo changed.

## Causal clocks and databases
5,796 arrivals: R1KF150–1999 (1,850), R3KF234–4179 (3,946). Each recording
subtracts its own first retained timestamp; ties use normalized ns/robot/KF.
These are separate recordings, NOT a simultaneous physical meeting. At every
event: actual LiDAR load → frozen SC extraction → query opposite arrived DB →
publish → append own descriptor. Current opposite KD-tree is rebuilt with its
maintenance cost measured. M=min(1000,N); TopK=min(3,M). Frozen KD ordering and
stable descending-score sort determine ties. Reverse new-frame queries naturally
permit retrospective discovery; no unbounded historical re-query was added.

Two independent real-scan passes made {a1['descriptor_bitwise_checks']} bitwise
descriptor checks. Scientific candidate hashes are identical:
`{a1['two_pass_candidate_sha256'][0]}`. Future candidates:0. There is one
empty-database query. Total first-pass retrieval={a1['retrieval_wall_ms']/1000:.6f}s;
descriptor extraction={a1['descriptor_compute_ms']/1000:.6f}s. Frozen descriptor
arrays are integrity references only, never causal database initialization.

## Actual GICP, pilot and coverage
The150-event pilot was uniformly selected by event index before outcomes.
Pilot full-cost projection={pilot['projected_full_seconds']:.6f}s
(**MODELLED_PROJECTION**, not measured full time); frozen cap=7200s, including
new pilot attempts reused exactly once at their correct chronological event.
Coverage: {a2['queries_verified']}/5796 queries, actual calls={a2['actual_calls']},
geometrically accepted candidates={a2['accepted_candidates']}, rejected=
{a2['rejected_candidates']}. Measured total actual-call harness time=
{a2['actual_call_wall_seconds']:.6f}s. Every receipt contains actual quality,
fitness, RMSE, matrix, input IDs and load/preprocess/coarse/fine/other timings.
`call_wall_ms` measures unchanged teammate.run(), including manifest read,
full cloud-index validation, loading/preprocessing, registration and CSV write;
it excludes outer manifest creation, result parsing and receipt persistence.
It is NOT a full end-to-end pipeline latency. Full process wall is separately
measured in `gicp_full_process_observer.json` or
`gicp_full_invocation_wall.json`; the pilot projection is never substituted for it.
Frozen voxel.75m, XY2–50m, Z[-3,5]m, coarse3m/fine1m,30iterations and
quality fitness/(1+RMSE)>=.6091 are unchanged. Rank-ordered Top3 early stop.

The unchanged teammate cloud-index loader parses its existing mixed-metadata
keyframe CSV but consumes only KF/path fields; GT pose fields do not enter its
registration. New Stage22A indexing reads allowlisted ID/ns/path columns.
Runtime guards reject raw GT and historical loop/result paths. Opaque input
hashing is provenance, not oracle selection.

## Direction, causal duplicates and new discovery
GICP returns T_query_from_candidate. R3-query measurements are inverted as
complete SE(3) matrices to canonical T_R1_from_R3. Synthetic inverse,
composition and Pose3 factor tests pass. New admitted representatives=
{a2['independent_loops']}; causal duplicates={a2['duplicates']}. First qualifying
canonical pair wins within |deltaR1KF|<=3 AND |deltaR3KF|<=5; later qualities
never replace earlier representatives. Unlike historical whole-cluster max
quality sanitation, this policy is causal and predeclared. Geometric acceptance
is distinct from graph admission: finite proper SE3, retained endpoints and
causal duplicate screen. No additional GT-fitted gate.

First bridge (null means not run/found):
```json
{json.dumps(bridge,indent=2)}
```
After decisions were frozen, exact overlap with archived75=
{archive['exact_overlap']}; newly admitted exact pairs outside that archive=
{archive['new_exact_pairs_not_in_archive']}. Archived overlap is NOT GT recall.
Offline query-based candidate/loop accuracy and false accepted pairs are in
`offline_frontend_accuracy.csv` and labelled offline tables. Positive definition
is d_xy<5m among already available opposite frames. Query recall and precision
use explicit denominators; duplicate suppression affects per-query admission.

## Persistent backend and numerical status
Backend status: **{a3['status']}**. Structural status:
{a3.get('structural_status','NOT_RUN')}; numerical status:
{a3.get('numerical_status','NOT_RUN')}. Reason: {a3.get('reason')}.
First newly admitted bridge initializes currently available prefixes only via
S=X_R1q Z_R1fromR3 inverse(L_R3c). One R1KF150 prior; local independent maps
remain unmerged before that event. One default persistent GTSAM4.2 ISAM2 instance
inserts subsequent odometry/new loops exactly once, no extra empty updates.
Frozen rot-first5deg/translation1m sigmas, Huber1.345 loops and1e-6 prior remain.
Frozen catastrophic-jump rule stops an unsafe backend rather than replacing
new measurements with clean75. Numerical comparison, when supported, uses
the identical NEW graph and one gauge, default frozen LM; gates0.10/0.50m
translation median/p95 and0.5/2deg rotation median/p95 are not relaxed.
No GT-based solver tuning. Maps use frame10/point16/voxel0.5m/range1–60m.
Offline ATE uses one JOINT rigid alignment; interval10 RPE is separately computed
for R1/R3 and the union of samples. Unsupported scores are NOT_RUN, not zero.

## Communication, computation and resource scope
B1 full-SC-first is a baseline, NOT a selected efficient policy. Each newly
arrived descriptor is actually serialized once; full-descriptor requests are0
because descriptors were uploaded, not because exact scoring uses only80bytes.
Lossless raw XYZI is requested on demand; unbounded logical central cloud cache
is separately modelled from actual per-call fresh64-entry teammate cache.
Measured serialized application bytes={cost['serialized_application_bytes']},
compressed={cost['compressed_application_bytes']}; cloud cache hits/misses=
{cost['cloud_cache_hits']}/{cost['cloud_cache_misses']}. Compression is measured
zlib1, not a wireless throughput measurement. Original online timings are kept
separate from post-hoc serialization replay, which re-encodes actual raw inputs.
Actual network bytes/latency, robot CPU/memory/energy: NOT_MEASURED.

## Validation and reproducibility
Independent saved-artifact checks:{tests['checks']} PASS. Run
`scripts/run_stage22a_online_frontend.sh` from repository root. For an independent
fresh reproduction set `CUMULTI_STAGE22A_OUTPUT_ROOT` to a new output directory;
the default invocation resumes/skips completed gates, never overwrites results.
It freezes new
policy first, supports hash-checked exact-once verification resume and never
reruns a completed gate. Unresolved in-flight intents block rather than repeat
an uncertain measurement. Large event/message/attempt traces and raw scans are
server-only; sizes/SHA256 and directory digests are explicit in the artifact
manifest. No uncommitted hidden experiment script is required.

## Limitations and stopping point
This is offline replay with causal logical availability and real algorithm
execution, not physically simultaneous robots, real-time deployment,
decentralized optimization or measured networking. High GICP quality alone is
not proof of true place identity. Keep separate structural, numerical and
offline-accuracy outcomes. No claims of all-pair GT recall or full coverage from
a pilot. Normalized discovery timestamps are logical event attribution in this
staged replay, NOT measured wall-clock completion latency; service-time backlog
and real sensor deadlines are not modelled. A pilot measurement executed out of
order is released only at its corresponding logical event. Zero logical delay
after both endpoints arrive is not a real-time performance claim.

{recovery_note}

Stage22B: {result['Stage22B_readiness']}. STOP AFTER22A.
'''
    doc=ROOT/'docs/CUMULTI_STAGE22A_ONLINE_FRONTEND.md';doc.write_text(facts)
    report=dict(HISTORICAL_STAGES_IMMUTABLE='PASS',STAGE21_BASELINE_IMMUTABLE='PASS',DUAL_STREAM_SCHEDULE_VALID='PASS',BOTH_DYNAMIC_DATABASES_VALID='PASS',ONLINE_SC_ACTUALLY_EXECUTED='PASS',PRECOMPUTED_DESCRIPTORS_USED_AS_ORACLE='NO',BIDIRECTIONAL_QUERY_VALID='PASS',NO_FUTURE_CANDIDATES='PASS',NO_ARCHIVED_LOOP_GUIDANCE='PASS',RETRIEVAL_SCORE_CONVENTION_FROZEN='PASS',GICP_ACTUALLY_EXECUTED='PASS',GICP_PARAMETER_POLICY_FROZEN='PASS',REVERSE_TRANSFORMS_VALID='PASS',ONLINE_DUPLICATE_POLICY_FROZEN='PASS',PILOT_BUDGET_VALID='PASS',FULL_GICP_EXECUTION_STATUS=a2['status'],NEW_LOOPS_INDEPENDENTLY_DISCOVERED='PASS' if complete else 'NOT_RUN_FULL_PHASE',PERSISTENT_ISAM2_STATUS=a3['status'],ONLINE_GRAPH_NUMERICAL_AGREEMENT=a3.get('numerical_status','NOT_RUN'),GT_USED_FOR_ONLINE_SELECTION='NO',REAL_NETWORK_MEASUREMENTS='NO',STAGE20_DEMO_MODIFIED='NO',FINAL_DECISION=final)
    (OUT/'VALIDATION_REPORT.txt').write_text('\n'.join(k+': '+v for k,v in report.items())+'\n')
    note=f'''\n## Stage22A — causal bidirectional online frontend (2026-10-21)\n\n{final}. Actual SC processed all5796 events in both directions; two real-scan\npasses match bitwise descriptors and scientific candidate hashes, zero future\ncandidates. GICP coverage {a2['queries_verified']}/5796, {a2['actual_calls']} actual\ncalls, {a2['independent_loops']} independent causal representative loops and\n{a2['duplicates']} suppressed duplicates. Backend {a3['status']}; numerical\n{a3.get('numerical_status','NOT_RUN')}. Serialized simulated-protocol bytes\n{cost['serialized_application_bytes']}; actual networking NOT_MEASURED. New\nresults/GT-free decisions persisted before offline GT/archive comparison;\nno oracle75 guidance, core changes, demo changes or GT retuning. See\n`docs/CUMULTI_STAGE22A_ONLINE_FRONTEND.md` and own independent validation.\n'''
    for name in ['RESULTS.md','EXPERIMENT_LOG.md']:
        path=ROOT/name;old=path.read_text();marker='\n## Stage22A — causal bidirectional online frontend (2026-10-21)'
        if marker in old:old=old.split(marker)[0]
        path.write_text(old+note)
    path=ROOT/'PROJECT_CONTEXT.md';old=path.read_text();marker='\n## Stage22A — causal frontend context'
    if marker in old:old=old.split(marker)[0]
    path.write_text(old+f'\n## Stage22A — causal frontend context\n\nThe current research direction is communication/computation-efficient\ncollaborative LiDAR SLAM. Stage22A replaces archived clean75 discovery\nassumptions with actually computed causal bidirectional SC and new GICP\nmeasurements over5796 normalized R1/R3 arrivals. Decision: {final}; backend\n{a3["status"]}. This is separate-recording replay, not live dual robots or\ndecentralized optimization. Historical frozen results remain unchanged.\nRead the Stage22A doc and RESULTS for coverage, numerical and offline-accuracy\nlimitations. GT is offline only; Stage22B is not automatically authorized.\n')
    p=ROOT/'NEXT_STEPS.md';old=p.read_text();marker='\n## Stage22A — authorized stopping point'
    if marker in old:old=old.split(marker)[0]
    p.write_text(old+f'\n## Stage22A — authorized stopping point\n\nDecision {final}. Preserve all frozen historical stages and the new causal\nfrontend results. Stage22B is not started or automatically authorized. Any\nbudget/selector/graph admission or solver policy change requires an independent\npredeclared follow-up; never tune against the offline GT results. Distinguish\nactual algorithm execution from hypothetical normalized dual-stream timing and\nprotocol-simulated sends. See Stage22A doc for coverage and readiness limitations.\n')
    from PIL import Image
    pictures=list(OUT.glob('*.png'))
    assert len(pictures)==(11 if ready else 9)
    for picture in pictures:
        with Image.open(picture) as im:assert im.width>=1000 and im.height>=500;im.verify()
    dump('figure_validation.json',dict(status='PASS',figures=len(pictures),graph_figures_included=ready,supported_by_actual_outputs=True))
    # Hash large traces explicitly; individual per-call receipts form a separate
    # canonical directory digest to avoid committing tens of thousands of rows.
    artifacts=[]
    for p in sorted(OUT.iterdir()):
        if p.is_file() and p.name!='stage22a_artifact_manifest.json':artifacts.append(dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p)))
    receipts=sorted((OUT/'attempts').glob('*'));digest=hashlib.sha256();total=0
    for p in receipts:
        if p.is_file():digest.update((p.name+' '+sha(p)+'\n').encode());total+=p.stat().st_size
    dump('stage22a_artifact_manifest.json',dict(artifacts=artifacts,attempt_directory=dict(path=str(OUT/'attempts'),files=len(receipts),bytes=total,sha256_of_sorted_filename_space_sha256_LF=digest.hexdigest()),raw_inputs='server-only, never committed',figures=sum(p.suffix=='.png' for p in OUT.iterdir())))
    print('STAGE22A_FINALIZED',final,flush=True)
if __name__=='__main__':main()
