"""Mechanical report/export summaries. No further optimization after GT."""
from stage23e_common import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def table(df,cols):
    def fmt(x):
        if pd.isna(x):return 'unavailable'
        return f'{x:.6g}' if isinstance(x,(float,np.floating)) else str(x)
    return '| '+' | '.join(cols)+' |\n| '+' | '.join(['---']*len(cols))+' |\n'+'\n'.join('| '+' | '.join(fmt(r[c]) for c in cols)+' |' for r in df.to_dict('records'))+'\n'
def main():
    decision=read(E/'stage23e_decision.json');audit=read(E/'stage23e_independent_artifact_validation.json');gates=read(E/'p6_gate_results.json');comparison=pd.read_csv(E/'p6_pareto_comparison.csv');accuracy=pd.read_csv(E/'p6_graph_accuracy.csv');quality=pd.read_csv(E/'p6_query_quality.csv');factor=pd.read_csv(E/'p6_factor_quality.csv');residual=pd.read_csv(E/'p6_loop_residual_summary.csv');graph_rows=[];latencies=[];artifacts={}
    for short in POLICIES:
        directory=G/short;graph=read(directory/'graph_integrity.json');row=dict(policy=short,**graph)
        if (directory/'online_incremental_vs_batch.csv').exists():row.update({'agreement_'+k:v for k,v in pd.read_csv(directory/'online_incremental_vs_batch.csv').iloc[0].to_dict().items()})
        graph_rows.append(row);trace=pd.read_csv(directory/'online_backend_event_trace.csv');active=trace[trace.status=='FINITE_CENTRAL']
        for field in ['isam2_update_ms','estimate_extract_ms','integrity_audit_ms','state_archive_ms']:
            latencies.append(dict(policy=short,stage=field,samples=len(active),median_ms=float(active[field].median()),p95_ms=float(active[field].quantile(.95)),sum_ms=float(active[field].sum()),scope='host category-conditional measurement, not realtime/network deadline proof'))
        archive=read(directory/'event_trajectory_manifest.json');dump(directory/'event_trajectory_artifact_summary.json',{k:v for k,v in archive.items() if k!='state_index'})
        artifacts[short]=read(directory/'pre_gt_policy_artifacts.json')
    csv(E/'p6_graph_solver_summary.csv',graph_rows);csv(E/'p6_backend_latency_summary.csv',latencies)
    gate_table=pd.DataFrame([dict(policy=r['policy'],**r['checks'],all_original_gates_pass=r['all_original_gates_pass'],ATE_ratio_vs_P0=r['ATE_ratio_vs_P0']) for r in gates['policies']]);csv(E/'p6_gate_results.csv',gate_table)
    labels=pd.read_csv(E/'p6_factor_labels.csv');cross=labels[labels.query_split!=labels.insertion_split].groupby('policy').size().to_dict()
    p5=comparison[comparison.policy=='P5'].iloc[0];p6=comparison[comparison.policy.isin(POLICIES)]
    tradeoff=dict(all_P6_full_query_recall_higher_than_P5=bool((p6.recall>p5.recall).all()),all_P6_full_query_precision_higher_than_P5=bool((p6.precision>p5.precision).all()),any_P6_full_incremental_joint_ATE_lower_than_P5=bool((p6.joint_ATE_RMSE_m<p5.joint_ATE_RMSE_m).any()),all_P6_use_more_bytes_than_P5=bool((p6.application_bytes>p5.application_bytes).all()),interpretation='query-verification gains relative to P5 do not demonstrate better global map accuracy; individual regional/RPE/NN changes remain separate')
    dump(E/'p6_quality_tradeoff_summary.json',tradeoff)
    offsets=[(-45,14),(-5,-20),(30,-28),(-40,35),(65,-8),(80,25)]
    for x,y,file,ylabel in [('application_bytes','recall','application_bytes_vs_recall','Geometric query recall · offline GT'),('application_bytes','joint_ATE_RMSE_m','application_bytes_vs_ATE','Joint rigid ATE RMSE · m · offline GT'),('GICP_receipt_attempts','admitted_factors','attempts_vs_admitted_factors','Admitted graph factors'),('application_bytes','map_NN_median_m','communication_vs_map_consistency','GT-free symmetric NN median · m')]:
        fig,ax=plt.subplots(figsize=(9,5.4));xx=comparison[x]/1e9 if x=='application_bytes' else comparison[x];colors=['#455a64','#de8e35','#2073b5','#40a997','#9966bd','#cd5272']
        for i,row in comparison.iterrows():
            ax.scatter(xx.iloc[i],row[y],s=65,color=colors[i],zorder=3);ax.annotate(row.policy,(xx.iloc[i],row[y]),xytext=offsets[i],textcoords='offset points',fontsize=9,arrowprops=dict(arrowstyle='-',color='#7a8790',lw=.7),bbox=dict(facecolor='white',edgecolor='none',alpha=.8,pad=.5))
        ax.set(xlabel='Application GB (protocol simulated)' if x=='application_bytes' else 'Selected historical GICP receipts',ylabel=ylabel,title='Frozen policy comparison · '+('communication / '+y if x=='application_bytes' else 'verification / factors'));ax.grid(alpha=.2);ax.margins(x=.1,y=.15);fig.text(.5,.012,'RETROSPECTIVE_GT_EXPOSED_RECORDING · no new GICP · no independent validation',ha='center',fontsize=8);fig.tight_layout(rect=[0,.035,1,1]);fig.savefig(E/('presentation_'+file+'.png'),dpi=160);plt.close(fig)
    repair=read(G/'stage23e_engineering_repair.json') if (G/'stage23e_engineering_repair.json').exists() else {}
    engineering_note=('Two pre-optimization engineering failures are retained: the GT-file guard initially rejected a Python source filename containing offline_, then metadata construction duplicated an already-frozen keyword. The guard now permits only reading source .py/.pyc files under src/cumulti while still rejecting GT data. The duplicate metadata keyword was removed. Neither failure called an optimizer or GICP. The original input manifest is preserved; narrowly allowlisted hashes of the two new Stage23E source repairs are recorded separately. Scientific parameters and original sources did not change. The pre-optimization partial directory is preserved, not relabeled a solve.' if repair else 'No pre-optimization engineering repair was needed in this replay. The backend GT guard permits scientific Python source reads while forbidding GT-bearing data.')
    if (E/'stage23e_evaluator_metadata_patch.json').exists():engineering_note+=' A final metadata compatibility fix permits a clean replay with no repair journal (SHA=null). It changes no metric formula or accepted result; the original execution hash stays in the GT-entry file and the source amendment is recorded separately. No optimizer or GT evaluator was rerun for this fix.'
    text='''# Stage23E — P6 graph reconstruction and mapping quality

## Separate final statuses

```json
'''+json.dumps(decision,indent=2)+'''
```

This is **RETROSPECTIVE_GT_EXPOSED_RECORDING**. Independent geographical validation is NOT_RUN and eligibility remains NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE. Artifact correctness, numerical agreement, empirical quality and communication savings are separate decisions. No P6 policy is promoted, no matched-budget experiment is run, and no GICP is executed.

## Frozen inputs and experiment record
Reference Stage23D: `2fb55ad08f56cd74198c175d7116462b84a89427`. The server-only `p6_verified_factors.csv` exists and matches the specified440808bytes and SHA256 `476a490b787a2912e61b9f1e46424990b0786008780c9f7eaca6713799d30bab`, as well as its original artifact manifest. No scheduler rerun was needed. Queue/budget archives, source files, policies, odometry, raw scan hashes and original metric definitions are frozen in `23e_p6_graphs/stage23e_input_manifest.json` with a policy lock. All Stage18–23D scientific artifacts and original dirty work remain untouched; this new report records the experiment without rewriting historical documents.

'''+engineering_note+'''

## Reconstruction and insertion causality
All four isolated runners validate canonical IDs, proper SE(3), receipt SHA256 and CSV digest, exact canonical transform equality, endpoint arrival by discovery, and discovery<=insertion. Factors sort by insertion_event, discovery_event, rank, Robot1KF, Robot3KF. **Only insertion_event controls backend insertion.** Discovery remains query identity and controls the original dev/embargo/heldout classification. Deferred measurements are never inserted retroactively. Same-event processing order is the explicitly requested deterministic order, not an undocumented priority-queue order.

The first causally inserted frozen factor initializes the common frame using S=Xq Zqc inverse(Lc); no GT or highest-quality fallback chooses it. Prior: Robot1KF150, sigma1e-6. Every graph retains5796 nodes and5794 original odometry edges. Rotation/translation sigmas are5degrees/1m, tangent order rotation then translation, and loop Huber1.345. Odometry is unrobustified diagonal noise. One persistent default iSAM2, identical frozen parameter text, one update per connected arrival and zero extra empty updates. Batch LM uses the existing Stage22A/B–Stage12D defaults unchanged. The brief's Stage22D-compatible wording refers to these existing parameter files; no new Stage22D solver was invented.

Graph and solver results:

'''+table(pd.DataFrame(graph_rows),['policy','status','nodes','odometry','loops','factors','first_bridge_discovery_event','first_bridge_insertion_event','new_catastrophic_jumps','agreement_status','agreement_incremental_error','agreement_batch_error','agreement_batch_iterations','agreement_batch_runtime_ms','runtime_including_audit_state_export_maps_s'])+'''
Incremental/Batch pose agreement, same graph and gauge; original limits0.10m median,0.50m p95,0.5degrees median,2degrees p95 are not relaxed:

'''+table(pd.DataFrame(graph_rows),['policy','agreement_status','agreement_translation_median_m','agreement_translation_p95_m','agreement_rotation_median_deg','agreement_rotation_p95_deg'])+'''
A structural PASS does not imply numerical agreement. Similar objective values do not establish pose equality. Objectives across DIFFERENT loop graphs are not direct mapping-accuracy measures. Batch runtime is native optimize time only; whole replay runtime additionally includes event extraction, integrity auditing, archive compression and map generation. Exact latency distributions are in `p6_backend_latency_summary.csv`, not an end-to-end real-time claim.

## Every-event trajectories and maps
Every one of5796 events per successful policy has a real float64 Nx4x4 state in the server-only ZIP, with entry shape/hash and prefix last IDs in the event trace. Rows are Robot1 ascending then Robot3 ascending. Before the first bridge the archive is explicitly LOCAL_ONLY: two independent local frames, not an overlaid common graph. After the bridge snapshots are the actual current iSAM2 estimates, without interpolation, future-state morphing or extra updates. CRC of all ZIP entries is checked; the independent auditor additionally inspects all insertion-event matrices, pre/at-bridge states, periodic500 states and FINAL. Backend runtime checks proper SE(3) at every event.

Actual incremental AND Batch maps use the same580 original scans, frame stride10, point stride16, range1–60m and per-source voxel0.5m. Each maps/ directory contains incremental_robot1.ply, incremental_robot3.ply, incremental_merged.ply, batch_robot1.ply, batch_robot3.ply and batch_merged.ply, hashed in online_map_manifest.json. Original metric XYZ and gauge remain intact: no independent best fit, GT transform, Z flattening, scaling or cosmetic adjustment. Separate per-source clouds preserve the blue/orange rendering provenance convention; merged XYZ is an additional union voxel map. No frozen demo was edited.

## Offline evaluation ordering and definitions
Only after all four backend results, factor schedules, trajectories, event archives and maps were hashed in `all_graphs_pre_gt_freeze.json` was `offline_evaluation_entry.json` written and GT opened. There is no optimization after seeing GT. Frozen P0/P5 values are COPIED from Stage22B tables, never re-evaluated or overwritten.

Query precision/recall and admitted precision/admission recall use the original discovery event and the frozen causal available-positive d_xy<5m denominator. Original receipt correctness is independently checked against those GT pairs; query-level success includes geometrically accepted duplicates, while admission is reported separately. Counts are not collapsed by insertion event. Original split: dev0–999, embargo1000–1099, heldout1100–5795.

Factors crossing query-discovery and insertion split boundaries (descriptive only):

```json
'''+json.dumps(cross,indent=2)+'''
```

ATE uses precisely the Stage22B one joint global rigid alignment for each final system trajectory. Robot/scope/region subsets reuse that same alignment; there is no independent robot, region or heldout fit. Heldout/region accuracy describes subsets of the FINAL frozen estimate, not the causal estimate available at query time. Frozen Stage22B does not provide heldout graph ATE: no new baseline value is fabricated. The original gate uses full incremental joint ATE ratio, as in Stage22B; additional heldout graph values are diagnostic.

RPE interval10 uses the same SE(3) relative-error definition; samples are calculated separately for each robot and combined by union, not by copying a global value into robot rows. Both interval endpoints must lie within the requested scope/region. Empty samples are unavailable, not zero. The eleven pre-existing regions are the seven temporal groups and four non-GT local-odometry bin pairs. Region definitions are verified through the frozen Stage23D asset manifest. Existing sensor/reference-frame limitations are retained; no new extrinsic or GT convention correction is introduced.

Full query geometric metrics:

'''+table(quality[(quality.scope=='full')&(quality.direction=='ALL')],['policy','queries','positive_queries','geometric_accepted','geometric_true','false_geometric','precision','recall'])+'''
Heldout query geometric metrics:

'''+table(quality[(quality.scope=='heldout')&(quality.direction=='ALL')],['policy','queries','positive_queries','geometric_accepted','geometric_true','precision','recall'])+'''
Full admitted-factor metrics:

'''+table(factor[(factor.scope=='full')&(factor.direction=='ALL')],['policy','admitted','admitted_true','false_admitted','admitted_precision','admission_recall'])+'''
Incremental final-trajectory accuracy, full and heldout subsets (joint RPE means union of robot samples):

'''+table(accuracy[(accuracy.condition=='incremental')&(accuracy.scope.isin(['full','heldout']))],['policy','scope','robot_id','nodes','ATE_RMSE_m','RPE_samples','RPE_translation_RMSE_m','RPE_rotation_RMSE_deg'])+'''
Per-region and Batch metrics remain fully available in the required CSVs. Loop residual summaries are GT-free measurement residuals, not GT pose errors:

'''+table(residual,['policy','condition','translation_median_m','translation_p95_m','rotation_median_deg','rotation_p95_deg','minimum_diagnostic_huber_weight'])+'''
## Resource/quality comparison and unchanged gates
Attempt savings and application-byte savings are relative to frozen P0. Extra bytes and ATE differences are relative to frozen P5. These are selected historical receipt workloads and protocol-simulated application bytes, not new GICP executions, physical networking or a measured complete frontend replay runtime.

'''+table(comparison,['policy','GICP_receipt_attempts','admitted_factors','application_bytes','application_byte_reduction_vs_P0','additional_bytes_vs_P5','recall','precision','joint_ATE_RMSE_m','ATE_difference_vs_P5_m','map_NN_median_m','map_NN_p95_m'])+'''
Observed tradeoff, generated directly from the comparison table:

```json
'''+json.dumps(tradeoff,indent=2)+'''
```

The query-level improvements relative to P5 must not be conflated with joint ATE improvement. Regional/RPE/NN diagnostics can move differently, and point-cloud overlap alone is not accuracy. D0's numerical agreement failure additionally limits interpretation of its incremental estimate. The unchanged primary noninferiority reference is P0, not a retrospectively substituted P5 target.

The original gate values are retained: attempt reduction>=25%, byte reduction>=20%, recall drop<=0.02 and precision drop<=0.01 in BOTH full and heldout scopes, full incremental joint ATE ratio<=1.05, no new catastrophic jump, proper factor convention, and original same-graph agreement. This is an empirical exposed-recording check, not a powered statistical noninferiority result.

'''+table(gate_table,['policy','attempt_reduction','application_byte_reduction','recall','precision','joint_ATE','no_new_catastrophic_jump','proper_factor_convention','incremental_batch_agreement','all_original_gates_pass','ATE_ratio_vs_P0'])+'''
The four Pareto plots show bytes/recall, bytes/ATE, receipt attempts/admitted factors, and bytes/map-NN. Readable `presentation_*.png` copies use separated callouts for nearly coincident points; the original evaluation plots and all numerical outputs remain unchanged. Pareto labels are descriptive only, not a selection/promotion rule. Global symmetric NN median/p95 includes non-overlap and cannot be read as literal absolute map alignment error. More factors or coverage bins alone do not demonstrate better maps. A lower ATE accompanied by higher cost than P5 is only a possible later MATCHED-BUDGET hypothesis; numerical and geometric gate failures must still be read. No such experiment starts here.

## Validation and limitations
Independent artifact audit status: '''+audit['status']+'''. It rebuilds final graphs without solving, checks chronological factor ledger equality, exact-once counts/connectivity, receipt/measurement convention, source timestamps and final pose schemas, recomputed objectives, same-graph pose statistics, all archive CRCs, representative real states and map artifact hashes. Original catastrophic rules are evaluated during EVERY backend event. Failures remain failures; there is no noise, solver, retrieval, GICP, threshold or GT-based retuning.

This is one previously exposed pair of separate recordings with deficient positive heldout dual-progress/direction coverage. Native sparse replay performance is not universal solver superiority or real-time SLAM. Offline metrics are diagnostic; graph structural validity and protocol savings do not establish accuracy, independent generalization, physical network efficiency or deployability. Stage23D and prior scientific artifacts remain frozen. No live/distributed demo or matched-budget study is started.

## Reproduction and exact locations

```bash
bash scripts/run_stage23e_p6_backend.sh
bash scripts/run_stage23e_evaluation.sh
```

The backend script resumes only verified completed per-policy results and creates the all-four pre-GT freeze. The evaluation script requires that freeze and preserves an existing completed evaluation. The isolated environment remains `/home/cas/.venvs/fyp_gtsam`; scientific helper files are imported, their entrypoints are NOT invoked against historical output paths.

Graph root: `/home/cas/fyp_place_recognition/outputs/cumulti_v1/23e_p6_graphs/`, children P6_D0/P6_D1/P6_D2/P6_D3. Evaluation root: `/home/cas/fyp_place_recognition/outputs/cumulti_v1/23e_p6_evaluation/`. Large full event ZIPs and PLY maps stay server-only with SHA256/size manifests; lightweight scientific CSVs, policy/freeze/validation reports and figures are versioned. This report is the new experiment-log entry; frozen history/context files are preserved.

STOP after Stage23E. Any next investigation requires separate authorization and must preserve the original gates. Independent recording eligibility remains NO_GO; independent validation is NOT_RUN.
'''
    (ROOT/'docs/CUMULTI_STAGE23E.md').write_text(text)
    dump(E/'stage23e_final_artifact_manifest.json',{str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p)) for folder in [G,E] for p in folder.rglob('*') if p.is_file() and p.name!='stage23e_final_artifact_manifest.json'})
    print('STAGE23E_REPORT_COMPLETE',flush=True)
if __name__=='__main__':main()
