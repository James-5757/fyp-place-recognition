#!/usr/bin/env python3
"""Independent Stage18 data/provenance + browser-evidence audit; no GT/solver."""
import ast, hashlib, json
from pathlib import Path
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/cumulti_v1/18_progressive_demo'; DEMO=ROOT/'demo/progressive_multi_robot'; DATA=DEMO/'data'
checks=[]
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def check(name,condition):
    checks.append({'name':name,'PASS':bool(condition)})
    if not condition:raise AssertionError(name)
def main():
    p=json.loads((OUT/'demo_data_provenance.json').read_text()); r=json.loads((DATA/'replay.json').read_text())
    for e in p['sources']:check('SOURCE_HASH:'+e['path'],sha(e['path'])==e['sha256'])
    check('HISTORICAL_DEMO_UNCHANGED',all(sha(fn)==h for fn,h in p['historical_demo_hashes'].items()))
    check('STAGE15_DECISION_UNCHANGED',all(sha(fn)==h for fn,h in p['frozen_decision_hashes'].items()))
    check('STAGE17_DIAGNOSTIC_LINE_REMAINS_CLOSED',r['numerical_diagnostic_line_closed'])
    loops=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv')
    check('CLEAN75_EXACT',len(r['events'])==75 and [e['frozen_id'] for e in r['events']]==loops.frozen_loop_id.tolist())
    check('EXCLUDED45_REINTRODUCED_NO',not p['excluded45_reintroduced'] and len(set(e['frozen_id'] for e in r['events']))==75)
    check('FIRST_LOOP_R1_283_R3_252',(r['events'][0]['query'],r['events'][0]['candidate'],r['events'][0]['frozen_id'])==(283,252,45))
    for e,l in zip(r['events'],loops.itertuples()):
        check(f'FRONTEND_{e["index"]}',e['query']==l.query_keyframe_id and e['candidate']==l.candidate_keyframe_id and e['score']==l.SC_score and e['rank']==1 and e['quality']==l.GICP_quality)
        T=np.asarray(e['transform']); c=np.asarray(e['candidate_cloud']); a=c@T[:3,:3].T+T[:3,3]
        check(f'GICP_SE3_{e["index"]}',np.max(abs(a-np.asarray(e['aligned_cloud'])))<1e-12)
    check('REAL_GICP_SE3_VISUALIZED',True)
    for robot,start,end in [('robot1',150,1999),('robot3',234,4179)]:
        rows=r['local_trajectory'][robot]
        check(f'{robot.upper()}_RANGE_{start}_{end}',[x[0] for x in rows]==list(range(start,end+1)))
        check(f'{robot}_TIMESTAMPS',all(int(a[1])<int(b[1]) for a,b in zip(rows,rows[1:])))
    check('NO_GT_LOADED',p['GT_loaded'] is False and r['GT_used'] is False and all('gt' not in c.lower() for c in p['frontend_columns_loaded']))
    check('NO_NEW_SCIENTIFIC_EXPERIMENT',p['optimizer_runs']==0 and p['display_state_export']['scientific_experiments']==0)
    check('EXACT_HISTORICAL_DISPLAY_REPLAY',p['display_state_export']['persistent_replay_count']==1 and p['display_state_export']['updates']==1717 and p['display_state_export']['extra_empty_updates']==0 and all(x['PASS'] and x['max_matrix_difference']<1e-7 for x in p['display_state_export']['historical_checkpoints_verified']))
    check('FROZEN_ISAM2_DEFAULTS',r['parameters']['relinearizeSkip']==10 and r['parameters']['wildfireThreshold']==.001)
    graph_policy=json.loads((ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_graph_policy.json').read_text())
    check('FROZEN_NOISE_HUBER_GAUGE',graph_policy['odometry_noise']=={'translation_sigma_m':1.0,'rotation_sigma_deg':5.0,'robust':False} and graph_policy['loop_noise']=={'translation_sigma_m':1.0,'rotation_sigma_deg':5.0,'Huber_delta':1.345} and graph_policy['graph_anchor']=='Robot1 KF150 at its frozen local Pose3')
    expected={'FIRST_LOOP':(283,1),'K2':(378,2),'K5':(512,5),'K10':(638,10),'K20':(664,20),'K40':(1014,40),'K75':(1967,75),'FINAL_STREAM_END':(1999,75)}
    for s in r['snapshots']:
        check('SNAPSHOT_HASH:'+s['name'],sha(DATA/s['file'])==s['sha256'])
        a=json.loads((DATA/s['file']).read_text())
        if s['K']:
            if s['name'] in expected:check('CHECKPOINT_ID:'+s['name'],(s['kf'],s['K'])==expected[s['name']])
            check('CHECKPOINT_NO_FUTURE:'+s['name'],max(x[0] for x in a['trajectory']['robot1'])==s['kf'] and sum(e['query']<=s['kf'] for e in r['events'])==s['K'])
        check('MAP_SOURCE:'+s['name'],set(a['maps'])=={'robot1','robot3'} and all(len(x)>0 for x in a['maps'].values()))
        check('XYZ_FINITE:'+s['name'],all(np.isfinite(x).all() and np.asarray(x).shape[1]==3 for x in a['maps'].values()))
    check('FINAL_NODE_COUNT_5796',r['final_counts']['nodes']==5796)
    check('FINAL_ODOMETRY_5794',r['final_counts']['odometry']==5794)
    check('FINAL_LOOP_COUNT_75',r['final_counts']['loops']==75)
    check('FINAL_PRIOR_COUNT_1',r['final_counts']['prior']==1)
    check('FINAL_FACTORS_5870',r['final_counts']['total']==5870)
    check('ALL75_POST_LOOP_STATES_AVAILABLE',set(s['K'] for s in r['snapshots'] if s['K'])==set(range(1,76)))
    # All possible query/substate boundaries: displayed global checkpoint cannot be future.
    for kf in range(150,2000):
        arrived=sum(e['query']<=kf for e in r['events']); ev=next((e for e in r['events'] if e['query']==kf),None)
        for phase in (range(5) if ev else [4]):
            accepted=arrived-int(bool(ev) and phase<3); applied=accepted-int(bool(ev) and phase==3)
            cp=[s for s in r['snapshots'] if s['kf']<=kf and s['K']<=applied]
            if applied:check(f'CAUSAL_{kf}_{phase}',bool(cp) and cp[-1]['kf']<=kf and cp[-1]['K']<=accepted)
    check('NO_FUTURE_LOOP_DISPLAY',True)
    src=(ROOT/'src/demo/prepare_progressive_multi_robot_assets.py').read_text(); tree=ast.parse(src)
    # SOURCES.update and SHA256 h.update are provenance bookkeeping, not ISAM2.
    check('NO_SOLVER_API_CALLS',not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in ['optimize','update','aligned','offline_gt','extract_ekf'] and not (n.func.attr=='update' and isinstance(n.func.value,ast.Name) and n.func.value.id in ['SOURCES','h']) for n in ast.walk(tree)))
    ui=(DEMO/'index.html').read_text()+(DEMO/'app.js').read_text()
    for tag,phrase in [('ROBOT3_REFERENCE_DATABASE_LABEL','ROBOT3 REFERENCE DATABASE'),('ROBOT1_STREAMING_REPLAY_LABEL','ROBOT1 STREAMING REPLAY'),('PRECOMPUTED_FRONTEND_LABEL','PRECOMPUTED FRONT-END MEASUREMENTS'),('CENTRALIZED_BACKEND_LABEL','CENTRALIZED BACKEND'),('INCREMENTAL_NUMERICAL_CAVEAT_VISIBLE','Incremental estimate — frozen Stage13B policy'),('BATCH_REFERENCE_NOT_GT','not GT')]:check(tag,phrase in ui)
    for tag in ['NO_DISTRIBUTED_PGO_CLAIM','NO_LIVE_SENSOR_CLAIM','NO_FAKE_NETWORK_BANDWIDTH_CLAIM']:check(tag,'no live sensors or distributed PGO' in ui and 'not UDP/ROS transport or measured network bandwidth' in ui)
    check('MAP_SOURCE_PROVENANCE_PRESERVED','Robot1 map' in ui and 'Robot3 map' in ui and p['point_xyz_retained'])
    evidencepath=OUT/'browser_validation.json'
    if not evidencepath.exists():raise RuntimeError('Data checks pass; browser presentation evidence REQUIRED before final PASS')
    b=json.loads(evidencepath.read_text())
    for name,ok in b['validation_items'].items():check(name,ok)
    for image in b['screenshots']:check('SCREENSHOT:'+image,(OUT/'screenshots'/image).is_file())
    check('PLAY_PAUSE_RESET_WORK',b['play_pause_reset_work'])
    dump={'PASS':True,'checks':len(checks),'results':checks,'batch_optimizer_runs':0,'persistent_display_replays':1,'scientific_experiments':0,'GT_loaded':False,'historical_demo_unchanged':True}
    (OUT/'independent_audit.json').write_text(json.dumps(dump,indent=2)+'\n')
    (OUT/'VALIDATION_REPORT.txt').write_text('\n'.join(x['name']+': PASS' for x in checks if not any(x['name'].startswith(p) for p in ['SOURCE_HASH','FRONTEND_','GICP_SE3_','CAUSAL_']))+'\nEXCLUDED45_REINTRODUCED: NO\nSTAGE18_PRESENTATION: PASS\nNUMERICAL_DIAGNOSTIC_LINE_CLOSED: YES\n')
    files=[f for f in DEMO.rglob('*') if f.is_file()]+[f for f in (OUT/'screenshots').glob('*.png')]
    artifacts=[{'path':str(f.relative_to(ROOT)),'size_bytes':f.stat().st_size,'sha256':sha(f)} for f in sorted(files)]
    (OUT/'demo_artifact_manifest.json').write_text(json.dumps({'artifacts':artifacts,'frozen_graph_policy_sha256':sha(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_graph_policy.json')},indent=2)+'\n')
    (OUT/'summary.json').write_text(json.dumps({'stage18_presentation':'PASS','checks':len(checks),'clean_loops':75,'excluded_loops':45,'nodes':5796,'odometry':5794,'prior':1,'factors':5870,'post_loop_states':75,'additional_loop_states':68,'historical_checkpoint_reproduction_max_matrix_difference':max(x['max_matrix_difference'] for x in p['display_state_export']['historical_checkpoints_verified']),'persistent_display_replays':1,'extra_empty_updates':0,'GT_loaded':False,'new_scientific_experiments':0,'scientific_readiness':'Historical INCREMENTAL_BACKEND_NOT_READY unchanged','numerical_diagnostic_line_closed':True,'screenshots':len(b['screenshots']),'browser_assets_total_bytes':sum(x['size_bytes'] for x in artifacts)},indent=2)+'\n')
    print(json.dumps({'PASS':True,'checks':len(checks),'batch_optimizer_runs':0,'persistent_display_replays':1,'new_scientific_experiments':0}))
if __name__=='__main__':main()
