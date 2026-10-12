#!/usr/bin/env python3
"""Independent read-only lineage/UI-evidence audit; no optimizer or GT load."""
import hashlib, json, subprocess
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
DATA=ROOT/'demo/progressive_multi_robot/data'
OUT=ROOT/'outputs/cumulti_v1/18_1_presentation_upgrade'
BASE='0447a62210d46a685285b8f6d649d49fd880960d'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return json.loads(Path(p).read_text())
def main():
    checks={}
    def gate(name,value):checks[name]='PASS' if value else 'FAIL'
    original=subprocess.check_output(['git','ls-tree','-r','--name-only',BASE,'demo/progressive_multi_robot/data'],cwd=ROOT,text=True).splitlines()
    for f in original:
        expected=subprocess.check_output(['git','show',f'{BASE}:{f}'],cwd=ROOT)
        gate('FROZEN_ASSET_'+Path(f).name,(ROOT/f).read_bytes()==expected)
    provenance=load(OUT/'presentation_data_provenance.json')
    for f,h in provenance['old_demo'].items():gate('OLD_DEMO_'+str(Path(f).relative_to(ROOT)),sha(f)==h)
    for f,h in provenance['frozen_decisions'].items():gate('FROZEN_DECISION_'+Path(f).name,sha(f)==h)
    pose=load(DATA/'robot1_causal_pose_stream.json');rows=pose['rows'];a=np.array([r[2:] for r in rows]);v=load(OUT/'pose_stream_validation.json')
    gate('CAUSAL_POSES_1850',len(rows)==1850 and [r[0] for r in rows]==list(range(150,2000)))
    gate('POSES_FINITE_UNIT_QUATERNIONS',np.isfinite(a).all() and np.max(abs(np.linalg.norm(a[:,3:],axis=1)-1))<1e-12)
    gate('ORIGINAL_TIMESTAMPS_PRESERVED',all(isinstance(r[1],str) for r in rows) and all(int(rows[i][1])<int(rows[i+1][1]) for i in range(len(rows)-1)))
    gate('EIGHT_FROZEN_CHECKPOINTS_REPRODUCED',v['PASS'] and len(v['checkpoints'])==8 and all(x['PASS'] and x['max_matrix_difference']<1e-7 for x in v['checkpoints']))
    gate('ONE_FROZEN_DISPLAY_EXTRACTION',v['updates']==1717 and v['loops']==75 and v['new_scientific_experiments']==0 and not v['GT_used'])
    gate('POSE_STREAM_HASH',sha(DATA/'robot1_causal_pose_stream.json')==v['stream_sha256'])
    replay=load(DATA/'replay.json');events=replay['events']
    gate('CLEAN75_EXACT',len(events)==75 and provenance['clean75_sha256']==sha(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv'))
    gate('FIRST_ENCOUNTER_R1_283_R3_252',events[0]['query']==283 and events[0]['candidate']==252 and events[0]['frozen_id']==45)
    bounds=load(DATA/'display_bounds.json')
    for f,b in bounds.items():
        d=load(DATA/f)
        for robot in ['robot1','robot3']:
            xyz=np.array(d['maps'][robot]+[r[2:5] for r in d['trajectory'][robot]])
            gate('ACTUAL_XYZ_BOUNDS_'+f+'_'+robot,np.array_equal(np.r_[xyz.min(0),xyz.max(0)],b['robots'][robot]))
    vendor=ROOT/'demo/progressive_multi_robot/vendor'
    gate('PINNED_OFFLINE_THREE_R170',"const REVISION = '170'" in (vendor/'three.module.js').read_text() and 'MIT License' in (vendor/'LICENSE_THREE').read_text())
    source=(ROOT/'demo/progressive_multi_robot/replay_controller.js').read_text()
    view=(ROOT/'demo/progressive_multi_robot/map3d_view.js').read_text()
    gate('EVENT_LOOKUPS_PREINDEXED','eventsByKF=new Map' in source and 'eventsByIndex=new Map' in source)
    gate('TRAJECTORY_LOOKUPS_PREINDEXED','poses.byKF=new Map' in source and 'd.lookup[robot]=new Map' in source)
    gate('SNAPSHOT_BOUNDS_PRECOMPUTED',len(bounds)==79 and 'this.bounds' in view)
    gate('NO_MAP_VERTEX_INTERPOLATION','Vertex buffers are never interpolated' in view and '.position.fromArray(previousPose' in view)
    browser=load(OUT/'browser_validation.json')
    for name,value in browser['validation_items'].items():checks[name]=value
    screenshots=list((OUT/'screenshots').glob('*.png'))
    gate('ELEVEN_SCREENSHOTS',all((OUT/'screenshots'/f).is_file() for f in browser['screenshots']) and len(browser['screenshots'])==11)
    perf=load(OUT/'performance_after.json')
    gate('NORMAL_PLAYBACK_PREFETCH_WORKS',perf['counters']['prefetch_hits']>0 and perf['counters']['state_fetch_waits']==0)
    gate('NO_OBVIOUS_NORMAL_PLAYBACK_FETCH_STALL',perf['counters']['state_fetch_waits']==0)
    gate('CANVAS_RESIZED_ONLY_ON_REAL_RESIZE',perf['counters']['canvas_resizes']==0)
    gate('STATIC_MAP_NOT_REDRAWN_EVERY_ANIMATION_FRAME',perf['counters']['map_redraws']==0 and perf['counters']['graph_redraws']==0 and perf['counters']['geometry_builds']<perf['counters']['gpu_render_frames'])
    sys=load(OUT/'performance_after_system.json')
    gate('SYSTEM_STATIC_MAP_CACHED',sys['counters']['map_redraws']<sys['counters']['dom_writes'] and sys['counters']['canvas_resizes']<sys['counters']['canvas_redraws'])
    checks['NO_OBVIOUS_PRESENTATION_STUTTER']='NOT_VERIFIED_AUTOMATION_RAF_THROTTLED'
    checks['TARGET_50_60_FPS']='NOT_VERIFIED_AUTOMATION_RAF_THROTTLED'
    checks.update(EXCLUDED45_REINTRODUCED='NO',GT_USED='NO',LIVE_OR_DISTRIBUTED_BACKEND='NOT_IMPLEMENTED')
    failures=[k for k,x in checks.items() if x=='FAIL']
    result={'functional_and_lineage_status':'PASS' if not failures else 'FAIL','smoothness_acceptance':'PENDING_NORMAL_FOREGROUND_BROWSER_REHEARSAL','checks':checks,'failures':failures,'screenshots':{p.name:{'bytes':p.stat().st_size,'sha256':sha(p)} for p in screenshots}}
    (OUT/'presentation_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    (OUT/'VALIDATION_REPORT.txt').write_text('STAGE18.1 FUNCTIONAL / LINEAGE: '+result['functional_and_lineage_status']+'\nPRESENTATION SMOOTHNESS: NOT VERIFIED (automation RAF ~1 Hz)\n'+'\n'.join(f'{k}: {x}' for k,x in checks.items())+'\n')
    print(result['functional_and_lineage_status'],len(checks),'checks; smoothness acceptance remains explicitly pending')
    if failures:raise SystemExit(1)
if __name__=='__main__':main()
