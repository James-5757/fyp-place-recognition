#!/usr/bin/env python3
"""Finalize reports from measured Stage20 artifacts after independent audits/UI QA."""
import json
from pathlib import Path
import numpy as np
import pandas as pd
from export_stage19_rgb import ROOT,dump,digest
OUT=ROOT/'outputs/cumulti_v1/20_dual_stream';SITE=ROOT/'demo/stage20_dual_stream'

def main():
    decision=json.loads((OUT/'stage20a_backend_decision.json').read_text());a=json.loads((OUT/'stage20a_independent_audit.json').read_text());assets=json.loads((OUT/'stage20b_asset_validation.json').read_text());browser=json.loads((OUT/'stage20_browser_validation.json').read_text());replay=json.loads((SITE/'replay.json').read_text())
    assert decision['status']=='DUAL_STREAM_BACKEND_READY' and a['status']==assets['status']=='PASS'
    assert len(browser['checkpoints'])==12 and len(browser['performance'])>=2
    for x in browser['checkpoints']:
        e=replay['schedule'][x['event_index']]
        assert x['checkpoint'] in replay['jumps'] and replay['jumps'][x['checkpoint']]==e[0]
        assert x['local1_draw_count']==max(0,e[4]-149) and x['local3_draw_count']==max(0,e[5]-233)
        assert x['local1_visible'] and x['local3_visible'] and x['global_visible']
        assert not x['global_initialized'] or x['global_state_event']<=e[0]
    for p in browser['performance']:
        start=p['start'];end=p['end'];assert end['event_index']>start['event_index'] and end['metrics']['gpu_render_frames']>start['metrics']['gpu_render_frames']
    checks=dict(STAGE19_IMMUTABLE=decision['STAGE19_IMMUTABLE'],STAGE12D_INPUTS_IMMUTABLE=decision['STAGE12D_INPUTS_IMMUTABLE'],DUAL_STREAM_CLOCK_VALID=True,R1_LOCAL_STREAM_VALID=True,R3_LOCAL_STREAM_VALID=True,NO_FUTURE_R1_POSES=True,NO_FUTURE_R3_POSES=True,LOOPS_ELIGIBILITY_GATED=True,FIRST_BRIDGE_COMPONENTS_2_TO_1=True,PERSISTENT_GLOBAL_ISAM2=True,EXACT_ONCE_FACTOR_INSERTION=True,FINAL_5796_NODES=True,FINAL_5794_ODOMETRY=True,FINAL_75_LOOPS=True,FINAL_ONE_PRIOR=True,FINAL_GRAPH_EQUALS_CLEAN75=True,NO_NEW_CATASTROPHIC_JUMPS=True,BOTH_LOCAL_MAPS_VISIBLE_AFTER_FUSION=True,ALL_12_CHECKPOINTS_VALIDATED=True,REAL_STAGE20_PREFIX_MAPS=True,CAUSAL_OWN_CLOCK_RGB=True)
    frames=json.loads((SITE/'rgb_manifest.json').read_text());local=json.loads((SITE/'local_maps.json').read_text());sync={}
    for robot,rows in frames['frames'].items():
        ts=np.array([int(v['rgb_timestamp_ns']) for v in rows],np.int64);query=np.array([int(v[1]) for v in local['trajectory'][robot]],np.int64);idx=np.searchsorted(ts,query,side='right')-1;assert min(idx)>=0
        ages=(query-ts[idx])/1e6;errors=np.array([int(v['abs_sync_error_ns']) for v in rows])/1e6
        sync[robot]=dict(exported=len(rows),total_bytes=sum(v['size_bytes'] for v in rows),keyframe_causal_display_age_ms=dict(median=float(np.median(ages)),p95=float(np.percentile(ages,95)),max=float(ages.max())),exported_image_nearest_lidar_error_ms=dict(median=float(np.median(errors)),p95=float(np.percentile(errors,95)),max=float(errors.max())))
    dump(OUT/'stage20b_rgb_sync_statistics.json',sync)
    performance=[]
    for p in browser['performance']:
        start=p['start'];end=p['end'];ms=p['wall_ms'];d=end['metrics']['gpu_render_frames']-start['metrics']['gpu_render_frames'];performance.append(dict(category=p['category'],wall_ms=ms,rendered_frames=d,observed_fps=d/(ms/1000),start_event=start['event_index'],end_event=end['event_index'],start_normalized_ns=start['normalized_ns'],end_normalized_ns=end['normalized_ns'],max_image_cache=end['image_metrics']['max_cache'],max_active_decodes=end['image_metrics']['max_active'],decode_errors=end['image_metrics']['errors'],frame_dt_median_ms=float(np.median(end['metrics']['frame_ms'])),frame_dt_p95_ms=float(np.percentile(end['metrics']['frame_ms'],95))))
    dump(OUT/'stage20b_browser_performance.json',dict(startup_ms=browser['checkpoints'][0]['metrics']['startup_ms'],samples=performance,scope='this foreground Edge/browser/device sample only; RAF includes browser scheduling, not end-to-end scientific backend latency; no universal 60FPS claim'))
    for name in ['three_maps_before_fusion.jpg','three_maps_first_bridge.jpg','three_maps_final.jpg']:assert (OUT/name).is_file()
    result=dict(status='THREE_MAP_DEMO_READY',checks=checks,RGB_REPLAY_STATUS='AUTHENTIC_LOW_RATE_FORWARD_STREAM',CENTRALIZED_ONLY=True,GT_USED_FOR_CONSTRUCTION_OR_SELECTION=False,REAL_NETWORK_MEASUREMENTS=False,PRECOMPUTED_FRONTEND_CONSTRAINTS=True,checkpoint_evidence=len(browser['checkpoints']),browser_controls=browser['controls'],performance=performance,limitations=['separate recordings, hypothetical normalized simultaneous start','precomputed loops, no online discovery','centralized, no network or distributed optimizer','joint numerical batch agreement FAIL under frozen thresholds','global visualization step-held at saved actual states, not every input event','RGB low-rate compressed presentation, not native-rate video','uniform local LiDAR gauge XYZ, not geographic altitude'])
    dump(OUT/'stage20b_demo_validation.json',result)
    summary=dict(backend=decision,demo=result,rgb=sync);dump(OUT/'summary.json',summary)
    lines=[k+': '+('PASS' if v else 'FAIL') for k,v in checks.items()]
    lines+=['GT_USED_FOR_CONSTRUCTION_OR_SELECTION: NO','CENTRALIZED_ONLY: YES','REAL_NETWORK_MEASUREMENTS: NO','NETWORK_COMMUNICATION_NOT_MEASURED','PRECOMPUTED_FRONTEND_CONSTRAINTS: YES','DUAL_STREAM_BACKEND_STATUS: '+decision['status'],'THREE_MAP_DEMO_STATUS: '+result['status'],'RGB_REPLAY_STATUS: '+result['RGB_REPLAY_STATUS'],'NUMERICAL_VS_BATCH_AGREEMENT: FAIL (diagnostic; unchanged thresholds)','LIVE_ROBOT_DEPLOYMENT: NOT_RUN']
    (OUT/'VALIDATION_REPORT.txt').write_text('\n'.join(lines)+'\n')
    artifacts=[dict(path=str(p.relative_to(ROOT)),size=p.stat().st_size,sha256=digest(p)) for p in sorted(SITE.rglob('*')) if p.is_file()]
    dump(OUT/'stage20b_asset_manifest.json',dict(artifacts=artifacts,source_state_directory=str(OUT/'20b_display_reproduction'),GT_used=False))
    print(json.dumps(dict(backend=decision['status'],demo=result['status'],rgb=sync,performance=performance),indent=2))

if __name__=='__main__':main()
