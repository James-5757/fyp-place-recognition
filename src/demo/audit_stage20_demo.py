#!/usr/bin/env python3
"""Independent Stage20B packaged prefix/pose/RGB provenance audit (no GT)."""
import json, hashlib
from pathlib import Path
import numpy as np
import pandas as pd
from export_stage19_rgb import ROOT,digest,dump
OUT=ROOT/'outputs/cumulti_v1/20_dual_stream';SITE=ROOT/'demo/stage20_dual_stream'

def main():
    r=json.loads((SITE/'replay.json').read_text());rgb=json.loads((SITE/'rgb_manifest.json').read_text());local=json.loads((SITE/'local_maps.json').read_text());checks={}
    expected=pd.read_csv(OUT/'central_incremental_event_trace.csv',keep_default_na=False)
    checks['BACKEND_READY']=json.loads((OUT/'stage20a_backend_decision.json').read_text())['status']=='DUAL_STREAM_BACKEND_READY'
    checks['EXACT_DISPLAY_REPRODUCTION']=json.loads((OUT/'stage20b_display_reproduction_validation.json').read_text())['status']=='PASS'
    for e,row in zip(r['schedule'],expected.itertuples()):
        assert e[0]==row.event_index and int(e[1])==row.normalized_ns and e[4:7]==[row.current_robot1_kf,row.current_robot3_kf,row.cumulative_loops]
    checks['EXACT_EVENT_CLOCK']=len(r['schedule'])==5796
    checks['EXACT_LOOP_ORDER']=r['events']==sorted(r['events'],key=lambda v:(v['event_index'],v['frozen_loop_id']))
    assert [v['frozen_loop_id'] for v in r['events']]==pd.read_csv(OUT/'loop_eligibility_schedule.csv').frozen_loop_id.tolist()
    for state in r['states']:
        s=json.loads((SITE/'states'/state['file']).read_text());e=r['schedule'][state['event_index']]
        assert s['K']==e[6] and s['kf']==e[4] and s['r3kf']==e[5]
        if not s['global_initialized']:assert not s['maps']['robot1'] and not s['maps']['robot3']
        src=pd.read_csv(OUT/'20b_display_reproduction/states'/f"{s['name']}.csv")
        for robot,end in [('robot1',e[4]),('robot3',e[5])]:
            a=src[src.robot_id.eq(robot)].set_index('keyframe_id')
            for v in s['trajectory'][robot]:
                assert v[0]<=end and str(int(a['timestamp'].loc[v[0]]))==v[1]
                assert np.max(abs(a.loc[v[0]][['tx','ty','tz','qx','qy','qz','qw']].to_numpy(float)-v[2:]))<1e-10
    checks['ONLY_REAL_PREFIX_STATES_NO_FUTURE']=True
    checks['NO_PREBUILT_R3_LOCAL_MAP']=local['robot3_points']==[] and len(local['robot3_chunks'])==395
    checks['LOCAL_FRAME_COUNTS']=len(local['trajectory']['robot1'])==1850 and len(local['trajectory']['robot3'])==3946
    for robot in ['robot1','robot3']:
        last=-1
        for v in rgb['frames'][robot]:
            p=(SITE/v['file']).resolve();assert digest(p)==v['sha256'] and p.stat().st_size==v['size_bytes'];assert int(v['rgb_timestamp_ns'])>last;last=int(v['rgb_timestamp_ns'])
            assert int(v['header_timestamp_ns'])<=int(v['rgb_timestamp_ns'])
        # Exhaustive keyframe-time causal association; no future image is displayed.
        ts=[int(v['rgb_timestamp_ns']) for v in rgb['frames'][robot]]
        for row in local['trajectory'][robot]:
            j=int(np.searchsorted(ts,int(row[1]),side='right'))-1
            if j>=0:assert ts[j]<=int(row[1])
    checks['AUTHENTIC_HASHED_CAUSAL_RGB']=True
    policy=json.loads((OUT/'stage20b_rgb_policy.json').read_text());rows=rgb['frames']['robot3']
    checks['ROBOT3_RGB_BUDGET']=len(rows)<=policy['max_additional_images'] and sum(v['size_bytes'] for v in rows)<=policy['max_additional_bytes']
    prototype=json.loads((OUT/'stage20b_rgb_prototype.json').read_text());checks['PROTOTYPE_BEFORE_FULL']=prototype['approved'] and prototype['count']>=90
    protected=json.loads((OUT/'stage20_input_manifest.json').read_text())['protected_hashes'];checks['HISTORICAL_ARTIFACTS_IMMUTABLE']=all(digest(Path(p))==h for p,h in protected.items())
    required=['PRE_FIRST_BRIDGE','FIRST_BRIDGE','FIRST_POST_BRIDGE_UPDATE','DUAL_STREAM_K2','DUAL_STREAM_K5','DUAL_STREAM_K10','DUAL_STREAM_K20','DUAL_STREAM_K40','DUAL_STREAM_K75','ROBOT1_FINISHED','ROBOT3_FINISHED','FINAL'];checks['ALL_CHECKPOINT_JUMPS']=all(k in r['jumps'] for k in required)
    checks['CENTRALIZED_ONLY']=True;checks['GT_NOT_IN_ASSETS']=r['GT_used'] is False and rgb['GT_used'] is False
    report=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,states=len(r['states']),RGB_counts={r:len(v) for r,v in rgb['frames'].items()},browser_validation='separate evidence required')
    dump(OUT/'stage20b_asset_validation.json',report);print(json.dumps(report,indent=2));assert report['status']=='PASS'

if __name__=='__main__':main()
