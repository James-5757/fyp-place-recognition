#!/usr/bin/env python3
"""Independent saved-asset audit, never runs a scientific experiment."""
import csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parents[2]
SITE=ROOT/'demo/stage19_rgb_three_map'
OUT=ROOT/'outputs/cumulti_v1/19_rgb_three_map'
BASE='bf30e1a23ee7b7ccb58d095949d443ab0ee2a7dd'

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    checks=[]
    def check(name,value):
        checks.append({'check':name,'pass':bool(value)})
    manifest=json.loads((SITE/'rgb_manifest.json').read_text())
    local=json.loads((SITE/'local_maps.json').read_text())
    old=ROOT/'demo/progressive_multi_robot/data'
    replay=json.loads((old/'replay.json').read_text())
    provenance=json.loads((OUT/'data_provenance_manifest.json').read_text())
    for p in provenance['sources']:
        check('source hash '+p['path'],sha(Path(p['path']))==p['sha256'])
    for p in sorted((ROOT/'demo/progressive_multi_robot').glob('*'))+sorted(old.glob('*.json')):
        if p.is_file():
            rel=p.relative_to(ROOT).as_posix()
            base=subprocess.run(['git','show',f'{BASE}:{rel}'],cwd=ROOT,capture_output=True)
            if base.returncode==0: check('accepted frontend unchanged '+rel,p.read_bytes()==base.stdout)
    check('75 frozen events',len(replay['events'])==75)
    check('local poses equal frozen Stage18',local['trajectory']==replay['local_trajectory'])
    check('185 causal local map chunks',len(local['robot1_chunks'])==185)
    check('local chunk order', [x['kf'] for x in local['robot1_chunks']]==list(range(150,2000,10)))
    totals={}
    for robot in ['robot1','robot3']:
        rows=manifest['frames'][robot];ts=[int(r['rgb_timestamp_ns']) for r in rows]
        with (OUT/f'{robot}_rgb_manifest.csv').open(newline='') as f: csv_rows=list(csv.DictReader(f))
        check(robot+' CSV equals JSON exactly',len(csv_rows)==len(rows) and all(all(c.get(k)==str(v) for k,v in r.items()) for c,r in zip(csv_rows,rows)))
        check(robot+' timestamp unique ordered',len(ts)==len(set(ts)) and sorted(ts)==ts)
        for r in rows:
            p=SITE/r['file'];im=cv2.imread(str(p))
            check(robot+' file '+r['file'],p.is_file() and p.stat().st_size==r['size_bytes'] and sha(p)==r['sha256'] and im is not None and im.shape[:2]==(r['height'],r['width']))
            check(robot+' exact timestamp delta '+r['file'],abs(int(r['rgb_timestamp_ns'])-int(r['lidar_timestamp_ns']))==int(r['abs_sync_error_ns']))
        a=manifest['robots'][robot];p=Path(a['archive'])
        check(robot+' raw stat unchanged',p.stat().st_size==a['archive_size'] and p.stat().st_mtime_ns==a['archive_mtime_ns'])
        totals[robot]={'images':len(rows),'bytes':sum(r['size_bytes'] for r in rows),'keyframe_nearest_error_ms':a['keyframe_nearest_rgb_error_ms']}
        offsets=np.array([int(r['header_timestamp_ns'])-int(r['rgb_timestamp_ns']) for r in rows])/1e6
        totals[robot]['header_minus_bag_ms']={'median':float(np.median(offsets)),'min':float(offsets.min()),'max':float(offsets.max()),'positive_count':int((offsets>0).sum())}
    rows=manifest['frames']['robot1'];ts=np.array([int(r['rgb_timestamp_ns']) for r in rows],np.int64)
    # Exhaustive 2-Hz KF selection and every retained camera timestamp; floor never future.
    age=[]
    for row in local['trajectory']['robot1']:
        t=int(row[1]);i=int(np.searchsorted(ts,t,side='right'))-1
        while i>=0 and int(rows[i]['header_timestamp_ns'])>t:i-=1
        check('R1 causal floor KF'+str(row[0]),i>=0 and int(ts[i])<=t)
        if i>=0:age.append((t-int(ts[i]))/1e6)
    for e in replay['events']:
        selected=manifest['robots']['robot3']['keyframe_rgb_indices'][str(e['candidate'])]
        check('R3 exact historical reference K'+str(e['index']),any(r['rgb_message_index']==selected for r in manifest['frames']['robot3']))
    totals['robot1']['causal_age_ms']={'median':float(np.median(age)),'p95':float(np.percentile(age,95)),'max':float(max(age))}
    check('presentation asset bytes <=120MiB',sum(totals[r]['bytes'] for r in ['robot1','robot3'])<=120*1024**2)
    # Browser evidence is mandatory, rather than inferring visual PASS from source strings.
    evidence=json.loads((OUT/'browser_validation.json').read_text()) if (OUT/'browser_validation.json').exists() else {}
    for name in ['PRE_FIRST_LOOP','FIRST_LOOP','K2','K5','K10','K20','K40','K75','FINAL']:
        e=evidence.get('checkpoints',{}).get(name,{})
        check('browser observed '+name,e.get('three_panels_visible') and e.get('rgb_ready') and e.get('causal_rgb') and e.get('state_consistent'))
    check('browser controls tested',all(evidence.get('controls',{}).get(k,False) for k in ['play_pause','reset','seek','speed','checkpoint_jumps']))
    for name in ['PRE_FIRST_LOOP','FIRST_LOOP','K2','K5','K10','K20','K40','K75','FINAL','FIRST_CANDIDATE','GICP']:
        p=OUT/'screenshots'/f'{name}.jpg'
        check('screenshot '+name,p.exists() and cv2.imread(str(p)) is not None)
    for name in ['performance_initial.json','performance_global.json']:
        p=OUT/name
        v=json.loads(p.read_text()) if p.exists() else {}
        check('measured '+name,v.get('seconds',0)>=20 and v.get('end_kf',0)>v.get('start_kf',0) and v.get('rgb',{}).get('errors')==0)
    p=OUT/'unit_tests.json';v=json.loads(p.read_text()) if p.exists() else {}
    check('unit test result',v.get('status')=='PASS' and all(v.get(k) for k in ['causal_floor','explicit_historical_reference','seek_token','missing_state','bounded_cache']))
    frontend={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in SITE.iterdir() if p.is_file()}
    (OUT/'presentation_artifact_manifest.json').write_text(json.dumps(frontend,indent=2)+'\n')
    report=dict(status='PASS' if all(c['pass'] for c in checks) else 'FAIL',checks=checks,totals=totals,limitations=['Presentation audit is not physical accuracy or live-system readiness.','Automated RAF measurements alone do not establish normal foreground smoothness.'])
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'independent_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    (OUT/'VALIDATION_REPORT.txt').write_text('STAGE19: '+report['status']+'\n'+f'{sum(c["pass"] for c in checks)}/{len(checks)} checks pass\n'+ '\n'.join('[FAIL] '+c['check'] for c in checks if not c['pass'])+'\nGT: offline height diagnosis only, never presentation alignment or tuning\nSC/GICP/optimizer reruns: 0\nLIVE / DISTRIBUTED: NOT_RUN\nSMOOTHNESS: foreground rehearsal still required; see performance.json\n')
    print(report['status'],len(checks),'checks',json.dumps(totals),flush=True)
    if report['status']!='PASS': raise SystemExit(1)

if __name__=='__main__': main()
