"""Append Stage24 memory and prepare an explicit lightweight publication list."""
from common import *

def run():
    marker='## Stage24A/B — camera feasibility checkpoint (2026-11-06)'
    text='''
Stage24A RGB/semantic3D maps remain BLOCKED: archived URDF reaches camera_link
but not the color optical frame. Do not guess mounting/optical transforms or
color unseen surfaces. New calibration-gated P5 demo and12 real SegFormerB0
2D predictions exist; RGB/semantic map modes explicitly show100%unknown.
RGB-D is BLOCKED for R1/R3: R3 depth missing; R1/R2 depth archives contain
camera PointCloud2 rather than native depth images. Actual R1 cloud is
unorganized,376811x1,with848x480 camera calibration; raw depth scale and
registered-image adaptation require validation. R1 camera-cloud/RGB bag sync
median1.38209ms,p956.2070072ms is not hardware exposure calibration.

Independent camera-only timelines contain1000/2099 R1/R3 visual keyframes.
Six fixed12s ORB-SLAM3 monocular windows each use60real RGB frames: valid
counts R1=0/47/58,R3=0/49/41. Arbitrary per-map scale is preserved; no LiDAR,
SC,GICP,GNSS,IMU or GT supplies tracking/geometry/scale. New offline image-only
retrieval72queries/36opposite candidates yields1epipolar-plausible nonmetric
pair,not a verified metric loop or measured place-recognition precision.
Later pilot frames may be in the offline database; this is not causal online
retrieval. No full visual SLAM or GT evaluation was run.

Three-robot inspection finds existing numeric_key/symbol mapping aliases
Robot2 with Robot3; generic registry/typed endpoints/connectivity and per-pair
budget migration are needed before a new3-agent run. R2 archives exist; R4
has only GT locally. No sensor measurements may be invented.
Historical10098 tracked files match their pre-stage hashes. Independent
artifact audit831checks PASS; browser orbit/zoom/pan/mode checks PASS, with
software-WebGL performance only. No old scientific artifact or frozen
P0/P5/P6 policy was changed. Details and reproducible commands:
docs/CUMULTI_STAGE24A_RGB_SEMANTIC_MAP.md and
docs/CUMULTI_STAGE24B_VISION_FEASIBILITY.md.
Stop after24A/B; Stage24C is NOT_STARTED. Minimum next work is measured optical
mounting/exposure calibration and validated camera-cloud-to-depth adaptation,
then small fixed-cohort projection/visual-geometry tests,not automatic full runs.
'''
    for name in ['PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md']:
        p=ROOT/name;old=p.read_text()
        if marker not in old:p.write_text(old.rstrip()+'\n\n'+marker+'\n\n'+text.strip()+'\n')
    allow=[];excluded=[]
    for base in [A,B]:
        for p in base.rglob('*'):
            if not p.is_file():continue
            rel=str(p.relative_to(ROOT))
            if any(x in p.parts for x in ['camera_frames','segmentation_arrays','bin','depth_work']) or p.suffix=='.log' or p.name=='native_file_access.log' or p.suffix not in ['.json','.csv','.txt','.png','.jpg','.tum','.yaml']:
                excluded.append(dict(path=rel,bytes=p.stat().st_size,sha256=sha(p)));continue
            if p.stat().st_size>5*1024**2:raise RuntimeError('Unexpected large publication artifact: '+rel)
            allow.append(rel)
    dump(A/'server_only_artifact_manifest.json',dict(artifacts=excluded,reason='raw frame sequences,model-confidence arrays,native executable/full access traces remain server-only; raw ZIPs and weights never staged'))
    allow.append(str((A/'server_only_artifact_manifest.json').relative_to(ROOT)))
    (A/'publication_allowlist.txt').write_text('\n'.join(sorted(set(allow)))+'\n')
    dump(A/'final_stage24_validation.json',dict(artifact_audit=json.loads((A/'independent_validation.json').read_text())['status'],
        browser_audit=json.loads((A/'demo_validation.json').read_text())['status'],
        preservation=json.loads((A/'historical_preservation_audit.json').read_text()),
        scientific_decision=json.loads((B/'vision_only_feasibility_decision.json').read_text()),
        meaningful_native_runs=6,accepted_metric_visual_loops=0,full_visual_run=False,Stage24C_started=False))
    print('LIGHTWEIGHT_ARTIFACTS',len(set(allow)),flush=True)

if __name__=='__main__':run()
