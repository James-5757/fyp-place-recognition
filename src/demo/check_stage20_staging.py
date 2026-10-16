#!/usr/bin/env python3
"""Reject unrelated, private, raw or oversized additions before the Stage20 commit."""
import json, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def main():
    names=subprocess.check_output(['git','diff','--cached','--name-only','-z'],cwd=ROOT).decode().strip('\0').split('\0')
    allowed={'RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md','docs/CUMULTI_STAGE20_DUAL_STREAM.md','scripts/run_stage20_dual_stream.sh'}
    allowed|={'src/cumulti/'+n for n in ['stage20_schedule.py','test_stage20_schedule.py','audit_stage20_backend.py','run_stage20_dual_stream.py']}
    allowed|={'src/demo/'+n for n in ['audit_stage20_demo.py','export_stage20_rgb.py','prepare_stage20_assets.py','finalize_stage20.py','check_stage20_staging.py']}
    total=0;maximum=0
    for n in names:
        assert n in allowed or n.startswith(('demo/stage20_dual_stream/','outputs/cumulti_v1/20_dual_stream/')),n
        p=Path(n);assert p.suffix.lower() not in ['.zip','.db3','.bag','.ply','.pcd','.npy','.npz','.pt','.pth','.pem','.key','.env'],n
        assert not any(s in n.lower().split('/') for s in ['.ssh','.git','__pycache__','raw','20b_display_reproduction','prototype','rgb_prototype','maps']),n
        data=subprocess.check_output(['git','show',':'+n],cwd=ROOT)
        assert len(data)<10*1024**2,(n,len(data));maximum=max(maximum,len(data));total+=len(data)
        if p.suffix.lower() in ['.py','.js','.sh','.json','.md','.txt','.csv','.html','.css']:
            for token in [b'-----BEGIN OPENSSH PRIVATE KEY-----',b'-----BEGIN RSA PRIVATE KEY-----',b'ghp_',b'github_pat_']:
                # The scanner's own inert signatures are not credentials.
                if n!=str(Path(__file__).relative_to(ROOT)):assert token not in data,(n,'secret signature')
    pictures=[n for n in names if n.startswith('demo/stage20_dual_stream/rgb/robot3/')]
    manifest=json.loads((ROOT/'demo/stage20_dual_stream/rgb_manifest.json').read_text())
    assert len(pictures)==len(manifest['frames']['robot3'])==2011
    assert all('demo/final_replay/' not in n and '/10_3_solver_audit/' not in n for n in names)
    print(json.dumps(dict(status='PASS',files=len(names),total_uncompressed_bytes=total,largest_file_bytes=maximum,robot3_presentation_images=len(pictures),raw_datasets_checkpoints_credentials_large_PLY_and_unrelated_changes_excluded=True)))
if __name__=='__main__':main()
