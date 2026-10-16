#!/usr/bin/env python3
"""Fail closed if the Stage19 commit stages unrelated or large/private material."""
import json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
names=subprocess.check_output(['git','diff','--cached','--name-only','-z'],cwd=ROOT).decode().strip('\0').split('\0')
exact={'RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md','docs/CUMULTI_STAGE19_RGB_THREE_MAP_DEMO.md','scripts/run_stage19_demo.sh',*[f'src/demo/{n}' for n in ['export_stage19_rgb.py','prepare_stage19_assets.py','audit_stage19_rgb_demo.py','test_stage19_rgb.mjs','check_stage19_staging.py']]}
total=0;maximum=('',0);images=0
for name in names:
    assert name in exact or name.startswith(('demo/stage19_rgb_three_map/','outputs/cumulti_v1/19_rgb_three_map/')),name
    assert not any(s in name.lower() for s in ['.db3','.bag','.zip','.npy','.npz','.pth','.ckpt','.env','private_key']),name
    if name.startswith('outputs/'):
        assert '/prototype/' not in name and Path(name).suffix in {'.json','.csv','.txt','.png','.jpg'},name
    blob=subprocess.check_output(['git','show',':'+name],cwd=ROOT)
    total+=len(blob)
    if len(blob)>maximum[1]:maximum=(name,len(blob))
    assert len(blob)<10*1024**2,(name,len(blob))
    if Path(name).suffix not in {'.jpg','.png'}:
        assert not re.search(rb'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|sk-[A-Za-z0-9]{30,}',blob),name
    if name.startswith('demo/stage19_rgb_three_map/rgb/') and name.endswith('.jpg'):images+=1
assert images==5757,images
assert total<120*1024**2,total
print(json.dumps(dict(status='PASS',files=len(names),presentation_images=images,total_staged_bytes=total,largest=maximum,unrelated_files_staged=False,raw_bags_checkpoints_credentials_staged=False),indent=2))
