#!/usr/bin/env python3
"""Fail closed on unrelated/private/oversized Stage21 Git additions."""
import json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def main():
    names=subprocess.check_output(['git','diff','--cached','--name-only','-z'],cwd=ROOT).decode().strip('\0').split('\0');count=total=maximum=0
    rootfiles={'RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md','docs/CUMULTI_STAGE21_COST_BASELINE.md','scripts/run_stage21_cost_baseline.sh'}
    for n in names:
        assert n in rootfiles or (n.startswith('src/cumulti/') and 'stage21' in n and n.endswith('.py')) or (n.startswith('outputs/cumulti_v1/21_cost_baseline/') and len(Path(n).parts)==4),n
        assert Path(n).suffix.lower() not in {'.zip','.db3','.bag','.npy','.npz','.ply','.pcd','.pt','.pth','.pem','.key','.env','.jpg'},n
        assert Path(n).name!='message_trace_by_mode.csv' and not any(x in Path(n).parts for x in ['frontend_pairs','consumed_field_validation','__pycache__','.ssh','raw']),n
        data=subprocess.check_output(['git','show',':'+n],cwd=ROOT);assert len(data)<8*1024**2,(n,len(data));maximum=max(maximum,len(data));total+=len(data);count+=1
        if n!=str(Path(__file__).relative_to(ROOT)):
            for token in [b'-----BEGIN OPENSSH PRIVATE KEY-----',b'-----BEGIN RSA PRIVATE KEY-----',b'github_pat_',b'ghp_']:assert token not in data,(n,'secret signature')
    assert 'outputs/cumulti_v1/21_cost_baseline/stage21_baseline_decision.json' in names
    print(json.dumps(dict(status='PASS',files=count,total_uncompressed_bytes=total,largest_file_bytes=maximum,excluded='raw clouds, bags, full143MiB trace, per-pair intermediates, private credentials, unrelated dirty files')))
if __name__=='__main__':main()
