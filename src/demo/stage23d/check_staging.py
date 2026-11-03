"""Exact Stage23D allowlist, asset hashes and credential/size checks."""
import sys,re,subprocess,json,gzip
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'cumulti/stage23d'))
from common import *

def main():
    protect_verify()
    allowed=[]
    allowed += [p for folder in [ROOT/'src/cumulti/stage23d',ROOT/'src/demo/stage23d'] for p in folder.glob('*.py')]
    allowed += [ROOT/'scripts/run_stage23d_demo.sh',ROOT/'scripts/run_stage23d_scheduler.sh',ROOT/'docs/CUMULTI_STAGE23D.md']
    allowed += [p for p in DEMO.rglob('*') if p.is_file()]
    for folder in [MAP,OUT]:
        for p in folder.iterdir():
            if not p.is_file():continue
            if p.name in ['historical_integrity_inventory.json.gz','p6_queue_event_trace.csv','p6_budget_event_trace.csv','p6_verified_factors.csv','p5_utility_components.csv','p5_rejection_receipt_availability.csv']:continue
            if p.suffix in ['.json','.csv','.png','.gz']:allowed.append(p)
    assert read(OUT/'stage23d_decision.json')['P0_P5_FINAL_DEMO']=='PASS'
    for p in allowed:
        assert p.stat().st_size<4*1024**2,('Oversized staged candidate',p)
        data=p.read_bytes()
        if p.suffix in ['.py','.js','.html','.json','.sh','.md','.csv']:
            assert not re.search(rb'-----BEGIN (?:OPENSSH|RSA|EC).*PRIVATE KEY|gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|sk-[A-Za-z0-9]{32,}',data),('Credential pattern',p)
    for relative,v in read(MAP/'demo_asset_manifest.json').items():assert sha(ROOT/relative)==v['sha256'],relative
    for name,v in read(OUT/'trace_archive_manifest.json').items():assert sha(OUT/name)==v['original_sha256'] and sha(OUT/v['archive'])==v['archive_sha256'] and gzip.decompress((OUT/v['archive']).read_bytes())==(OUT/name).read_bytes()
    relative=sorted(str(p.relative_to(ROOT)) for p in allowed)
    subprocess.run(['git','add','-f','--']+relative,cwd=ROOT,check=True)
    staged=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT,text=True).splitlines();assert staged==relative,('Unexpected staged files',set(staged)^set(relative))
    subprocess.run(['git','-c','core.whitespace=cr-at-eol','diff','--cached','--check'],cwd=ROOT,check=True)
    print(json.dumps(dict(status='PASS',staged_files=len(relative),total_bytes=sum(p.stat().st_size for p in allowed),only_new_Stage23D_paths=True,no_raw_scans_bags_models_credentials=True,old_dirty_files_unstaged=True)),flush=True)
if __name__=='__main__':main()
