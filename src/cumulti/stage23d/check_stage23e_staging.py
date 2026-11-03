"""Stage23E exact allowlist: scientific summaries, no maps or multi-GB states."""
from stage23e_common import *
import re
def main():
    lock_verify();protect_verify();assert read(E/'stage23e_independent_artifact_validation.json')['status']=='PASS'
    recorded=read(E/'offline_evaluation_entry.json')['evaluator_sha256'];current=sha(ROOT/'src/cumulti/stage23d/evaluate_stage23e.py')
    if recorded!=current:
        patch=read(E/'stage23e_evaluator_metadata_patch.json');assert patch['original_execution_sha256']==recorded and patch['corrected_source_sha256']==current and patch['scientific_formulas_changed'] is False
    for name,v in read(E/'stage23e_final_artifact_manifest.json').items():assert sha(ROOT/name)==v['sha256'],name
    paths=list((ROOT/'src/cumulti/stage23d').glob('*stage23e*.py'))+[ROOT/'scripts/run_stage23e_p6_backend.sh',ROOT/'scripts/run_stage23e_evaluation.sh',ROOT/'docs/CUMULTI_STAGE23E.md']
    paths += [p for p in G.glob('*.json')]
    paths += [p for short in POLICIES for p in (G/short).iterdir() if p.is_file() and p.suffix in ['.json','.csv','.txt'] and p.name not in ['event_trajectory_manifest.json','progress.json']]
    paths += [p for p in E.iterdir() if p.is_file() and p.suffix in ['.json','.csv','.png']]
    paths=sorted(set(paths));names=sorted(str(p.relative_to(ROOT)) for p in paths)
    for p in paths:
        assert p.stat().st_size<4*1024**2,('Oversized',p)
        if p.suffix!='.png':assert not re.search(rb'-----BEGIN (?:OPENSSH|RSA|EC).*PRIVATE KEY|gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|sk-[A-Za-z0-9]{32,}',p.read_bytes()),('Credential pattern',p)
    subprocess.run(['git','add','-f','--']+names,cwd=ROOT,check=True)
    staged=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT,text=True).splitlines();assert staged==names,('Unexpected staged files',set(staged)^set(names))
    subprocess.run(['git','-c','core.whitespace=cr-at-eol','diff','--cached','--check'],cwd=ROOT,check=True)
    print(json.dumps(dict(status='PASS',staged_files=len(paths),bytes=sum(p.stat().st_size for p in paths),raw_datasets_checkpoints_PLY_and_event_ZIPs_excluded=True,all_old_dirty_files_unstaged=True)),flush=True)
if __name__=='__main__':main()
