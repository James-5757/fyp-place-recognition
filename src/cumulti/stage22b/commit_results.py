"""Explicit Stage22B-only lightweight commit and push; unrelated dirt is never staged."""
import argparse
from common import *


def main(push=False):
    decision=read(OUT/'stage22b_method_decision.json')
    assert read(OUT/'stage22b_independent_validation.json')['status']=='PASS'
    assert_source_unchanged()
    # No surprises: fail if another task already staged unrelated changes.
    assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT).strip()
    files=[p for p in (SRC/'stage22b').glob('*') if p.suffix in ['.py','.json']]
    files += [ROOT/'scripts/run_stage22b_budget_aware.sh',ROOT/'docs/CUMULTI_STAGE22B_BUDGET_AWARE.md',
              ROOT/'EXPERIMENT_LOG.md',ROOT/'RESULTS.md',ROOT/'NEXT_STEPS.md']
    files += [p for p in OUT.iterdir() if p.is_file() and p.suffix in ['.json','.csv','.png','.txt'] and p.stat().st_size<=4*1024**2]
    # Full per-event traces, raw GICP files, heavy maps and process logs remain on server.
    allowed=[]
    for p in files:
        assert p.is_file() and p.stat().st_size<=5*1024**2,str(p)
        allowed.append(str(p.relative_to(ROOT)))
    print(json.dumps(dict(commit_paths=allowed,scientific_status=decision['status'],push=push),indent=2))
    if not push:return
    subprocess.run(['git','add','--force','--',*allowed],cwd=ROOT,check=True)
    subprocess.run(['git','diff','--cached','--check'],cwd=ROOT,check=True)
    staged=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT,text=True).splitlines()
    assert set(staged)<=set(allowed)
    subprocess.run(['git','commit','-m','feat: evaluate budget-aware inter-robot verification'],cwd=ROOT,check=True)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    subprocess.run(['git','push','origin','HEAD'],cwd=ROOT,check=True,
        env=dict(os.environ,GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=15'))
    print('PUSHED_COMMIT',commit,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--push',action='store_true');main(p.parse_args().push)
