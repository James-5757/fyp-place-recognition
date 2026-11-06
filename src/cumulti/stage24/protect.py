"""Hash tracked historical science and original dirty work before/after Stage24."""
import subprocess,sys
from common import *

def snapshot():
    paths=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().split('\0')
    # Root context docs may receive appended Stage24 entries; never rewrite history.
    exempt={'PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md'}
    return {p:dict(bytes=(ROOT/p).stat().st_size,sha256=sha(ROOT/p)) for p in paths if p and p not in exempt and (ROOT/p).is_file() and not any(x in p for x in ['stage24','STAGE24','24a_rgb_semantic','24b_vision_feasibility'])}

if __name__=='__main__':
    p=A/'historical_input_hashes.json'
    if sys.argv[1]=='freeze':
        if p.exists():raise RuntimeError('Already frozen')
        dump(p,snapshot());print('FROZEN',len(json.loads(p.read_text())))
    else:
        original=json.loads(p.read_text());now=snapshot();bad=[k for k,v in original.items() if now.get(k)!=v]
        dump(A/'historical_preservation_audit.json',dict(status='PASS' if not bad else 'FAIL',files_checked=len(original),changed=bad))
        assert not bad,bad
