"""Bounded, resumable Stage22B-only pipeline. No uncertain actual GICP retries."""
import fcntl
import time
from common import *
NATIVE='/home/cas/.venvs/fyp_gtsam/bin/python'
GEOMETRY='/home/cas/fyp_robot13_geometry_20260915/.venv/bin/python'


def execute(module,python=NATIVE,args=()):
    print('PHASE',module,list(args),flush=True)
    subprocess.run([python,str(SRC/'stage22b'/module),*args],cwd=ROOT,check=True)


def main():
    if not OUT.exists():execute('freeze.py')
    with (OUT/'execution.lock').open('a') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        verify_lock()
        execute('test_stage22b.py')
        execute('schema_audit.py')
        execute('environment.py')
        execute('environment.py',GEOMETRY)
        if not (OUT/'pre_gt_decision_lock.json').exists():execute('replay.py')
        if not (OUT/'gate_G.json').exists():execute('actual.py',GEOMETRY)
        assert read(OUT/'gate_G.json')['status']=='PASS','Actual benchmark affected phase failed'
        names=read(OUT/'stage22b_experiment_policy.json')['policies'][:5]+['P5_BUDGET_AWARE_C75_B80']
        for name in names:execute('graph.py',args=[name])
        if not (OUT/'communication_validation.json').exists():execute('communication.py')
        execute('evaluate.py')
        execute('validate.py')
        execute('report.py')
        print('STAGE22B_FINISHED',flush=True)

if __name__=='__main__':main()
