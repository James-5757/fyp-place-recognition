"""Explicit two-environment Stage22A launcher; durable gates and bounded resume.

--finish-only waits for an already executing verification job. No duplicate
registrations are launched. --commit is opt-in, used only after the independent
audit/report pass; explicit paths/size/secret checks protect unrelated dirt.
"""
import argparse,subprocess,time
from stage22a_common import *
NATIVE='/home/cas/.venvs/fyp_gtsam/bin/python'
GEOMETRY='/home/cas/fyp_robot13_geometry_20260915/.venv/bin/python'

def run(name,python=NATIVE,args=()):
    started=time.monotonic()
    subprocess.run([python,str(ROOT/'src/cumulti'/name),*args],cwd=ROOT,check=True)
    if name=='run_stage22a_verification.py' and list(args)==['full']:
        dump('gicp_full_invocation_wall.json',dict(measured_wall_seconds=time.monotonic()-started,scope='complete verification subprocess invocation, including imports, manifests and checkpoint I/O'))

def main(finish_only=False,commit=False):
    if finish_only:
        deadline=time.monotonic()+7500
        while not (OUT/'stage22a2_decision.json').exists():
            if time.monotonic()>deadline:raise TimeoutError('No A2 result; inspect running verification, do not launch a duplicate job')
            time.sleep(15)
    else:
        if not OUT.exists():run('stage22a_common.py')
        verify_freeze()
        if not (OUT/'synthetic_unit_tests.json').exists():run('test_stage22a_online.py')
        if not (OUT/'stage22a1_decision.json').exists():run('run_stage22a_retrieval.py')
        if not (OUT/'gicp_pilot_summary.json').exists():run('run_stage22a_verification.py',GEOMETRY,['pilot'])
        if not (OUT/'stage22a2_decision.json').exists():run('run_stage22a_verification.py',GEOMETRY,['full'])
    verify_freeze()
    if not (OUT/'stage22a3_decision.json').exists():
        if read('stage22a2_decision.json')['status']=='ONLINE_GICP_VERIFICATION_READY':run('run_stage22a_backend.py')
        else:
            reason='Stage22A-2 incomplete under predeclared budget; no backend integration permitted'
            dump('stage22a3_decision.json',dict(status='NOT_RUN',structural_status='NOT_RUN',numerical_status='NOT_RUN',reason=reason))
            dump('online_graph_integrity.json',dict(status='NOT_RUN',reason=reason,nodes=0,odometry=0,loops=0,factors=0))
            csv('online_incremental_vs_batch.csv',[dict(status='NOT_RUN',reason=reason)]);csv('online_map_consistency.csv',[dict(status='NOT_RUN',reason=reason)])
            dump('pre_gt_online_decision.json',dict(frontend=read('stage22a2_decision.json')['status'],backend='NOT_RUN',GT_read_yet=False))
    if not (OUT/'online_cost_summary.json').exists():run('run_stage22a_costs.py')
    if not (OUT/'offline_evaluation_audit.json').exists():run('evaluate_stage22a_offline.py')
    run('audit_stage22a_online.py');run('finalize_stage22a_online.py')
    if commit:run('commit_stage22a_online.py')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--finish-only',action='store_true');p.add_argument('--commit',action='store_true');a=p.parse_args();main(a.finish_only,a.commit)
