"""Gate H worker: reuse unchanged Stage22A backend on new independent policy factors."""
import argparse
from common import *


def main(name):
    verify_lock()
    lock=read(OUT/'pre_gt_decision_lock.json')
    directory=OUT/'policies'/name
    assert sha(directory/'online_verified_loops.csv')==lock['decisions'][name]['loops_sha256']
    if (directory/'stage22a3_decision.json').exists():
        print('EXISTING_GRAPH_RESULT',name,flush=True);return
    write_json(directory/'stage22a2_decision.json',dict(status='ONLINE_GICP_VERIFICATION_READY',
        meaning='complete selected-policy causal receipt replay, NOT new full GICP execution',evidence_type=REPLAY))
    import stage22a_common as shared
    import run_stage22a_backend as backend
    # Only redirect the frozen runner's output I/O. All scientific source stays byte-identical.
    shared.OUT=directory;backend.OUT=directory
    backend.verify_freeze=verify_lock
    backend.main()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('policy');main(parser.parse_args().policy)
