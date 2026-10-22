"""Independent Stage22B paths and immutable, explicitly typed evidence."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
import numpy as np
import pandas as pd

ROOT=Path('/home/cas/fyp_place_recognition')
SRC=ROOT/'src/cumulti'
sys.path.insert(0,str(SRC))
BASE='c929fc55a9f42c149ce571205618da73db7d4bc5'
A=ROOT/'outputs/cumulti_v1/22a_online_frontend'
OUT=Path(os.environ.get('CUMULTI_STAGE22B_OUTPUT_ROOT',ROOT/'outputs/cumulti_v1/22b_budget_aware'))
PROC=Path('/home/cas/CU-Multi/processed_v1/full_2hz')
FROZEN=['stage22b_input_manifest.json','stage22b_experiment_policy.json',
        'stage22b_budget_policy.json','stage22b_data_split.json','stage22b_predeclared_acceptance.json']
REPLAY='MEASURED_RECEIPTS_COUNTERFACTUAL_REPLAY'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(4*1024**2),b''):h.update(block)
    return h.hexdigest()


def write_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    with tmp.open('w') as f:
        json.dump(data,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)


def read(path):return json.loads(Path(path).read_text())
def csv(path,data,columns=None):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    (data if isinstance(data,pd.DataFrame) else pd.DataFrame(data,columns=columns)).to_csv(path,index=False)


def verify_lock():
    lock=read(OUT/'policy_lock.json')
    for name,digest in lock['files'].items():assert sha(OUT/name)==digest,('Frozen policy changed',name)


def guard():
    """Policy processes may hash opaque inputs but never read offline labels/GT/archive."""
    forbidden=['offline_','gt_utm','gt_rel','ordered_rank1_clean_loops','rank1_sanitized_edges',
               'candidate_interface','09_sc_gicp_integration/','10_2_pgo_policy_selection/']
    def hook(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            p=os.fsdecode(args[0]).lower()
            if any(x in p for x in forbidden):raise PermissionError('Stage22B causal access forbidden: '+p)
    sys.addaudithook(hook)


def input_tables():
    return (pd.read_csv(A/'dual_stream_event_schedule.csv').to_dict('records'),
            pd.read_csv(A/'online_candidate_trace.csv'),pd.read_csv(A/'online_gicp_attempts.csv'))


def assert_source_unchanged():
    manifest=read(OUT/'stage22b_input_manifest.json')
    for path,digest in manifest['inputs'].items():assert sha(path)==digest,('Input changed',path)
    for path,digest in manifest['protected_worktree'].items():assert sha(ROOT/path)==digest,('Historical file changed',path)


def inv(T):
    v=np.eye(4);v[:3,:3]=T[:3,:3].T;v[:3,3]=-v[:3,:3]@T[:3,3];return v
