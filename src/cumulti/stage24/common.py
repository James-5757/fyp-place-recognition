"""Stage24 isolated, explicit paths and camera-only data-access guard."""
import csv, hashlib, json, os, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'outputs/cumulti_v1'
RUN_BASE = Path(os.environ.get('STAGE24_OUTPUT_ROOT',str(BASE))).resolve()
A = RUN_BASE / '24a_rgb_semantic'
B = RUN_BASE / '24b_vision_feasibility'
RAW = Path('/home/cas/CU-Multi/raw')
SITE = Path(os.environ.get('STAGE24_DEMO_DIR',str(ROOT / 'demo/stage24_rgb_semantic'))).resolve()
# Exported manifests use repository-relative paths; keep new runs inside it.
assert RUN_BASE.is_relative_to(ROOT) and SITE.parent == ROOT/'demo'

def dump(p, v):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')

def write_csv(p, rows, fields=None):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields or list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for x in iter(lambda:f.read(8*1024**2),b''):h.update(x)
    return h.hexdigest()

def stats(x):
    x=np.asarray(x,dtype=float)
    return dict(count=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(np.max(x))) if len(x) else dict(count=0,median=None,p95=None,max=None)

def camera_guard(log):
    """Deny scientific sensor/GT inputs at OS-open time in camera processes."""
    def audit(event,args):
        if event!='open' or not isinstance(args[0],(str,bytes,os.PathLike)):return
        p=str(args[0]);low=p.lower()
        if p.endswith(('.py','.pyc')):return
        forbidden=('gt_utm','gt_rel','/ground_truth/','imu_gps','imu.csv','gnss','/full_2hz/','scan_context','/22a_','/22b_','/23e_','/12d_','_lidar.zip','gicp_results','clean_local_trajectory')
        if any(x in low for x in forbidden):
            log.append(dict(path=p,status='DENIED'));raise PermissionError('Stage24 camera-only guard: '+p)
        if '/CU-Multi/' in p or '/24b_' in p:log.append(dict(path=p,status='ALLOWED'))
    sys.addaudithook(audit)

def decode_image(msg):
    a=np.frombuffer(msg.data,dtype=np.uint8).reshape(int(msg.height),int(msg.step))
    if msg.encoding not in ('rgb8','bgr8'):raise ValueError(msg.encoding)
    a=a[:,:int(msg.width)*3].reshape(int(msg.height),int(msg.width),3)
    return a.copy() if msg.encoding=='rgb8' else a[:,:,::-1].copy()
