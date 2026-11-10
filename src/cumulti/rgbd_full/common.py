"""Full camera-only run: isolated paths, frozen projection helper, atomic records."""
import csv,hashlib,importlib.util,json,os
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'outputs/cumulti_v1/24_rgbd_full_v1';SITE=ROOT/'demo/stage24_full_rgbd'
RAW=Path('/home/cas/CU-Multi/raw/main_campus')
spec=importlib.util.spec_from_file_location('full_frozen_projection',ROOT/'src/cumulti/rgbd_demo/export.py');old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
align_depth,guard=old.align_depth,old.guard
def dump(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);t=p.with_name(p.name+f'.{os.getpid()}.tmp');t.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n');t.replace(p)
def writecsv(p,rows,fields=None):
 p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=fields or list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(8*1024**2),b''):h.update(b)
 return h.hexdigest()
def stats(values):
 a=np.asarray(values,float)
 return dict(count=len(a),min=float(a.min()),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max())) if len(a) else dict(count=0)
