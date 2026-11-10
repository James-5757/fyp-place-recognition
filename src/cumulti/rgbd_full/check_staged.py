"""Scope/size/secret checks for explicitly curated full RGB-D commits."""
import re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]

if __name__=='__main__':
 paths=subprocess.check_output(['git','diff','--cached','--name-only','-z'],cwd=ROOT).decode().split('\0');paths=[p for p in paths if p];total=0
 for path in paths:
  assert path in ['PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md'] or path.startswith(('src/cumulti/rgbd_full/','demo/stage24_full_rgbd/','docs/CUMULTI_RGBD_FULL_DATA_AUDIT.md','docs/CUMULTI_STAGE24_FULL_RGBD_MAPPING.md','outputs/cumulti_v1/24_rgbd_full_v1/','scripts/build_rgbd_full.sh','scripts/run_rgbd_full.sh')),path
  assert not any(s in path for s in ['/work/','/bin/','/rgb/','/depth/','__pycache__','native_access','stdout','sequence.txt']),path
  assert not path.endswith(('.db3','.zip','.ply','.pt','.pth','.env','.tar')),path
  data=subprocess.check_output(['git','show',':'+path],cwd=ROOT);assert len(data)<16*1024**2,(path,len(data));total+=len(data)
  assert not re.search(rb'ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|sk-[A-Za-z0-9]{20,}|-----BEGIN [A-Z ]*PRIVATE KEY',data),('Secret-like staged content',path)
 assert total<64*1024**2,total
 subprocess.run(['git','diff','--cached','--check'],cwd=ROOT,check=True)
 print('STAGED_SCOPE_SIZE_SECRET_CHECK_PASS',len(paths),'files',total,'bytes')
