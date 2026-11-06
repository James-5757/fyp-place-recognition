"""Validate the exact Stage24 publication, never modify the Git index."""
import re,subprocess
from common import *

def run():
    files=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=ROOT,text=True).splitlines()
    root_docs={'PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md'}
    prefixes=['src/cumulti/stage24/','demo/stage24_rgb_semantic/','outputs/cumulti_v1/24a_rgb_semantic/','outputs/cumulti_v1/24b_vision_feasibility/']
    scripts={'scripts/build_stage24_camera_pilot.sh','scripts/run_stage24_ab.sh','scripts/setup_stage24_semantics.py','docs/CUMULTI_STAGE24A_RGB_SEMANTIC_MAP.md','docs/CUMULTI_STAGE24B_VISION_FEASIBILITY.md'}
    patterns=[re.compile(x) for x in [r'-----BEGIN (?:OPENSSH|RSA|EC) PRIVATE KEY-----',r'gh[pousr]_[A-Za-z0-9]{30,}',r'AKIA[0-9A-Z]{16}',r'sk-[A-Za-z0-9]{35,}']]
    total=0
    for name in files:
        assert name in root_docs|scripts or any(name.startswith(p) for p in prefixes),name
        p=ROOT/name;total+=p.stat().st_size;assert p.stat().st_size<=5*1024**2,name
        assert p.suffix not in ['.zip','.db3','.ply','.npy','.npz','.pt','.pth','.safetensors','.whl','.log'],name
        assert not any(x in p.parts for x in ['camera_frames','bin','__pycache__']),name
        if p.suffix in ['.py','.cc','.js','.html','.json','.md','.csv','.sh','.txt','.yaml']:
            text=p.read_text(errors='replace');assert not any(regex.search(text) for regex in patterns),name+' secret-like token'
    manifest=json.loads((A/'map_asset_manifest.json').read_text())
    for item in manifest['artifacts']:
        blob=subprocess.check_output(['git','show',':'+item['path']],cwd=ROOT)
        assert hashlib.sha256(blob).hexdigest()==item['sha256'],'Demo asset missing/stale in Git index: '+item['path']
    subprocess.run(['git','-c','core.whitespace=cr-at-eol','diff','--cached','--check'],cwd=ROOT,check=True)
    print(json.dumps(dict(status='PASS',staged_files=len(files),bytes=total,no_raw_models_secrets_unnecessary_large=True,original_unrelated_dirt_not_staged=True)))

if __name__=='__main__':run()
