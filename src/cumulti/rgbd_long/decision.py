"""Freeze an explicit shared GT-free stability decision before any fusion."""
import argparse,json
from export import OUT,dump,policy

def run(final=False):
    p=policy();results={};selected=None
    for config in ['baseline1000','fallback2000']:
        paths=[OUT/r/config/'metrics.json' for r in ['robot1','robot3']]
        if not all(path.exists() for path in paths):continue
        values=[json.loads(path.read_text()) for path in paths];results[config]=values
        if all(v['fusion_gate']=='PASS' for v in values):selected=config;break
    if not results:raise RuntimeError('Both baseline robot reports required before decision')
    status='READY_FOR_BOUNDED_RGBD_FUSION' if selected else ('NO_GO_RGBD_FUSION' if final else 'BASELINE_FAILED_RUN_PREDECLARED_FALLBACK')
    value=dict(status=status,selected_configuration=selected,gate_policy=p['fusion_gate'],
        results={c:{v['robot']:dict(gate=v['fusion_gate'],checks=v['checks']) for v in values} for c,values in results.items()},
        GT_read=False,LiDAR_read=False,decision_scope='120-second fixed camera cohort, not full-sequence robustness',
        fallback_changed_only='ORB feature capacity1000→2000; same frames, optics, depth registration, noise, FAST and backend',
        reset_regression=json.loads((OUT/'reset_regression/validation.json').read_text())['status'])
    if selected or final:
        path=OUT/'pre_fusion_decision.json'
        if path.exists():assert json.loads(path.read_text())==value,'Frozen stability decision changed'
        else:dump(path,value)
    else:dump(OUT/'baseline_decision.json',value)
    print(json.dumps(value),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--final',action='store_true');run(p.parse_args().final)
