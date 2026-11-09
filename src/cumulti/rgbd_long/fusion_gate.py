"""Record the downstream stop explicitly; never treat an audit PASS as tracking PASS."""
import json
from export import OUT,dump
def run():
    d=json.loads((OUT/'pre_fusion_decision.json').read_text());out=OUT/'fusion';out.mkdir(exist_ok=True)
    ready=d['status']=='READY_FOR_BOUNDED_RGBD_FUSION'
    dump(out/'status.json',dict(status='READY_FOR_SEPARATE_FUSION_IMPLEMENTATION' if ready else 'NOT_RUN_STABILITY_GATE_FAILED',
        reason='Both robots must satisfy the frozen shared-config stability checks before any camera retrieval, geometric matching or joint optimization',
        source_decision=d['status'],retrieval_run=False,geometric_verification_run=False,joint_graph_run=False,
        inter_robot_constraints_created=0,GT=False,LiDAR=False))
    dump(out/'graph_status.json',dict(status='NOT_RUN_STABILITY_GATE_FAILED' if not ready else 'NOT_RUN_PENDING_SEPARATE_FUSION_IMPLEMENTATION',GT=False))
if __name__=='__main__':run()
