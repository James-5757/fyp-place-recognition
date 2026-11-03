"""Gate offline GT behind hashes of ALL four completed backend attempts."""
from stage23e_common import *
import zipfile
def main():
    lock_verify();files={};status={}
    for short in POLICIES:
        directory=G/short;integrity=read(directory/'graph_integrity.json');status[short]=integrity['status'];receipt=read(directory/'pre_gt_policy_artifacts.json')
        for relative,v in receipt['files'].items():
            p=directory/relative;assert p.stat().st_size==v['bytes'] and sha(p)==v['sha256'],str(p);files[str(p)]=v
        files[str(directory/'pre_gt_policy_artifacts.json')]=dict(bytes=(directory/'pre_gt_policy_artifacts.json').stat().st_size,sha256=sha(directory/'pre_gt_policy_artifacts.json'))
        if integrity['status']=='PASS':
            assert integrity['nodes']==5796 and integrity['odometry']==5794 and integrity['loops']==COUNTS[short] and integrity['factors']==5795+COUNTS[short]
            with zipfile.ZipFile(directory/'event_trajectories_float64.zip') as z:assert sorted(z.namelist())==[f'event_{i:06d}.npy' for i in range(5796)]
    value=dict(status='ALL_FOUR_BACKEND_RESULTS_FROZEN',graph_status=status,GT_read=False,new_GICP_calls=0,files=files,input_manifest_sha256=sha(G/'stage23e_input_manifest.json'),policy_sha256=sha(G/'stage23e_policy.json'),frozen_unix_ns=time.time_ns())
    path=G/'all_graphs_pre_gt_freeze.json'
    if path.exists():
        old=read(path);assert old['files']==value['files'] and old['graph_status']==status;print('EXISTING_ALL_FOUR_GRAPH_FREEZE_VERIFIED',flush=True);return
    dump(path,value);print('ALL_FOUR_P6_GRAPHS_FROZEN_BEFORE_GT',json.dumps(status),flush=True)
if __name__=='__main__':main()
