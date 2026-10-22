"""Opt-in Stage22A-only commit/push after independent validation; no broad add."""
import re,subprocess
from stage22a_common import *

def git(*args,**kw):return subprocess.check_output(['git',*args],cwd=ROOT,**kw).decode().strip()
def main():
    assert read('independent_validation.json')['status']=='PASS'
    final=read('stage22a_final_decision.json');assert final['status'] in ['CAUSAL_ONLINE_GRAPH_READY','CAUSAL_ONLINE_GRAPH_NOT_READY','FRONTEND_VERIFICATION_INCOMPLETE']
    assert not git('diff','--cached','--name-only'),'Existing user-staged files: stop, do not mix commits'
    source=[ROOT/'src/cumulti'/name for name in ['stage22a_common.py','run_stage22a_retrieval.py','run_stage22a_verification.py','test_stage22a_online.py','run_stage22a_backend.py','evaluate_stage22a_offline.py','run_stage22a_costs.py','audit_stage22a_online.py','finalize_stage22a_online.py','run_stage22a_pipeline.py','commit_stage22a_online.py','observe_stage22a_process.py','inspect_stage22a_checkpoint.py']]
    paths=source+[ROOT/'scripts/run_stage22a_online_frontend.sh',ROOT/'docs/CUMULTI_STAGE22A_ONLINE_FRONTEND.md',ROOT/'PROJECT_CONTEXT.md',ROOT/'RESULTS.md',ROOT/'EXPERIMENT_LOG.md',ROOT/'NEXT_STEPS.md']
    server_only=['online_candidate_trace.csv','online_gicp_attempts.csv','online_rejected_candidates.csv','online_protocol_message_trace.csv','physical_GICP_execution_trace.csv','independent_audit_checks.csv','lidar_input_manifest.csv','online_retrieval_latency.csv','retrieval_repeat_latency.csv','database_growth.csv','dual_stream_event_schedule.csv','online_verification_queries.csv','online_communication_cost.csv','online_frontend_compute_cost.csv','online_backend_event_trace.csv','offline_causal_query_evaluation.csv','causal_timeline_audit.csv','online_incremental_trajectory.csv','online_batch_trajectory.csv']
    for p in OUT.iterdir():
        if p.is_file() and p.suffix in ['.json','.csv','.txt','.png'] and p.name not in server_only and p.name not in ['git_commit_receipt.json'] and p.stat().st_size<=1024*1024:paths.append(p)
    required_manifest=OUT/'stage22a_input_manifest.json'
    if required_manifest not in paths:paths.append(required_manifest)
    # Small explicitly partial previews are discoverable via their filenames.
    for name in ['online_candidate_trace.csv','online_gicp_attempts.csv','online_communication_cost.csv']:
        frame=pd.read_csv(OUT/name,nrows=10);target=OUT/(Path(name).stem+'_PARTIAL_PREVIEW.csv');frame.to_csv(target,index=False,lineterminator='\n');paths.append(target)
    refs=[str(p.relative_to(ROOT)) for p in paths];assert len(refs)==len(set(refs))
    for p in paths:
        assert p.stat().st_size<=2*1024*1024 and p.suffix.lower() not in ['.npy','.npz','.ply','.zip','.db3','.pt','.pth','.env']
        if p.suffix!='.png':assert not re.search(rb'-----BEGIN (?:OPENSSH|RSA|EC|DSA) PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|sk-[A-Za-z0-9]{30,}|AKIA[0-9A-Z]{16}',p.read_bytes()),'potential secret in '+str(p)
    # Historical .gitignore excludes outputs/** and PNG. Force only the exact
    # individually size/secret-checked allowlist, never a directory/glob add.
    subprocess.run(['git','add','-f','--',*refs],cwd=ROOT,check=True)
    staged=git('diff','--cached','--name-only').splitlines();assert set(staged)<=set(refs) and staged
    subprocess.run(['git','-c','core.whitespace=cr-at-eol','diff','--cached','--check'],cwd=ROOT,check=True)
    print(git('status','--short'),flush=True);print(git('diff','--cached','--stat'),flush=True)
    subprocess.run(['git','commit','--quiet','-m','feat: implement causal bidirectional inter-robot loop discovery'],cwd=ROOT,check=True)
    commit=git('rev-parse','HEAD');env=dict(os.environ)
    proxy=Path('/tmp/fyp_stage13c_git_ssh.sh')
    if proxy.exists():env['GIT_SSH_COMMAND']=str(proxy)
    push=subprocess.run(['git','push','--quiet','origin','main'],cwd=ROOT,env=env,timeout=180,capture_output=True,text=True)
    remote=git('ls-remote','origin','refs/heads/main',env=env) if push.returncode==0 else None
    dump('git_commit_receipt.json',dict(commit=commit,branch=git('branch','--show-current'),files_committed=staged,server_only_examples=server_only,push_status='SUCCESS' if push.returncode==0 and remote.split()[0]==commit else 'FAILED',push_error=push.stderr[-1000:] if push.returncode else None))
    assert read('git_commit_receipt.json')['push_status']=='SUCCESS'
    print('STAGE22A_COMMITTED_AND_PUSHED',commit,flush=True)
if __name__=='__main__':main()
