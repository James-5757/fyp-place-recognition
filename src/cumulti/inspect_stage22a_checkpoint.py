"""Verify durable receipts before resuming; never rerun a measurement."""
from stage22a_common import *
def main():
    verify_freeze();rows=[];unresolved=[];used=0.
    for p in sorted((OUT/'attempts').glob('*.intent.json')):
        stem=p.name.removesuffix('.intent.json');receipt=p.with_name(stem+'.json');raw=p.with_name(stem+'.csv')
        if not receipt.exists():unresolved.append(stem);continue
        data=json.loads(receipt.read_text());m=data['measurement'];assert sha(raw)==m['result_csv_sha256']
        assert data['candidate_identity'][0]==m['event_index'] and data['candidate_identity'][1]==m['candidate_rank']
        used+=m['call_wall_ms']/1000;rows.append(stem)
    assert len(rows)==len(set(rows))
    report=dict(status='SAFE_TO_RESUME' if not unresolved else 'BLOCKED_UNRESOLVED_INFLIGHT',durable_actual_calls=len(rows),uncertain_attempts=unresolved,actual_call_budget_used_seconds=used,frozen_total_cap_seconds=7200,remaining_budget_seconds=max(0,7200-used),measurements_rerun=0,prior_progress=read('verification_progress.json'),interruption='SSH connection reset; old verification process no longer running; prior finish-only waiter timed out',recovery='reuse hash-verified event/rank receipts; chronological replay rebuilds duplicate state without rerunning GICP',GT_used=False)
    dump('checkpoint_recovery_audit.json',report);print(json.dumps(report),flush=True);assert not unresolved
if __name__=='__main__':main()
