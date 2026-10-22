"""Correct sequential LRU cost estimation only; never rewrite primary frozen decisions."""
from common import *
from replay import Data,run


def main():
    verify_lock();guard()
    primary_hash=sha(OUT/'pre_gt_decision_lock.json')
    old=OUT/'cache_sensitivity.csv'
    backup=OUT/'cache_sensitivity_PRE_LRU_ENGINEERING_FIX.csv'
    if not backup.exists():old.rename(backup)
    data=Data();threshold=read(OUT/'finalist_lock.json')['threshold'];rows=[];selected={}
    for name in ['P0_FULL_TOP3','P5_BUDGET_AWARE_C75_B80']:
        for capacity in [None,1000,128]:
            result,attempts,_,_=run(data,name,threshold,.75 if name.startswith('P5') else 1,.8 if name.startswith('P5') else 1,capacity=capacity)
            assert result['application_bytes']==result['point_cloud_bytes']+result['factor_update_bytes']+result['descriptor_bytes']+301340
            rows.append(result)
            selected[name+':'+str(capacity)]=dict(policy=name,capacity=capacity,
                pairs=[[int(r['event_index']),int(r['candidate_rank'])] for r in attempts])
    csv(old,rows)
    write_json(OUT/'cache_sensitivity_selected_pairs.json',selected)
    assert sha(OUT/'pre_gt_decision_lock.json')==primary_hash
    write_json(OUT/'cache_sensitivity_repair.json',dict(reason='A query miss can evict an old candidate between two accesses. Simulate request sequence rather than set-membership-only planning.',
        primary_unbounded_results_unchanged=True,GT_used=False,old_sha256=sha(backup),new_sha256=sha(old),
        scope='finite LRU sensitivity accounting and its budget-responsive choices only; bug is not a new policy or tuned parameter'))

if __name__=='__main__':main()
