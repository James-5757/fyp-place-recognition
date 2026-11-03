"""Isolated Stage23C methodology paths, immutable inputs, no model fitting."""
import hashlib,json,os,resource,sys,time
from pathlib import Path
import numpy as np,pandas as pd,gtsam
from scipy.spatial.transform import Rotation
ROOT=Path('/home/cas/fyp_place_recognition');SRC=ROOT/'src/cumulti';OLD=ROOT/'outputs/cumulti_v1/23_information_aware';OUT=ROOT/'outputs/cumulti_v1/23c_information_validity'
sys.path.insert(0,str(SRC/'stage23'));import diagnostic_model as m
P0=m.P0;P5=m.P5;A=m.A;B=m.B;SIG=m.SIG
def sha(p):return m.sha(p)
def read(p):return m.read(p)
def dump(n,x):
    p=OUT/n;tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n');os.replace(tmp,p)
def csv(n,x):
    (x if isinstance(x,pd.DataFrame) else pd.DataFrame(x)).to_csv(OUT/n,index=False,lineterminator='\n')
def graph_without(full,idx,pairs):
    remove={idx[tuple(x)] for x in pairs};g=gtsam.NonlinearFactorGraph()
    for i in range(full.size()):
        if i not in remove:g.add(full.at(i))
    return g
def Cpair(marg,ks):
    c=marg.jointMarginalCovariance(gtsam.KeyVector(ks));return np.block([[c.at(a,b) for b in ks] for a in ks])
def Jbetween(v,ks):
    a=np.zeros((6,6),order='F');b=a.copy(order='F');v.atPose3(ks[0]).between(v.atPose3(ks[1]),a,b);return np.hstack([a,b])/SIG[:,None]
def residual(v,q,c,Z):
    E=gtsam.Pose3(Z).inverse().compose(v.atPose3(m.key(('robot1',q))).between(v.atPose3(m.key(('robot3',c)))))
    e=gtsam.Pose3.Logmap(E);n=float(np.linalg.norm(e/SIG));return e,n,min(1.,1.345/n) if n else 1.
def gaussian_A(factor,v,keys):
    gf=factor.linearize(v);A,_=gf.jacobian();fk=list(gf.keys());out=np.zeros((len(A),6*len(keys)));pos={k:i for i,k in enumerate(keys)}
    for i,k in enumerate(fk):out[:,6*pos[k]:6*pos[k]+6]=A[:,6*i:6*i+6]
    return out
def gain(A,C):
    V=A@C@A.T;V=(V+V.T)/2;ev=np.linalg.eigvalsh(V);assert ev.min()>-1e-5
    return float(.5*np.log1p(np.maximum(ev,0)).sum())
def budget(start):
    policy=read(OUT/'stage23c_policy.json')
    if time.monotonic()-start>policy['additional_calculation_cap_s'] or resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024>2048:raise TimeoutError('Frozen Stage23C900s/2GiB limit')
def freeze():
    assert not OUT.exists(),'Fresh output directory required; never overwrite a stage'
    OUT.mkdir(parents=True)
    import subprocess
    subprocess.run(['git','merge-base','--is-ancestor','9e802e72ddfb8afc03bbc98ecde8990c06da93d6','HEAD'],cwd=ROOT,check=True)
    inputs=[p for p in OLD.rglob('*') if p.is_file() and '__pycache__' not in str(p)]
    inputs += list((SRC/'stage23').glob('*.py'))+[ROOT/'docs/CUMULTI_STAGE23B_CRITICAL_LOOP_DIAGNOSTICS.md',ROOT/'NEXT_STEPS.md',ROOT/'RESULTS.md',ROOT/'EXPERIMENT_LOG.md',ROOT/'PROJECT_CONTEXT.md']
    inputs += [B/'policies'/n/f for n in [P0,P5] for f in ['online_verified_loops.csv','online_batch_trajectory.csv','online_backend_policy.json']]
    inputs += [A/'dual_stream_event_schedule.csv',B/'stage22b_experiment_policy.json',ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv']
    seen=set()
    for n in [P0,P5]:
        for row in pd.read_csv(B/'policies'/n/'online_verified_loops.csv').itertuples():
            ident=(int(row.event_index),int(row.candidate_rank))
            if ident in seen:continue
            seen.add(ident);p=A/'attempts'/f'e{ident[0]:04d}_r{ident[1]}.json';inputs.extend([p,p.with_suffix('.csv')])
    optional=ROOT/'outputs/cumulti_v1/20_dual_stream/stage20_policy.json'
    if optional.exists():inputs.append(optional)
    dump('stage23c_input_manifest.json',dict(reference_commit='9e802e72ddfb8afc03bbc98ecde8990c06da93d6',inputs={str(p):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in inputs},existing_dirty_worktree=os.popen('git -C '+str(ROOT)+' status --short').read(),raw_scans_not_needed=True,raw_GT_not_read_by_calculation=True))
    dump('stage23c_policy.json',dict(additional_calculation_cap_s=900,RSS_cap_GiB=2,new_nonlinear_solver_calls=0,new_GICP_calls=0,pilot_groups=['GROUP_CLUSTER_3','GROUP_CLUSTER_0'],representatives='unchanged14 Stage23B pilot IDs; no GT-driven case selection',cohorts=dict(P0_only=141,P5_only=84,common=147,total=372),bootstrap_seed=2303,bootstrap_replicates=1000,primary_bootstrap='seven frozen Stage23B missing-loop temporal clusters resampled with replacement; preserve within-cluster dependence; CI exploratory with only7 clusters',secondary_bootstrap='IID factors, explicitly dependence-ignoring sensitivity',top_K=[10,20],small_N_rule='n<20: no bootstrap CI, exploratory correlation only',state_rule='never substitute full/future optimized values for a missing prior-event optimized state',fixed_information='sparse robust Gaussian factor Jacobians at frozen full-P0 Batch reference, with the SAME values for full/removed-group graph; group covariance preserves joint cross-blocks',nonlinear_comparison='read already solved frozen LOO states; no new optimize; state+robust reweight effects not uniquely causally separated',group_information='exact fixed-Gaussian logdet joint addition to graph without group; compare sum individual additions to that SAME group-removed base',signed_impact='unchanged Stage23B signed deltas; no clipping negatives or fitting cutoffs',predictions='no optimized combination, learned weights, thresholds, classifier or deployed selector',independent_validation='NO_GO_INSUFFICIENT_BIDIRECTIONAL_COVERAGE'))
    dump('stage23c_lock.json',dict(input_manifest_sha256=sha(OUT/'stage23c_input_manifest.json'),policy_sha256=sha(OUT/'stage23c_policy.json')))
def verify():
    lock=read(OUT/'stage23c_lock.json');assert sha(OUT/'stage23c_policy.json')==lock['policy_sha256'] and sha(OUT/'stage23c_input_manifest.json')==lock['input_manifest_sha256']
    for p,x in read(OUT/'stage23c_input_manifest.json')['inputs'].items():assert sha(p)==x['sha256'],p
if __name__=='__main__':freeze()
