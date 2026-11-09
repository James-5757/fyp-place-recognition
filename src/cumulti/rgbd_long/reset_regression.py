"""One real known-reset regression plus identity/gap corner cases; old outputs read-only."""
import csv,hashlib,json,os,shutil,subprocess,time
import numpy as np
from export import ROOT,OUT,dump
from track import ORB,frame_identity_events,segments

def run():
    out=OUT/'reset_regression';out.mkdir(parents=True,exist_ok=True)
    if (out/'validation.json').exists():return
    # Same native map before/after a gap must NOT become a new coordinate frame;
    # reused map ID with a new initial KF must. This guards two different bugs.
    rows=[dict(map_id='0',map_init_kf=k) for k in ['0','0','0','0','7']]
    valid=[True,False,True,True,True];ts=np.arange(5,dtype=float)
    fp,events=frame_identity_events(rows,valid,ts)
    assert fp==[(0,0),(0,7)] and len(events)==1 and events[0]['index']==4
    assert len(segments(rows,valid,ts))==3
    old=ROOT/'outputs/cumulti_v1/24_rgbd_demo_v1/robot1'
    source=json.loads((old/'export.json').read_text());frames=source['frames'];origin=int(frames[0]['rgb_bag_ns'])
    cfg=out/'camera.yaml';shutil.copyfile(old/'paced/camera.yaml',cfg)
    seq=out/'sequence.txt';seq.write_text(''.join(f"{int(f['rgb_bag_ns'])-origin} {ROOT/f['rgb']} {ROOT/f['depth']}\n" for f in frames))
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=':'.join(map(str,[ORB/'lib',ORB.parent/'pangolin-install/lib',ORB/'Thirdparty/DBoW2/lib',ORB/'Thirdparty/g2o/lib'])))
    cmd=['strace','-f','-e','trace=open,openat','-o',str(out/'native_access.log'),str(OUT/'bin/rgbd_long'),str(ORB/'Vocabulary/ORBvoc.txt'),str(cfg),str(seq),str(out/'frame_poses.csv'),str(out/'corrected_native_ns.tum')]
    start=time.monotonic()
    with (out/'stdout.log').open('w') as log:p=subprocess.run(cmd,env=env,cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=90)
    assert p.returncode==0
    trace=(out/'native_access.log').read_text();bad=[line for line in trace.splitlines() if any(s in line.lower() for s in ['_lidar.zip','gt_utm','gt_rel','imu_gps','/full_2hz/','/22b_','/12d_'])];assert not bad
    observed=list(csv.DictReader((out/'frame_poses.csv').open()));valid=[r['valid_pose']=='1' for r in observed];times=np.array([int(r['relative_ns'])/1e9 for r in observed])
    fp,events=frame_identity_events(observed,valid,times);text=(out/'stdout.log').read_text();resets=text.count('LM: Active map reset recieved')
    real=bool(resets>=1 and any(e['previous_map']==e['map_id'] and e['previous_origin_kf']!=e['origin_kf'] for e in events))
    value=dict(status='PASS_REAL_MAP_ID_REUSE_RESET_DETECTED' if real else 'NOT_EXERCISED_REAL_RESET',frames=len(observed),valid_frames=sum(valid),fingerprints=fp,
        actual_native_reset_log_count=resets,events=events,synthetic_gap_and_reused_id_checks='PASS',
        immutable_source=str(old),settings_sha256=hashlib.sha256(cfg.read_bytes()).hexdigest(),wall_s=time.monotonic()-start,
        GT=False,LiDAR=False,native_forbidden_reads=bad,legacy_results_overwritten=False)
    dump(out/'validation.json',value);print(json.dumps(value),flush=True)

if __name__=='__main__':run()
