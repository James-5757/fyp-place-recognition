"""Fixed native-rate configurations, observed map fingerprints and strict fusion gate."""
import argparse,csv,fcntl,hashlib,json,os,re,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from export import ROOT,OUT,dump,policy
ORB=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')

def writecsv(path,rows,fields=None):
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields or list(rows[0]));writer.writeheader();writer.writerows(rows)

def segments(rows,valid,times):
    result=[];begin=None;previous=None
    for i,(r,ok) in enumerate(zip(rows,valid)):
        fp=(r['map_id'],r['map_init_kf'])
        if ok and (begin is None or fp==previous):
            if begin is None:begin=i
        else:
            if begin is not None:result.append(dict(start=begin,end=i-1,duration_s=float(times[i-1]-times[begin]),frames=i-begin,map_id=previous[0],map_init_kf=previous[1]));begin=None
            if ok:begin=i
        previous=fp if ok else None
    if begin is not None:result.append(dict(start=begin,end=len(rows)-1,duration_s=float(times[-1]-times[begin]),frames=len(rows)-begin,map_id=previous[0],map_init_kf=previous[1]))
    return result

def frame_identity_events(rows,valid,times):
    fingerprints=[];events=[];previous=None;last_valid_index=None
    for i,(r,good) in enumerate(zip(rows,valid)):
        if not good:continue
        fp=(int(r['map_id']),int(r['map_init_kf']))
        if fp!=previous:
            if previous is not None:events.append(dict(index=i,relative_s=float(times[i]),previous_map=previous[0],previous_origin_kf=previous[1],map_id=fp[0],origin_kf=fp[1],last_valid_index=last_valid_index,classification='COORDINATE_FRAME_CHANGE_OR_RESET'))
            fingerprints.append(fp);previous=fp
        last_valid_index=i
    return fingerprints,events

def run(robot,config):
    spec=policy();parameters=next(c for c in spec['configurations'] if c['name']==config)
    integrity=dict(max_adjacent_translation_m=2.,max_adjacent_speed_m_s=10.,max_adjacent_rotation_deg=45.,
        observer_min_map_vote_fraction=.8,scope='adjacent valid frames in the same native map fingerprint; conservative engineering limits, not GT-derived')
    declared=OUT/'integrity_policy.json'
    if declared.exists():assert json.loads(declared.read_text())==integrity
    else:dump(declared,integrity)
    out=OUT/robot/config;out.mkdir(parents=True,exist_ok=True)
    # A resume orchestration waits for this same configuration's active owner,
    # then reuses its atomic completed report instead of launching a duplicate.
    lock=(out/'.tracking.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
    if (out/'metrics.json').exists():print('TRACK_REUSED',robot,config,flush=True);return
    data=json.loads((OUT/robot/'export.json').read_text());K=np.array(data['scaled_K']);D=data['camera_info']['D']
    settings={'Camera1.fx':K[0,0],'Camera1.fy':K[1,1],'Camera1.cx':K[0,2],'Camera1.cy':K[1,2],
        'Camera1.k1':D[0],'Camera1.k2':D[1],'Camera1.p1':D[2],'Camera1.p2':D[3],'Camera1.k3':D[4],
        'Camera.width':640,'Camera.height':400,'Camera.fps':10,'Camera.RGB':0,
        'Stereo.b':.05,'Stereo.ThDepth':40.,'RGBD.DepthMapFactor':1000.,
        'ORBextractor.nFeatures':parameters['ORB_features'],'ORBextractor.scaleFactor':1.2,'ORBextractor.nLevels':8,
        'ORBextractor.iniThFAST':20,'ORBextractor.minThFAST':7,'Viewer.KeyFrameSize':.05,
        'Viewer.KeyFrameLineWidth':1.,'Viewer.GraphLineWidth':.9,'Viewer.PointSize':2.,'Viewer.CameraSize':.08,
        'Viewer.CameraLineWidth':3.,'Viewer.ViewpointX':0.,'Viewer.ViewpointY':-.7,'Viewer.ViewpointZ':-1.8,'Viewer.ViewpointF':500.}
    cfg=out/'camera.yaml';cfg.write_text('%YAML:1.0\nFile.version: "1.0"\nCamera.type: "PinHole"\n'+''.join(f'{k}: {v}\n' for k,v in settings.items()))
    frames=data['frames'];origin=int(frames[0]['rgb_bag_ns']);seq=out/'sequence.txt'
    seq.write_text(''.join(f"{int(f['rgb_bag_ns'])-origin} {ROOT/f['rgb']} {ROOT/f['depth']}\n" for f in frames))
    native_env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=':'.join(map(str,[ORB/'lib',ORB.parent/'pangolin-install/lib',ORB/'Thirdparty/DBoW2/lib',ORB/'Thirdparty/g2o/lib'])))
    command=['strace','-f','-e','trace=open,openat','-o',str(out/'native_access.log'),str(OUT/'bin/rgbd_long'),str(ORB/'Vocabulary/ORBvoc.txt'),str(cfg),str(seq),str(out/'frame_poses.csv'),str(out/'corrected_native_ns.tum')]
    start=time.monotonic()
    with (out/'stdout.log').open('w') as log:
        result=subprocess.run(['/usr/bin/time','-v']+command,cwd=out,env=native_env,stdout=log,stderr=subprocess.STDOUT,timeout=360)
    elapsed=time.monotonic()-start
    trace=(out/'native_access.log').read_text();denied=[line for line in trace.splitlines() if any(s in line.lower() for s in ['_lidar.zip','gt_utm','gt_rel','imu_gps','/full_2hz/','/22b_','/12d_','clean_local_trajectory'])]
    assert result.returncode==0 and not denied,(result.returncode,denied[:1])
    rows=list(csv.DictReader((out/'frame_poses.csv').open()));assert len(rows)==len(frames)
    valid=np.array([r['valid_pose']=='1' for r in rows]);times=np.array([int(r['relative_ns'])/1e9 for r in rows]);assert np.all(np.diff(times)>0)
    finite=True
    for r,good in zip(rows,valid):
        if good:
            p=np.array([float(r[k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]);finite &= bool(np.isfinite(p).all() and abs(np.linalg.norm(p[3:])-1)<1e-5)
    spans=segments(rows,valid,times);fingerprints,events=frame_identity_events(rows,valid,times)
    # Cover each sampled interval using its left endpoint's actual tracked state.
    weights=np.diff(times);weighted=float(np.sum(weights*valid[:-1])/weights.sum())
    lost_start=None;lost=[]
    for i,good in enumerate(valid):
        if not good and lost_start is None:lost_start=i
        if good and lost_start is not None:lost.append(times[i]-times[lost_start]);lost_start=None
    if lost_start is not None:lost.append(times[-1]-times[lost_start]+np.median(weights))
    text=(out/'stdout.log').read_text();active_resets=text.count('LM: Active map reset recieved')
    gate=spec['fusion_gate'];maximum_lost=float(max(lost,default=0));longest=max((s['duration_s'] for s in spans),default=0)
    checks=dict(valid_fraction=float(valid.mean())>=gate['valid_pose_fraction_min'],
        time_weighted_fraction=weighted>=gate['time_weighted_tracking_fraction_min'],longest_continuous=longest>=90,
        coordinate_frame_changes=len(events)==0 and active_resets==0,max_untracked_gap=maximum_lost<=1,
        finite_unit_poses=finite,native_input_audit=not denied)
    corrected=[];native=out/'corrected_native_ns.tum';by_ns={int(r['relative_ns']):(i,r) for i,r in enumerate(rows) if r['valid_pose']=='1'}
    if native.exists():
        for line in native.read_text().splitlines():
            values=line.split();stamp=int(round(float(values[0])))
            if stamp not in by_ns:continue # Never admit RECENTLY_LOST predicted poses.
            i,r=by_ns[stamp];v=np.array([float(x) for x in values[1:]]);assert len(v)==7 and np.isfinite(v).all() and abs(np.linalg.norm(v[3:])-1)<1e-5
            corrected.append(dict(index=i,RGB_original_index=frames[i]['original_RGB_index'],RGB_bag_ns=frames[i]['rgb_bag_ns'],relative_ns=stamp,
                **dict(zip(['tx','ty','tz','qx','qy','qz','qw'],v))))
    if corrected:
        assert len({r['index'] for r in corrected})==len(corrected)
        writecsv(out/'corrected_camera_trajectory.csv',corrected)
    checks['final_native_trajectory_coverage']=len(corrected)/len(rows)>=.95
    motion=[]
    for i in range(1,len(rows)):
        a,b=rows[i-1],rows[i]
        if not (valid[i-1] and valid[i] and (a['map_id'],a['map_init_kf'])==(b['map_id'],b['map_init_kf'])):continue
        x=np.array([float(a[k]) for k in ['tx','ty','tz']]);y=np.array([float(b[k]) for k in ['tx','ty','tz']]);distance=float(np.linalg.norm(y-x))
        qa=Rotation.from_quat([float(a[k]) for k in ['qx','qy','qz','qw']]);qb=Rotation.from_quat([float(b[k]) for k in ['qx','qy','qz','qw']]);angle=float(np.rad2deg((qa.inv()*qb).magnitude()))
        motion.append(dict(index=i,relative_s=float(times[i]),dt_s=float(times[i]-times[i-1]),translation_m=distance,speed_m_s=distance/(times[i]-times[i-1]),rotation_deg=angle))
    checks['no_catastrophic_adjacent_jump']=bool(motion) and all(r['translation_m']<=2 and r['speed_m_s']<=10 and r['rotation_deg']<=45 for r in motion)
    writecsv(out/'motion_samples.csv',motion,['index','relative_s','dt_s','translation_m','speed_m_s','rotation_deg'])
    writecsv(out/'tracking_segments.csv',spans,['start','end','duration_s','frames','map_id','map_init_kf'])
    writecsv(out/'reset_events.csv',events,['index','relative_s','previous_map','previous_origin_kf','map_id','origin_kf','last_valid_index','classification'])
    rss=re.search(r'Maximum resident set size \(kbytes\): (\d+)',text)
    metrics=dict(robot=robot,config=config,frames=len(rows),duration_s=float(times[-1]),valid_frames=int(valid.sum()),valid_fraction=float(valid.mean()),time_weighted_fraction=weighted,
        longest_continuous_s=longest,max_untracked_gap_s=float(maximum_lost),coordinate_fingerprints=fingerprints,
        observed_frame_changes=len(events),native_active_reset_log_count=active_resets,corrected_tracked_poses=len(corrected),
        native_wall_s=elapsed,median_track_ms=float(np.median([float(r['track_ms']) for r in rows])),p95_track_ms=float(np.percentile([float(r['track_ms']) for r in rows],95)),
        peak_RSS_MiB=int(rss.group(1))/1024 if rss else None,first12s_valid_frames=int(valid[times<12].sum()),first12s_input_frames=int((times<12).sum()),
        checks=checks,fusion_gate='PASS' if all(checks.values()) else 'FAIL',GT=False,LiDAR=False,IMU=False,native_forbidden_reads=denied,
        motion_max=dict(translation_m=max((r['translation_m'] for r in motion),default=None),speed_m_s=max((r['speed_m_s'] for r in motion),default=None),rotation_deg=max((r['rotation_deg'] for r in motion),default=None)),
        settings=settings,binary_sha256=hashlib.sha256((OUT/'bin/rgbd_long').read_bytes()).hexdigest())
    dump(out/'metrics.json',metrics);print(json.dumps(metrics),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('robot',choices=['robot1','robot3']);p.add_argument('--config',choices=['baseline1000','fallback2000'],default='baseline1000');a=p.parse_args();run(a.robot,a.config)
