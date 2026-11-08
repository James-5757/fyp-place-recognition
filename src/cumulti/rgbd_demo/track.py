"""Run a fixed bounded RGB-D window, audit inputs and report resets/failures honestly."""
import csv, fcntl, json, os, re, subprocess, time
from pathlib import Path
import numpy as np
from export import ROOT, OUT, dump

ORB=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')

def run(robot,mode='paced'):
    base=OUT/robot;out=base/'paced' if mode=='paced' else base;out.mkdir(parents=True,exist_ok=True);report=out/'tracking.json'
    lock=(out/'.tracking.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if report.exists():print('TRACK_ALREADY_COMPLETE',robot,flush=True);return
    data=json.loads((base/'export.json').read_text());K=np.array(data['scaled_K']);D=data['camera_info']['D']
    # RGB-D's pseudo-stereo baseline is an explicit algorithm parameter, not the
    # RGB/depth optical mounting offset or an inferred physical IR baseline.
    baseline=.05
    settings={'Camera1.fx':K[0,0],'Camera1.fy':K[1,1],'Camera1.cx':K[0,2],'Camera1.cy':K[1,2],
        'Camera1.k1':D[0],'Camera1.k2':D[1],'Camera1.p1':D[2],'Camera1.p2':D[3],'Camera1.k3':D[4],
        'Camera.width':640,'Camera.height':400,'Camera.fps':5,'Camera.RGB':0,
        'Stereo.b':baseline,'Stereo.ThDepth':40.,'RGBD.DepthMapFactor':1000.,
        'ORBextractor.nFeatures':1000,'ORBextractor.scaleFactor':1.2,'ORBextractor.nLevels':8,
        'ORBextractor.iniThFAST':20,'ORBextractor.minThFAST':7,'Viewer.KeyFrameSize':.05,
        'Viewer.KeyFrameLineWidth':1.,'Viewer.GraphLineWidth':.9,'Viewer.PointSize':2.,'Viewer.CameraSize':.08,
        'Viewer.CameraLineWidth':3.,'Viewer.ViewpointX':0.,'Viewer.ViewpointY':-.7,'Viewer.ViewpointZ':-1.8,'Viewer.ViewpointF':500.}
    cfg=out/'camera.yaml';cfg.write_text('%YAML:1.0\nFile.version: "1.0"\nCamera.type: "PinHole"\n'+''.join(f'{k}: {v}\n' for k,v in settings.items()))
    # Subtract first bag timestamp BEFORE float conversion to preserve nanosecond source IDs.
    origin=int(data['frames'][0]['rgb_bag_ns'])
    seq=out/'sequence.txt';seq.write_text(''.join(f"{(int(f['rgb_bag_ns'])-origin)/1e9:.9f} {ROOT/f['rgb']} {ROOT/f['depth']}\n" for f in data['frames']))
    native_env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=':'.join(map(str,[ORB/'lib',ORB.parent/'pangolin-install/lib',ORB/'Thirdparty/DBoW2/lib',ORB/'Thirdparty/g2o/lib'])))
    command=['strace','-f','-e','trace=open,openat','-o',str(out/'native_access.log'),str(OUT/'bin/rgbd_pilot'),
        str(ORB/'Vocabulary/ORBvoc.txt'),str(cfg),str(seq),str(out/'frame_poses.csv'),str(out/'final_trajectory.tum'),mode]
    start=time.monotonic()
    with (out/'tracking_stdout.log').open('w') as log:
        result=subprocess.run(['/usr/bin/time','-v']+command,cwd=out,env=native_env,stdout=log,stderr=subprocess.STDOUT,timeout=240)
    runtime=time.monotonic()-start
    trace=(out/'native_access.log').read_text()
    forbidden=[line for line in trace.splitlines() if any(s in line.lower() for s in ['_lidar.zip','gt_utm','gt_rel','imu_gps','/full_2hz/','/22b_','/12d_','clean_local_trajectory'])]
    assert not forbidden
    assert result.returncode==0,('native failed',result.returncode)
    rows=list(csv.DictReader((out/'frame_poses.csv').open()));good=[r for r in rows if r['valid_pose']=='1']
    assert len(rows)==60
    for row in good:
        values=np.array([float(row[k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]);assert np.isfinite(values).all()
        assert abs(np.linalg.norm(values[3:])-1)<1e-5
    maps=sorted({int(r['map_id']) for r in good});epochs=sorted({int(r['tracking_epoch']) for r in good});longest=current=0;last=None
    for row in rows:
        if row['valid_pose']=='1':current=current+1 if row['map_id']==last else 1;last=row['map_id']
        else:current=0;last=None
        longest=max(longest,current)
    text=(out/'tracking_stdout.log').read_text();rss=re.search(r'Maximum resident set size \(kbytes\): (\d+)',text)
    report_data=dict(robot=robot,frames=len(rows),valid_poses=len(good),tracking_fraction=len(good)/len(rows),
        map_ids=maps,tracking_epochs=epochs,longest_continuous_tracked_frames=longest,wall_s=runtime,pacing=mode,
        median_track_ms=float(np.median([float(r['track_ms']) for r in rows])),
        p95_track_ms=float(np.percentile([float(r['track_ms']) for r in rows],95)),
        peak_RSS_MiB=int(rss.group(1))/1024 if rss else None,
        runtime_includes='strace input audit, vocabulary loading and shutdown; no real-time claim',
        metric_source='original RealSense camera XYZ metres, registered color Z; no LiDAR/GT/IMU',
        native_forbidden_reads=forbidden,returncode=result.returncode,relative_timestamp_origin_ns=str(origin),
        settings=settings,baseline_source='predeclared 0.05m virtual RGB-D pseudo-stereo baseline, not a measured IR stereo baseline; close/far threshold 2m',
        demo_acceptance_rule='>=45/60 tracked, >=40 continuous frames, exactly one map and tracking epoch, finite unit poses; no GT',
        status='PASS' if len(good)>=45 and longest>=40 and len(maps)==1 and len(epochs)==1 else 'PARTIAL_TRACKING_FAILURE')
    dump(report,report_data);print(json.dumps(report_data),flush=True)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('robot',choices=['robot1','robot3']);p.add_argument('--mode',choices=['paced','unpaced'],default='paced');a=p.parse_args();run(a.robot,a.mode)
