"""Small genuine monocular ORB-SLAM3 + image-only retrieval/geometry pilot."""
import os,subprocess,time,re,resource
from pathlib import Path
import cv2,pandas as pd
from scipy.spatial.transform import Rotation
from common import *

ORB=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')

def configuration(info):
    K=np.array(info['K']).reshape(3,3)*.5;K[2,2]=1;D=list(info['D'])+[0]*5
    return '%YAML:1.0\nFile.version: "1.0"\nCamera.type: "PinHole"\n'+''.join(f'{k}: {v}\n' for k,v in {
        'Camera1.fx':K[0,0],'Camera1.fy':K[1,1],'Camera1.cx':K[0,2],'Camera1.cy':K[1,2],
        'Camera1.k1':D[0],'Camera1.k2':D[1],'Camera1.p1':D[2],'Camera1.p2':D[3],'Camera1.k3':D[4],
        'Camera.width':640,'Camera.height':400,'Camera.fps':5,'Camera.RGB':0,
        'ORBextractor.nFeatures':1000,'ORBextractor.scaleFactor':1.2,'ORBextractor.nLevels':8,
        'ORBextractor.iniThFAST':20,'ORBextractor.minThFAST':7,'Viewer.KeyFrameSize':.05,
        'Viewer.KeyFrameLineWidth':1.,'Viewer.GraphLineWidth':.9,'Viewer.PointSize':2.,'Viewer.CameraSize':.08,
        'Viewer.CameraLineWidth':3.,'Viewer.ViewpointX':0.,'Viewer.ViewpointY':-.7,'Viewer.ViewpointZ':-1.8,'Viewer.ViewpointF':500.}.items())

def feature(frame):
    image=cv2.imread(str(ROOT/frame['file']),cv2.IMREAD_GRAYSCALE)
    k,d=cv2.ORB_create(nfeatures=1500).detectAndCompute(image,None)
    return np.array([x.pt for x in k],np.float32),d

def geometry(a,b,ka,kb):
    pa,da=a;pb,db=b
    result=dict(matches=0,inliers=0,inlier_ratio=0.,cheirality_inliers=0,median_parallax_deg=None,
                status='REJECT_INSUFFICIENT_MATCHES',relative_transform='T_candidateOptical_from_queryOptical; t direction only, no metric scale')
    if da is None or db is None:return result
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING);forward=matcher.knnMatch(da,db,k=2);backward=matcher.knnMatch(db,da,k=2)
    f={v[0].queryIdx:v[0].trainIdx for v in forward if len(v)==2 and v[0].distance<.75*v[1].distance}
    r={v[0].queryIdx:v[0].trainIdx for v in backward if len(v)==2 and v[0].distance<.75*v[1].distance}
    pairs=[(i,j) for i,j in f.items() if r.get(j)==i];result['matches']=len(pairs)
    if len(pairs)<12:return result
    def rays(p,info):
        K=np.array(info['K']).reshape(3,3).copy();K[:2]*=.5
        return cv2.undistortPoints(p.reshape(-1,1,2),K,np.array(info['D'])).reshape(-1,2)
    qa=rays(pa[[i for i,j in pairs]],ka);cb=rays(pb[[j for i,j in pairs]],kb)
    cv2.setRNGSeed(2401);E,mask=cv2.findEssentialMat(qa,cb,np.eye(3),method=cv2.RANSAC,prob=.999,threshold=.003)
    if E is None or E.shape!=(3,3):result['status']='REJECT_ESSENTIAL_ESTIMATION';return result
    result['inliers']=int(mask.sum());result['inlier_ratio']=float(mask.mean())
    n,R,t,good=cv2.recoverPose(E,qa,cb,np.eye(3),mask=mask.copy());result['cheirality_inliers']=int(n)
    ra=np.c_[qa,np.ones(len(qa))];rb=np.c_[cb,np.ones(len(cb))]@R
    ang=np.rad2deg(np.arccos(np.clip(np.einsum('ij,ij->i',ra,rb)/np.linalg.norm(ra,axis=1)/np.linalg.norm(rb,axis=1),-1,1)))
    active=good.ravel()>0;par=float(np.median(ang[active])) if active.any() else 0.
    result['median_parallax_deg']=par
    result.update(rotation_matrix=json.dumps(R.tolist()),translation_direction=json.dumps(t.ravel().tolist()))
    result['status']='EPIPOLAR_PLAUSIBLE_NONMETRIC' if len(pairs)>=50 and result['inlier_ratio']>=.5 and n>=30 and par>=1. else 'REJECT_WEAK_OR_DEGENERATE_GEOMETRY'
    return result

def run():
    if (B/'runtime_memory_pilot.json').exists():raise RuntimeError('Refuse to repeat accepted pilot')
    access=[];camera_guard(access);manifest=json.loads((B/'camera_data_manifest.json').read_text());policy=manifest['policy']
    recipe=dict(native_backend='existing ORB-SLAM3 MONOCULAR; viewer=false; no IMU',windows=policy['pilot_windows_camera_elapsed_s'],
        independent_map_per_window=True,scale='arbitrary monocular scale separately in each window/map; no metric oracle, no joint SE3 alignment',
        settings='example ORB defaults1000/1.2/8/20/7, actual native K/D scaled to640x400',
        candidate_method='independent camera-only OpenCLIP Single RGB global pilot opposite-robot ranking',
        candidate_selection='one frame nearest each elapsed integer second inside predeclared windows; no SC pool',
        verification=dict(mutual_ORB_ratio=.75,min_matches=50,normalized_RANSAC_threshold=.003,min_inlier_ratio=.5,min_cheirality=30,min_parallax_deg=1.),
        accepted_metric_loop_factors=0,GT_evaluation='NOT_RUN')
    dump(B/'visual_pilot_policy.json',recipe)
    runs=[];native_denied=[];evidence=[]
    exe=B/'bin/stage24_mono_pilot';vocab=ORB/'Vocabulary/ORBvoc.txt';lib=ORB/'lib/libORB_SLAM3.so'
    evidence=[dict(path=str(x),bytes=x.stat().st_size,sha256=sha(x)) for x in [exe,vocab,lib,ORB/'include/System.h',ORB/'src/System.cc']]
    for robot,data in manifest['robots'].items():
        for lo,hi in policy['pilot_windows_camera_elapsed_s']:
            out=B/'orb_pilot'/f'{robot}_{lo:03d}_{hi:03d}';out.mkdir(parents=True,exist_ok=True)
            frames=[x for x in data['pilot_frames'] if lo<=x['elapsed_s']<hi+.15]
            cfg=out/'camera.yaml';cfg.write_text(configuration(data['camera_info']))
            seq=out/'sequence.txt';seq.write_text(''.join(f"{int(f['camera_bag_timestamp_ns'])/1e9:.9f} {ROOT/f['file']}\n" for f in frames))
            cmd=['strace','-f','-e','trace=open,openat','-o',str(out/'native_file_access.log'),str(exe),str(vocab),str(cfg),str(seq),str(out/'frame_poses.csv'),str(out/'keyframes.tum')]
            start=time.monotonic()
            with (out/'stdout.log').open('w') as log:
                # Use native SDK dependencies, not cv2's injected Python-wheel libs.
                native_env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',
                    LD_LIBRARY_PATH=':'.join(map(str,[ORB/'lib',ORB.parent/'pangolin-install/lib',ORB/'Thirdparty/DBoW2/lib',ORB/'Thirdparty/g2o/lib'])))
                p=subprocess.run(['/usr/bin/time','-v']+cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=180,env=native_env)
            elapsed=time.monotonic()-start;text=(out/'stdout.log').read_text();trace=(out/'native_file_access.log').read_text()
            bad=[l for l in trace.splitlines() if any(k in l.lower() for k in ['gt_utm','gt_rel','/full_2hz/','scan_context','/22b_','_lidar.zip','clean_local_trajectory'])];native_denied.extend(bad)
            f=pd.read_csv(out/'frame_poses.csv');valid=f.valid_pose==1;good=f.loc[valid,['tx','ty','tz','qx','qy','qz','qw']].to_numpy()
            assert np.isfinite(good).all()
            if len(good):assert np.max(abs(np.linalg.norm(good[:,3:],axis=1)-1))<1e-5
            longest=cur=0
            for v in valid:cur=cur+1 if v else 0;longest=max(longest,cur)
            rss=re.search(r'Maximum resident set size \(kbytes\): (\d+)',text)
            runs.append(dict(robot_id=robot,window=[lo,hi],frames=len(f),returncode=p.returncode,valid_poses=int(valid.sum()),
                tracking_fraction=float(valid.mean()),longest_continuous_tracked_frames=longest,track_ms=stats(f.track_ms),
                wall_s=elapsed,peak_RSS_MiB=int(rss.group(1))/1024 if rss else None,
                new_map_log_count=text.count('Creation of new map'),reset_log_count=text.count('Reset'),metric_scale=False,trajectory=str((out/'frame_poses.csv').relative_to(ROOT)),
                interpretation='native tracking feasibility, NOT metric multi-robot SLAM or globally consistent continuous trajectory'))
            print('NATIVE_MONOCULAR',robot,lo,runs[-1]['valid_poses'],'/',len(f),flush=True)
    assert not native_denied,native_denied[:3]
    # Independent image-only candidate generation; no historical embeddings or SC IDs.
    import torch,open_clip
    from PIL import Image
    torch.set_num_threads(2);checkpoint=ROOT/'models/openclip/vit_b32_laion400m_e32.pt'
    model,_,preprocess=open_clip.create_model_and_transforms('ViT-B-32-quickgelu',pretrained=str(checkpoint));model.eval()
    selected={}
    for robot,data in manifest['robots'].items():
        selected[robot]=[]
        for lo,hi in policy['pilot_windows_camera_elapsed_s']:
            available=[f for f in data['pilot_frames'] if lo<=f['elapsed_s']<hi+.15]
            selected[robot].extend(min(available,key=lambda f:(abs(f['elapsed_s']-t),f['rgb_message_index'])) for t in range(lo,hi))
    embs={};features={};encode_start=time.monotonic()
    for robot,frames in selected.items():
        with torch.inference_mode():
            batches=[]
            for k in range(0,len(frames),4):
                inp=torch.stack([preprocess(Image.open(ROOT/f['file']).convert('RGB')) for f in frames[k:k+4]])
                e=model.encode_image(inp);e=e/e.norm(dim=1,keepdim=True);batches.append(e.numpy())
            embs[robot]=np.vstack(batches)
        features[robot]=[feature(f) for f in frames]
    encode_s=time.monotonic()-encode_start;rows=[]
    for qrobot,drobot in [('robot1','robot3'),('robot3','robot1')]:
        sims=embs[qrobot]@embs[drobot].T
        for i,q in enumerate(selected[qrobot]):
            j=int(np.argmax(sims[i]));candidate=selected[drobot][j]
            start=time.monotonic();g=geometry(features[qrobot][i],features[drobot][j],manifest['robots'][qrobot]['camera_info'],manifest['robots'][drobot]['camera_info'])
            rows.append(dict(query_robot=qrobot,query_RGB_message_index=q['rgb_message_index'],query_camera_timestamp_ns=q['camera_bag_timestamp_ns'],candidate_robot=drobot,
                candidate_RGB_message_index=candidate['rgb_message_index'],candidate_camera_timestamp_ns=candidate['camera_bag_timestamp_ns'],
                cosine_score=float(sims[i,j]),candidate_database_size=len(selected[drobot]),verification_ms=(time.monotonic()-start)*1000,**g))
    fields=list(dict.fromkeys(k for r in rows for k in r));write_csv(B/'visual_geometric_pilot.csv',[{k:r.get(k) for k in fields} for r in rows],fields)
    dump(B/'runtime_memory_pilot.json',dict(native_runs=runs,image_only_retrieval=dict(model='ViT-B-32-quickgelu',weights='laion400m_e32',checkpoint_sha256=sha(checkpoint),
        encoded_frames=sum(len(v) for v in selected.values()),encoding_and_features_s=encode_s,queries=len(rows),epipolar_plausible=sum(r['status']=='EPIPOLAR_PLAUSIBLE_NONMETRIC' for r in rows)),
        process_peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,native_binary_provenance=evidence,GT_used=False,metric_visual_loop_factors_created=0))
    dump(B/'visual_frontend_dependency_audit.json',dict(status='PASS',python_open_guard_enabled=True,native_strace_enabled=True,native_forbidden_reads=native_denied,
        input_sources=['original camera RGB','actual camera K/D','fixed pretrained image weights','ORB vocabulary'],
        LiDAR=False,SC=False,GICP=False,LiDAR_odometry=False,GT=False,Python_accesses=access,
        caveat='native strace audits file opens; not an OS sandbox. Camera-only graph is isolated per short window; no hidden metric scale.'))

if __name__=='__main__':run()
