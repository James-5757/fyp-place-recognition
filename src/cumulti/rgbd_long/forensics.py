"""Describe every tracking-loss interval from the accepted runs; no parameter search."""
import csv,json
import cv2,numpy as np
from export import ROOT,OUT,dump
from track import writecsv

def run():
    cv2.setNumThreads(1);losses=[];summaries=[]
    for robot in ['robot1','robot3']:
        source=json.loads((OUT/robot/'export.json').read_text());frames=source['frames'];quality=[]
        orb=cv2.ORB_create(nfeatures=1000)
        for f in frames:
            im=cv2.imread(str(ROOT/f['rgb']),cv2.IMREAD_GRAYSCALE)
            keys=orb.detect(im,None);depth=cv2.imread(str(ROOT/f['depth']),cv2.IMREAD_UNCHANGED)
            uv=np.rint([k.pt for k in keys]).astype(int).reshape(-1,2)
            depth_support=float(np.mean(depth[uv[:,1],uv[:,0]]>0)) if len(uv) else 0.
            quality.append(dict(index=f['index'],mean_intensity=float(im.mean()),intensity_std=float(im.std()),
                Laplacian_variance=float(cv2.Laplacian(im,cv2.CV_64F).var()),
                bright_pixel_fraction=float(np.mean(im>=245)),dark_pixel_fraction=float(np.mean(im<=10)),
                OpenCV_ORB1000_proxy_keypoints=len(keys),ORB_proxy_valid_depth_fraction=depth_support,depth_coverage=f['depth_coverage']))
        writecsv(OUT/robot/'image_quality_proxy.csv',quality)
        ts=np.array([int(f['rgb_bag_ns']) for f in frames]);cadence=np.diff(ts)/1e6
        for config in ['baseline1000','fallback2000']:
            folder=OUT/robot/config;p=folder/'frame_poses.csv'
            if not p.exists():continue
            rows=list(csv.DictReader(p.open()));good=np.array([r['valid_pose']=='1' for r in rows]);times=np.array([int(r['relative_ns'])/1e9 for r in rows])
            bad=np.flatnonzero(~good);groups=np.split(bad,np.flatnonzero(np.diff(bad)>1)+1) if len(bad) else []
            for number,indices in enumerate(groups):
                a=int(indices[0]);b=int(indices[-1]);duration=float(times[min(b+1,len(times)-1)]-times[a])
                row=dict(robot=robot,config=config,loss_interval=number,start_index=a,end_index=b,start_s=float(times[a]),end_s=float(times[b]),duration_to_recovery_s=duration,
                    source_RGB_first=frames[a]['original_RGB_index'],source_RGB_last=frames[b]['original_RGB_index'],
                    depth_coverage_median=float(np.median([quality[i]['depth_coverage'] for i in indices])),
                    image_Laplacian_median=float(np.median([quality[i]['Laplacian_variance'] for i in indices])),
                    ORB_proxy_keypoints_median=float(np.median([quality[i]['OpenCV_ORB1000_proxy_keypoints'] for i in indices])),
                    max_bag_sync_ms=max(frames[i]['bag_sync_ms'] for i in indices),
                    max_header_sync_ms=max(frames[i]['header_sync_ms'] for i in indices),missing_depth_frames=sum(frames[i]['depth_status']!='PAIRED' for i in indices))
                losses.append(row)
                chosen=[max(0,a-5),a,(a+b)//2,b,min(len(frames)-1,b+5)];sheet=np.zeros((240,len(chosen)*320,3),np.uint8)
                for column,i in enumerate(chosen):
                    image=cv2.resize(cv2.imread(str(ROOT/frames[i]['rgb'])),(320,200));sheet[:200,column*320:(column+1)*320]=image
                    cv2.putText(sheet,f"f{i} t{times[i]:.2f}s state{rows[i]['tracking_state']}",(column*320+4,219),cv2.FONT_HERSHEY_SIMPLEX,.4,(240,240,240),1)
                    cv2.putText(sheet,f"depth{quality[i]['depth_coverage']:.2f} ORB{quality[i]['OpenCV_ORB1000_proxy_keypoints']}",(column*320+4,235),cv2.FONT_HERSHEY_SIMPLEX,.35,(190,220,200),1)
                cv2.imwrite(str(folder/f'loss_interval_{number:02d}.jpg'),sheet)
            def median(field,mask):return float(np.median([q[field] for q,active in zip(quality,mask) if active])) if mask.any() else None
            text=(folder/'stdout.log').read_text()
            measured=json.loads((folder/'metrics.json').read_text())
            summaries.append(dict(robot=robot,config=config,loss_intervals=len(groups),
                RGB_gap_max_ms=float(cadence.max()),RGB_gap_p95_ms=float(np.percentile(cadence,95)),
                input_depth_all_paired=all(f['depth_status']=='PAIRED' for f in frames),
                paired_header_deltas_all_zero=all(f['header_sync_ms']==0 for f in frames),
                tracked_depth_coverage_median=median('depth_coverage',good),untracked_depth_coverage_median=median('depth_coverage',~good),
                tracked_Laplacian_median=median('Laplacian_variance',good),untracked_Laplacian_median=median('Laplacian_variance',~good),
                tracked_ORB_proxy_median=median('OpenCV_ORB1000_proxy_keypoints',good),untracked_ORB_proxy_median=median('OpenCV_ORB1000_proxy_keypoints',~good),
                tracked_ORB_depth_support_median=median('ORB_proxy_valid_depth_fraction',good),untracked_ORB_depth_support_median=median('ORB_proxy_valid_depth_fraction',~good),
                new_map_creation_logs=text.count('Creation of new map with id:')-1,
                active_reset_logs=text.count('LM: Active map reset recieved'),
                observed_fingerprint_transitions=measured['observed_frame_changes'],
                lifecycle_log_vs_fingerprint_agreement=(text.count('Creation of new map with id:')-1+text.count('LM: Active map reset recieved'))==measured['observed_frame_changes'],
                causal_attribution='UNRESOLVED; descriptive image/depth proxies and native logs, not a causal intervention'))
    writecsv(OUT/'loss_windows.csv',losses)
    dump(OUT/'failure_forensics.json',dict(robot_config_summaries=summaries,
        conclusion='NO_MISSING_RGBD_PAIRS_OR_OBSERVED_HEADER_MISMATCH_IN_THIS_COHORT; native visual tracking/map lifecycle remains limiting',
        not_ruled_out=['depth measurement/registration error','appearance/blur/dynamic objects','native feature association and mapping sensitivity'],
        GT=False,additional_tracking_runs=0))
    print(json.dumps(summaries),flush=True)

if __name__=='__main__':run()
