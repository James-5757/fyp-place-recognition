"""Independent long-window diagnostics; no GT/SLAM rerun or pose repair."""
import csv,hashlib,json
from pathlib import Path
import cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from export import ROOT,OUT,dump
from track import writecsv

def run():
    decision=json.loads((OUT/'pre_fusion_decision.json').read_text());stats=[];sync=[]
    for robot in ['robot1','robot3']:
        source=json.loads((OUT/robot/'export.json').read_text());frames=source['frames'];paired=[f for f in frames if f['depth_status']=='PAIRED']
        # The overlapping old 5Hz frames must be the same real input images.
        old=json.loads((ROOT/f'outputs/cumulti_v1/24_rgbd_demo_v1/{robot}/export.json').read_text());by_ns={f['rgb_bag_ns']:f for f in frames}
        same=0;different=0;same_depth=0
        for f in old['frames']:
            n=by_ns[f['rgb_bag_ns']]
            if hashlib.sha256((ROOT/f['rgb']).read_bytes()).digest()==hashlib.sha256((ROOT/n['rgb']).read_bytes()).digest():same+=1
            else:different+=1
            if hashlib.sha256((ROOT/f['depth']).read_bytes()).digest()==hashlib.sha256((ROOT/n['depth']).read_bytes()).digest():same_depth+=1
        sync.append(dict(robot=robot,RGB_frames=len(frames),paired_depth_frames=len(paired),missing_depth_frames=len(frames)-len(paired),
            paired_bag_sync_median_ms=float(np.median([f['bag_sync_ms'] for f in paired])),paired_bag_sync_p95_ms=float(np.percentile([f['bag_sync_ms'] for f in paired],95)),
            paired_header_sync_max_ms=max(f['header_sync_ms'] for f in paired),median_valid_depth_fraction=float(np.median([f['depth_coverage'] for f in frames])),
            frozen_RGB_images_byte_identical=same,frozen_RGB_images_JPEG_different=different,
            frozen_depth_images_byte_identical=same_depth,
            hardware_clock_calibration='NOT_VERIFIED',GT_read=False))
        configs=[c for c in ['baseline1000','fallback2000'] if (OUT/robot/c/'metrics.json').exists()]
        fig,axes=plt.subplots(3,len(configs),figsize=(7*len(configs),9),squeeze=False,sharex='col')
        for col,config in enumerate(configs):
            folder=OUT/robot/config;m=json.loads((folder/'metrics.json').read_text());rows=list(csv.DictReader((folder/'frame_poses.csv').open()));t=np.array([int(r['relative_ns'])/1e9 for r in rows]);valid=np.array([r['valid_pose']=='1' for r in rows])
            stats.append({k:m[k] for k in ['robot','config','frames','duration_s','valid_frames','valid_fraction','time_weighted_fraction','longest_continuous_s','max_untracked_gap_s','observed_frame_changes','native_active_reset_log_count','corrected_tracked_poses','median_track_ms','p95_track_ms','native_wall_s','first12s_valid_frames','first12s_input_frames','fusion_gate']})
            axes[0,col].plot(t,[int(r['tracking_state']) for r in rows],lw=.7,color='#4b88b9');axes[0,col].set_ylabel('Native tracking state');axes[0,col].set_title(f"{robot} {config}: {m['fusion_gate']} ({m['valid_fraction']:.1%} valid)")
            spans=list(csv.DictReader((folder/'tracking_segments.csv').open()))
            for s in spans:
                ids=np.arange(int(s['start']),int(s['end'])+1);axes[1,col].plot(t[ids],[float(rows[i]['tz']) for i in ids],lw=.8)
            axes[1,col].set_ylabel('Local camera gauge z (m)\nNo line across lost/reset frames')
            axes[2,col].plot(t,[f['depth_coverage'] for f in frames],lw=.7,color='#4b9a64');axes[2,col].set_ylabel('Valid depth fraction');axes[2,col].set_ylim(-.03,1.03);axes[2,col].set_xlabel('Camera time since cohort start (s)')
            for ax in axes[:,col]:
                ax.grid(alpha=.18)
            for i in np.flatnonzero(~valid):
                for ax in axes[:,col]:ax.axvspan(t[i],t[min(i+1,len(t)-1)],color='red',alpha=.15,lw=0)
            events=list(csv.DictReader((folder/'reset_events.csv').open()))
            if events:
                selected=events[:3];sheet=np.zeros((len(selected)*240,3*320,3),np.uint8)
                for row,event in enumerate(selected):
                    center=int(event['index'])
                    for column,i in enumerate([max(0,center-1),center,min(len(frames)-1,center+1)]):
                        image=cv2.resize(cv2.imread(str(ROOT/frames[i]['rgb'])),(320,200));sheet[row*240:row*240+200,column*320:(column+1)*320]=image
                        cv2.putText(sheet,f"f{i} t={t[i]:.2f}s initKF={rows[i]['map_init_kf']}",(column*320+4,row*240+222),cv2.FONT_HERSHEY_SIMPLEX,.4,(230,230,230),1)
                cv2.imwrite(str(folder/'reset_contact_sheet.jpg'),sheet)
        fig.suptitle('RGB-D tracking diagnostics — no GT / no interpolated poses\nRed = untracked; local z is optical gauge, not terrain elevation',fontsize=11);fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(OUT/f'{robot}_stability.png',dpi=140);plt.close(fig)
    writecsv(OUT/'stability_metrics.csv',stats);writecsv(OUT/'synchronization_report.csv',sync)
    checks=[]
    for robot in ['robot1','robot3']:
        e=json.loads((OUT/robot/'export.json').read_text());times=[int(f['rgb_bag_ns']) for f in e['frames']]
        assert all(b>a for a,b in zip(times,times[1:]))
        assert e['RGB_archive']['CRC_validated_at_EOF'] and e['depth_archive']['CRC_validated_at_EOF']
        assert not list((OUT/robot/'work').glob('*.db3'))
        for config in ['baseline1000','fallback2000']:
            p=OUT/robot/config/'metrics.json'
            if not p.exists():continue
            m=json.loads(p.read_text());assert m['checks']['finite_unit_poses'] and not m['native_forbidden_reads'];checks.append(dict(robot=robot,config=config,gate=m['fusion_gate']))
    reset=json.loads((OUT/'reset_regression/validation.json').read_text());assert reset['status']=='PASS_REAL_MAP_ID_REUSE_RESET_DETECTED'
    dump(OUT/'validation.json',dict(status='PASS_AUDIT',stability_decision=decision['status'],actual_tracking=checks,
        real_reset_regression=reset['status'],old_output_overwrite=False,GT=False,LiDAR=False,
        fusion='NOT_RUN_STABILITY_GATE_FAILED' if decision['status']=='NO_GO_RGBD_FUSION' else 'AUTHORIZED_BY_FROZEN_STABILITY_GATE',
        scope='120-second experiment and reset management only; audit PASS does not imply tracking gate PASS'))
    print(json.dumps(dict(validation='PASS_AUDIT',decision=decision['status'],sync=sync)),flush=True)

if __name__=='__main__':run()
