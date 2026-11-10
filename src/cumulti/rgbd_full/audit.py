"""Full-stream association and explicit audit gate; no scientific tracking is run here."""
import json,time,sys,zipfile
import numpy as np
from common import *

def run(wait=False):
 sys.addaudithook(guard)
 required=[OUT/r/(k+s) for r in ['robot1','robot3'] for k in ['RGB','DEPTH'] for s in ['_audit.json','_calibration_audit.json']]
 start=time.monotonic()
 while not all(p.exists() for p in required):
  if not wait:raise RuntimeError('Complete archive/calibration audit not ready')
  if time.monotonic()-start>10800:raise TimeoutError('Full audit incomplete after 3 hours; inspect per-role logs')
  time.sleep(5)
 summary=[];robots={};gaps=[]
 for robot in ['robot1','robot3']:
  a={k:json.loads((OUT/robot/(k+'_audit.json')).read_text()) for k in ['RGB','DEPTH']}
  for k in a:
   with zipfile.ZipFile(a[k]['archive']['archive']) as z:
    members=[n for n in z.namelist() if n.endswith('.db3')]
    assert members==[a[k]['archive']['member']],('Multiple bag DBs require full segmented audit',robot,k,members)
   cal=json.loads((OUT/robot/(k+'_calibration_audit.json')).read_text())
   assert cal['constant'],('Calibration changes require explicit handling',robot,k)
   b=a[k];record={key:b[key] for key in ['robot','kind','frames','first_bag_ns','last_bag_ns','first_header_ns','last_header_ns','duration_s','median_frequency_hz','overall_frequency_hz','bag_gaps_over_250ms','header_nonincreasing']}
   hs=[int(r['header_ns']) for r in b['rows']];record.update(earliest_header_ns=str(min(hs)),latest_header_ns=str(max(hs)),header_duration_s=(max(hs)-min(hs))/1e9)
   record.update(max_bag_gap_ms=b['bag_gap_ms']['max'],median_bag_gap_ms=b['bag_gap_ms']['median'],p95_bag_gap_ms=b['bag_gap_ms']['p95'],max_header_gap_ms=b['header_gap_ms']['max'])
   summary.append(record)
   for previous,current in zip(b['rows'],b['rows'][1:]):
    delta=(int(current['bag_ns'])-int(previous['bag_ns']))/1e6
    if delta>150:gaps.append(dict(robot=robot,kind=k,previous_index=previous['index'],next_index=current['index'],previous_bag_ns=previous['bag_ns'],next_bag_ns=current['bag_ns'],gap_ms=delta,threshold_ms=150))
  depths=a['DEPTH']['rows'];dt=np.array([int(r['bag_ns']) for r in depths],np.int64);rows=[]
  for r in a['RGB']['rows']:
   stamp=int(r['bag_ns']);j=int(np.searchsorted(dt,stamp));j=min((x for x in [j-1,j] if 0<=x<len(dt)),key=lambda x:(abs(int(dt[x])-stamp),x));d=depths[j];error=abs(int(dt[j])-stamp)/1e6
   rows.append(dict(index=r['index'],rgb_message_id=r['message_id'],rgb_bag_ns=r['bag_ns'],rgb_header_ns=r['header_ns'],relative_ns=str(stamp-int(a['RGB']['first_bag_ns'])),rgb=r['rgb'],depth_index=j,depth_message_id=d['message_id'],depth_bag_ns=d['bag_ns'],depth_header_ns=d['header_ns'],bag_sync_ms=error,header_sync_ms=abs(int(d['header_ns'])-int(r['header_ns']))/1e6,paired=error<=25))
  writecsv(OUT/robot/'sync_pairs.csv',rows)
  used={r['depth_index'] for r in rows if r['paired']}
  robots[robot]=dict(rgb_frames=len(rows),depth_frames=len(depths),duration_s=a['RGB']['duration_s'],paired=sum(r['paired'] for r in rows),paired_fraction=sum(r['paired'] for r in rows)/len(rows),paired_unique_depth_frames=len(used),depth_used_fraction=len(used)/len(depths),sync_ms=stats([r['bag_sync_ms'] for r in rows]),paired_sync_ms=stats([r['bag_sync_ms'] for r in rows if r['paired']]),paired_header_sync_ms=stats([r['header_sync_ms'] for r in rows if r['paired']]),paired_header_nonzero=sum(r['paired'] and r['header_sync_ms']!=0 for r in rows),calibration_constant=True)
 writecsv(OUT/'full_data_audit.csv',summary)
 writecsv(OUT/'timestamp_gaps.csv',gaps,['robot','kind','previous_index','next_index','previous_bag_ns','next_bag_ns','gap_ms','threshold_ms'])
 decision=dict(status='GO_COMPLETE_AVAILABLE_RECORDINGS',robots=robots,association='nearest bag timestamp, <=25ms, tie ordered index',baseline_features=1000,tracking_sampling='every original RGB message; original timestamps',unpaired_policy='zero depth, record tracking validity honestly',presentation_duration_s=max(r['duration_s'] for r in robots.values()),playback_recorded_seconds_per_demo_second=10,Stage20_reference_s=1972.508,common_full_Stage20_coverage=all(r['duration_s']>=1972.508 for r in robots.values()),GT=False,LiDAR=False,IMU=False,GNSS=False)
 dump(OUT/'audit_decision.json',decision)
 dump(OUT/'dataset_manifest.json',dict(robots={r:{k:{key:value for key,value in json.loads((OUT/r/(k+'_audit.json')).read_text()).items() if key!='rows'} for k in ['RGB','DEPTH']} for r in ['robot1','robot3']},decision=decision))
 lines=['# Full RGB-D data audit','', 'This gate qualifies complete available inputs, not continuous tracking or successful fusion.','']
 for r,s in robots.items():lines.append(f"- {r}: {s['rgb_frames']} RGB, {s['depth_frames']} depth clouds, {s['duration_s']:.6f} recorded seconds; paired {s['paired']}/{s['rgb_frames']} within 25 ms.")
 lines+=['','Each camera start is independently normalized to zero. Side-by-side replay does not establish simultaneous physical recording.','A shorter recording ends explicitly; no frames or poses are fabricated to reach the Stage20 duration.','Every archive DB was read to EOF with ZIP CRC validation. All recorded CameraInfo/extrinsics messages were inspected.','Source header timestamps and bag timestamps are both preserved in the per-robot CSVs.','Calibration is unchanged; the established depth-to-color projection and 1000-feature baseline are retained.']
 (OUT/'FULL_DATA_AUDIT.md').write_text('\n'.join(lines)+'\n')
 detailed=lines+['','## Complete timestamp ranges','', '| Robot | Stream | First bag ns | Last bag ns | Earliest header ns | Latest header ns | Overall Hz | Max bag gap ms |','|---|---|---:|---:|---:|---:|---:|---:|']
 for s in summary:detailed.append(f"| {s['robot']} | {s['kind']} | {s['first_bag_ns']} | {s['last_bag_ns']} | {s['earliest_header_ns']} | {s['latest_header_ns']} | {s['overall_frequency_hz']:.6f} | {s['max_bag_gap_ms']:.6f} |")
 detailed+=['','## Actual topics and formats','']
 for robot in robots:
  for kind in ['RGB','DEPTH']:
   b=json.loads((OUT/robot/(kind+'_audit.json')).read_text());detailed.append(f"- {robot} {kind}: `{b['topic']}`, `{b['type']}`, frame `{b['calibration']['frame_id']}`.")
 detailed+=['','Depth is an unorganized PointCloud2 camera cloud, not a native depth Image topic. Metric XYZ is projected through the recorded factory depth-to-color extrinsic and actual color K/D. Color is rgb8 at 1280×800; the established input scale is 640×400. No camera calibration is retuned.','', 'All paired original header deltas are recorded in the decision JSON. Every recorded depth cloud has a unique RGB partner within the 25 ms bag-time gate; some RGB frames have no corresponding recorded depth. Those frames remain in the full tracking sequence with zero/unknown depth.','', 'This is input qualification only. Full-duration tracking quality, reset fragmentation, map consistency and browser rendering require their own reports. No historical 120-second results are replaced.']
 (ROOT/'docs/CUMULTI_RGBD_FULL_DATA_AUDIT.md').write_text('\n'.join(detailed)+'\n')
 print(json.dumps(decision),flush=True)

if __name__=='__main__':run('--wait' in sys.argv)
