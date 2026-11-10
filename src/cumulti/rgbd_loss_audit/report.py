"""Evidence-graded loss diagnosis from frozen logs and offline native features."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];OLD=ROOT/'outputs/cumulti_v1/24_rgbd_long_v1';OUT=ROOT/'outputs/cumulti_v1/24_rgbd_loss_rootcause_v1';SDK=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def rows(p):return list(csv.DictReader(p.open()))
def run():
 features=rows(OUT/'native_feature_support.csv');policy=json.loads((OUT/'diagnostic_policy.json').read_text());expected=2*sum(policy['selected_frames'].values());assert len(features)==expected
 assert (OUT/'native_access.log').read_text().rstrip().endswith('exited with 0 +++')
 dump(OUT/'native_complete.json',dict(rows=len(features),sha256=hashlib.sha256((OUT/'native_feature_support.csv').read_bytes()).hexdigest(),native_returncode=0,SLAM_rerun=False))
 by={(r['robot'],int(r['capacity']),int(r['index'])):r for r in features};episodes=[]
 for loss in rows(OLD/'loss_windows.csv'):
  robot,config=loss['robot'],loss['config'];capacity=1000 if config=='baseline1000' else 2000;a,b=int(loss['start_index']),int(loss['end_index'])
  native=rows(OLD/robot/config/'frame_poses.csv');data=json.loads((OLD/robot/'export.json').read_text());f=by[robot,capacity,a];previous=by[robot,capacity,a-1];times=np.array([int(r['relative_ns'])/1e9 for r in native]);end=min(b+1,len(native)-1)
  states=[]
  for r in native[a:end+1]:
   if not states or states[-1]!=int(r['tracking_state']):states.append(int(r['tracking_state']))
  output=dict(robot=robot,config=config,start_frame=a,end_frame=b,test_start_s=float(times[a]),source_camera_start_s=data['frames'][a]['elapsed_camera_s'],
   recovery_s=float(times[end]),untracked_frames=b-a+1,duration_s=float(times[end]-times[a]),observed_state_sequence=str(states),
   previous_saved_map_associations=int(native[a-1]['tracked_points']),first_bad_saved_map_associations=int(native[a]['tracked_points']),
   native_detected_features=int(f['native_features']),native_features_with_depth=int(f['valid_depth_features']),native_close_features=int(f['close_depth_features']),
   adjacent_mutual_matches=int(f['mutual_matches']),adjacent_fundamental_inliers=int(f['fundamental_inliers']),adjacent_both_depth_matches=int(f['pairs_with_depth']),
   median_pixel_motion=float(f['median_motion_px']),gray_mean_abs_difference=float(f['gray_mean_abs_difference']),
   previous_pair_median_motion=float(previous['median_motion_px']),first_bad_track_ms=float(native[a]['track_ms']),
   original_RGB_id=data['frames'][a]['original_RGB_index'],
   interpretation='RECENTLY_LOST_TIMEOUT_AND_REINITIALIZATION' if b-a+1>=20 else 'EARLY_SMALL_MAP_RESET_AFTER_NEW_MAP')
  episodes.append(output)
 with (OUT/'loss_episode_evidence.csv').open('w',newline='') as file:
  w=csv.DictWriter(file,fieldnames=list(episodes[0]));w.writeheader();w.writerows(episodes)
 comparisons=[]
 for e in episodes:
  if e['config']!='fallback2000':continue
  robot=e['robot'];index=e['start_frame'];base=rows(OLD/robot/'baseline1000/frame_poses.csv')[index]
  comparisons.append(dict(robot=robot,index=index,baseline_same_RGB_frame_tracked=base['valid_pose']=='1',
   baseline_native_features=int(by[robot,1000,index]['native_features']),fallback_native_features=int(by[robot,2000,index]['native_features']),
   baseline_depth_features=int(by[robot,1000,index]['valid_depth_features']),fallback_depth_features=int(by[robot,2000,index]['valid_depth_features'])))
 dump(OUT/'same_image_capacity_comparison.json',comparisons)
 evidence=dict(status='PASS_DIAGNOSTIC_AUDIT',SLAM_rerun=False,new_poses=False,GT=False,LiDAR=False,
  confirmed=['All input frames were processed; these are tracking-state losses, not missing dataset frames',
   'RGB/depth pairs are present and header-aligned in this bounded cohort',
   'Five long loss intervals align with native non-inertial RGB-D3s relocalization timeout followed by map initialization',
   'Two short loss intervals follow new-map creation and active reset',
   'All7 first-bad frames retain numerous native image features and independent adjacent 2D geometric matches'],
  supported_but_not_unique='Native existing-map tracking/association fails despite adjacent-frame image compatibility; R3 baseline failure accompanies larger image motion and brightness/view change',
  not_proven=['Whether motion-model/reference-KF initialization or TrackLocalMap optimization fails first',
   'Exact native mnMatchesInliers at the first failure (saved tracked_points is NOT that counter)',
   'A unique causal allocation to blur, depth noise/edges, camera calibration, local-map drift or asynchronous mapping'],
  next_minimum_observation='Separate logging-only diagnostic replay with motion/reference tracking booleans, pre/post pose-optimization inliers, map-point visibility/search counts and RGB-D residuals; preserve all accepted runs and gates',
  detection_counts_not_pose_guarantees=True,adjacent_F_inliers_not_native_map_inliers=True,
  episodes=episodes,native_feature_rows=len(features))
 dump(OUT/'diagnosis.json',evidence)
 # Verify every canonical artifact of the accepted long run remains byte-identical.
 old_manifest=json.loads((OLD/'artifact_manifest.json').read_text())
 for a in old_manifest['files']:assert hashlib.sha256((ROOT/a['path']).read_bytes()).hexdigest()==a['sha256']
 source={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [SDK/'src/Tracking.cc',SDK/'src/Frame.cc',SDK/'src/ORBextractor.cc',SDK/'include/Tracking.h',SDK/'lib/libORB_SLAM3.so']}
 dump(OUT/'provenance.json',dict(source_hashes=source,original_result_checkpoint='6f57e358188d6a536026aa0944d7b6d308743b2b',original_artifacts_unchanged=len(old_manifest['files']),
  SDK_modified_by_this_audit=False,SDK_preexisting_changes_preserved=True,native_feature_process='ORBextractor + depth lookup + adjacent image matching only; no System/TrackRGBD call',
  source_locations=dict(relocalization_timeout='Tracking.cc:2006 (RGB-D >3.0s; the5s field belongs to inertial branch)',tracking_message='Tracking.cc:2131 (generic bOK failure, not proof TrackLocalMap was called)',
    local_inlier_tests='Tracking.cc:3029-3065 (50 near recent relocalization; nominal RGB-D30)',depth_lookup='Frame.cc:984-1006 (float keypoint to integer pixel truncation)'),GT=False))
 text='# RGB-D tracking-loss cause audit\n\nDate: 2026-11-10 (Asia/Shanghai). Frozen result parent: `6f57e35`. No SLAM, retrieval, graph optimization or GT evaluation rerun.\n\n'
 text+='**These are lost tracking states, not missing image files.** All2400 RGB/depth inputs in the previous study exist and were consumed; paired exposure headers match. The accepted NO_GO decision is unchanged.\n\n'
 text+='## What is established\n\nThe native source separates image detection, existing-map tracking, relocalization and map lifecycle. If visual tracking fails, the non-inertial RGB-D branch enters RECENTLY_LOST and attempts relocalization. After more than3.0s without recovery, it becomes LOST and starts a new map or resets a small active map. The observed roughly3.1s intervals are this recovery policy plus the next input/reinitialization frame, not three seconds of missing sensor data. The constructor’s5s recently-lost parameter is used by a different inertial branch; it is not the active RGB-D timeout. Two one-frame gaps happen after creation of tiny new maps and coincide with active-reset logs.\n\n'
 text+='Native feature extraction has now been checked directly using the actual ORB-SLAM3 ORBextractor, saved1000/2000 YAML and original640x400 JPEG/depth images. This does NOT instantiate the SLAM system. The selected cohort includes every loss interval±20 frames and one-per-second controls: '+str(len(features))+' offline feature records across both capacities. Selection is failure-conditioned diagnostics, not a fresh unbiased benchmark.\n\n'
 text+='| Robot/config | First bad frame | Test time s | Gap s | Saved map associations before→bad | Native features / with depth | Adjacent 2D F inliers | Motion px |\n|---|---:|---:|---:|---:|---:|---:|---:|\n'
 for e in episodes:text+=f"| {e['robot']} {e['config']} | {e['start_frame']} | {e['test_start_s']:.3f} | {e['duration_s']:.3f} | {e['previous_saved_map_associations']}→{e['first_bad_saved_map_associations']} | {e['native_detected_features']} / {e['native_features_with_depth']} | {e['adjacent_fundamental_inliers']} | {e['median_pixel_motion']:.2f} |\n"
 text+='\nFrame IDs here are RGB-D runner input indices, not LiDAR keyframe IDs. Add roughly120s to these test times for the original camera timeline; exact source camera times and original RGB IDs are in loss_episode_evidence.csv.\n\n'
 text+='## Robot3 default configuration: strongest localized evidence\n\nAt frame255 (test25.588s, original RGB1455), saved map associations fall117→22. But the same image still has1006 native keypoints,269 measured-depth keypoints,292 mutual adjacent matches and237 fundamental-matrix inliers. Only42 adjacent matches have measured depth in both frames. From frames250–255, median matched image displacement rises from about12.1 to14.9px per frame, and mean absolute image difference rises from21.4 to26.6 gray levels. This is compatible with viewpoint/motion and photometric change stressing existing-map tracking, not an absence of image texture. It is not a unique causal proof.\n\n'
 text+='The existing map is not recovered: the observer sees RECENTLY_LOST, then NO_IMAGES_YET as the native internal LOST/new-map path executes, then OK in map1 at28.697s. A transient internal LOST state need not appear as state4 in the post-call CSV. Do not join the new identity-frame poses onto the old trajectory.\n\n'
 text+='## Why increasing feature count did not cure it\n\nAll six fallback first-bad frames retain hundreds/thousands of native keypoints and substantial adjacent-image geometric matches. Robot3 fallback fails as early as0.899s despite2005 detected features and924 with depth, so fast motion or absent depth cannot explain every episode. Robot1 baseline tracks the SAME input images through frames where fallback fails. Feature capacity changes which spatial features and map points are admitted, reference/map history, keyframe decisions and workload; more detections are not a monotonic guarantee of more reliable established-map inliers. Causal allocation among those mechanisms remains unmeasured.\n\n'
 text+='## Limits and next diagnostic\n\nThe saved tracked_points count is nonbad associated MapPoints, not native mnMatchesInliers. The log “Fail to track local map!” is printed when bOK is false, even when an earlier motion/reference tracking step already failed. Native RGB-D normally requires30 local-map inliers and50 near recent relocalization; there is also a >10-inlier early-success branch in RECENTLY_LOST. These state-dependent rules cannot be reconstructed from saved association counts alone. Independent adjacent-frame fundamental inliers are also not native map/PnP inliers or GT-correct matches.\n\n'
 text+='Previous OpenCV-ORB depth-support proxies are not native extraction measurements. They also used rounding; native Frame::ComputeStereoFromRGBD truncates the original pixel coordinates. This audit uses the exact native detector and lookup rule. A3x3 depth consistency count is a diagnostic, not an applied filter or calibrated confidence.\n\n'
 text+='A conclusive next intervention is a separate logging-only replay of the same failing interval with its initialization history, recording TrackWithMotionModel/TrackReferenceKeyFrame/TrackLocalMap return values, pre/post optimization inliers, reprojection/depth residuals and map/keyframe statistics. No such replay was run here. Merely increasing the3s timeout changes recovery waiting time; it does not repair the first tracking failure. No threshold or historical metric has been altered, and no fusion was started.\n'
 (ROOT/'docs/CUMULTI_RGBD_LOSS_ROOTCAUSE_AUDIT.md').write_text(text)
 heading='## RGB-D loss-cause audit (2026-11-10)'
 note='\n\n'+heading+'\n\nOffline native feature/source audit confirms tracking loss, not missing data: roughly3s relocalization timeout then map restart/reset. All7 first-bad images retain many native keypoints and adjacent 2D geometric matches; native map/PnP inlier diagnostics are still missing, so the unique initial failure cause is unresolved. Robot3 baseline atRGB-D frame255: saved map associations117→22,1006 native features,269 with depth,237 independent adjacent F inliers. No SLAM/GT/fusion rerun or frozen result change. See docs/CUMULTI_RGBD_LOSS_ROOTCAUSE_AUDIT.md.\n'
 for name in ['PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md']:
  p=ROOT/name;old=p.read_text()
  if heading not in old:p.write_text(old.rstrip()+note)
 artifacts=[]
 for p in OUT.rglob('*'):
  if not p.is_file() or 'bin' in p.parts or p.suffix not in ['.json','.csv','.jpg']:continue
  if p.name=='artifact_manifest.json':continue
  artifacts.append(dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
 dump(OUT/'artifact_manifest.json',dict(files=artifacts,SLAM_rerun=False))
 print(json.dumps(dict(status='PASS_DIAGNOSTIC_AUDIT',episodes=len(episodes),native_feature_rows=len(features),old_artifacts_unchanged=len(old_manifest['files']),SLAM_rerun=False)))
if __name__=='__main__':run()
