"""Generate result documentation from this isolated run's measured artifacts."""
import hashlib,json,subprocess
from pathlib import Path
from export import ROOT,OUT,dump

def run():
    mapdata=json.loads((OUT/'map_manifest.json').read_text());validation=json.loads((OUT/'validation.json').read_text())
    browserpath=OUT/'screenshots/browser_validation.json'
    browser=json.loads(browserpath.read_text()) if browserpath.exists() else None
    sdk=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')
    files=[sdk/'lib/libORB_SLAM3.so',sdk/'Vocabulary/ORBvoc.txt',sdk/'include/System.h',sdk/'src/System.cc',
        OUT/'bin/rgbd_pilot',ROOT/'src/cumulti/rgbd_demo/rgbd_pilot.cc']
    for robot in ['robot1','robot3']:
        files.extend([OUT/robot/'paced/frame_poses.csv',OUT/robot/'paced/camera.yaml',OUT/robot/'export.json'])
    provenance=dict(ORB_SDK_revision=subprocess.check_output(['git','-C',str(sdk),'rev-parse','HEAD'],text=True).strip(),
        ORB_SDK_modified_by_this_work=False,ORB_license='GPLv3; existing separately installed SDK; binary/vocabulary not committed',
        files=[dict(path=str(p),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files],
        Python=subprocess.check_output(['/home/cas/.venvs/fyp_stage24/bin/python','--version'],text=True).strip(),
        native_OpenCV=subprocess.check_output(['pkg-config','--modversion','opencv4'],text=True).strip(),
        native_mode='RGBD, viewer=false; no IMU',fyp_gtsam_environment_modified=False,fyp_slam_environment_modified=False)
    dump(OUT/'native_provenance.json',provenance)
    server_files=[]
    for robot in ['robot1','robot3']:
        server_files.extend((OUT/robot/'frames').glob('*'))
        server_files.extend((OUT/robot/'paced').glob('*.log'))
        server_files.extend((OUT/robot/'paced').glob('*.tum'))
    dump(OUT/'server_only_artifact_manifest.json',dict(reason='native depth sequence / traces remain server-only; display JPEG assets are separately bounded and versioned',
        files=[dict(path=str(p),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in server_files if p.is_file()]))
    text='# Camera-only RGB-D mapping demo — bounded prototype\n\n'
    text+='Date: 2026-11-08. User authorized a separate RGB-D map demo; semantic mapping is deferred.\n\n'
    text+='Functional/artifact validation: **'+validation['status']+'**. Full-window tracking: **'+validation['full_window_tracking']+'**.\n\n'
    text+='No LiDAR-to-camera extrinsic is needed for this route: depth-optical XYZ is transformed with the actual depth_to_color calibration, then paired with original color images and new RGB-D camera poses. This does not unblock coloring the old LiDAR P5 map.\n\n'
    text+='| Robot | Tracked / input | Continuous max | Tracking epochs | RGB map points across separate epochs | Median / p95 bag sync ms | Native wall s | Track median / p95 ms |\n|---|---:|---:|---:|---:|---:|---:|---:|\n'
    for robot,s in mapdata['robots'].items():
        t=s['tracking'];sync=s['bag_sync_ms']
        text+=f"| {robot} | {t['valid_poses']} / {t['frames']} | {t['longest_continuous_tracked_frames']} | {len(t['tracking_epochs'])} | {s['points']} | {sync['median']:.6f} / {sync['p95']:.6f} | {t['wall_s']:.6f} | {t['median_track_ms']:.6f} / {t['p95_track_ms']:.6f} |\n"
    text+='\n## Tracking integrity\n\n'
    legacy=OUT/'robot1/tracking.json'
    if legacy.exists():
        previous=json.loads(legacy.read_text())
        text+=f"The first unpaced Robot1 engineering run tracked {previous['valid_poses']}/{previous['frames']} frames and reset its active map. Its report remains at robot1/tracking.json. Pacing did not eliminate the Robot1 loss; its paced report remains PARTIAL, with no threshold relaxation or GT-based tuning. Active-map ID 0 was reused across reset. "
    text+='The paced implementation preserves sensor frame intervals and explicitly records tracking epochs. Map ID alone is not evidence of a shared frame. Every post-gap epoch is conservatively isolated; the browser never connects or overlays reset maps. A functional demo is not a full-window SLAM success.\n\n'
    text+='The fixed acceptance gate is >=45/60 valid poses, >=40 continuous frames, one map and tracking epoch, finite/unit poses. Parameter settings are recorded in each paced/tracking.json. There were no additional window searches or optimizer/feature retuning to force this gate.\n\n'
    text+='## Adapter and geometry\n\n'
    text+='The actual ZIP DBs are CRC-checked at EOF, opened read-only, and decoded using embedded ROS2 definitions. The input is unorganized camera PointCloud2, not native depth Image. Actual field offsets, strides, endian, depth optical frame, camera K/D and column-major depth_to_color are used. Real RGB is the accepted fixed camera-pilot JPEG export (640x400, 5Hz, camera elapsed 120–132s); no new camera/LiDAR sync grid is introduced.\n\n'
    for robot in ['robot1','robot3']:
        e=json.loads((OUT/robot/'export.json').read_text());rows=e['frames']
        import numpy as np
        text+=f"{robot}: depth valid coverage median {np.median([f['valid_depth_fraction'] for f in rows])*100:.3f}%; maximum RGB/depth header delta {max(f['header_sync_ms'] for f in rows):.6f}ms. This does not prove a LiDAR hardware clock calibration.\n\n"
    text+='Depth-optical and color Z are gated at 0.3–10m, saturation/NaN/rear/FoV failures discarded. Closest color Z wins per pixel; holes remain zero. PNG is uint16 mm with native DepthMapFactor=1000. RGB-D pseudo-stereo baseline 0.05m is an explicit virtual algorithm parameter, not an inferred IR baseline; close/far multiplier 40. Depth images are backprojected through undistorted color rays and the corresponding recorded TrackRGBD Twc. Dense-map policy: every second valid frame (plus first of a submap), pixel stride 4x4, 5cm voxels, retain first observation, cap100000 vertices per submap.\n\n'
    text+='Synthetic direction/occlusion/saturation/NaN tests and actual image/depth/buffer/pose/timestamp/hash checks are independent of GT. Projection/backprojection roundtrip tests numerical self-consistency, not physical calibration accuracy. The alignment overlays are qualitative, not ground-truth accuracy measurements. Original archives are unchanged; temporary DB copies are removed after successful export.\n\n'
    text+='## Demo, limitations and reproduction\n\n'
    text+='Open `http://127.0.0.1:8021/rgbd_camera_mapping/` after serving the whole demo parent. Native pipeline: `bash scripts/run_rgbd_demo.sh`. Camera exports and native SDK are server prerequisites. Repeated default invocation reuses completed phases. Browser validation command and controls are in demo/rgbd_camera_mapping/README.md.\n\n'
    if browser:
        text+=f"Browser checks: {browser['status']}; startup {browser['startup_ms']:.3f}ms, measured rendering {browser['measured_software_render_fps']:.3f}fps on headless software WebGL. Play/pause/reset/seek/final, actual left orbit/right pan/wheel zoom, image decode, reset-epoch separation and cached buffers are checked. Software rendering remains a performance limitation, not a hardware frame-rate or real-time SLAM claim.\n\n"
    text+='Both panels remain independent local optical gauges. Replay is offline; separate runs are not simultaneous physical robots. RGB is measured image color; preview pseudocolor represents measured depth, not semantic inference. No camera-only inter-robot match/factor/common frame is established. Do not label this a fused global camera map. Earlier dense samples are not reprojected after local BA; ghosting, depth holes and tracking loss remain. The 100000-point presentation budget is reached by Robot3: the UI marks DISPLAY CAP, later camera poses/images still advance but additional map vertices are omitted. No full-run robustness or GT accuracy evaluation, semantic inference, visual cross-robot fusion, or live deployment was performed.\n\n'
    text+='Frozen LiDAR/SC/GICP/P0/P5/P6/Stage18–23E results and Stage24A/B accepted reports are untouched. The previous RGB/semantic LiDAR-projection blocker remains valid. Next step: diagnose RGB-D tracking loss and validate longer camera-only continuity before proposing camera-based multi-robot map fusion.\n'
    doc=ROOT/'docs/CUMULTI_RGBD_CAMERA_MAPPING_DEMO.md';doc.write_text(text)
    heading='## Bounded camera RGB-D local-map demo (2026-11-08)'
    note='\n\n'+heading+'\n\n'+validation['status']+'. '+validation['full_window_tracking']+'. New independent camera-only maps, original depth PointCloud2 adaptation, timestamp-aligned RGB and native paced ORB-SLAM3 RGBD. No LiDAR/GT/IMU input or semantic map; reset epochs never merged. No cross-robot visual fusion or full-sequence run. See docs/CUMULTI_RGBD_CAMERA_MAPPING_DEMO.md and demo/rgbd_camera_mapping/README.md. Historical Stage24A/B blocked reports are preserved; Robot3 depth arrival resolves availability, not missing LiDAR-optical calibration.\n'
    for name in ['PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md','AGENTS.md']:
        path=ROOT/name;old=path.read_text()
        if heading not in old:path.write_text(old.rstrip()+note)
        counts='Verified short-window tracking: '+ '; '.join(f"{r} {s['tracking']['valid_poses']}/{s['tracking']['frames']} valid frames, {len(s['tracking']['tracking_epochs'])} separate tracking epoch(s), {s['points']} display points across its epochs" for r,s in mapdata['robots'].items())+'. No joint camera map has been established.'
        old=path.read_text()
        if counts not in old:path.write_text(old.rstrip()+'\n\n'+counts+'\n')
    print(str(doc))

if __name__=='__main__':run()
