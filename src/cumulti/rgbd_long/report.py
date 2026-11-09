"""Measured results and project memory; distinguish completed audit from fusion readiness."""
import hashlib,json,subprocess
from pathlib import Path
from export import ROOT,OUT,dump

def run():
    decision=json.loads((OUT/'pre_fusion_decision.json').read_text());audit=json.loads((OUT/'validation.json').read_text())
    regression=json.loads((OUT/'reset_regression/validation.json').read_text());results=[];sources=[];server=[]
    for robot in ['robot1','robot3']:
        export=json.loads((OUT/robot/'export.json').read_text());frames=export['frames'];paired=[r for r in frames if r['depth_status']=='PAIRED']
        import numpy as np
        sources.append(dict(robot=robot,RGB_frames=len(frames),paired_depth=len(paired),missing_depth=len(frames)-len(paired),
            bag_sync_median_ms=float(np.median([r['bag_sync_ms'] for r in paired])),bag_sync_p95_ms=float(np.percentile([r['bag_sync_ms'] for r in paired],95)),
            header_sync_max_ms=max(r['header_sync_ms'] for r in paired),RGB_archive=export['RGB_archive'],depth_archive=export['depth_archive']))
        for config in ['baseline1000','fallback2000']:
            p=OUT/robot/config/'metrics.json'
            if p.exists():results.append(json.loads(p.read_text()))
        for p in (OUT/robot/'frames').glob('*'):
            server.append(dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    sdk=Path('/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3')
    code=list((ROOT/'src/cumulti/rgbd_long').glob('*.py'))+list((ROOT/'src/cumulti/rgbd_long').glob('*.cc'))+[
        ROOT/'src/cumulti/rgbd_demo/export.py',sdk/'src/Map.cc',sdk/'src/Tracking.cc',sdk/'src/System.cc',
        sdk/'lib/libORB_SLAM3.so',sdk/'Vocabulary/ORBvoc.txt',OUT/'bin/rgbd_long']
    dump(OUT/'provenance.json',dict(SDK_revision=subprocess.check_output(['git','-C',str(sdk),'rev-parse','HEAD'],text=True).strip(),
        SDK_modified_by_this_stage=False,SDK_preexisting_local_changes=bool(subprocess.check_output(['git','-C',str(sdk),'status','--porcelain'],text=True).strip()),
        Python_environment='/home/cas/.venvs/fyp_stage24',new_packages_installed=False,
        original_short_checkpoint='88dfda9c5544b1e2e7f5d1e6c02619094f6f4ebe',raw_archives=sources,
        files=[dict(path=str(p),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in code],
        GT=False,LiDAR=False,IMU=False,semantics=False,native_input_audit='strace file opens; not a kernel sandbox'))
    dump(OUT/'server_only_frames_manifest.json',dict(files=server,reason='1200 RGB / depth pairs per robot are not committed; raw bags, native binary, SDK/vocabulary and native traces also stay server-only'))
    text='# Native-rate RGB-D long-window tracking and reset-management audit\n\n'
    text+='Run started: 2026-11-08; finalization: 2026-11-09 (Asia/Shanghai). Accepted short-demo parent: `88dfda9c5544b1e2e7f5d1e6c02619094f6f4ebe`.\n\n'
    text+=f"Audit: **{audit['status']}**. Pre-fusion stability decision: **{decision['status']}**. Selected shared configuration: **{decision['selected_configuration']}**.\n\n"
    text+='## Fixed experiment and gate\n\n'
    text+='Both independent camera timelines use elapsed [120,240)s, every original RGB frame at nominal10Hz, with original IDs/header/bag timestamps. This extends the accepted 12s pilot; it is a120s qualification, not a complete1000s/2099s run. Native timestamps pace replay. Missing associated depth is zero/unknown; RGB frames are retained, without an identity pose or depth fabrication.\n\n'
    text+='Before native long tracking, policy.json fixed >=95%valid frames and time-weighted tracking, >=90s continuous tracking, no native coordinate-frame changes/resets, <=1s lost interval, finite/unit poses and no forbidden input reads. Final native corrected poses must cover >=95%input frames and are filtered to actual online state2 tracked frames. integrity_policy.json additionally rejects adjacent valid same-frame jumps >2m, >10m/s or >45degrees, using conservative engineering limits, not GT.\n\n'
    text+='Baseline1000 uses the original1000 ORB features; if either robot fails, fallback2000 is evaluated on both, changing only feature capacity. Fixed optics, depth registration/range, scale pyramid1.2/8, FAST20/7, virtual RGB-D baseline0.05m and close/far40 are retained. Lowest shared passing configuration wins; no further searches or gate relaxation. Correct nominal Camera.fps10 accompanies the denser input, so the old5Hz comparison is not a controlled proof that rate alone causes differences.\n\n'
    text+='## Measured native results\n\n| Robot | Config | Tracked/input | Valid % | Time-weighted % | Longest s | Longest loss s | Frame changes / native resets | Corrected tracked poses | Median/p95 tracking ms | Gate |\n|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|\n'
    for m in results:text+=f"| {m['robot']} | {m['config']} | {m['valid_frames']}/{m['frames']} | {100*m['valid_fraction']:.3f} | {100*m['time_weighted_fraction']:.3f} | {m['longest_continuous_s']:.3f} | {m['max_untracked_gap_s']:.3f} | {m['observed_frame_changes']} / {m['native_active_reset_log_count']} | {m['corrected_tracked_poses']} | {m['median_track_ms']:.3f}/{m['p95_track_ms']:.3f} | {m['fusion_gate']} |\n"
    text+='\nNative wall time includes vocabulary loading, real-time pacing, traced file I/O and shutdown; do not treat it as unpaced compute throughput. No real-time deployment claim is made. Exact runtimes, peak memory and all failed checks are recorded in metrics.json.\n\n'
    for m in results:
        failed=[k for k,v in m['checks'].items() if not v]
        text+=f"{m['robot']} {m['config']} failed checks: {', '.join(failed) if failed else 'none'}. First12s: {m['first12s_valid_frames']}/{m['first12s_input_frames']} valid, compared with the separately preserved5Hz short experiment.\n\n"
    text+='## Actual reset identification\n\n'
    text+=f"Regression: **{regression['status']}**. Same immutable60 Robot1 inputs, {regression['valid_frames']} valid poses, {regression['actual_native_reset_log_count']} native reset log event. Observed fingerprints: {regression['fingerprints']}. The old map ID was reused, while initial keyframe ID changed. Full events and time are in reset_regression/validation.json.\n\n"
    text+='SDK inspection confirms Map::AddKeyFrame assigns a new initial-KF ID when the emptied map is repopulated; ResetActiveMap clears the map without resetting the global next-keyframe counter. The observer votes tracked nonbad MapPoints to obtain map ID and initial-KF fingerprint. Gaps with an unchanged fingerprint are tracking gaps, not automatically coordinate resets. A changed fingerprint forbids odometry/path/map stitching. Synthetic gap/reused-ID tests and the actual native log independently cross-check this distinction. Native core source was not patched.\n\n'
    text+='Native final-BA export uses the explicit single-map SaveTrajectoryEuRoC overload only when exactly one frame was witnessed. Raw final exports can contain predicted recently-lost frames; corrected_camera_trajectory.csv deliberately retains only timestamps independently observed as online valid state2. Multiple/reset gauges are not exported as one canonical corrected trajectory. No missing poses are interpolated.\n\n'
    text+='## Sensor adaptation\n\n'
    for s in sources:text+=f"{s['robot']}: {s['RGB_frames']} RGB, {s['paired_depth']} paired depth, {s['missing_depth']} explicit missing depth; paired bag delta median/p95={s['bag_sync_median_ms']:.6f}/{s['bag_sync_p95_ms']:.6f}ms; maximum paired header delta={s['header_sync_max_ms']:.6f}ms.\n\n"
    text+='All large bag DB members were decoded after complete EOF/CRC validation, opened read-only and removed after this run’s exports. Actual embedded ROS2 definitions/field strides/endian, depth optical frame and column-major depth_to_color are used. Projection remains nearest color Z per pixel, no filling, 0.3–10m engineering range, uint16mm/DepthMapFactor1000 and actual scaled color K/D. No LiDAR optical mounting extrinsic is needed for independent RGB-D maps. This does not resolve the separate LiDAR RGB/semantic projection blocker. Header matching is not a proof of hardware exposure calibration. The overlapping old60 JPEGs are hash-compared in synchronization_report.csv; no GT/local-LiDAR trajectories define the new camera grid.\n\n'
    text+='## Fusion disposition and limitations\n\n'
    forensic_path=OUT/'failure_forensics.json'
    if forensic_path.exists():
        forensic=json.loads(forensic_path.read_text())
        text+='Input/loss-window forensics: **'+forensic['conclusion']+'**. All2400 input RGB frames in this cohort have paired depth and zero RGB/depth header delta; no missing-frame explanation is supported here. The old overlapping60 RGB AND depth images per robot are byte-identical to this new export. Camera cadence, depth coverage, Laplacian/brightness and independent OpenCV ORB-count proxies are recorded for every input frame, with every lost interval and contact sheet.\n\n'
        for f in forensic['robot_config_summaries']:
            text+=f"{f['robot']} {f['config']}: {f['loss_intervals']} lost interval(s), {f['new_map_creation_logs']} new-map creation(s), {f['active_reset_logs']} active reset(s); fingerprint/log agreement={f.get('lifecycle_log_vs_fingerprint_agreement','not checked')}. Maximum recorded RGB gap {f['RGB_gap_max_ms']:.3f}ms.\n\n"
        text+='These are descriptive diagnostics, not proof that blur, depth coverage or a particular frontend component caused the failures. Depth/registration errors, appearance and native association/mapping sensitivity are not ruled out. Increasing feature capacity did not cure this fixed cohort; do not infer universal superiority of1000features.\n\n'
        text+='Qualitative Robot3 baseline loss-window inspection shows strong magenta/yellow casts, bright ground/shadow boundaries and viewpoint change. The independent image-only ORB proxy still detects about its configured capacity; raw keypoint availability alone does not explain native tracking failure. Per-frame ORB keypoint depth-support fractions are included to distinguish image texture from valid metric feature support, without claiming a causal diagnosis.\n\n'
    if decision['status']=='NO_GO_RGBD_FUSION':
        text+='**Dual-robot fusion: NOT RUN.** Neither predeclared shared configuration passed both robots. No camera match, common-frame transform, cross-robot factor or fused global map is claimed. This is a tracking/continuity limitation, not a request to download more sensor data. The existing short demo remains functional and frozen.\n\n'
    else:text+='Both robots passed the frozen tracking gate. This authorizes a bounded camera-only visual/RGB-D geometry fusion study; it does not itself establish any inter-robot transform. Fusion artifacts, if generated, must separately document candidate selection, verification and graph/map results without GT.\n\n'
    text+='Visual tracking state/finite poses alone do not prove absolute accuracy or drift bounds. Local optical z is not terrain elevation; diagnostic plots do not connect lost/reset intervals. No GT ATE, semantic inference, LiDAR input, IMU/GNSS, full-run robustness, universal solver claims, or live dual-robot operation. Frozen historical KITTI, SC/GICP, P0/P5/P6 and Stage18–24 short results are untouched.\n\n'
    text+='## Reproduction\n\nFrom repository root on the server: `bash scripts/run_rgbd_long.sh`. The existing camera Python environment, separate ORB-SLAM3 SDK/vocabulary, original archives and server-only processed camera frames are prerequisites. The regression and overlap hash checks also reference the accepted short-pilot processed frames. Completed phases are reused; there are no hidden required scripts. The final verification is a cached server resume/analysis check, not a second fresh raw-to-native experiment. A Git-only clone can read the lightweight results but cannot execute image diagnostics without those separately supplied/processed datasets and frames. A fresh native multithreaded solve may not be bitwise identical; hashes identify the accepted source, settings, inputs and captured results. Full raw/expanded bags, native executables, vocabulary, traces and frame sequences are intentionally server-only.\n'
    (ROOT/'docs/CUMULTI_RGBD_LONG_TRACKING_RESET_AUDIT.md').write_text(text)
    summary=dict(status=audit['status'],fusion_readiness=decision['status'],selected_config=decision['selected_configuration'],
        robot_results=[{k:m[k] for k in ['robot','config','frames','valid_frames','valid_fraction','longest_continuous_s','observed_frame_changes','fusion_gate']} for m in results],
        reset_regression=regression['status'],GT=False,LiDAR=False)
    dump(OUT/'summary.json',summary)
    heading='## Native-rate RGB-D long-window stability (2026-11-08)'
    note='\n\n'+heading+'\n\n'+audit['status']+'. Frozen pre-fusion decision: '+decision['status']+'. Each robot uses120s and all original RGB frames. Real reset regression proves map ID alone is insufficient; use map ID plus initial-keyframe ID. No GT/LiDAR/IMU or semantic input. See docs/CUMULTI_RGBD_LONG_TRACKING_RESET_AUDIT.md and outputs/cumulti_v1/24_rgbd_long_v1/pre_fusion_decision.json.\n\n'
    note+='; '.join(f"{m['robot']} {m['config']}: {m['valid_frames']}/{m['frames']} tracked, {m['observed_frame_changes']} frame change(s), gate {m['fusion_gate']}" for m in results)+'.\n'
    if decision['status']=='NO_GO_RGBD_FUSION':note+='Dual-robot fusion not run because stability gate failed. Diagnose tracking/depth/motion before a separately documented revised experiment; do not lower the frozen gate or join reset maps. No further download is required for this cohort.\n'
    for name in ['AGENTS.md','PROJECT_CONTEXT.md','RESULTS.md','EXPERIMENT_LOG.md','NEXT_STEPS.md']:
        path=ROOT/name;previous=path.read_text()
        if heading not in previous:path.write_text(previous.rstrip()+note)
        else:
            previous=previous.replace('before authorizing a new revised configuration','before a separately documented revised experiment')
            path.write_text(previous)
    print(json.dumps(summary),flush=True)

if __name__=='__main__':run()
