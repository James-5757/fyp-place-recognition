"""Machine-derived Stage24 decisions/documentation; no scientific re-execution."""
import platform,subprocess
import pandas as pd
from common import *

def table(rows,columns):
    def text(x):return f'{x:.6g}' if isinstance(x,float) else str(x)
    return '| '+' | '.join(columns)+' |\n| '+' | '.join(['---']*len(columns))+' |\n'+'\n'.join('| '+' | '.join(text(r.get(c,'')) for c in columns)+' |' for r in rows)

def run():
    cam=json.loads((B/'camera_data_manifest.json').read_text());depth=json.loads((B/'depth_camera_audit.json').read_text());runtime=json.loads((B/'runtime_memory_pilot.json').read_text());seg=json.loads((A/'segmentation_runtime.json').read_text());cal=json.loads((A/'calibration_audit.json').read_text());assets=json.loads((A/'map_asset_manifest.json').read_text())
    decision=dict(STAGE24A_RGB_MAP='BLOCKED',STAGE24A_SEMANTIC_MAP='BLOCKED',STAGE24B_RGBD_FEASIBILITY='BLOCKED',
        STAGE24B_MONOCULAR_FEASIBILITY='PARTIAL',THREE_ROBOT_READINESS='AUDITED',
        implementation=dict(camera_only_timeline='PASS',native_monocular_pilot='EXECUTED_WITH_INITIALIZATION_FAILURES',pretrained_2D_semantics='PASS',calibration_gated_projection_implementation='SYNTHETIC_TESTS_PASS_REAL_PROJECTION_BLOCKED',P5_geometry_demo=json.loads((A/'demo_validation.json').read_text())['status'] if (A/'demo_validation.json').exists() else 'PASS_PENDING_SEPARATE_BROWSER_VALIDATION'),
        real_metric_visual_loop_factors=0,real_3D_RGB_observations=0,real_3D_semantic_observations=0,
        missing_data=cal['missing'],no_GT_used_in_current_camera_pipeline=True,
        GT_evaluation='NOT_RUN',full_visual_SLAM='NOT_RUN',three_robot_replay='NOT_RUN',network_scalability='NOT_MEASURED',distributed_optimization=False,
        interpretation='Completed authorized feasibility work; NOT a completed RGB/semantic3D map or accepted vision-only multi-robot SLAM system. Native monocular trajectories have arbitrary independent scale.',
        Stage24C='NOT_STARTED')
    dump(B/'vision_only_feasibility_decision.json',decision)
    dump(A/'stage24a_decision.json',{k:v for k,v in decision.items() if k.startswith('STAGE24A') or k in ['implementation','missing_data','real_3D_RGB_observations','real_3D_semantic_observations']})
    research=dict(ORB_SLAM3=dict(source='https://github.com/UZ-SLAMLab/ORB_SLAM3',revision='4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4',
        installation='existing SDK from /home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3; read-only reuse, no reinstall',
        actual_tested_library='local build with existing external-project patches (including sparse map export), not claimed bit-identical upstream wheel',
        requirements='C++14/Eigen/OpenCV/Pangolin/DBoW2/g2o; actual native dependencies found and linked; no GPU required for this CPU pilot',
        RGBD_adapter='TUM association file RGB+registered depth, correct K/D/resolution/depth units/fps; current camera point clouds require independently verified z-buffer/registration adapter',
        monocular_adapter='actual color CameraInfo;640x400 K scaled0.5;D unchanged;original camera timestamps;5Hz sampled windows;no IMU',
        pose_export='camera-to-local T_world_camera as inverse TrackMonocular T_camera_world; valid tracking only; keyframe TUM export',
        map_export='upstream sparse landmark API; installed SDK additionally has SaveSparseMapPLY (not run here)',
        scale='arbitrary monocular scale; each new native Atlas/map may have its own gauge; use explicit Sim3 only in future, no scale oracle',
        license='GPLv3; dependent library licenses apply; binaries/vocabulary NOT committed'),
        MASt3R_SLAM=dict(source='https://github.com/rmurai0610/MASt3R-SLAM',license_source='https://github.com/rmurai0610/MASt3R-SLAM/blob/main/LICENSE.md',
        checkpoint_notice='https://github.com/naver/mast3r/blob/mast3r_sfm/CHECKPOINTS_NOTICE',
        compatibility='official Python3.11 + matching CUDA PyTorch2.5.1/torchvision0.20.1 + compiled/submodule dependencies; existing isolated3.10 CPU GTSAM env is not the target',
        reference_hardware='official experiments RTX4090; minimum VRAM NOT_SPECIFIED_OR_BENCHMARKED here; current server no available NVIDIA runtime discovered',
        input='RGB image folders/video; known intrinsics optional; no LiDAR needed; learned3D/metric priors must not be confused with directly observed depth or ground-truth metric scale',
        export='official mast3r_slam/evaluate.py save_traj and PLY reconstruction export inspected',
        license='CC BY-NC-SA4.0 code; checkpoint/submodule terms separate; research/commercial redistribution constraints require review',
        installed=False,executed=False),
        segmentation=dict(source='https://huggingface.co/nvidia/segformer-b0-finetuned-ade-512-512',
        revision='489d5cd81a0b59fab9b7ea758d3548ebe99677da',license_source='https://github.com/NVlabs/SegFormer/blob/master/LICENSE',
        license='custom NVIDIA SegFormer terms; model card links upstream license; use restricted to this documented research, no weights committed',
        environment='/home/cas/.venvs/fyp_stage24; separate venv layered over installed Python packages; overrides only in new venv; fyp_slam and fyp_gtsam not modified'),
        primary_source_retrieval='official README/LICENSE/model config and export code retrieved; SHA provenance below. No third-party framework benchmarks inferred.')
    sources={}
    for p in Path('/tmp/stage24_sources').glob('*.txt'):
        if p.stat().st_size>100:sources[p.name]=dict(bytes=p.stat().st_size,sha256=sha(p))
    research['retrieved_primary_source_hashes']=sources;dump(B/'visual_framework_feasibility.json',research)
    keysrc=['src/cumulti/stage22a_common.py','src/cumulti/run_stage22a_backend.py','src/cumulti/run_stage13b_incremental_isam2.py','src/cumulti/run_stage103_gtsam_solver_audit.py','src/cumulti/stage22b/replay.py','src/cumulti/stage22b/communication.py','demo/stage20_dual_stream/dual_maps.js','demo/stage23d_policy_map_compare/viewer.js']
    findings=[
        dict(area='robot IDs / sampling',source='stage22a_common.py STARTS; stage20 dual maps robots/starts',finding='hardcoded robot1/robot3, starts150/234 and5796-node schedules',recommendation='RobotRegistry, source-specific Camera/LiDAR Keyframe Registry, measured per-robot timelines; do not invent R2 stable range'),
        dict(area='GTSAM key encoding',source='run_stage13b_incremental_isam2.py numeric_key -> run_stage103 key',finding="any non-robot1 maps to numeric3; any non-1 maps to symbol b. Robot2 and3 would collide",recommendation='new explicit collision-checked mapping; preserve frozen a=R1,b=R3, allocate c=R2,d=R4 for new replay only'),
        dict(area='pair candidates / transform direction',source='stage22b/replay.py canonical_ids and neighborhood',finding='canonical q/c fields explicitly R1/R3, asymmetric delta3/delta5 neighborhoods',recommendation='Pairwise Candidate Manager with full endpoint identities and explicit T_target_source; invert measurement when swapping endpoints'),
        dict(area='communication caches / duplicate suppression',source='stage22b/replay.py Cache; communication.py',finding='cache tuple(robot,kf) partly generic, but tables reserve5796 fixed-size LiDAR clouds and two-robot receipts',recommendation='typed sensor/modality/hash keys; per-unordered-robot-pair Verified Factor Ledger and separate per-pair duplicate policy'),
        dict(area='first bridge / disconnected components',source='run_stage22a_backend.py first bridge S and one persistentISAM2',finding='one S aligns all R3 to R1 and assumptions cover2 components only',recommendation='Connectivity Manager/DSU; independent component gauges until actual bridges, one anchor per connected component with explicit merge/rebase; never submit singular disconnected graph'),
        dict(area='queue / budgets',source='stage22b/replay.py rolling counters; stage23d/scheduler.py',finding='two-direction fairness and fixed query/candidate schema; no3-pair allocation experiment',recommendation='per-pair queues and global/per-pair hard accounting; explicit prospective fairness before comparison; no real-network claims'),
        dict(area='renderer',source='stage20 dual_maps.js and Stage23D comparison assets',finding='two source colors, two local panels and fixed loop endpoint lookup',recommendation='Global Map Renderer consumes registry/list of robots, source colors and real connectivity; stage19/20 historical dashboard unchanged')]
    dump(B/'three_robot_readiness_audit.json',dict(status='AUDITED',source_hashes={p:sha(ROOT/p) for p in keysrc if (ROOT/p).exists()},findings=findings,
        proposed_components=['RobotRegistry','Camera/LiDAR Keyframe Registry','Pairwise Candidate Manager','Verified Factor Ledger','Connectivity Manager','Central Pose Graph Backend','Global Map Renderer'],
        robot2='RGB,LiDAR,IMU/GPS,camera depth archives locally present; current depth metadata only, stable odometry and camera adapter NOT validated for3-agent replay',
        robot4='only GT CSV present in local raw inventory; no sensor measurements can be invented',three_or_four_robot_run=False))
    # Record engineering failures separately from accepted computations.
    dump(B/'engineering_recovery.json',dict(events=[
        dict(event='native adapter compile/link preflight',reason='pinned Pangolin HAVE_EPOXY and display symbols required',native_track_calls=0,repair='match installed SDK compile definitions and link libraries; no SDK modification'),
        dict(event='native shared-library preflight',reason='transitive Pangolin windowing library not found',native_track_calls=0,repair='isolated subprocess LD_LIBRARY_PATH; original loader log retained'),
        dict(event='adapter syntax preflight',native_track_calls=0,repair='remove excess parenthesis before tracking'),
        dict(event='depth legacy schema decode',reason='actual embedded Extrinsics includes std_msgs/Header; old manually registered schema rejected by CDR size check',repair='register actual embedded ros2msg definitions before decoding; no fabricated calibration',
             cost='two temporary full depth DB expansions occurred before the successful schema-aware audit; first lost SSH stderr, second explicit log retained; first exact runtime NOT_MEASURED. Extra I/O is not hidden as successful decode or warmup'),
        dict(event='dependency network fallback',reason='server download intermittent/slow',repair='official pinned model and PyPI wheels fetched on local host and copied to new stage/cache; shared environments not modified')],
        scientific_threshold_changes=False,failed_native_or_depth_artifacts_preserved=True,native_scientific_runs_completed=6,GT_based_retuning=False))
    sync=pd.read_csv(B/'camera_time_sync_audit.csv');camera_rows=[]
    for robot,x in cam['robots'].items():
        camera_rows.append(dict(robot=robot,original_RGB_frames=x['original_frames'],independent_visual_keyframes=int((pd.read_csv(B/'visual_keyframe_manifest.csv').robot_id==robot).sum()),
            pilot_frames=len(x['pilot_frames']),pilot_JPEG_bytes=x['export_bytes'],export_s=x['elapsed_s'],
            header_minus_bag_median_ms=float(sync[sync.robot_id==robot].header_minus_bag_ms.median())))
    docA=f'''# CU-Multi Stage24A — calibration-gated RGB / semantic mapping

STAGE24A_RGB_MAP: **BLOCKED**. STAGE24A_SEMANTIC_MAP: **BLOCKED**.
Implementation and2D segmentation are available, but no real RGB/semantic3D projection is claimed.

## Inputs and preserved science
Frozen Stage22B P5 incremental poses are the demonstration reference. The new viewer reuses byte-identical capped P5 point buffers from Stage23D, retaining580 original source scans (frame10/point16/voxel0.5m/range1–60m), metric XYZ and optimized poses. The trajectory SHA is `{assets['trajectory_sha256']}`. No P0/P5/P6 retuning, Stage18–23E artifact change, GT alignment, new factor or mesh. These are separate recordings and a centralized LiDAR-based prototype; geometry is not camera-only or geographic ENU.

## Exact availability / coordinate audit
See `24a_rgb_semantic/data_availability_report.json` for actual archive paths/bytes/topics and explicit decode scopes. R1/R3 RGB ZIP members were read through EOF/CRC and real frames decoded. R2 RGB/depth are inventoried, not newly fully decoded. R1 depth-camera PointCloud2 was decoded; R3 depth is absent. No native depth Image topic is present in the inspected R1/R2 depth metadata. Depth scale cannot be silently replaced by LiDAR.

URDF joint origins are **T_parent_child**, not T_child_parent. Chain composition/inversion is recorded in calibration_audit.json. URDF reaches camera_link but not camera_color_optical_frame; all archived URDF/xacro optical mentions are audited. Mount geometry visual rotations are not measurement calibration. The missing optical mounting translation/rotation is not supplied by depth_to_color, which links two camera optical sensors. No standard-axis guess is promoted to measured sensor extrinsics. Image optical convention is x right/y down/z forward. Intrinsics use native1280×800, actual plumb_bob coefficients, and scaled K only for resized640×400 pilot images.

`rgb_projection_validation.json` explicitly records real_projection=False. The reusable projector rejects missing calibration and bad sync (>50ms bag difference), removes negative/out-of-FoV points, applies distortion and exact-pixel nearest-depth z-buffer. Synthetic near/far/occlusion/direction/NaN tests pass. This does **not** validate real-world occlusion, hardware exposure timing or calibration. No observations are fused across scans until calibration is resolved; original voxel representatives are kept, so duplicate RGB/semantic observations are neither averaged nor invented.

## Actual semantic pilot
Pretrained [SegFormer B0 ADE20K](https://huggingface.co/nvidia/segformer-b0-finetuned-ade-512-512), pinned revision489d5cd81a0b59fab9b7ea758d3548ebe99677da,512×512 input, CPU2threads, confidence threshold0.5. Exact150-class mapping is frozen in semantic_model_policy.json: building/house/skyscraper→building; road/sidewalk/path/runway→road/asphalt bucket; grass→grass; named plant/tree groups→vegetation; named vehicles→vehicle; earth/sand/land/etc→dirt/other; all unrepresented classes and low-confidence pixels→unknown. The road bucket does not establish asphalt material. Raw ADE IDs/confidence arrays are preserved server-side with hashes. No LiDAR labels or semantic GT used.

Twelve deterministic real-image samples ran; inference seconds median/p95/max: {seg['inference_seconds']['median']:.6f}/{seg['inference_seconds']['p95']:.6f}/{seg['inference_seconds']['max']:.6f}. Native overexposure, magenta tint and motion blur are visible; vegetation boundaries/fine objects and road/grass boundaries can fail. The all-unknown overexposed opening frame is retained. Qualitative image inspection is not IoU/mIoU accuracy; no semantic GT was available or accessed. semantic_class_distribution.csv is explicitly **image pixels only**, never3D coverage.

## Demo and validation
`demo/stage24_rgb_semantic/` offers robot/RGB/semantic modes with persistent camera controls. Robot mode blue/orange shows provenance. RGB/semantic modes show100%unknown gray because observed3D vertices=0. This deliberately does not advertise a colored map as complete. Twelve authentic image/prediction pairs are displayed separately as2D evidence. Region coverage CSV records zero colored/labeled points and explicit missing regional point counts, not fake observed zeros as calibrated accuracy. Separate browser validation/screenshots and independent artifact audit establish interface/data behavior only.

Serve parent demo over HTTP (`python -m http.server 8021 --bind 127.0.0.1 --directory demo`) then `/stage24_rgb_semantic/`. See README for exact command. Do not double-click index.html. Runtime/map manifests preserve deterministic source SHA/bytes; raw bags, temporary DBs, weights and oversized sequences are not committed. SegFormer has [custom upstream terms](https://github.com/NVlabs/SegFormer/blob/master/LICENSE), not an assumed MIT/Apache license.

## Minimum separately authorized Stage24C
Obtain measured, frame-identified camera_link↔color/depth optical transforms (recorded static TF or matching serial/driver calibration), verify exposure/header/bag synchronization, then validate projection overlays, FoV/occlusions/depth ordering and duplicate-observation handling on a small fixed cohort. Only then project RGB/semantic labels into frozen P5 geometry and report actual3D coverage. Do not use GT or hand-tuned frame rotations to make overlays look right. No Stage24C started.
'''
    native_rows=[dict(robot=r['robot_id'],window=str(r['window']),frames=r['frames'],tracked=r['valid_poses'],fraction=r['tracking_fraction'],track_median_ms=r['track_ms']['median'],wall_s=r['wall_s'],peak_MiB=r['peak_RSS_MiB']) for r in runtime['native_runs']]
    docB=f'''# CU-Multi Stage24B — isolated vision-only feasibility

STAGE24B_RGBD_FEASIBILITY: **BLOCKED**. STAGE24B_MONOCULAR_FEASIBILITY: **PARTIAL**. THREE_ROBOT_READINESS: **AUDITED**.
This is a small retrospective data/implementation feasibility study, not independent route validation or a full multi-robot visual SLAM result.

## Three distinct modes
**H1 hybrid visual loop closure** retains frozen LiDAR-derived local odometry and adds visual retrieval/geometry; never call it vision-only. **V1 RGB-D** permits only RGB and camera depth with calibrated metric SE3 visual motion/factors; no LiDAR, SC, GICP, GNSS/IMU or LiDAR-derived odometry. **V2 strict RGB-only** permits only images and intrinsics; monocular scale is arbitrary, cross-map scale may require Sim3. No metric scale oracle, RGB-D/LiDAR scale injection or pretending independently initialized maps share one gauge.

## Source-backed visual recovery
Actual Stage5/6 source was inspected and hashed in crossmax_baseline_review.json. ModelViT-B-32-quickgelu /laion400m_e32; Single=normalized cosine, Mean5=causal5 embedding mean then renormalize, Cross-Max=max25 pairwise cosines. All historically rerank the **frozen SC Top20** of2Hz LiDAR-indexed frames; they do not track visual motion, make depth, or map. Stage5 end-to-end R@1 SC/Single/Mean/Cross=0.993453/0.702128/0.682488/0.714675; Single4rescues538regressions, Mean2/572, Cross1/512. Stage6 strict Single gate0.993999 with1rescue0regressions and4.31%invocation; no general viewpoint-invariance or end-to-end visual SLAM claim. Historical results unchanged.

## Camera-only timeline / adaptation
Camera bag timestamps alone define independent1Hz keyframes; original message IDs/indexes/timestamps are retained. Native camera streams remain near10Hz; the1Hz timeline and5Hz pilot are deliberate subsamples, not sensor rates. Three predeclared12s windows start at camera elapsed0/120/240 for each robot. Exported180frames/robot640×400 JPEG quality85, not the historical LiDAR-selected RGB set. JPEG compression/downsampling are pilot limitations, not raw lossless reproduction. Native1280×800 plumb_bob K/D, original headers and bag offsets are explicit. Bag time is not calibrated exposure time; no undocumented clock correction.

{table(camera_rows,list(camera_rows[0]))}

RGB-D metadata has camera PointCloud2 rather than native depth images. A real R1 cloud and embedded Header-bearing Extrinsics definition were decoded; R2 depth is metadata-only, R3 depth missing. R1 depth intrinsics are848×480, while the actual first camera cloud is unorganized: height1,width376811,fields x/y/z/rgb,frame robot1_camera_depth_optical_frame. It cannot simply be reshaped into a depth image. The sample maximumZ65.535m suggests a saturation/invalid-depth issue needing a validated range policy; finite coordinates alone do not certify valid depth. No silent Ouster substitute or uint16 scale guess. Full RGB/depth sync statistics (R1 camera-cloud vs RGB bag time) are recorded separately, not advertised as exposure calibration. A calibrated camera-cloud-to-depth z-buffer reconstruction plus depth/RGB registration/units/coverage audit is needed before RGB-D ORB-SLAM3. This adapter was not fabricated or silently added.

## Actual native monocular experiment
Read-only reuse of the already installed [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) SDK at revision4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4, with existing external-project patches; exact library/header/source/vocabulary SHA recorded. No external SDK or old research file modified. New original headless C++ adapter uses MONOCULAR,viewer=false,zeroIMU and original example ORB defaults. Native library dependencies isolated by subprocess LD_LIBRARY_PATH. Tracking pose export inverts T_camera_world to T_world_camera and leaves invalid frames empty, never identity fill. Each window starts a separate Atlas; resets/new maps may have independent scale/gauge. No metric SE3 loop or cross-window continuous map is claimed. Native keyframe TUM exports are retained; upstream supports sparse landmarks and the installed SDK also has a custom PLY export API (not exercised here).

{table(native_rows,list(native_rows[0]))}

Two opening windows never initialized and remain failures. Finite/unit-quaternion tests apply only to valid tracked poses; they establish numeric validity, not accuracy. No GT evaluator run. track_ms and wall/RSS are separate; native strace adds overhead. These short windows do not demonstrate end-to-end real-time or full-route stability. No solver parameters were tuned after results.

## Image-only cross-robot retrieval and verification
New72 image embeddings (36/robot from camera-only integer-second pilot ticks), original fixed OpenCLIP model, full36-image opposite pilot database, top1 cosine; **no SC pool** or historical LiDAR embeddings. This is offline full-pilot retrieval: later camera frames within the fixed cohort may be present. It is not Stage22-style causal online retrieval, and no candidate-arrival or live timing claim is made. {runtime['image_only_retrieval']['epipolar_plausible']}/72 satisfy the predeclared nonmetric epipolar plausibility rule: mutual ORB ratio0.75,>=50matches, RANSAC normalized threshold0.003/prob0.999,>=50%inliers,>=30cheirality inliers and>=1degree median parallax. Essential recovery is T_candidateOptical_from_queryOptical with unit translation direction, **not** metric T_query_from_candidate; future canonical factors would require inversion and observable scale. No RGB-D3D-3D/PnP run because a validated depth-image adapter is unavailable. No accepted visual loop factors exported. A plausible epipolar fit does not establish a correct place match or precision/recall; GT was not used to choose/evaluate these pairs.

## Leakage guards and research scope
Python audit hook rejects GT,LiDAR caches/scans,SC and historical backend input paths. Native strace all-thread open records are checked for forbidden inputs. This is a file-access audit, not a native OS sandbox. The camera-only processes read original RGB, CameraInfo, vocabulary/weights and their own outputs only. No GT-selected clips,candidates,scale or parameters. Stage24A separately reads P5 LiDAR geometry, never passed into this tracking process. Independent saved-artifact auditing, including an actual denied-open guard test, is recorded without rerunning native tracking. All Stage18–23E tracked artifacts are hash-checked unchanged.

## Other baseline feasibility / licenses
ORB-SLAM3 CPU compatibility is demonstrated by the pilot; GPLv3 and dependency licenses apply. RGB-D adaptation remains blocked; no all-framework installation. [MASt3R-SLAM](https://github.com/rmurai0610/MASt3R-SLAM) official code expects separatePython3.11,matching CUDA PyTorch2.5.1 and compiled/submodule dependencies. Official reference runs use RTX4090; minimum VRAM is not specified/benchmarked here and no usable NVIDIA runtime was found. Native code exports trajectories and PLY via evaluate.py; learned metric priors are not directly sensed depth or a scale oracle. Its [CC BY-NC-SA4.0 code license](https://github.com/rmurai0610/MASt3R-SLAM/blob/main/LICENSE.md) and [checkpoint notices](https://github.com/naver/mast3r/blob/mast3r_sfm/CHECKPOINTS_NOTICE) are distinct. It was not installed or run. Exact primary-source hashes/compatibility findings are in visual_framework_feasibility.json; no invented memory/performance benchmark.

## Three-/four-robot readiness — inspection only
{table(findings,['area','source','finding','recommendation'])}

Recommended components: RobotRegistry; Camera/LiDAR Keyframe Registry; Pairwise Candidate Manager; Verified Factor Ledger; Connectivity Manager; Central Pose Graph Backend; Global Map Renderer. Current numeric_key aliases all nonR1 toR3 and symbol b: addingR2 unchanged would collide. Preserve frozen a=R1,b=R3, allocate explicit c=R2,d=R4 in a new collision-checked replay; never rewrite frozen outputs. Generalized loop identity must include both robot/sensor/frame IDs; canonical swaps invert transforms. Separate disconnected local gauges until real bridges; handle each component/anchor and merge explicitly. Per-pair queues/global budgets require prospective accounting over3or6unordered pairs, not invented measurements. This is centralized, not distributed SLAM or a real-network scalability result.

R2 has RGB,LiDAR,depth-camera and IMU/GPS archives, but new3-agent stable-range/odometry/camera adapters are not validated. R4 has only GT locally; no sensor replay can be synthesized from poses. No3/4-robot run, no new odometry or loop discovery.

## Reproduction / minimum Stage24C
Use `scripts/run_stage24_ab.sh`; it resumes only completed phases and refuses scientific-output overwrite. Prerequisites: existing hashed ORB SDK, original raw camera archives, frozen P5 assets, `/home/cas/.venvs/fyp_gtsam`, and isolated semantic venv. Create semantic venv separately with system site-packages; install transformers4.57.1/tokenizers0.22.1/huggingface-hub0.36.0 only there. `scripts/setup_stage24_semantics.py` fetches only pinned public model files and verifies weight SHA; weights/vocabulary/binaries/temporary DBs/raw sequences are not committed. Browser tests are separate, followed by historical preservation and staging audits. Engineering compile/link/loader/legacy-message-schema failures are logged separately; no thresholds changed, no hidden repeated successful native experiment. Root memory receives append-only Stage24 findings.

For a fresh scientific re-export, use new directories; the committed lightweight results intentionally omit the server-only360-frame sequence and native executable. Do not ask the resume path to treat a checkout without those heavy assets as a complete local run:

```bash
STAGE24_OUTPUT_ROOT="$PWD/outputs/cumulti_v1/stage24_reproduction_v1" \\
STAGE24_DEMO_DIR="$PWD/demo/stage24_reproduction_v1" \\
bash scripts/run_stage24_ab.sh
```

Fresh-run reports stay under the new output root rather than overwriting accepted Stage24 documentation. Full fresh native/IO re-execution was not repeated after inspecting results; only resume/audit and alternate-path import/shell checks were run. Browser functional measurements belong to the workstation test, not the native tracking benchmark.

Minimum next authorization: verify missing optical mounting/exposure calibration; validate camera-cloud-to-registered-depth conversion and obtain R3 camera depth if retaining R1/R3 RGB-D; expand a prospectively fixed monocular camera-only sequence with explicit initialization/reset/Sim3 handling and image-only geometric rejection. Generic registry migration precedes any3-agent run. Do not expand to5796 visual events or another framework automatically. **Stop after24A/B; Stage24C not started.**
'''
    report_dir=ROOT/'docs' if RUN_BASE==BASE else RUN_BASE/'reports';report_dir.mkdir(parents=True,exist_ok=True)
    (report_dir/'CUMULTI_STAGE24A_RGB_SEMANTIC_MAP.md').write_text(docA)
    (report_dir/'CUMULTI_STAGE24B_VISION_FEASIBILITY.md').write_text(docB)
    dump(B/'stage24_environment.json',dict(python=platform.python_version(),camera_python=sys.executable,
        semantic_env='/home/cas/.venvs/fyp_stage24',shared_GTSAM_numpy='1.26.4 verified after semantic setup',
        source_files={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/cumulti/stage24').glob('*') if p.is_file()}))
    print(json.dumps(decision,indent=2),flush=True)

if __name__=='__main__':run()
