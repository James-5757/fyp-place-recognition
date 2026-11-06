# CU-Multi Stage24B — isolated vision-only feasibility

STAGE24B_RGBD_FEASIBILITY: **BLOCKED**. STAGE24B_MONOCULAR_FEASIBILITY: **PARTIAL**. THREE_ROBOT_READINESS: **AUDITED**.
This is a small retrospective data/implementation feasibility study, not independent route validation or a full multi-robot visual SLAM result.

## Three distinct modes
**H1 hybrid visual loop closure** retains frozen LiDAR-derived local odometry and adds visual retrieval/geometry; never call it vision-only. **V1 RGB-D** permits only RGB and camera depth with calibrated metric SE3 visual motion/factors; no LiDAR, SC, GICP, GNSS/IMU or LiDAR-derived odometry. **V2 strict RGB-only** permits only images and intrinsics; monocular scale is arbitrary, cross-map scale may require Sim3. No metric scale oracle, RGB-D/LiDAR scale injection or pretending independently initialized maps share one gauge.

## Source-backed visual recovery
Actual Stage5/6 source was inspected and hashed in crossmax_baseline_review.json. ModelViT-B-32-quickgelu /laion400m_e32; Single=normalized cosine, Mean5=causal5 embedding mean then renormalize, Cross-Max=max25 pairwise cosines. All historically rerank the **frozen SC Top20** of2Hz LiDAR-indexed frames; they do not track visual motion, make depth, or map. Stage5 end-to-end R@1 SC/Single/Mean/Cross=0.993453/0.702128/0.682488/0.714675; Single4rescues538regressions, Mean2/572, Cross1/512. Stage6 strict Single gate0.993999 with1rescue0regressions and4.31%invocation; no general viewpoint-invariance or end-to-end visual SLAM claim. Historical results unchanged.

## Camera-only timeline / adaptation
Camera bag timestamps alone define independent1Hz keyframes; original message IDs/indexes/timestamps are retained. Native camera streams remain near10Hz; the1Hz timeline and5Hz pilot are deliberate subsamples, not sensor rates. Three predeclared12s windows start at camera elapsed0/120/240 for each robot. Exported180frames/robot640×400 JPEG quality85, not the historical LiDAR-selected RGB set. JPEG compression/downsampling are pilot limitations, not raw lossless reproduction. Native1280×800 plumb_bob K/D, original headers and bag offsets are explicit. Bag time is not calibrated exposure time; no undocumented clock correction.

| robot | original_RGB_frames | independent_visual_keyframes | pilot_frames | pilot_JPEG_bytes | export_s | header_minus_bag_median_ms |
| --- | --- | --- | --- | --- | --- | --- |
| robot1 | 9997 | 1000 | 180 | 9499690 | 216.855 | -105.986 |
| robot3 | 20977 | 2099 | 180 | 8805316 | 549.936 | -104.588 |

RGB-D metadata has camera PointCloud2 rather than native depth images. A real R1 cloud and embedded Header-bearing Extrinsics definition were decoded; R2 depth is metadata-only, R3 depth missing. R1 depth intrinsics are848×480, while the actual first camera cloud is unorganized: height1,width376811,fields x/y/z/rgb,frame robot1_camera_depth_optical_frame. It cannot simply be reshaped into a depth image. The sample maximumZ65.535m suggests a saturation/invalid-depth issue needing a validated range policy; finite coordinates alone do not certify valid depth. No silent Ouster substitute or uint16 scale guess. Full RGB/depth sync statistics (R1 camera-cloud vs RGB bag time) are recorded separately, not advertised as exposure calibration. A calibrated camera-cloud-to-depth z-buffer reconstruction plus depth/RGB registration/units/coverage audit is needed before RGB-D ORB-SLAM3. This adapter was not fabricated or silently added.

## Actual native monocular experiment
Read-only reuse of the already installed [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) SDK at revision4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4, with existing external-project patches; exact library/header/source/vocabulary SHA recorded. No external SDK or old research file modified. New original headless C++ adapter uses MONOCULAR,viewer=false,zeroIMU and original example ORB defaults. Native library dependencies isolated by subprocess LD_LIBRARY_PATH. Tracking pose export inverts T_camera_world to T_world_camera and leaves invalid frames empty, never identity fill. Each window starts a separate Atlas; resets/new maps may have independent scale/gauge. No metric SE3 loop or cross-window continuous map is claimed. Native keyframe TUM exports are retained; upstream supports sparse landmarks and the installed SDK also has a custom PLY export API (not exercised here).

| robot | window | frames | tracked | fraction | track_median_ms | wall_s | peak_MiB |
| --- | --- | --- | --- | --- | --- | --- | --- |
| robot1 | [0, 12] | 60 | 0 | 0 | 50.9681 | 15.5025 | 501 |
| robot1 | [120, 132] | 60 | 47 | 0.783333 | 16.2321 | 10.1857 | 510.996 |
| robot1 | [240, 252] | 60 | 58 | 0.966667 | 16.8641 | 9.93634 | 514 |
| robot3 | [0, 12] | 60 | 0 | 0 | 61.4082 | 11.786 | 501.996 |
| robot3 | [120, 132] | 60 | 49 | 0.816667 | 16.3622 | 14.7019 | 508.852 |
| robot3 | [240, 252] | 60 | 41 | 0.683333 | 16.3422 | 11.6927 | 503.852 |

Two opening windows never initialized and remain failures. Finite/unit-quaternion tests apply only to valid tracked poses; they establish numeric validity, not accuracy. No GT evaluator run. track_ms and wall/RSS are separate; native strace adds overhead. These short windows do not demonstrate end-to-end real-time or full-route stability. No solver parameters were tuned after results.

## Image-only cross-robot retrieval and verification
New72 image embeddings (36/robot from camera-only integer-second pilot ticks), original fixed OpenCLIP model, full36-image opposite pilot database, top1 cosine; **no SC pool** or historical LiDAR embeddings. This is offline full-pilot retrieval: later camera frames within the fixed cohort may be present. It is not Stage22-style causal online retrieval, and no candidate-arrival or live timing claim is made. 1/72 satisfy the predeclared nonmetric epipolar plausibility rule: mutual ORB ratio0.75,>=50matches, RANSAC normalized threshold0.003/prob0.999,>=50%inliers,>=30cheirality inliers and>=1degree median parallax. Essential recovery is T_candidateOptical_from_queryOptical with unit translation direction, **not** metric T_query_from_candidate; future canonical factors would require inversion and observable scale. No RGB-D3D-3D/PnP run because a validated depth-image adapter is unavailable. No accepted visual loop factors exported. A plausible epipolar fit does not establish a correct place match or precision/recall; GT was not used to choose/evaluate these pairs.

## Leakage guards and research scope
Python audit hook rejects GT,LiDAR caches/scans,SC and historical backend input paths. Native strace all-thread open records are checked for forbidden inputs. This is a file-access audit, not a native OS sandbox. The camera-only processes read original RGB, CameraInfo, vocabulary/weights and their own outputs only. No GT-selected clips,candidates,scale or parameters. Stage24A separately reads P5 LiDAR geometry, never passed into this tracking process. Independent saved-artifact auditing, including an actual denied-open guard test, is recorded without rerunning native tracking. All Stage18–23E tracked artifacts are hash-checked unchanged.

## Other baseline feasibility / licenses
ORB-SLAM3 CPU compatibility is demonstrated by the pilot; GPLv3 and dependency licenses apply. RGB-D adaptation remains blocked; no all-framework installation. [MASt3R-SLAM](https://github.com/rmurai0610/MASt3R-SLAM) official code expects separatePython3.11,matching CUDA PyTorch2.5.1 and compiled/submodule dependencies. Official reference runs use RTX4090; minimum VRAM is not specified/benchmarked here and no usable NVIDIA runtime was found. Native code exports trajectories and PLY via evaluate.py; learned metric priors are not directly sensed depth or a scale oracle. Its [CC BY-NC-SA4.0 code license](https://github.com/rmurai0610/MASt3R-SLAM/blob/main/LICENSE.md) and [checkpoint notices](https://github.com/naver/mast3r/blob/mast3r_sfm/CHECKPOINTS_NOTICE) are distinct. It was not installed or run. Exact primary-source hashes/compatibility findings are in visual_framework_feasibility.json; no invented memory/performance benchmark.

## Three-/four-robot readiness — inspection only
| area | source | finding | recommendation |
| --- | --- | --- | --- |
| robot IDs / sampling | stage22a_common.py STARTS; stage20 dual maps robots/starts | hardcoded robot1/robot3, starts150/234 and5796-node schedules | RobotRegistry, source-specific Camera/LiDAR Keyframe Registry, measured per-robot timelines; do not invent R2 stable range |
| GTSAM key encoding | run_stage13b_incremental_isam2.py numeric_key -> run_stage103 key | any non-robot1 maps to numeric3; any non-1 maps to symbol b. Robot2 and3 would collide | new explicit collision-checked mapping; preserve frozen a=R1,b=R3, allocate c=R2,d=R4 for new replay only |
| pair candidates / transform direction | stage22b/replay.py canonical_ids and neighborhood | canonical q/c fields explicitly R1/R3, asymmetric delta3/delta5 neighborhoods | Pairwise Candidate Manager with full endpoint identities and explicit T_target_source; invert measurement when swapping endpoints |
| communication caches / duplicate suppression | stage22b/replay.py Cache; communication.py | cache tuple(robot,kf) partly generic, but tables reserve5796 fixed-size LiDAR clouds and two-robot receipts | typed sensor/modality/hash keys; per-unordered-robot-pair Verified Factor Ledger and separate per-pair duplicate policy |
| first bridge / disconnected components | run_stage22a_backend.py first bridge S and one persistentISAM2 | one S aligns all R3 to R1 and assumptions cover2 components only | Connectivity Manager/DSU; independent component gauges until actual bridges, one anchor per connected component with explicit merge/rebase; never submit singular disconnected graph |
| queue / budgets | stage22b/replay.py rolling counters; stage23d/scheduler.py | two-direction fairness and fixed query/candidate schema; no3-pair allocation experiment | per-pair queues and global/per-pair hard accounting; explicit prospective fairness before comparison; no real-network claims |
| renderer | stage20 dual_maps.js and Stage23D comparison assets | two source colors, two local panels and fixed loop endpoint lookup | Global Map Renderer consumes registry/list of robots, source colors and real connectivity; stage19/20 historical dashboard unchanged |

Recommended components: RobotRegistry; Camera/LiDAR Keyframe Registry; Pairwise Candidate Manager; Verified Factor Ledger; Connectivity Manager; Central Pose Graph Backend; Global Map Renderer. Current numeric_key aliases all nonR1 toR3 and symbol b: addingR2 unchanged would collide. Preserve frozen a=R1,b=R3, allocate explicit c=R2,d=R4 in a new collision-checked replay; never rewrite frozen outputs. Generalized loop identity must include both robot/sensor/frame IDs; canonical swaps invert transforms. Separate disconnected local gauges until real bridges; handle each component/anchor and merge explicitly. Per-pair queues/global budgets require prospective accounting over3or6unordered pairs, not invented measurements. This is centralized, not distributed SLAM or a real-network scalability result.

R2 has RGB,LiDAR,depth-camera and IMU/GPS archives, but new3-agent stable-range/odometry/camera adapters are not validated. R4 has only GT locally; no sensor replay can be synthesized from poses. No3/4-robot run, no new odometry or loop discovery.

## Reproduction / minimum Stage24C
Use `scripts/run_stage24_ab.sh`; it resumes only completed phases and refuses scientific-output overwrite. Prerequisites: existing hashed ORB SDK, original raw camera archives, frozen P5 assets, `/home/cas/.venvs/fyp_gtsam`, and isolated semantic venv. Create semantic venv separately with system site-packages; install transformers4.57.1/tokenizers0.22.1/huggingface-hub0.36.0 only there. `scripts/setup_stage24_semantics.py` fetches only pinned public model files and verifies weight SHA; weights/vocabulary/binaries/temporary DBs/raw sequences are not committed. Browser tests are separate, followed by historical preservation and staging audits. Engineering compile/link/loader/legacy-message-schema failures are logged separately; no thresholds changed, no hidden repeated successful native experiment. Root memory receives append-only Stage24 findings.

For a fresh scientific re-export, use new directories; the committed lightweight results intentionally omit the server-only360-frame sequence and native executable. Do not ask the resume path to treat a checkout without those heavy assets as a complete local run:

```bash
STAGE24_OUTPUT_ROOT="$PWD/outputs/cumulti_v1/stage24_reproduction_v1" \
STAGE24_DEMO_DIR="$PWD/demo/stage24_reproduction_v1" \
bash scripts/run_stage24_ab.sh
```

Fresh-run reports stay under the new output root rather than overwriting accepted Stage24 documentation. Full fresh native/IO re-execution was not repeated after inspecting results; only resume/audit and alternate-path import/shell checks were run. Browser functional measurements belong to the workstation test, not the native tracking benchmark.

Minimum next authorization: verify missing optical mounting/exposure calibration; validate camera-cloud-to-registered-depth conversion and obtain R3 camera depth if retaining R1/R3 RGB-D; expand a prospectively fixed monocular camera-only sequence with explicit initialization/reset/Sim3 handling and image-only geometric rejection. Generic registry migration precedes any3-agent run. Do not expand to5796 visual events or another framework automatically. **Stop after24A/B; Stage24C not started.**
