# FYP project context

The FYP studies LiDAR place recognition and inter-robot data association. Historical KITTI Sequence 00 work uses Scan Context for high-recall geometric candidates, then tests BEV, RGB/OpenCLIP and temporal Cross-Max visual filtering/reranking. Cross-Max takes the strongest pairwise cosine similarity between query/candidate causal temporal windows rather than averaging embeddings. VLM was evaluated only as a semantic verifier and is excluded from the current CU-Multi main pipeline.

The documented formal split uses frame step 5, split 2500, gap 100, Robot A through frame 2400 and Robot B from frame 2600; positives have ground-truth x-z distance below 5 m. Ground truth is strictly offline for synchronization, overlap/positive definition, analysis and evaluation.

Existing layout: src/ contains retrieval, BEV/RGB/temporal, VLM, analysis and canonical-v2 code; outputs/ contains historical metrics and generated artefacts; data/, models/ and archive/ are private/large and ignored. README_zh.md is the detailed original report.

The next stage is CU-Multi Main Campus robot1/robot2. Its first milestone is a separate 100-keyframe-per-robot adapter/validation; it must not overwrite KITTI outputs/canonical_v2. Scan Context remains the candidate anchor. The historical frozen Top-20-to-Top-5 filtering fusion is 0.6*SC + 0.4*Cross-Max; 0.7 SC was a secondary ranking ablation.

## Stage22A — causal frontend context

The current research direction is communication/computation-efficient
collaborative LiDAR SLAM. Stage22A replaces archived clean75 discovery
assumptions with actually computed causal bidirectional SC and new GICP
measurements over5796 normalized R1/R3 arrivals. Decision: CAUSAL_ONLINE_GRAPH_READY; backend
CAUSAL_ONLINE_GRAPH_READY. This is separate-recording replay, not live dual robots or
decentralized optimization. Historical frozen results remain unchanged.
Read the Stage22A doc and RESULTS for coverage, numerical and offline-accuracy
limitations. GT is offline only; Stage22B is not automatically authorized.

## Stage24A/B — camera feasibility checkpoint (2026-11-06)

Stage24A RGB/semantic3D maps remain BLOCKED: archived URDF reaches camera_link
but not the color optical frame. Do not guess mounting/optical transforms or
color unseen surfaces. New calibration-gated P5 demo and12 real SegFormerB0
2D predictions exist; RGB/semantic map modes explicitly show100%unknown.
RGB-D is BLOCKED for R1/R3: R3 depth missing; R1/R2 depth archives contain
camera PointCloud2 rather than native depth images. Actual R1 cloud is
unorganized,376811x1,with848x480 camera calibration; raw depth scale and
registered-image adaptation require validation. R1 camera-cloud/RGB bag sync
median1.38209ms,p956.2070072ms is not hardware exposure calibration.

Independent camera-only timelines contain1000/2099 R1/R3 visual keyframes.
Six fixed12s ORB-SLAM3 monocular windows each use60real RGB frames: valid
counts R1=0/47/58,R3=0/49/41. Arbitrary per-map scale is preserved; no LiDAR,
SC,GICP,GNSS,IMU or GT supplies tracking/geometry/scale. New offline image-only
retrieval72queries/36opposite candidates yields1epipolar-plausible nonmetric
pair,not a verified metric loop or measured place-recognition precision.
Later pilot frames may be in the offline database; this is not causal online
retrieval. No full visual SLAM or GT evaluation was run.

Three-robot inspection finds existing numeric_key/symbol mapping aliases
Robot2 with Robot3; generic registry/typed endpoints/connectivity and per-pair
budget migration are needed before a new3-agent run. R2 archives exist; R4
has only GT locally. No sensor measurements may be invented.
Historical10098 tracked files match their pre-stage hashes. Independent
artifact audit831checks PASS; browser orbit/zoom/pan/mode checks PASS, with
software-WebGL performance only. No old scientific artifact or frozen
P0/P5/P6 policy was changed. Details and reproducible commands:
docs/CUMULTI_STAGE24A_RGB_SEMANTIC_MAP.md and
docs/CUMULTI_STAGE24B_VISION_FEASIBILITY.md.
Stop after24A/B; Stage24C is NOT_STARTED. Minimum next work is measured optical
mounting/exposure calibration and validated camera-cloud-to-depth adaptation,
then small fixed-cohort projection/visual-geometry tests,not automatic full runs.

## Bounded camera RGB-D local-map demo (2026-11-08)

PASS_FUNCTIONAL_DEMO_TRACKING_PARTIAL. PARTIAL_PREDECLARED_GATE_NOT_MET. New independent camera-only maps, original depth PointCloud2 adaptation, timestamp-aligned RGB and native paced ORB-SLAM3 RGBD. No LiDAR/GT/IMU input or semantic map; reset epochs never merged. No cross-robot visual fusion or full-sequence run. See docs/CUMULTI_RGBD_CAMERA_MAPPING_DEMO.md and demo/rgbd_camera_mapping/README.md. Historical Stage24A/B blocked reports are preserved; Robot3 depth arrival resolves availability, not missing LiDAR-optical calibration.

Verified short-window tracking: robot1 44/60 valid frames, 2 separate tracking epoch(s), 76260 display points across its epochs; robot3 60/60 valid frames, 1 separate tracking epoch(s), 100000 display points across its epochs. No joint camera map has been established.

## Native-rate RGB-D long-window stability (2026-11-08)

PASS_AUDIT. Frozen pre-fusion decision: NO_GO_RGBD_FUSION. Each robot uses120s and all original RGB frames. Real reset regression proves map ID alone is insufficient; use map ID plus initial-keyframe ID. No GT/LiDAR/IMU or semantic input. See docs/CUMULTI_RGBD_LONG_TRACKING_RESET_AUDIT.md and outputs/cumulti_v1/24_rgbd_long_v1/pre_fusion_decision.json.

robot1 baseline1000: 1200/1200 tracked, 0 frame change(s), gate PASS; robot1 fallback2000: 1136/1200 tracked, 4 frame change(s), gate FAIL; robot3 baseline1000: 1168/1200 tracked, 1 frame change(s), gate FAIL; robot3 fallback2000: 1137/1200 tracked, 2 frame change(s), gate FAIL.
Dual-robot fusion not run because stability gate failed. Diagnose tracking/depth/motion before a separately documented revised experiment; do not lower the frozen gate or join reset maps. No further download is required for this cohort.

## RGB-D loss-cause audit (2026-11-10)

Offline native feature/source audit confirms tracking loss, not missing data: roughly3s relocalization timeout then map restart/reset. All7 first-bad images retain many native keypoints and adjacent 2D geometric matches; native map/PnP inlier diagnostics are still missing, so the unique initial failure cause is unresolved. Robot3 baseline atRGB-D frame255: saved map associations117→22,1006 native features,269 with depth,237 independent adjacent F inliers. No SLAM/GT/fusion rerun or frozen result change. See docs/CUMULTI_RGBD_LOSS_ROOTCAUSE_AUDIT.md.


## 2026-11-10 — Full available camera RGB-D mapping

Full available camera-only experiment and segmented replay are validated. RGB durations are 999.789160747 s / 2098.463330423 s, so Robot1 cannot supply the Stage20 1972.5-second duration. The new demo spans both complete available streams with a 10:1 clock and an explicit shorter-recording end. This is not proof of uninterrupted tracking or a complete global map.

- robot1: 9864/9997 valid camera poses (0.986696), 6 Map Epochs, 1004.565 s native wall time, 3010.7734375 MiB native peak RSS.
- robot3: 20494/20977 valid camera poses (0.976975), 20 Map Epochs, 2110.655 s native wall time, 6957.453125 MiB native peak RSS.

Bridge diagnostic: NO_RELIABLE_RGBD_BRIDGE_ESTABLISHED; 0 single-pair geometric passes, 0 supported epoch-pair bridges. No unrelated reset gauges are glued. See docs/CUMULTI_STAGE24_FULL_RGBD_MAPPING.md, demo/stage24_full_rgbd/README.md and outputs/cumulti_v1/24_rgbd_full_v1/VALIDATION_REPORT.txt. All scientific and presentation limitations remain explicit.
