# CU-Multi Stage24A — calibration-gated RGB / semantic mapping

STAGE24A_RGB_MAP: **BLOCKED**. STAGE24A_SEMANTIC_MAP: **BLOCKED**.
Implementation and2D segmentation are available, but no real RGB/semantic3D projection is claimed.

## Inputs and preserved science
Frozen Stage22B P5 incremental poses are the demonstration reference. The new viewer reuses byte-identical capped P5 point buffers from Stage23D, retaining580 original source scans (frame10/point16/voxel0.5m/range1–60m), metric XYZ and optimized poses. The trajectory SHA is `7c022da3db16e2bf09d0d92493ebb37683e986684fcc7b38eed0b9f4c406fdd0`. No P0/P5/P6 retuning, Stage18–23E artifact change, GT alignment, new factor or mesh. These are separate recordings and a centralized LiDAR-based prototype; geometry is not camera-only or geographic ENU.

## Exact availability / coordinate audit
See `24a_rgb_semantic/data_availability_report.json` for actual archive paths/bytes/topics and explicit decode scopes. R1/R3 RGB ZIP members were read through EOF/CRC and real frames decoded. R2 RGB/depth are inventoried, not newly fully decoded. R1 depth-camera PointCloud2 was decoded; R3 depth is absent. No native depth Image topic is present in the inspected R1/R2 depth metadata. Depth scale cannot be silently replaced by LiDAR.

URDF joint origins are **T_parent_child**, not T_child_parent. Chain composition/inversion is recorded in calibration_audit.json. URDF reaches camera_link but not camera_color_optical_frame; all archived URDF/xacro optical mentions are audited. Mount geometry visual rotations are not measurement calibration. The missing optical mounting translation/rotation is not supplied by depth_to_color, which links two camera optical sensors. No standard-axis guess is promoted to measured sensor extrinsics. Image optical convention is x right/y down/z forward. Intrinsics use native1280×800, actual plumb_bob coefficients, and scaled K only for resized640×400 pilot images.

`rgb_projection_validation.json` explicitly records real_projection=False. The reusable projector rejects missing calibration and bad sync (>50ms bag difference), removes negative/out-of-FoV points, applies distortion and exact-pixel nearest-depth z-buffer. Synthetic near/far/occlusion/direction/NaN tests pass. This does **not** validate real-world occlusion, hardware exposure timing or calibration. No observations are fused across scans until calibration is resolved; original voxel representatives are kept, so duplicate RGB/semantic observations are neither averaged nor invented.

## Actual semantic pilot
Pretrained [SegFormer B0 ADE20K](https://huggingface.co/nvidia/segformer-b0-finetuned-ade-512-512), pinned revision489d5cd81a0b59fab9b7ea758d3548ebe99677da,512×512 input, CPU2threads, confidence threshold0.5. Exact150-class mapping is frozen in semantic_model_policy.json: building/house/skyscraper→building; road/sidewalk/path/runway→road/asphalt bucket; grass→grass; named plant/tree groups→vegetation; named vehicles→vehicle; earth/sand/land/etc→dirt/other; all unrepresented classes and low-confidence pixels→unknown. The road bucket does not establish asphalt material. Raw ADE IDs/confidence arrays are preserved server-side with hashes. No LiDAR labels or semantic GT used.

Twelve deterministic real-image samples ran; inference seconds median/p95/max: 0.434138/0.633044/0.648337. Native overexposure, magenta tint and motion blur are visible; vegetation boundaries/fine objects and road/grass boundaries can fail. The all-unknown overexposed opening frame is retained. Qualitative image inspection is not IoU/mIoU accuracy; no semantic GT was available or accessed. semantic_class_distribution.csv is explicitly **image pixels only**, never3D coverage.

## Demo and validation
`demo/stage24_rgb_semantic/` offers robot/RGB/semantic modes with persistent camera controls. Robot mode blue/orange shows provenance. RGB/semantic modes show100%unknown gray because observed3D vertices=0. This deliberately does not advertise a colored map as complete. Twelve authentic image/prediction pairs are displayed separately as2D evidence. Region coverage CSV records zero colored/labeled points and explicit missing regional point counts, not fake observed zeros as calibrated accuracy. Separate browser validation/screenshots and independent artifact audit establish interface/data behavior only.

Serve parent demo over HTTP (`python -m http.server 8021 --bind 127.0.0.1 --directory demo`) then `/stage24_rgb_semantic/`. See README for exact command. Do not double-click index.html. Runtime/map manifests preserve deterministic source SHA/bytes; raw bags, temporary DBs, weights and oversized sequences are not committed. SegFormer has [custom upstream terms](https://github.com/NVlabs/SegFormer/blob/master/LICENSE), not an assumed MIT/Apache license.

## Minimum separately authorized Stage24C
Obtain measured, frame-identified camera_link↔color/depth optical transforms (recorded static TF or matching serial/driver calibration), verify exposure/header/bag synchronization, then validate projection overlays, FoV/occlusions/depth ordering and duplicate-observation handling on a small fixed cohort. Only then project RGB/semantic labels into frozen P5 geometry and report actual3D coverage. Do not use GT or hand-tuned frame rotations to make overlays look right. No Stage24C started.
