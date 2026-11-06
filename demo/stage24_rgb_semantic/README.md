# Stage24A — calibration-gated RGB / semantic mapping presentation

Serve the **parent demo directory** over HTTP, e.g. from repository root:

```bash
python -m http.server 8021 --bind 127.0.0.1 --directory demo
```

Open `http://127.0.0.1:8021/stage24_rgb_semantic/`. Do not open index.html directly.
Orbit/pan/zoom controls bind directly to the canvas. All displayed metric XYZ
buffers are SHA-identical copies of the frozen Stage23D P5 incremental display
assets. The original580 selected scans, frame10/point16/voxel0.5 protocol and
P5 poses are preserved. The viewer displays capped25000 points per robot;
it does not claim complete-resolution browser geometry.

Modes: robot provenance, RGB attributes, semantic categories. **RGB and semantic
3D projection are BLOCKED**: locally inspected calibration does not establish
the complete camera-link-to-color-optical mounting transform. Their map modes
show unknown gray, never borrowed robot colors or fabricated semantic colors.
The12 real-camera image/SegFormer prediction pairs are explicitly **2D only**.
Raw ADE IDs and softmax confidence arrays remain server-side with SHA256 in the
segmentation report; confidence below0.5 and unrepresented classes map to unknown.

This is LiDAR geometry with a calibration-gated camera-attribute adapter,
not a vision-only or live/distributed SLAM map. No mesh, GT alignment, altered
vertical coordinate, new factor, backend optimization or scientific retuning.
See `docs/CUMULTI_STAGE24A_RGB_SEMANTIC_MAP.md` for the blocker and next validation.
