# Stage24 dual-direction causal RGB-D replay

The Stage22A event order is applied to the complete Stage24 camera recordings on independently normalized clocks. At each scheduled query the native RGB BoW proposer searches the opposite stream's arrived descriptors, then publishes its own descriptor. Eligible frames have a valid recorded online pose and a paired depth image. Each side contributes descriptors at approximately 1 Hz and queries at approximately 0.2 Hz. The replay waits 25 ms after the RGB bag timestamp so the original pairing window is available. Events sort by normalized availability time, robot and frame index. Replay time models data availability rather than execution or network latency.

The original RGB-D geometry thresholds are shared with the Stage24 offline baseline. The reverse transform is inverted into the Robot1-from-Robot3 convention. Exact duplicate accepted frame pairs count once toward bridge support. The first qualifying prefix per epoch pair freezes the transform. A bridge needs three distinct frames per side, the existing temporal spread and consistent metric transforms. Later evidence cannot move an earlier bridge into the past.

| Query side | Arrived descriptors | Queries | Tested candidates |
|---|---:|---:|---:|
| Robot1 | 882 | 184 | 550 |
| Robot3 | 1907 | 390 | 1167 |

All 1717 proposals passed the arrival-order audit. The verifier rejected 1709 for insufficient mutual RGB matches and 8 for insufficient measured depth matches. There were zero verified single pairs and zero epoch bridges.

The Global Merged Map panel displays cumulative checks and the final bridge status. If a future run verifies an epoch pair, the panel can show its two RGB-colored point clouds in the Robot1 epoch gauge, starting at the bridge's admission time and clipped to the selected frame time. Browser checks covered the actual no-bridge result and admission/rewind behavior with a test-only transform. The real run produced no merged point cloud. Local map growth continues within each recorded epoch. Continuous reconstruction across native Atlas components still requires final optimized poses and reprojection of earlier measurements.

Run `bash scripts/build_rgbd_causal.sh`, then `/home/cas/.venvs/fyp_stage24/bin/python src/cumulti/rgbd_full/causal.py` with the complete Stage24 native tracking and depth assets. Results are in `outputs/cumulti_v1/24_rgbd_full_v1/causal_rgbd/`. Serve the full demo directory over HTTP. The demo uses the same Stage24 local map assets.
