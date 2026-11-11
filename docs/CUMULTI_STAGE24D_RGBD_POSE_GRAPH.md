# Stage24D verified RGB-D epoch graph

Stage24D uses the saved Stage24 online RGB-D poses and depth measurements. The original assets and measurements at commit `45960c9` remain available. The default demo opens Verified Atlas View. Raw Epoch View and the Stage24 baseline are available from its navigation.

## Baseline and retrieval

The baseline browser check reproduced 7,861 visible Robot3 points at 62 s and 9,749 at 72 s in the same component across original epochs 0→1. It also checked the bridge-gated merged panel, the switch to an unrelated component, and removal of future observations on rewind.

The historical index contains 3,648 valid, depth-paired keyframes sampled at least 0.75 seconds apart per epoch. Normalized RGB appearance and gradient descriptors score every unordered epoch pair across both robots. Up to 12 candidates per epoch pair are selected, excluding same-robot frame pairs closer than 2 seconds and spacing selected endpoints at least 1 second apart on each side. This examined all 325 epoch pairs, with candidates in 321 and 2,231 candidates verified. The other four pairs had no eligible endpoints under the temporal rule. Thirty candidates passed frame geometry. None produced an additional accepted epoch bridge.

## Verification and graph

The shared verifier keeps mutual SIFT ratio 0.75, at least 30 PnP inliers and 50% PnP inlier fraction, a 2.5 px RANSAC threshold, positive depth, spatial support and post-fit 90th-percentile reprojection at most 3 px. Independent depth samples must pass the 12 cm nearest-neighbor check in both directions with at least 300 points and 35% support per direction. At least three distinct frame indices per side and the original temporal-span and SE(3) agreement gates are required to accept an epoch bridge. Previously verified measurements remain available and duplicate frame pairs count once.

The persistent `pose_graph.json` stores 26 original epoch nodes, measured edges with supporting frames and residuals, component membership, and `T_atlas_from_epoch` for each node. A maximum-support spanning forest estimates transforms within each connected component. Non-tree closures, when present, are audited against the estimate. Tree residuals are zero by construction and do not measure registration accuracy. The replay reprojects measured depth into these component gauges and reveals points by their original timestamps. The merged component panel becomes available after all its selected tree links are available. This is offline replay using the final epoch graph.

| Metric | Stage24 baseline | Stage24D |
|---|---:|---:|
| Original epochs (Robot1 / Robot3) | 6 / 20 | 6 / 20 |
| Local components (Robot1 / Robot3) | 5 / 18 | 5 / 18 |
| Joint connected components | 22 | 22 |
| Accepted bridges (intra / inter) | 3 / 1 | 3 / 1 |
| Reprojected map points (Robot1 / Robot3) | 662,962 / 1,633,246 | 662,979 / 1,633,179 |
| Shared component points | 68,100 | 68,050 |

The four bridges have 8, 4, 8 and 3 supporting frame pairs respectively for R1 0↔2, R1 0↔R3 0, R3 0↔1 and R3 16↔19. Maximum support-to-anchor disagreement across the four bridges is below 0.18 m and 0.5°. The largest per-frame inlier depth median among supporting pairs is below 0.07 m. These measurements support registration consistency, with no ground-truth precision estimate.

The shared component becomes visible at 72.254 s. At 12 cm, directional whole-map overlap is 29.80% from Robot1 to Robot3 and 37.53% in the reverse direction. Median distance among overlapping sampled points is 6.37 cm and 6.09 cm respectively. The shared component contains Robot1 epochs 0 and 2 and Robot3 epochs 0 and 1. The other 21 components retain independent coordinate frames.

## Browser evidence

The full browser suite passed 69 seek cases with actual RGB previews and no browser errors. The focused graph test confirmed the default mode, Raw Epoch navigation, 22 listed components, four timed bridge events, the merge gate at 72.254 s, and clearing future observations on rewind. Robot3 stayed in component 0 while crossing original epoch 0→1, with 7,758 visible points at 62 s and 9,618 at 72 s after Stage24D reprojection. The baseline counts are preserved in the baseline replay. See `baseline_browser.json` and `browser_validation/` in the results directory.

Run `bash scripts/run_stage24d_rgbd.sh` with the existing Stage24 RGB-D and bridge evidence. Generated artifacts are in `outputs/cumulti_v1/24d_rgbd_pose_graph/`, with browser assets under `demo/stage24_full_rgbd/stage24d/`.
