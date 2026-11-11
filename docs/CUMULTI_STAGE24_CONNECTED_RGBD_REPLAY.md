# Stage24 connected RGB-D replay

Open `http://127.0.0.1:8026/stage24_full_rgbd/?atlas=1` with the generated presentation assets installed. The replay uses the original full camera recordings and the original RGB-D projection. Within each connected component, points accumulate by source timestamp. Returning to a previously seen component restores its accumulated map. The component selector keeps unrelated coordinate systems separate.

The full native Atlas export was attempted. Robot1 stopped during shutdown and Robot3 stopped before the full stream finished. This replay instead registers the previously saved online camera poses with measured RGB-D frame pairs. It is an offline reconstruction and does not contain final bundle-adjusted native Atlas poses.

The cross-robot experiment retained the 1,717 candidates from the dual-direction Stage22A-style replay. SIFT matches and metric PnP proposed camera transforms. Bidirectional nearest-neighbor checks independently verified their agreement with sampled measured depth clouds. Thirty-six candidate pairs passed. Five supported the first consistent Robot1–Robot3 component bridge at normalized time 16.134 s. The full-resolution merged point cloud for that component has 68,100 RGB-colored points.

Within each robot, the search evaluated 3,094 frame pairs across neighboring and earlier epochs. Seventy-one pairs passed the frame geometry checks. Consistent support from distinct frames on both sides admitted three component links. Robot1 links epochs 0 and 2, reducing six epochs to five connected components. Robot3 links 0 with 1 and 16 with 19, reducing 20 epochs to 18 components. The cross-robot merge includes Robot1 epochs 0 and 2 and Robot3 epochs 0 and 1. The remaining components retain their own coordinate systems, and full-recording continuity is unresolved.

Browser validation confirmed that the Robot3 map remained in the same component while crossing the original epoch 0→1 boundary, and its visible point count grew from 7,861 at 62 s to 9,749 at 72 s. The actual merged map rendered at the end of the replay and disappeared on rewind before the bridge.

Run `bash scripts/run_rgbd_registered_replay.sh` on the server with the completed Stage24 RGB-D and causal candidate assets. Evidence is written to `outputs/cumulti_v1/24_rgbd_atlas_v1/`, including the PLY and its checksum in `summary.json`. Browser assets are generated under `demo/stage24_full_rgbd/atlas/`. LiDAR and ground truth are excluded from registration.
