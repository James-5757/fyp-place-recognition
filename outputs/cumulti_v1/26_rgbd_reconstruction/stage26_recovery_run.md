# Stage26 bounded recovery run

The Robot1 RGB and depth archives have arrived. All four camera ZIPs for Robot1 and Robot3 are available on the server. A selective export run is active in `/home/cas/fyp_stage26`. It expands only the SQLite members to scratch and publishes their final names after ZIP decompression succeeds. Original archives are read-only.

The frozen run requires 106 RGB-D frame indices for Robot1 and 268 for Robot3, including the 13 Stage24E cases, 11 failure controls and four historical non-adjacent pairs. The extra pairs and fit/holdout memberships come from the frozen Stage24D appearance ranking. The first recovered receipt verifies 106 Robot1 RGB observations against the original message ID, bag and header timestamps, topic and frame. Gate 1 requires all 748 individual assets and hash checks before registration starts.

The bounded runner compares fit-only RGB/PnP or identity-initialized ICP with mutual FPFH/RANSAC plus fit-only ICP, then measures both directions on the frozen validation frames. Pixel-disjoint observations from reused frames are flagged. A candidate must also pass independent repeats, temporal span and SE(3) agreement to qualify as a bridge. This run stops after the bounded comparison. It does not admit graph edges or launch full reconstruction.

Reproduce from the Stage26 checkout using `bash scripts/run_stage26_bounded.sh` with the existing Stage24 Python environment and Open3D 0.19.0. The non-adjacent plan can be rebuilt with `python src/cumulti/rgbd_full/stage26_plan.py`. Recovered images, scratch SQLite bags and submaps are server artifacts.
