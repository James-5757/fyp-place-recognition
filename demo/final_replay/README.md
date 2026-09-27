# CU-Multi Final Replay Demo

Offline, deterministic visualization of frozen Rank1 Scan Context + GICP + GTSAM artifacts. Run `bash scripts/run_final_replay_demo.sh` on the server and open the printed local URL. No internet, GT display, live sensor input, networking, or distributed PGO is used.

The replay has six scenes: Independent Local Mapping; Scan Context Candidate Retrieval; GICP Geometric Verification; Initial Common Frame; GTSAM Pose Graph Optimization; and Merged Map. Scenes 1–3 use true frozen, non-GT local odometry. Scene 4 uses the frozen single-loop common-frame initialization; Scenes 5–6 use the frozen Rank1 GTSAM result. Trajectories progressively render from replay-relative time zero, use current-pose markers and short tails, and do not overlay unaligned local frames.

The map view uses genuinely different frozen geometries: pre-PGO map points built from the single frozen initialization and post-PGO points built from the Rank1 GTSAM trajectory. Robot provenance is retained. The overlap zoom derives from frozen loop endpoints. GICP thumbnails apply the exact frozen `T_query_from_candidate` SE(3) transform to candidate points; no pixel-shift alignment is used. No GT, live sensor input, networking, or distributed PGO is used.
