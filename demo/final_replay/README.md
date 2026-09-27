# CU-Multi Final Replay Demo

Offline, deterministic visualization of frozen Rank1 Scan Context + GICP + GTSAM artifacts. Run `bash scripts/run_final_replay_demo.sh` on the server and open the printed local URL. No internet, GT display, live sensor input, networking, or distributed PGO is used.

The replay has five explicit scenes: separate local trajectories; Scan Context candidate; GICP acceptance; global GTSAM pose graph; and merged map. Trajectories progressively render from replay-relative time zero, use current-pose markers and short tails, and do not overlay unaligned local frames. Loop edges show only on their frozen event by default; the checkbox reveals all accepted edges for debugging.

The map view retains a BEV display, supports before/after collaboration switching, colors robot-attributed presentation points, and includes an overlap zoom-in. Map point attribution is a display-only nearest-optimized-trajectory heuristic; it does not alter the frozen Rank1 merged PLY. LiDAR thumbnails are deliberately compact frozen samples, not raw streaming clouds.
