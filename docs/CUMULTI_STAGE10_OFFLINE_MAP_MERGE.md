# CU-Multi Stage 10 Offline Map Merge

Stage 10 runs offline only with 2,000 Robot1 and 4,180 Robot3 non-GT EKF-aligned
keyframes, 6,178 sequential odometry factors, and the frozen 139 Stage-9.1
sanitized candidate loop constraints. The highest online GICP-quality loop
(Robot1 keyframe 283, Robot3 keyframe 252, quality 0.749343) initializes the
single-loop baseline. GT is loaded only after graph construction for one joint
rigid alignment and evaluation.

The frozen solver reached its fixed 25-evaluation cap for both PGO conditions.
Non-robust PGO did not reduce its reported objective. Robust PGO reduced the
objective from 422472.42 to 5280.42, and improves the GT-free map NN median from
11.960 m (single-loop) to 2.579 m, but its joint offline ATE worsens from
68.432 m to 75.298 m. Consequently Stage 10 is **FAIL**, not a validated mapping
success: the predeclared convergence pass condition is not met and robust PGO
does not improve every primary consistency measure.

This result must not be repaired by GT-based tuning. The next experiment, if
declared, must freeze a revised solver-convergence/robustness protocol before
reading these GT metrics. No live demo is authorized by this offline failure.
