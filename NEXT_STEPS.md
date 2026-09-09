# Next steps

Move from KITTI simulated multi-robot splits to CU-Multi Main Campus robot1/robot2. Keep raw CU-Multi separate; inspect actual topics, timestamps, pose/heading and calibration before assumptions; build lidar/, rgb/, timestamps.csv, poses.csv and sync_index.csv; validate only 100 keyframes per robot before full processing or Scan Context.

Preserve SC + Cross-Max as the retrieval baseline while resolving the metric discrepancy in RESULTS.md. Measure OpenCLIP viewpoint sensitivity explicitly and compare single view, mean pooling, Cross-Max and controlled multi-view aggregation under a fixed candidate protocol. Ground truth remains offline only. Record dataset version, selected frames, synchronization thresholds, pose frame, calibration and commands for every experiment.
