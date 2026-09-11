# Experiment log

Reconstructed from source names, output directories and README_zh.md. Historical KITTI and CU-Multi work are deliberately separate.

1. Formal Scan Context baseline: `scan_context.py` and `retrieval_baseline*.py`; `outputs/baseline_*` and `formal_split*`; establishes the geometry anchor.
2. Candidate recall/case analysis: `analyze_sc_candidate_recall.py`, `analyze_retrieval_cases.py` and visualization scripts; documents candidate ceilings and misses.
3. BEV/yaw validation: `generate_bev.py`, `validate_sc_yaw.py` and `full_bev_reranking.py`; BEV-only reranking regressed against Scan Context.
4. RGB and temporal mean pooling: `full_rgb_reranking.py` and `full_temporal_rgb_reranking.py`; mean pooling was worse than single-frame RGB.
5. Temporal Cross-Max/filtering: `full_temporal_cross_frame_reranking.py` and `top20_crossmax_visual_filtering.py`; Cross-Max outperformed mean pooling and enabled selective filtering.
6. Canonical-v2/VLM: `src/canonical_v2` and selective-VLM scripts; VLM override caused net regressions.
7. CU-Multi acquisition: downloaded Main Campus robot1/robot2 raw archives and calibration to `/home/cas/CU-Multi/raw/`, kept outside Git and unchanged.
8. CU-Multi Stage 1 adapter validation: `src/cumulti/prepare_stage1_validation.py`; input was the ROS2 SQLite bags and UTM GT CSV. It temporarily expanded one `.db3` at a time, produced exactly 100 float32 XYZI clouds and 100 nearest RGB PNGs per robot, and wrote timestamps, poses and sync indices to `/home/cas/CU-Multi/processed_v1/`. Mean absolute RGB/GT synchronization errors were 24.884/10.928 ms (robot1) and 25.532/10.831 ms (robot2). Artifacts and caveats: `docs/CUMULTI_STAGE1_VALIDATION.md`. No retrieval algorithm was run and GT was used only offline.
9. CU-Multi Stage 2 frozen Scan Context baseline: fixed 2 Hz LiDAR grids (robot1 2,000; robot2 2,230), offline `d_xy < 5 m` labels, and Robot1→Robot2 R@1 0.994756 over 1,907 valid queries. The baseline remains frozen; no visual model or GT-based candidate selection was used.
10. CU-Multi Stage 2.5 diagnostic: `src/cumulti/run_stage25_diagnostics.py`; read existing rankings only to diagnose the high baseline. Common-start stationarity contributes but does not explain the near-saturation; moving-only R@1 remains 0.993835. All ten Rank-1 failures were analysed by offline GT distance/yaw and visual panels. See `docs/CUMULTI_STAGE2_5_DIAGNOSTICS.md`.
11. CU-Multi Stage 3 pair selection: `src/cumulti/run_stage3_pair_selection.py`; downloaded only the two robot3/4 UTM GT CSVs and evaluated all 12 directions across six unordered pairs using timestamp-based 2 Hz samples and offline `d_xy < 5 m` labels. Recommended robot1–robot3 as hard-but-usable; no robot3/4 sensor archive or retrieval algorithm was used. See `docs/CUMULTI_STAGE3_PAIR_SELECTION.md`.
