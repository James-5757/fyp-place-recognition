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
