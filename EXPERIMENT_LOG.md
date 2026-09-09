# Experiment log

Reconstructed from source names, output directories and README_zh.md.

1. Formal Scan Context baseline: scan_context.py and retrieval_baseline*.py; outputs/baseline_* and formal_split*; establishes the geometry anchor.
2. Candidate recall/case analysis: analyze_sc_candidate_recall.py, analyze_retrieval_cases.py and visualization scripts; documents candidate ceilings and misses.
3. BEV/yaw validation: generate_bev.py, validate_sc_yaw.py and full_bev_reranking.py; BEV-only reranking regressed against Scan Context.
4. RGB and temporal mean pooling: full_rgb_reranking.py and full_temporal_rgb_reranking.py; mean pooling was worse than single-frame RGB.
5. Temporal Cross-Max/filtering: full_temporal_cross_frame_reranking.py and top20_crossmax_visual_filtering.py; Cross-Max outperformed mean pooling and enabled selective filtering.
6. Canonical-v2/VLM: src/canonical_v2 and selective-VLM scripts; VLM override caused net regressions.
7. CU-Multi: Main Campus robot1/robot2 raw transfer and 100-frame adapter validation are separate from historical KITTI outputs; no Scan Context run during initial validation.
