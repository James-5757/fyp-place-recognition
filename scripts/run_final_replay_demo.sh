#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/cas/fyp_place_recognition
cd "$ROOT"
required=(
  outputs/cumulti_v1/09_sc_gicp_integration/rank1_gicp_results.csv
  outputs/cumulti_v1/10_2_pgo_policy_selection/rank1_sanitized_edges.csv
  outputs/cumulti_v1/10_3_solver_audit/rank1_gtsam_trajectory.csv
  outputs/cumulti_v1/10_3_solver_audit/rank1_gtsam_merged_map.ply
  outputs/cumulti_v1/10_3_solver_audit/pre_gt_gtsam_decision.json
  outputs/cumulti_v1/10_3_solver_audit/gtsam_solver_results.csv
)
for path in "${required[@]}"; do [[ -f "$path" ]] || { echo "Missing frozen artifact: $path" >&2; exit 1; }; done
if [[ ! -f demo/final_replay/data/replay.json ]]; then
  source /home/cas/miniconda3/etc/profile.d/conda.sh
  conda activate fyp_slam
  python src/demo/prepare_final_replay_assets.py
fi
echo 'CU-Multi Offline Replay — Centralized Backend'
echo 'Open: http://localhost:8000 (or use SSH port forwarding)'
cd demo/final_replay
exec python -m http.server 8000
