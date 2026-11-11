#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python="/home/cas/.venvs/fyp_stage24/bin/python"
out=outputs/cumulti_v1/24d_rgbd_pose_graph
"$python" src/cumulti/rgbd_full/stage24d_retrieve.py
"$python" src/cumulti/rgbd_full/atlas_geometry.py --candidates "$out/candidates.csv" --output "$out"
"$python" src/cumulti/rgbd_full/stage24d_graph.py
