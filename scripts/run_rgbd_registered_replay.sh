#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python="/home/cas/.venvs/fyp_stage24/bin/python"
"$python" src/cumulti/rgbd_full/atlas_geometry.py --dense
"$python" src/cumulti/rgbd_full/atlas_geometry.py --history
"$python" src/cumulti/rgbd_full/atlas_replay.py --registered
