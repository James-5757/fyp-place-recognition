#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python="/home/cas/.venvs/fyp_stage24/bin/python"
out=outputs/cumulti_v1/24e_rgbd_feasibility
"$python" src/cumulti/rgbd_full/stage24e_audit.py
"$python" src/cumulti/rgbd_full/atlas_geometry.py --diagnostics --candidates "$out/boundary_candidates.csv" --output "$out"
"$python" src/cumulti/rgbd_full/stage24e_boundaries.py
