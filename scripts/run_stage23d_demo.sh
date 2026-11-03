#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=/home/cas/.venvs/fyp_gtsam/bin/python
if [ ! -d outputs/cumulti_v1/23d_policy_map_compare ]; then "$PY" src/cumulti/stage23d/common.py; fi
if [ ! -f outputs/cumulti_v1/23d_policy_map_compare/map_comparison_validation.json ]; then
  "$PY" src/demo/stage23d/prepare.py
fi
if [ "${1:-}" = --serve ]; then
  exec "$PY" -m http.server 8024 --bind 127.0.0.1 --directory demo
fi
echo 'Final-map assets ready. Use --serve; open /stage23d_policy_map_compare/ under port8024.'
