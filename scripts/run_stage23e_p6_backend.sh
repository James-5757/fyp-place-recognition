#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=/home/cas/.venvs/fyp_gtsam/bin/python
SRC=src/cumulti/stage23d
if [ ! -d outputs/cumulti_v1/23e_p6_graphs ]; then "$PY" "$SRC/stage23e_common.py"; fi
for policy in P6_D0 P6_D1 P6_D2 P6_D3; do "$PY" -u "$SRC/run_stage23e_backend.py" "$policy"; done
"$PY" "$SRC/freeze_stage23e_graphs.py"
