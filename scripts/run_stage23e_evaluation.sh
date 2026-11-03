#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=/home/cas/.venvs/fyp_gtsam/bin/python
"$PY" src/cumulti/stage23d/evaluate_stage23e.py
"$PY" src/cumulti/stage23d/validate_stage23e.py
"$PY" src/cumulti/stage23d/report_stage23e.py
