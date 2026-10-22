#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /home/cas/.venvs/fyp_gtsam/bin/python -u src/cumulti/stage22b/run.py "$@"
