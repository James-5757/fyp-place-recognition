#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
# Phase0 inventory/retrospective linearization ONLY, not a selector experiment.
/home/cas/.venvs/fyp_gtsam/bin/python src/cumulti/stage23/preflight.py
exec /home/cas/.venvs/fyp_gtsam/bin/python src/cumulti/stage23/finalize_plan.py
