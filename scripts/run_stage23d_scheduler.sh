#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=/home/cas/.venvs/fyp_gtsam/bin/python
if [ ! -d outputs/cumulti_v1/23d_budget_scheduler ]; then "$PY" src/cumulti/stage23d/common.py; fi
"$PY" src/cumulti/stage23d/test_scheduler.py
if [ ! -f outputs/cumulti_v1/23d_budget_scheduler/scheduler_trace_freeze.json ]; then
  "$PY" -u src/cumulti/stage23d/scheduler.py
fi
"$PY" src/cumulti/stage23d/validate.py
"$PY" src/cumulti/stage23d/report.py
