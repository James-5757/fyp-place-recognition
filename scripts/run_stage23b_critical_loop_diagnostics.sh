#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python=/home/cas/.venvs/fyp_gtsam/bin/python
"$python" -u src/cumulti/stage23/diagnostic_execute.py
if [ ! -f outputs/cumulti_v1/23_information_aware/stage23b_offline_audit.json ]; then
  "$python" -u src/cumulti/stage23/diagnostic_evaluate.py
fi
exec "$python" -u src/cumulti/stage23/diagnostic_finalize.py
