#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY="${RGBD_PYTHON:-/home/cas/.venvs/fyp_stage24/bin/python}"
bash scripts/build_rgbd_long.sh
"$PY" src/cumulti/rgbd_long/reset_regression.py
for robot in robot1 robot3; do
 "$PY" src/cumulti/rgbd_long/export.py "$robot"
 "$PY" src/cumulti/rgbd_long/track.py "$robot"
done
"$PY" src/cumulti/rgbd_long/decision.py
if [[ ! -f outputs/cumulti_v1/24_rgbd_long_v1/pre_fusion_decision.json ]]; then
 for robot in robot1 robot3; do
  "$PY" src/cumulti/rgbd_long/track.py "$robot" --config fallback2000
 done
 "$PY" src/cumulti/rgbd_long/decision.py --final
fi
"$PY" src/cumulti/rgbd_long/analyze.py
"$PY" src/cumulti/rgbd_long/forensics.py
"$PY" src/cumulti/rgbd_long/fusion_gate.py
"$PY" src/cumulti/rgbd_long/report.py
"$PY" src/cumulti/rgbd_long/validate.py
