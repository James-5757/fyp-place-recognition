#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY="${RGBD_PYTHON:-/home/cas/.venvs/fyp_stage24/bin/python}"
bash scripts/build_rgbd_demo.sh
for robot in robot1 robot3; do
 "$PY" src/cumulti/rgbd_demo/export.py "$robot"
 "$PY" src/cumulti/rgbd_demo/track.py "$robot"
done
if [[ ! -f outputs/cumulti_v1/24_rgbd_demo_v1/map_manifest.json ]]; then
 "$PY" src/cumulti/rgbd_demo/package.py
fi
"$PY" src/cumulti/rgbd_demo/validate.py
