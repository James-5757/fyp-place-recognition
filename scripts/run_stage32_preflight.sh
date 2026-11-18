#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${STAGE32_PYTHON:-/home/cas/.venvs/fyp_stage24/bin/python}"
scratch="${STAGE32_SCRATCH:-$PWD/work}"
mkdir -p "$scratch/stage32"
exec 9>"$scratch/stage32/checkpoint.lock"
flock -n 9
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$python" src/cumulti/rgbd_full/stage32_inventory.py     --output outputs/cumulti_v1/32_native_tracking_forensics/stage32_dependency_search.json
"$python" src/cumulti/rgbd_full/stage32_preflight.py --scratch "$scratch"
