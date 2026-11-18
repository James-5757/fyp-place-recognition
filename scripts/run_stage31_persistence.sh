#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${STAGE31_PYTHON:-/home/cas/.venvs/fyp_stage24/bin/python}"
scratch="${STAGE31_SCRATCH:-$PWD/work}"
packages="${STAGE31_OPEN3D_RUNTIME:-$PWD/open3d-runtime}"
mkdir -p "$scratch"
exec 9>"$scratch/stage31.lock"
flock -n 9
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ -d "$packages" ]]; then
    export PYTHONPATH="$packages${PYTHONPATH:+:$PYTHONPATH}"
fi
"$python" src/cumulti/rgbd_full/stage31_persistence.py --scratch "$scratch"
