#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /home/cas/.venvs/fyp_gtsam/bin/python src/cumulti/run_stage16_measurement_information.py "$@"
