#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "${STAGE5B_PYTHON:-/home/cas/.venvs/fyp_stage24/bin/python}" -u src/cumulti/run_stage5b_independent_visual_retrieval.py --work "${STAGE5B_WORK:-/home/cas/fyp_stage5b/work/full_rankings}" "$@"
