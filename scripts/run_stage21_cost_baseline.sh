#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
# Set CUMULTI_STAGE21_OUTPUT_ROOT to a NEW directory for independent reproduction.
# The fixed workload/algorithms are never tuned; existing outputs are refused.
PY=/home/cas/fyp_robot13_geometry_20260915/.venv/bin/python
"$PY" src/cumulti/prepare_stage21_policy.py
"$PY" src/cumulti/run_stage21_payloads.py
"$PY" src/cumulti/run_stage21_frontend.py
"$PY" src/cumulti/run_stage21_protocols.py
"$PY" src/cumulti/validate_stage21_consumed_fields.py
"$PY" src/cumulti/summarize_stage21_costs.py
"$PY" src/cumulti/audit_stage21_costs.py
"$PY" src/cumulti/finalize_stage21_costs.py
