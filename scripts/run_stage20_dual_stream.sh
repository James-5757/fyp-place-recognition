#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
exec /home/cas/.venvs/fyp_gtsam/bin/python src/cumulti/run_stage20_dual_stream.py "$@"
