#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
exec /home/cas/.venvs/fyp_gtsam/bin/python src/cumulti/run_stage13c_isam2_convergence.py "$@"
