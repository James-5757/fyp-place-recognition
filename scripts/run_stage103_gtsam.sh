#!/usr/bin/env bash
set -euo pipefail
source /home/cas/.venvs/fyp_gtsam/bin/activate
cd /home/cas/fyp_place_recognition
exec python src/cumulti/run_stage103_gtsam_solver_audit.py "$@"
