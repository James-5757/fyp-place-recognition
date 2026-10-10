#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
exec /home/cas/.venvs/fyp_gtsam/bin/python src/cumulti/run_stage14_final_consistency.py "$@"
