#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
exec /home/cas/.venvs/fyp_gtsam/bin/python src/cumulti/run_stage15_informative_loop_selection.py "$@"
