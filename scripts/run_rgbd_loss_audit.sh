#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
bash scripts/build_rgbd_loss_audit.sh
/home/cas/.venvs/fyp_stage24/bin/python src/cumulti/rgbd_loss_audit/run.py
/home/cas/.venvs/fyp_stage24/bin/python src/cumulti/rgbd_loss_audit/report.py
