#!/usr/bin/env bash
set -euo pipefail

stage12a_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stage12a_python="${STAGE12A_PYTHON:-/home/cas/miniconda3/envs/fyp_slam/bin/python}"
exec "$stage12a_python" "$stage12a_repo/src/cumulti/run_stage12a_trajectory_integrity.py"
