#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../demo/progressive_multi_robot"
exec /home/cas/.venvs/fyp_gtsam/bin/python -m http.server "${PORT:-8018}" --bind "${BIND:-127.0.0.1}"
