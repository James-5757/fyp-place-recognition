#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../demo"
exec "${PYTHON:-/home/cas/.venvs/fyp_gtsam/bin/python}" -m http.server "${PORT:-8021}" --bind "${BIND:-127.0.0.1}"
