#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PY="${STAGE24_CAMERA_PYTHON:-/home/cas/.venvs/fyp_gtsam/bin/python}"
SEM="${STAGE24_SEMANTIC_PYTHON:-/home/cas/.venvs/fyp_stage24/bin/python}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
A="${STAGE24_OUTPUT_ROOT:-$PWD/outputs/cumulti_v1}/24a_rgb_semantic"
B="${STAGE24_OUTPUT_ROOT:-$PWD/outputs/cumulti_v1}/24b_vision_feasibility"
# Complete phases are immutable and reused; science is not silently re-executed.
test -f "$A/historical_input_hashes.json" || "$PY" src/cumulti/stage24/protect.py freeze
test -f "$B/camera_data_manifest.json" || "$PY" src/cumulti/stage24/export_camera.py
test -f "$B/depth_camera_audit.json" || "$PY" src/cumulti/stage24/audit_depth.py
test -f "$B/bin/stage24_mono_pilot" || bash scripts/build_stage24_camera_pilot.sh
test -f "$B/runtime_memory_pilot.json" || "$PY" src/cumulti/stage24/vision_pilot.py
test -f "$A/segmentation_runtime.json" || "$SEM" src/cumulti/stage24/segment_images.py
"$PY" src/cumulti/stage24/make_contact_sheets.py
"$PY" src/cumulti/stage24/audit_and_package.py
"$PY" src/cumulti/stage24/protect.py verify
"$PY" src/cumulti/stage24/validate.py
"$PY" src/cumulti/stage24/report.py
# Run the separate browser validation before committing. Stop after24A/B.
