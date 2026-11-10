#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PY=/home/cas/.venvs/fyp_stage24/bin/python
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
OUT="$PWD/outputs/cumulti_v1/24_rgbd_full_v1"
mkdir -p "$OUT"
exec 9>"$OUT/orchestration.lock"
flock -n 9 || { echo 'A full-duration owner already exists'; exit 1; }
# Independent audit roles may already be running; per-role locks prevent duplicates.
for robot in robot1 robot3; do
 for kind in RGB DEPTH; do
  "$PY" src/cumulti/rgbd_full/prepare.py "$robot" "$kind" >"$OUT/${robot}_${kind}_prepare.log" 2>&1 &
  "$PY" src/cumulti/rgbd_full/calibration_audit.py "$robot" "$kind" >"$OUT/${robot}_${kind}_calibration.log" 2>&1 &
 done
done
"$PY" src/cumulti/rgbd_full/audit.py --wait >"$OUT/audit_stdout.log" 2>&1
bash scripts/build_rgbd_full.sh >"$OUT/build.log" 2>&1
for robot in robot1 robot3; do
 "$PY" src/cumulti/rgbd_full/depth_parallel.py "$robot" >"$OUT/${robot}_projection.log" 2>&1 &
 "$PY" src/cumulti/rgbd_full/track.py "$robot" >"$OUT/${robot}_tracking.log" 2>&1 &
done
wait
"$PY" src/cumulti/rgbd_full/finalize.py >"$OUT/finalization_stdout.log" 2>&1
echo 'FULL_TRACKING_AND_MAPPING_COMPLETE; browser validation is a separate qualification.'
