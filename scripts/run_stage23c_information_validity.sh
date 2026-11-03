#!/usr/bin/env bash
set -euo pipefail
cd /home/cas/fyp_place_recognition
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=/home/cas/.venvs/fyp_gtsam/bin/python
OUT=outputs/cumulti_v1/23c_information_validity
SRC=src/cumulti/stage23c
# Resume only completed components; input/policy hashes are verified each time.
if [ ! -d "$OUT" ]; then "$PY" "$SRC/common.py"; fi
if [ ! -f "$OUT/synthetic_information_sanity.json" ]; then "$PY" "$SRC/synthetic.py"; fi
if [ ! -f "$OUT/stage23c_graph_computation_audit.json" ]; then "$PY" "$SRC/graph_audit.py"; fi
"$PY" -c 'import json,sys; p=json.load(open("outputs/cumulti_v1/23c_information_validity/stage23c_graph_computation_audit.json")); sys.exit(0 if p["status"]=="COMPLETE" else "PARTIAL: stop at frozen resource limit; retain partial artifacts")'
if [ ! -f "$OUT/criticality_mechanism_summary.json" ]; then "$PY" "$SRC/analyze.py"; fi
"$PY" "$SRC/finalize.py"
