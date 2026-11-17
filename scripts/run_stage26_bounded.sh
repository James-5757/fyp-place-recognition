#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${STAGE26_PYTHON:-/home/cas/.venvs/fyp_stage24/bin/python}"
scratch="${STAGE26_SCRATCH:-$PWD/work}"
out="$PWD/outputs/cumulti_v1/26_rgbd_reconstruction"
mkdir -p "$scratch"
exec 9>"$scratch/stage26-bounded.lock"
flock -n 9
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

recover() {
    local robot="$1" kind="$2" name archive db
    name="${robot}_main_campus_camera_${kind}"
    archive="/home/cas/CU-Multi/raw/main_campus/$robot/$name.zip"
    db="$scratch/$name.db3"
    if [[ ! -f "$db" ]]; then
        unzip -p "$archive" "$name/$name.db3" > "$db.partial"
        mv "$db.partial" "$db"
    fi
    "$python" src/cumulti/rgbd_full/stage26_recover.py --scratch "$scratch" --robot "$robot" --kind "$kind"
}

if [[ "${1:-}" != "--recoveries-running" ]]; then
    # Default sequential expansion keeps storage pressure predictable.
    for robot in robot1 robot3; do
        for kind in rgb depth; do recover "$robot" "$kind"; done
    done
else
    deadline=$((SECONDS+3600))
    until [[ -s "$out/robot1_rgb_receipts.csv" && -s "$out/robot1_depth_receipts.csv" &&
             -s "$out/robot3_rgb_receipts.csv" && -s "$out/robot3_depth_receipts.csv" ]]; do
        if (( SECONDS >= deadline )); then
            echo 'Timed out waiting for the four selective exports' >&2
            exit 1
        fi
        sleep 10
    done
fi
"$python" src/cumulti/rgbd_full/stage26_recover.py --scratch "$scratch"
if [[ -d "$PWD/python-packages" ]]; then
    export PYTHONPATH="$PWD/python-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
"$python" src/cumulti/rgbd_full/stage26_register.py
