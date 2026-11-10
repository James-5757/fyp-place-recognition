#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
ORB_ROOT="${ORB_ROOT:-/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3}"
OUT="$PWD/outputs/cumulti_v1/24_rgbd_loss_rootcause_v1/bin"
mkdir -p "$OUT"
g++ -std=c++17 -O2 src/cumulti/rgbd_loss_audit/native_features.cc -I"$ORB_ROOT" -I"$ORB_ROOT/include" \
 $(pkg-config --cflags opencv4) -L"$ORB_ROOT/lib" -lORB_SLAM3 $(pkg-config --libs opencv4) \
 -Wl,-rpath,"$ORB_ROOT/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ORB_ROOT/Thirdparty/g2o/lib:$ORB_ROOT/../pangolin-install/lib" -o "$OUT/native_features"
