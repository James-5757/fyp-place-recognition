#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
ORB_ROOT="${ORB_ROOT:-/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3}"
PANGO_ROOT="${PANGO_ROOT:-/home/cas/fyp_mcd_stereo_inertial/tools/pangolin-install}"
STAGE24_BIN="${STAGE24_OUTPUT_ROOT:-$PWD/outputs/cumulti_v1}/24b_vision_feasibility/bin"
mkdir -p "$STAGE24_BIN"
g++ -std=c++14 -O2 -DCOMPILEDWITHC11 -DHAVE_EPOXY -DHAVE_EIGEN src/cumulti/stage24/mono_pilot.cc \
 -I"$ORB_ROOT" -I"$ORB_ROOT/include" -I"$ORB_ROOT/include/CameraModels" \
 -I"$ORB_ROOT/Thirdparty/Sophus" -I/usr/include/eigen3 -I"$PANGO_ROOT/include" \
 $(pkg-config --cflags opencv4) -L"$ORB_ROOT/lib" -lORB_SLAM3 \
 $(pkg-config --libs opencv4) -L"$PANGO_ROOT/lib" -lpango_display -lpango_opengl -lpango_core -lepoxy \
 -Wl,-rpath,"$ORB_ROOT/lib:$PANGO_ROOT/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ORB_ROOT/Thirdparty/g2o/lib" \
 -o "$STAGE24_BIN/stage24_mono_pilot"
