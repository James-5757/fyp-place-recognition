#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
ORB_ROOT="${ORB_ROOT:-/home/cas/fyp_mcd_stereo_inertial/tools/ORB_SLAM3}"
PANGO_ROOT="${PANGO_ROOT:-/home/cas/fyp_mcd_stereo_inertial/tools/pangolin-install}"
OUT="$PWD/outputs/cumulti_v1/24_rgbd_full_v1/bin"
mkdir -p "$OUT"
TMP_BIN="$OUT/rgbd_full.build.$$"
g++ -std=c++17 -O2 -DCOMPILEDWITHC11 -DHAVE_EPOXY -DHAVE_EIGEN src/cumulti/rgbd_full/track.cc \
 -I"$ORB_ROOT" -I"$ORB_ROOT/include" -I"$ORB_ROOT/include/CameraModels" \
 -I"$ORB_ROOT/Thirdparty/Sophus" -I/usr/include/eigen3 -I"$PANGO_ROOT/include" \
 $(pkg-config --cflags opencv4) -L"$ORB_ROOT/lib" -lORB_SLAM3 -L"$ORB_ROOT/Thirdparty/DBoW2/lib" -lDBoW2 \
 $(pkg-config --libs opencv4) -L"$PANGO_ROOT/lib" -lpango_display -lpango_opengl -lpango_core -lepoxy \
 -Wl,-rpath,"$ORB_ROOT/lib:$PANGO_ROOT/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ORB_ROOT/Thirdparty/g2o/lib" -o "$TMP_BIN"
mv -- "$TMP_BIN" "$OUT/rgbd_full"

# Independent RGB-only candidate proposer; same pinned ORB vocabulary.
g++ -std=c++17 -O2 -DCOMPILEDWITHC11 -DHAVE_EPOXY -DHAVE_EIGEN src/cumulti/rgbd_full/propose.cc \
 -I"$ORB_ROOT" -I"$ORB_ROOT/include" -I"$ORB_ROOT/include/CameraModels" \
 -I"$ORB_ROOT/Thirdparty/Sophus" -I/usr/include/eigen3 -I"$PANGO_ROOT/include" \
 $(pkg-config --cflags opencv4) -L"$ORB_ROOT/lib" -lORB_SLAM3 -L"$ORB_ROOT/Thirdparty/DBoW2/lib" -lDBoW2 \
 $(pkg-config --libs opencv4) -L"$PANGO_ROOT/lib" -lpango_display -lpango_opengl -lpango_core -lepoxy \
 -Wl,-rpath,"$ORB_ROOT/lib:$PANGO_ROOT/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ORB_ROOT/Thirdparty/g2o/lib" -o "$TMP_BIN"
mv -- "$TMP_BIN" "$OUT/rgbd_propose"
