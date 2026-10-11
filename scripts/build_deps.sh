#!/bin/bash
# Builds the third-party libraries the release app needs as STATIC arm64
# libraries targeting macOS 12, into build_deps/install:
#   gflags -> glog -> libpng -> Ceres Solver (WITHOUT SuiteSparse).
#
# Why from source: the Homebrew builds target the host macOS (15+), so an app
# linked against them cannot run on macOS 12-14, and Homebrew's Ceres drags in
# SuiteSparse (CHOLMOD/SPQR carry GPL terms), METIS, OpenBLAS and TBB. Ceres
# built with -DSUITESPARSE=OFF -DACCELERATESPARSE=ON uses Apple's Accelerate
# sparse Cholesky for the SPARSE_NORMAL_CHOLESKY solves instead, which keeps
# the distributed binary free of those components (see THIRD_PARTY_NOTICES.md).
# Eigen is header-only; the installed Homebrew headers are used.
#
# Usage: scripts/build_deps.sh      (idempotent; delete build_deps/ to rebuild)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEPS="$ROOT/build_deps"
SRC="$DEPS/src"
PREFIX="$DEPS/install"
EIGEN_CMAKE_DIR="${EIGEN3_DIR:-/opt/homebrew/share/eigen3/cmake}"
JOBS="$(sysctl -n hw.ncpu)"

export MACOSX_DEPLOYMENT_TARGET=12.0
COMMON=(-DCMAKE_BUILD_TYPE=Release
        -DCMAKE_OSX_ARCHITECTURES=arm64
        -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0
        -DCMAKE_INSTALL_PREFIX="$PREFIX"
        -DCMAKE_PREFIX_PATH="$PREFIX"
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_TESTING=OFF)

mkdir -p "$SRC" "$PREFIX"

fetch() {  # name url tag
  if [ ! -d "$SRC/$1" ]; then
    git clone --quiet --depth 1 --branch "$3" "$2" "$SRC/$1"
  fi
  echo "$1 @ $(git -C "$SRC/$1" describe --tags --always)"
}

build() {  # name [extra cmake args...]
  local name="$1"; shift
  if [ -f "$DEPS/.done-$name" ]; then echo "[$name] already built"; return; fi
  echo "[$name] configure + build"
  cmake -S "$SRC/$name" -B "$DEPS/build-$name" -G "Unix Makefiles" "${COMMON[@]}" "$@" > "$DEPS/log-$name.txt" 2>&1
  cmake --build "$DEPS/build-$name" -j "$JOBS" >> "$DEPS/log-$name.txt" 2>&1
  cmake --install "$DEPS/build-$name" >> "$DEPS/log-$name.txt" 2>&1
  touch "$DEPS/.done-$name"
}

fetch gflags https://github.com/gflags/gflags.git v2.3.1
fetch glog   https://github.com/google/glog.git v0.7.1
fetch libpng https://github.com/pnggroup/libpng.git v1.6.50
fetch ceres-solver https://github.com/ceres-solver/ceres-solver.git 2.2.0

# Ceres 2.2.0 predates Eigen 5. These are the same two upstream commits
# Homebrew backports (CMake Eigen version check, minus its docs hunk, and
# Eigen 5 support); the second one's sha256 is checked against Homebrew's.
# Both files live next to this script (downloaded from
# https://github.com/ceres-solver/ceres-solver/commit/<sha>.patch).
if [ ! -f "$DEPS/.patched-ceres" ]; then
  [ "$(shasum -a 256 "$ROOT/scripts/0002-eigen5-support.patch" | cut -d' ' -f1)" = \
    "019006cc850b19b442e108118c599c98b18af8eb06ab37c22e6698c55d55a512" ] || { echo "patch checksum mismatch"; exit 1; }
  git -C "$SRC/ceres-solver" apply --exclude='docs/*' "$ROOT/scripts/0001-eigen-version-check.patch"
  git -C "$SRC/ceres-solver" apply "$ROOT/scripts/0002-eigen5-support.patch"
  touch "$DEPS/.patched-ceres"
fi

build gflags -DGFLAGS_BUILD_STATIC_LIBS=ON -DGFLAGS_BUILD_SHARED_LIBS=OFF -DGFLAGS_BUILD_TESTING=OFF
build glog -DWITH_GFLAGS=ON -DWITH_GTEST=OFF -DWITH_UNWIND=OFF
build libpng -DPNG_SHARED=OFF -DPNG_STATIC=ON -DPNG_TESTS=OFF -DPNG_TOOLS=OFF -DPNG_FRAMEWORK=OFF
build ceres-solver -DSUITESPARSE=OFF -DACCELERATESPARSE=ON -DEIGENSPARSE=OFF -DLAPACK=ON \
      -DUSE_CUDA=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF \
      -DEigen3_DIR="$EIGEN_CMAKE_DIR"

echo "Done. Static libraries in $PREFIX/lib:"
ls "$PREFIX/lib"
