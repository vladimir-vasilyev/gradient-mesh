#!/bin/bash
# Builds and runs ceres_joint_drift_probe with the real Ceres install
# (Homebrew) on this Mac, and saves its output to
# spike/ceres_joint_drift_probe_output.txt next to this script -- Claude
# reads that log file back afterward, so you don't need to paste anything.
#
# Run from anywhere:
#   bash spike/run_joint_drift_probe.sh
set -e
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake .. -DCMAKE_PREFIX_PATH="$(brew --prefix)" >/dev/null
cmake --build . -j4 --target ceres_joint_drift_probe
cd ..
{
  echo "=== 5x5 mesh, 6 repeats ==="
  ./build/ceres_joint_drift_probe --input ../gradient.png --rows 5 --cols 5 --repeats 6
  echo
  echo "=== 9x9 mesh, 6 repeats ==="
  ./build/ceres_joint_drift_probe --input ../gradient.png --rows 9 --cols 9 --repeats 6
} | tee ceres_joint_drift_probe_output.txt
echo
echo "Saved output to spike/ceres_joint_drift_probe_output.txt"
