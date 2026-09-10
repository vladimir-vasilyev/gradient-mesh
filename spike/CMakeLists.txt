# spike/CMakeLists.txt — standalone build for the Ceres feasibility spike.
# Deliberately NOT wired into the root CMakeLists.txt: this is a throwaway
# Phase-0 experiment (see README/plan discussion), not a build dependency
# of gmesh_cli or the macOS app. Build it separately:
#
#   cd spike && mkdir -p build && cd build
#   cmake .. -DCMAKE_PREFIX_PATH="$(brew --prefix)"
#   cmake --build . -j4
#   ./ceres_geom_spike
#
# If `find_package(Ceres REQUIRED)` can't find it, try being explicit:
#   cmake .. -DCMAKE_PREFIX_PATH="$(brew --prefix ceres-solver);$(brew --prefix eigen)"

cmake_minimum_required(VERSION 3.16)
project(gmcore_ceres_spike CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)

# ceres_joint_drift_probe loads gradient.png -- needs libpng the same way
# the root CMakeLists.txt's gmcore/gmesh_cli do (see that file's comment);
# without this, Image::load silently falls back to the PPM-only path and
# aborts on a .png input.
find_package(PNG QUIET)

add_executable(ceres_geom_spike
    ceres_geom_spike.cpp
    ../core/src/GradientMesh.cpp
    ../core/src/Image.cpp
)
target_include_directories(ceres_geom_spike PRIVATE ../core/include)
target_link_libraries(ceres_geom_spike PRIVATE Ceres::ceres)

# Phase 0 of the full joint (geometry+color) solve investigation -- see
# ceres_joint_spike.cpp's header comment for why this is a separate,
# still-throwaway spike rather than an extension of ceres_geom_spike.
add_executable(ceres_joint_spike
    ceres_joint_spike.cpp
    ../core/src/GradientMesh.cpp
    ../core/src/Image.cpp
)
target_include_directories(ceres_joint_spike PRIVATE ../core/include)
target_link_libraries(ceres_joint_spike PRIVATE Ceres::ceres)

# Repeated-call drift probe for the README's "useCeresJoint repeated-run
# drift" diagnosis (see ceres_joint_drift_probe.cpp's header comment) --
# needs the FULL gmcore (MeshOptimizer's coarse-to-fine driver, both the
# hand-rolled and Ceres geometry/joint paths), unlike the two spikes above
# which only need GradientMesh/Image directly.
add_executable(ceres_joint_drift_probe
    ceres_joint_drift_probe.cpp
    ../core/src/GradientMesh.cpp
    ../core/src/Image.cpp
    ../core/src/MeshOptimizer.cpp
    ../core/src/MeshOptimizerCeres.cpp
    ../core/src/ColorSpace.cpp
    ../core/src/BezierSpline.cpp
)
target_include_directories(ceres_joint_drift_probe PRIVATE ../core/include)
target_compile_definitions(ceres_joint_drift_probe PRIVATE GMCORE_WITH_CERES)
target_link_libraries(ceres_joint_drift_probe PRIVATE Ceres::ceres)
if(PNG_FOUND)
  target_link_libraries(ceres_joint_drift_probe PRIVATE PNG::PNG)
  target_compile_definitions(ceres_joint_drift_probe PRIVATE GMCORE_USE_LIBPNG)
endif()
