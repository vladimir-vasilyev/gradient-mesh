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

add_executable(ceres_geom_spike
    ceres_geom_spike.cpp
    ../core/src/GradientMesh.cpp
    ../core/src/Image.cpp
)
target_include_directories(ceres_geom_spike PRIVATE ../core/include)
target_link_libraries(ceres_geom_spike PRIVATE Ceres::ceres)
