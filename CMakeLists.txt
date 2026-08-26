cmake_minimum_required(VERSION 3.16)
project(GradientMeshStudio CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# ---------------------------------------------------------------------------
# gmcore: the portable C++ algorithm library (Ferguson patches, gradient
# mesh, Levenberg-Marquardt-style optimizer, SVG export). No GUI, no
# external dependencies beyond an optional system libpng for convenience
# PNG I/O in the CLI tool -- the macOS app instead loads/saves images via
# native AppKit ImageIO (see mac/DocumentModel.mm) so it needs nothing
# extra at all.
# ---------------------------------------------------------------------------
file(GLOB GMCORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/core/src/*.cpp)
add_library(gmcore STATIC ${GMCORE_SOURCES})
target_include_directories(gmcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include)

find_package(PNG QUIET)
if(PNG_FOUND)
  target_link_libraries(gmcore PUBLIC PNG::PNG)
  target_compile_definitions(gmcore PUBLIC GMCORE_USE_LIBPNG)
  message(STATUS "gmcore: libpng found, PNG I/O enabled in gmcore/CLI")
else()
  message(STATUS "gmcore: libpng not found, only the built-in PPM format is available in gmcore/CLI (the macOS app is unaffected -- it uses AppKit ImageIO)")
endif()

# ---------------------------------------------------------------------------
# gmesh_cli: dependency-free command-line reference/test harness. Builds
# and runs on Linux/macOS/Windows -- this is how the algorithm was
# validated during development, since this sandbox cannot run Xcode.
# ---------------------------------------------------------------------------
add_executable(gmesh_cli cli/main_cli.cpp)
target_link_libraries(gmesh_cli PRIVATE gmcore)

# ---------------------------------------------------------------------------
# GradientMeshStudio.app: the interactive macOS AppKit front-end. Only
# builds on Apple platforms. On macOS, generate the actual Xcode project
# with:
#     cmake -G Xcode -B build .
#     open build/GradientMeshStudio.xcodeproj
# (see README.md for the full walkthrough).
# ---------------------------------------------------------------------------
if(APPLE)
  file(GLOB GMSTUDIO_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/mac/*.mm)
  add_executable(GradientMeshStudio MACOSX_BUNDLE ${GMSTUDIO_SOURCES})
  target_link_libraries(GradientMeshStudio PRIVATE gmcore
    "-framework Cocoa" "-framework AppKit" "-framework QuartzCore" "-framework CoreGraphics" "-framework ImageIO" "-framework UniformTypeIdentifiers")
  set_target_properties(GradientMeshStudio PROPERTIES
    MACOSX_BUNDLE_BUNDLE_NAME "Gradient Mesh Studio"
    MACOSX_BUNDLE_GUI_IDENTIFIER "com.gradientmeshstudio.app"
    MACOSX_BUNDLE_BUNDLE_VERSION "1.0"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0"
    MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/mac/Info.plist
    XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC "YES"
    XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++17"
    XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++"
    XCODE_ATTRIBUTE_MACOSX_DEPLOYMENT_TARGET "12.0"
  )
endif()
