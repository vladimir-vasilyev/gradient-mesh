// SVGExporter.h — exports a GradientMesh as a standards-based SVG 2
// <meshgradient>/<meshpatch> document: the actual vector gradient-mesh
// primitive, not an approximation, so the mesh stays a real editable
// vector object (openable/editable in tools that implement SVG2 mesh
// gradients, e.g. Inkscape) -- this is the "scalable, editable output"
// the paper is ultimately building towards (Sec. 1, 6).
//
// See the .cpp for the implementation note on how each patch's corner
// colors are encoded (the SVG2 mesh-patch stop-color convention is
// under-specified enough in practice that we document our reading of it
// explicitly rather than leave it implicit).
#pragma once
#include "gmcore/GradientMesh.h"
#include <string>

namespace gmcore {

// sourceIsCIELUV: pass true when `mesh`'s vertex colors (MeshVertex::C) are
// CIELUV (L*,u*,v*) values rather than sRGB -- i.e. when the mesh was built/
// optimized with DocumentModel.useCIELUVColorSpace on (see ColorSpace.h).
// SVG stop-color is always sRGB hex, so each patch corner's color is
// converted back to sRGB before being hex-formatted; nothing else about the
// exported geometry changes. Defaults to false (sRGB, the original
// behavior) so every existing call site keeps compiling/behaving unchanged.
std::string exportGradientMeshSVG(const GradientMesh& mesh, int canvasWidth, int canvasHeight,
                                   bool sourceIsCIELUV = false);

} // namespace gmcore
