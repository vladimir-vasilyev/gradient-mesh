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

std::string exportGradientMeshSVG(const GradientMesh& mesh, int canvasWidth, int canvasHeight);

} // namespace gmcore
