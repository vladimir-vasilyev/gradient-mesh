// MeshRenderBuffers.h -- GPU-backend-agnostic mesh data for the (planned)
// GPU reconstruction renderer. See GLShaderSources.h for the OpenGL
// shaders that consume this; a future Vulkan backend (see README/commit
// history -- explicitly planned, not yet built) reuses this same struct
// and builder verbatim, since it is pure data with no GL/Vulkan/windowing
// dependency at all. Only context creation and the draw-call plumbing
// around this data are backend-specific (mac/GLReconstructionView.mm for
// OpenGL today).
#pragma once
#include "gmcore/GradientMesh.h"
#include <vector>
#include <cstdint>

namespace gmcore {

// vertexData layout: 18 floats per mesh grid vertex, row-major (same
// indexing as GradientMesh::idx(row,col) == row*cols+col):
//   [0,1]      P.x,  P.y     (geometry position)
//   [2,3]      Pu.x, Pu.y    (geometry u-tangent, free unknown)
//   [4,5]      Pv.x, Pv.y    (geometry v-tangent, free unknown)
//   [6,7,8]    C.r,  C.g,  C.b    (colour)
//   [9,10,11]  Cu.r, Cu.g, Cu.b   (colour u-tangent)
//   [12,13,14] Cv.r, Cv.g, Cv.b   (colour v-tangent)
//   [15,16,17] Cuv.r,Cuv.g,Cuv.b  (colour twist)
// Geometry twist (Puv) is deliberately NOT stored: GradientMesh::geomCorner
// hardcodes it to {0,0} (see GradientMesh.h's header comment / Sec 3 of the
// paper -- "the values of muv are usually set to zero"), so every consumer
// of this buffer (see GLShaderSources.h's vertex shader) must do the same
// rather than expecting a 19th/20th float that was never written.
struct MeshRenderBuffers {
    std::vector<float> vertexData;   // rows*cols*18 floats
    std::vector<float> uvTemplate;   // (samplesPerEdge+1)^2 * 2 floats -- one (u,v) pair per tessellation node, shared by every patch instance
    std::vector<uint32_t> indices;   // 2 triangles per tessellation cell (samplesPerEdge^2 cells), same winding as GradientMesh::render()'s rasterizeTriangle(...,i00,i10,i11,...)/(...,i00,i11,i01,...) calls
    int rows = 0, cols = 0;
    int patchRows = 0, patchCols = 0;  // = rows-1, cols-1 (clamped to >=0); GPU instance count = patchRows*patchCols
    int samplesPerEdge = 0;            // == std::max(2, samplesPerPatchEdge), mirrors GradientMesh::render()'s own clamp

    // Builds the buffers from a live mesh. samplesPerPatchEdge mirrors
    // GradientMesh::render()'s parameter of the same name (and the same
    // >=2 clamp).
    static MeshRenderBuffers build(const GradientMesh& mesh, int samplesPerPatchEdge);
};

} // namespace gmcore
