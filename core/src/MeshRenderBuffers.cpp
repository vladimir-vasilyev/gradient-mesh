#include "gmcore/MeshRenderBuffers.h"
#include <algorithm>

namespace gmcore {

MeshRenderBuffers MeshRenderBuffers::build(const GradientMesh& mesh, int samplesPerPatchEdge) {
    MeshRenderBuffers out;
    out.rows = mesh.rows;
    out.cols = mesh.cols;
    out.patchRows = std::max(0, mesh.rows - 1);
    out.patchCols = std::max(0, mesh.cols - 1);
    int n = std::max(2, samplesPerPatchEdge);
    out.samplesPerEdge = n;

    out.vertexData.resize(size_t(mesh.rows) * size_t(mesh.cols) * 18);
    for (int row = 0; row < mesh.rows; ++row) {
        for (int col = 0; col < mesh.cols; ++col) {
            const MeshVertex& v = mesh.at(row, col);
            float* p = &out.vertexData[size_t(mesh.idx(row, col)) * 18];
            p[0]  = (float)v.P.x;   p[1]  = (float)v.P.y;
            p[2]  = (float)v.Pu.x;  p[3]  = (float)v.Pu.y;
            p[4]  = (float)v.Pv.x;  p[5]  = (float)v.Pv.y;
            p[6]  = (float)v.C.r;   p[7]  = (float)v.C.g;   p[8]  = (float)v.C.b;
            p[9]  = (float)v.Cu.r;  p[10] = (float)v.Cu.g;  p[11] = (float)v.Cu.b;
            p[12] = (float)v.Cv.r;  p[13] = (float)v.Cv.g;  p[14] = (float)v.Cv.b;
            p[15] = (float)v.Cuv.r; p[16] = (float)v.Cuv.g; p[17] = (float)v.Cuv.b;
        }
    }

    out.uvTemplate.reserve(size_t(n + 1) * (n + 1) * 2);
    for (int i = 0; i <= n; ++i) {
        double vv = double(i) / n;
        for (int j = 0; j <= n; ++j) {
            double uu = double(j) / n;
            out.uvTemplate.push_back((float)uu);
            out.uvTemplate.push_back((float)vv);
        }
    }

    auto nodeIdx = [n](int i, int j) { return uint32_t(i * (n + 1) + j); };
    out.indices.reserve(size_t(n) * n * 6);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            uint32_t i00 = nodeIdx(i, j), i10 = nodeIdx(i, j + 1), i01 = nodeIdx(i + 1, j), i11 = nodeIdx(i + 1, j + 1);
            // Same two-triangle split/winding as GradientMesh::render()'s
            // rasterizeTriangle(pos[i00],pos[i10],pos[i11], col[i00],col[i10],col[i11])
            // and (pos[i00],pos[i11],pos[i01], col[i00],col[i11],col[i01]) calls.
            out.indices.push_back(i00); out.indices.push_back(i10); out.indices.push_back(i11);
            out.indices.push_back(i00); out.indices.push_back(i11); out.indices.push_back(i01);
        }
    }
    return out;
}

} // namespace gmcore
