// GradientMesh.h — a grid of control points ("vertices") forming a
// topologically-rectangular arrangement of Ferguson patches, exactly the
// representation described in Sun et al. 2007, Sec. 3-4.
//
// Simplification vs. the paper (documented in README "Known
// simplifications"): geometry tangents/twist (Pu, Pv, Puv) are *derived*
// from neighboring vertex positions via centered finite differences
// (Catmull-Rom style) rather than kept as independent optimization
// unknowns; only the position P is a free geometry unknown. Color keeps
// the full, independent (C, Cu, Cv, Cuv) unknown set per vertex, which is
// what actually gives a gradient mesh its smooth-shading expressiveness.
#pragma once
#include "gmcore/Vec2.h"
#include "gmcore/Color.h"
#include "gmcore/FergusonPatch.h"
#include "gmcore/BezierSpline.h"
#include "gmcore/Image.h"
#include <vector>
#include <array>

namespace gmcore {

struct MeshVertex {
    Vec2 P;
    Color C, Cu, Cv, Cuv;
    bool isBoundary = false;
    int boundarySide = -1;   // which of the 4 CubicBezier boundary segments, or -1
    double boundaryT = 0.0;  // parameter along that segment (kept up to date by the optimizer)
};

class GradientMesh {
public:
    int rows = 0, cols = 0; // control-point grid is rows x cols; patches are (rows-1) x (cols-1)
    std::vector<MeshVertex> vertices;
    std::array<CubicBezier, 4> boundary; // 0=top(u,0..1 at v=0) 1=right 2=bottom 3=left -- see buildInitial

    int idx(int row, int col) const { return row * cols + col; }
    MeshVertex& at(int row, int col) { return vertices[idx(row, col)]; }
    const MeshVertex& at(int row, int col) const { return vertices[idx(row, col)]; }

    // Builds an initial mesh of the given resolution by transfinite
    // (Coons) interpolation between the 4 boundary curves, with initial
    // colors bilinearly sampled from `target` at each control point.
    static GradientMesh buildInitial(int rows, int cols, const std::array<CubicBezier, 4>& boundary,
                                      const Image& target);

    // Derived geometry tangents (see class comment).
    Vec2 tangentU(int row, int col) const;
    Vec2 tangentV(int row, int col) const;
    Vec2 twist(int row, int col) const;

    HermiteCorner<Vec2> geomCorner(int row, int col) const;
    HermiteCorner<Color> colorCorner(int row, int col) const;

    // Evaluate patch (patchRow, patchCol) at local params u,v in [0,1].
    Vec2 evalPos(int patchRow, int patchCol, double u, double v, Vec2* dU = nullptr, Vec2* dV = nullptr) const;
    Color evalColor(int patchRow, int patchCol, double u, double v) const;

    // Rasterizes the mesh into an image of the given size (used for
    // previews, error visualization and PNG export -- NOT used inside the
    // optimizer's hot loop, see MeshOptimizer).
    Image render(int outW, int outH, int samplesPerPatchEdge = 10) const;

    // Uniformly scales all positions (used when moving a mesh from a
    // coarse pyramid level to the next, finer one).
    void scalePositions(double sx, double sy);

    double reconstructionRMSE(const Image& target, int samplesPerPatchEdge = 6) const;
};

} // namespace gmcore
