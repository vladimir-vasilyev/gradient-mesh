// GradientMesh.h — a grid of control points ("vertices") forming a
// topologically-rectangular arrangement of Ferguson patches, exactly the
// representation described in Sun et al. 2007, Sec. 3-4.
//
// Geometry tangents Pu, Pv AND the twist Puv are all free per-vertex
// unknowns (as in the paper), jointly optimized alongside P by the
// geometry Gauss-Newton step in MeshOptimizer -- see MeshVertex below.
// Puv used to be *derived* from neighboring vertex positions via centered
// finite differences (Catmull-Rom style); that simplification is gone as
// of the pass that added Puv to the free-unknown set (see git history --
// GradientMesh::twist() still exists, now serving only as Puv's initial
// seed value and the optimizer's soft "prior" target, exactly the role
// tangentU/V already play for Pu/Pv). Color keeps the full, independent
// (C, Cu, Cv, Cuv) unknown set per vertex, unaffected by this.
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
    // Geometry tangents Pu, Pv AND the twist Puv are all FREE unknowns
    // (matching the paper), not derived via finite differences -- see
    // GradientMesh::tangentU/V/twist, which still exist but now serve only
    // as (a) the initial value seeded in buildInitial and (b) the
    // optimizer's soft "prior" target that keeps them grounded near the
    // position-implied estimate instead of drifting unconstrained (see
    // MeshOptimizer's geomTangentPriorWeight, which anchors all three).
    Vec2 Pu, Pv, Puv;
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

    // Position-implied tangent/twist estimate (centered finite difference
    // of neighboring P's) -- NOT what geomCorner()/evalPos() actually use
    // for Pu/Pv/Puv any more (those read the free MeshVertex::Pu/Pv/Puv
    // fields directly). Used only to seed the free unknowns in
    // buildInitial() and as MeshOptimizer's soft "prior" anchor (see
    // MeshVertex comment).
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
