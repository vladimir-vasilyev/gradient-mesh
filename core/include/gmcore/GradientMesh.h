// GradientMesh.h — a grid of control points ("vertices") forming a
// topologically-rectangular arrangement of Ferguson patches, exactly the
// representation described in Sun et al. 2007, Sec. 3-4.
//
// Geometry tangents Pu, Pv are free per-vertex unknowns (as in the paper),
// jointly optimized alongside P by the geometry Gauss-Newton step in
// MeshOptimizer -- see MeshVertex below. The twist Puv is NOT free: the
// paper states plainly (Sec 3), "The mu, mv, muv are the partial
// derivatives. In practice, the values of muv are usually set to zero" --
// so geomCorner() hardcodes it to {0,0} rather than reading a per-vertex
// value or deriving it via finite differences. (An earlier pass in this
// project briefly promoted Puv to a free unknown too, for completeness;
// reverted after rereading the paper's own text on this point -- see git
// history.) The MeshVertex::Puv field below is kept only as inert storage
// (never read by geomCorner/evalPos, never touched by the optimizer) so
// serialization/CSV code that references it doesn't need to change; it is
// always {0,0}. Color keeps the full, independent (C, Cu, Cv, Cuv) unknown
// set per vertex, unaffected by any of this.
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
    // Geometry tangents Pu, Pv are FREE unknowns (matching the paper), not
    // derived via finite differences -- see GradientMesh::tangentU/V, which
    // still exist but now serve only as (a) the initial value seeded in
    // buildInitial and (b) the optimizer's soft "prior" target that keeps
    // them grounded near the position-implied estimate instead of drifting
    // unconstrained (see MeshOptimizer's geomTangentPriorWeight, which
    // anchors both). Puv (twist) is NOT free -- see the file header comment
    // above; this field is inert storage, always {0,0}, never read by
    // geomCorner/evalPos.
    Vec2 Pu, Pv, Puv;
    Color C, Cu, Cv, Cuv;
    bool isBoundary = false;
    int boundarySide = -1;   // which of the 4 boundary sides (each a BezierSpline -- one or more cubic segments), or -1
    double boundaryT = 0.0;  // parameter along that segment (kept up to date by the optimizer)
};

class GradientMesh {
public:
    int rows = 0, cols = 0; // control-point grid is rows x cols; patches are (rows-1) x (cols-1)
    std::vector<MeshVertex> vertices;
    std::array<BezierSpline, 4> boundary; // 0=top(u,0..1 at v=0) 1=right 2=bottom 3=left -- see buildInitial. Each side is one-or-more cubic segments (see BezierSpline.h), not a single CubicBezier any more -- Sun et al. Sec. 4: "each boundary consists of one or more cubic Bezier splines".

    int idx(int row, int col) const { return row * cols + col; }
    MeshVertex& at(int row, int col) { return vertices[idx(row, col)]; }
    const MeshVertex& at(int row, int col) const { return vertices[idx(row, col)]; }

    // Builds an initial mesh of the given resolution by transfinite
    // (Coons) interpolation between the 4 boundary curves, with initial
    // colors bilinearly sampled from `target` at each control point.
    static GradientMesh buildInitial(int rows, int cols, const std::array<BezierSpline, 4>& boundary,
                                      const Image& target);

    // Position-implied tangent/twist estimate (centered finite difference
    // of neighboring P's). tangentU/tangentV are NOT what geomCorner()/
    // evalPos() use for Pu/Pv any more (those read the free
    // MeshVertex::Pu/Pv fields directly) -- used only to seed the free
    // unknowns in buildInitial() and as MeshOptimizer's soft "prior" anchor
    // (see MeshVertex comment). twist() is unused by geomCorner() (which
    // hardcodes Puv to {0,0} per the paper's Sec 3) but is kept as a
    // utility / for any future experimentation.
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

    // Same sampling (same quadrature points, same samplesPerPatchEdge
    // convention) and the same per-channel-scalar treatment as
    // reconstructionRMSE (r/g/b differences are pooled as independent
    // scalar samples, not combined into a per-sample Euclidean norm) --
    // just mean(|diff|) instead of sqrt(mean(diff^2)). Reported alongside
    // RMSE specifically because RMSE's squaring makes it disproportionately
    // sensitive to a few large-error samples (e.g. a sharp edge the mesh
    // hasn't caught yet); MAE weighs every sample equally, so comparing the
    // two together says something RMSE alone can't -- a run with similar
    // RMSE but higher MAE (relative to another run) is making more
    // widespread small errors and fewer large ones, or vice versa.
    double reconstructionMAE(const Image& target, int samplesPerPatchEdge = 6) const;
};

} // namespace gmcore
