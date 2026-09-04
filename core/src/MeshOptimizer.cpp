#include "gmcore/MeshOptimizer.h"
#include "gmcore/SparseBlockSolver.h"
#include "gmcore/FergusonPatch.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace gmcore {

namespace {

struct RowEntry { int vertex; int sub; double coeff; };

// Accumulates one scalar Gauss-Newton residual row (r(x) ~ r0 + J*delta)
// into the normal equations H*delta = g (g holds +J^T*(-r0), i.e. the
// right-hand side such that solving gives the Gauss-Newton update step).
// Pointer+count so the very hot per-sample call sites below (the geometry
// data term's buildChanneled output and the vector-line term's rowU/rowV,
// both always a fixed size, and solveColorExact's row) can pass a
// stack-allocated std::array's .data()/.size() directly instead of first
// heap-allocating a std::vector just to hold the same fixed-size data --
// same entries, same iteration order, so results are bit-identical either
// way; this only removes the allocation, not any computation.
void accumulateGNRow(SparseBlockMatrix& H, std::vector<double>& g, const RowEntry* row, int n,
                      double r0, double weight) {
    for (int i = 0; i < n; ++i) {
        const RowEntry& ei = row[i];
        g[ei.vertex * H.blockSize + ei.sub] += -weight * ei.coeff * r0;
        double diagVal = weight * ei.coeff * ei.coeff;
        H.addScalar(ei.vertex, ei.vertex, ei.sub, ei.sub, diagVal);
        for (int j = i + 1; j < n; ++j) {
            const RowEntry& ej = row[j];
            double val = weight * ei.coeff * ej.coeff;
            if (ei.vertex == ej.vertex) {
                H.addScalar(ei.vertex, ej.vertex, ei.sub, ej.sub, val);
                H.addScalar(ei.vertex, ej.vertex, ej.sub, ei.sub, val);
            } else {
                H.addScalar(ei.vertex, ej.vertex, ei.sub, ej.sub, val);
            }
        }
    }
}

// Convenience overload for the infrequent call sites (smoothness/tangent-
// prior/boundary terms below -- once per vertex/triple per GN iteration,
// not once per sample, so their own small std::vector allocation cost is
// negligible) that still build a literal braced-init-list. Delegates to
// the pointer+count version above so there's exactly one copy of the
// actual accumulation logic to keep in sync.
void accumulateGNRow(SparseBlockMatrix& H, std::vector<double>& g, const std::vector<RowEntry>& row,
                      double r0, double weight) {
    accumulateGNRow(H, g, row.data(), (int)row.size(), r0, weight);
}

// The (bi,bj) block-connectivity pattern of BOTH the geometry-GN normal
// equations (SparseBlockMatrix blockSize=6, built in
// optimizeAtCurrentResolution below) and solveColorExact's normal
// equations (blockSize=4) is identical -- it depends only on which vertex
// PAIRS ever co-occur in one accumulateGNRow call above, which for every
// term in this file depends only on mesh TOPOLOGY (mesh.rows/mesh.cols),
// never on vertex VALUES (position/color), which pyramid level this is,
// or whether vector lines are present:
//   - patch data term (and, when present, the vector-line term -- see its
//     call site below, which touches the SAME 4 corner vertices as the
//     data term for that same patch, never a different patch's): full
//     clique among a patch's 4 corner vertices.
//   - smoothness (addSmoothnessTerms): all pairs among each row/col
//     second-difference triple (i0,i1,i2).
//   - every other term (addTangentPriorTerms, the boundary normal-only
//     constraint, colorDerivRidge, and the Levenberg damping loop) only
//     ever touches a single vertex's own diagonal block -- covered
//     unconditionally by the `for (i) addPair(i,i)` below regardless of
//     which of those terms are actually active (e.g.
//     geomTangentPriorWeight<=0 skips addTangentPriorTerms itself, but
//     the diagonal block it WOULD have touched is already declared).
// So this is the same for every GN sub-iteration and outer iteration at a
// given resolution level (mesh.rows/cols don't change mid-level) --
// building it once per optimizeAtCurrentResolution/solveColorExact call
// and reusing it via SparseBlockMatrix::reserveBlocks is what lets every
// later addScalar resolve its block via binary search instead of
// inserting into a map on first touch.
std::vector<std::pair<int,int>> buildMeshBlockPattern(const GradientMesh& mesh) {
    std::vector<std::pair<int,int>> pairs;
    pairs.reserve((size_t)mesh.vertices.size() + (size_t)(mesh.rows - 1) * (mesh.cols - 1) * 10 +
                  (size_t)mesh.rows * mesh.cols * 3);
    auto addPair = [&](int i, int j) {
        if (i > j) std::swap(i, j);
        pairs.emplace_back(i, j);
    };
    for (int i = 0; i < (int)mesh.vertices.size(); ++i) addPair(i, i);
    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            int verts[4] = {mesh.idx(pr, pc), mesh.idx(pr + 1, pc), mesh.idx(pr, pc + 1), mesh.idx(pr + 1, pc + 1)};
            for (int a = 0; a < 4; ++a)
                for (int b = a; b < 4; ++b)
                    addPair(verts[a], verts[b]);
        }
    }
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
            addPair(i0, i1); addPair(i1, i2); addPair(i0, i2);
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
            addPair(i0, i1); addPair(i1, i2); addPair(i0, i2);
        }
    }
    return pairs; // reserveBlocks() itself sorts+dedupes -- no need to do it twice.
}

double areaWeightAt(const GradientMesh& mesh, int pr, int pc, double u, double v, double duv) {
    Vec2 dU, dV;
    mesh.evalPos(pr, pc, u, v, &dU, &dV);
    double a = std::abs(dU.cross(dV)) * duv;
    return std::max(a, 1e-6);
}

// Anisotropic relaxation for the position-smoothness term: how much to
// scale smoothWeightGeom DOWN at a vertex currently sitting near a strong
// local image gradient, so the isotropic "keep evenly spaced" regularizer
// doesn't fight a mesh-line trying to bunch up against a sharp edge (which
// is, almost by definition, a large local departure from even spacing).
// factor=1 in flat regions (full smoothing, same as before this existed);
// factor -> smoothGeomMinFactor near a strong edge (smoothing mostly, but
// not entirely, relaxed -- a hard floor keeps the geometry solve
// well-posed instead of letting some triple go fully unregularized).
// Recomputed fresh from the CURRENT position every time it's called (each
// GN sub-iteration re-linearizes here, same as everywhere else in this
// file), so it adapts as a vertex approaches an edge rather than being
// fixed at the mesh's starting shape.
//
// NOTE: this whole edgeRelaxFactor/anisotropic-smoothing mechanism is NOT
// in the paper (Sec 4.1's smoothness term is a single flat isotropic
// weight lambda, applied to position only) -- it was added earlier in this
// project as a workaround for pinching resistance. Left in place (default
// smoothGeomEdgeGain=40) since removing it is a separate, larger change;
// flagging here for honesty about what is/isn't paper-faithful.
double edgeRelaxFactor(const Image& target, const Vec2& p, double edgeGain, double minFactor) {
    ColorGrad grad = target.sampleGradient(p.x, p.y);
    double mag2 = grad.dx.r * grad.dx.r + grad.dx.g * grad.dx.g + grad.dx.b * grad.dx.b +
                  grad.dy.r * grad.dy.r + grad.dy.g * grad.dy.g + grad.dy.b * grad.dy.b;
    double mag = std::sqrt(mag2);
    double factor = 1.0 / (1.0 + edgeGain * mag);
    return std::max(factor, minFactor);
}

// Sec 4.2's vector-line-guided term evaluates a local direction field at
// every interior (u,v) sample of a patch (not just at discrete
// mesh-vertex edges), with a Gaussian falloff weight based on distance to
// "the nearest vector line" -- quoting the paper directly: "wu(m(u,v)) =
// G(d|0,sigma_v^2) where d is the distance from the location m(u,v) to
// the nearest vector line... We set the standard deviation sigma_v to one
// third of the width of the narrow band[,] set as one fifth of the length
// of the vector line." So each line's own band/sigma is derived from ITS
// OWN length -- not a single global pixel radius (which an earlier,
// coarser version of this term used, checked only at discrete mesh edges
// rather than this dense per-sample grid, and never touched Pu/Pv at
// all). This finds the single globally-nearest (line segment, point) pair
// first (matching "the nearest vector line", singular), then applies THAT
// line's own band cutoff and Gaussian using its own sigma.
struct VectorLineMatch { bool found = false; Vec2 dir{1, 0}; double weight = 0.0; };

VectorLineMatch nearestVectorLineField(const std::vector<VectorLine>& lines, const Vec2& p) {
    VectorLineMatch result;
    double bestD2 = 1e300;
    Vec2 bestDir{1, 0};
    double bestSigma = 0.0;
    for (const auto& line : lines) {
        double totalLen = 0.0;
        for (size_t i = 0; i + 1 < line.points.size(); ++i) totalLen += (line.points[i + 1] - line.points[i]).length();
        if (totalLen < 1e-6) continue;
        double sigma = (totalLen / 5.0) / 3.0; // sigma = bandWidth/3, bandWidth = length/5
        for (size_t i = 0; i + 1 < line.points.size(); ++i) {
            Vec2 a = line.points[i], b = line.points[i + 1];
            Vec2 ab = b - a;
            double len2 = ab.lengthSq();
            if (len2 < 1e-9) continue;
            double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / len2));
            Vec2 proj = a + ab * t;
            double d2 = (p - proj).lengthSq();
            if (d2 < bestD2) { bestD2 = d2; bestDir = ab.normalized(); bestSigma = sigma; }
        }
    }
    if (bestSigma <= 1e-9) return result; // no usable (nonzero-length) line
    double d = std::sqrt(bestD2);
    double bandWidth = bestSigma * 3.0;
    if (d > bandWidth) return result; // outside the narrow band -- "not found", matching Sec 4.2
    result.found = true;
    result.dir = bestDir;
    result.weight = std::exp(-(d * d) / (2.0 * bestSigma * bestSigma));
    return result;
}

void addSmoothnessTerms(SparseBlockMatrix& H, std::vector<double>& g, const GradientMesh& mesh,
                         const std::vector<Vec2>& P, const Image& target, const OptimizerOptions& opts) {
    // second differences along u (columns) and v (rows), on x and y separately.
    // Each triple's weight is the base smoothWeightGeom scaled down by
    // edgeRelaxFactor at the middle vertex's current position -- see that
    // function's comment. This is what lets two neighboring mesh-lines
    // actually squeeze together around a sharp edge instead of the
    // isotropic "stay evenly spaced" pull resisting it everywhere equally.
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
            double weight = opts.smoothWeightGeom *
                edgeRelaxFactor(target, P[i1], opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            double rx = P[i0].x - 2 * P[i1].x + P[i2].x;
            double ry = P[i0].y - 2 * P[i1].y + P[i2].y;
            accumulateGNRow(H, g, {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}}, rx, weight);
            accumulateGNRow(H, g, {{i0, 1, 1}, {i1, 1, -2}, {i2, 1, 1}}, ry, weight);
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
            double weight = opts.smoothWeightGeom *
                edgeRelaxFactor(target, P[i1], opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            double rx = P[i0].x - 2 * P[i1].x + P[i2].x;
            double ry = P[i0].y - 2 * P[i1].y + P[i2].y;
            accumulateGNRow(H, g, {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}}, rx, weight);
            accumulateGNRow(H, g, {{i0, 1, 1}, {i1, 1, -2}, {i2, 1, 1}}, ry, weight);
        }
    }
}

// Soft ridge pulling each vertex's free Pu/Pv (block subs 2,3,4,5) toward
// the CURRENT position-implied finite-difference estimate. Re-anchored
// every GN sub-iteration (tu/tv are recomputed from the mesh's current P
// each call), so this is a "prior around the current linearization point",
// not a hard constraint -- it lets the data term pull Pu/Pv away from that
// estimate when it has real signal to, while keeping them from drifting
// unboundedly where the signal is weak or absent. The Jacobian only
// includes the direct d(residual)/d(Pu or Pv) = 1 term, not the indirect
// dependence of tu/tv on neighboring P's. (This whole prior is likewise
// NOT in the paper -- Sec 4.1 has no separate tangent regularizer -- but
// removing it isn't safe without also revisiting the block-coordinate GN
// scheme's stability, see MeshOptimizer.h's OptimizerOptions comment.)
//
// Puv/twist is NOT included here -- per the paper, Sec 3: "In practice,
// the values of muv are usually set to zero," so it's fixed at {0,0}
// rather than free or derived; see GradientMesh::geomCorner. (An earlier
// pass in this project briefly made Puv free too; reverted after
// rereading the paper's own text on this point.)
void addTangentPriorTerms(SparseBlockMatrix& H, std::vector<double>& g, const GradientMesh& mesh,
                           double weight) {
    if (weight <= 0.0) return;
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 0; c < mesh.cols; ++c) {
            int i = mesh.idx(r, c);
            const MeshVertex& mv = mesh.at(r, c);
            Vec2 tu = mesh.tangentU(r, c), tv = mesh.tangentV(r, c);
            accumulateGNRow(H, g, {{i, 2, 1}}, mv.Pu.x - tu.x, weight);
            accumulateGNRow(H, g, {{i, 3, 1}}, mv.Pu.y - tu.y, weight);
            accumulateGNRow(H, g, {{i, 4, 1}}, mv.Pv.x - tv.x, weight);
            accumulateGNRow(H, g, {{i, 5, 1}}, mv.Pv.y - tv.y, weight);
        }
    }
}

// Total weighted energy the geometry Gauss-Newton step actually minimizes:
// data term (area-weighted reconstruction error) + vector-line term +
// smoothness + tangent-prior + boundary, using the exact same residual
// formulas and sample density as optimizeAtCurrentResolution's H/g
// assembly. This -- not a cheap, differently-sampled RMSE proxy -- is
// what backtracking must check a step against; using a mismatched
// acceptance metric let the line search reject genuinely energy-decreasing
// steps (or accept non-decreasing ones), which starved geometry of any
// real movement after the first couple of outer iterations and left the
// mesh looking effectively rectangular.
double computeGeometryEnergy(const GradientMesh& mesh, const Image& target,
                              const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts) {
    int n = std::max(2, opts.samplesPerPatchEdge);
    double duv = 1.0 / (n * n);
    double energy = 0.0;
    bool hasLines = !vectorLines.empty();

    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            for (int i = 0; i <= n; ++i) {
                double v = double(i) / n;
                for (int j = 0; j <= n; ++j) {
                    double u = double(j) / n;
                    Vec2 dU, dV;
                    Vec2 pos = mesh.evalPos(pr, pc, u, v, &dU, &dV);
                    Color cmesh = mesh.evalColor(pr, pc, u, v);
                    Color ctarget = target.sampleBilinear(pos.x, pos.y);
                    double w = areaWeightAt(mesh, pr, pc, u, v, duv);
                    Color d = cmesh - ctarget;
                    energy += w * opts.geomDataWeight * d.lengthSq();

                    // Vector-line guided term (Sec 4.2), evaluated at this
                    // same dense sample grid as the data term -- must
                    // mirror the GN assembly's version of this exactly,
                    // for the same line-search-consistency reason as
                    // everything else in this function. See
                    // nearestVectorLineField's comment for the formula.
                    if (hasLines) {
                        VectorLineMatch match = nearestVectorLineField(vectorLines, pos);
                        if (match.found) {
                            double ru = dU.cross(match.dir);
                            double rv = dV.cross(match.dir);
                            energy += opts.vectorLineWeight * match.weight * (ru * ru + rv * rv);
                        }
                    }
                }
            }
        }
    }

    std::vector<Vec2> P(mesh.vertices.size());
    for (size_t i = 0; i < P.size(); ++i) P[i] = mesh.vertices[i].P;

    // Must mirror addSmoothnessTerms's per-triple edgeRelaxFactor scaling
    // exactly, for the same line-search-consistency reason noted above.
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i1 = mesh.idx(r, c);
            double w = opts.smoothWeightGeom *
                edgeRelaxFactor(target, P[i1], opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            Vec2 d = P[mesh.idx(r, c - 1)] - P[i1] * 2.0 + P[mesh.idx(r, c + 1)];
            energy += w * (d.x * d.x + d.y * d.y);
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i1 = mesh.idx(r, c);
            double w = opts.smoothWeightGeom *
                edgeRelaxFactor(target, P[i1], opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            Vec2 d = P[mesh.idx(r - 1, c)] - P[i1] * 2.0 + P[mesh.idx(r + 1, c)];
            energy += w * (d.x * d.x + d.y * d.y);
        }
    }

    // Soft prior pulling the free Pu/Pv tangents toward the position-implied
    // finite-difference estimate -- must mirror addTangentPriorTerms above
    // exactly, for the same reason computeGeometryEnergy has to mirror the
    // rest of the GN assembly (see this function's header comment).
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 0; c < mesh.cols; ++c) {
            const MeshVertex& mv = mesh.at(r, c);
            Vec2 tu = mesh.tangentU(r, c), tv = mesh.tangentV(r, c);
            Vec2 du = mv.Pu - tu, dv = mv.Pv - tv;
            energy += opts.geomTangentPriorWeight * (du.x * du.x + du.y * du.y + dv.x * dv.x + dv.y * dv.y);
        }
    }

    // Boundary control points should be free to slide ALONG their spline
    // (Sec 4: "control points on the boundary only move along the
    // splines") -- only the off-curve (normal) component should be
    // penalized, never the along-curve (tangential) one. Penalizing the
    // full 2D displacement from a fixed re-projected point, as an earlier
    // version of this did, resists tangential motion just as strongly as
    // normal motion -- which is NOT "free to slide", it's "stay near this
    // one point", and pins boundary vertices in place far more than the
    // paper intends. Must mirror the GN boundary term below exactly.
    for (const auto& mv : mesh.vertices) {
        if (!mv.isBoundary) continue;
        const CubicBezier& spline = mesh.boundary[mv.boundarySide];
        Vec2 t = spline.eval(mv.boundaryT);
        Vec2 normal = Vec2{-spline.evalDeriv(mv.boundaryT).y, spline.evalDeriv(mv.boundaryT).x}.normalized();
        double d = (mv.P - t).dot(normal);
        energy += opts.boundaryWeight * (d * d);
    }

    return energy;
}

} // namespace

// See MeshOptimizer.h's comment on this declaration for why it's a real
// function rather than leaving every caller to test GMCORE_WITH_CERES
// itself.
bool builtWithCeres() {
#ifdef GMCORE_WITH_CERES
    return true;
#else
    return false;
#endif
}

void solveColorExact(GradientMesh& mesh, const Image& target, const OptimizerOptions& opts) {
    int numV = (int)mesh.vertices.size();
    int n = std::max(2, opts.samplesPerPatchEdge);
    double duv = 1.0 / (n * n);

    SparseBlockMatrix H;
    H.init(4, numV);
    H.reserveBlocks(buildMeshBlockPattern(mesh)); // see that function's comment
    std::vector<double> gR(numV * 4, 0.0), gG(numV * 4, 0.0), gB(numV * 4, 0.0);
    std::vector<double> x0R(numV * 4), x0G(numV * 4), x0B(numV * 4);
    for (int i = 0; i < numV; ++i) {
        const MeshVertex& mv = mesh.vertices[i];
        Color c[4] = {mv.C, mv.Cu, mv.Cv, mv.Cuv};
        for (int k = 0; k < 4; ++k) {
            x0R[i * 4 + k] = c[k].r; x0G[i * 4 + k] = c[k].g; x0B[i * 4 + k] = c[k].b;
        }
    }

    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            for (int i = 0; i <= n; ++i) {
                double v = double(i) / n;
                for (int j = 0; j <= n; ++j) {
                    double u = double(j) / n;
                    Vec2 pos = mesh.evalPos(pr, pc, u, v);
                    Color cmesh = mesh.evalColor(pr, pc, u, v);
                    Color ctarget = target.sampleBilinear(pos.x, pos.y);
                    double w = areaWeightAt(mesh, pr, pc, u, v, duv);
                    PatchWeights pw = PatchWeights::at(u, v);

                    // Fixed size (4 corners * 4 color kinds), always fully
                    // filled below -- std::array instead of the old
                    // heap-allocated std::vector avoids an allocation on
                    // every one of these (n+1)^2 samples per patch (this
                    // loop's whole point is exact-solving color, so it
                    // dominates solveColorExact's cost the same way the
                    // data term dominates the geometry GN step).
                    std::array<RowEntry, 16> row;
                    int rowCount = 0;
                    for (int a = 0; a < 2; ++a) {
                        for (int b = 0; b < 2; ++b) {
                            int base = (a * 2 + b) * 4;
                            int vert = mesh.idx(pr + b, pc + a);
                            for (int k = 0; k < 4; ++k)
                                row[rowCount++] = {vert, k, pw.w[base + k]};
                        }
                    }
                    double r0R = cmesh.r - ctarget.r;
                    double r0G = cmesh.g - ctarget.g;
                    double r0B = cmesh.b - ctarget.b;
                    accumulateGNRow(H, gR, row.data(), rowCount, r0R, w);
                    accumulateGNRow(H, gG, row.data(), rowCount, r0G, w);
                    accumulateGNRow(H, gB, row.data(), rowCount, r0B, w);
                }
            }
        }
    }

    // Smoothness on base color (kind 0) + ridge on Cu,Cv,Cuv (kinds 1..3).
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
            std::vector<RowEntry> row = {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}};
            double r0r = mesh.vertices[i0].C.r - 2 * mesh.vertices[i1].C.r + mesh.vertices[i2].C.r;
            double r0g = mesh.vertices[i0].C.g - 2 * mesh.vertices[i1].C.g + mesh.vertices[i2].C.g;
            double r0b = mesh.vertices[i0].C.b - 2 * mesh.vertices[i1].C.b + mesh.vertices[i2].C.b;
            accumulateGNRow(H, gR, row, r0r, opts.smoothWeightColor);
            accumulateGNRow(H, gG, row, r0g, opts.smoothWeightColor);
            accumulateGNRow(H, gB, row, r0b, opts.smoothWeightColor);
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
            std::vector<RowEntry> row = {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}};
            double r0r = mesh.vertices[i0].C.r - 2 * mesh.vertices[i1].C.r + mesh.vertices[i2].C.r;
            double r0g = mesh.vertices[i0].C.g - 2 * mesh.vertices[i1].C.g + mesh.vertices[i2].C.g;
            double r0b = mesh.vertices[i0].C.b - 2 * mesh.vertices[i1].C.b + mesh.vertices[i2].C.b;
            accumulateGNRow(H, gR, row, r0r, opts.smoothWeightColor);
            accumulateGNRow(H, gG, row, r0g, opts.smoothWeightColor);
            accumulateGNRow(H, gB, row, r0b, opts.smoothWeightColor);
        }
    }
    for (int i = 0; i < numV; ++i) {
        for (int k = 1; k < 4; ++k) {
            std::vector<RowEntry> row = {{i, k, 1}};
            Color c[4] = {mesh.vertices[i].C, mesh.vertices[i].Cu, mesh.vertices[i].Cv, mesh.vertices[i].Cuv};
            accumulateGNRow(H, gR, row, c[k].r, opts.colorDerivRidge);
            accumulateGNRow(H, gG, row, c[k].g, opts.colorDerivRidge);
            accumulateGNRow(H, gB, row, c[k].b, opts.colorDerivRidge);
        }
    }

    auto deltaR = solveSPD_PCG(H, gR, std::vector<double>(numV * 4, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);
    auto deltaG = solveSPD_PCG(H, gG, std::vector<double>(numV * 4, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);
    auto deltaB = solveSPD_PCG(H, gB, std::vector<double>(numV * 4, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);

    for (int i = 0; i < numV; ++i) {
        MeshVertex& mv = mesh.vertices[i];
        Color* slots[4] = {&mv.C, &mv.Cu, &mv.Cv, &mv.Cuv};
        for (int k = 0; k < 4; ++k) {
            slots[k]->r = x0R[i * 4 + k] + deltaR[i * 4 + k];
            slots[k]->g = x0G[i * 4 + k] + deltaG[i * 4 + k];
            slots[k]->b = x0B[i * 4 + k] + deltaB[i * 4 + k];
        }
    }
}

void MeshOptimizer::optimizeAtCurrentResolution(GradientMesh& mesh, const Image& target,
                                                 const std::vector<VectorLine>& vectorLines,
                                                 const OptimizerOptions& opts,
                                                 const OptimizerProgressCallback& cb, int level,
                                                 int totalLevels) {
    int numV = (int)mesh.vertices.size();
    int n = std::max(2, opts.samplesPerPatchEdge);
    double duv = 1.0 / (n * n);

    double lambda = opts.geomDampingInitial;

    // Convergence tracking for the early-exit check at the bottom of this
    // loop -- see OptimizerOptions::outerConvergenceRelTol's comment for
    // why this exists (outerIterationsPerLevel is now a much higher
    // ceiling than before, and this is what keeps an already-converged
    // case from always burning the whole thing).
    double prevOuterEnergy = std::numeric_limits<double>::infinity();
    // Consecutive-stall counter -- see the early-exit block below for why a
    // SINGLE flat outer iteration must not be enough to stop the loop: it
    // can just mean lambda transiently ratcheted up after an unlucky
    // rejected GN step, not that a real local optimum was reached.
    int stalledOuterIters = 0;

    for (int outer = 0; outer < opts.outerIterationsPerLevel; ++outer) {
        // 1) Re-project boundary vertices onto their spline (soft constraint target).
        for (auto& v : mesh.vertices) {
            if (!v.isBoundary) continue;
            v.boundaryT = mesh.boundary[v.boundarySide].closestT(v.P);
        }

        // 2+3) Either solve color (closed form, below) then geometry
        // (Gauss-Newton or useCeresGeometry, further below) separately --
        // the block-coordinate-descent scheme this file has always used --
        // or replace BOTH steps with a single fully-joint Ceres solve over
        // position, tangents AND all four free color unknowns at once.
        // See OptimizerOptions::useCeresJoint for the motivation (closing
        // the remaining gap to the paper's Fig. 4 pinch that
        // useCeresGeometry alone didn't close). `jointSolvedByCeres` gates
        // both the color block and the geometry block below so they
        // become no-ops this outer iteration when the joint solve already
        // did the work -- same explicit-flag pattern as
        // `geometrySolvedByCeres` already uses for useCeresGeometry.
        bool jointSolvedByCeres = false;
#ifdef GMCORE_WITH_CERES
        if (opts.useCeresJoint) {
            optimizeJointCeres(mesh, target, vectorLines, opts);
            jointSolvedByCeres = true;
        }
#else
        if (opts.useCeresJoint) {
            static bool warnedJoint = false;
            if (!warnedJoint) {
                std::fprintf(stderr,
                    "OptimizerOptions::useCeresJoint=true but this binary was built "
                    "without Ceres (GMCORE_WITH_CERES not defined) -- falling back to "
                    "the hand-rolled color+geometry solve.\n");
                warnedJoint = true;
            }
        }
#endif

        // 2) Solve for colors exactly (data term is linear in color unknowns).
        if (!jointSolvedByCeres) {
            solveColorExact(mesh, target, opts);
        }

        // 3) Refine geometry -- position P AND the free tangents Pu, Pv.
        // Either the hand-rolled damped Gauss-Newton + backtracking below
        // (default, dependency-free), or -- opt-in, see
        // OptimizerOptions::useCeresGeometry -- a ceres::Problem solve
        // over the exact same residuals (MeshOptimizerCeres.cpp), when
        // this binary was built with Ceres available. Both mutate
        // mesh.vertices[*].P/Pu/Pv in place; nothing below this block
        // changes based on which path ran. `geometrySolvedByCeres` gates
        // the hand-rolled loop below so it's a no-op when Ceres already
        // did the work this outer iteration -- kept as an explicit flag
        // (rather than if/else across the #ifdef) so this stays easy to
        // read and can't silently run both paths.
        bool geometrySolvedByCeres = jointSolvedByCeres; // joint solve already did geometry too
#ifdef GMCORE_WITH_CERES
        if (!jointSolvedByCeres && opts.useCeresGeometry) {
            optimizeGeometryCeres(mesh, target, vectorLines, opts);
            geometrySolvedByCeres = true;
        }
#else
        if (!jointSolvedByCeres && opts.useCeresGeometry) {
            static bool warned = false;
            if (!warned) {
                std::fprintf(stderr,
                    "OptimizerOptions::useCeresGeometry=true but this binary was built "
                    "without Ceres (GMCORE_WITH_CERES not defined) -- falling back to the "
                    "hand-rolled geometry solver.\n");
                warned = true;
            }
        }
#endif
        // Each vertex contributes a 6-wide unknown block: subs 0,1 =
        // P.x,P.y, 2,3 = Pu.x,Pu.y, 4,5 = Pv.x,Pv.y. Twist (Puv) is fixed
        // at {0,0} per the paper (Sec 3: "In practice, the values of muv
        // are usually set to zero") -- it is not a free unknown here; see
        // GradientMesh::geomCorner.
        for (int gi = 0; !geometrySolvedByCeres && gi < opts.geomGaussNewtonItersPerOuter; ++gi) {
            // Re-project boundary vertices onto their spline EVERY GN
            // sub-iteration, not just once per outer iteration (step 1,
            // above, only runs once per outer loop). The boundary term
            // below is a LINEARIZATION around (target, normal) taken at the
            // vertex's current boundaryT -- valid only for small
            // displacements from that point. Found via a stress test: a
            // strong vector-line pull (this file's new Sec-4.2-faithful
            // term) can move a boundary vertex far enough in one GN step
            // that the stale target/normal from the top of the outer
            // iteration badly mis-linearizes the constraint, and the
            // vertex can drift outside the image entirely over a few GN
            // sub-iterations before the next outer-iteration reprojection
            // catches it. Re-running closestT() here (global 40-sample +
            // Newton search, see BezierSpline.h -- correct regardless of
            // how far P has drifted) keeps the linearization point current
            // within an outer iteration too, at the same per-substep
            // frequency backtracking already re-checks the true energy at.
            for (auto& v : mesh.vertices) {
                if (!v.isBoundary) continue;
                v.boundaryT = mesh.boundary[v.boundarySide].closestT(v.P);
            }

            SparseBlockMatrix H;
            H.init(6, numV);
            H.reserveBlocks(buildMeshBlockPattern(mesh)); // see that function's comment
            std::vector<double> g(numV * 6, 0.0);
            std::vector<Vec2> P(numV), Pu(numV), Pv(numV);
            for (int i = 0; i < numV; ++i) {
                P[i] = mesh.vertices[i].P;
                Pu[i] = mesh.vertices[i].Pu;
                Pv[i] = mesh.vertices[i].Pv;
            }

            bool hasLines = !vectorLines.empty();

            for (int pr = 0; pr < mesh.rows - 1; ++pr) {
                for (int pc = 0; pc < mesh.cols - 1; ++pc) {
                    for (int i = 0; i <= n; ++i) {
                        double v = double(i) / n;
                        for (int j = 0; j <= n; ++j) {
                            double u = double(j) / n;
                            Vec2 dU, dV;
                            Vec2 pos = mesh.evalPos(pr, pc, u, v, &dU, &dV);
                            Color cmesh = mesh.evalColor(pr, pc, u, v);
                            Color ctarget = target.sampleBilinear(pos.x, pos.y);
                            ColorGrad grad = target.sampleGradient(pos.x, pos.y);
                            double w = areaWeightAt(mesh, pr, pc, u, v, duv);
                            PatchWeights pw = PatchWeights::at(u, v);

                            // corner vertex + weight for each of the 3 FREE
                            // Hermite kinds this patch corner contributes to
                            // pos(u,v): Value(P, sub-pair 0,1), TangentU(Pu,
                            // sub-pair 2,3), TangentV(Pv, sub-pair 4,5) --
                            // reusing RowEntry, with `sub` repurposed here to
                            // mean "which sub-pair" (0/1/2), not a literal
                            // block sub-index (buildChanneled below expands
                            // it to the real x/y sub-indices).
                            // Fixed size (4 corners * 3 free Hermite kinds),
                            // always fully filled below -- std::array
                            // instead of a heap-allocated std::vector: this
                            // is the single hottest loop in the whole
                            // optimizer (patches * samples * GN sub-iters *
                            // outer iters * pyramid levels), so avoiding an
                            // allocation here (and the 3 more below, one per
                            // color channel via buildChanneled) matters far
                            // more than anywhere else this same pattern
                            // appears in this file.
                            std::array<RowEntry, 12> rowCorners;
                            int rcCount = 0;
                            for (int a = 0; a < 2; ++a) {
                                for (int b = 0; b < 2; ++b) {
                                    int base = (a * 2 + b) * 4;
                                    int vert = mesh.idx(pr + b, pc + a);
                                    rowCorners[rcCount++] = {vert, 0, pw.w[base + 0]}; // P
                                    rowCorners[rcCount++] = {vert, 1, pw.w[base + 1]}; // Pu
                                    rowCorners[rcCount++] = {vert, 2, pw.w[base + 2]}; // Pv
                                }
                            }
                            double r0r = cmesh.r - ctarget.r;
                            double r0g = cmesh.g - ctarget.g;
                            double r0b = cmesh.b - ctarget.b;

                            // Returns a stack-allocated (not heap) array --
                            // same 24 entries, same order as the old
                            // std::vector version, just without the heap
                            // round-trip. .data() on the returned-by-value
                            // temporary is valid for the lifetime of the
                            // accumulateGNRow call it's passed into (the
                            // temporary lives until the end of that full
                            // expression).
                            auto buildChanneled = [&](double gx, double gy) {
                                std::array<RowEntry, 24> row;
                                int n2 = 0;
                                for (int k = 0; k < rcCount; ++k) {
                                    const RowEntry& rc = rowCorners[k];
                                    int subX = rc.sub * 2 + 0;
                                    int subY = rc.sub * 2 + 1;
                                    row[n2++] = {rc.vertex, subX, -gx * rc.coeff};
                                    row[n2++] = {rc.vertex, subY, -gy * rc.coeff};
                                }
                                return row;
                            };
                            // geomDataWeight scales ONLY this photometric term -- see
                            // OptimizerOptions::geomDataWeight's comment -- not the
                            // vector-line term just below, which has its own weight.
                            double wData = w * opts.geomDataWeight;
                            accumulateGNRow(H, g, buildChanneled(grad.dx.r, grad.dy.r).data(), 24, r0r, wData);
                            accumulateGNRow(H, g, buildChanneled(grad.dx.g, grad.dy.g).data(), 24, r0g, wData);
                            accumulateGNRow(H, g, buildChanneled(grad.dx.b, grad.dy.b).data(), 24, r0b, wData);

                            // Vector-line guided term (Sec 4.2): penalizes
                            // the component of the ANALYTIC surface tangent
                            // (dU=d(pos)/du, dV=d(pos)/dv -- already
                            // computed above for free via evalPos's output
                            // params) that is perpendicular to the nearest
                            // guide line's direction, Gaussian-weighted by
                            // distance to that line (see
                            // nearestVectorLineField). Evaluated at this
                            // same dense per-patch sample grid as the data
                            // term -- unlike an earlier version of this
                            // term, which only ever checked the coarse
                            // straight edge between two adjacent control
                            // points at its midpoint, with a flat pixel-
                            // radius cutoff, and never touched Pu/Pv at
                            // all. d(dU)/d(corner param) uses PatchWeights'
                            // `wu` array (the u-partial of the Hermite
                            // basis weight) exactly the way the data term
                            // above uses the plain `w` array for d(pos);
                            // d(dV)/d(corner param) likewise uses `wv`.
                            if (hasLines) {
                                VectorLineMatch match = nearestVectorLineField(vectorLines, pos);
                                if (match.found) {
                                    double weight = opts.vectorLineWeight * match.weight;
                                    double ru = dU.cross(match.dir);
                                    double rv = dV.cross(match.dir);
                                    // Fixed size (4 corners * 6 subs = 24),
                                    // always fully filled below -- same
                                    // std::array-instead-of-std::vector
                                    // rationale as rowCorners/buildChanneled
                                    // above (this term runs at the same
                                    // per-sample frequency as the data term
                                    // whenever vector lines are in use).
                                    // NOTE: this was originally (wrongly)
                                    // sized 12, silently overflowing by 12
                                    // RowEntry writes past the end of each
                                    // array on every sample once the b/a
                                    // loop below reached its 3rd/4th corner
                                    // -- caught by -Wall's "iteration 1
                                    // invokes undefined behavior" warning
                                    // and fixed before this refactor shipped.
                                    std::array<RowEntry, 24> rowU, rowV;
                                    int ruCount = 0, rvCount = 0;
                                    for (int a = 0; a < 2; ++a) {
                                        for (int b = 0; b < 2; ++b) {
                                            int base = (a * 2 + b) * 4;
                                            int vert = mesh.idx(pr + b, pc + a);
                                            double wuP = pw.wu[base + 0], wuPu = pw.wu[base + 1], wuPv = pw.wu[base + 2];
                                            double wvP = pw.wv[base + 0], wvPu = pw.wv[base + 1], wvPv = pw.wv[base + 2];
                                            rowU[ruCount++] = {vert, 0,  wuP  * match.dir.y};
                                            rowU[ruCount++] = {vert, 1, -wuP  * match.dir.x};
                                            rowU[ruCount++] = {vert, 2,  wuPu * match.dir.y};
                                            rowU[ruCount++] = {vert, 3, -wuPu * match.dir.x};
                                            rowU[ruCount++] = {vert, 4,  wuPv * match.dir.y};
                                            rowU[ruCount++] = {vert, 5, -wuPv * match.dir.x};
                                            rowV[rvCount++] = {vert, 0,  wvP  * match.dir.y};
                                            rowV[rvCount++] = {vert, 1, -wvP  * match.dir.x};
                                            rowV[rvCount++] = {vert, 2,  wvPu * match.dir.y};
                                            rowV[rvCount++] = {vert, 3, -wvPu * match.dir.x};
                                            rowV[rvCount++] = {vert, 4,  wvPv * match.dir.y};
                                            rowV[rvCount++] = {vert, 5, -wvPv * match.dir.x};
                                        }
                                    }
                                    accumulateGNRow(H, g, rowU.data(), ruCount, ru, weight);
                                    accumulateGNRow(H, g, rowV.data(), rvCount, rv, weight);
                                }
                            }
                        }
                    }
                }
            }

            addSmoothnessTerms(H, g, mesh, P, target, opts);
            addTangentPriorTerms(H, g, mesh, opts.geomTangentPriorWeight);

            // Normal-only soft constraint -- see computeGeometryEnergy's
            // comment above for why (must mirror it exactly). A single
            // scalar residual (displacement projected onto the curve
            // normal) instead of two independent x/y residuals, so
            // along-curve motion is completely free and only off-curve
            // drift is resisted.
            for (int i = 0; i < numV; ++i) {
                const MeshVertex& mv = mesh.vertices[i];
                if (!mv.isBoundary) continue;
                const CubicBezier& spline = mesh.boundary[mv.boundarySide];
                Vec2 target_ = spline.eval(mv.boundaryT);
                Vec2 normal = Vec2{-spline.evalDeriv(mv.boundaryT).y, spline.evalDeriv(mv.boundaryT).x}.normalized();
                double r0 = (mv.P - target_).dot(normal);
                accumulateGNRow(H, g, {{i, 0, normal.x}, {i, 1, normal.y}}, r0, opts.boundaryWeight);
            }

            // Levenberg damping (scale-aware: proportional to each diagonal entry).
            for (int i = 0; i < numV; ++i) {
                const double* diag = H.findBlock(i, i);
                for (int k = 0; k < 6; ++k) {
                    double dk = diag ? diag[k * 6 + k] : 1.0;
                    H.addScalar(i, i, k, k, lambda * std::max(dk, 1e-6));
                }
            }

            auto delta = solveSPD_PCG(H, g, std::vector<double>(numV * 6, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);

            // Hard-fix the 4 mesh corners' POSITION (sub-indices 0,1 = P.x,P.y)
            // -- zero whatever the linear solve proposed for them, at every
            // backtracking alpha, so they never move at all. Pu/Pv (subs
            // 2..5) are left free -- only position is being hard-fixed here.
            //
            // Each corner is the exact junction of TWO boundary splines, not
            // an interior point of a single one -- "slide along the spline"
            // (Sec 4) is ambiguous there (which of the two splines?), and
            // GradientMesh::buildInitial's boundarySide assignment picks one
            // somewhat arbitrarily (see its if/else-if chain). The soft
            // normal-only boundary constraint (see computeGeometryEnergy's
            // comment) is only a valid linearization for a vertex that
            // actually lives on the interior of its assigned curve; at a
            // t=0/1 endpoint shared with a DIFFERENT curve, a large step can
            // end up "tangential" with respect to the wrong curve's
            // direction and drift arbitrarily far before the soft penalty
            // pushes back -- diagnosed this way after the new vector-line
            // term's stress test moved a corner vertex outside the image
            // bounds even at low vectorLineWeight (see README). Corners are
            // also structurally redundant as free unknowns: buildInitial()
            // sets them to the boundary curves' own endpoints (P00/P10/P01/
            // P11) exactly, so there's nothing to usefully solve for there
            // anyway -- hard-fixing removes the failure mode entirely rather
            // than just mitigating it. Simplest correct way to pin specific
            // unknowns without restructuring the sparse solve into a smaller
            // system: solve as if free, then discard the proposed delta for
            // exactly those components before applying.
            // (idx() may coincide for degenerate 1-row/1-col meshes -- listing
            // all 4 corners and zeroing each is harmless even if some alias.)
            for (int i : {mesh.idx(0, 0), mesh.idx(0, mesh.cols - 1),
                          mesh.idx(mesh.rows - 1, 0), mesh.idx(mesh.rows - 1, mesh.cols - 1)}) {
                delta[i * 6 + 0] = 0.0;
                delta[i * 6 + 1] = 0.0;
            }
#ifdef GMCORE_DEBUG_GEOM
            double maxDelta = 0; for (double d : delta) maxDelta = std::max(maxDelta, std::abs(d));
            double gNorm = 0; for (double v : g) gNorm += v*v; gNorm = std::sqrt(gNorm);
#endif

            // Backtracking must check the step against the SAME objective the
            // step was computed to reduce (data + vector-line + smoothness +
            // tangent-prior + boundary energy, at the same sample density) --
            // not a cheaper, differently-sampled RMSE proxy, which can
            // disagree with it and reject perfectly good steps. See
            // computeGeometryEnergy's comment above for why this matters.
            double baseEnergy = computeGeometryEnergy(mesh, target, vectorLines, opts);
            double alpha = 1.0;
            bool improved = false;
            for (int tries = 0; tries < 4; ++tries) {
                for (int i = 0; i < numV; ++i) {
                    mesh.vertices[i].P.x  = P[i].x  + alpha * delta[i * 6 + 0];
                    mesh.vertices[i].P.y  = P[i].y  + alpha * delta[i * 6 + 1];
                    mesh.vertices[i].Pu.x = Pu[i].x + alpha * delta[i * 6 + 2];
                    mesh.vertices[i].Pu.y = Pu[i].y + alpha * delta[i * 6 + 3];
                    mesh.vertices[i].Pv.x = Pv[i].x + alpha * delta[i * 6 + 4];
                    mesh.vertices[i].Pv.y = Pv[i].y + alpha * delta[i * 6 + 5];
                }
                double newEnergy = computeGeometryEnergy(mesh, target, vectorLines, opts);
                if (newEnergy <= baseEnergy) { improved = true; break; }
                alpha *= 0.4;
            }
#ifdef GMCORE_DEBUG_GEOM
            std::fprintf(stderr, "    [GN gi=%d] lambda=%.4g |g|=%.4g maxDelta=%.4g improved=%d alpha=%.3g\n",
                         gi, lambda, gNorm, maxDelta, (int)improved, alpha);
#endif
            if (!improved) {
                for (int i = 0; i < numV; ++i) {
                    mesh.vertices[i].P = P[i]; mesh.vertices[i].Pu = Pu[i]; mesh.vertices[i].Pv = Pv[i]; // revert
                }
                lambda = std::min(lambda * 4.0, 1e6);
            } else {
                lambda = std::max(lambda * 0.6, 1e-8);
            }
        }

        double rmse = mesh.reconstructionRMSE(target, opts.samplesPerPatchEdge);
        // target.width/height (this LEVEL's downsampled size, not the full-
        // res image) -- see OptimizerProgress::levelWidth/levelHeight's
        // comment for why callers need this.
        if (cb) cb({level, totalLevels, outer, opts.outerIterationsPerLevel, rmse, target.width, target.height});

        // Early-exit once several outer iterations IN A ROW show relative
        // improvement in the composite geometry energy (data + vector-line
        // + smoothness + tangent-prior + boundary -- the same objective
        // backtracking already checks every GN sub-iteration) below
        // outerConvergenceRelTol. Skipped on the very first iteration
        // (nothing to compare against yet) and disabled entirely when
        // outerConvergenceRelTol <= 0. See OptimizerOptions's comment for
        // why this exists: outerIterationsPerLevel is a much higher
        // ceiling than before specifically so genuinely-still-improving
        // cases aren't cut off early (see that field's comment for the
        // regression that motivated this), and this is what keeps an
        // already-converged case (e.g. the smooth synthetic sphere) from
        // then always burning the whole, now much larger, ceiling.
        //
        // Requiring a RUN of stalled iterations (not just one) fixes a real
        // bug found after a follow-up report that re-running the optimizer
        // on its own output still kept improving RMSE even with this
        // early-exit in place: when a GN sub-iteration's step is rejected
        // (see the backtracking block above), the mesh is reverted and
        // `lambda` is bumped 4x -- the geometry genuinely did not change
        // that outer iteration, so its energy is (near-)identical to the
        // previous one. A single-iteration check reads that as "converged"
        // and stops immediately, even though the real cause is just that
        // `lambda` transiently got too large for THIS outer iteration's
        // linearization -- the very next outer iteration, still working
        // from the same (unimproved) point but now with more damping
        // headroom already spent, can easily find a good step once `lambda`
        // works back down. Calling optimizeCoarseToFine() a second time
        // resets `lambda` to opts.geomDampingInitial and gives the solve
        // exactly that fresh chance -- which is why "run it twice" kept
        // helping even after the single-iteration version of this check was
        // added. Requiring outerConvergencePatience consecutive stalled
        // iterations before actually stopping means a transient lambda
        // ratchet no longer looks identical to real convergence: a lone
        // stalled iteration just increments the counter and the loop keeps
        // going, and the counter resets the moment any iteration improves
        // enough again.
        if (opts.outerConvergenceRelTol > 0.0 && outer > 0) {
            double energyNow = computeGeometryEnergy(mesh, target, vectorLines, opts);
            double denom = std::max(prevOuterEnergy, 1e-12);
            double relImprovement = (prevOuterEnergy - energyNow) / denom;
            prevOuterEnergy = energyNow;
            if (relImprovement < opts.outerConvergenceRelTol) {
                if (++stalledOuterIters >= std::max(1, opts.outerConvergencePatience)) break;
            } else {
                stalledOuterIters = 0;
            }
        } else if (opts.outerConvergenceRelTol > 0.0) {
            prevOuterEnergy = computeGeometryEnergy(mesh, target, vectorLines, opts);
        }
    }
}

void MeshOptimizer::optimizeCoarseToFine(GradientMesh& mesh, const Image& fullResTarget,
                                          const std::vector<VectorLine>& vectorLinesFullRes,
                                          int numPyramidLevels, const OptimizerOptions& opts,
                                          const OptimizerProgressCallback& cb) {
    auto levels = Image::buildPyramid(fullResTarget, std::max(1, numPyramidLevels));
    int L = (int)levels.size();

    // See OptimizerOptions::pyramidRestarts for why this outer loop exists:
    // a single descend-to-coarsest/climb-to-finest sweep reaches A local
    // optimum of this non-convex block-coordinate-descent problem, not
    // necessarily the best one reachable -- re-descending to the coarsest
    // level and re-climbing from an already-refined mesh can (and does,
    // measurably) find a marginally better one. Each iteration here is
    // byte-for-byte the same sweep this function always did; the only
    // change for the default pyramidRestarts=1 is that this loop now runs
    // once instead of the sweep being inline, so existing callers see zero
    // behavior change.
    int restarts = std::max(1, opts.pyramidRestarts);
    for (int restart = 0; restart < restarts; ++restart) {
        // Move the (full-resolution) mesh down to the coarsest level.
        double sxDown = double(levels[L - 1].width) / double(levels[0].width);
        double syDown = double(levels[L - 1].height) / double(levels[0].height);
        mesh.scalePositions(sxDown, syDown);

        for (int li = L - 1; li >= 0; --li) {
            std::vector<VectorLine> scaledLines;
            double sx = double(levels[li].width) / double(levels[0].width);
            double sy = double(levels[li].height) / double(levels[0].height);
            scaledLines.reserve(vectorLinesFullRes.size());
            for (const auto& line : vectorLinesFullRes) {
                VectorLine sl;
                sl.points.reserve(line.points.size());
                for (const auto& p : line.points) sl.points.push_back({p.x * sx, p.y * sy});
                scaledLines.push_back(std::move(sl));
            }
            // levelOpts is currently just a copy of opts: the old per-level
            // vectorLineInfluenceRadius rescaling (levelOpts.vectorLineInfluenceRadius
            // *= max(sx,sy)) is gone -- the current vector-line term derives its
            // band width from each (already per-axis-scaled) line's own polyline
            // length every call (see nearestVectorLineField), so no separate
            // level-dependent rescaling of a flat radius is needed any more.
            const OptimizerOptions& levelOpts = opts;

            optimizeAtCurrentResolution(mesh, levels[li], scaledLines, levelOpts, cb, li, L);

            if (li > 0) {
                double sxUp = double(levels[li - 1].width) / double(levels[li].width);
                double syUp = double(levels[li - 1].height) / double(levels[li].height);
                mesh.scalePositions(sxUp, syUp);
            }
        }
    }
}

} // namespace gmcore
