// MeshOptimizerCeres.cpp — optional Ceres-based replacement for the
// geometry Gauss-Newton block in MeshOptimizer.cpp's optimizeAtCurrentResolution
// (see OptimizerOptions::useCeresGeometry). Compiles to nothing (this
// whole file is a no-op translation unit) unless GMCORE_WITH_CERES is
// defined, which CMakeLists.txt only does when find_package(Ceres)
// actually succeeds -- so the default, dependency-free build is
// completely unaffected by this file's existence.
//
// Every residual/Jacobian here is a direct transcription of the
// corresponding hand-rolled term in MeshOptimizer.cpp's anonymous
// namespace (addSmoothnessTerms, addTangentPriorTerms, the boundary and
// vector-line blocks inside the geometry GN loop, and areaWeightAt for
// the data term) -- NOT a re-derivation. Each one was cross-checked
// before being written here: the data term against ceres::GradientChecker
// via spike/ceres_geom_spike.cpp (see that file's history -- one real bug
// found and fixed: the per-sample area weight must be frozen at the
// linearization point, not re-differentiated through the trial
// parameters), and the other four terms (smoothness, tangent-prior,
// boundary, vector-line) against a standalone finite-difference harness
// (not checked into the repo -- see the session's commit message for
// this file) before being transcribed into ceres::CostFunction form here.
//
// Weighting/freezing convention, consistently applied to every term
// below: any quantity that depends on the CURRENT mesh geometry but isn't
// itself one of this CostFunction's free parameters (the per-sample area
// weight, the anisotropic edge-relax factor, the tangent-prior's
// finite-difference target tu/tv, the vector-line direction, the
// boundary's target point) is evaluated ONCE from a snapshot of the mesh,
// not re-evaluated as Ceres's own internal LM iterations move the trial
// parameters. optimizeGeometryCeres() re-snapshots and re-solves
// opts.geomGaussNewtonItersPerOuter times (see ceresSolveOnce's comment)
// so this freezing happens at the same cadence as the hand-rolled path's
// GN sub-iterations, not once per outer iteration -- an earlier version
// of this file froze only once per outer iteration and let Ceres run up
// to 30 internal iterations against those now-stale weights, which
// produced a *worse* result than the hand-rolled path on the 25x25
// gradient.png regression (reconstruction RMSE climbing for several
// outer iterations instead of decreasing). Not otherwise a new
// approximation introduced by this port -- the hand-rolled path already
// freezes these same quantities at GN-sub-iteration granularity.
#ifdef GMCORE_WITH_CERES

#include "gmcore/MeshOptimizer.h"
#include "gmcore/FergusonPatch.h"

#include <ceres/ceres.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace gmcore {
namespace {

double areaWeightAt(const GradientMesh& mesh, int pr, int pc, double u, double v, double duv) {
    Vec2 dU, dV;
    mesh.evalPos(pr, pc, u, v, &dU, &dV);
    double a = std::abs(dU.cross(dV)) * duv;
    return std::max(a, 1e-6);
}

double edgeRelaxFactor(const Image& target, const Vec2& p, double edgeGain, double minFactor) {
    ColorGrad grad = target.sampleGradient(p.x, p.y);
    double mag2 = grad.dx.r * grad.dx.r + grad.dx.g * grad.dx.g + grad.dx.b * grad.dx.b +
                  grad.dy.r * grad.dy.r + grad.dy.g * grad.dy.g + grad.dy.b * grad.dy.b;
    double mag = std::sqrt(mag2);
    double factor = 1.0 / (1.0 + edgeGain * mag);
    return std::max(factor, minFactor);
}

// Direct transcription of MeshOptimizer.cpp's nearestVectorLineField (same
// function, anonymous-namespace/file-local here so not shared directly) --
// see that file's comment for the Sec 4.2 formula this implements: finds
// the single globally-nearest (line segment, point) pair, derives THAT
// line's own sigma = (its own polyline length / 5) / 3, hard-cuts outside
// a band of width 3*sigma, and returns a Gaussian falloff weight. Replaces
// the old nearestVectorLineDir (flat global pixel radius, discrete
// edge-midpoint only) that used to live here.
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
        double sigma = (totalLen / 5.0) / 3.0;
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
    if (bestSigma <= 1e-9) return result;
    double d = std::sqrt(bestD2);
    double bandWidth = bestSigma * 3.0;
    if (d > bandWidth) return result;
    result.found = true;
    result.dir = bestDir;
    result.weight = std::exp(-(d * d) / (2.0 * bestSigma * bestSigma));
    return result;
}

// Direct transcription of MeshOptimizer.cpp's computeGeometryEnergy (same
// function name there, anonymous-namespace/file-local so not shared
// directly): the TRUE geometry energy, with every weight/target
// (area weight, edge-relax factor, tangent-prior tu/tv, vector-line
// direction) evaluated FRESH from whatever mesh is passed in -- no
// freezing at all here, unlike ceresSolveOnce's residuals. This exists
// for exactly the reason MeshOptimizer.cpp's version does (see its own
// header comment): a step must be judged against the real energy it's
// supposed to be decreasing, not a cheaper/frozen proxy, or a genuinely
// energy-increasing step can look like an improvement. Needed here
// because ceresSolveOnce has no equivalent of the hand-rolled path's
// backtracking-against-fresh-weights safety net -- Ceres's own trust
// region only ever checks its *frozen*-weight cost function, so nothing
// stops it from happily walking uphill in the true energy while still
// reducing its own frozen approximation of it. Found on the 25x25
// gradient.png regression: even after re-freezing every GN sub-iteration
// (see ceresSolveOnce), reconstruction RMSE still climbed steadily after
// the first sub-step -- refreezing more often slowed the drift but this
// verify-and-revert gate is what actually stops it.
double computeTrueGeometryEnergy(const GradientMesh& mesh, const Image& target,
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
                    energy += w * d.lengthSq();

                    // Vector-line guided term (Sec 4.2) -- must mirror
                    // MeshOptimizer.cpp's computeGeometryEnergy exactly
                    // (same dense per-sample grid, same nearestVectorLineField
                    // formula), for the same line-search-consistency reason
                    // as the rest of this function.
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

    // Puv is NOT included here -- fixed at {0,0} per the paper's Sec 3, not
    // free/derived -- see GradientMesh.h and MeshOptimizer.cpp's
    // addTangentPriorTerms comment.
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 0; c < mesh.cols; ++c) {
            const MeshVertex& mv = mesh.at(r, c);
            Vec2 tu = mesh.tangentU(r, c), tv = mesh.tangentV(r, c);
            Vec2 du = mv.Pu - tu, dv = mv.Pv - tv;
            energy += opts.geomTangentPriorWeight *
                (du.x * du.x + du.y * du.y + dv.x * dv.x + dv.y * dv.y);
        }
    }

    // Normal-only soft constraint -- see MeshOptimizer.cpp's
    // computeGeometryEnergy for why (must mirror it exactly): boundary
    // vertices should be free to slide ALONG their spline (Sec 4), so only
    // the off-curve (normal) displacement component is penalized.
    for (const auto& mv : mesh.vertices) {
        if (!mv.isBoundary) continue;
        const CubicBezier& spline = mesh.boundary[mv.boundarySide];
        Vec2 t = spline.eval(mv.boundaryT);
        Vec2 normal = Vec2{-spline.evalDeriv(mv.boundaryT).y, spline.evalDeriv(mv.boundaryT).x}.normalized();
        double d = (mv.P - t).dot(normal);
        energy += opts.boundaryWeight * (d * d);
    }

    // (Vector-line term already folded into the main sample loop above,
    // matching MeshOptimizer.cpp's computeGeometryEnergy -- no separate
    // discrete-edge pass any more.)

    return energy;
}

// Extends computeTrueGeometryEnergy with the two color-side energy terms
// the closed-form color solve minimizes (color smoothness, color ridge on
// Cu/Cv/Cuv) -- see MeshOptimizer.cpp's color-solve step 2 for the
// original. The data term above already uses mesh.evalColor(...) on
// whatever `mesh` is passed in, so it already reflects the CURRENT
// (live, not frozen) color -- no separate accounting needed for that
// part; only the two purely-color-side regularizers need adding. Used
// only by optimizeJointCeres's verify-and-shrink gate, for the same
// reason computeTrueGeometryEnergy exists: Ceres's own trust region only
// ever checks its frozen-weight cost function, so a step that looks good
// there can still be bad against the true, freshly-evaluated energy.
double computeTrueJointEnergy(const GradientMesh& mesh, const Image& target,
                               const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts) {
    double energy = computeTrueGeometryEnergy(mesh, target, vectorLines, opts);

    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            const Color& c0 = mesh.at(r, c - 1).C;
            const Color& c1 = mesh.at(r, c).C;
            const Color& c2 = mesh.at(r, c + 1).C;
            Color d = c0 - c1 * 2.0 + c2;
            energy += opts.smoothWeightColor * d.lengthSq();
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            const Color& c0 = mesh.at(r - 1, c).C;
            const Color& c1 = mesh.at(r, c).C;
            const Color& c2 = mesh.at(r + 1, c).C;
            Color d = c0 - c1 * 2.0 + c2;
            energy += opts.smoothWeightColor * d.lengthSq();
        }
    }
    for (const auto& mv : mesh.vertices) {
        energy += opts.colorDerivRidge * (mv.Cu.lengthSq() + mv.Cv.lengthSq() + mv.Cuv.lengthSq());
    }

    return energy;
}

// ---- Data term: one residual block per patch (see spike/ceres_geom_spike.cpp) ----
class PatchDataCostFunction : public ceres::CostFunction {
public:
    PatchDataCostFunction(const GradientMesh& snapshot, const Image& target,
                           int patchRow, int patchCol, int samplesPerEdge)
        : snapshot_(snapshot), target_(target), pr_(patchRow), pc_(patchCol),
          n_(std::max(2, samplesPerEdge)) {
        int numSamples = (n_ + 1) * (n_ + 1);
        set_num_residuals(3 * numSamples);
        for (int k = 0; k < 4; ++k) mutable_parameter_block_sizes()->push_back(6);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        GradientMesh mesh = snapshot_;
        for (int a = 0; a < 2; ++a) {
            for (int b = 0; b < 2; ++b) {
                int k = a * 2 + b;
                int vert = mesh.idx(pr_ + b, pc_ + a);
                const double* p = parameters[k];
                mesh.vertices[vert].P  = {p[0], p[1]};
                mesh.vertices[vert].Pu = {p[2], p[3]};
                mesh.vertices[vert].Pv = {p[4], p[5]};
                // Puv left at snapshot's value -- always {0,0}, fixed, not a
                // parameter (see GradientMesh.h / geomCorner()).
            }
        }

        double duv = 1.0 / (n_ * n_);
        int rowsTotal = num_residuals();
        if (jacobians) {
            for (int k = 0; k < 4; ++k)
                if (jacobians[k]) std::fill(jacobians[k], jacobians[k] + rowsTotal * 6, 0.0);
        }

        int row = 0;
        for (int i = 0; i <= n_; ++i) {
            double v = double(i) / n_;
            for (int j = 0; j <= n_; ++j, row += 3) {
                double u = double(j) / n_;
                Vec2 pos = mesh.evalPos(pr_, pc_, u, v);
                Color cmesh = mesh.evalColor(pr_, pc_, u, v);
                Color ctarget = target_.sampleBilinear(pos.x, pos.y);
                ColorGrad grad = target_.sampleGradient(pos.x, pos.y);
                // Frozen: from snapshot_, not the trial mesh -- see file header.
                double w = areaWeightAt(snapshot_, pr_, pc_, u, v, duv);
                double sw = std::sqrt(w);
                PatchWeights pw = PatchWeights::at(u, v);

                double r0[3] = {cmesh.r - ctarget.r, cmesh.g - ctarget.g, cmesh.b - ctarget.b};
                residuals[row + 0] = sw * r0[0];
                residuals[row + 1] = sw * r0[1];
                residuals[row + 2] = sw * r0[2];
                if (!jacobians) continue;

                double gx[3] = {grad.dx.r, grad.dx.g, grad.dx.b};
                double gy[3] = {grad.dy.r, grad.dy.g, grad.dy.b};
                for (int a = 0; a < 2; ++a) {
                    for (int b = 0; b < 2; ++b) {
                        int k = a * 2 + b;
                        if (!jacobians[k]) continue;
                        int base = (a * 2 + b) * 4;
                        double wP = pw.w[base + 0], wPu = pw.w[base + 1], wPv = pw.w[base + 2];
                        double* J = jacobians[k];
                        for (int c = 0; c < 3; ++c) {
                            int rr = (row + c) * 6;
                            J[rr + 0] = sw * (-gx[c] * wP);
                            J[rr + 1] = sw * (-gy[c] * wP);
                            J[rr + 2] = sw * (-gx[c] * wPu);
                            J[rr + 3] = sw * (-gy[c] * wPu);
                            J[rr + 4] = sw * (-gx[c] * wPv);
                            J[rr + 5] = sw * (-gy[c] * wPv);
                        }
                    }
                }
            }
        }
        return true;
    }

private:
    const GradientMesh& snapshot_;
    const Image& target_;
    int pr_, pc_, n_;
};

// ---- Joint data term: same as PatchDataCostFunction above, but color is
// now a LIVE parameter block too, not frozen -- see
// OptimizerOptions::useCeresJoint and spike/ceres_joint_spike.cpp. The
// geometry side is the same 6-double-per-corner (P,Pu,Pv) layout as
// PatchDataCostFunction -- Puv is fixed at {0,0}, not a parameter (see
// GradientMesh.h; an earlier pass briefly made Puv free too and used an
// 8-wide block here, reverted after rereading the paper's Sec 3). The
// color block was cross-checked against both a standalone finite-
// difference harness and ceres::GradientChecker (max relative error
// 3.5e-8, pure FD noise) and is unaffected by any of this.
// Used ONLY by optimizeJointCeres/jointSolveOnce -- optimizeGeometryCeres
// keeps using the frozen-color PatchDataCostFunction above, unchanged. ----
//
// 8 parameter blocks per patch: [0..3] = geometry corners (6 doubles:
// P.x,P.y,Pu.x,Pu.y,Pv.x,Pv.y -- same layout/order as PatchDataCostFunction),
// [4..7] = color corners (12 doubles: C.r,C.g,C.b, Cu.r,Cu.g,Cu.b, Cv.r,
// Cv.g,Cv.b, Cuv.r,Cuv.g,Cuv.b). Block k and block 4+k are always the SAME
// physical vertex. P/Pu/Pv AND all four color kinds are free unknowns, so
// the geometry Jacobian uses PatchWeights kinds 0..2 and the color one
// uses all four.
class JointPatchDataCostFunction : public ceres::CostFunction {
public:
    JointPatchDataCostFunction(const GradientMesh& snapshot, const Image& target,
                                int patchRow, int patchCol, int samplesPerEdge)
        : snapshot_(snapshot), target_(target), pr_(patchRow), pc_(patchCol),
          n_(std::max(2, samplesPerEdge)) {
        int numSamples = (n_ + 1) * (n_ + 1);
        set_num_residuals(3 * numSamples);
        for (int k = 0; k < 4; ++k) mutable_parameter_block_sizes()->push_back(6);   // geometry
        for (int k = 0; k < 4; ++k) mutable_parameter_block_sizes()->push_back(12);  // color
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        GradientMesh mesh = snapshot_;
        for (int a = 0; a < 2; ++a) {
            for (int b = 0; b < 2; ++b) {
                int k = a * 2 + b;
                int vert = mesh.idx(pr_ + b, pc_ + a);
                const double* pg = parameters[k];
                mesh.vertices[vert].P  = {pg[0], pg[1]};
                mesh.vertices[vert].Pu = {pg[2], pg[3]};
                mesh.vertices[vert].Pv = {pg[4], pg[5]};
                const double* pc = parameters[4 + k];
                mesh.vertices[vert].C   = {pc[0],  pc[1],  pc[2]};
                mesh.vertices[vert].Cu  = {pc[3],  pc[4],  pc[5]};
                mesh.vertices[vert].Cv  = {pc[6],  pc[7],  pc[8]};
                mesh.vertices[vert].Cuv = {pc[9],  pc[10], pc[11]};
            }
        }

        double duv = 1.0 / (n_ * n_);
        int rowsTotal = num_residuals();
        if (jacobians) {
            for (int k = 0; k < 4; ++k) {
                if (jacobians[k]) std::fill(jacobians[k], jacobians[k] + rowsTotal * 6, 0.0);
                if (jacobians[4 + k]) std::fill(jacobians[4 + k], jacobians[4 + k] + rowsTotal * 12, 0.0);
            }
        }

        int row = 0;
        for (int i = 0; i <= n_; ++i) {
            double v = double(i) / n_;
            for (int j = 0; j <= n_; ++j, row += 3) {
                double u = double(j) / n_;
                Vec2 pos = mesh.evalPos(pr_, pc_, u, v);
                Color cmesh = mesh.evalColor(pr_, pc_, u, v);
                Color ctarget = target_.sampleBilinear(pos.x, pos.y);
                ColorGrad grad = target_.sampleGradient(pos.x, pos.y);
                // Frozen: from snapshot_, not the trial mesh -- see
                // PatchDataCostFunction's comment above for why.
                double w = areaWeightAt(snapshot_, pr_, pc_, u, v, duv);
                double sw = std::sqrt(w);
                PatchWeights pw = PatchWeights::at(u, v);

                double r0[3] = {cmesh.r - ctarget.r, cmesh.g - ctarget.g, cmesh.b - ctarget.b};
                residuals[row + 0] = sw * r0[0];
                residuals[row + 1] = sw * r0[1];
                residuals[row + 2] = sw * r0[2];

                if (!jacobians) continue;

                double gx[3] = {grad.dx.r, grad.dx.g, grad.dx.b};
                double gy[3] = {grad.dy.r, grad.dy.g, grad.dy.b};

                for (int a = 0; a < 2; ++a) {
                    for (int b = 0; b < 2; ++b) {
                        int k = a * 2 + b;
                        int base = (a * 2 + b) * 4;

                        // Geometry: d(r0)/d(geom) = -(grad . d(pos)/d(geom)) --
                        // cmesh doesn't depend on geometry (color surface
                        // evaluated at fixed parametric (u,v)). Puv is fixed,
                        // not a parameter, so only P/Pu/Pv columns exist.
                        if (jacobians[k]) {
                            double wP  = pw.w[base + 0];
                            double wPu = pw.w[base + 1];
                            double wPv = pw.w[base + 2];
                            double* J = jacobians[k];
                            for (int c = 0; c < 3; ++c) {
                                int rr = (row + c) * 6;
                                J[rr + 0] = sw * (-gx[c] * wP);
                                J[rr + 1] = sw * (-gy[c] * wP);
                                J[rr + 2] = sw * (-gx[c] * wPu);
                                J[rr + 3] = sw * (-gy[c] * wPu);
                                J[rr + 4] = sw * (-gx[c] * wPv);
                                J[rr + 5] = sw * (-gy[c] * wPv);
                            }
                        }

                        // Color: d(r0[c])/d(colorParam[kind*3+c2]) =
                        // sw*pw.w[base+kind] if c2==c else 0 -- ctarget
                        // doesn't depend on color, and channels don't mix.
                        if (jacobians[4 + k]) {
                            double* J = jacobians[4 + k];
                            for (int kind = 0; kind < 4; ++kind) {
                                double wk = pw.w[base + kind];
                                for (int c = 0; c < 3; ++c) {
                                    int rr = (row + c) * 12;
                                    int col = kind * 3 + c;
                                    J[rr + col] = sw * wk;
                                }
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

private:
    const GradientMesh& snapshot_;
    const Image& target_;
    int pr_, pc_, n_;
};

// ---- Color smoothness: one residual block per row/col triple on the base
// color C only (mirrors MeshOptimizer.cpp's color-solve step: "Smoothness
// on base color (kind 0)"). 3 parameter blocks of 12 doubles each (the
// full color block, same layout as JointPatchDataCostFunction's color
// params); only indices 0,1,2 (C.r,C.g,C.b) participate -- Cu,Cv,Cuv are
// untouched by this term, exactly like the hand-rolled closed-form solve. ----
class ColorSmoothTripleCostFunction : public ceres::CostFunction {
public:
    explicit ColorSmoothTripleCostFunction(double weight) : sw_(std::sqrt(std::max(weight, 0.0))) {
        set_num_residuals(3); // rr, rg, rb
        for (int k = 0; k < 3; ++k) mutable_parameter_block_sizes()->push_back(12);
    }
    bool Evaluate(double const* const* p, double* residuals, double** jacobians) const override {
        for (int c = 0; c < 3; ++c)
            residuals[c] = sw_ * (p[0][c] - 2 * p[1][c] + p[2][c]);
        if (!jacobians) return true;
        double coeff[3] = {1.0, -2.0, 1.0};
        for (int k = 0; k < 3; ++k) {
            if (!jacobians[k]) continue;
            double* J = jacobians[k];
            std::fill(J, J + 3 * 12, 0.0);
            for (int c = 0; c < 3; ++c) J[c * 12 + c] = sw_ * coeff[k];
        }
        return true;
    }
private:
    double sw_;
};

// ---- Color ridge: one residual block per vertex, pulling Cu,Cv,Cuv
// toward zero (mirrors MeshOptimizer.cpp's "ridge on Cu,Cv,Cuv (kinds
// 1..3)"). C itself (kind 0) is untouched. 9 residuals = 3 kinds x 3
// channels, indices 3..11 of the 12-wide color block. ----
class ColorRidgeCostFunction : public ceres::CostFunction {
public:
    explicit ColorRidgeCostFunction(double weight) : sw_(std::sqrt(std::max(weight, 0.0))) {
        set_num_residuals(9);
        mutable_parameter_block_sizes()->push_back(12);
    }
    bool Evaluate(double const* const* p, double* residuals, double** jacobians) const override {
        for (int i = 0; i < 9; ++i) residuals[i] = sw_ * p[0][3 + i];
        if (!jacobians || !jacobians[0]) return true;
        double* J = jacobians[0];
        std::fill(J, J + 9 * 12, 0.0);
        for (int i = 0; i < 9; ++i) J[i * 12 + (3 + i)] = sw_;
        return true;
    }
private:
    double sw_;
};

// ---- Smoothness: one residual block per row/col triple (mirrors addSmoothnessTerms).
// Only ever reads/writes P.x,P.y (indices 0,1) of the 6-wide (P,Pu,Pv)
// geometry block every other geometry term shares. ----
class SmoothTripleCostFunction : public ceres::CostFunction {
public:
    explicit SmoothTripleCostFunction(double weight) : sw_(std::sqrt(std::max(weight, 0.0))) {
        set_num_residuals(2); // rx, ry
        for (int k = 0; k < 3; ++k) mutable_parameter_block_sizes()->push_back(6);
    }
    bool Evaluate(double const* const* p, double* residuals, double** jacobians) const override {
        double rx = p[0][0] - 2 * p[1][0] + p[2][0];
        double ry = p[0][1] - 2 * p[1][1] + p[2][1];
        residuals[0] = sw_ * rx;
        residuals[1] = sw_ * ry;
        if (!jacobians) return true;
        double coeff[3] = {1.0, -2.0, 1.0};
        for (int k = 0; k < 3; ++k) {
            if (!jacobians[k]) continue;
            double* J = jacobians[k];
            std::fill(J, J + 2 * 6, 0.0);
            J[0 * 6 + 0] = sw_ * coeff[k]; // d(rx)/d(P.x)
            J[1 * 6 + 1] = sw_ * coeff[k]; // d(ry)/d(P.y)
        }
        return true;
    }
private:
    double sw_;
};

// ---- Tangent-prior: one residual block per vertex (mirrors
// addTangentPriorTerms). 4 residuals: Pu.x,Pu.y,Pv.x,Pv.y vs. their
// finite-difference targets. Puv is NOT included -- it's fixed at {0,0}
// per the paper's Sec 3, not a free unknown (see GradientMesh.h). ----
class TangentPriorCostFunction : public ceres::CostFunction {
public:
    TangentPriorCostFunction(double weight, Vec2 tu, Vec2 tv)
        : sw_(std::sqrt(std::max(weight, 0.0))), tu_(tu), tv_(tv) {
        set_num_residuals(4);
        mutable_parameter_block_sizes()->push_back(6);
    }
    bool Evaluate(double const* const* p, double* residuals, double** jacobians) const override {
        residuals[0] = sw_ * (p[0][2] - tu_.x);
        residuals[1] = sw_ * (p[0][3] - tu_.y);
        residuals[2] = sw_ * (p[0][4] - tv_.x);
        residuals[3] = sw_ * (p[0][5] - tv_.y);
        if (!jacobians || !jacobians[0]) return true;
        double* J = jacobians[0];
        std::fill(J, J + 4 * 6, 0.0);
        J[0 * 6 + 2] = sw_; J[1 * 6 + 3] = sw_; J[2 * 6 + 4] = sw_; J[3 * 6 + 5] = sw_;
        return true;
    }
private:
    double sw_; Vec2 tu_, tv_;
};

// ---- Boundary: one residual block per boundary vertex. Normal-only (see
// MeshOptimizer.cpp's computeGeometryEnergy comment): boundary control
// points must stay free to slide ALONG their spline (Sec 4: "control
// points on the boundary only move along the splines"), so this penalizes
// only the displacement component along the curve's normal at the current
// boundaryT -- a single scalar residual, not two independent x/y ones,
// which would resist along-curve motion exactly as hard as off-curve
// drift and effectively pin the vertex in place instead of letting it
// slide. ----
class BoundaryCostFunction : public ceres::CostFunction {
public:
    BoundaryCostFunction(double weight, Vec2 targetPos, Vec2 normal)
        : sw_(std::sqrt(std::max(weight, 0.0))), target_(targetPos), normal_(normal) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->push_back(6);
    }
    bool Evaluate(double const* const* p, double* residuals, double** jacobians) const override {
        double dx = p[0][0] - target_.x, dy = p[0][1] - target_.y;
        residuals[0] = sw_ * (dx * normal_.x + dy * normal_.y);
        if (!jacobians || !jacobians[0]) return true;
        double* J = jacobians[0];
        std::fill(J, J + 1 * 6, 0.0);
        J[0] = sw_ * normal_.x; J[1] = sw_ * normal_.y;
        return true;
    }
private:
    double sw_; Vec2 target_; Vec2 normal_;
};

// ---- Vector-line (Sec 4.2): one residual block per PATCH, dense over the
// same (u,v) sample grid as the data term -- direct transcription of
// MeshOptimizer.cpp's hand-rolled vector-line block inside
// optimizeAtCurrentResolution's GN loop (see that file's comment on
// nearestVectorLineField for the formula). Replaces the old
// VectorLineCostFunction, which only checked the coarse discrete straight
// edge between two adjacent control points at its midpoint against a flat
// global pixel radius, and never touched Pu/Pv at all.
//
// Which line (if any) matches, and its Gaussian weight, is evaluated ONCE
// per sample from the frozen `snapshot` position -- same freezing
// convention as areaWeightAt/edgeRelaxFactor elsewhere in this file (see
// file header comment): nearestVectorLineField's "nearest line" and
// Gaussian falloff are themselves nonlinear/discontinuous functions of
// position, so re-differentiating through them at every trial step inside
// Ceres's own internal iterations would need this cost function to track
// its own match set as it goes, which is unnecessary complexity for a term
// that -- like the others here -- is already re-frozen every GN
// sub-iteration when the whole snapshot is retaken (see ceresSolveOnce's
// comment). Only the two tangent residuals (ru, rv) are actually
// differentiated live, exactly matching what the hand-rolled path treats
// as the free part of this term.
//
// The number of residuals is 2 * (number of samples that found a match in
// `snapshot`) -- can be zero for a patch far from every line, in which
// case the caller must skip AddResidualBlock entirely (Ceres does not
// accept a zero-residual cost function).
class VectorLineCostFunction : public ceres::CostFunction {
public:
    VectorLineCostFunction(const GradientMesh& snapshot, const std::vector<VectorLine>& lines,
                            int patchRow, int patchCol, int samplesPerEdge, double vectorLineWeight)
        : pr_(patchRow), pc_(patchCol), n_(std::max(2, samplesPerEdge)), vectorLineWeight_(vectorLineWeight) {
        for (int i = 0; i <= n_; ++i) {
            double v = double(i) / n_;
            for (int j = 0; j <= n_; ++j) {
                double u = double(j) / n_;
                Vec2 pos = snapshot.evalPos(pr_, pc_, u, v);
                VectorLineMatch match = nearestVectorLineField(lines, pos);
                if (match.found) matches_.push_back({i, j, match});
            }
        }
        set_num_residuals(2 * (int)matches_.size());
        for (int k = 0; k < 4; ++k) mutable_parameter_block_sizes()->push_back(6);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        GradientMesh mesh; // only need geomCorner/evalPos -- fabricate a 2x2-vertex mesh for this one patch
        mesh.rows = 2; mesh.cols = 2;
        mesh.vertices.resize(4);
        for (int a = 0; a < 2; ++a) {
            for (int b = 0; b < 2; ++b) {
                int k = a * 2 + b;
                const double* p = parameters[k];
                MeshVertex& mv = mesh.vertices[mesh.idx(b, a)];
                mv.P  = {p[0], p[1]};
                mv.Pu = {p[2], p[3]};
                mv.Pv = {p[4], p[5]};
            }
        }

        int rowsTotal = num_residuals();
        if (jacobians) {
            for (int k = 0; k < 4; ++k)
                if (jacobians[k]) std::fill(jacobians[k], jacobians[k] + rowsTotal * 6, 0.0);
        }

        for (size_t m = 0; m < matches_.size(); ++m) {
            double v = double(matches_[m].i) / n_;
            double u = double(matches_[m].j) / n_;
            const Vec2& dir = matches_[m].match.dir;
            double weight = vectorLineWeight_ * matches_[m].match.weight;
            double sw = std::sqrt(std::max(weight, 0.0));

            Vec2 dU, dV;
            mesh.evalPos(0, 0, u, v, &dU, &dV);
            double ru = dU.cross(dir);
            double rv = dV.cross(dir);
            residuals[2 * m + 0] = sw * ru;
            residuals[2 * m + 1] = sw * rv;
            if (!jacobians) continue;

            PatchWeights pw = PatchWeights::at(u, v);
            for (int a = 0; a < 2; ++a) {
                for (int b = 0; b < 2; ++b) {
                    int k = a * 2 + b;
                    if (!jacobians[k]) continue;
                    int base = (a * 2 + b) * 4;
                    double wuP = pw.wu[base + 0], wuPu = pw.wu[base + 1], wuPv = pw.wu[base + 2];
                    double wvP = pw.wv[base + 0], wvPu = pw.wv[base + 1], wvPv = pw.wv[base + 2];
                    double* J = jacobians[k];
                    int rrU = (2 * (int)m + 0) * 6;
                    int rrV = (2 * (int)m + 1) * 6;
                    J[rrU + 0] = sw * ( wuP  * dir.y); J[rrU + 1] = sw * (-wuP  * dir.x);
                    J[rrU + 2] = sw * ( wuPu * dir.y); J[rrU + 3] = sw * (-wuPu * dir.x);
                    J[rrU + 4] = sw * ( wuPv * dir.y); J[rrU + 5] = sw * (-wuPv * dir.x);
                    J[rrV + 0] = sw * ( wvP  * dir.y); J[rrV + 1] = sw * (-wvP  * dir.x);
                    J[rrV + 2] = sw * ( wvPu * dir.y); J[rrV + 3] = sw * (-wvPu * dir.x);
                    J[rrV + 4] = sw * ( wvPv * dir.y); J[rrV + 5] = sw * (-wvPv * dir.x);
                }
            }
        }
        return true;
    }

private:
    struct SampleMatch { int i, j; VectorLineMatch match; };
    int pr_, pc_, n_;
    double vectorLineWeight_;
    std::vector<SampleMatch> matches_;
};

} // namespace

// Runs ONE Ceres solve using weights/targets frozen from `snapshot`
// (the current mesh state), for at most `maxIters` of Ceres's own
// internal LM iterations, then writes the result back into `mesh`.
// Factored out of optimizeGeometryCeres() so that function can call this
// repeatedly, re-snapshotting between calls -- see that function's
// comment for why: letting Ceres run many internal iterations against
// weights frozen once at the very start let those weights go stale as
// the mesh moved, which on the harder 25x25 gradient.png regression test
// produced a *worse* result than the hand-rolled path (reconstruction
// RMSE climbing for several outer iterations in a row instead of
// decreasing) -- diagnosed by comparing refresh cadence against the
// hand-rolled loop, which re-freezes these same quantities every single
// GN sub-iteration (opts.geomGaussNewtonItersPerOuter times per outer
// iteration), not once per outer iteration.
static void ceresSolveOnce(GradientMesh& mesh, const Image& target,
                            const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts,
                            int maxIters) {
    int numV = (int)mesh.vertices.size();
    int n = std::max(2, opts.samplesPerPatchEdge);
    double duv = 1.0 / (n * n);
    (void)duv;

    // Re-project boundary vertices onto their spline before taking the
    // snapshot below -- mirrors MeshOptimizer.cpp's per-GN-substep
    // reprojection (see that file's comment in optimizeAtCurrentResolution
    // for why this must happen every substep, not just once per outer
    // iteration: a strong vector-line pull can move a boundary vertex far
    // enough in one substep that a stale target/normal badly mis-
    // linearizes the constraint on the next).
    for (auto& v : mesh.vertices) {
        if (!v.isBoundary) continue;
        v.boundaryT = mesh.boundary[v.boundarySide].closestT(v.P);
    }

    // Snapshot: the frozen linearization point for every weight/target
    // quantity below (area weight, edge-relax factor, tangent-prior
    // tu/tv, vector-line direction/weight, and now boundaryT/target/normal
    // too) -- see file header comment. A plain copy, not a reference into
    // mesh, since mesh is about to be mutated by Ceres's own trial
    // evaluations via the parameter-block pointers.
    GradientMesh snapshot = mesh;

    // One 6-double parameter block per vertex: 0,1=P.x,P.y; 2,3=Pu.x,Pu.y;
    // 4,5=Pv.x,Pv.y -- same layout as the hand-rolled path's H/g blocks.
    // Puv is fixed at {0,0}, not a parameter (see GradientMesh.h).
    std::vector<std::array<double, 6>> params(numV);
    for (int i = 0; i < numV; ++i) {
        const MeshVertex& mv = mesh.vertices[i];
        params[i] = {mv.P.x, mv.P.y, mv.Pu.x, mv.Pu.y, mv.Pv.x, mv.Pv.y};
    }

    ceres::Problem problem;
    // Cost functions constructed with `new` and no ownership transfer
    // ceres::TAKE_OWNERSHIP (the default) is what AddResidualBlock uses --
    // Ceres deletes them when `problem` is destroyed.

    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            auto* cost = new PatchDataCostFunction(snapshot, target, pr, pc, n);
            std::vector<double*> blocks;
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 2; ++b)
                    blocks.push_back(params[mesh.idx(pr + b, pc + a)].data());
            problem.AddResidualBlock(cost, nullptr, blocks);
        }
    }

    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
            double weight = opts.smoothWeightGeom *
                edgeRelaxFactor(target, snapshot.vertices[i1].P, opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            auto* cost = new SmoothTripleCostFunction(weight);
            problem.AddResidualBlock(cost, nullptr, params[i0].data(), params[i1].data(), params[i2].data());
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
            double weight = opts.smoothWeightGeom *
                edgeRelaxFactor(target, snapshot.vertices[i1].P, opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            auto* cost = new SmoothTripleCostFunction(weight);
            problem.AddResidualBlock(cost, nullptr, params[i0].data(), params[i1].data(), params[i2].data());
        }
    }

    if (opts.geomTangentPriorWeight > 0.0) {
        for (int r = 0; r < mesh.rows; ++r) {
            for (int c = 0; c < mesh.cols; ++c) {
                int i = mesh.idx(r, c);
                Vec2 tu = snapshot.tangentU(r, c), tv = snapshot.tangentV(r, c);
                auto* cost = new TangentPriorCostFunction(opts.geomTangentPriorWeight, tu, tv);
                problem.AddResidualBlock(cost, nullptr, params[i].data());
            }
        }
    }

    for (int i = 0; i < numV; ++i) {
        const MeshVertex& mv = mesh.vertices[i];
        if (!mv.isBoundary) continue;
        const CubicBezier& spline = mesh.boundary[mv.boundarySide];
        Vec2 targetPos = spline.eval(mv.boundaryT);
        Vec2 normal = Vec2{-spline.evalDeriv(mv.boundaryT).y, spline.evalDeriv(mv.boundaryT).x}.normalized();
        auto* cost = new BoundaryCostFunction(opts.boundaryWeight, targetPos, normal);
        problem.AddResidualBlock(cost, nullptr, params[i].data());
    }

    // Vector-line (Sec 4.2): one residual block per patch, dense over the
    // same sample grid as the data term -- see VectorLineCostFunction's
    // comment. Skip patches where the frozen snapshot found no match
    // anywhere in the patch (Ceres rejects a zero-residual cost function).
    if (!vectorLines.empty()) {
        for (int pr = 0; pr < mesh.rows - 1; ++pr) {
            for (int pc = 0; pc < mesh.cols - 1; ++pc) {
                auto* cost = new VectorLineCostFunction(snapshot, vectorLines, pr, pc, n, opts.vectorLineWeight);
                if (cost->num_residuals() == 0) { delete cost; continue; }
                std::vector<double*> blocks;
                for (int a = 0; a < 2; ++a)
                    for (int b = 0; b < 2; ++b)
                        blocks.push_back(params[mesh.idx(pr + b, pc + a)].data());
                problem.AddResidualBlock(cost, nullptr, blocks);
            }
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::CGNR;
    options.preconditioner_type = ceres::JACOBI;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = std::max(1, maxIters);
    options.minimizer_progress_to_stdout = false;
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (int i = 0; i < numV; ++i) {
        MeshVertex& mv = mesh.vertices[i];
        mv.P  = {params[i][0], params[i][1]};
        mv.Pu = {params[i][2], params[i][3]};
        mv.Pv = {params[i][4], params[i][5]};
    }
}

// Same idea as ceresSolveOnce (frozen snapshot, one ceres::Problem, at
// most maxIters of Ceres's own internal LM iterations), but for the FULL
// joint problem: geometry (P,Pu,Pv, 6 doubles/vertex -- Puv is fixed, not a
// parameter) AND color (C,Cu,Cv,Cuv, 12 doubles/vertex) as
// separate-but-simultaneously-solved parameter blocks. Geometry-side
// residuals (smoothness, tangent-prior, boundary, vector-line) are
// UNCHANGED from ceresSolveOnce -- they don't involve color at all, so the
// existing SmoothTripleCostFunction/TangentPriorCostFunction/
// BoundaryCostFunction/VectorLineCostFunction are reused verbatim, just fed
// the geometry half of `paramsGeom`. Only
// the data term (now JointPatchDataCostFunction, with both geometry AND
// color live) and the two new color-side terms (ColorSmoothTripleCostFunction,
// ColorRidgeCostFunction) are new.
static void jointSolveOnce(GradientMesh& mesh, const Image& target,
                            const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts,
                            int maxIters) {
    int numV = (int)mesh.vertices.size();
    int n = std::max(2, opts.samplesPerPatchEdge);

    // Re-project boundary vertices before snapshotting -- see
    // ceresSolveOnce's comment for why this must happen every substep.
    for (auto& v : mesh.vertices) {
        if (!v.isBoundary) continue;
        v.boundaryT = mesh.boundary[v.boundarySide].closestT(v.P);
    }

    GradientMesh snapshot = mesh; // frozen linearization point, same convention as ceresSolveOnce

    // Geometry blocks are 6-wide (P,Pu,Pv) -- Puv fixed at {0,0}, not a
    // parameter (see GradientMesh.h).
    std::vector<std::array<double, 6>> paramsGeom(numV);
    std::vector<std::array<double, 12>> paramsColor(numV);
    for (int i = 0; i < numV; ++i) {
        const MeshVertex& mv = mesh.vertices[i];
        paramsGeom[i] = {mv.P.x, mv.P.y, mv.Pu.x, mv.Pu.y, mv.Pv.x, mv.Pv.y};
        paramsColor[i] = {mv.C.r, mv.C.g, mv.C.b, mv.Cu.r, mv.Cu.g, mv.Cu.b,
                           mv.Cv.r, mv.Cv.g, mv.Cv.b, mv.Cuv.r, mv.Cuv.g, mv.Cuv.b};
    }

    ceres::Problem problem;

    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            auto* cost = new JointPatchDataCostFunction(snapshot, target, pr, pc, n);
            std::vector<double*> blocks;
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 2; ++b)
                    blocks.push_back(paramsGeom[mesh.idx(pr + b, pc + a)].data());
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 2; ++b)
                    blocks.push_back(paramsColor[mesh.idx(pr + b, pc + a)].data());
            problem.AddResidualBlock(cost, nullptr, blocks);
        }
    }

    // Geometry-side terms: identical to ceresSolveOnce.
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
            double weight = opts.smoothWeightGeom *
                edgeRelaxFactor(target, snapshot.vertices[i1].P, opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            auto* cost = new SmoothTripleCostFunction(weight);
            problem.AddResidualBlock(cost, nullptr, paramsGeom[i0].data(), paramsGeom[i1].data(), paramsGeom[i2].data());
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
            double weight = opts.smoothWeightGeom *
                edgeRelaxFactor(target, snapshot.vertices[i1].P, opts.smoothGeomEdgeGain, opts.smoothGeomMinFactor);
            auto* cost = new SmoothTripleCostFunction(weight);
            problem.AddResidualBlock(cost, nullptr, paramsGeom[i0].data(), paramsGeom[i1].data(), paramsGeom[i2].data());
        }
    }
    if (opts.geomTangentPriorWeight > 0.0) {
        for (int r = 0; r < mesh.rows; ++r) {
            for (int c = 0; c < mesh.cols; ++c) {
                int i = mesh.idx(r, c);
                Vec2 tu = snapshot.tangentU(r, c), tv = snapshot.tangentV(r, c);
                auto* cost = new TangentPriorCostFunction(opts.geomTangentPriorWeight, tu, tv);
                problem.AddResidualBlock(cost, nullptr, paramsGeom[i].data());
            }
        }
    }
    for (int i = 0; i < numV; ++i) {
        const MeshVertex& mv = mesh.vertices[i];
        if (!mv.isBoundary) continue;
        const CubicBezier& spline = mesh.boundary[mv.boundarySide];
        Vec2 targetPos = spline.eval(mv.boundaryT);
        Vec2 normal = Vec2{-spline.evalDeriv(mv.boundaryT).y, spline.evalDeriv(mv.boundaryT).x}.normalized();
        auto* cost = new BoundaryCostFunction(opts.boundaryWeight, targetPos, normal);
        problem.AddResidualBlock(cost, nullptr, paramsGeom[i].data());
    }
    // Vector-line (Sec 4.2): same dense per-patch term as ceresSolveOnce --
    // see VectorLineCostFunction's comment.
    if (!vectorLines.empty()) {
        for (int pr = 0; pr < mesh.rows - 1; ++pr) {
            for (int pc = 0; pc < mesh.cols - 1; ++pc) {
                auto* cost = new VectorLineCostFunction(snapshot, vectorLines, pr, pc, n, opts.vectorLineWeight);
                if (cost->num_residuals() == 0) { delete cost; continue; }
                std::vector<double*> blocks;
                for (int a = 0; a < 2; ++a)
                    for (int b = 0; b < 2; ++b)
                        blocks.push_back(paramsGeom[mesh.idx(pr + b, pc + a)].data());
                problem.AddResidualBlock(cost, nullptr, blocks);
            }
        }
    }

    // Color-side terms: new (mirrors MeshOptimizer.cpp's color-solve step).
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
            auto* cost = new ColorSmoothTripleCostFunction(opts.smoothWeightColor);
            problem.AddResidualBlock(cost, nullptr, paramsColor[i0].data(), paramsColor[i1].data(), paramsColor[i2].data());
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
            auto* cost = new ColorSmoothTripleCostFunction(opts.smoothWeightColor);
            problem.AddResidualBlock(cost, nullptr, paramsColor[i0].data(), paramsColor[i1].data(), paramsColor[i2].data());
        }
    }
    for (int i = 0; i < numV; ++i) {
        auto* cost = new ColorRidgeCostFunction(opts.colorDerivRidge);
        problem.AddResidualBlock(cost, nullptr, paramsColor[i].data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::CGNR;
    options.preconditioner_type = ceres::JACOBI;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = std::max(1, maxIters);
    options.minimizer_progress_to_stdout = false;
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (int i = 0; i < numV; ++i) {
        MeshVertex& mv = mesh.vertices[i];
        mv.P  = {paramsGeom[i][0], paramsGeom[i][1]};
        mv.Pu = {paramsGeom[i][2], paramsGeom[i][3]};
        mv.Pv = {paramsGeom[i][4], paramsGeom[i][5]};
        mv.C   = {paramsColor[i][0], paramsColor[i][1], paramsColor[i][2]};
        mv.Cu  = {paramsColor[i][3], paramsColor[i][4], paramsColor[i][5]};
        mv.Cv  = {paramsColor[i][6], paramsColor[i][7], paramsColor[i][8]};
        mv.Cuv = {paramsColor[i][9], paramsColor[i][10], paramsColor[i][11]};
    }
}

void optimizeGeometryCeres(GradientMesh& mesh, const Image& target,
                            const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts) {
    // Re-snapshot and re-solve opts.geomGaussNewtonItersPerOuter times,
    // each with a small iteration budget, checked against
    // computeTrueGeometryEnergy() -- see that function's comment for why
    // some such gate is necessary at all (refreezing more often alone
    // slowed but did not stop RMSE from climbing on the 25x25
    // gradient.png regression).
    //
    // The gate itself must be GRADUAL, not binary accept/revert-all: a
    // first version that either took the whole Ceres sub-step or threw it
    // away entirely made things *worse* (25x25: 43.7% vs the hand-rolled
    // path's 62.0%, down from 58.5% before the gate existed at all).
    // Ceres's internal trust region only ever checks its own step against
    // the FROZEN-weight cost function, so by the time a multi-iteration
    // sub-step is done, the step can be large enough that the frozen
    // linearization is no longer a good local model of the true
    // (freshly-reweighted) energy -- at which point the true energy often
    // *does* increase, and a binary gate has to throw the entire sub-step
    // away, wasting the (usually still locally-correct) early part of it.
    // The hand-rolled path never faces this because it only ever takes
    // ONE linearized GN step before re-checking -- but it still needs its
    // own alpha-shrinking backtracking (see the `for (int tries...)` loop
    // above in MeshOptimizer.cpp) to guard against exactly this kind of
    // over-shoot. So: treat Ceres's proposed sub-step the same way that
    // loop treats a single GN step -- as a *direction* from `before` to
    // `after`, and shrink how far along it we actually go (alpha = 1.0,
    // 0.4, 0.16, 0.064) until computeTrueGeometryEnergy() stops
    // objecting, before falling back to a full revert.
    //
    // 6 Ceres iterations per sub-step is a deliberately modest budget:
    // enough for Ceres's trust region to meaningfully outdo a single
    // hand-rolled GN+backtracking step, not so many that the frozen
    // weights it's using stop reflecting the mesh it's actually solving
    // for by the time this sub-step's result gets checked.
    const int itersPerSubStep = 6;
    for (int gi = 0; gi < opts.geomGaussNewtonItersPerOuter; ++gi) {
        std::vector<MeshVertex> before = mesh.vertices;
        double energyBefore = computeTrueGeometryEnergy(mesh, target, vectorLines, opts);

        ceresSolveOnce(mesh, target, vectorLines, opts, itersPerSubStep);
        std::vector<MeshVertex> after = mesh.vertices; // full (alpha=1) proposed step

        const int numV = (int)mesh.vertices.size();
        double alpha = 1.0;
        bool improved = false;
        for (int tries = 0; tries < 4; ++tries) {
            for (int i = 0; i < numV; ++i) {
                mesh.vertices[i].P  = before[i].P  + (after[i].P  - before[i].P)  * alpha;
                mesh.vertices[i].Pu = before[i].Pu + (after[i].Pu - before[i].Pu) * alpha;
                mesh.vertices[i].Pv = before[i].Pv + (after[i].Pv - before[i].Pv) * alpha;
            }
            double newEnergy = computeTrueGeometryEnergy(mesh, target, vectorLines, opts);
            if (newEnergy <= energyBefore) { improved = true; break; }
            alpha *= 0.4;
        }
        if (!improved) {
            mesh.vertices = before; // all alphas failed: revert this sub-step entirely
        }
    }
}

// The fully-joint counterpart to optimizeGeometryCeres: same re-snapshot/
// re-solve/gradual-backtrack structure, but jointSolveOnce solves BOTH
// geometry and color at once each sub-step, and the backtracking
// interpolates ALL SEVEN per-vertex fields (P,Pu,Pv,C,Cu,Cv,Cuv) toward
// the proposed state together (color and geometry are proposed and
// accepted/shrunk as one coupled step, not independently -- that
// coupling, unavailable to the block-coordinate-descent hand-rolled path,
// is the entire point of this experiment). Gated against
// computeTrueJointEnergy (data + all geometry-side terms + the two
// color-side terms), for the same reason optimizeGeometryCeres's gate
// exists: Ceres's own trust region only ever checks its frozen-weight
// cost function, not the true energy.
void optimizeJointCeres(GradientMesh& mesh, const Image& target,
                         const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts) {
    const int itersPerSubStep = 6; // same modest budget as optimizeGeometryCeres, same rationale
    for (int gi = 0; gi < opts.geomGaussNewtonItersPerOuter; ++gi) {
        std::vector<MeshVertex> before = mesh.vertices;
        double energyBefore = computeTrueJointEnergy(mesh, target, vectorLines, opts);

        jointSolveOnce(mesh, target, vectorLines, opts, itersPerSubStep);
        std::vector<MeshVertex> after = mesh.vertices;

        const int numV = (int)mesh.vertices.size();
        double alpha = 1.0;
        bool improved = false;
        for (int tries = 0; tries < 4; ++tries) {
            for (int i = 0; i < numV; ++i) {
                mesh.vertices[i].P   = before[i].P   + (after[i].P   - before[i].P)   * alpha;
                mesh.vertices[i].Pu  = before[i].Pu  + (after[i].Pu  - before[i].Pu)  * alpha;
                mesh.vertices[i].Pv  = before[i].Pv  + (after[i].Pv  - before[i].Pv)  * alpha;
                mesh.vertices[i].C   = before[i].C   + (after[i].C   - before[i].C)   * alpha;
                mesh.vertices[i].Cu  = before[i].Cu  + (after[i].Cu  - before[i].Cu)  * alpha;
                mesh.vertices[i].Cv  = before[i].Cv  + (after[i].Cv  - before[i].Cv)  * alpha;
                mesh.vertices[i].Cuv = before[i].Cuv + (after[i].Cuv - before[i].Cuv) * alpha;
            }
            double newEnergy = computeTrueJointEnergy(mesh, target, vectorLines, opts);
            if (newEnergy <= energyBefore) { improved = true; break; }
            alpha *= 0.4;
        }
        if (!improved) {
            mesh.vertices = before;
        }
    }
}

} // namespace gmcore

#endif // GMCORE_WITH_CERES
