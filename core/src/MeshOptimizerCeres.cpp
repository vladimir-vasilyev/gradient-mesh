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
// boundary's target point) is evaluated ONCE from a snapshot of the mesh
// taken at the start of optimizeGeometryCeres(), not re-evaluated as
// Ceres's own internal LM iterations move the trial parameters. This
// mirrors exactly how the hand-rolled path already treats these
// quantities (frozen per GN sub-iteration there; frozen for the whole
// optimizeGeometryCeres() call here, since Ceres iterates internally
// where the hand-rolled path re-enters this function's Gauss-Newton loop
// explicitly) -- it is not a new approximation introduced by this port.
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

Vec2 nearestVectorLineDir(const std::vector<VectorLine>& lines, const Vec2& p, double radius, bool& found) {
    found = false;
    double bestD2 = radius * radius;
    Vec2 bestDir{1, 0};
    for (const auto& line : lines) {
        for (size_t i = 0; i + 1 < line.points.size(); ++i) {
            Vec2 a = line.points[i], b = line.points[i + 1];
            Vec2 ab = b - a;
            double len2 = ab.lengthSq();
            if (len2 < 1e-9) continue;
            double t = (p - a).dot(ab) / len2;
            t = std::max(0.0, std::min(1.0, t));
            Vec2 proj = a + ab * t;
            double d2 = (p - proj).lengthSq();
            if (d2 < bestD2) { bestD2 = d2; bestDir = ab.normalized(); found = true; }
        }
    }
    return bestDir;
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

// ---- Smoothness: one residual block per row/col triple (mirrors addSmoothnessTerms) ----
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

// ---- Tangent-prior: one residual block per vertex (mirrors addTangentPriorTerms) ----
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

// ---- Boundary: one residual block per boundary vertex ----
class BoundaryCostFunction : public ceres::CostFunction {
public:
    BoundaryCostFunction(double weight, Vec2 targetPos)
        : sw_(std::sqrt(std::max(weight, 0.0))), target_(targetPos) {
        set_num_residuals(2);
        mutable_parameter_block_sizes()->push_back(6);
    }
    bool Evaluate(double const* const* p, double* residuals, double** jacobians) const override {
        residuals[0] = sw_ * (p[0][0] - target_.x);
        residuals[1] = sw_ * (p[0][1] - target_.y);
        if (!jacobians || !jacobians[0]) return true;
        double* J = jacobians[0];
        std::fill(J, J + 2 * 6, 0.0);
        J[0 * 6 + 0] = sw_; J[1 * 6 + 1] = sw_;
        return true;
    }
private:
    double sw_; Vec2 target_;
};

// ---- Vector-line: one residual block per near-line mesh edge ----
class VectorLineCostFunction : public ceres::CostFunction {
public:
    VectorLineCostFunction(double weight, Vec2 dir)
        : sw_(std::sqrt(std::max(weight, 0.0))), dir_(dir) {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->push_back(6);
        mutable_parameter_block_sizes()->push_back(6);
    }
    bool Evaluate(double const* const* p, double* residuals, double** jacobians) const override {
        Vec2 e{p[1][0] - p[0][0], p[1][1] - p[0][1]};
        double r0 = e.x * dir_.y - e.y * dir_.x;
        residuals[0] = sw_ * r0;
        if (!jacobians) return true;
        if (jacobians[0]) {
            std::fill(jacobians[0], jacobians[0] + 6, 0.0);
            jacobians[0][0] = sw_ * (-dir_.y);
            jacobians[0][1] = sw_ * (dir_.x);
        }
        if (jacobians[1]) {
            std::fill(jacobians[1], jacobians[1] + 6, 0.0);
            jacobians[1][0] = sw_ * (dir_.y);
            jacobians[1][1] = sw_ * (-dir_.x);
        }
        return true;
    }
private:
    double sw_; Vec2 dir_;
};

} // namespace

void optimizeGeometryCeres(GradientMesh& mesh, const Image& target,
                            const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts) {
    int numV = (int)mesh.vertices.size();
    int n = std::max(2, opts.samplesPerPatchEdge);
    double duv = 1.0 / (n * n);

    // Snapshot: the frozen linearization point for every weight/target
    // quantity below (area weight, edge-relax factor, tangent-prior
    // tu/tv, vector-line direction) -- see file header comment. A plain
    // copy, not a reference into mesh, since mesh is about to be mutated
    // by Ceres's own trial evaluations via the parameter-block pointers.
    GradientMesh snapshot = mesh;

    // One 6-double parameter block per vertex: 0,1=P.x,P.y; 2,3=Pu.x,Pu.y;
    // 4,5=Pv.x,Pv.y -- same layout as the hand-rolled path's H/g blocks.
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
        Vec2 targetPos = mesh.boundary[mv.boundarySide].eval(mv.boundaryT);
        auto* cost = new BoundaryCostFunction(opts.boundaryWeight, targetPos);
        problem.AddResidualBlock(cost, nullptr, params[i].data());
    }

    if (!vectorLines.empty()) {
        auto addEdge = [&](int v1, int v2) {
            Vec2 mid = (snapshot.vertices[v1].P + snapshot.vertices[v2].P) * 0.5;
            bool found = false;
            Vec2 dir = nearestVectorLineDir(vectorLines, mid, opts.vectorLineInfluenceRadius, found);
            if (!found) return;
            auto* cost = new VectorLineCostFunction(opts.vectorLineWeight, dir);
            problem.AddResidualBlock(cost, nullptr, params[v1].data(), params[v2].data());
        };
        for (int r = 0; r < mesh.rows; ++r)
            for (int c = 0; c < mesh.cols - 1; ++c) addEdge(mesh.idx(r, c), mesh.idx(r, c + 1));
        for (int c = 0; c < mesh.cols; ++c)
            for (int r = 0; r < mesh.rows - 1; ++r) addEdge(mesh.idx(r, c), mesh.idx(r + 1, c));
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::CGNR;
    options.preconditioner_type = ceres::JACOBI;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = std::max(10, opts.geomGaussNewtonItersPerOuter * 10);
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

} // namespace gmcore

#endif // GMCORE_WITH_CERES
