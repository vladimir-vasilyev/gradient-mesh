// spike/ceres_geom_spike.cpp
//
// Ceres feasibility spike (Phase 0 of the "replace hand-rolled geometry
// Gauss-Newton with Ceres" plan). Does NOT touch MeshOptimizer.cpp or any
// production code path -- this is a throwaway, standalone check that
// (a) Ceres + Eigen actually link and run against this project's real
// gmcore headers, and (b) an analytic CostFunction for the geometry data
// term, transcribed from the exact residual/Jacobian formulas already
// used in MeshOptimizer.cpp (its `rowCorners`/`buildChanneled` block,
// lines ~371-424 there), matches Ceres's own numeric differentiation via
// ceres::GradientChecker.
//
// If this passes, porting the real MeshOptimizer.cpp geometry GN loop to
// Ceres is a transcription exercise with much lower risk of a silent
// Jacobian bug. If it fails, we find out here, cheaply, instead of after
// wiring it into the real optimizer.

#include "gmcore/GradientMesh.h"
#include "gmcore/Image.h"
#include "gmcore/FergusonPatch.h"

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace gmcore;

namespace {

// Mirrors MeshOptimizer.cpp's anonymous-namespace areaWeightAt() exactly
// (same abs(cross-product) area weighting). Duplicated here rather than
// shared because that function is file-local (anonymous namespace) in
// MeshOptimizer.cpp; if Phase 1 promotes it to a shared header, this
// duplication goes away.
double areaWeightAt(const GradientMesh& mesh, int pr, int pc, double u, double v, double duv) {
    Vec2 dU, dV;
    mesh.evalPos(pr, pc, u, v, &dU, &dV);
    double a = std::abs(dU.cross(dV)) * duv;
    return std::max(a, 1e-6);
}

// One residual block per patch, covering the FULL data term of that patch
// (all n x n samples x 3 color channels at once), depending on the 4
// corner vertices' free geometry unknowns (P, Pu, Pv -- 6 doubles each,
// same layout as MeshOptimizer.cpp's 6-wide GN block: 0,1=P.x,P.y;
// 2,3=Pu.x,Pu.y; 4,5=Pv.x,Pv.y). Twist (Puv) is not a parameter here,
// exactly as in the real optimizer (still derived).
//
// Residual convention: Ceres minimizes 0.5*sum(residual^2) per block, and
// our energy convention (see MeshOptimizer.cpp's accumulateGNRow) is
// sum(weight * r0^2). So residual := sqrt(weight) * r0 makes Ceres's
// 0.5*residual^2 = 0.5*weight*r0^2 -- same argmin, just an overall 0.5
// scale on the total energy, which doesn't matter for gradient-checking
// or for where the minimum sits.
class PatchDataCostFunction : public ceres::CostFunction {
public:
    PatchDataCostFunction(const GradientMesh& meshTemplate, const Image& target,
                           int patchRow, int patchCol, int samplesPerEdge)
        : mesh_(meshTemplate), target_(target), pr_(patchRow), pc_(patchCol),
          n_(std::max(2, samplesPerEdge)) {
        int numSamples = (n_ + 1) * (n_ + 1);
        set_num_residuals(3 * numSamples);
        for (int k = 0; k < 4; ++k) mutable_parameter_block_sizes()->push_back(6);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        // Corner order matches MeshOptimizer.cpp's (a,b) loop: a=u-side(0/1
        // -> column offset), b=v-side(0/1 -> row offset); vertex =
        // mesh.idx(pr+b, pc+a). Parameter block k = a*2+b, matching
        // PatchWeights::at()'s `base = (a*2+b)*4` indexing.
        GradientMesh mesh = mesh_; // local mutable copy so we can push the
                                   // trial parameters into real MeshVertex
                                   // fields and reuse evalPos/evalColor
                                   // unchanged -- keeps this test on the
                                   // exact same evaluation code path the
                                   // real renderer/optimizer use.
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
                // IMPORTANT: the area weight is computed from mesh_ (the
                // ORIGINAL, unperturbed linearization point), not from the
                // trial `mesh` with the current candidate parameters
                // plugged in. This matches exactly how MeshOptimizer.cpp's
                // real GN loop uses areaWeightAt() -- as a fixed per-
                // sub-iteration reweighting computed once from the current
                // (pre-step) mesh state, not differentiated through. Using
                // the trial mesh here instead (as an earlier version of
                // this file did) makes the *residual* implicitly depend on
                // the parameters through w too, which the analytic
                // Jacobian below does NOT account for (it treats sw as a
                // constant) -- that mismatch was the actual cause of the
                // first GradientChecker failure (906/1800 bad entries,
                // max relative error 1.81), confirmed via a standalone
                // finite-difference check outside Ceres.
                double w = areaWeightAt(mesh_, pr_, pc_, u, v, duv);
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
                        // d(pos)/d(P) = pw.w[base+0], d(pos)/d(Pu) = pw.w[base+1],
                        // d(pos)/d(Pv) = pw.w[base+2] -- same weights
                        // MeshOptimizer.cpp's rowCorners/buildChanneled use.
                        // cmesh doesn't depend on geometry here (color is a
                        // separate, frozen parameter block, exactly as in
                        // the real block-coordinate optimizer), so
                        // d(r0)/d(param) = -d(ctarget)/d(param) =
                        // -(grad . d(pos)/d(param)).
                        double wP  = pw.w[base + 0];
                        double wPu = pw.w[base + 1];
                        double wPv = pw.w[base + 2];
                        double* J = jacobians[k]; // row-major, 6 cols
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
    const GradientMesh& mesh_;
    const Image& target_;
    int pr_, pc_, n_;
};

// Small synthetic test image with a real, strong sharp edge (not just
// noise) -- exactly the kind of local gradient the geometry data term's
// Jacobian needs to be checked against, since d(ctarget)/d(pos) (via
// sampleGradient) is the whole reason this Jacobian is nontrivial. A flat
// image would let a wrong Jacobian accidentally pass (everything zero).
Image makeEdgeTestImage(int w, int h) {
    Image img(w, h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double t = double(x) / w;
            double edge = 1.0 / (1.0 + std::exp(-18.0 * (t - 0.5))); // sharp sigmoid edge near x=w/2
            Color c{0.15 + 0.7 * edge, 0.2 + 0.5 * edge, 0.85 - 0.6 * edge};
            img.set(x, y, c);
        }
    }
    return img;
}

} // namespace

int main() {
    Image target = makeEdgeTestImage(48, 48);

    double x0 = 4, y0 = 4, x1 = 44, y1 = 44;
    std::array<CubicBezier, 4> boundary;
    boundary[0] = {Vec2{x0, y0}, Vec2{x0 + (x1 - x0) / 3, y0}, Vec2{x0 + 2 * (x1 - x0) / 3, y0}, Vec2{x1, y0}};
    boundary[1] = {Vec2{x1, y0}, Vec2{x1, y0 + (y1 - y0) / 3}, Vec2{x1, y0 + 2 * (y1 - y0) / 3}, Vec2{x1, y1}};
    boundary[2] = {Vec2{x1, y1}, Vec2{x0 + 2 * (x1 - x0) / 3, y1}, Vec2{x0 + (x1 - x0) / 3, y1}, Vec2{x0, y1}};
    boundary[3] = {Vec2{x0, y1}, Vec2{x0, y0 + 2 * (y1 - y0) / 3}, Vec2{x0, y0 + (y1 - y0) / 3}, Vec2{x0, y0}};

    // 3x3 control points -> 2x2 patches; patch (0,0) sits right where the
    // sharp edge is, so its data-term Jacobian genuinely depends on a
    // strong, nontrivial image gradient rather than a flat region where a
    // wrong Jacobian could accidentally check out fine.
    GradientMesh mesh = GradientMesh::buildInitial(3, 3, boundary, target);

    // Nudge P/Pu/Pv off their perfectly-regular initial values so the
    // check isn't evaluated at some accidentally-degenerate symmetric
    // point.
    for (auto& mv : mesh.vertices) {
        mv.P.x += 0.7; mv.P.y -= 0.4;
        mv.Pu.x *= 1.1; mv.Pu.y *= 0.9;
        mv.Pv.x *= 0.95; mv.Pv.y *= 1.05;
    }

    int patchRow = 0, patchCol = 0;
    int samplesPerEdge = 4;
    PatchDataCostFunction cost(mesh, target, patchRow, patchCol, samplesPerEdge);

    double* params[4];
    std::vector<std::array<double, 6>> blocks(4);
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            int k = a * 2 + b;
            const MeshVertex& mv = mesh.at(patchRow + b, patchCol + a);
            blocks[k] = {mv.P.x, mv.P.y, mv.Pu.x, mv.Pu.y, mv.Pv.x, mv.Pv.y};
            params[k] = blocks[k].data();
        }
    }

    ceres::NumericDiffOptions numDiffOpts;
    ceres::GradientChecker checker(&cost, nullptr, numDiffOpts);
    ceres::GradientChecker::ProbeResults results;
    bool ok = checker.Probe(params, 1e-6, &results);

    std::printf("=== Ceres GradientChecker spike (patch data term) ===\n");
    std::printf("Probe() returned: %s\n", ok ? "true (within tolerance)" : "false (MISMATCH)");
    std::printf("Maximum relative error: %.6g\n", results.maximum_relative_error);
    if (!ok) {
        std::printf("---- error log ----\n%s\n", results.error_log.c_str());
    }
    std::printf("(residual dim=%d, 4 parameter blocks x 6 doubles)\n", cost.num_residuals());
    return ok ? 0 : 1;
}
