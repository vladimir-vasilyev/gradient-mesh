// spike/ceres_joint_spike.cpp
//
// Ceres feasibility spike, Phase 0 of the "full joint (geometry+color)
// Ceres solve" plan (the original "option A" from the very start of this
// whole Ceres investigation, now revisited because the geometry-only
// Ceres port -- however successfully backtracked -- still can't reproduce
// the paper's Fig. 4 pinch on a 5x5 mesh; see README's "Deeper finding"
// and "Fixed: anisotropic smoothWeightGeom" sections for everything
// already tried and rejected).
//
// This does NOT touch MeshOptimizer.cpp or MeshOptimizerCeres.cpp -- it's
// a throwaway, standalone check of the single riskiest new piece: a data
// term whose residual depends on TWO kinds of live parameter blocks at
// once (geometry: P/Pu/Pv, 6 doubles/corner; color: C/Cu/Cv/Cuv, 12
// doubles/corner), where previously (MeshOptimizerCeres.cpp's
// PatchDataCostFunction) color was always frozen and only geometry was a
// live parameter.
//
// Why this is the risky part: unlike the geometry-only spike (where only
// the *target sample position* moved with the parameters, and the "mesh
// color at (u,v)" side of the residual was a constant), here BOTH sides
// of the residual move: moving geometry changes WHERE in the target image
// we sample (same as before), and moving color changes WHAT the mesh
// predicts there (new). Getting the joint Jacobian's block structure
// right -- 8 parameter blocks per patch (4 geometry corners, then 4 color
// corners), each with correct partial derivatives and correctly *zero*
// cross terms where there's no dependence -- is exactly the kind of thing
// that's easy to get subtly wrong and have it silently produce a locally
// "working" but mathematically incorrect solve. Hence: GradientChecker
// first, before writing a single line of the real optimizer integration.
//
// If this passes, Phase 1 (full port: add color smoothness/ridge terms,
// wire into MeshOptimizer.cpp behind a new useCeresJoint flag) proceeds
// with much lower risk of a silent Jacobian bug undermining the whole
// experiment.

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

// Mirrors MeshOptimizerCeres.cpp's areaWeightAt() exactly.
double areaWeightAt(const GradientMesh& mesh, int pr, int pc, double u, double v, double duv) {
    Vec2 dU, dV;
    mesh.evalPos(pr, pc, u, v, &dU, &dV);
    double a = std::abs(dU.cross(dV)) * duv;
    return std::max(a, 1e-6);
}

// One residual block per patch, covering the full data term of that patch
// (all n x n samples x 3 color channels). 8 parameter blocks:
//   blocks[0..3] = geometry corners, 6 doubles each: 0,1=P.x,P.y;
//                  2,3=Pu.x,Pu.y; 4,5=Pv.x,Pv.y (twist Puv still derived,
//                  not a parameter -- unchanged from the geometry-only
//                  spike/production code).
//   blocks[4..7] = color corners, 12 doubles each, layout
//                  [C.r,C.g,C.b, Cu.r,Cu.g,Cu.b, Cv.r,Cv.g,Cv.b,
//                   Cuv.r,Cuv.g,Cuv.b] -- unlike geometry's twist, color's
//                  Cuv IS a free unknown (matches MeshVertex/colorCorner),
//                  so all 4 PatchWeights kinds (Value/TangentU/TangentV/
//                  Twist) are live here, not just 3.
// Both corner orderings use the same (a,b) -> k=a*2+b convention as
// MeshOptimizerCeres.cpp's PatchDataCostFunction, so blocks[k] and
// blocks[4+k] refer to the SAME physical vertex.
//
// Residual: sqrt(w) * (meshColor(u,v; trial C,Cu,Cv,Cuv) -
// target.sampleBilinear(pos(u,v; trial P,Pu,Pv))) -- same sqrt(weight)
// convention as every other CostFunction in this project (see
// MeshOptimizerCeres.cpp's file header). `w` (area weight) is frozen from
// mesh_ (the snapshot/linearization point), exactly as in the
// geometry-only version, for the same reason (see that file's comment on
// PatchDataCostFunction) -- not re-litigated here, this spike is only
// about the NEW color-parameter dependency.
class JointPatchDataCostFunction : public ceres::CostFunction {
public:
    JointPatchDataCostFunction(const GradientMesh& meshTemplate, const Image& target,
                                int patchRow, int patchCol, int samplesPerEdge)
        : mesh_(meshTemplate), target_(target), pr_(patchRow), pc_(patchCol),
          n_(std::max(2, samplesPerEdge)) {
        int numSamples = (n_ + 1) * (n_ + 1);
        set_num_residuals(3 * numSamples);
        for (int k = 0; k < 4; ++k) mutable_parameter_block_sizes()->push_back(6);   // geometry
        for (int k = 0; k < 4; ++k) mutable_parameter_block_sizes()->push_back(12);  // color
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        GradientMesh mesh = mesh_;
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
                double w = areaWeightAt(mesh_, pr_, pc_, u, v, duv); // frozen, see header comment
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

                        // Geometry block: d(r0)/d(geom) = -d(ctarget)/d(geom)
                        // = -(grad . d(pos)/d(geom)) -- cmesh doesn't depend
                        // on geometry (color surface is evaluated at fixed
                        // parametric (u,v), independent of where that (u,v)
                        // physically lands). Identical to the geometry-only
                        // spike/production code.
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

                        // Color block: d(r0[c])/d(colorParam[kind*3+c2]) =
                        // sw * pw.w[base+kind] if c2==c else 0 -- ctarget
                        // doesn't depend on color (position is unaffected
                        // by the color unknowns), and cmesh's channels
                        // don't mix (R depends only on the 4 corners' R
                        // components, etc). Block-diagonal per channel.
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
    const GradientMesh& mesh_;
    const Image& target_;
    int pr_, pc_, n_;
};

// Same sharp-edge synthetic test image as the geometry-only spike -- a
// flat image would let a wrong geometry-Jacobian pass trivially (grad=0
// everywhere), and the color-Jacobian check needs the residual to
// actually be nonzero/nontrivial too.
Image makeEdgeTestImage(int w, int h) {
    Image img(w, h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double t = double(x) / w;
            double edge = 1.0 / (1.0 + std::exp(-18.0 * (t - 0.5)));
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

    GradientMesh mesh = GradientMesh::buildInitial(3, 3, boundary, target);

    // Nudge geometry AND color off their initial values so the check isn't
    // evaluated at some accidentally-degenerate symmetric point.
    for (auto& mv : mesh.vertices) {
        mv.P.x += 0.7; mv.P.y -= 0.4;
        mv.Pu.x *= 1.1; mv.Pu.y *= 0.9;
        mv.Pv.x *= 0.95; mv.Pv.y *= 1.05;
        mv.C.r = std::min(1.0, mv.C.r * 1.1 + 0.03);
        mv.C.g = std::min(1.0, mv.C.g * 0.9 + 0.02);
        mv.C.b = std::min(1.0, mv.C.b * 1.05);
        mv.Cu.r += 0.04; mv.Cu.g -= 0.03; mv.Cu.b += 0.02;
        mv.Cv.r -= 0.02; mv.Cv.g += 0.05; mv.Cv.b -= 0.01;
        mv.Cuv.r += 0.01; mv.Cuv.g += 0.01; mv.Cuv.b -= 0.02;
    }

    int patchRow = 0, patchCol = 0;
    int samplesPerEdge = 4;
    JointPatchDataCostFunction cost(mesh, target, patchRow, patchCol, samplesPerEdge);

    double* params[8];
    std::vector<std::array<double, 6>> geomBlocks(4);
    std::vector<std::array<double, 12>> colorBlocks(4);
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            int k = a * 2 + b;
            const MeshVertex& mv = mesh.at(patchRow + b, patchCol + a);
            geomBlocks[k] = {mv.P.x, mv.P.y, mv.Pu.x, mv.Pu.y, mv.Pv.x, mv.Pv.y};
            params[k] = geomBlocks[k].data();
            colorBlocks[k] = {mv.C.r, mv.C.g, mv.C.b, mv.Cu.r, mv.Cu.g, mv.Cu.b,
                               mv.Cv.r, mv.Cv.g, mv.Cv.b, mv.Cuv.r, mv.Cuv.g, mv.Cuv.b};
            params[4 + k] = colorBlocks[k].data();
        }
    }

    ceres::NumericDiffOptions numDiffOpts;
    ceres::GradientChecker checker(&cost, nullptr, numDiffOpts);
    ceres::GradientChecker::ProbeResults results;
    bool ok = checker.Probe(params, 1e-6, &results);

    std::printf("=== Ceres GradientChecker spike (JOINT geometry+color data term) ===\n");
    std::printf("Probe() returned: %s\n", ok ? "true (within tolerance)" : "false (MISMATCH)");
    std::printf("Maximum relative error: %.6g\n", results.maximum_relative_error);
    if (!ok) {
        std::printf("---- error log ----\n%s\n", results.error_log.c_str());
    }
    std::printf("(residual dim=%d, 4 geometry blocks x 6 + 4 color blocks x 12)\n", cost.num_residuals());
    return ok ? 0 : 1;
}
