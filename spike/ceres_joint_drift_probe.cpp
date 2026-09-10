// spike/ceres_joint_drift_probe.cpp
//
// Empirical check of the README's "useCeresJoint repeated-run drift"
// diagnosis: JointGeomStepDampingCostFunction's damping reference
// (`snapshot`) is re-anchored fresh at the START of every single
// jointSolveOnce call (see MeshOptimizerCeres.cpp), which -- per that
// code-reading analysis -- damps a large jump WITHIN one call but does
// nothing to stop slow drift ACROSS many separate top-level calls. That
// analysis was never verified with a real run because this project's
// usual Linux sandbox has no Ceres/cmake available; this spike is meant
// to be built and run on a real Mac (which has Ceres via Homebrew) to
// close that gap.
//
// What this does: builds ONE initial mesh, then calls
// MeshOptimizer::optimizeCoarseToFine on the SAME mesh object repeatedly
// (not rebuilding it between calls) -- exactly what happens when a user
// clicks the app's "Optimize" button several times in a row on the same
// mesh. Repeats this for three solver modes (hand-rolled, useCeresGeometry,
// useCeresJoint) so joint's behavior can be compared against the other
// two under the identical repeated-call pattern, and reports RMSE plus
// mean/max control-point displacement after each repeat.
//
// Standalone, throwaway (see spike/CMakeLists.txt's own header comment) --
// not wired into the root build, does not touch MeshOptimizer.cpp/
// MeshOptimizerCeres.cpp, just calls their existing public API.

#include "gmcore/GradientMesh.h"
#include "gmcore/Image.h"
#include "gmcore/MeshOptimizer.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace gmcore;

namespace {

GradientMesh buildRectMesh(int rows, int cols, int margin, const Image& target) {
    double x0 = margin, y0 = margin, x1 = target.width - margin, y1 = target.height - margin;
    std::array<CubicBezier, 4> boundary;
    boundary[0] = {Vec2{x0, y0}, Vec2{x0 + (x1 - x0) / 3, y0}, Vec2{x0 + 2 * (x1 - x0) / 3, y0}, Vec2{x1, y0}};
    boundary[1] = {Vec2{x1, y0}, Vec2{x1, y0 + (y1 - y0) / 3}, Vec2{x1, y0 + 2 * (y1 - y0) / 3}, Vec2{x1, y1}};
    boundary[2] = {Vec2{x1, y1}, Vec2{x0 + 2 * (x1 - x0) / 3, y1}, Vec2{x0 + (x1 - x0) / 3, y1}, Vec2{x0, y1}};
    boundary[3] = {Vec2{x0, y1}, Vec2{x0, y0 + 2 * (y1 - y0) / 3}, Vec2{x0, y0 + (y1 - y0) / 3}, Vec2{x0, y0}};
    return GradientMesh::buildInitial(rows, cols, boundary, target);
}

// Mean and max Euclidean displacement of every control point's P between
// two mesh snapshots of identical rows/cols.
void meshDisplacement(const GradientMesh& a, const GradientMesh& b, double* meanOut, double* maxOut) {
    double sum = 0.0, mx = 0.0;
    int n = (int)a.vertices.size();
    for (int i = 0; i < n; ++i) {
        double dx = a.vertices[i].P.x - b.vertices[i].P.x;
        double dy = a.vertices[i].P.y - b.vertices[i].P.y;
        double d = std::sqrt(dx * dx + dy * dy);
        sum += d;
        mx = std::max(mx, d);
    }
    *meanOut = n ? sum / n : 0.0;
    *maxOut = mx;
}

void runMode(const char* label, const Image& target, int rows, int cols, int margin,
             int pyramidLevels, int repeats, bool useCeresGeom, bool useCeresJoint) {
    std::printf("\n=== %s ===\n", label);
#ifndef GMCORE_WITH_CERES
    if (useCeresGeom || useCeresJoint) {
        std::printf("(this binary was built WITHOUT Ceres -- OptimizerOptions silently falls back\n"
                     " to the hand-rolled path; this mode's numbers will just repeat the\n"
                     " hand-rolled ones and prove nothing about the Ceres-specific drift claim)\n");
    }
#endif
    OptimizerOptions opts;
    opts.useCeresGeometry = useCeresGeom;
    opts.useCeresJoint = useCeresJoint;

    GradientMesh mesh = buildRectMesh(rows, cols, margin, target);
    GradientMesh prev = mesh;
    double rmse0 = mesh.reconstructionRMSE(target, 6);
    std::printf("repeat   RMSE       meanDisp   maxDisp\n");
    std::printf("initial  %.6f   --         --\n", rmse0);

    for (int r = 1; r <= repeats; ++r) {
        prev = mesh; // snapshot BEFORE this repeat's optimize call
        MeshOptimizer::optimizeCoarseToFine(mesh, target, {}, pyramidLevels, opts, nullptr);
        double rmse = mesh.reconstructionRMSE(target, 6);
        double meanDisp, maxDisp;
        meshDisplacement(mesh, prev, &meanDisp, &maxDisp);
        std::printf("%-8d %.6f   %.6f   %.6f\n", r, rmse, meanDisp, maxDisp);
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string inputPath = "gradient.png";
    int rows = 5, cols = 5, margin = 6, pyramidLevels = 4, repeats = 6;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() { return std::string(argv[++i]); };
        if (a == "--input") inputPath = next();
        else if (a == "--rows") rows = std::stoi(next());
        else if (a == "--cols") cols = std::stoi(next());
        else if (a == "--margin") margin = std::stoi(next());
        else if (a == "--pyramid-levels") pyramidLevels = std::stoi(next());
        else if (a == "--repeats") repeats = std::stoi(next());
        else { std::fprintf(stderr, "Unknown arg: %s\n", a.c_str()); return 1; }
    }

#ifdef GMCORE_WITH_CERES
    std::printf("Built WITH Ceres -- useCeresGeometry/useCeresJoint are real.\n");
#else
    std::printf("Built WITHOUT Ceres -- useCeresGeometry/useCeresJoint modes below are a NO-OP\n"
                "(silently fall back to hand-rolled); this run cannot test the real question.\n");
#endif

    Image target = Image::load(inputPath);
    std::printf("Loaded %s (%dx%d), mesh %dx%d, %d repeats of a %d-level "
                "optimizeCoarseToFine call on the SAME mesh (mimics clicking\n"
                "\"Optimize\" repeatedly in the app).\n",
                inputPath.c_str(), target.width, target.height, rows, cols, repeats, pyramidLevels);

    runMode("Hand-rolled (baseline)", target, rows, cols, margin, pyramidLevels, repeats, false, false);
    runMode("useCeresGeometry", target, rows, cols, margin, pyramidLevels, repeats, true, false);
    runMode("useCeresJoint", target, rows, cols, margin, pyramidLevels, repeats, false, true);

    std::printf("\nWhat to look for: does useCeresJoint's RMSE keep improving and its\n"
                "meanDisp/maxDisp keep shrinking across repeats (converging, like the other\n"
                "two modes should), or does RMSE stop improving / get WORSE while\n"
                "meanDisp/maxDisp stays large (still moving a lot without actually improving\n"
                "-- i.e. drifting) even after several repeats?\n");
    return 0;
}
