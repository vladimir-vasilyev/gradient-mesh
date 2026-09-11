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
// UPDATED with a second diagnosis attempt after the first confirmed run
// (see README): every optimizer path (hand-rolled included) actually
// minimizes an AREA-WEIGHTED data term internally (areaWeightAt: the true
// image-space area each parametric quadrature sample covers, |dU x dV| *
// duv -- see MeshOptimizer.cpp/MeshOptimizerCeres.cpp), not the plain
// per-parametric-sample average that reconstructionRMSE (the number
// actually reported to the user and printed by this probe) computes.
// Those two measures can legitimately disagree if the mesh becomes very
// non-uniform in image-space area (some patches tiny, others huge): the
// area-weighted internal energy under-counts a poorly-fit but tiny-area
// patch and over-counts a poorly-fit huge-area one relative to
// reconstructionRMSE's flat per-sample average, which weighs every
// parametric sample equally regardless of the area it actually covers.
// useCeresJoint's fully-coupled 18-unknowns-per-vertex step (documented
// in JointGeomStepDampingCostFunction's own header comment as able to
// trade position accuracy for a locally-better color fit) is a plausible
// candidate for exploiting exactly this gap far more than the other two
// paths' decoupled position/color updates ever get the chance to. This
// revision adds a `energy` column (computeTrueJointEnergy -- the same
// composite, area-weighted objective optimizeJointCeres's own
// verify-and-shrink gate already checks every sub-step, called here with
// no step-damping reference since this is a plain post-hoc readout, not a
// gated comparison) alongside RMSE for all three modes, specifically to
// see whether `energy` keeps decreasing (or holds flat) while `RMSE`
// creeps up -- which would confirm this second hypothesis -- or whether
// `energy` ALSO increases (which would mean this hypothesis is wrong and
// something else is going on).
//
// UPDATED AGAIN after the energy-column run (see README): `energy` moved
// WITH `RMSE`, not against it, on both mesh sizes tested -- refuting the
// H2 (area-weighted-vs-naive-RMSE) hypothesis above. energy is computed
// fresh every time against the SAME full-resolution target/opts, so an
// increase in it is a genuine regression in the true composite objective,
// not a metric-mismatch artifact. Reading optimizeCoarseToFine(): each
// repeat first scales the mesh DOWN to the COARSEST pyramid level and
// re-optimizes there against a blurred/downsampled target, then climbs
// back up through progressively sharper levels to level 0 (full-res).
// On a mesh that's already well-fit to the SHARP full-res image, that
// initial coarse-level re-fit can move geometry toward whatever is
// optimal for the BLURRY image -- which need not be what's optimal for
// the sharp one -- and the climb back up has no guarantee of fully
// undoing that. This (H3) would explain the data better: it's not
// Ceres/joint-specific (the coarse-to-fine mechanism is shared by all
// three modes, and hand-rolled did show a smaller version of the same
// drift), just apparently worse for useCeresJoint's fully-coupled step.
// This revision adds a per-pyramid-level trace (piggybacking on the
// EXISTING OptimizerProgressCallback -- no changes to MeshOptimizer.cpp/
// MeshOptimizerCeres.cpp needed) that snapshots the mesh once per level,
// rescales that snapshot to full resolution (per OptimizerProgress's own
// documented recipe), and reports ITS RMSE/energy against the real
// full-res target -- to see directly whether full-res quality dips when
// the sweep is down at a coarse level and how much of that dip survives
// the climb back to level 0.
//
// Standalone, throwaway (see spike/CMakeLists.txt's own header comment) --
// not wired into the root build, does not touch MeshOptimizer.cpp/
// MeshOptimizerCeres.cpp (only reads two of its already non-static, just
// not header-declared, functions via the extern forward declarations
// below), just calls their existing public API otherwise.

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

#ifdef GMCORE_WITH_CERES
namespace gmcore {
// Forward declarations of MeshOptimizerCeres.cpp internals -- defined
// there (external linkage, just not declared in any header since they
// were never meant to be called from outside that file until now). Exact
// signatures copied verbatim; see that file for the real definitions and
// full documentation of what each term means.
double computeTrueGeometryEnergy(const GradientMesh& mesh, const Image& target,
                                  const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts);
double computeTrueJointEnergy(const GradientMesh& mesh, const Image& target,
                               const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts,
                               const std::vector<MeshVertex>* geomStepReference);
} // namespace gmcore
#endif

namespace {

GradientMesh buildRectMesh(int rows, int cols, int margin, const Image& target) {
    double x0 = margin, y0 = margin, x1 = target.width - margin, y1 = target.height - margin;
    // BezierSpline (one-or-more cubic segments per side) replaced plain
    // CubicBezier for GradientMesh::boundary since this spike was last
    // touched (see README's "Adaptive multi-segment boundary fitting") --
    // a rectangle's sides are already exactly straight, so each is just
    // wrapped as a single-segment spline, identical geometry as before.
    std::array<BezierSpline, 4> boundary;
    boundary[0].segments = {CubicBezier{Vec2{x0, y0}, Vec2{x0 + (x1 - x0) / 3, y0}, Vec2{x0 + 2 * (x1 - x0) / 3, y0}, Vec2{x1, y0}}};
    boundary[1].segments = {CubicBezier{Vec2{x1, y0}, Vec2{x1, y0 + (y1 - y0) / 3}, Vec2{x1, y0 + 2 * (y1 - y0) / 3}, Vec2{x1, y1}}};
    boundary[2].segments = {CubicBezier{Vec2{x1, y1}, Vec2{x0 + 2 * (x1 - x0) / 3, y1}, Vec2{x0 + (x1 - x0) / 3, y1}, Vec2{x0, y1}}};
    boundary[3].segments = {CubicBezier{Vec2{x0, y1}, Vec2{x0, y0 + 2 * (y1 - y0) / 3}, Vec2{x0, y0 + (y1 - y0) / 3}, Vec2{x0, y0}}};
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
#ifdef GMCORE_WITH_CERES
    // No step-damping reference here (nullptr) -- this is a plain post-hoc
    // readout of the same composite objective optimizeJointCeres's gate
    // checks every sub-step, not a gated before/after comparison, so there
    // is no "step start" to reference. See file header comment.
    double energy0 = computeTrueJointEnergy(mesh, target, {}, opts, nullptr);
    std::printf("repeat   RMSE       energy         meanDisp   maxDisp\n");
    std::printf("initial  %.6f   %.6f     --         --\n", rmse0, energy0);
#else
    std::printf("repeat   RMSE       meanDisp   maxDisp\n");
    std::printf("initial  %.6f   --         --\n", rmse0);
#endif

    // H3 diagnostic (see file header): fires once per OUTER GN iteration
    // during optimizeCoarseToFine (existing hook, MeshOptimizer.cpp/
    // MeshOptimizer.h -- no core changes needed), but only PRINTS the
    // first firing seen for each NEW pyramidLevel value, i.e. one row per
    // pyramid level per repeat, right after that level's first outer
    // iteration -- enough to see the trajectory across levels without
    // flooding the log with every one of up to outerIterationsPerLevel=40
    // iterations. `mesh` is captured by reference and read at whatever
    // state it's in at that instant (the callback fires synchronously,
    // after that iteration's update -- see MeshOptimizer.cpp) -- a COPY is
    // taken immediately so rescaling it to full-res doesn't disturb the
    // live optimization. Rescale recipe is exactly what OptimizerProgress's
    // own doc comment prescribes: level pixel dims -> full-res pixel dims.
    int lastLevelSeen = -1;
    auto traceProgress = [&](const OptimizerProgress& p) {
        if (p.pyramidLevel == lastLevelSeen) return;
        lastLevelSeen = p.pyramidLevel;
        if (p.levelWidth <= 0 || p.levelHeight <= 0) return; // shouldn't happen inside a real callback
        GradientMesh snapshot = mesh;
        double sx = double(target.width) / double(p.levelWidth);
        double sy = double(target.height) / double(p.levelHeight);
        snapshot.scalePositions(sx, sy);
        double fullResRmse = snapshot.reconstructionRMSE(target, 6);
#ifdef GMCORE_WITH_CERES
        double fullResEnergy = computeTrueJointEnergy(snapshot, target, {}, opts, nullptr);
        std::printf("    [level %d/%d outer %2d @ %4dx%-4d] levelRMSE=%.6f  fullResRMSE=%.6f  fullResEnergy=%.6f\n",
                    p.pyramidLevel, p.totalPyramidLevels, p.outerIteration, p.levelWidth, p.levelHeight,
                    p.rmse, fullResRmse, fullResEnergy);
#else
        std::printf("    [level %d/%d outer %2d @ %4dx%-4d] levelRMSE=%.6f  fullResRMSE=%.6f\n",
                    p.pyramidLevel, p.totalPyramidLevels, p.outerIteration, p.levelWidth, p.levelHeight,
                    p.rmse, fullResRmse);
#endif
    };

    for (int r = 1; r <= repeats; ++r) {
        prev = mesh; // snapshot BEFORE this repeat's optimize call
        lastLevelSeen = -1; // reset so this repeat's coarsest level prints its own row
        MeshOptimizer::optimizeCoarseToFine(mesh, target, {}, pyramidLevels, opts, traceProgress);
        double rmse = mesh.reconstructionRMSE(target, 6);
        double meanDisp, maxDisp;
        meshDisplacement(mesh, prev, &meanDisp, &maxDisp);
#ifdef GMCORE_WITH_CERES
        double energy = computeTrueJointEnergy(mesh, target, {}, opts, nullptr);
        std::printf("%-8d %.6f   %.6f     %.6f   %.6f\n", r, rmse, energy, meanDisp, maxDisp);
#else
        std::printf("%-8d %.6f   %.6f   %.6f\n", r, rmse, meanDisp, maxDisp);
#endif
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

#ifdef GMCORE_WITH_CERES
    std::printf("\nWhat to look for (updated -- see file header for the full second-hypothesis\n"
                "writeup): first, does useCeresJoint's RMSE drift the way the original probe\n"
                "found (stops improving / gets WORSE across repeats while meanDisp/maxDisp\n"
                "stays large)? If so, look at ITS OWN energy column: does `energy` keep\n"
                "decreasing or hold flat while `RMSE` climbs? That would confirm the\n"
                "area-weighted-internal-energy-vs-naive-RMSE divergence hypothesis -- the\n"
                "optimizer is legitimately doing its job by its own (area-weighted) yardstick,\n"
                "the yardstick just doesn't match what reconstructionRMSE reports. If `energy`\n"
                "ALSO climbs alongside RMSE, that hypothesis is wrong and something else is\n"
                "causing the drift (e.g. the gate/damping mechanics themselves). Also sanity-\n"
                "check hand-rolled/useCeresGeometry: their `energy` and `RMSE` columns should\n"
                "move together (both down) every repeat, since those paths don't have joint's\n"
                "coupled 18-unknowns-per-vertex step -- if they don't, something is off with\n"
                "the energy readout itself, not with useCeresJoint.\n");
#else
    std::printf("\nWhat to look for: does useCeresJoint's RMSE keep improving and its\n"
                "meanDisp/maxDisp keep shrinking across repeats (converging, like the other\n"
                "two modes should), or does RMSE stop improving / get WORSE while\n"
                "meanDisp/maxDisp stays large (still moving a lot without actually improving\n"
                "-- i.e. drifting) even after several repeats? (Built without Ceres -- rerun\n"
                "on a Mac with Ceres to also get the `energy` column and test the second,\n"
                "more specific hypothesis; see file header.)\n");
#endif
    return 0;
}
