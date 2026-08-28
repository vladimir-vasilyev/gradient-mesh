// MeshOptimizer.h — fits a GradientMesh to a target image by minimizing
// the same energy Sun et al. 2007 describe (Sec. 4): a data (reconstruction
// error) term, a smoothness regularizer, a boundary-spline constraint and
// an optional vector-line direction constraint, solved with a
// Levenberg-Marquardt-style scheme and coarse-to-fine over a Gaussian
// pyramid.
//
// Implementation note (see README "Known simplifications"): rather than
// one fully-joint LM solve over all (position, tangent, color) unknowns,
// this implementation uses block-coordinate descent: colors (which enter
// the data term *linearly*) are solved exactly via one sparse linear
// solve, and geometry (which enters *nonlinearly*, through the target
// image lookup) is refined with a handful of damped Gauss-Newton steps
// with backtracking -- both stages minimize the same energy functional,
// just alternating rather than jointly.
#pragma once
#include "gmcore/GradientMesh.h"
#include "gmcore/VectorLine.h"
#include <functional>
#include <vector>

namespace gmcore {

struct OptimizerOptions {
    int samplesPerPatchEdge = 6;      // data-term sampling density per patch, per axis
    double smoothWeightGeom = 0.02;   // 2nd-difference regularization on control-point positions.
                                       // Empirically tuned (was 40, then 2, now 0.02 -- see README
                                       // "Known simplifications" for the full history): even at 2,
                                       // a *coarse* mesh's whole interior line still couldn't snap
                                       // onto a sharp internal edge the way the paper's Fig. 4 shows
                                       // -- position moved sub-pixel amounts regardless of how much
                                       // data-term signal was actually present (confirmed via
                                       // GMCORE_DEBUG_GEOM instrumentation, not just guessed). This
                                       // value was found together with making Pu/Pv free unknowns
                                       // (see GradientMesh.h) and adding geomTangentPriorWeight
                                       // below to keep THOSE grounded: with geomTangentPriorWeight
                                       // also set to ~0 (no grounding at all for the free tangents),
                                       // this produced visibly garbled reconstructions (wildly
                                       // inconsistent per-vertex Pu/Pv, folded-looking patches) even
                                       // though the coarse-sample RMSE metric still looked fine --
                                       // so a low smoothWeightGeom is only safe paired with a
                                       // meaningfully nonzero geomTangentPriorWeight.
    double smoothGeomEdgeGain = 40.0; // Makes smoothWeightGeom ANISOTROPIC: at a vertex sitting
                                       // near a strong local image gradient, the effective
                                       // smoothing weight is smoothWeightGeom / (1 + smoothGeomEdgeGain
                                       // * localGradientMagnitude) (floored by smoothGeomMinFactor
                                       // below), instead of the flat smoothWeightGeom everywhere.
                                       // Rationale: the isotropic 2nd-difference term keeps
                                       // neighboring control points evenly spaced, which is exactly
                                       // what must NOT hold for two mesh-lines to squeeze together
                                       // around a sharp edge (Fig. 4-style pinching) -- relaxing it
                                       // specifically where the image justifies it lets that happen
                                       // without giving up smoothing in flat regions. 0 = isotropic
                                       // (old behavior, no relaxation).
    double smoothGeomMinFactor = 0.05; // floor on the relaxation factor above -- never fully zero
                                       // out smoothing even directly on the sharpest edge, so the
                                       // geometry solve stays well-posed.
    double smoothWeightColor = 4.0;   // 2nd-difference regularization on control-point base color
    double colorDerivRidge = 1e-3;    // small ridge on Cu,Cv,Cuv for a well-posed linear solve
    double boundaryWeight = 200.0;    // soft pull of boundary vertices back onto their spline
    double vectorLineWeight = 60.0;   // soft alignment of nearby mesh edges to user guide lines
    double vectorLineInfluenceRadius = 25.0; // pixels, in the *current pyramid level's* scale
    double geomTangentPriorWeight = 0.6; // soft pull of free Pu/Pv toward the position-implied
                                       // finite-difference estimate (GradientMesh::tangentU/V),
                                       // re-anchored every GN sub-iteration. 0 = fully free (as in
                                       // the paper); very large = old fully-derived behavior. Needed
                                       // because Pu/Pv are now free unknowns (see GradientMesh.h's
                                       // MeshVertex comment) and, unlike a position, a tangent has no
                                       // sensible "pull toward zero" prior -- zero would collapse the
                                       // patch -- so this grounds them near a sane default instead.
    int outerIterationsPerLevel = 8;
    int geomGaussNewtonItersPerOuter = 3;
    int cgMaxIterations = 200;
    double cgRelTolerance = 1e-5;
    double geomDampingInitial = 1e-2; // relative Levenberg damping added to the GN normal equations

    // Opt-in: replace the hand-rolled geometry Gauss-Newton block (assembly
    // of H/g, our own sparse block CG, manual Levenberg damping and
    // backtracking) with a ceres::Problem solve, when this binary was built
    // with Ceres available (GMCORE_WITH_CERES). The residuals/analytic
    // Jacobians are transcribed 1:1 from the hand-rolled path (see
    // MeshOptimizerCeres.cpp) and were cross-checked against
    // ceres::GradientChecker / a standalone finite-difference harness
    // before being wired in -- see spike/ceres_geom_spike.cpp and the
    // commit history for that verification. Defaults to false: the
    // dependency-free hand-rolled path remains the default for everyone
    // without Ceres installed, and behaves byte-for-byte as before when
    // this flag is left off even on a Ceres-enabled build. Ignored (with a
    // one-time stderr warning) if GMCORE_WITH_CERES was not defined at
    // build time.
    bool useCeresGeometry = false;

    // Opt-in, stronger than useCeresGeometry: replace BOTH the closed-form
    // color linear solve AND the geometry Gauss-Newton block with a single
    // ceres::Problem solving position (P,Pu,Pv), tangents and ALL FOUR
    // free color unknowns (C,Cu,Cv,Cuv) jointly, in one nonlinear least-
    // squares problem per re-snapshot -- the fully-joint solve described
    // in the class header comment above ("rather than one fully-joint LM
    // solve... this implementation uses block-coordinate descent"), which
    // was previously blocked by the hand-rolled SparseBlockMatrix's fixed
    // `double tmp[16]` scratch buffers (a joint block is 6+12=18 doubles/
    // vertex). Ceres has no such ceiling and doesn't even need geometry
    // and color unified into one parameter block -- see
    // MeshOptimizerCeres.cpp's optimizeJointCeres/jointSolveOnce.
    //
    // Motivation: investigating why even the geometry-only Ceres path
    // (useCeresGeometry) still can't reproduce the paper's Fig. 4 pinch on
    // a 5x5 mesh -- every indirect fix tried (free tangents, tangent-
    // prior, anisotropic smoothWeightGeom) only relaxes resistance to
    // pinching, none of them add a force that couples geometry and color
    // tightly enough to produce it. A true joint solve, where a color
    // discontinuity can pull geometry toward it in the SAME step that
    // geometry's own data term does, is a plausible candidate for that
    // missing coupling; if it still doesn't reproduce Fig. 4, that's
    // evidence the paper's fold-over technique (Sec. 3.4) is doing
    // something block-coordinate/joint optimization alone can't.
    //
    // If both this and useCeresGeometry are true, this one wins (color
    // solve and geometry solve are both replaced; useCeresGeometry's
    // separate geometry-only Ceres call never runs that outer iteration).
    // Same graceful-fallback contract as useCeresGeometry: ignored with a
    // one-time stderr warning when GMCORE_WITH_CERES wasn't defined.
    bool useCeresJoint = false;
};

struct OptimizerProgress {
    int pyramidLevel = 0;
    int totalPyramidLevels = 1;
    int outerIteration = 0;
    int totalOuterIterations = 0;
    double rmse = 0.0;
};
using OptimizerProgressCallback = std::function<void(const OptimizerProgress&)>;

class MeshOptimizer {
public:
    // Runs coarse-to-fine optimization in place on `mesh` (which must
    // already be sized/positioned for the FINEST (full-resolution) level --
    // it gets internally rescaled down for the coarse levels and rescaled
    // back up as it proceeds). `vectorLinesFullRes` are in full-resolution
    // pixel coordinates.
    static void optimizeCoarseToFine(GradientMesh& mesh, const Image& fullResTarget,
                                      const std::vector<VectorLine>& vectorLinesFullRes,
                                      int numPyramidLevels, const OptimizerOptions& opts,
                                      const OptimizerProgressCallback& cb = nullptr);

    // Single-resolution entry point (used internally, and directly useful
    // for small images / testing without a pyramid).
    static void optimizeAtCurrentResolution(GradientMesh& mesh, const Image& target,
                                             const std::vector<VectorLine>& vectorLines,
                                             const OptimizerOptions& opts,
                                             const OptimizerProgressCallback& cb, int level, int totalLevels);
};

#ifdef GMCORE_WITH_CERES
// Implemented in MeshOptimizerCeres.cpp (only compiled in when Ceres was
// found at configure time -- see CMakeLists.txt). Replaces exactly the
// geometry Gauss-Newton block inside optimizeAtCurrentResolution's outer
// loop; the color linear-solve step around it is untouched. Mutates
// mesh.vertices[*].P/Pu/Pv in place, same contract as the hand-rolled path.
void optimizeGeometryCeres(GradientMesh& mesh, const Image& target,
                            const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts);

// Also in MeshOptimizerCeres.cpp: the fully-joint (geometry+color) solve,
// see OptimizerOptions::useCeresJoint above. Replaces both the color
// linear-solve step and the geometry GN block; mutates
// mesh.vertices[*].P/Pu/Pv/C/Cu/Cv/Cuv in place.
void optimizeJointCeres(GradientMesh& mesh, const Image& target,
                         const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts);
#endif

} // namespace gmcore
