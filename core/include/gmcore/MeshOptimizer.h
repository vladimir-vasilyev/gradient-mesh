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
    double smoothWeightGeom = 2.0;    // 2nd-difference regularization on control-point positions.
                                       // Empirically tuned: much above this (the original default
                                       // was 40) the regularizer dominates the -- comparatively
                                       // small, once colors have absorbed most of the local error --
                                       // data-term gradient, and mesh-lines stop bending to follow
                                       // internal color/gradient boundaries at all, no matter how
                                       // sharp the underlying edge is. See README "Known
                                       // simplifications" for the related line-search-vs-objective
                                       // bug this was found alongside.
    double smoothWeightColor = 4.0;   // 2nd-difference regularization on control-point base color
    double colorDerivRidge = 1e-3;    // small ridge on Cu,Cv,Cuv for a well-posed linear solve
    double boundaryWeight = 200.0;    // soft pull of boundary vertices back onto their spline
    double vectorLineWeight = 60.0;   // soft alignment of nearby mesh edges to user guide lines
    double vectorLineInfluenceRadius = 25.0; // pixels, in the *current pyramid level's* scale
    int outerIterationsPerLevel = 8;
    int geomGaussNewtonItersPerOuter = 3;
    int cgMaxIterations = 200;
    double cgRelTolerance = 1e-5;
    double geomDampingInitial = 1e-2; // relative Levenberg damping added to the GN normal equations
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

} // namespace gmcore
