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

// True iff this binary was compiled with Ceres available (GMCORE_WITH_CERES
// defined) -- i.e. whether OptimizerOptions::useCeresGeometry/useCeresJoint
// can actually take effect, or silently no-op back to the hand-rolled path
// (see MeshOptimizer.cpp's jointSolvedByCeres/geometrySolvedByCeres comments
// and the one-time stderr warning). Exposed as a real function (defined in
// MeshOptimizer.cpp, which is unconditionally compiled with whatever the
// project's actual GMCORE_WITH_CERES setting is) rather than left for each
// caller to test the macro itself, because a caller's own translation unit
// (e.g. mac/DocumentModel.mm) isn't guaranteed to see the same preprocessor
// definition the core library was built with -- this is exactly the kind of
// "solver silently fell back and nobody noticed" gap the debug-data export
// (DocumentModel -exportDebugDataToURL:error:) exists to catch.
bool builtWithCeres();

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
    double geomDataWeight = 1.0;      // multiplies the geometry step's PHOTOMETRIC data term only
                                       // (MeshOptimizer.cpp's computeGeometryEnergy w*d.lengthSq()
                                       // and the matching accumulateGNRow calls in
                                       // optimizeAtCurrentResolution; mirrored in
                                       // MeshOptimizerCeres.cpp's PatchDataCostFunction/
                                       // JointPatchDataCostFunction via sw=sqrt(w*geomDataWeight)) --
                                       // NOT the color-solve data term (solveColorExact has no such
                                       // knob; see smoothWeightColor/colorDerivRidge below, which are
                                       // already scale-covariant with their own data term and need no
                                       // equivalent). Exists because this term's magnitude is tied to
                                       // the TARGET IMAGE'S local color gradient (ColorGrad from
                                       // target.sampleGradient), so it scales with whatever working
                                       // color-space magnitude the target is expressed in, while
                                       // smoothWeightGeom/geomTangentPriorWeight/boundaryWeight/
                                       // vectorLineWeight are pure position quantities that don't --
                                       // e.g. switching the target from sRGB (~[0,1]) to raw CIELUV
                                       // (~[0,100]) grows this term's contribution to the Gauss-Newton
                                       // normal equations by roughly (100)^2=10000x relative to those
                                       // fixed regularizers (H's diagonal is weight*coeff^2, and coeff
                                       // here embeds the image gradient -- see accumulateGNRow), which
                                       // is exactly what let geometry bend aggressively onto sharp
                                       // edges (and, unchecked, produce border artifacts/shuffled
                                       // patches) in this project's first CIELUV attempt, before
                                       // ColorSpace.h's kCIELUVWorkingScale=100 pulled CIELUV back to
                                       // sRGB's own magnitude and this term's relative strength along
                                       // with it. Default 1.0 reproduces current/sRGB-equivalent
                                       // behavior unchanged (including the current CIELUV mode, which
                                       // is pre-normalized to sRGB's own scale). Raising it lets the
                                       // photometric term outweigh the position regularizers again
                                       // without having to know to shrink four different fields by the
                                       // same factor and without touching smoothWeightColor/
                                       // colorDerivRidge (unaffected either way -- see above).
    double smoothWeightColor = 4.0;   // 2nd-difference regularization on control-point base color
    double colorDerivRidge = 1e-3;    // small ridge on Cu,Cv,Cuv for a well-posed linear solve
    double boundaryWeight = 200.0;    // soft pull of boundary vertices back onto their spline,
                                       // NORMAL direction only (see MeshOptimizer.cpp's
                                       // computeGeometryEnergy comment) -- along-curve sliding is
                                       // always free, matching the paper's "control points on the
                                       // boundary only move along the splines" (Sec 4)
    double vectorLineWeight = 20.0;   // soft alignment of the analytic surface tangent (dm/du,
                                       // dm/dv) to nearby user guide lines, evaluated at every
                                       // dense data-term sample -- rewritten to match the paper's
                                       // Sec 4.2 formula exactly (Eq. in that section): for each
                                       // sample, find the globally nearest guide line, compute a
                                       // Gaussian weight wu/wv = G(d | 0, sigma^2) where d is the
                                       // distance to that line and sigma = (line's own polyline
                                       // length / 5) / 3 (i.e. sigma is 1/3 of a "narrow band" whose
                                       // width is 1/5 of the line's length -- both straight from the
                                       // paper's text), hard-cut to zero outside that band, and add
                                       // weight * (perp-component of the tangent along the line
                                       // direction)^2. This replaced an older, coarser approximation
                                       // that only checked discrete straight mesh-edges (not the
                                       // analytic Hermite tangent) against edge midpoints within a
                                       // single flat global pixel radius -- see git history and
                                       // MeshOptimizer.cpp's nearestVectorLineField for the current,
                                       // paper-faithful version. Default 20.0 matches the paper's
                                       // stated beta=20 default for this term.
    double vectorLineInfluenceRadius = 25.0; // UNUSED by the current (Sec-4.2-faithful) vector-line
                                       // term -- the effective band width is now derived per-line
                                       // from that line's own length (see vectorLineWeight above),
                                       // not from a single flat radius. Left in OptimizerOptions,
                                       // inert, for ABI/config-file compatibility; may be removed
                                       // later if nothing external depends on it.
    double geomTangentPriorWeight = 0.6; // soft pull of free Pu/Pv toward the position-implied
                                       // finite-difference estimate (GradientMesh::tangentU/V),
                                       // re-anchored every GN sub-iteration. 0 = fully free (as in
                                       // the paper); very large = old fully-derived behavior. Needed
                                       // because Pu/Pv are free unknowns (see GradientMesh.h's
                                       // MeshVertex comment) and, unlike a position, a tangent has
                                       // no sensible "pull toward zero" prior -- zero would collapse the
                                       // patch -- so this grounds them near a sane default instead.
                                       // Puv is NOT included here: it's fixed at {0,0} per the
                                       // paper's Sec 3 ("the values of muv are usually set to
                                       // zero"), not free and not derived -- see GradientMesh.h.
    int outerIterationsPerLevel = 40;  // upper CEILING, not a target -- see
                                       // outerConvergenceRelTol below. Was 8;
                                       // raised after a report that running
                                       // the hand-rolled optimizer a SECOND
                                       // time on its own already-optimized
                                       // output kept reducing RMSE
                                       // substantially (one 5x5 gradient.png
                                       // case: 0.0315 -> 0.0260) -- i.e. 8
                                       // outer iterations/level was simply
                                       // stopping before convergence, not
                                       // reaching a real local optimum.
                                       // Reproduced directly: on that same
                                       // test case, 1 pass at outer-iters=8
                                       // gave RMSE 0.03147; running that same
                                       // 8-iteration pass twice in a row
                                       // (16 total, but through the full
                                       // coarse-to-fine pyramid schedule
                                       // twice) reached 0.03101; a single
                                       // pass at outer-iters=40 (through the
                                       // pyramid schedule once) reached
                                       // 0.02817 -- clearly still a real gap
                                       // at 8, and 40 closes most of it in
                                       // one pass. See outerConvergenceRelTol
                                       // for why raising this ceiling 5x
                                       // doesn't make every run 5x slower.
    double outerConvergenceRelTol = 1e-3; // early-exit the outer loop once
                                       // the per-outer-iteration relative
                                       // improvement in computeGeometryEnergy
                                       // (the same composite data+vector-line
                                       // +smoothness+tangent-prior+boundary
                                       // energy backtracking already checks
                                       // every GN sub-iteration) drops below
                                       // this fraction -- added alongside the
                                       // outerIterationsPerLevel bump above
                                       // so an already-converged case (e.g.
                                       // the smooth synthetic sphere) still
                                       // stops in a handful of iterations
                                       // instead of always burning the full,
                                       // now much higher, ceiling. Set to 0
                                       // to disable early-exit entirely and
                                       // always run the full
                                       // outerIterationsPerLevel count (the
                                       // old, pre-this-change behavior,
                                       // modulo the new default ceiling).
                                       //
                                       // Value tuned by sweeping 1e-5..3e-3 on
                                       // three cases (synthetic sphere 9x9,
                                       // gradient.png 25x25 and 5x5): 1e-5 is
                                       // needlessly tight -- it costs 24-40%
                                       // more wall-clock than 1e-3 on EVERY
                                       // case (e.g. sphere: 6703ms vs 4027ms;
                                       // gradient 25x25: 42750ms vs 32583ms)
                                       // for no measurable RMSE benefit (often
                                       // slightly worse, since a Levenberg
                                       // step accepted late can still be a
                                       // small net negative -- stopping a
                                       // touch earlier isn't strictly a
                                       // quality tradeoff here). 1e-3 matches
                                       // or beats 1e-5's RMSE on all three
                                       // cases while being consistently
                                       // faster. Looser still (3e-3) starts
                                       // to cost real quality on the hard
                                       // case (5x5 gradient.png RMSE 0.03107
                                       // vs 0.03041 at 1e-3 -- most of the
                                       // fix's benefit over the old default
                                       // is lost), so 1e-3 is the sweet spot,
                                       // not just "looser is always fine."
    int outerConvergencePatience = 3; // number of CONSECUTIVE stalled outer
                                       // iterations (relative improvement
                                       // below outerConvergenceRelTol)
                                       // required before the early-exit
                                       // above actually stops the loop.
                                       // Added after a follow-up report that
                                       // "run it twice" still kept improving
                                       // RMSE even with a single-iteration
                                       // version of this check: a rejected
                                       // Gauss-Newton step reverts the mesh
                                       // and bumps the Levenberg damping
                                       // (lambda) 4x, so that outer
                                       // iteration's energy is unchanged for
                                       // a reason that has nothing to do
                                       // with having reached a real local
                                       // optimum -- a single-iteration check
                                       // can't tell that apart from genuine
                                       // convergence and stops right there,
                                       // which is exactly what re-running
                                       // (which resets lambda back down) was
                                       // then able to undo. Requiring several
                                       // stalled iterations in a row before
                                       // stopping gives lambda room to work
                                       // back down and try again first. See
                                       // MeshOptimizer.cpp's early-exit block
                                       // for the full explanation. 1 recovers
                                       // the old (buggy) single-iteration
                                       // behavior.
    int pyramidRestarts = 1; // number of times optimizeCoarseToFine repeats
                                       // its FULL descend-to-coarsest /
                                       // climb-to-finest sweep. Added after
                                       // isolating a SEPARATE, smaller
                                       // phenomenon from the
                                       // outerConvergencePatience bug above:
                                       // once that bug was fixed, calling
                                       // optimizeAtCurrentResolution
                                       // (single, FIXED resolution) twice in
                                       // a row on its own output showed an
                                       // honest ~0.00% gap -- genuine
                                       // convergence, confirmed. But calling
                                       // the FULL optimizeCoarseToFine
                                       // pipeline twice still showed a small
                                       // real gap (~0.2-0.3% RMSE per repeat
                                       // on a 5x5 gradient.png test case).
                                       // Root cause: every call re-descends
                                       // the mesh to the COARSEST pyramid
                                       // level and re-climbs -- on the
                                       // second call this happens from an
                                       // already-refined mesh instead of the
                                       // crude initial one, and because this
                                       // is non-convex block-coordinate
                                       // descent, that different starting
                                       // point can (and measurably does)
                                       // land in a marginally better basin
                                       // by the time it reaches the finest
                                       // level again -- structurally a
                                       // multi-restart effect, not a
                                       // stopping-criterion bug (there's
                                       // nothing wrong with any single
                                       // level's convergence; each level
                                       // genuinely reaches a local optimum
                                       // given ITS starting point). This
                                       // field automates exactly the manual
                                       // "run it again" workaround in a
                                       // single call: 1 (default) reproduces
                                       // the old single-sweep behavior
                                       // exactly; 2-3 harvests most of the
                                       // measured gain cheaply (each restart
                                       // costs roughly one more full
                                       // optimizeCoarseToFine call, ~1-1.4s
                                       // on that same 5x5 test case -- scale
                                       // accordingly for larger meshes).
                                       // Left at 1 by default rather than
                                       // silently multiplying every job's
                                       // runtime; the macOS app / CLI can
                                       // opt in explicitly (see
                                       // --pyramid-restarts).
    int geomGaussNewtonItersPerOuter = 3;
    int cgMaxIterations = 200;
    double cgRelTolerance = 1e-5;
    double geomDampingInitial = 1e-2; // relative Levenberg damping added to the GN normal equations

    // ceres::Solver::Options::num_threads for EVERY ceres::Solve call this
    // binary makes (useCeresGeometry's ceresSolveOnce and useCeresJoint's
    // jointSolveOnce, see MeshOptimizerCeres.cpp's resolveCeresNumThreads).
    // 0 (default) = auto: use every core std::thread::hardware_concurrency()
    // reports (falling back to 1 if that returns 0, which the standard
    // technically allows). A positive value pins that exact thread count --
    // in particular 1 forces the old (pre-this-option) single-threaded
    // behavior. Exists as a toggle/knob rather than silently always-on
    // multithreading for two reasons: (1) it's the natural way to A/B
    // "is a result different/worse because of this" against a
    // single-threaded run of the SAME weights -- SPARSE_NORMAL_CHOLESKY's
    // per-call Jacobian evaluation and sparse factorization can, in
    // principle, sum floating-point contributions in a different order
    // across threads than single-threaded, which (unlike the exact GN
    // normal-equations assembly in the hand-rolled path) COULD make a
    // multithreaded run's result not bit-identical to a single-threaded one
    // even for the same inputs, though it should not be responsible for a
    // qualitatively different (better/worse-LOOKING) result -- each
    // ceres::Solve here re-solves an independent, freshly-snapshotted
    // problem from scratch (see MeshOptimizerCeres.cpp's header comment on
    // the freezing convention), so there's no cross-call state threading
    // could disturb; and (2) some environments (a sandboxed CI runner, a
    // machine already saturated by other work) may want to cap Ceres to
    // fewer threads than the core count. Ignored entirely on the
    // hand-rolled path (which has no threading of its own yet).
    int ceresNumThreads = 0;

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
    // ceres::Problem solving position (P,Pu,Pv) and ALL FOUR free
    // color unknowns (C,Cu,Cv,Cuv) jointly, in one nonlinear least-squares
    // problem per re-snapshot -- the fully-joint solve described in the
    // class header comment above ("rather than one fully-joint LM
    // solve... this implementation uses block-coordinate descent"), which
    // was previously blocked by the hand-rolled SparseBlockMatrix's fixed
    // `double tmp[16]` scratch buffers (a joint block is 6+12=18 doubles/
    // vertex). Ceres has no such ceiling and doesn't even need geometry and
    // color unified into one parameter block -- see MeshOptimizerCeres.cpp's
    // optimizeJointCeres/jointSolveOnce.
    //
    // Motivation: investigating why even the geometry-only Ceres path
    // (useCeresGeometry) still can't reproduce the paper's Fig. 4 pinch on
    // a 5x5 mesh -- every indirect fix tried (free tangents, tangent-
    // prior, anisotropic smoothWeightGeom) only relaxes resistance to
    // pinching, none of them add a force that couples geometry and color
    // tightly enough to produce it. A true joint solve, where a color
    // discontinuity can pull geometry toward it in the SAME step that
    // geometry's own data term does, is a plausible candidate for that
    // missing coupling. (NOTE: an earlier version of this comment cited a
    // paper "fold-over technique, Sec. 3.4" as a fallback explanation if
    // joint-solve didn't close the gap -- that section/technique does not
    // exist in the actual paper, see the project history; the paper's real,
    // already-implemented mechanism for sharp edges is Sec 4.2's vector-
    // line guidance, VectorLine/vectorLineWeight above.)
    //
    // If both this and useCeresGeometry are true, this one wins (color
    // solve and geometry solve are both replaced; useCeresGeometry's
    // separate geometry-only Ceres call never runs that outer iteration).
    // Same graceful-fallback contract as useCeresGeometry: ignored with a
    // one-time stderr warning when GMCORE_WITH_CERES wasn't defined.
    bool useCeresJoint = false;

    // useCeresJoint-only: soft penalty (see MeshOptimizerCeres.cpp's
    // JointGeomStepDampingCostFunction) on how far P/Pu/Pv move, per
    // vertex, within a SINGLE jointSolveOnce call, relative to the mesh
    // state that call started from. Exists to curb a real, measured
    // failure mode: useCeresJoint's combined 18-unknowns-per-vertex
    // (position+color) linearized step can trade geometric plausibility
    // for a better LOCAL color fit at the dense data-term quadrature
    // samples, because moving a vertex changes where those samples land on
    // the target image just as surely as changing color changes what gets
    // compared there -- something neither the hand-rolled alternating
    // scheme nor useCeresGeometry's frozen-color residual can do, since
    // neither ever has position and "the thing being matched against" both
    // free in the same linearized step. Confirmed via three real,
    // debug-data-exported 9x9-mesh runs on the same image (see README):
    // hand-rolled RMSE 0.0156, useCeresGeometry 0.0166 (+6.6%), useCeresJoint
    // 0.0234 (+50%, and ~2x slower) -- with useCeresJoint's vertex positions
    // in the image's hardest region diverging from the other two by up to
    // ~15px and Pu/Pv tangent magnitudes shrinking ~30-40%, right where its
    // per-patch RMSE was worst. This weight makes moving P/Pu/Pv "cost"
    // something in the joint least-squares objective (same idea as
    // geomTangentPriorWeight/colorDerivRidge already keeping OTHER unknowns
    // grounded), so Ceres's own internal LM iterations are discouraged from
    // PROPOSING such a step in the first place -- it does NOT replace the
    // alpha-backtracking gate in optimizeJointCeres (still catches a bad
    // full step after the fact via computeTrueJointEnergy, which this
    // weight's contribution is also folded into for accept/reject
    // consistency -- see that function's geomStepReference parameter).
    // 0 disables it entirely (residual blocks skipped). NOT YET TUNED
    // against a real Ceres build -- this starting value is a reasoned
    // guess, not a measured optimum; please re-run useCeresJoint on the
    // same mesh/image with Export Debug Data (or the auto-export
    // checkbox) and compare its RMSE/vertex-position spread against this
    // baseline, then adjust up (more damping, closer to useCeresGeometry's
    // behavior/quality but less "true joint" benefit) or down (less
    // damping, closer to the original unconstrained joint) from there.
    double jointGeomStepDampingWeight = 0.3;
};

// Exact closed-form solve for the 4 free color unknowns (C,Cu,Cv,Cuv) at
// every vertex, given FIXED geometry -- the data term is linear in color
// (unlike position, which enters nonlinearly through the target image
// lookup), so this isn't a linearized approximation the way the geometry
// Gauss-Newton step is: it's the literal, exact minimizer of (data +
// color-smoothness + color-ridge) energy for the mesh's current shape.
// Factored out of optimizeAtCurrentResolution's block-coordinate-descent
// color step (declared here, not file-local, specifically so
// MeshOptimizerCeres.cpp's optimizeJointCeres can call it too -- see that
// function's comment for why a guaranteed-exact color pass matters even
// for the joint solve). Mutates mesh.vertices[*].C/Cu/Cv/Cuv in place;
// does not touch geometry (P/Pu/Pv) at all.
void solveColorExact(GradientMesh& mesh, const Image& target, const OptimizerOptions& opts);

struct OptimizerProgress {
    int pyramidLevel = 0;
    int totalPyramidLevels = 1;
    int outerIteration = 0;
    int totalOuterIterations = 0;
    double rmse = 0.0;
    // Pixel dimensions of the pyramid level THIS callback fired at (i.e.
    // what mesh.P/Pu/Pv/boundary are currently scaled to -- see
    // optimizeCoarseToFine's mesh.scalePositions calls: the mesh lives in
    // level pyramidLevel's downsampled coordinate space for the whole
    // duration of that level's optimizeAtCurrentResolution call, only
    // getting rescaled up once that level finishes entirely). Together with
    // the caller's own full-resolution (level 0) target image dimensions,
    // this is exactly what's needed to map a mesh snapshot taken mid-level
    // back to full-res/level-0 pixel space:
    //   GradientMesh::scalePositions(fullResWidth / (double)levelWidth,
    //                                 fullResHeight / (double)levelHeight)
    // -- see DocumentModel.mm's livePreviewDuringOptimize snapshot, the
    // motivating use case: without this, the live-preview mesh grid would
    // be drawn shrunk into a corner of the (always full-res) canvas at
    // every pyramid level except the finest (0 itself, where the factor is
    // trivially 1.0). 0 before the first callback of a run, same as every
    // other field's zero-initialized default.
    int levelWidth = 0;
    int levelHeight = 0;
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
