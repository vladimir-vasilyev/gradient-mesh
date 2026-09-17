#import "DocumentModel.h"
#import <ImageIO/ImageIO.h>
#import <CoreGraphics/CoreGraphics.h>
#include "gmcore/Image.h"
#include "gmcore/GradientMesh.h"
#include "gmcore/MeshOptimizer.h"
#include "gmcore/SVGExporter.h"
#include "gmcore/ColorSpace.h"
#include "gmcore/MeshRenderBuffers.h"
#include "gmcore/LazySnapping.h"
#include "gmcore/ContourTracing.h"
#include <vector>
#include <array>
#include <memory>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <random>

using gmcore::Vec2;
using gmcore::Color;
using gmcore::CubicBezier;
using gmcore::BezierSpline;
using gmcore::GradientMesh;
using gmcore::MeshVertex;
using gmcore::Image;
using gmcore::VectorLine;
using gmcore::OptimizerOptions;
using gmcore::OptimizerProgress;

static NSError* gmError(NSString* msg) {
    return [NSError errorWithDomain:@"GradientMeshStudio" code:1 userInfo:@{NSLocalizedDescriptionKey: msg}];
}

// --- Collision helper for "Animate Mesh" (see -animationTick:) ---

// Standard parametric segment-segment intersection test: [A,B] and [C,D]
// cross iff both t (how far along A->B) and u (how far along C->D) land
// in [0,1]. A small epsilon widens that range slightly so a crossing
// exactly at an endpoint counts as a hit rather than being lost to
// floating-point rounding. Parallel/collinear segments (rxs ~= 0) are
// reported as a miss -- an edge-on-edge slide in that exact configuration
// is ambiguous anyway, and it's a measure-zero case in practice. On a hit,
// outT receives t (0..1 along A->B), used by the caller to pick the
// EARLIEST crossing along a vertex's intended path when it would cross
// more than one edge in a single tick.
static bool gmSegmentsIntersect(const Vec2& A, const Vec2& B, const Vec2& C, const Vec2& D, double* outT) {
    Vec2 r = B - A;
    Vec2 s = D - C;
    double rxs = r.cross(s);
    if (std::fabs(rxs) < 1e-9) return false;
    Vec2 qp = C - A;
    double t = qp.cross(s) / rxs;
    double u = qp.cross(r) / rxs;
    const double kEps = 1e-6;
    if (t < -kEps || t > 1.0 + kEps || u < -kEps || u > 1.0 + kEps) return false;
    if (outT) *outT = t;
    return true;
}

// Evaluates the exact same cubic Bezier convention
// -meshEdgeBezierFromRow:col:toRow:col: uses to draw a mesh edge -- B0=P0,
// B1=P0+T0/3, B2=P1-T1/3, B3=P1 -- at parameter t in [0,1].
static Vec2 gmCubicBezierAt(const Vec2& B0, const Vec2& B1, const Vec2& B2, const Vec2& B3, double t) {
    double mt = 1.0 - t;
    double a = mt * mt * mt, b = 3 * mt * mt * t, c = 3 * mt * t * t, d = t * t * t;
    return Vec2(a * B0.x + b * B1.x + c * B2.x + d * B3.x, a * B0.y + b * B1.y + c * B2.y + d * B3.y);
}

// Shortest distance from point P to segment [A,B] -- clamps the
// projection of P onto the segment's line to [0,1] first, so a point
// beyond either endpoint measures against that endpoint rather than the
// segment's infinite extension.
static double gmPointSegmentDistance(const Vec2& P, const Vec2& A, const Vec2& B) {
    Vec2 ab = B - A;
    double len2 = ab.dot(ab);
    if (len2 < 1e-12) return (P - A).length(); // degenerate (near-zero-length) segment
    double t = std::max(0.0, std::min(1.0, (P - A).dot(ab) / len2));
    Vec2 proj = A + ab * t;
    return (P - proj).length();
}

// Shortest distance between two segments [A0,A1] and [B0,B1] -- 0.0 if
// they cross (gmSegmentsIntersect), otherwise the smallest of the four
// endpoint-to-opposite-segment distances (the standard robust reduction:
// for two straight segments, the closest pair of points is always either
// a genuine crossing or one segment's endpoint against the other).
static double gmSegmentSegmentDistance(const Vec2& A0, const Vec2& A1, const Vec2& B0, const Vec2& B1) {
    double t;
    if (gmSegmentsIntersect(A0, A1, B0, B1, &t)) return 0.0;
    double d1 = gmPointSegmentDistance(A0, B0, B1);
    double d2 = gmPointSegmentDistance(A1, B0, B1);
    double d3 = gmPointSegmentDistance(B0, A0, A1);
    double d4 = gmPointSegmentDistance(B1, A0, A1);
    return std::min(std::min(d1, d2), std::min(d3, d4));
}

// Full-mesh self-intersection check over the ACTUAL rendered mesh edges --
// the cubic Ferguson-patch-edge Bezier curves
// -meshEdgeBezierFromRow:col:toRow:col: draws between grid-adjacent
// vertices, NOT the straight P-to-P wireframe an earlier version of this
// check used. That straight-wireframe check turned out to be
// insufficient: two edges whose straight P-to-P segments never cross can
// still have their real curves cross, because each curve can bulge toward
// the other one, given tangents (Pu/Pv) large enough relative to the
// edge's own length -- exactly what a fitted mesh's tangents can look
// like around a sharp local silhouette feature. Confirmed with a
// standalone repro during development: two perfectly parallel straight
// rows, given matching perpendicular tangents, produced curves that both
// bulge to the exact same midpoint by construction, even though the rows
// themselves never get any closer together.
//
// Two DISTINCT things are checked, with two DIFFERENT criteria, because
// they turned out to need them:
//
// 1. A single edge's own curve looping back and crossing ITSELF. A cubic
// Bezier's tangent magnitudes are fixed (Pu/Pv, read from the frozen base
// mesh), but its CHORD length changes as its endpoints animate -- at
// large amplitude a chord can shrink, or its direction can flip relative
// to where it started, while the tangent stays exactly as large as it
// was sized for the ORIGINAL chord. That combination is the classic
// condition for a cubic Bezier to fold back on itself. This is checked
// with the EXACT crossing test only (gmSegmentsIntersect, zero tolerance)
// -- NOT the clearance distance below -- because a perfectly smooth,
// non-looping curve can slow down enough in one stretch that two
// non-adjacent samples end up just as close in space as a genuine tiny
// loop's crossing point would be. A standalone repro confirmed both cases
// can have near-identical (even sub-pixel) separation ALONG the curve, so
// there is no reliable way to tell "just slow there" from "a real fold"
// from polyline samples alone; applying the clearance margin here
// produced false positives on ordinary, perfectly clean meshes during
// development. Segments are only exempted from this check when they are
// truly ADJACENT within the same curve (share a sample point, and so
// trivially "touch") -- unlike an earlier version of this function, which
// skipped EVERY pair from the same curve and so could never see this at
// all.
//
// 2. Two DIFFERENT edges' curves coming closer than `minClearanceDistance`
// to each other, not just literally crossing. Even a mesh with zero
// technical self-intersections can still show visible rendering artifacts
// (thin slivers, near-degenerate overlaps) once two different patch
// boundaries drift close enough together, so this check treats "too
// close" the same as "crossed" for any two edges that don't share a mesh
// vertex (edges that DO share a vertex legitimately meet there and are
// exempted entirely, same as before).
//
// `pos` gives every vertex's CURRENT (possibly animated) position;
// `baseMesh` supplies the tangents (Pu/Pv), which this animation never
// touches, so they always come from the frozen base mesh regardless of
// which frame's positions are being checked. Each grid edge's curve is
// tessellated into a `samplesPerEdge`-segment polyline, and polyline
// SEGMENTS (not whole curves) are broad-phased through the same flat
// uniform grid the old straight-wireframe check used -- bucketed by each
// little segment's own tiny bounding box, padded by minClearanceDistance
// so a near (not just overlapping) pair still lands in a shared bucket;
// bucketCellSize itself remains a performance tuning value only, never a
// correctness one. Verified (standalone prototype) against a synthetic
// cross-edge crossing case, the self-loop case above, a battery of clean
// regular grids at varied cell sizes/tangent magnitudes (zero false
// positives), a constructed near-miss that passes within the clearance
// margin without literally crossing, and a stress sweep reproducing a
// real large-amplitude run (9x9 mesh, amplitude=100, temperature=10):
// every frame the bisection in -startMeshAnimationWithRedraw: corrects
// re-verifies clean afterward.
static bool gmMeshHasCurvedSelfIntersection(const std::vector<Vec2>& pos, const GradientMesh& baseMesh,
                                             int samplesPerEdge, double bucketCellSize, double minClearanceDistance,
                                             int* outA1, int* outB1, int* outA2, int* outB2) {
    int rows = baseMesh.rows, cols = baseMesh.cols;
    struct GMEdgeCurveRef { int a, b, firstSample, sampleCount; };
    std::vector<GMEdgeCurveRef> edgeRefs;
    std::vector<Vec2> allSamples;
    edgeRefs.reserve((size_t)(rows * cols) * 2);
    allSamples.reserve((size_t)(rows * cols) * 2 * (size_t)(samplesPerEdge + 1));

    auto addEdge = [&](int a, int b, const Vec2& T0, const Vec2& T1) {
        Vec2 P0 = pos[(size_t)a], P1 = pos[(size_t)b];
        Vec2 B0 = P0, B1 = P0 + T0 * (1.0 / 3.0), B2 = P1 - T1 * (1.0 / 3.0), B3 = P1;
        GMEdgeCurveRef ref{a, b, (int)allSamples.size(), samplesPerEdge + 1};
        for (int i = 0; i <= samplesPerEdge; ++i) {
            allSamples.push_back(gmCubicBezierAt(B0, B1, B2, B3, (double)i / samplesPerEdge));
        }
        edgeRefs.push_back(ref);
    };
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int idxA = r * cols + c;
            if (c + 1 < cols) { // horizontal edge: tangent at each end is Pu
                int idxB = idxA + 1;
                addEdge(idxA, idxB, baseMesh.vertices[(size_t)idxA].Pu, baseMesh.vertices[(size_t)idxB].Pu);
            }
            if (r + 1 < rows) { // vertical edge: tangent at each end is Pv
                int idxB = idxA + cols;
                addEdge(idxA, idxB, baseMesh.vertices[(size_t)idxA].Pv, baseMesh.vertices[(size_t)idxB].Pv);
            }
        }
    }

    double minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
    for (const Vec2& p : allSamples) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    double clearance = std::max(0.0, minClearanceDistance);
    double margin = clearance; // pad AABBs so a near (not just overlapping) pair still shares a bucket
    double cell = bucketCellSize > 1e-6 ? bucketCellSize : 1.0;
    int gw = std::max(1, std::min(4096, (int)std::floor((maxX - minX) / cell) + 2));
    int gh = std::max(1, std::min(4096, (int)std::floor((maxY - minY) / cell) + 2));
    std::vector<std::vector<int>> grid((size_t)gw * (size_t)gh); // stores segment-global-index

    std::vector<int> segToEdge;
    std::vector<int> segFirstSample; // index of the segment's first sample within allSamples
    std::vector<int> segLocalIndex;  // this segment's index WITHIN its own curve (0-based)
    for (size_t e = 0; e < edgeRefs.size(); ++e) {
        const GMEdgeCurveRef& ref = edgeRefs[e];
        for (int s = 0; s + 1 < ref.sampleCount; ++s) {
            segToEdge.push_back((int)e);
            segFirstSample.push_back(ref.firstSample + s);
            segLocalIndex.push_back(s);
        }
    }
    auto clampIdx = [](int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); };
    for (size_t si = 0; si < segToEdge.size(); ++si) {
        const Vec2& P = allSamples[(size_t)segFirstSample[si]];
        const Vec2& Q = allSamples[(size_t)segFirstSample[si] + 1];
        int cx0 = clampIdx((int)std::floor((std::min(P.x, Q.x) - margin - minX) / cell), 0, gw - 1);
        int cx1 = clampIdx((int)std::floor((std::max(P.x, Q.x) + margin - minX) / cell), 0, gw - 1);
        int cy0 = clampIdx((int)std::floor((std::min(P.y, Q.y) - margin - minY) / cell), 0, gh - 1);
        int cy1 = clampIdx((int)std::floor((std::max(P.y, Q.y) + margin - minY) / cell), 0, gh - 1);
        for (int cy = cy0; cy <= cy1; ++cy)
            for (int cx = cx0; cx <= cx1; ++cx)
                grid[(size_t)(cy * gw + cx)].push_back((int)si);
    }

    for (int cy = 0; cy < gh; ++cy) {
        for (int cx = 0; cx < gw; ++cx) {
            const std::vector<int>& idxs = grid[(size_t)(cy * gw + cx)];
            for (size_t i = 0; i < idxs.size(); ++i) {
                for (size_t j = i + 1; j < idxs.size(); ++j) {
                    int s1 = idxs[i], s2 = idxs[j];
                    int e1 = segToEdge[(size_t)s1], e2 = segToEdge[(size_t)s2];
                    const Vec2& A0 = allSamples[(size_t)segFirstSample[(size_t)s1]];
                    const Vec2& A1 = allSamples[(size_t)segFirstSample[(size_t)s1] + 1];
                    const Vec2& C0 = allSamples[(size_t)segFirstSample[(size_t)s2]];
                    const Vec2& C1 = allSamples[(size_t)segFirstSample[(size_t)s2] + 1];
                    if (e1 == e2) {
                        // Same curve -- exact-crossing self-loop check only
                        // (see this function's comment for why the
                        // clearance margin doesn't apply here). Adjacent
                        // samples (sharing a sample point) always
                        // trivially "touch" and are skipped; anything
                        // farther apart that still crosses is a real fold.
                        int li1 = segLocalIndex[(size_t)s1], li2 = segLocalIndex[(size_t)s2];
                        if (std::abs(li1 - li2) <= 1) continue;
                        double t;
                        if (gmSegmentsIntersect(A0, A1, C0, C1, &t)) {
                            const GMEdgeCurveRef& r1 = edgeRefs[(size_t)e1];
                            if (outA1) *outA1 = r1.a;
                            if (outB1) *outB1 = r1.b;
                            if (outA2) *outA2 = r1.a;
                            if (outB2) *outB2 = r1.b;
                            return true;
                        }
                    } else {
                        const GMEdgeCurveRef& r1 = edgeRefs[(size_t)e1];
                        const GMEdgeCurveRef& r2 = edgeRefs[(size_t)e2];
                        // Skip curve pairs that share a mesh vertex -- they
                        // legitimately meet AT that shared endpoint.
                        if (r1.a == r2.a || r1.a == r2.b || r1.b == r2.a || r1.b == r2.b) continue;
                        if (gmSegmentSegmentDistance(A0, A1, C0, C1) < clearance) {
                            if (outA1) *outA1 = r1.a;
                            if (outB1) *outB1 = r1.b;
                            if (outA2) *outA2 = r2.a;
                            if (outB2) *outB2 = r2.b;
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}
// One frame of the precomputed "Animate Mesh" loop, for
// -exportMeshAnimationTraceToURL:error: (see DocumentModel.h and that
// method's comment) -- NOT used by the animation itself (playback just
// indexes into _animCachedFrames, see -animationTick:), purely a
// debugging record of what the one-time precompute in
// -startMeshAnimationWithRedraw: did. Since the whole animation is now a
// single fixed-length loop rather than an open-ended real-time run, one
// entry is recorded per precomputed frame, always including its final
// positions -- no thinning/truncation policy needed, unlike the old
// real-time trace this replaces (the frame count is bounded to a few
// hundred by construction, not by an arbitrary cap).
struct GMAnimTraceFrame {
    double t = 0.0; // this frame's phase within the loop, in radians (see -startMeshAnimationWithRedraw:)
    bool curveSelfIntersectionDetected = false; // did the unmodified sine target for this frame cross itself (curve-aware)?
    int xa1 = -1, xb1 = -1, xa2 = -1, xb2 = -1; // offending edge endpoints if so, else -1
    bool bisectionEngaged = false; // did precompute have to scale this frame's displacement back toward the base mesh?
    double lambdaApplied = 1.0; // 1.0 = unmodified sine target used; see -startMeshAnimationWithRedraw:
    std::vector<Vec2> positions; // this frame's final (possibly corrected) vertex positions -- always recorded
};

// --- Debug-data export helpers (see -exportDebugDataToURL:error:) ---

// Finds the git repo root by walking up from THIS SOURCE FILE'S OWN
// compile-time path (__FILE__) looking for a .git directory/file (a git
// worktree's .git is a file, not a directory, hence -fileExistsAtPath:
// rather than checking isDirectory). This works because Xcode compiles
// this .mm from its real location in the working copy and __FILE__
// captures whatever absolute path was actually passed to the compiler --
// it does NOT depend on the app's runtime bundle location (which, for a
// built .app, has nothing to do with where the source/repo lives) or on
// any build-time-generated version header. Returns nil (not a hard
// failure) if no .git is found within a handful of parent directories --
// e.g. if this binary was ever built from a source tree copied out from
// under its .git, or moved after building. Callers must handle nil.
static NSString* gmFindRepoRootFromSourceFile(void) {
    NSString* dir = [[NSString stringWithUTF8String:__FILE__] stringByDeletingLastPathComponent];
    NSFileManager* fm = [NSFileManager defaultManager];
    for (int i = 0; i < 8 && dir.length > 1; ++i) {
        if ([fm fileExistsAtPath:[dir stringByAppendingPathComponent:@".git"]]) return dir;
        NSString* parent = [dir stringByDeletingLastPathComponent];
        if ([parent isEqualToString:dir]) break;
        dir = parent;
    }
    return nil;
}

// Runs `git -C repoRoot <args>` and returns trimmed stdout, or nil on any
// failure (repoRoot nil, git not found, non-zero exit, ...) -- deliberately
// forgiving since this only feeds an informational debug dump, never
// something the app's own correctness depends on.
static NSString* gmRunGit(NSString* repoRoot, NSArray<NSString*>* args) {
    if (!repoRoot) return nil;
    NSTask* task = [[NSTask alloc] init];
    task.launchPath = @"/usr/bin/env";
    NSMutableArray<NSString*>* full = [NSMutableArray arrayWithObjects:@"git", @"-C", repoRoot, nil];
    [full addObjectsFromArray:args];
    task.arguments = full;
    NSPipe* outPipe = [NSPipe pipe];
    task.standardOutput = outPipe;
    task.standardError = [NSPipe pipe]; // discarded
    @try {
        [task launch];
    } @catch (NSException* __unused exc) {
        return nil;
    }
    NSData* data = [outPipe.fileHandleForReading readDataToEndOfFile];
    [task waitUntilExit];
    if (task.terminationStatus != 0) return nil;
    NSString* out = [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding];
    return [out stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
}

@interface DocumentModel () {
    gmcore::Image _target;
    // CIELUV conversion of _target, recomputed alongside it in
    // -loadImageAtURL:error: (see -workingTargetImage and
    // useCIELUVColorSpace's doc comment in DocumentModel.h). Computed
    // eagerly (a one-time, one-per-image-load cost) rather than lazily so
    // there's no first-use stall/branch to reason about.
    gmcore::Image _targetLUV;
    BOOL _hasImage;
    std::vector<Vec2> _boundaryPolygon;
    std::array<BezierSpline, 4> _boundary; // each side: one or more cubic segments (see BezierSpline.h/fitBezierSpline)
    BOOL _hasBoundary;
    std::unique_ptr<GradientMesh> _mesh;
    // Post-outer-iteration snapshot copy for livePreviewDuringOptimize (see
    // DocumentModel.h) -- nullptr except during a run that had the toggle
    // on, from the first snapshot's arrival to that run's end (reset both
    // at the start and the end of -optimizeWithPyramidLevels:...). A
    // SEPARATE mesh from _mesh, never aliased to it: the whole point is
    // that -meshForReading can hand it to the main thread while _mesh
    // itself is still being mutated by the background optimizer thread,
    // with no synchronization needed because the copy that produced it
    // happened on that SAME background thread, synchronously, between two
    // outer iterations (see the lambda in -optimizeWithPyramidLevels:...).
    std::unique_ptr<GradientMesh> _previewMesh;
    std::vector<VectorLine> _vectorLines;
    // Foreground/background scribble strokes for the Lazy-Snapping-style
    // segmentation tool (see -segmentBoundaryFromScribblesWithError: below).
    // Reuses VectorLine as a generic polyline container purely for
    // convenience (it's just {std::vector<Vec2> points;}) -- these are NOT
    // rendered as vector guide lines and have nothing to do with _vectorLines.
    std::vector<VectorLine> _fgScribbles;
    std::vector<VectorLine> _bgScribbles;
    BOOL _isOptimizing;
    double _lastRMSE;
    double _lastMAE; // see -currentMAE's doc comment in DocumentModel.h
    // Which colour space _mesh's C/Cu/Cv/Cuv are actually stored in --
    // snapshotted from self.useCIELUVColorSpace at -buildInitialMeshRows:
    // cols: time (see that method) and used everywhere _mesh's colours are
    // read/written from then on, INSTEAD OF re-reading the live
    // useCIELUVColorSpace property. This is deliberate: it's what makes
    // toggling the property after a mesh already exists harmless (a no-op
    // until the next rebuild) rather than a silent colour-space mismatch
    // between the mesh and whatever -optimizeWithPyramidLevels:... would
    // otherwise feed it.
    BOOL _meshColorSpaceIsCIELUV;
    // State captured for -exportDebugDataToURL:error: (see DocumentModel.h)
    // -- the exact OptimizerOptions the most recent
    // -optimizeWithPyramidLevels:progress:completion: call used, its full
    // per-outer-iteration progress history, whether any run has happened
    // yet, and how long it took wall-clock. Snapshotted/reset at the START
    // of each optimize call (see that method), so a debug export mid-run
    // or right after reflects the run actually in flight/just finished.
    OptimizerOptions _lastOptsUsed;
    std::vector<OptimizerProgress> _lastRunHistory;
    BOOL _hasRunOptimize;
    double _lastRunWallClockSeconds;
    // Whether the CURRENT _mesh object has completed at least one full
    // coarse-to-fine pyramid pass -- reset to NO whenever a new _mesh is
    // (re)built (-buildInitialMeshRows:cols:) or dropped (-loadImageAtURL:
    // error:, -setBoundaryPolygonPoints:), set to YES at the end of any
    // -optimizeWithPyramidLevels:progress:completion: run. See that
    // method's use of this: a repeat "Optimize" click on a mesh that's
    // already had a full pass refines at full resolution ONLY, skipping
    // the coarse levels -- see that method's comment for why.
    BOOL _meshHasHadFullPyramidPass;
    // --- "Animate Mesh" state (see DocumentModel.h's isAnimatingMesh/
    // startMeshAnimationWithRedraw:/stopMeshAnimation) -- a purely
    // cosmetic, UI-side wiggle, entirely separate from how _mesh/
    // _previewMesh are used during -optimizeWithPyramidLevels:... above.
    // Reuses _previewMesh as the live animated copy (see -meshForReading)
    // but owns its OWN frozen base snapshot here, since _previewMesh
    // during an optimize run means something different (a post-iteration
    // snapshot of the mesh MID-OPTIMIZATION) than it does here (a copy
    // whose vertex positions get displaced sinusoidally every timer tick,
    // while _mesh itself is never touched).
    GradientMesh _animBaseMesh;
    std::vector<Vec2> _animDirections;   // per-vertex, fixed for the whole animation
    std::vector<double> _animAmplitudes; // per-vertex, sampled once at start -- see meshAnimationTemperature
    // The whole one-period animation loop, precomputed ONCE in
    // -startMeshAnimationWithRedraw: (see that method's comment) and
    // simply indexed into every tick thereafter by -animationTick: -- no
    // per-tick geometry checking left at all. Each entry is one frame's
    // full vertex position array, in the same order as
    // _animBaseMesh.vertices.
    std::vector<std::vector<Vec2>> _animCachedFrames;
    NSInteger _animFrameIndex; // index into _animCachedFrames currently applied to _previewMesh
    NSTimer* _animTimer;
    BOOL _isAnimatingMesh;
    // Caller-supplied redraw callback (see -startMeshAnimationWithRedraw:)
    // -- invoked once per timer tick, right after that tick's _previewMesh
    // update, so it's always called with fresh geometry to draw.
    void (^_animRedrawBlock)(void);
    // Average base-mesh edge length, computed once at
    // -startMeshAnimationWithRedraw: time -- passed to
    // gmMeshHasCurvedSelfIntersection as its broad-phase bucket size (a
    // performance tuning value only, not a correctness one; see that
    // function's comment).
    double _animBucketCellSize;
    // One entry per precomputed frame, filled in during the
    // -startMeshAnimationWithRedraw: precompute pass (and exported right
    // there, before a single frame has even been drawn -- see
    // -exportMeshAnimationTraceToURL:error:). Purely a debugging record of
    // what that one-time precompute did -- never read by playback itself,
    // and (unlike the old real-time trace) not cleared by -stopMeshAnimation
    // either, so it stays available to export even after the animation has
    // been stopped, until the next -startMeshAnimationWithRedraw: replaces it.
    std::vector<GMAnimTraceFrame> _animTraceFrames;
    // URL of the currently loaded image (set in -loadImageAtURL:error:) --
    // used only to locate the "DebugOut" folder for -autoExportDebugData
    // (see DocumentModel.h), sibling to wherever the image actually lives.
    NSURL* _imageURL;
}
@property (nonatomic, strong, nullable) NSImage* displayImage;
@property (nonatomic, strong, nullable) NSString* lastDebugExportPath;
@property (nonatomic, strong, nullable) NSString* lastMeshAnimationTracePath;
@property (nonatomic, strong, nullable) NSString* lastPresetSavePath;
// The Image that -buildInitialMeshRows:cols:/-optimizeWithPyramidLevels:...
// should actually fit against: _target (sRGB) or _targetLUV (CIELUV),
// chosen by the LIVE useCIELUVColorSpace property. Only ever consulted at
// -buildInitialMeshRows:cols: time -- see that method and
// _meshColorSpaceIsCIELUV's comment above for why.
- (const gmcore::Image&)workingTargetImage;

// The mesh meshVertexPositionAtRow:col:/meshVertexColorAtRow:col:/
// meshEdgeBezierFromRow:col:toRow:col: should actually read from: _mesh
// normally, or _previewMesh whenever one is available (see
// hasLivePreviewMesh's comment in DocumentModel.h) -- centralizing this
// choice here means those three methods don't each need their own
// isOptimizing/_previewMesh branch, and it can't drift between them.
- (const GradientMesh*)meshForReading;
@end

// Plain NSObject subclass declared in DocumentModel.h (see GMGPUMeshBuffers'
// doc comment there) -- needs its own @implementation even though every
// field is an auto-synthesized @property, or the compiler never emits the
// class's Objective-C metadata at all (caught at LINK time as an undefined
// "_OBJC_CLASS_$_GMGPUMeshBuffers" symbol, not at compile time -- this was
// missed originally because nothing in this dev environment can link/run
// an actual Objective-C binary to catch it; a real Xcode build did).
@implementation GMGPUMeshBuffers
@end

@implementation DocumentModel

- (instancetype)init {
    if ((self = [super init])) {
        [self resetWeightsToDefaults];
        // Default YES ("auto", every core) -- matches gmcore::OptimizerOptions::
        // ceresNumThreads' own default of 0 ("auto"), NOT this BOOL property's
        // own zero-value (which would be NO/single-threaded) -- see
        // ceresMultithreaded's comment in DocumentModel.h for the YES/NO <->
        // 0/1 mapping.
        self.ceresMultithreaded = YES;
        // "Animate Mesh" defaults -- see meshAnimationMaxAmplitude/
        // meshAnimationTemperature's doc comments in DocumentModel.h. 8px
        // is a small, clearly-visible wiggle at typical image resolutions
        // without the mesh grid swamping the underlying image; temperature
        // 1.0 is a plain uniform amplitude draw (neither "cold" nor "hot").
        self.meshAnimationMaxAmplitude = 8.0;
        self.meshAnimationTemperature = 1.0;
        // 2px -- per the user's request after seeing rendering artifacts
        // at large amplitude even once literal self-intersections were
        // fixed: reject not just a crossing but any two different edges'
        // curves coming closer than this (see
        // meshAnimationMinClearanceDistance's doc comment).
        self.meshAnimationMinClearanceDistance = 2.0;
    }
    return self;
}

// Single source of truth for these eight starting values is
// gmcore::OptimizerOptions' own member-initializers (MeshOptimizer.h) --
// default-constructing one here and copying out of it, rather than typing
// the same eight numbers again, means they can never silently drift out of
// sync with the struct's real compiled-in defaults.
- (void)resetWeightsToDefaults {
    OptimizerOptions defaults;
    self.smoothWeightGeom = defaults.smoothWeightGeom;
    self.smoothWeightColor = defaults.smoothWeightColor;
    self.colorDerivRidge = defaults.colorDerivRidge;
    self.boundaryWeight = defaults.boundaryWeight;
    self.geomTangentPriorWeight = defaults.geomTangentPriorWeight;
    self.vectorLineWeight = defaults.vectorLineWeight;
    self.geomDataWeight = defaults.geomDataWeight;
    self.smoothGeomEdgeGain = defaults.smoothGeomEdgeGain;
}

- (BOOL)hasImage { return _hasImage; }
- (NSInteger)imageWidth { return _hasImage ? _target.width : 0; }
- (NSInteger)imageHeight { return _hasImage ? _target.height : 0; }
- (BOOL)hasBoundary { return _hasBoundary; }
- (BOOL)hasMesh { return _mesh != nullptr; }
- (BOOL)isOptimizing { return _isOptimizing; }
- (NSInteger)meshRows { return _mesh ? _mesh->rows : 0; }
- (NSInteger)meshCols { return _mesh ? _mesh->cols : 0; }
- (double)currentRMSE { return _lastRMSE; }
- (double)currentMAE { return _lastMAE; }
- (double)lastRunWallClockSeconds { return _lastRunWallClockSeconds; }
- (BOOL)meshColorSpaceIsCIELUV { return _mesh ? _meshColorSpaceIsCIELUV : NO; }
- (BOOL)hasLivePreviewMesh { return _isOptimizing && _previewMesh != nullptr; }
- (BOOL)isAnimatingMesh { return _isAnimatingMesh; }
- (const GradientMesh*)meshForReading {
    // See DocumentModel.h's isAnimatingMesh comment: while animating,
    // _previewMesh holds the live wiggling copy and _mesh itself is never
    // touched -- checked FIRST and independently of hasLivePreviewMesh,
    // which is specifically about an in-flight Optimize run and stays NO
    // for the entire duration of an animation.
    if (_isAnimatingMesh && _previewMesh) return _previewMesh.get();
    return self.hasLivePreviewMesh ? _previewMesh.get() : _mesh.get();
}

#pragma mark - Image loading

- (BOOL)loadImageAtURL:(NSURL*)url error:(NSError**)error {
    CGImageSourceRef src = CGImageSourceCreateWithURL((__bridge CFURLRef)url, NULL);
    if (!src) { if (error) *error = gmError(@"Could not open image file."); return NO; }
    CGImageRef cgImage = CGImageSourceCreateImageAtIndex(src, 0, NULL);
    CFRelease(src);
    if (!cgImage) { if (error) *error = gmError(@"Could not decode image."); return NO; }

    size_t w = CGImageGetWidth(cgImage), h = CGImageGetHeight(cgImage);
    if (w == 0 || h == 0) { CGImageRelease(cgImage); if (error) *error = gmError(@"Image has zero size."); return NO; }

    std::vector<uint8_t> buffer(w * h * 4, 0);
    // Explicit sRGB, NOT CGColorSpaceCreateDeviceRGB() -- see
    // -renderReconstructionPreview's comment for the full story (a
    // reported "reconstruction looks washed out on screen, but the
    // exported PNG looks correctly saturated" bug traced to exactly this
    // call using the ambiguous/legacy "device RGB" space instead of a
    // named, unambiguous one). Used here too for the same reason and for
    // consistency: this is the conversion that turns the source file's
    // bytes (whatever color space THEY were tagged in, e.g. a wide-gamut
    // Display P3 photo) into _target's raw numeric buffer, and every
    // consumer of that buffer (the optimizer's data term, and
    // -renderReconstructionPreview's render of the fitted mesh) should be
    // working in the same well-defined color space as what actually gets
    // displayed and exported, not an ambiguous one.
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef ctx = CGBitmapContextCreate(buffer.data(), w, h, 8, w * 4, cs,
                                              kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    CGColorSpaceRelease(cs);
    if (!ctx) { CGImageRelease(cgImage); if (error) *error = gmError(@"Could not create bitmap context."); return NO; }
    CGContextSetBlendMode(ctx, kCGBlendModeCopy);
    // NO flip here -- confirmed on-device via the "[GMCORE _target
    // orientation check]" NSLog below: a plain CGContextDrawImage into a
    // freshly created (non-view-backed) CGBitmapContext already puts row 0
    // of `buffer` at the image's true TOP row. An earlier version of this
    // code added a CGContextTranslateCTM/CGContextScaleCTM(1,-1) flip here,
    // reasoning (by analogy with the well-known "CGContextDrawImage draws
    // upside down" gotcha, which is real but applies to a DIFFERENT
    // scenario -- see below) that it was needed; the diagnostic log proved
    // that reasoning backwards -- loading gradient.png with that flip in
    // place printed TL=blue/TR=yellow/BL=red/BR=green, i.e. top and bottom
    // swapped relative to the file's real corners (TL=red/TR=green/
    // BL=blue/BR=yellow). Removing the flip is what actually matches.
    //
    // For anyone re-deriving this: the classic gotcha the earlier comment
    // invoked is real, but it's specifically about drawing into a context
    // that ALREADY has a flip applied to its CTM by something else (most
    // commonly an AppKit view with isFlipped=YES compensating for its own
    // top-left/y-down convention -- see CanvasView.mm's -drawRect:, which
    // had exactly that bug for its NSImage draw and needed exactly that
    // fix). A bare CGBitmapContextCreate context here has no such
    // pre-existing flip to compensate for, so adding one manually
    // over-corrected. Every other part of this codebase (Image::loadPNG/
    // loadPPM in core/src/Image.cpp, GradientMesh's boundary/vertex
    // construction, CanvasView's click mapping, SVGExporter) already uses
    // "row 0 / y=0 = top, y grows down" -- this now matches them too.
    CGContextDrawImage(ctx, CGRectMake(0, 0, w, h), cgImage);

    Image img((int)w, (int)h);
    for (size_t y = 0; y < h; ++y) {
        for (size_t x = 0; x < w; ++x) {
            const uint8_t* px = &buffer[(y * w + x) * 4];
            img.set((int)x, (int)y, Color(px[0] / 255.0, px[1] / 255.0, px[2] / 255.0));
        }
    }
    CGContextRelease(ctx);
    CGImageRelease(cgImage);

    _target = std::move(img);
    // See useCIELUVColorSpace's doc comment in DocumentModel.h: computed
    // eagerly here, once per image load, regardless of whether CIELUV mode
    // is currently on -- so flipping the checkbox and clicking "Build
    // Initial Mesh" never has to wait on (or forget to trigger) a
    // conversion pass.
    _targetLUV = gmcore::imageSRGBToCIELUV(_target);

    // Diagnostic: log _target's 4 corner colors so orientation can be
    // checked directly against a known test image (e.g. a corner-colored
    // gradient.png) rather than inferred indirectly from how the mesh looks
    // after optimizing -- added after a report that, even after fixing the
    // on-screen display flip (see CanvasView.mm's -drawRect:), the
    // OPTIMIZED mesh still looked wrong, raising the question of whether
    // THIS flip (the CGContextDrawImage one, below) is actually correct or
    // was itself backwards. _target.at(0,0) should equal the image's real
    // top-left pixel -- for gradient.png specifically, that's red
    // (~254,1,1); top-right ~green, bottom-left ~blue, bottom-right
    // ~yellow, per the file's own actual pixel data (confirmed with an
    // independent tool, outside this app). Check Xcode's debug console
    // after loading gradient.png and compare.
    {
        int iw = (int)w, ih = (int)h;
        Color tl = _target.at(0, 0);
        Color tr = _target.at(iw - 1, 0);
        Color bl = _target.at(0, ih - 1);
        Color br = _target.at(iw - 1, ih - 1);
        NSLog(@"[GMCORE _target orientation check] TL=(%.0f,%.0f,%.0f) TR=(%.0f,%.0f,%.0f) "
              @"BL=(%.0f,%.0f,%.0f) BR=(%.0f,%.0f,%.0f) -- expect TL=red TR=green BL=blue BR=yellow "
              @"for gradient.png",
              tl.r * 255, tl.g * 255, tl.b * 255, tr.r * 255, tr.g * 255, tr.b * 255,
              bl.r * 255, bl.g * 255, bl.b * 255, br.r * 255, br.g * 255, br.b * 255);
    }

    [self stopMeshAnimation]; // a new image invalidates any in-progress animation's base mesh
    _imageURL = url; // see -autoExportDebugDataIfEnabled's use of this
    _hasImage = YES;
    _hasBoundary = NO;
    _mesh.reset();
    _meshHasHadFullPyramidPass = NO;
    _vectorLines.clear();
    _fgScribbles.clear();
    _bgScribbles.clear();
    _boundaryPolygon.clear();
    self.displayImage = [[NSImage alloc] initWithContentsOfURL:url];
    return YES;
}

#pragma mark - Boundary

- (void)setBoundaryPolygonPoints:(NSArray<NSValue*>*)points {
    _boundaryPolygon.clear();
    for (NSValue* v in points) {
        NSPoint p = [v pointValue];
        _boundaryPolygon.push_back(Vec2(p.x, p.y));
    }
    _hasBoundary = NO;
    // A brand new polygon just replaced whatever boundary (if any) the
    // current mesh was built against -- that mesh (and any in-flight
    // preview snapshot of it) no longer corresponds to anything real, so
    // drop it rather than leaving a stale mesh grid rendered on screen
    // against the new boundary. Both callers of this method (manual
    // click-tracing's double-click-to-close, and
    // -segmentBoundaryFromScribblesWithError:) go through here, so this
    // one spot covers "retrace/re-segment while a mesh already exists" for
    // both paths. Harmless no-op the first time a boundary is ever set
    // (nothing built yet). CanvasView's own leftover UI state from the
    // previous boundary (_boundaryDraft, picked corner indices) is a
    // separate concern -- see MainWindowController's onBoundaryChanged/
    // -segmentBoundary: callers, which reset those via
    // -resetBoundaryDrawing/-resetCornerPicking right after calling this.
    [self stopMeshAnimation]; // the mesh this animation (if any) was based on is about to be dropped
    _mesh.reset();
    _previewMesh.reset();
    _meshHasHadFullPyramidPass = NO;
}

- (NSArray<NSValue*>*)boundaryPolygonPoints {
    NSMutableArray* arr = [NSMutableArray arrayWithCapacity:_boundaryPolygon.size()];
    for (auto& p : _boundaryPolygon) [arr addObject:[NSValue valueWithPoint:NSMakePoint(p.x, p.y)]];
    return arr;
}

- (BOOL)fitBoundaryWithCornerIndices:(NSArray<NSNumber*>*)fourIndices {
    if (fourIndices.count != 4 || _boundaryPolygon.size() < 4) return NO;
    int idx[4];
    for (int i = 0; i < 4; ++i) idx[i] = [fourIndices[i] intValue];
    int n = (int)_boundaryPolygon.size();
    for (int side = 0; side < 4; ++side) {
        int a = idx[side], b = idx[(side + 1) % 4];
        std::vector<Vec2> pts;
        int i = a;
        while (true) {
            pts.push_back(_boundaryPolygon[i]);
            if (i == b) break;
            i = (i + 1) % n;
        }
        // Adaptive multi-segment fit (was a single fitCubicBezier) --
        // see BezierSpline.h: a single cubic structurally cannot track a
        // non-convex silhouette (e.g. one produced by LazySnapping's
        // segmentation), so this now subdivides wherever the fit exceeds
        // maxErrorPixels rather than forcing the whole side through one
        // curve. A already-smooth side (the common case for a hand-picked,
        // roughly-convex boundary) still comes back as exactly one segment.
        _boundary[side] = gmcore::fitBezierSpline(pts, /*maxErrorPixels=*/3.0);
    }
    _hasBoundary = YES;
    return YES;
}

- (void)useRectangularBoundaryWithMargin:(double)marginPixels {
    if (!_hasImage) return;
    double x0 = marginPixels, y0 = marginPixels;
    double x1 = _target.width - marginPixels, y1 = _target.height - marginPixels;
    // A perfect rectangle's sides are already exactly straight -- always
    // exactly one segment, never split (fitBezierSpline would agree, but
    // there's no polyline to fit here in the first place, just the 4
    // literal corner points, so build the single-segment BezierSpline
    // directly).
    _boundary[0].segments = {CubicBezier{Vec2(x0, y0), Vec2(x0 + (x1 - x0) / 3, y0), Vec2(x0 + 2 * (x1 - x0) / 3, y0), Vec2(x1, y0)}};
    _boundary[1].segments = {CubicBezier{Vec2(x1, y0), Vec2(x1, y0 + (y1 - y0) / 3), Vec2(x1, y0 + 2 * (y1 - y0) / 3), Vec2(x1, y1)}};
    _boundary[2].segments = {CubicBezier{Vec2(x1, y1), Vec2(x0 + 2 * (x1 - x0) / 3, y1), Vec2(x0 + (x1 - x0) / 3, y1), Vec2(x0, y1)}};
    _boundary[3].segments = {CubicBezier{Vec2(x0, y1), Vec2(x0, y0 + 2 * (y1 - y0) / 3), Vec2(x0, y0 + (y1 - y0) / 3), Vec2(x0, y0)}};
    _hasBoundary = YES;
    _boundaryPolygon = {Vec2(x0, y0), Vec2(x1, y0), Vec2(x1, y1), Vec2(x0, y1)};
}

#pragma mark - Mesh

- (const gmcore::Image&)workingTargetImage {
    return self.useCIELUVColorSpace ? _targetLUV : _target;
}

- (void)buildInitialMeshRows:(NSInteger)rows cols:(NSInteger)cols {
    if (!_hasImage || !_hasBoundary) return;
    [self stopMeshAnimation]; // rebuilding replaces _mesh -- any animation of the previous instance is now stale
    // Snapshot NOW, at build time -- everything downstream (this run's
    // -optimizeWithPyramidLevels:..., -renderReconstructionPreview,
    // -exportSVGToURL:, -exportDebugDataToURL:, the vertex color
    // swatch/picker) reads THIS, not the live property, so a later toggle
    // of useCIELUVColorSpace can never mismatch against what's actually
    // stored in _mesh until the next rebuild. See its declaration's comment.
    _meshColorSpaceIsCIELUV = self.useCIELUVColorSpace;
    const gmcore::Image& target = [self workingTargetImage];
    _mesh = std::make_unique<GradientMesh>(GradientMesh::buildInitial((int)rows, (int)cols, _boundary, target));
    _meshHasHadFullPyramidPass = NO;
    _lastRMSE = _mesh->reconstructionRMSE(target, 6);
    _lastMAE = _mesh->reconstructionMAE(target, 6);
}

- (NSPoint)meshVertexPositionAtRow:(NSInteger)row col:(NSInteger)col {
    const GradientMesh* m = [self meshForReading];
    if (!m) return NSZeroPoint;
    Vec2 p = m->at((int)row, (int)col).P;
    return NSMakePoint(p.x, p.y);
}

- (NSColor*)meshVertexColorAtRow:(NSInteger)row col:(NSInteger)col {
    const GradientMesh* m = [self meshForReading];
    if (!m) return [NSColor blackColor];
    Color c = m->at((int)row, (int)col).C;
    // _mesh->C is only actually sRGB when _meshColorSpaceIsCIELUV is NO --
    // see that ivar's comment and ColorSpace.h. clamped01() must run AFTER
    // this conversion: it assumes an sRGB-range [0,1] triple, and would
    // silently mangle a raw CIELUV (L* up to 100, u*/v* often negative)
    // value if applied first.
    if (_meshColorSpaceIsCIELUV) c = gmcore::cieluvToSRGB(c);
    c = c.clamped01();
    return [NSColor colorWithCalibratedRed:c.r green:c.g blue:c.b alpha:1.0];
}

- (void)setMeshVertexPosition:(NSPoint)p atRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return;
    _mesh->at((int)row, (int)col).P = Vec2(p.x, p.y);
}

- (void)setMeshVertexColor:(NSColor*)color atRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return;
    NSColor* rgb = [color colorUsingColorSpace:[NSColorSpace deviceRGBColorSpace]];
    Color c(rgb.redComponent, rgb.greenComponent, rgb.blueComponent);
    // The color picker always hands back sRGB; convert INTO whichever space
    // _mesh actually stores (see _meshColorSpaceIsCIELUV's comment) so a
    // manual edit stays consistent with every other vertex's C field.
    if (_meshColorSpaceIsCIELUV) c = gmcore::srgbToCIELUV(c);
    _mesh->at((int)row, (int)col).C = c;
}

- (BOOL)findNearestVertexToPoint:(NSPoint)p maxDistance:(double)maxDist row:(NSInteger*)outRow col:(NSInteger*)outCol {
    if (!_mesh) return NO;
    double best = maxDist * maxDist;
    BOOL found = NO;
    for (int r = 0; r < _mesh->rows; ++r) {
        for (int c = 0; c < _mesh->cols; ++c) {
            Vec2 v = _mesh->at(r, c).P;
            double dx = v.x - p.x, dy = v.y - p.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best) { best = d2; *outRow = r; *outCol = c; found = YES; }
        }
    }
    return found;
}

- (NSArray<NSValue*>*)meshEdgeBezierFromRow:(NSInteger)r0 col:(NSInteger)c0
                                       toRow:(NSInteger)r1 col:(NSInteger)c1 {
    const GradientMesh* m = [self meshForReading];
    if (!m) return @[];
    Vec2 P0 = m->at((int)r0, (int)c0).P;
    Vec2 P1 = m->at((int)r1, (int)c1).P;
    Vec2 T0, T1;
    if (r0 == r1 && c1 == c0 + 1) {
        // horizontal edge: position varies with col (u); the relevant
        // derivative is the FREE Pu at each end (Pu/Pv are optimized
        // unknowns now, not derived -- see GradientMesh.h/MeshOptimizer.h).
        T0 = m->at((int)r0, (int)c0).Pu;
        T1 = m->at((int)r1, (int)c1).Pu;
    } else if (c0 == c1 && r1 == r0 + 1) {
        // vertical edge: position varies with row (v); the free Pv at each end.
        T0 = m->at((int)r0, (int)c0).Pv;
        T1 = m->at((int)r1, (int)c1).Pv;
    } else {
        return @[]; // not a grid-adjacent pair
    }
    Vec2 B0 = P0;
    Vec2 B1 = P0 + T0 * (1.0 / 3.0);
    Vec2 B2 = P1 - T1 * (1.0 / 3.0);
    Vec2 B3 = P1;
    return @[
        [NSValue valueWithPoint:NSMakePoint(B0.x, B0.y)],
        [NSValue valueWithPoint:NSMakePoint(B1.x, B1.y)],
        [NSValue valueWithPoint:NSMakePoint(B2.x, B2.y)],
        [NSValue valueWithPoint:NSMakePoint(B3.x, B3.y)],
    ];
}

- (NSArray<NSArray<NSValue*>*>*)fittedBoundaryCurves {
    // Each of the 4 sides may now be MULTIPLE cubic segments (see
    // BezierSpline.h/fitBezierSpline) -- this flattens all of them, in
    // order (side 0's segments, then side 1's, ...), into one list of
    // [p0,p1,p2,p3] arrays. CanvasView's -drawBoundary doesn't need to
    // know or care how many came from which side: it just chains a
    // curveToPoint: for every array it's handed, in order, which already
    // produces the correct continuous closed path whether a side is one
    // segment or several (consecutive segments share their endpoint
    // exactly -- see fitBezierSpline's continuity guarantee).
    if (!_hasBoundary) return @[];
    NSMutableArray<NSArray<NSValue*>*>* out = [NSMutableArray array];
    for (int i = 0; i < 4; ++i) {
        for (const CubicBezier& b : _boundary[i].segments) {
            [out addObject:@[
                [NSValue valueWithPoint:NSMakePoint(b.p0.x, b.p0.y)],
                [NSValue valueWithPoint:NSMakePoint(b.p1.x, b.p1.y)],
                [NSValue valueWithPoint:NSMakePoint(b.p2.x, b.p2.y)],
                [NSValue valueWithPoint:NSMakePoint(b.p3.x, b.p3.y)],
            ]];
        }
    }
    return out;
}

- (NSPoint)meshVertexTangentUAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return NSZeroPoint;
    Vec2 t = _mesh->at((int)row, (int)col).Pu; // free unknown, not derived
    return NSMakePoint(t.x, t.y);
}

- (NSPoint)meshVertexTangentVAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return NSZeroPoint;
    Vec2 t = _mesh->at((int)row, (int)col).Pv; // free unknown, not derived
    return NSMakePoint(t.x, t.y);
}

#pragma mark - Vector lines

- (void)addVectorLineWithPoints:(NSArray<NSValue*>*)points {
    if (points.count < 2) return;
    VectorLine line;
    for (NSValue* v in points) {
        NSPoint p = [v pointValue];
        line.points.push_back(Vec2(p.x, p.y));
    }
    _vectorLines.push_back(std::move(line));
}

- (void)removeLastVectorLine {
    if (!_vectorLines.empty()) _vectorLines.pop_back();
}

- (void)clearVectorLines { _vectorLines.clear(); }

- (NSArray<NSArray<NSValue*>*>*)vectorLinesPoints {
    NSMutableArray* lines = [NSMutableArray arrayWithCapacity:_vectorLines.size()];
    for (auto& line : _vectorLines) {
        NSMutableArray* pts = [NSMutableArray arrayWithCapacity:line.points.size()];
        for (auto& p : line.points) [pts addObject:[NSValue valueWithPoint:NSMakePoint(p.x, p.y)]];
        [lines addObject:pts];
    }
    return lines;
}

#pragma mark - Scribble-based segmentation (Lazy Snapping)

- (void)addForegroundScribbleWithPoints:(NSArray<NSValue*>*)points {
    if (points.count < 1) return;
    VectorLine line;
    for (NSValue* v in points) {
        NSPoint p = [v pointValue];
        line.points.push_back(Vec2(p.x, p.y));
    }
    _fgScribbles.push_back(std::move(line));
}

- (void)addBackgroundScribbleWithPoints:(NSArray<NSValue*>*)points {
    if (points.count < 1) return;
    VectorLine line;
    for (NSValue* v in points) {
        NSPoint p = [v pointValue];
        line.points.push_back(Vec2(p.x, p.y));
    }
    _bgScribbles.push_back(std::move(line));
}

- (void)removeLastForegroundScribble {
    if (!_fgScribbles.empty()) _fgScribbles.pop_back();
}

- (void)removeLastBackgroundScribble {
    if (!_bgScribbles.empty()) _bgScribbles.pop_back();
}

- (void)clearScribbles {
    _fgScribbles.clear();
    _bgScribbles.clear();
}

- (NSArray<NSArray<NSValue*>*>*)foregroundScribblePoints {
    NSMutableArray* lines = [NSMutableArray arrayWithCapacity:_fgScribbles.size()];
    for (auto& line : _fgScribbles) {
        NSMutableArray* pts = [NSMutableArray arrayWithCapacity:line.points.size()];
        for (auto& p : line.points) [pts addObject:[NSValue valueWithPoint:NSMakePoint(p.x, p.y)]];
        [lines addObject:pts];
    }
    return lines;
}

- (NSArray<NSArray<NSValue*>*>*)backgroundScribblePoints {
    NSMutableArray* lines = [NSMutableArray arrayWithCapacity:_bgScribbles.size()];
    for (auto& line : _bgScribbles) {
        NSMutableArray* pts = [NSMutableArray arrayWithCapacity:line.points.size()];
        for (auto& p : line.points) [pts addObject:[NSValue valueWithPoint:NSMakePoint(p.x, p.y)]];
        [lines addObject:pts];
    }
    return lines;
}

- (BOOL)hasForegroundScribbles { return !_fgScribbles.empty(); }
- (BOOL)hasBackgroundScribbles { return !_bgScribbles.empty(); }

- (BOOL)segmentBoundaryFromScribblesWithError:(NSError**)error {
    if (!_hasImage) { if (error) *error = gmError(@"Load an image first."); return NO; }
    if (_fgScribbles.empty() || _bgScribbles.empty()) {
        if (error) *error = gmError(@"Scribble both foreground (green) and background (pink) before segmenting.");
        return NO;
    }

    // Rasterize each stroke's stored points into pixel-coordinate scribble
    // sets by stamping a filled disc around every point -- a single click
    // (or a widely-spaced drag) should still paint a usable patch of
    // scribble, not just a lone pixel. Radius matches CanvasView's 3px
    // drag-sampling density so a drawn stroke ends up as a solid band, not
    // a dotted line, once rasterized. Uses the raw sRGB _target, not the
    // CIELUV conversion -- segmentation is a pre-mesh-building step,
    // unaffected by useCIELUVColorSpace (see -workingTargetImage).
    const int kScribbleRadius = 4;
    const int w = _target.width, h = _target.height;
    gmcore::SegmentationScribbles scribbles;
    auto stampDisc = [&](double cx, double cy, std::vector<std::pair<int, int>>& out) {
        int ix = (int)std::lround(cx), iy = (int)std::lround(cy);
        for (int dy = -kScribbleRadius; dy <= kScribbleRadius; ++dy) {
            for (int dx = -kScribbleRadius; dx <= kScribbleRadius; ++dx) {
                if (dx * dx + dy * dy > kScribbleRadius * kScribbleRadius) continue;
                int x = ix + dx, y = iy + dy;
                if (x < 0 || y < 0 || x >= w || y >= h) continue;
                out.emplace_back(x, y);
            }
        }
    };
    for (auto& line : _fgScribbles)
        for (auto& p : line.points) stampDisc(p.x, p.y, scribbles.foreground);
    for (auto& line : _bgScribbles)
        for (auto& p : line.points) stampDisc(p.x, p.y, scribbles.background);

    std::vector<uint8_t> mask = gmcore::segmentForeground(_target, scribbles);
    std::vector<Vec2> contour = gmcore::traceOuterContour(mask, w, h);
    if (contour.size() < 3) {
        if (error) *error = gmError(@"Segmentation didn't find a usable region -- try adding more scribbles.");
        return NO;
    }
    std::vector<Vec2> simplified = gmcore::simplifyClosedPolygon(contour, /*epsilonPixels=*/2.0);

    NSMutableArray<NSValue*>* poly = [NSMutableArray arrayWithCapacity:simplified.size()];
    for (auto& p : simplified) [poly addObject:[NSValue valueWithPoint:NSMakePoint(p.x, p.y)]];
    // Reuses the EXACT same entry point manual click-tracing already feeds
    // -- corner-picking and boundary Bezier fitting are unchanged from here.
    [self setBoundaryPolygonPoints:poly];
    return YES;
}

#pragma mark - Optimize

- (void)optimizeWithPyramidLevels:(NSInteger)levels
                          progress:(void (^)(double, NSInteger, NSInteger, NSInteger, NSInteger))progress
                        completion:(void (^)(void))completion {
    if (!_mesh || _isOptimizing) { if (completion) completion(); return; }
    [self stopMeshAnimation]; // an Optimize run mutates _mesh directly -- any running animation's base snapshot would go stale
    _isOptimizing = YES;
    _previewMesh.reset(); // stale snapshot from a previous run, if any -- see hasLivePreviewMesh

    // MeshOptimizer mutates the mesh's std::vector storage; keep the mesh
    // pointer stable and avoid touching it from the main thread while this
    // runs (CanvasView checks isOptimizing -- or, when livePreviewDuringOptimize
    // is on, hasLivePreviewMesh -- before reading mesh geometry, never _mesh
    // itself mid-run).
    GradientMesh* meshPtr = _mesh.get();
    // Read once, here, before the background dispatch -- same reasoning as
    // opts below (a toggle mid-run must not half-apply).
    BOOL livePreview = self.livePreviewDuringOptimize;
    // Must match the color space _mesh's C/Cu/Cv/Cuv were actually BUILT in
    // (_meshColorSpaceIsCIELUV, snapshotted at -buildInitialMeshRows:cols:
    // time), NOT the live useCIELUVColorSpace property -- see that ivar's
    // comment. Using the wrong one here would feed the optimizer's data
    // term a target in a different space than the mesh colors it's
    // comparing against, silently corrupting every run.
    Image targetCopy = _meshColorSpaceIsCIELUV ? _targetLUV : _target; // Image is a small value type wrapping a vector; a private copy for thread safety.
    std::vector<VectorLine> linesCopy = _vectorLines;
    OptimizerOptions opts;
    // See DocumentModel.h's comment on these two properties: "joint wins" if
    // both are set, mirroring MeshOptimizer.cpp's own jointSolvedByCeres
    // gating and the CLI's --use-ceres/--use-ceres-joint precedence.
    opts.useCeresGeometry = self.useCeresGeometry;
    opts.useCeresJoint = self.useCeresJoint;
    // YES -> 0 ("auto", every core); NO -> 1 (pinned single-threaded) --
    // see ceresMultithreaded's comment in DocumentModel.h.
    opts.ceresNumThreads = self.ceresMultithreaded ? 0 : 1;
    // 0/unset -> 1: see DocumentModel.h's comment on this property.
    opts.pyramidRestarts = (int)std::max((NSInteger)1, self.pyramidRestarts);
    // Geometry/colour energy weights -- see DocumentModel.h's comment on
    // these eight properties. Read fresh here (not cached), so a value
    // edited in the UI since the last run takes effect on THIS "Optimize"
    // click.
    opts.smoothWeightGeom = self.smoothWeightGeom;
    opts.smoothWeightColor = self.smoothWeightColor;
    opts.colorDerivRidge = self.colorDerivRidge;
    opts.boundaryWeight = self.boundaryWeight;
    opts.geomTangentPriorWeight = self.geomTangentPriorWeight;
    opts.vectorLineWeight = self.vectorLineWeight;
    opts.geomDataWeight = self.geomDataWeight;
    opts.smoothGeomEdgeGain = self.smoothGeomEdgeGain;

    // Snapshot the exact opts this run uses and reset the per-run progress
    // log/timer, for -exportDebugDataToURL:error: -- must happen here, on
    // the main thread, before the background dispatch below, not inside it,
    // so a debug export triggered mid-run (or right after) always reflects
    // THIS run's actual settings, not a stale opts struct left over from an
    // earlier call with different solver toggles.
    _lastOptsUsed = opts;
    _lastRunHistory.clear();
    _hasRunOptimize = YES;
    NSDate* runStart = [NSDate date];
    // Full-res (pyramid level 0) pixel size, captured once here rather than
    // capturing the whole (potentially large) targetCopy Image into the C++
    // progress lambda below -- see its use just below scalePositions'
    // comment for why the live-preview snapshot needs this.
    int fullResW = targetCopy.width, fullResH = targetCopy.height;
    // See _meshHasHadFullPyramidPass's declaration: only the FIRST
    // "Optimize" click on a given mesh (or the first one after it's been
    // rebuilt/dropped) does the full coarse-to-fine sweep; every later
    // click on the SAME already-fitted mesh refines directly at full
    // resolution instead, skipping the coarse-level detour that measurably
    // degrades full-res fit before climbing back (see
    // spike/ceres_joint_drift_probe.cpp's per-pyramid-level trace and the
    // README -- confirmed worse for useCeresJoint's fully-coupled step than
    // for the other two solver paths, but present in all three). Read here
    // (main thread), same reasoning as opts/livePreview above.
    BOOL needsFullPyramidPass = !_meshHasHadFullPyramidPass;

    dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
        auto progressCb = [progress, self, meshPtr, livePreview, fullResW, fullResH](const OptimizerProgress& p) {
            // Snapshot HERE, still on the background thread, still
            // inside the synchronous callback optimizeAtCurrentResolution
            // invokes between one outer iteration's writes finishing and
            // the next one's starting (see MeshOptimizer.cpp) -- so this
            // copy can never race a concurrent write to *meshPtr. Cheap
            // (a std::vector<MeshVertex> copy, no allocation-heavy
            // fields) but still only paid when the toggle is on.
            std::shared_ptr<GradientMesh> snap;
            if (livePreview) {
                snap = std::make_shared<GradientMesh>(*meshPtr);
                // *meshPtr lives in pyramid level p.pyramidLevel's OWN
                // downsampled coordinate space for the whole duration of
                // that level (see OptimizerProgress::levelWidth/Height's
                // comment) -- only the FINEST level (0) happens to
                // already be full-res. CanvasView always draws against
                // full-res image pixel coordinates, so without this the
                // live mesh grid would render shrunk into a corner of
                // the canvas at every level except the last one. Scaling
                // up front here (background thread, on the already-
                // independent copy) means CanvasView/DocumentModel's
                // read accessors don't need to know or care which
                // pyramid level produced the snapshot they're reading.
                if (p.levelWidth > 0 && p.levelHeight > 0) {
                    snap->scalePositions(double(fullResW) / p.levelWidth, double(fullResH) / p.levelHeight);
                }
            }
            dispatch_async(dispatch_get_main_queue(), ^{
                // Recorded unconditionally, even if the caller passed a
                // nil UI progress block -- this is the history
                // -exportDebugDataToURL:error: dumps, independent of
                // whether anything was listening for live UI updates.
                self->_lastRunHistory.push_back(p);
                if (snap) self->_previewMesh = std::make_unique<GradientMesh>(*snap);
                if (progress) progress(p.rmse, p.pyramidLevel, p.totalPyramidLevels, p.outerIteration, p.totalOuterIterations);
            });
        };
        if (needsFullPyramidPass) {
            gmcore::MeshOptimizer::optimizeCoarseToFine(*meshPtr, targetCopy, linesCopy, (int)levels, opts, progressCb);
        } else {
            // Full resolution IS pyramid level 0 -- same function
            // optimizeCoarseToFine itself calls for its finest level, just
            // invoked directly with no coarser levels before it. `mesh` is
            // already sized for targetCopy's full resolution (a full
            // pyramid pass always ends at level 0, and this branch only
            // runs once one has happened), so no scalePositions is needed
            // here the way optimizeCoarseToFine needs it between levels.
            gmcore::MeshOptimizer::optimizeAtCurrentResolution(*meshPtr, targetCopy, linesCopy, opts, progressCb,
                                                                /*level=*/0, /*totalLevels=*/1);
        }
        double finalRmse = meshPtr->reconstructionRMSE(targetCopy, 6);
        double finalMae = meshPtr->reconstructionMAE(targetCopy, 6);
        dispatch_async(dispatch_get_main_queue(), ^{
            self->_isOptimizing = NO;
            self->_previewMesh.reset(); // run over -- CanvasView goes back to reading the live (now-settled) _mesh
            self->_meshHasHadFullPyramidPass = YES; // true whether THIS run did the full sweep or just refined at full-res
            self->_lastRMSE = finalRmse;
            self->_lastMAE = finalMae;
            self->_lastRunWallClockSeconds = -[runStart timeIntervalSinceNow];
            // Unconditional console log of this run's timing -- unlike
            // -exportDebugDataToURL:error:'s JSON (which already carries
            // this same number, see that method's "lastRunWallClockSeconds"
            // field) this needs neither autoExportDebugData nor a loaded
            // image to have somewhere to write to, so it's the one place
            // every run's duration is always recorded, visible in Xcode's
            // debug console (Cmd+Shift+Y) without any extra setup.
            NSLog(@"[GradientMeshStudio] Optimize finished in %.2fs (mesh %ldx%ld, RMSE=%.4f, MAE=%.4f)",
                  self->_lastRunWallClockSeconds, (long)self->_mesh->rows, (long)self->_mesh->cols, finalRmse, finalMae);
            // Must run AFTER the state above is updated (it dumps
            // _lastRMSE/_lastRunHistory/_lastRunWallClockSeconds) but
            // BEFORE completion(), so a completion handler that checks
            // -lastDebugExportPath (see MainWindowController's -optimize:)
            // sees this run's result, not a stale one from before it.
            [self autoExportDebugDataIfEnabled];
            if (completion) completion();
        });
    });
}

#pragma mark - Mesh animation ("Animate Mesh")

// See DocumentModel.h's isAnimatingMesh/meshAnimationMaxAmplitude/
// meshAnimationTemperature doc comments for the full design (motion law,
// temperature-controlled amplitude sampling, boundary vertices pinned).
- (void)startMeshAnimationWithRedraw:(void (^)(void))redraw {
    if (!_mesh || _isOptimizing || _isAnimatingMesh) return;

    _animBaseMesh = *_mesh; // frozen snapshot -- _mesh itself is never touched while animating
    NSInteger n = (NSInteger)_animBaseMesh.vertices.size();
    _animDirections.assign((size_t)n, Vec2(0, 0));
    _animAmplitudes.assign((size_t)n, 0.0);

    // Read once, here, not live during the animation -- editing the
    // amplitude/temperature widgets mid-animation has no effect until the
    // NEXT -startMeshAnimationWithRedraw: call, same "read once at start"
    // convention as -optimizeWithPyramidLevels:...'s opts.
    double maxAmplitude = std::max(0.0, self.meshAnimationMaxAmplitude);
    double temperature = std::max(1e-6, self.meshAnimationTemperature); // guard against div-by-zero
    // Minimum required separation between two DIFFERENT edges' curves --
    // see gmMeshHasCurvedSelfIntersection's comment for why this is only
    // applied to cross-edge pairs, not a single edge's own self-loop
    // check. Clamped to >= 0 here the same way maxAmplitude is above;
    // meshAnimationMinClearanceDistance's own doc comment covers the
    // default and what it trades off.
    double minClearanceDistance = std::max(0.0, self.meshAnimationMinClearanceDistance);
    const double kTwoPi = 6.283185307179586;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> unit01(0.0, 1.0);
    for (NSInteger i = 0; i < n; ++i) {
        const MeshVertex& v = _animBaseMesh.vertices[(size_t)i];
        if (v.isBoundary) continue; // pinned at direction={0,0}/amplitude=0 -- see isAnimatingMesh's comment
        double angle = unit01(rng) * kTwoPi;
        // amp = maxAmplitude * uniform01^(1/temperature): temperature==1 is
        // a plain uniform draw; <1 ("cold") concentrates amplitudes near 0;
        // >1 ("hot") pushes them toward maxAmplitude. See
        // meshAnimationTemperature's doc comment in DocumentModel.h.
        double amp = maxAmplitude * std::pow(unit01(rng), 1.0 / temperature);
        _animDirections[(size_t)i] = Vec2(std::cos(angle), std::sin(angle));
        _animAmplitudes[(size_t)i] = amp;
    }

    // Average base-mesh edge length -- see gmMeshHasCurvedSelfIntersection's
    // comment: this is a broad-phase performance tuning value for the
    // precompute pass below, not a correctness threshold.
    {
        double sumLen = 0.0;
        int edgeCount = 0;
        for (int r = 0; r < _animBaseMesh.rows; ++r) {
            for (int c = 0; c < _animBaseMesh.cols; ++c) {
                int idx = r * _animBaseMesh.cols + c;
                if (c + 1 < _animBaseMesh.cols) {
                    sumLen += (_animBaseMesh.vertices[(size_t)idx + 1].P - _animBaseMesh.vertices[(size_t)idx].P).length();
                    edgeCount++;
                }
                if (r + 1 < _animBaseMesh.rows) {
                    sumLen += (_animBaseMesh.vertices[(size_t)idx + _animBaseMesh.cols].P - _animBaseMesh.vertices[(size_t)idx].P).length();
                    edgeCount++;
                }
            }
        }
        _animBucketCellSize = edgeCount > 0 ? std::max(1.0, sumLen / edgeCount) : 1.0;
    }

    // --- Precompute the whole loop, once ------------------------------------
    // The animation is exactly periodic (sin(t), period 2*pi) and, now that
    // every frame's UNOBSTRUCTED target is a pure function of that frame's
    // own phase alone (base_i + dir_i*amp_i*sin(phase) -- no dependency on
    // any earlier frame, since per-vertex "sliding" is gone), correcting a
    // self-intersection no longer needs to be a live, stateful, per-tick
    // process either: bisecting a frame's displacement back toward the
    // ALWAYS-clean base mesh (never toward the previous frame) is itself a
    // pure function of that one frame. So the entire loop can be computed
    // once, right here, and simply played back by index afterward -- see
    // -animationTick: below, which does no geometry work at all anymore.
    // This replaces the old per-tick per-vertex "slide" heuristic entirely:
    // that heuristic existed only to produce a reasonable-looking real-time
    // collision response without knowing the future, which stops being a
    // constraint once the whole loop is known before playback even starts.
    // See isAnimatingMesh's doc comment for the full picture, including the
    // one trade-off this introduces: a one-time startup delay here, running
    // synchronously on the calling thread (this method is only ever called
    // in response to the user pressing "Animate Mesh", not on any
    // per-frame path, and there's no way to verify a background-queue
    // version of this without a compiler -- see that comment for measured
    // worst-case timings).
    //
    // Curve-aware, not straight-wireframe: each candidate frame is checked
    // via gmMeshHasCurvedSelfIntersection against the ACTUAL rendered
    // Ferguson-patch-edge Bezier curves (see that function's comment) --
    // the straight-line check an earlier version of this feature used
    // could miss a real crossing between two curves whose straight P-to-P
    // segments never crossed, confirmed as a real, reproducible gap during
    // development.
    static const int kAnimSamplesPerEdge = 16; // curve tessellation density, see gmMeshHasCurvedSelfIntersection
    static const int kAnimBisectionIters = 10;
    NSInteger frameCount = (NSInteger)std::lround(kTwoPi * 60.0); // one full sin() period at ~60fps
    frameCount = std::max((NSInteger)1, frameCount);

    _animCachedFrames.assign((size_t)frameCount, std::vector<Vec2>((size_t)n));
    _animTraceFrames.clear();
    _animTraceFrames.reserve((size_t)frameCount);

    for (NSInteger k = 0; k < frameCount; ++k) {
        double phase = kTwoPi * (double)k / (double)frameCount;
        double s = std::sin(phase);
        std::vector<Vec2> candidate((size_t)n);
        for (NSInteger i = 0; i < n; ++i) {
            const MeshVertex& base = _animBaseMesh.vertices[(size_t)i];
            candidate[(size_t)i] = base.isBoundary
                ? base.P
                : base.P + _animDirections[(size_t)i] * (_animAmplitudes[(size_t)i] * s);
        }

        GMAnimTraceFrame trace;
        trace.t = phase;
        int xa1 = -1, xb1 = -1, xa2 = -1, xb2 = -1;
        if (gmMeshHasCurvedSelfIntersection(candidate, _animBaseMesh, kAnimSamplesPerEdge, _animBucketCellSize,
                                             minClearanceDistance, &xa1, &xb1, &xa2, &xb2)) {
            trace.curveSelfIntersectionDetected = true;
            trace.xa1 = xa1; trace.xb1 = xb1; trace.xa2 = xa2; trace.xb2 = xb2;
            trace.bisectionEngaged = true;
            // Bisect against the BASE mesh (always known clean by
            // construction), not the previous frame -- unlike the old
            // real-time design, there's no "previous frame" dependency to
            // preserve here, and anchoring every correction to the same
            // fixed base keeps every frame's correction independent of
            // frame order, which matters for a perfectly looping result:
            // the frameCount-1 -> 0 wraparound has to be just as clean a
            // transition as any other consecutive pair.
            double lo = 0.0, hi = 1.0, bestSafeLambda = 0.0;
            std::vector<Vec2> lerped((size_t)n);
            for (int iter = 0; iter < kAnimBisectionIters; ++iter) {
                double mid = (lo + hi) * 0.5;
                for (NSInteger i = 0; i < n; ++i) {
                    const MeshVertex& base = _animBaseMesh.vertices[(size_t)i];
                    lerped[(size_t)i] = base.P + (candidate[(size_t)i] - base.P) * mid;
                }
                if (gmMeshHasCurvedSelfIntersection(lerped, _animBaseMesh, kAnimSamplesPerEdge, _animBucketCellSize,
                                                     minClearanceDistance, nullptr, nullptr, nullptr, nullptr)) {
                    hi = mid;
                } else {
                    lo = mid;
                    bestSafeLambda = mid;
                }
            }
            trace.lambdaApplied = bestSafeLambda;
            for (NSInteger i = 0; i < n; ++i) {
                const MeshVertex& base = _animBaseMesh.vertices[(size_t)i];
                candidate[(size_t)i] = base.P + (candidate[(size_t)i] - base.P) * bestSafeLambda;
            }
        }
        trace.positions = candidate;
        _animTraceFrames.push_back(std::move(trace));
        _animCachedFrames[(size_t)k] = std::move(candidate);
    }

    // Export the just-finished precompute's trace right away -- there's no
    // later "run just stopped" moment to hang this off of anymore, since
    // the whole loop (and its trace) is already fully known at this point,
    // before a single frame has even been drawn. See
    // -autoExportMeshAnimationTraceIfEnabled and -stopMeshAnimation below.
    [self autoExportMeshAnimationTraceIfEnabled];

    _previewMesh = std::make_unique<GradientMesh>(_animBaseMesh);
    for (size_t i = 0; i < _animCachedFrames[0].size(); ++i) {
        _previewMesh->vertices[i].P = _animCachedFrames[0][i];
    }
    _animFrameIndex = 0;
    _isAnimatingMesh = YES;
    _animRedrawBlock = [redraw copy];

    // NSRunLoopCommonModes (NOT scheduledTimerWithTimeInterval:, which only
    // fires in NSDefaultRunLoopMode) so the animation keeps running while
    // the user drags/resizes the window or interacts with a control --
    // standard real-time-Cocoa-UI-timer pattern. All this timer does now is
    // advance an index into _animCachedFrames -- see -animationTick:.
    _animTimer = [NSTimer timerWithTimeInterval:1.0 / 60.0
                                          target:self
                                        selector:@selector(animationTick:)
                                        userInfo:nil
                                         repeats:YES];
    [[NSRunLoop currentRunLoop] addTimer:_animTimer forMode:NSRunLoopCommonModes];
}

// Timer-fired, ~60x/sec while animating. The ENTIRE loop was already
// computed once, up front, by -startMeshAnimationWithRedraw: (see that
// method's comment for why that's now possible and safe to do) -- so all
// this does is advance to the next cached frame and copy its positions
// into _previewMesh. No sine evaluation, no collision checking, no
// per-vertex loop over the mesh: every one of those already happened once
// during precompute, for every frame in the loop, not just this one.
- (void)animationTick:(NSTimer*)timer {
    if (!_isAnimatingMesh || !_previewMesh || _animCachedFrames.empty()) return;
    _animFrameIndex = (_animFrameIndex + 1) % (NSInteger)_animCachedFrames.size();
    const std::vector<Vec2>& frame = _animCachedFrames[(size_t)_animFrameIndex];
    for (size_t i = 0; i < frame.size(); ++i) {
        _previewMesh->vertices[i].P = frame[i];
    }
    if (_animRedrawBlock) _animRedrawBlock();
}

- (void)stopMeshAnimation {
    if (!_isAnimatingMesh) return;
    [_animTimer invalidate];
    _animTimer = nil;
    _isAnimatingMesh = NO;
    _previewMesh.reset(); // -meshForReading goes back to reading the live _mesh, which was never touched
    _animRedrawBlock = nil;
    _animCachedFrames.clear();
    _animFrameIndex = 0;
    // _animTraceFrames is deliberately left alone here -- unlike the old
    // real-time design, the trace is already complete and already
    // auto-exported (see -startMeshAnimationWithRedraw:) the moment
    // precompute finishes, well before the user ever presses stop, so
    // -exportMeshAnimationTraceToURL:error: can still read the most recent
    // run's trace after the animation has been stopped, not only while
    // it's running. The next -startMeshAnimationWithRedraw: call replaces it.
}

#pragma mark - Debug data auto-export

// Sibling "DebugOut" folder next to the currently loaded image, creating it
// if needed. Returns nil if there's no loaded image to be a sibling of, or
// if the folder couldn't be created (e.g. the image's folder is read-only).
- (nullable NSURL*)debugOutDirectoryURL {
    if (!_imageURL) return nil;
    NSURL* dir = [[_imageURL URLByDeletingLastPathComponent] URLByAppendingPathComponent:@"DebugOut" isDirectory:YES];
    NSError* mkdirErr = nil;
    if (![[NSFileManager defaultManager] createDirectoryAtURL:dir withIntermediateDirectories:YES
                                                     attributes:nil error:&mkdirErr]) {
        NSLog(@"[GMCORE] autoExportDebugData: could not create DebugOut folder at %@: %@", dir, mkdirErr);
        return nil;
    }
    return dir;
}

// "gm_debug_<solver>[_cieluv]_<rows>x<cols>_<timestamp>.json" -- same
// <solver>_<rows>x<cols> convention as the old manual export's default
// filename, plus a millisecond-resolution local timestamp (fixed
// yyyyMMdd-HHmmss-SSS format, en_US_POSIX locale so it can't come out
// looking different on a machine set to another locale/calendar) so
// repeated runs pile up in DebugOut side by side instead of each one
// silently overwriting the last -- the whole point of turning this into an
// always-on checkbox is comparing a SEQUENCE of runs after the fact. The
// "_cieluv" tag (see the JSON body's own "colorSpace" field, the actual
// source of truth) is deliberately visible in the filename too: a CIELUV
// run's lastRMSE is in different units than an sRGB run's (see
// useCIELUVColorSpace's doc comment) and this project has already once
// mixed up two RMSE numbers that looked comparable but weren't (see
// README) -- worth avoiding a repeat by making the units visible at a
// glance, not just inside the file.
- (NSString*)debugExportFilename {
    NSString* solverTag = self.useCeresJoint ? @"ceres_joint" : self.useCeresGeometry ? @"ceres_geom" : @"hand_rolled";
    NSString* colorSpaceTag = _meshColorSpaceIsCIELUV ? @"_cieluv" : @"";
    static NSDateFormatter* fmt;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        fmt = [[NSDateFormatter alloc] init];
        fmt.locale = [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"];
        fmt.dateFormat = @"yyyyMMdd-HHmmss-SSS";
    });
    NSString* stamp = [fmt stringFromDate:[NSDate date]];
    return [NSString stringWithFormat:@"gm_debug_%@%@_%ldx%ld_%@.json", solverTag, colorSpaceTag,
            (long)self.meshRows, (long)self.meshCols, stamp];
}

// See DocumentModel.h's comment on autoExportDebugData. Called at the end
// of every -optimizeWithPyramidLevels:progress:completion: run (regardless
// of outcome -- there's always a mesh by that point, since the method
// bails out early if there wasn't one to begin with).
- (void)autoExportDebugDataIfEnabled {
    if (!self.autoExportDebugData || !_mesh) return;
    NSURL* dir = [self debugOutDirectoryURL];
    if (!dir) { self.lastDebugExportPath = nil; return; }
    NSURL* fileURL = [dir URLByAppendingPathComponent:[self debugExportFilename]];
    NSError* error = nil;
    if ([self exportDebugDataToURL:fileURL error:&error]) {
        self.lastDebugExportPath = fileURL.path;
    } else {
        NSLog(@"[GMCORE] autoExportDebugData: export failed: %@", error);
        self.lastDebugExportPath = nil;
    }
}

// "gm_meshanim_<rows>x<cols>_<timestamp>.json" -- same timestamp
// convention as -debugExportFilename, deliberately a different prefix
// ("meshanim" not "debug") so the two kinds of export are easy to tell
// apart at a glance in a DebugOut folder that accumulates both.
- (NSString*)meshAnimationTraceFilename {
    static NSDateFormatter* fmt;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        fmt = [[NSDateFormatter alloc] init];
        fmt.locale = [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"];
        fmt.dateFormat = @"yyyyMMdd-HHmmss-SSS";
    });
    NSString* stamp = [fmt stringFromDate:[NSDate date]];
    return [NSString stringWithFormat:@"gm_meshanim_%ldx%ld_%@.json", (long)_animBaseMesh.rows, (long)_animBaseMesh.cols, stamp];
}

// See DocumentModel.h's comment on this method. Reads _animTraceFrames as
// filled in by the one-time precompute in -startMeshAnimationWithRedraw:
// -- by the time any animation is actually running (or has been stopped
// since), that precompute has already finished and this data is already
// final; unlike the old real-time trace, nothing here changes while the
// animation plays back, and -stopMeshAnimation no longer clears it either
// (see that method), so this can still be called after the user has
// pressed stop.
- (BOOL)exportMeshAnimationTraceToURL:(NSURL*)url error:(NSError**)error {
    if (_animTraceFrames.empty()) {
        if (error) *error = gmError(@"No mesh animation trace recorded yet -- run \"Animate Mesh\" first.");
        return NO;
    }

    NSMutableDictionary* root = [NSMutableDictionary dictionary];

    if (@available(macOS 10.12, *)) {
        NSISO8601DateFormatter* iso = [[NSISO8601DateFormatter alloc] init];
        root[@"exportedAt"] = [iso stringFromDate:[NSDate date]];
    }

    NSString* repoRoot = gmFindRepoRootFromSourceFile();
    NSString* commit = gmRunGit(repoRoot, @[@"rev-parse", @"HEAD"]) ?: @"unknown";
    NSString* describe = gmRunGit(repoRoot, @[@"describe", @"--always", @"--dirty", @"--long"]) ?: @"unknown";
    NSString* porcelain = gmRunGit(repoRoot, @[@"status", @"--porcelain"]);
    root[@"git"] = @{
        @"repoRootFound": @(repoRoot != nil),
        @"commit": commit,
        @"describe": describe,
        @"dirty": @(porcelain != nil && porcelain.length > 0),
    };

    root[@"mesh"] = @{ @"rows": @(_animBaseMesh.rows), @"cols": @(_animBaseMesh.cols) };

    long curveSelfIntersectionCount = 0;
    long bisectionEngagedCount = 0;
    double worstLambdaApplied = 1.0;
    NSNumber* firstEventFrame = nil;
    for (size_t i = 0; i < _animTraceFrames.size(); ++i) {
        const GMAnimTraceFrame& f = _animTraceFrames[i];
        if (f.curveSelfIntersectionDetected) {
            curveSelfIntersectionCount++;
            if (!firstEventFrame) firstEventFrame = @(i);
        }
        if (f.bisectionEngaged) bisectionEngagedCount++;
        worstLambdaApplied = std::min(worstLambdaApplied, f.lambdaApplied);
    }
    root[@"animation"] = @{
        @"maxAmplitude": @(self.meshAnimationMaxAmplitude),
        @"temperature": @(self.meshAnimationTemperature),
        @"bucketCellSizeUsed": @(_animBucketCellSize),
        // The whole animation is now one fixed-length precomputed loop --
        // frameCount/60fps is its playback duration, NOT a wall-clock
        // measurement of anything that actually ran (unlike the old
        // real-time trace's durationSeconds).
        @"frameCount": @(_animTraceFrames.size()),
        @"loopDurationSecondsAt60fps": @(_animTraceFrames.size() / 60.0),
        @"curveSelfIntersectionCount": @(curveSelfIntersectionCount),
        @"bisectionEngagedCount": @(bisectionEngagedCount),
        @"worstLambdaApplied": @(worstLambdaApplied),
        @"firstEventFrame": firstEventFrame ?: [NSNull null],
    };

    NSMutableArray<NSDictionary*>* framesJSON = [NSMutableArray arrayWithCapacity:_animTraceFrames.size()];
    for (size_t i = 0; i < _animTraceFrames.size(); ++i) {
        const GMAnimTraceFrame& f = _animTraceFrames[i];
        NSMutableDictionary* fj = [NSMutableDictionary dictionary];
        fj[@"frame"] = @(i);
        fj[@"t"] = @(f.t);
        fj[@"curveSelfIntersectionDetected"] = @(f.curveSelfIntersectionDetected);
        if (f.curveSelfIntersectionDetected) {
            fj[@"intersectingEdgeVertices"] = @[@(f.xa1), @(f.xb1), @(f.xa2), @(f.xb2)];
        }
        fj[@"bisectionEngaged"] = @(f.bisectionEngaged);
        fj[@"lambdaApplied"] = @(f.lambdaApplied);
        // Always present now -- every frame in the (bounded, few-hundred-
        // entry) precomputed loop is recorded in full, no thinning policy
        // needed anymore (see GMAnimTraceFrame's comment).
        NSMutableArray<NSArray*>* posJSON = [NSMutableArray arrayWithCapacity:f.positions.size()];
        for (const Vec2& p : f.positions) [posJSON addObject:@[@(p.x), @(p.y)]];
        fj[@"positions"] = posJSON;
        [framesJSON addObject:fj];
    }
    root[@"frames"] = framesJSON;

    NSError* jsonErr = nil;
    NSData* data = [NSJSONSerialization dataWithJSONObject:root
                                                     options:(NSJSONWritingPrettyPrinted | NSJSONWritingSortedKeys)
                                                       error:&jsonErr];
    if (!data) { if (error) *error = jsonErr ?: gmError(@"Could not serialize mesh animation trace."); return NO; }
    BOOL ok = [data writeToURL:url options:NSDataWritingAtomic error:&jsonErr];
    if (!ok && error) *error = jsonErr ?: gmError(@"Could not write mesh animation trace file.");
    return ok;
}

// Mirrors -autoExportDebugDataIfEnabled above, deliberately reusing the
// SAME autoExportDebugData toggle rather than adding a new checkbox: by
// the time a user has "Animate Mesh" running AND debug auto-export
// enabled, they are already in the same "collecting data to diagnose
// something" mode this trace is for, and piggybacking avoids needing new
// UI wiring for what is, for now, a developer-facing diagnostic. Called
// from -startMeshAnimationWithRedraw:, right after the one-time precompute
// finishes and _animTraceFrames is filled in for the whole loop -- unlike
// the old real-time trace, there's no later "run just stopped" moment to
// hang this off of, since the entire loop (and its trace) already exists
// before a single frame has been drawn or -stopMeshAnimation has ever run.
- (void)autoExportMeshAnimationTraceIfEnabled {
    if (!self.autoExportDebugData || _animTraceFrames.empty()) return;
    NSURL* dir = [self debugOutDirectoryURL];
    if (!dir) { self.lastMeshAnimationTracePath = nil; return; }
    NSURL* fileURL = [dir URLByAppendingPathComponent:[self meshAnimationTraceFilename]];
    NSError* error = nil;
    if ([self exportMeshAnimationTraceToURL:fileURL error:&error]) {
        self.lastMeshAnimationTracePath = fileURL.path;
    } else {
        NSLog(@"[GMCORE] autoExportMeshAnimationTrace: export failed: %@", error);
        self.lastMeshAnimationTracePath = nil;
    }
}

#pragma mark - Output

- (NSImage*)renderReconstructionPreview {
    // See -meshForReading's comment: reads the animated preview mesh while
    // isAnimatingMesh (so "Show reconstruction" actually animates along
    // with the mesh grid, not just a frozen pre-animation render), the
    // live-optimize snapshot while hasLivePreviewMesh, or the settled
    // _mesh otherwise -- the same accessor every other mesh-reading method
    // in this class already goes through.
    const GradientMesh* m = [self meshForReading];
    if (!m || !_hasImage) return nil;
    Image rendered = m->render((int)_target.width, (int)_target.height, 8);
    // GradientMesh::render() fully patch-interpolates (Sec 3's Ferguson patches),
    // producing per-pixel POINT VALUES throughout -- so, unlike Cu/Cv/Cuv,
    // the raster it returns is safe/correct to convert wholesale. Must run
    // before the clamped01() loop below (see that call's own note on why).
    if (_meshColorSpaceIsCIELUV) rendered = gmcore::imageCIELUVToSRGB(rendered);
    int w = rendered.width, h = rendered.height;
    std::vector<uint8_t> buffer(size_t(w) * h * 4, 255);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            Color c = rendered.at(x, y).clamped01();
            uint8_t* px = &buffer[(size_t(y) * w + x) * 4];
            px[0] = (uint8_t)std::lround(c.r * 255.0);
            px[1] = (uint8_t)std::lround(c.g * 255.0);
            px[2] = (uint8_t)std::lround(c.b * 255.0);
            px[3] = 255;
        }
    }
    // Explicit sRGB, NOT CGColorSpaceCreateDeviceRGB() -- fixes a reported
    // bug: the reconstruction preview looked visibly "washed out"/less
    // saturated on screen in CanvasView, while exporting it to a PNG via
    // -exportPNGToURL: (which rasterizes this SAME NSImage/CGImage) and
    // reopening that file showed the correct, fully-saturated colors. Both
    // paths draw the exact same numeric RGB bytes -- CGColorSpaceCreateDeviceRGB()
    // produces an untagged/"generic device" color space, which is legacy
    // CoreGraphics terminology, not a request to bypass color management.
    // On-screen, AppKit/ColorSync has to pick SOME interpretation for that
    // ambiguous tag when compositing into the window's (wide-gamut,
    // typically Display P3) backing store, and can fall back to an old
    // "Generic RGB" profile with a flatter, less saturated response than
    // sRGB; a PNG viewer opening the exported file, on the other hand,
    // treats an untagged/generic image as sRGB by convention (the modern
    // default assumption), which is why the file looked correct while the
    // live view did not. Explicitly naming the space as sRGB removes the
    // ambiguity at the source instead of relying on inconsistent fallback
    // behavior in two different code paths.
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef ctx = CGBitmapContextCreate(buffer.data(), w, h, 8, w * 4, cs,
                                              kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    CGColorSpaceRelease(cs);
    CGImageRef cgImage = CGBitmapContextCreateImage(ctx);
    CGContextRelease(ctx);
    NSImage* img = [[NSImage alloc] initWithCGImage:cgImage size:NSMakeSize(w, h)];
    CGImageRelease(cgImage);
    return img;
}

- (nullable GMGPUMeshBuffers*)gpuMeshBuffersWithSamplesPerPatchEdge:(NSInteger)samplesPerPatchEdge {
    // See -meshForReading's comment -- same "animated preview while
    // isAnimatingMesh" routing -renderReconstructionPreview now uses, so
    // the GPU preview animates along with the mesh grid too.
    const GradientMesh* m = [self meshForReading];
    if (!m) return nil;
    gmcore::MeshRenderBuffers buf = gmcore::MeshRenderBuffers::build(*m, (int)samplesPerPatchEdge);

    // Mesh geometry (P) bounding box -- see GMGPUMeshBuffers.minX/etc's
    // doc comment; lets GLReconstructionView fit+center the mesh the same
    // way CanvasView's -imageDisplayRect letterboxes the CPU preview.
    double minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
    for (int row = 0; row < m->rows; ++row) {
        for (int col = 0; col < m->cols; ++col) {
            const Vec2& p = m->at(row, col).P;
            minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
        }
    }

    GMGPUMeshBuffers* out = [GMGPUMeshBuffers new];
    out.vertexData = [NSData dataWithBytes:buf.vertexData.data() length:buf.vertexData.size() * sizeof(float)];
    out.uvTemplate = [NSData dataWithBytes:buf.uvTemplate.data() length:buf.uvTemplate.size() * sizeof(float)];
    out.indexData = [NSData dataWithBytes:buf.indices.data() length:buf.indices.size() * sizeof(uint32_t)];
    out.cols = buf.cols;
    out.patchRows = buf.patchRows;
    out.patchCols = buf.patchCols;
    out.cieluv = _meshColorSpaceIsCIELUV;
    out.minX = minX; out.minY = minY; out.maxX = maxX; out.maxY = maxY;
    return out;
}

- (BOOL)exportPNGToURL:(NSURL*)url error:(NSError**)error {
    NSImage* preview = [self renderReconstructionPreview];
    if (!preview) { if (error) *error = gmError(@"No mesh to render yet."); return NO; }
    CGImageRef cgImage = [preview CGImageForProposedRect:NULL context:nil hints:nil];
    if (!cgImage) { if (error) *error = gmError(@"Could not rasterize preview."); return NO; }
    CGImageDestinationRef dest = CGImageDestinationCreateWithURL((__bridge CFURLRef)url, CFSTR("public.png"), 1, NULL);
    if (!dest) { if (error) *error = gmError(@"Could not create PNG file."); return NO; }
    CGImageDestinationAddImage(dest, cgImage, NULL);
    BOOL ok = CGImageDestinationFinalize(dest);
    CFRelease(dest);
    if (!ok && error) *error = gmError(@"Could not write PNG file.");
    return ok;
}

- (BOOL)exportSVGToURL:(NSURL*)url error:(NSError**)error {
    if (!_mesh) { if (error) *error = gmError(@"No mesh to export yet."); return NO; }
    std::string svg = gmcore::exportGradientMeshSVG(*_mesh, _target.width, _target.height, _meshColorSpaceIsCIELUV);
    NSString* str = [NSString stringWithUTF8String:svg.c_str()];
    NSError* writeErr = nil;
    BOOL ok = [str writeToURL:url atomically:YES encoding:NSUTF8StringEncoding error:&writeErr];
    if (!ok && error) *error = writeErr ?: gmError(@"Could not write SVG file.");
    return ok;
}

- (BOOL)exportDebugDataToURL:(NSURL*)url error:(NSError**)error {
    if (!_mesh) { if (error) *error = gmError(@"No mesh to export debug data for yet."); return NO; }

    NSMutableDictionary* root = [NSMutableDictionary dictionary];

    if (@available(macOS 10.12, *)) {
        NSISO8601DateFormatter* iso = [[NSISO8601DateFormatter alloc] init];
        root[@"exportedAt"] = [iso stringFromDate:[NSDate date]];
    }

    NSString* repoRoot = gmFindRepoRootFromSourceFile();
    NSString* commit = gmRunGit(repoRoot, @[@"rev-parse", @"HEAD"]) ?: @"unknown";
    NSString* describe = gmRunGit(repoRoot, @[@"describe", @"--always", @"--dirty", @"--long"]) ?: @"unknown";
    NSString* porcelain = gmRunGit(repoRoot, @[@"status", @"--porcelain"]);
    root[@"git"] = @{
        @"repoRootFound": @(repoRoot != nil),
        @"commit": commit,
        @"describe": describe,
        @"dirty": @(porcelain != nil && porcelain.length > 0),
    };

    // Which solver was REQUESTED (the UI toggle) vs. which one actually ran
    // -- these can differ silently: useCeresGeometry/useCeresJoint are
    // no-ops (with only a one-time stderr warning, easy to miss) in a
    // build without Ceres found. This is exactly the ambiguity that made
    // an earlier SVG-only analysis unreliable -- see gmcore::builtWithCeres().
    BOOL builtWithCeres = gmcore::builtWithCeres();
    NSString* requestedSolver = self.useCeresJoint ? @"ceresJoint"
                               : self.useCeresGeometry ? @"ceresGeometry" : @"handRolled";
    NSString* effectiveSolver = builtWithCeres ? requestedSolver : @"handRolled";
    root[@"solver"] = @{
        @"builtWithCeres": @(builtWithCeres),
        @"requested": requestedSolver,
        @"effective": effectiveSolver,
    };

    // Which colour space _mesh's C/Cu/Cv/Cuv (below) and lastRMSE are
    // actually in -- see useCIELUVColorSpace's doc comment in
    // DocumentModel.h and ColorSpace.h. Not present in JSONs exported
    // before this field existed; treat its absence there as "sRGB" (the
    // only space that existed at the time). lastRMSE from a "CIELUV" export
    // is measured in CIELUV units (L* roughly 0..100) and is NOT directly
    // comparable by raw number to an "sRGB" export's lastRMSE.
    root[@"colorSpace"] = _meshColorSpaceIsCIELUV ? @"CIELUV" : @"sRGB";

    root[@"image"] = @{ @"width": @(_target.width), @"height": @(_target.height) };
    // Same metric, same sample density (6) the UI's status label and the
    // filename-embedded RMSE convention already use -- see
    // -optimizeWithPyramidLevels:progress:completion:'s finalRmse.
    root[@"lastRMSE"] = @(_lastRMSE);
    // Same sampling/units convention as lastRMSE, just mean(|diff|) instead
    // of sqrt(mean(diff^2)) -- see gmcore::GradientMesh::reconstructionMAE's
    // comment for why it's worth reporting alongside RMSE. Absent in JSONs
    // exported before this field existed.
    root[@"lastMAE"] = @(_lastMAE);
    root[@"pyramidRestarts"] = @(self.pyramidRestarts);
    root[@"hasRunOptimize"] = @(_hasRunOptimize);
    root[@"lastRunWallClockSeconds"] = @(_lastRunWallClockSeconds);

    NSMutableArray<NSDictionary*>* history = [NSMutableArray arrayWithCapacity:_lastRunHistory.size()];
    for (const auto& p : _lastRunHistory) {
        [history addObject:@{
            @"pyramidLevel": @(p.pyramidLevel),
            @"totalPyramidLevels": @(p.totalPyramidLevels),
            @"outerIteration": @(p.outerIteration),
            @"totalOuterIterations": @(p.totalOuterIterations),
            @"rmse": @(p.rmse),
        }];
    }
    root[@"lastRunHistory"] = history;

    const OptimizerOptions& o = _lastOptsUsed;
    root[@"optimizerOptions"] = @{
        @"samplesPerPatchEdge": @(o.samplesPerPatchEdge),
        @"smoothWeightGeom": @(o.smoothWeightGeom),
        @"smoothGeomEdgeGain": @(o.smoothGeomEdgeGain),
        @"smoothGeomMinFactor": @(o.smoothGeomMinFactor),
        @"geomDataWeight": @(o.geomDataWeight),
        @"smoothWeightColor": @(o.smoothWeightColor),
        @"colorDerivRidge": @(o.colorDerivRidge),
        @"boundaryWeight": @(o.boundaryWeight),
        @"vectorLineWeight": @(o.vectorLineWeight),
        @"vectorLineInfluenceRadius": @(o.vectorLineInfluenceRadius),
        @"geomTangentPriorWeight": @(o.geomTangentPriorWeight),
        @"outerIterationsPerLevel": @(o.outerIterationsPerLevel),
        @"outerConvergenceRelTol": @(o.outerConvergenceRelTol),
        @"outerConvergencePatience": @(o.outerConvergencePatience),
        @"pyramidRestarts": @(o.pyramidRestarts),
        @"geomGaussNewtonItersPerOuter": @(o.geomGaussNewtonItersPerOuter),
        @"cgMaxIterations": @(o.cgMaxIterations),
        @"cgRelTolerance": @(o.cgRelTolerance),
        @"geomDampingInitial": @(o.geomDampingInitial),
        @"useCeresGeometry": @(o.useCeresGeometry),
        @"useCeresJoint": @(o.useCeresJoint),
        @"ceresNumThreads": @(o.ceresNumThreads),
        // Added late -- omitted from the first round of exported debug
        // JSONs (they predate this OptimizerOptions field), so its
        // absence there means "this build's default (0.3), not recorded",
        // not "damping was off".
        @"jointGeomStepDampingWeight": @(o.jointGeomStepDampingWeight),
    };

    // Full mesh state -- the actual point of this export. Unlike the SVG
    // exporter (4 corner colors per patch, no Cu/Cv/Cuv, no explicit
    // geometry tangents) this carries every field GradientMesh::evalPos/
    // evalColor actually read, so an offline analysis can reproduce
    // reconstructionRMSE (and the render) exactly rather than approximate
    // it from Bezier control points.
    NSMutableArray<NSDictionary*>* vertsJSON = [NSMutableArray arrayWithCapacity:_mesh->vertices.size()];
    for (int r = 0; r < _mesh->rows; ++r) {
        for (int c = 0; c < _mesh->cols; ++c) {
            const gmcore::MeshVertex& mv = _mesh->at(r, c);
            [vertsJSON addObject:@{
                @"row": @(r), @"col": @(c),
                @"P": @[@(mv.P.x), @(mv.P.y)],
                @"Pu": @[@(mv.Pu.x), @(mv.Pu.y)],
                @"Pv": @[@(mv.Pv.x), @(mv.Pv.y)],
                @"Puv": @[@(mv.Puv.x), @(mv.Puv.y)], // always {0,0} -- see GradientMesh.h
                @"C": @[@(mv.C.r), @(mv.C.g), @(mv.C.b)],
                @"Cu": @[@(mv.Cu.r), @(mv.Cu.g), @(mv.Cu.b)],
                @"Cv": @[@(mv.Cv.r), @(mv.Cv.g), @(mv.Cv.b)],
                @"Cuv": @[@(mv.Cuv.r), @(mv.Cuv.g), @(mv.Cuv.b)],
                @"isBoundary": @(mv.isBoundary),
                @"boundarySide": @(mv.boundarySide),
                @"boundaryT": @(mv.boundaryT),
            }];
        }
    }
    root[@"mesh"] = @{
        @"rows": @(_mesh->rows),
        @"cols": @(_mesh->cols),
        @"vertices": vertsJSON,
    };

    NSMutableArray<NSArray*>* boundaryJSON = [NSMutableArray arrayWithCapacity:4];
    if (_hasBoundary) {
        for (int i = 0; i < 4; ++i) {
            NSMutableArray<NSArray*>* segmentsJSON = [NSMutableArray arrayWithCapacity:_boundary[i].segments.size()];
            for (const CubicBezier& b : _boundary[i].segments) {
                [segmentsJSON addObject:@[
                    @[@(b.p0.x), @(b.p0.y)], @[@(b.p1.x), @(b.p1.y)],
                    @[@(b.p2.x), @(b.p2.y)], @[@(b.p3.x), @(b.p3.y)],
                ]];
            }
            [boundaryJSON addObject:segmentsJSON];
        }
    }
    // [top,right,bottom,left], each now a LIST of one-or-more [p0,p1,p2,p3]
    // segments (see BezierSpline.h/fitBezierSpline) rather than exactly
    // one -- a schema change from before this side could be more than a
    // single cubic; nothing else in this repo parses this field back in.
    root[@"boundary"] = boundaryJSON;

    NSMutableArray<NSArray*>* linesJSON = [NSMutableArray arrayWithCapacity:_vectorLines.size()];
    for (const auto& line : _vectorLines) {
        NSMutableArray<NSArray*>* pts = [NSMutableArray arrayWithCapacity:line.points.size()];
        for (const auto& p : line.points) [pts addObject:@[@(p.x), @(p.y)]];
        [linesJSON addObject:pts];
    }
    root[@"vectorLines"] = linesJSON;

    NSError* jsonErr = nil;
    NSData* data = [NSJSONSerialization dataWithJSONObject:root
                                                     options:(NSJSONWritingPrettyPrinted | NSJSONWritingSortedKeys)
                                                       error:&jsonErr];
    if (!data) { if (error) *error = jsonErr ?: gmError(@"Could not serialize debug data."); return NO; }
    BOOL ok = [data writeToURL:url options:NSDataWritingAtomic error:&jsonErr];
    if (!ok && error) *error = jsonErr ?: gmError(@"Could not write debug data file.");
    return ok;
}

#pragma mark - Presets

// Sibling "Presets" folder next to the currently loaded image -- see
// -debugOutDirectoryURL above, which this is a direct copy of (down to the
// nil cases) with only the folder name changed.
- (nullable NSURL*)presetsDirectoryURL {
    if (!_imageURL) return nil;
    NSURL* dir = [[_imageURL URLByDeletingLastPathComponent] URLByAppendingPathComponent:@"Presets" isDirectory:YES];
    NSError* mkdirErr = nil;
    if (![[NSFileManager defaultManager] createDirectoryAtURL:dir withIntermediateDirectories:YES
                                                     attributes:nil error:&mkdirErr]) {
        NSLog(@"[GMCORE] presets: could not create Presets folder at %@: %@", dir, mkdirErr);
        return nil;
    }
    return dir;
}

// Same "<solver>[_cieluv]_<rows>x<cols>_<timestamp>" convention as
// -debugExportFilename (see that method's comment), "preset" instead of
// "debug".
- (NSString*)presetExportFilename {
    NSString* solverTag = self.useCeresJoint ? @"ceres_joint" : self.useCeresGeometry ? @"ceres_geom" : @"hand_rolled";
    NSString* colorSpaceTag = _meshColorSpaceIsCIELUV ? @"_cieluv" : @"";
    static NSDateFormatter* fmt;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        fmt = [[NSDateFormatter alloc] init];
        fmt.locale = [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"];
        fmt.dateFormat = @"yyyyMMdd-HHmmss-SSS";
    });
    NSString* stamp = [fmt stringFromDate:[NSDate date]];
    return [NSString stringWithFormat:@"gm_preset_%@%@_%ldx%ld_%@.json", solverTag, colorSpaceTag,
            (long)self.meshRows, (long)self.meshCols, stamp];
}

// See DocumentModel.h's comment: same source data (_lastOptsUsed,
// _meshColorSpaceIsCIELUV) as -exportDebugDataToURL:error:'s
// "optimizerOptions"/"colorSpace" fields, same key names too (so a saved
// preset and a debug log's optimizerOptions block are directly
// eyeball-comparable) -- just without the mesh/history/git noise a debug
// dump also carries, since a preset is only ever meant to be re-loaded as
// settings, not inspected as a run record.
- (BOOL)savePresetToURL:(NSURL*)url error:(NSError**)error {
    if (!_hasRunOptimize) {
        if (error) *error = gmError(@"Run Optimize at least once before saving a preset.");
        return NO;
    }

    NSMutableDictionary* root = [NSMutableDictionary dictionary];
    root[@"kind"] = @"GradientMeshStudioPreset";
    if (@available(macOS 10.12, *)) {
        NSISO8601DateFormatter* iso = [[NSISO8601DateFormatter alloc] init];
        root[@"savedAt"] = [iso stringFromDate:[NSDate date]];
    }
    root[@"colorSpace"] = _meshColorSpaceIsCIELUV ? @"CIELUV" : @"sRGB";
    root[@"meshRows"] = @(self.meshRows);
    root[@"meshCols"] = @(self.meshCols);
    // For context when browsing saved presets later -- NOT reapplied by
    // -loadPresetNamed:rows:cols:error: (a preset restores SETTINGS, not a
    // remembered result; the numbers themselves depend on the image too).
    root[@"lastRMSE"] = @(_lastRMSE);
    root[@"lastMAE"] = @(_lastMAE);

    const OptimizerOptions& o = _lastOptsUsed;
    root[@"optimizerOptions"] = @{
        @"samplesPerPatchEdge": @(o.samplesPerPatchEdge),
        @"smoothWeightGeom": @(o.smoothWeightGeom),
        @"smoothGeomEdgeGain": @(o.smoothGeomEdgeGain),
        @"smoothGeomMinFactor": @(o.smoothGeomMinFactor),
        @"geomDataWeight": @(o.geomDataWeight),
        @"smoothWeightColor": @(o.smoothWeightColor),
        @"colorDerivRidge": @(o.colorDerivRidge),
        @"boundaryWeight": @(o.boundaryWeight),
        @"vectorLineWeight": @(o.vectorLineWeight),
        @"vectorLineInfluenceRadius": @(o.vectorLineInfluenceRadius),
        @"geomTangentPriorWeight": @(o.geomTangentPriorWeight),
        @"outerIterationsPerLevel": @(o.outerIterationsPerLevel),
        @"outerConvergenceRelTol": @(o.outerConvergenceRelTol),
        @"outerConvergencePatience": @(o.outerConvergencePatience),
        @"pyramidRestarts": @(o.pyramidRestarts),
        @"geomGaussNewtonItersPerOuter": @(o.geomGaussNewtonItersPerOuter),
        @"cgMaxIterations": @(o.cgMaxIterations),
        @"cgRelTolerance": @(o.cgRelTolerance),
        @"geomDampingInitial": @(o.geomDampingInitial),
        @"useCeresGeometry": @(o.useCeresGeometry),
        @"useCeresJoint": @(o.useCeresJoint),
        @"ceresNumThreads": @(o.ceresNumThreads),
        @"jointGeomStepDampingWeight": @(o.jointGeomStepDampingWeight),
    };

    NSError* jsonErr = nil;
    NSData* data = [NSJSONSerialization dataWithJSONObject:root
                                                     options:(NSJSONWritingPrettyPrinted | NSJSONWritingSortedKeys)
                                                       error:&jsonErr];
    if (!data) { if (error) *error = jsonErr ?: gmError(@"Could not serialize preset."); return NO; }
    BOOL ok = [data writeToURL:url options:NSDataWritingAtomic error:&jsonErr];
    if (ok) {
        self.lastPresetSavePath = url.path;
    } else if (error) {
        *error = jsonErr ?: gmError(@"Could not write preset file.");
    }
    return ok;
}

- (NSArray<NSString*>*)availablePresetNames {
    NSURL* dir = [self presetsDirectoryURL];
    if (!dir) return @[];
    NSArray<NSURL*>* contents = [[NSFileManager defaultManager] contentsOfDirectoryAtURL:dir
                                                                includingPropertiesForKeys:@[NSURLContentModificationDateKey]
                                                                                   options:NSDirectoryEnumerationSkipsHiddenFiles
                                                                                     error:nil];
    if (!contents) return @[];
    NSArray<NSURL*>* jsonFiles = [contents filteredArrayUsingPredicate:
        [NSPredicate predicateWithBlock:^BOOL(NSURL* u, NSDictionary* __unused bindings) {
            return [u.pathExtension.lowercaseString isEqualToString:@"json"];
        }]];
    NSArray<NSURL*>* sorted = [jsonFiles sortedArrayUsingComparator:^NSComparisonResult(NSURL* a, NSURL* b) {
        NSDate* da = nil; NSDate* db = nil;
        [a getResourceValue:&da forKey:NSURLContentModificationDateKey error:nil];
        [b getResourceValue:&db forKey:NSURLContentModificationDateKey error:nil];
        return [(db ?: [NSDate distantPast]) compare:(da ?: [NSDate distantPast])]; // newest first
    }];
    NSMutableArray<NSString*>* names = [NSMutableArray arrayWithCapacity:sorted.count];
    for (NSURL* u in sorted) [names addObject:u.lastPathComponent.stringByDeletingPathExtension];
    return names;
}

- (BOOL)loadPresetNamed:(NSString*)name rows:(NSInteger*)outRows cols:(NSInteger*)outCols
                   error:(NSError**)error {
    NSURL* dir = [self presetsDirectoryURL];
    if (!dir) { if (error) *error = gmError(@"No Presets folder yet (load an image first)."); return NO; }
    NSURL* url = [dir URLByAppendingPathComponent:[name stringByAppendingPathExtension:@"json"]];
    NSData* data = [NSData dataWithContentsOfURL:url options:0 error:error];
    if (!data) return NO;
    NSError* jsonErr = nil;
    id parsed = [NSJSONSerialization JSONObjectWithData:data options:0 error:&jsonErr];
    if (![parsed isKindOfClass:[NSDictionary class]]) {
        if (error) *error = jsonErr ?: gmError(@"Preset file is not valid JSON.");
        return NO;
    }
    NSDictionary* root = (NSDictionary*)parsed;
    NSDictionary* o = root[@"optimizerOptions"];
    if (![o isKindOfClass:[NSDictionary class]]) {
        if (error) *error = gmError(@"Preset is missing its optimizerOptions.");
        return NO;
    }

    self.smoothWeightGeom = [o[@"smoothWeightGeom"] doubleValue];
    self.smoothWeightColor = [o[@"smoothWeightColor"] doubleValue];
    self.colorDerivRidge = [o[@"colorDerivRidge"] doubleValue];
    self.boundaryWeight = [o[@"boundaryWeight"] doubleValue];
    self.geomTangentPriorWeight = [o[@"geomTangentPriorWeight"] doubleValue];
    self.vectorLineWeight = [o[@"vectorLineWeight"] doubleValue];
    self.geomDataWeight = [o[@"geomDataWeight"] doubleValue];
    self.smoothGeomEdgeGain = [o[@"smoothGeomEdgeGain"] doubleValue];
    self.pyramidRestarts = [o[@"pyramidRestarts"] integerValue];
    self.useCeresGeometry = [o[@"useCeresGeometry"] boolValue];
    self.useCeresJoint = [o[@"useCeresJoint"] boolValue];
    // ceresNumThreads' YES/NO <-> 0/1 mapping -- see ceresMultithreaded's
    // comment in DocumentModel.h: 0 or absent (an older preset) -> YES
    // ("auto", this property's own default); any pinned positive value ->
    // NO.
    self.ceresMultithreaded = (o[@"ceresNumThreads"] == nil || [o[@"ceresNumThreads"] integerValue] == 0);
    self.useCIELUVColorSpace = [root[@"colorSpace"] isEqualToString:@"CIELUV"];

    if (outRows) *outRows = [root[@"meshRows"] integerValue];
    if (outCols) *outCols = [root[@"meshCols"] integerValue];
    return YES;
}

@end
