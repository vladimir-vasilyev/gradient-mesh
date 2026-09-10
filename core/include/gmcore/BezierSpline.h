// BezierSpline.h — cubic Bezier curves used for the mesh's four boundary
// segments (Sun et al., Sec. 4: boundary points are constrained to lie on
// splines fitted to the user-drawn/segmented object outline).
#pragma once
#include "gmcore/Vec2.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace gmcore {

struct CubicBezier {
    Vec2 p0, p1, p2, p3;

    Vec2 eval(double t) const {
        double mt = 1 - t;
        double a = mt * mt * mt, b = 3 * mt * mt * t, c = 3 * mt * t * t, d = t * t * t;
        return p0 * a + p1 * b + p2 * c + p3 * d;
    }
    Vec2 evalDeriv(double t) const {
        double mt = 1 - t;
        return (p1 - p0) * (3 * mt * mt) + (p2 - p1) * (6 * mt * t) + (p3 - p2) * (3 * t * t);
    }

    // Closest point search: coarse uniform sample + local refinement.
    double closestT(const Vec2& pt, int coarseSamples = 40) const {
        double bestT = 0, bestD = 1e300;
        for (int i = 0; i <= coarseSamples; ++i) {
            double t = double(i) / coarseSamples;
            double d = (eval(t) - pt).lengthSq();
            if (d < bestD) { bestD = d; bestT = t; }
        }
        // Newton-ish refinement using derivative/second-derivative approx via finite diff.
        double t = bestT;
        for (int it = 0; it < 12; ++it) {
            double h = 1e-4;
            double tp = std::min(1.0, t + h), tm = std::max(0.0, t - h);
            double fp = (eval(tp) - pt).lengthSq();
            double fm = (eval(tm) - pt).lengthSq();
            double f0 = (eval(t) - pt).lengthSq();
            double g = (fp - fm) / (tp - tm + 1e-12);
            double h2 = (fp - 2 * f0 + fm) / (h * h);
            if (std::abs(h2) < 1e-9) break;
            double step = g / h2;
            t -= step;
            if (t < 0) t = 0;
            if (t > 1) t = 1;
        }
        return t;
    }
    Vec2 closestPoint(const Vec2& pt) const { return eval(closestT(pt)); }
};

// Least-squares fit of a single cubic Bezier through a polyline of points,
// with the endpoints pinned to pts.front()/pts.back() (classic two-point
// tangent-based fit, chord-length parameterized; see Graphics Gems I,
// "An Algorithm for Automatically Fitting Digitized Curves").
CubicBezier fitCubicBezier(const std::vector<Vec2>& pts);

// A single mesh boundary side as a CHAIN of cubic Bezier segments, not
// just one -- Sun et al. Sec. 4 explicitly says "each boundary consists
// of one or more cubic Bezier splines" (a single cubic has at most one
// inflection point, which is nowhere near enough to track a real,
// non-convex object silhouette -- e.g. one produced by LazySnapping.h's
// segmentation -- the way it could a roughly-convex hand-picked side).
// Segments are stored end-to-end (segments[i].p3 == segments[i+1].p0,
// exactly, by construction -- see fitBezierSpline below) and the whole
// chain is parameterized by ONE t in [0,1] spanning all of them, so
// GradientMesh/MeshOptimizer/MeshOptimizerCeres can use a BezierSpline
// exactly where they used to use a plain CubicBezier -- eval/evalDeriv/
// closestT are the same three-method interface, just resolving to
// whichever segment t falls into first.
struct BezierSpline {
    std::vector<CubicBezier> segments;

    // Maps global t in [0,1] to (segment index, local t in [0,1] within
    // that segment). segments.empty() is a degenerate/unfitted spline --
    // callers should treat it as they would a null curve (not expected in
    // practice: fitBezierSpline always returns at least one segment for
    // any non-empty input).
    void segmentAndLocalT(double t, size_t& segIdx, double& localT) const {
        size_t n = segments.size();
        if (n == 0) { segIdx = 0; localT = 0; return; }
        t = std::max(0.0, std::min(1.0, t));
        double scaled = t * double(n);
        segIdx = std::min(n - 1, size_t(scaled));
        localT = scaled - double(segIdx);
    }

    Vec2 eval(double t) const {
        if (segments.empty()) return Vec2{};
        size_t segIdx; double localT;
        segmentAndLocalT(t, segIdx, localT);
        return segments[segIdx].eval(localT);
    }
    // NOTE: this is the derivative of the LOCAL segment w.r.t. its own
    // [0,1] parameter, not w.r.t. the spline's global t (which would carry
    // an extra factor of `segments.size()`, canceling out of every actual
    // use in this codebase -- MeshOptimizer/MeshOptimizerCeres only ever
    // normalize this into a unit tangent/normal direction, so the missing
    // constant scale factor is irrelevant there). Matches CubicBezier's
    // own evalDeriv in that same "not arc-length calibrated" sense.
    Vec2 evalDeriv(double t) const {
        if (segments.empty()) return Vec2{};
        size_t segIdx; double localT;
        segmentAndLocalT(t, segIdx, localT);
        return segments[segIdx].evalDeriv(localT);
    }

    // Closest point search across every segment, each via CubicBezier's
    // own coarse-sample + Newton-refine closestT -- fine for the point
    // counts here (a handful to a few dozen segments per boundary side).
    double closestT(const Vec2& pt) const {
        if (segments.empty()) return 0.0;
        size_t bestSeg = 0; double bestLocalT = 0.0, bestD = 1e300;
        for (size_t i = 0; i < segments.size(); ++i) {
            double t = segments[i].closestT(pt);
            double d = (segments[i].eval(t) - pt).lengthSq();
            if (d < bestD) { bestD = d; bestSeg = i; bestLocalT = t; }
        }
        return (double(bestSeg) + bestLocalT) / double(segments.size());
    }
    Vec2 closestPoint(const Vec2& pt) const { return eval(closestT(pt)); }
};

// Adaptive multi-segment fit (Graphics Gems I's recursive curve-splitting
// scheme, built on fitCubicBezier above): fits one cubic through all of
// `pts`; if every point's distance to that curve is within
// maxErrorPixels, returns it as a single-segment BezierSpline (this is
// exactly fitCubicBezier's old single-cubic behavior, now the "already
// good enough" special case of this function rather than a separate code
// path). Otherwise splits `pts` at whichever point deviates most and
// recursively fits each half (sharing that split point, so consecutive
// segments always join exactly), up to maxDepth levels of recursion
// (default 6 -- a hard cap of 2^6=64 segments per side, purely
// defensive: real input here is a handful to a few dozen points, already
// simplified by ContourTracing.h's simplifyClosedPolygon or by however
// many corners a human clicked, so this cap is never expected to bind in
// practice). maxErrorPixels defaults to 3.0px -- slightly above
// simplifyClosedPolygon's own default 2.0px epsilon, so this isn't
// fighting to reproduce deviations already smaller than that
// simplification pass considered noise; chosen, not measured -- a
// reasonable starting point, not verified against real photos yet (see
// README).
BezierSpline fitBezierSpline(const std::vector<Vec2>& pts, double maxErrorPixels = 3.0, int maxDepth = 6);

} // namespace gmcore
