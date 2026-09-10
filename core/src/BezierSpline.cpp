#include "gmcore/BezierSpline.h"
#include <algorithm>

namespace gmcore {

static inline double B0(double t) { double mt = 1 - t; return mt * mt * mt; }
static inline double B1(double t) { double mt = 1 - t; return 3 * mt * mt * t; }
static inline double B2(double t) { double mt = 1 - t; return 3 * mt * t * t; }
static inline double B3(double t) { return t * t * t; }

CubicBezier fitCubicBezier(const std::vector<Vec2>& pts) {
    CubicBezier curve;
    int n = (int)pts.size();
    if (n < 2) { curve.p0 = curve.p1 = curve.p2 = curve.p3 = (n == 1 ? pts[0] : Vec2{}); return curve; }

    Vec2 p0 = pts.front(), p3 = pts.back();
    curve.p0 = p0;
    curve.p3 = p3;

    Vec2 leftTangent = (n >= 2) ? (pts[1] - pts[0]).normalized() : Vec2{1, 0};
    Vec2 rightTangent = (n >= 2) ? (pts[n - 2] - pts[n - 1]).normalized() : Vec2{-1, 0};
    if (leftTangent.lengthSq() < 1e-12) leftTangent = (p3 - p0).normalized();
    if (rightTangent.lengthSq() < 1e-12) rightTangent = (p0 - p3).normalized();

    // Chord-length parameterization.
    std::vector<double> u(n, 0.0);
    double total = 0.0;
    for (int i = 1; i < n; ++i) total += (pts[i] - pts[i - 1]).length();
    if (total < 1e-9) total = 1.0;
    double acc = 0.0;
    for (int i = 0; i < n; ++i) {
        if (i > 0) acc += (pts[i] - pts[i - 1]).length();
        u[i] = acc / total;
    }

    double C00 = 0, C01 = 0, C11 = 0, X0 = 0, X1 = 0;
    for (int i = 0; i < n; ++i) {
        double t = u[i];
        Vec2 a0 = leftTangent * B1(t);
        Vec2 a1 = rightTangent * B2(t);
        Vec2 base = p0 * (B0(t) + B1(t)) + p3 * (B2(t) + B3(t)); // Bezier(p0,p0,p3,p3,t)
        Vec2 tmp = pts[i] - base;
        C00 += a0.dot(a0);
        C01 += a0.dot(a1);
        C11 += a1.dot(a1);
        X0 += a0.dot(tmp);
        X1 += a1.dot(tmp);
    }

    double det = C00 * C11 - C01 * C01;
    double alpha1, alpha2;
    double chordLen = (p3 - p0).length();
    if (std::abs(det) > 1e-9) {
        alpha1 = (X0 * C11 - X1 * C01) / det;
        alpha2 = (C00 * X1 - C01 * X0) / det;
    } else {
        alpha1 = alpha2 = chordLen / 3.0;
    }
    double minAlpha = chordLen * 1e-3;
    if (alpha1 < minAlpha || alpha2 < minAlpha) {
        alpha1 = alpha2 = chordLen / 3.0;
    }

    curve.p1 = p0 + leftTangent * alpha1;
    curve.p2 = p3 + rightTangent * alpha2;
    return curve;
}

namespace {

// Point-to-curve squared distance via CubicBezier's own closestT --
// consistent with how BezierSpline::closestT itself measures distance,
// so "does this candidate fit within tolerance" and "how do later
// queries see this curve" never disagree.
double squaredDeviation(const CubicBezier& curve, const Vec2& pt) {
    return (curve.eval(curve.closestT(pt)) - pt).lengthSq();
}

// Recursive Graphics-Gems-style split-and-refit. `pts` always has at
// least 2 points on entry (the depth==0/pts.size()<4 base cases below
// both return early via fitCubicBezier itself, which already handles
// n<2 degenerately -- see its own comment). Appends the resulting
// segment(s), in order, onto `out`.
void fitRecursive(const std::vector<Vec2>& pts, double maxErrorSq, int depth, std::vector<CubicBezier>& out) {
    CubicBezier candidate = fitCubicBezier(pts);
    // Too few points to meaningfully split further, or hit the recursion
    // cap -- accept whatever fitCubicBezier produced even if it's above
    // tolerance (a straight 2-3 point run can't be improved by splitting;
    // see fitBezierSpline's own comment on why the depth cap is purely
    // defensive and not expected to bind on this project's real inputs).
    if (depth <= 0 || pts.size() < 4) { out.push_back(candidate); return; }

    size_t worstIdx = 0; double worstD = -1.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        double d = squaredDeviation(candidate, pts[i]);
        if (d > worstD) { worstD = d; worstIdx = i; }
    }
    if (worstD <= maxErrorSq) { out.push_back(candidate); return; }

    // Split at the worst point, sharing it between both halves so the two
    // recursively-fitted sub-curves join exactly (no gap). Guard against a
    // degenerate split (worst point at an end) producing an empty half --
    // falls back to accepting the single-cubic fit rather than recursing
    // forever on an unsplittable point set.
    if (worstIdx < 2 || worstIdx > pts.size() - 3) { out.push_back(candidate); return; }
    std::vector<Vec2> left(pts.begin(), pts.begin() + worstIdx + 1);
    std::vector<Vec2> right(pts.begin() + worstIdx, pts.end());
    fitRecursive(left, maxErrorSq, depth - 1, out);
    fitRecursive(right, maxErrorSq, depth - 1, out);
}

} // namespace

BezierSpline fitBezierSpline(const std::vector<Vec2>& pts, double maxErrorPixels, int maxDepth) {
    BezierSpline spline;
    if (pts.size() < 2) {
        if (!pts.empty()) { CubicBezier c; c.p0 = c.p1 = c.p2 = c.p3 = pts[0]; spline.segments.push_back(c); }
        return spline;
    }
    fitRecursive(pts, maxErrorPixels * maxErrorPixels, std::max(0, maxDepth), spline.segments);
    return spline;
}

} // namespace gmcore
