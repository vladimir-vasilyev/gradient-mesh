// BezierSpline.h — cubic Bezier curves used for the mesh's four boundary
// segments (Sun et al., Sec. 4: boundary points are constrained to lie on
// splines fitted to the user-drawn/segmented object outline).
#pragma once
#include "gmcore/Vec2.h"
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

} // namespace gmcore
