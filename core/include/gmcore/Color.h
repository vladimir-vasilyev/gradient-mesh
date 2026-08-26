// Color.h — RGB color in [0,1] double precision (unclamped internally so
// optimization residuals can go negative/over-range during solving).
#pragma once

namespace gmcore {

struct Color {
    double r = 0.0, g = 0.0, b = 0.0;

    Color() = default;
    Color(double r_, double g_, double b_) : r(r_), g(g_), b(b_) {}

    Color operator+(const Color& o) const { return {r + o.r, g + o.g, b + o.b}; }
    Color operator-(const Color& o) const { return {r - o.r, g - o.g, b - o.b}; }
    Color operator*(double s) const { return {r * s, g * s, b * s}; }
    Color operator/(double s) const { return {r / s, g / s, b / s}; }
    Color& operator+=(const Color& o) { r += o.r; g += o.g; b += o.b; return *this; }
    Color& operator-=(const Color& o) { r -= o.r; g -= o.g; b -= o.b; return *this; }

    double dot(const Color& o) const { return r * o.r + g * o.g + b * o.b; }
    double lengthSq() const { return r * r + g * g + b * b; }

    Color clamped01() const {
        auto c = [](double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); };
        return {c(r), c(g), c(b)};
    }
};

inline Color operator*(double s, const Color& c) { return c * s; }

} // namespace gmcore
