// Vec2.h — minimal 2D vector type used for mesh geometry (positions, tangents).
#pragma once
#include <cmath>

namespace gmcore {

struct Vec2 {
    double x = 0.0, y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double cross(const Vec2& o) const { return x * o.y - y * o.x; }
    double length() const { return std::sqrt(x * x + y * y); }
    double lengthSq() const { return x * x + y * y; }
    Vec2 normalized() const { double l = length(); return l > 1e-12 ? Vec2{x / l, y / l} : Vec2{0, 0}; }
};

inline Vec2 operator*(double s, const Vec2& v) { return v * s; }

} // namespace gmcore
