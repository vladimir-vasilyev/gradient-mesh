#include "gmcore/ColorSpace.h"
#include <cmath>

namespace gmcore {

namespace {

// D65 reference white in CIEXYZ (the same white point sRGB's own matrices
// are defined against), and its CIE 1976 u',v' chromaticity -- computed
// from the constants below (denom = Xn + 15*Yn + 3*Zn), not hand-typed
// separately, so the two can never silently drift apart.
constexpr double kXn = 0.95047, kYn = 1.00000, kZn = 1.08883;
constexpr double kWhiteDenom = kXn + 15.0 * kYn + 3.0 * kZn;
constexpr double kUn = 4.0 * kXn / kWhiteDenom;
constexpr double kVn = 9.0 * kYn / kWhiteDenom;

// CIE's standard L* piecewise-linear/cube-root breakpoint, shared with
// CIELAB (same constants, same rationale: avoid an infinite slope at 0).
constexpr double kEpsilon = 216.0 / 24389.0;   // (6/29)^3
constexpr double kKappa = 24389.0 / 27.0;      // (29/3)^3

// sign(x) * f(|x|) -- see ColorSpace.h's file header on why every step here
// is extended this way instead of clamping/asserting on out-of-range input.
inline double signedExtend(double x, double (*f)(double)) {
    return x < 0.0 ? -f(-x) : f(x);
}

double srgbGammaDecode(double c) {
    return c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
}
double srgbGammaEncode(double lin) {
    return lin <= 0.0031308 ? lin * 12.92 : 1.055 * std::pow(lin, 1.0 / 2.4) - 0.055;
}

// Linear sRGB <-> CIEXYZ (D65), the standard sRGB/IEC 61966-2-1 matrices.
struct XYZ { double X, Y, Z; };

XYZ linearSRGBToXYZ(double r, double g, double b) {
    return {
        0.4124564 * r + 0.3575761 * g + 0.1804375 * b,
        0.2126729 * r + 0.7151522 * g + 0.0721750 * b,
        0.0193339 * r + 0.1191920 * g + 0.9503041 * b,
    };
}
void xyzToLinearSRGB(const XYZ& xyz, double* r, double* g, double* b) {
    *r =  3.2404542 * xyz.X - 1.5371385 * xyz.Y - 0.4985314 * xyz.Z;
    *g = -0.9692660 * xyz.X + 1.8760108 * xyz.Y + 0.0415560 * xyz.Z;
    *b =  0.0556434 * xyz.X - 0.2040259 * xyz.Y + 1.0572252 * xyz.Z;
}

} // namespace

Color srgbToCIELUV(const Color& srgb) {
    double r = signedExtend(srgb.r, srgbGammaDecode);
    double g = signedExtend(srgb.g, srgbGammaDecode);
    double b = signedExtend(srgb.b, srgbGammaDecode);
    XYZ xyz = linearSRGBToXYZ(r, g, b);

    double denom = xyz.X + 15.0 * xyz.Y + 3.0 * xyz.Z;
    // denom == 0 only at/near black (X=Y=Z=0); fall back to the reference
    // white's chromaticity so u*/v* come out exactly 0 there instead of a
    // 0/0 NaN (matches the well-known convention for CIELUV at L*=0).
    double uPrime = (denom != 0.0) ? 4.0 * xyz.X / denom : kUn;
    double vPrime = (denom != 0.0) ? 9.0 * xyz.Y / denom : kVn;

    double yr = xyz.Y / kYn;
    // std::cbrt (not std::pow(yr, 1.0/3.0)) is well-defined for yr < 0,
    // which the signed extension above can produce for an out-of-gamut
    // intermediate value mid-optimization -- see file header.
    double L = (yr > kEpsilon) ? (116.0 * std::cbrt(yr) - 16.0) : (kKappa * yr);

    double u = 13.0 * L * (uPrime - kUn);
    double v = 13.0 * L * (vPrime - kVn);
    return Color(L, u, v);
}

Color cieluvToSRGB(const Color& luv) {
    double L = luv.r, u = luv.g, v = luv.b;

    // Inverse of the L* piecewise formula above.
    double yr = (L > kKappa * kEpsilon) ? std::pow((L + 16.0) / 116.0, 3.0) : (L / kKappa);
    double Y = yr * kYn;

    // Inverse of u*=13L(u'-un'), v*=13L(v'-vn'); guard L==0 (u*/v* are then
    // 0/0 by construction of the forward formula, so any u',v' work -- use
    // the reference white's, same convention as the forward direction).
    double uPrime = (L != 0.0) ? (u / (13.0 * L) + kUn) : kUn;
    double vPrime = (L != 0.0) ? (v / (13.0 * L) + kVn) : kVn;

    XYZ xyz;
    xyz.Y = Y;
    if (vPrime != 0.0) {
        xyz.X = Y * 9.0 * uPrime / (4.0 * vPrime);
        xyz.Z = Y * (12.0 - 3.0 * uPrime - 20.0 * vPrime) / (4.0 * vPrime);
    } else {
        xyz.X = 0.0;
        xyz.Z = 0.0;
    }

    double rLin, gLin, bLin;
    xyzToLinearSRGB(xyz, &rLin, &gLin, &bLin);
    double r = signedExtend(rLin, srgbGammaEncode);
    double g = signedExtend(gLin, srgbGammaEncode);
    double b = signedExtend(bLin, srgbGammaEncode);
    return Color(r, g, b);
}

Image imageSRGBToCIELUV(const Image& srgb) {
    Image out(srgb.width, srgb.height);
    for (int y = 0; y < srgb.height; ++y)
        for (int x = 0; x < srgb.width; ++x)
            out.set(x, y, srgbToCIELUV(srgb.at(x, y)));
    return out;
}

Image imageCIELUVToSRGB(const Image& luv) {
    Image out(luv.width, luv.height);
    for (int y = 0; y < luv.height; ++y)
        for (int x = 0; x < luv.width; ++x)
            out.set(x, y, cieluvToSRGB(luv.at(x, y)));
    return out;
}

} // namespace gmcore
