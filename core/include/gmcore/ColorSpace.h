// ColorSpace.h — sRGB <-> CIELUV conversion, added after reading Hogervorst
// (2017, "Colour Interpolation in Gradient Meshes", Bachelor's thesis,
// University of Groningen). That thesis compared several colour spaces for
// interpolating colour across a gradient-mesh patch and concluded CIELUV
// gives the best results: it is perceptually uniform (unlike sRGB/linear
// sRGB), and unlike CIELAB it does not show an unnatural blue->purple->green
// artifact along some transitions. Its own baseline tool already used a
// perceptually uniform space (CIELAB); this project's optimizer/renderer
// instead work directly on raw sRGB-ish Color{r,g,b} triples with no colour
// space at all (see Color.h) -- these conversions are what lets
// DocumentModel optionally run the ENTIRE fit (mesh construction, the
// optimizer's data term, rendering) in CIELUV instead, per that finding.
//
// Design note: everywhere else in gmcore, `Color` is treated as an opaque
// 3-vector -- GradientMesh/MeshOptimizer/MeshOptimizerCeres never interpret
// r/g/b as "red/green/blue" specifically, they just do linear algebra on
// whatever three numbers are there (see FergusonPatch.h's evalHermitePatch,
// generic over T). That is exactly what makes this feature cheap and low
// risk to add: convert the target Image to CIELUV ONCE at the boundary
// (DocumentModel, right after loading), run the *unmodified* existing
// pipeline against that converted image (mesh vertex C/Cu/Cv/Cuv end up
// holding (L*,u*,v*) instead of (r,g,b), and every difference/sum/dot the
// optimizer computes on them remains exactly as valid as before -- it's
// still just vector arithmetic), and convert back to sRGB only at the few
// places that actually need real display-able colour: the final raster
// (screen preview / PNG export), the SVG exporter's per-corner stop colors,
// and the mesh-vertex colour swatch/picker in the UI. See DocumentModel.mm.
//
// IMPORTANT: this only ever converts POINT VALUES (a colour), never a
// derivative (Cu/Cv/Cuv). A "colour derivative" only has meaning within the
// colour space it was computed in -- converting e.g. a CIELUV Cu to sRGB by
// running it through the same nonlinear sRGB<->CIELUV formula as a point
// value would be mathematically wrong (that formula's real derivative
// involves a Jacobian, not a naive per-component substitution). Cu/Cv/Cuv
// are therefore left alone; only the point value (mesh vertex C, or a fully
// patch-interpolated raster pixel, which is a point value even though it
// was produced via a formula that also used derivatives) ever gets
// converted.
//
// The conversion is deliberately "signed-extended" beyond the usual [0,1]
// (sRGB) / valid-gamut (CIELUV) domains: Color.h documents that values go
// unclamped and can go negative or over-range mid-optimization, and this
// project's own Gauss-Newton/Ceres solvers rely on that (see Color.h's
// header comment and clamped01()'s doc). Both the sRGB gamma curve and the
// CIELUV L* formula are extended by sign (f(-x) = -f(x)) using std::cbrt
// (which is well-defined for negative inputs, unlike std::pow with a
// fractional exponent) so a solver that briefly overshoots into
// out-of-gamut territory mid-solve gets a smooth, defined, invertible
// mapping rather than NaNs -- not a physically meaningful colour in that
// regime, but exactly the kind of "well-defined algebra on out-of-range
// intermediate values" this codebase already leans on elsewhere.
#pragma once
#include "gmcore/Color.h"
#include "gmcore/Image.h"

namespace gmcore {

// kCIELUVWorkingScale -- WHY IT'S HERE (found from real on-device runs, not
// anticipated up front): OptimizerOptions' weights (smoothWeightGeom,
// boundaryWeight, geomTangentPriorWeight, vectorLineWeight, smoothWeightColor,
// colorDerivRidge -- see MeshOptimizer.h) are all fixed absolute constants,
// empirically tuned assuming colour differences are roughly O(0.01-1), i.e.
// sRGB's own natural [0,1]-per-channel scale. Standard CIELUV's L* alone
// spans the full [0,100] range for the same luminance span sRGB covers in
// [0,1] -- a ~100x larger numeric scale -- and u*/v* are comparable or
// larger still for saturated colours. A first real on-device CIELUV run
// confirmed the predicted failure mode exactly: RMSE ~2 orders of magnitude
// larger (expected, just a unit change) but ALSO visibly disordered/
// "shuffled" patch placement and artifacts right at the image border, with
// (as a side effect of the same cause) noticeably sharper colour
// transitions. The mechanism: geometry's data-term residual and Jacobian
// both carry a factor of the colour scale, so in a Gauss-Newton normal-
// equations solve its influence relative to the FIXED, colour-scale-
// unaware position/tangent/boundary regularizers grows roughly with the
// SQUARE of that scale (~100x colour scale -> ~10000x more data-term pull
// relative to smoothWeightGeom/boundaryWeight/geomTangentPriorWeight) --
// enough to overpower boundaryWeight=200 and produce exactly the reported
// border artifacts and incoherent neighboring positions. Separately,
// smoothWeightColor/colorDerivRidge (regularizers ON the colour channel
// itself) become proportionally far weaker at ~100x colour scale, which is
// the more likely real explanation for the "sharper" transitions users
// noticed -- not CIELUV's perceptual uniformity per se, but its
// regularization being comparatively under-strength. Dividing by 100 here
// (sRGB [0,1] <-> L* [0,100] is where 100 comes from -- not a fitted
// constant) brings the working representation's magnitude back to roughly
// what every one of those weights was actually tuned against, restoring
// their intended balance against the data term. This is a coarse, only
// approximately-right fix (u*/v* aren't strictly bounded by 100 the way L*
// is), expected to also measurably REDUCE the sharpening effect (same root
// cause as the artifacts it fixes) -- if a user still wants deliberately
// sharper transitions after this, that calls for a dedicated, decoupled
// style control instead of leaning on this scale mismatch (see README).
constexpr double kCIELUVWorkingScale = 100.0;

// A single colour, sRGB (gamma-encoded, nominally 0..1) -> this project's
// CIELUV *working* representation: standard CIE 1976 L*u*v* (D65 white)
// divided by kCIELUVWorkingScale, i.e. roughly L* in [0,1], u*/v* order
// +-1..1.5 for in-gamut sRGB colours -- NOT the raw textbook L*[0,100]
// values (see kCIELUVWorkingScale's comment for why the extra /100 is
// there). See the file header for why out-of-[0,1]/out-of-gamut inputs are
// handled (signed extension) instead of asserting or clamping.
Color srgbToCIELUV(const Color& srgb);

// Inverse of srgbToCIELUV (undoes both the CIE formulas and the /
// kCIELUVWorkingScale division). Round-trips srgbToCIELUV closely, but
// NOT to full floating-point precision (an earlier version of this
// comment claimed it did -- corrected after core/tests/test_main.cpp's
// cieluv_roundtrip test measured otherwise: up to ~3.8e-6 absolute error
// on real sample colors, not the ~1e-15 double-precision noise floor
// "floating-point precision" would imply). Root cause: the sRGB<->XYZ
// conversion below uses the standard published 3x3 matrices, each
// direction independently rounded to 7 significant figures -- they are
// two separately-rounded approximations, not exact algebraic inverses of
// each other, so even a mathematically perfect round-trip through them
// (plus the sRGB gamma curve and cbrt-based L*u*v* formulas, each adding
// their own tiny rounding) accumulates real, structural error on this
// order. Not considered a bug worth fixing -- deriving an exact algebraic
// inverse matrix pair would be needless complexity for a cosmetic
// precision claim, and ~1e-6 in a [0,1]-ish colour channel is far below
// anything visible or anything the optimizer's own convergence
// tolerances care about -- but the claim itself is corrected here to
// match reality rather than left overstated. Holds for any input
// (including the signed-extended out-of-gamut regime described above).
Color cieluvToSRGB(const Color& luv);

// Per-pixel srgbToCIELUV/cieluvToSRGB applied to an entire Image (same
// width/height, new buffer). Used once per loaded image (see
// DocumentModel.mm's `_targetLUV`) and once per rendered raster when
// useCIELUVColorSpace is on.
Image imageSRGBToCIELUV(const Image& srgb);
Image imageCIELUVToSRGB(const Image& luv);

} // namespace gmcore
