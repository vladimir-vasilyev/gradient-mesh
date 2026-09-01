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

// A single colour, sRGB (gamma-encoded, nominally 0..1) -> CIELUV
// (L* nominally 0..100, u*/v* roughly -100..100 for in-gamut sRGB colours,
// D65 reference white, standard CIE 1976 formulas). See the file header for
// why out-of-[0,1]/out-of-gamut inputs are handled (signed extension)
// instead of asserting or clamping.
Color srgbToCIELUV(const Color& srgb);

// Inverse of srgbToCIELUV. Round-trips srgbToCIELUV to within floating-point
// precision for any input (including the signed-extended out-of-gamut
// regime described above).
Color cieluvToSRGB(const Color& luv);

// Per-pixel srgbToCIELUV/cieluvToSRGB applied to an entire Image (same
// width/height, new buffer). Used once per loaded image (see
// DocumentModel.mm's `_targetLUV`) and once per rendered raster when
// useCIELUVColorSpace is on.
Image imageSRGBToCIELUV(const Image& srgb);
Image imageCIELUVToSRGB(const Image& luv);

} // namespace gmcore
