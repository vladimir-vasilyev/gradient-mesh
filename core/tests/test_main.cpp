// core/tests/test_main.cpp — permanent, dependency-free regression suite
// for gmcore.
//
// WHY THIS EXISTS: this project has no Objective-C++/AppKit compiler
// anywhere in its development environment (only g++ on Linux, via this
// sandbox's device-bridge shell) -- every mac/*.mm change ships genuinely
// unverified by the author of those changes until the user's own Xcode
// build runs. The pure-C++ core/ library is the one part that CAN be
// compiled and run right here, and this project's history is full of real
// bugs that a suite like this would have caught immediately instead of
// needing a human to notice a washed-out preview or a mirrored image (see
// README's "Known simplifications"/bug-fix sections): two separate
// vertical-flip bugs, two separate "ambiguous color space" bugs, a solver
// early-exit bug, a boundary-vertex-frozen bug, a Ceres color-gating bug,
// and more. Ad hoc one-off verification programs were written and run for
// several of these (e.g. the MeshRenderBuffers/GLSL-shader cross-checks --
// see GLShaderSources.h's header comment) but lived only in a scratch
// directory and were thrown away afterward. This file promotes that
// practice into a permanent, checked-in suite that runs in seconds and is
// meant to be re-run after every core/ change, from now on.
//
// No external test framework (matches the rest of gmcore's "no external
// dependency" convention -- see SparseBlockSolver.h's header comment) --
// just a tiny CHECK/CHECK_NEAR macro pair that records failures and keeps
// going, plus a table of named test functions run from main(). Build/run:
//   cmake -B build . && cmake --build build -j --target gmcore_tests
//   ./build/gmcore_tests
#include "gmcore/Vec2.h"
#include "gmcore/Color.h"
#include "gmcore/BezierSpline.h"
#include "gmcore/ColorSpace.h"
#include "gmcore/GradientMesh.h"
#include "gmcore/MeshOptimizer.h"
#include "gmcore/MeshRenderBuffers.h"
#include "gmcore/SparseBlockSolver.h"
#include "gmcore/SVGExporter.h"
#include "gmcore/Image.h"
#include "gmcore/MaxFlowGraph.h"
#include "gmcore/LazySnapping.h"
#include "gmcore/ContourTracing.h"

#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>

using namespace gmcore;

// ---------------------------------------------------------------------------
// Tiny assertion framework
// ---------------------------------------------------------------------------
namespace {

int g_checksThisTest = 0;
int g_failuresThisTest = 0;
int g_totalChecks = 0;
int g_totalFailures = 0;

void recordCheck(bool cond, const char* exprText, const char* file, int line, const std::string& detail) {
    ++g_checksThisTest;
    ++g_totalChecks;
    if (!cond) {
        ++g_failuresThisTest;
        ++g_totalFailures;
        std::fprintf(stderr, "    FAIL: %s (%s:%d)%s%s\n", exprText, file, line,
                     detail.empty() ? "" : " -- ", detail.c_str());
    }
}

} // namespace

#define CHECK(cond) recordCheck((cond), #cond, __FILE__, __LINE__, "")

#define CHECK_NEAR(a, b, tol) do { \
    double _a = (double)(a), _b = (double)(b), _tol = (double)(tol); \
    double _diff = std::fabs(_a - _b); \
    char _buf[160]; \
    std::snprintf(_buf, sizeof(_buf), "%s=%.10g vs %s=%.10g, |diff|=%.3e > tol=%.3e", #a, _a, #b, _b, _diff, _tol); \
    recordCheck(_diff <= _tol, #a " ~= " #b, __FILE__, __LINE__, _buf); \
} while (0)

// ---------------------------------------------------------------------------
// Test bodies
// ---------------------------------------------------------------------------

// CubicBezier: endpoints exact, and closestPoint/closestT actually finds a
// point it was given (i.e. the coarse-sample + Newton refinement in
// BezierSpline.h's closestT doesn't get stuck).
static void test_bezier_endpoints_and_closest() {
    CubicBezier b;
    b.p0 = {10, 20}; b.p1 = {40, -10}; b.p2 = {70, 50}; b.p3 = {100, 30};
    CHECK_NEAR(b.eval(0.0).x, b.p0.x, 1e-9);
    CHECK_NEAR(b.eval(0.0).y, b.p0.y, 1e-9);
    CHECK_NEAR(b.eval(1.0).x, b.p3.x, 1e-9);
    CHECK_NEAR(b.eval(1.0).y, b.p3.y, 1e-9);

    for (double t : {0.1, 0.37, 0.5, 0.82}) {
        Vec2 onCurve = b.eval(t);
        Vec2 nearest = b.closestPoint(onCurve);
        CHECK_NEAR(nearest.x, onCurve.x, 1e-2);
        CHECK_NEAR(nearest.y, onCurve.y, 1e-2);
    }
}

// fitCubicBezier: fitting a perfectly straight, evenly-spaced polyline
// should reproduce (very close to) that line, endpoints pinned exactly.
static void test_bezier_fit_straight_line() {
    std::vector<Vec2> pts;
    for (int i = 0; i <= 10; ++i) pts.push_back(Vec2{double(i) * 5.0, double(i) * 5.0});
    CubicBezier fit = fitCubicBezier(pts);
    CHECK_NEAR(fit.p0.x, pts.front().x, 1e-9);
    CHECK_NEAR(fit.p0.y, pts.front().y, 1e-9);
    CHECK_NEAR(fit.p3.x, pts.back().x, 1e-9);
    CHECK_NEAR(fit.p3.y, pts.back().y, 1e-9);
    Vec2 mid = fit.eval(0.5);
    Vec2 expectedMid = (pts.front() + pts.back()) * 0.5;
    CHECK_NEAR(mid.x, expectedMid.x, 1e-6);
    CHECK_NEAR(mid.y, expectedMid.y, 1e-6);
}

// fitBezierSpline: a straight, evenly-spaced polyline already fits a
// single cubic essentially exactly (see test_bezier_fit_straight_line
// above), so the adaptive splitter should NOT split it further -- this is
// the "already good enough" fast path fitBezierSpline's own header
// comment describes, and it matters in practice: most manually-traced or
// segmented sides are reasonably smooth, and needlessly splitting them
// would just add mesh-irrelevant extra curve segments for no benefit.
static void test_bezier_spline_no_split_for_smooth_input() {
    std::vector<Vec2> pts;
    for (int i = 0; i <= 20; ++i) pts.push_back(Vec2{double(i) * 5.0, double(i) * 5.0});
    BezierSpline sp = fitBezierSpline(pts, /*maxErrorPixels=*/3.0, /*maxDepth=*/6);
    CHECK(sp.segments.size() == 1);
    CHECK_NEAR(sp.eval(0.0).x, pts.front().x, 1e-9);
    CHECK_NEAR(sp.eval(0.0).y, pts.front().y, 1e-9);
    CHECK_NEAR(sp.eval(1.0).x, pts.back().x, 1e-9);
    CHECK_NEAR(sp.eval(1.0).y, pts.back().y, 1e-9);
}

// fitBezierSpline: a sharp zigzag ("W" shape, 5 straight legs meeting at
// hard corners) is exactly the case a SINGLE cubic Bezier structurally
// cannot track (a cubic has at most one inflection point) -- this is the
// direct stand-in for a real, non-convex object silhouette from
// LazySnapping-based segmentation. The adaptive splitter should (a) split
// into multiple segments, (b) keep every input point within
// maxErrorPixels of the fitted spline, (c) still pin the true start/end
// endpoints exactly, and (d) stay perfectly continuous across every
// internal segment joint (consecutive segments must share their
// endpoint exactly, by construction -- a visible gap in the rendered
// boundary would be a much worse regression than the single-cubic
// approximation error this feature exists to fix).
static void test_bezier_spline_splits_for_sharp_zigzag() {
    std::vector<Vec2> verts = {{0, 0}, {20, 40}, {40, 0}, {60, 40}, {80, 0}, {100, 40}};
    std::vector<Vec2> pts;
    for (size_t seg = 0; seg + 1 < verts.size(); ++seg) {
        for (int i = 0; i < 20; ++i) {
            double t = double(i) / 20.0;
            pts.push_back(verts[seg] + (verts[seg + 1] - verts[seg]) * t);
        }
    }
    pts.push_back(verts.back());

    const double maxErrorPixels = 3.0;
    BezierSpline sp = fitBezierSpline(pts, maxErrorPixels, /*maxDepth=*/6);
    CHECK(sp.segments.size() > 1);

    CHECK_NEAR(sp.eval(0.0).x, pts.front().x, 1e-9);
    CHECK_NEAR(sp.eval(0.0).y, pts.front().y, 1e-9);
    CHECK_NEAR(sp.eval(1.0).x, pts.back().x, 1e-9);
    CHECK_NEAR(sp.eval(1.0).y, pts.back().y, 1e-9);

    double maxDeviation = 0.0;
    for (auto& p : pts) {
        Vec2 nearest = sp.closestPoint(p);
        maxDeviation = std::max(maxDeviation, (nearest - p).length());
    }
    CHECK(maxDeviation <= maxErrorPixels + 1e-6);

    for (size_t i = 0; i + 1 < sp.segments.size(); ++i) {
        CHECK_NEAR(sp.segments[i].p3.x, sp.segments[i + 1].p0.x, 1e-9);
        CHECK_NEAR(sp.segments[i].p3.y, sp.segments[i + 1].p0.y, 1e-9);
    }
}

// BezierSpline::closestT: for a point genuinely ON the curve (sampled via
// eval at a handful of global t values spanning multiple segments),
// closestT should recover a t whose eval() lands back on that same
// point -- i.e. round-tripping through closestT doesn't drift to some
// OTHER segment's closer-looking-but-wrong point near a joint, which is
// exactly the kind of bug that would silently corrupt boundary-vertex
// re-projection during optimization (see MeshOptimizer.cpp/
// MeshOptimizerCeres.cpp's `v.boundaryT = mesh.boundary[...].closestT(v.P)`
// use of this same method).
static void test_bezier_spline_closest_t_roundtrip() {
    std::vector<Vec2> verts = {{0, 0}, {20, 40}, {40, 0}, {60, 40}, {80, 0}, {100, 40}};
    std::vector<Vec2> pts;
    for (size_t seg = 0; seg + 1 < verts.size(); ++seg) {
        for (int i = 0; i < 20; ++i) {
            double t = double(i) / 20.0;
            pts.push_back(verts[seg] + (verts[seg + 1] - verts[seg]) * t);
        }
    }
    pts.push_back(verts.back());
    BezierSpline sp = fitBezierSpline(pts, 3.0, 6);
    CHECK(sp.segments.size() > 1); // sanity: this test only means something if it actually split

    for (double t : {0.0, 0.1, 0.25, 0.37, 0.5, 0.63, 0.75, 0.9, 1.0}) {
        Vec2 onCurve = sp.eval(t);
        double foundT = sp.closestT(onCurve);
        Vec2 roundTripped = sp.eval(foundT);
        CHECK_NEAR(roundTripped.x, onCurve.x, 1e-2);
        CHECK_NEAR(roundTripped.y, onCurve.y, 1e-2);
    }
}

// ColorSpace.h's cieluvToSRGB doc comment used to claim srgbToCIELUV/
// cieluvToSRGB round-trip "to within floating-point precision" for ANY
// input, including out-of-[0,1]/out-of-gamut values (the signed-extension
// design -- see that header's comment). Running this test with a 1e-9
// tolerance (genuine double-precision floating-point noise) FAILED on 33
// of 36 checks, with differences up to ~3.8e-6 -- i.e. the original claim
// was measurably wrong, not just imprecisely worded. Root cause (see
// ColorSpace.cpp): the sRGB<->XYZ conversion uses the standard published
// 3x3 matrices, independently rounded to 7 significant figures in each
// direction -- the forward and inverse matrices are NOT exact algebraic
// inverses of each other, they're two separately-rounded approximations,
// so even a mathematically perfect round-trip through them (plus the sRGB
// gamma curve and cbrt-based L*u*v* formulas, each adding their own tiny
// rounding) accumulates real, structural (not just floating-point) error
// on this order. Not a functional bug worth "fixing" (deriving an exact
// algebraic inverse pair would be needless complexity for a cosmetic
// precision claim, and ~1e-6 in a [0,1]-ish color channel is far below
// anything visible), but the doc comment's claim was corrected to match
// this measured reality -- see ColorSpace.h. This test's tolerance
// (1e-5) is set from the measured ~3.8e-6 worst case above, generously
// rounded, so it stays a real regression guard (would still catch a
// genuine round-trip break) without re-asserting the disproven
// "floating-point precision" claim.
static void test_cieluv_roundtrip() {
    const std::vector<Color> samples = {
        {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {0.5, 0.5, 0.5},
        {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
        {0.2, 0.7, 0.9}, {0.83, 0.12, 0.44},
        // Out-of-[0,1] / out-of-gamut -- exactly the regime the optimizer
        // can transiently visit mid-solve (see Color.h/ColorSpace.h).
        {-0.1, 1.3, 0.4}, {-0.5, -0.5, -0.5}, {2.0, -1.0, 0.3}, {-2.5, 3.1, -0.05},
    };
    for (const Color& c : samples) {
        Color luv = srgbToCIELUV(c);
        Color back = cieluvToSRGB(luv);
        CHECK_NEAR(back.r, c.r, 1e-5);
        CHECK_NEAR(back.g, c.g, 1e-5);
        CHECK_NEAR(back.b, c.b, 1e-5);
    }
}

// GradientMesh::evalPos/evalColor's exact indexing convention (derived
// directly from GradientMesh.cpp's evalPos: corners[a][b] =
// geomCorner(patchRow + b, patchCol + a)) means the bicubic Hermite patch
// must reproduce its 4 corner CONTROL POINTS exactly at (u,v) in
// {0,1}x{0,1} -- a basic, structural correctness property of the Ferguson
// patch representation itself (Sun et al. 2007 Sec. 3). This is exactly
// the kind of row/col indexing invariant this project has gotten wrong
// before in OTHER contexts (the two vertical-flip bugs -- see README) --
// cheap to check here and would catch a similar mistake in geomCorner/
// colorCorner/evalPos/evalColor immediately, in seconds, without needing a
// human to notice a warped reconstruction.
static void test_mesh_hermite_corner_exactness() {
    GradientMesh mesh;
    mesh.rows = 3; mesh.cols = 4;
    mesh.vertices.resize(size_t(mesh.rows) * mesh.cols);
    for (int row = 0; row < mesh.rows; ++row) {
        for (int col = 0; col < mesh.cols; ++col) {
            MeshVertex v;
            v.P = {col * 37.0, row * 51.0}; // distinct per vertex, easy to tell apart
            v.Pu = {0, 0}; v.Pv = {0, 0}; v.Puv = {0, 0}; // zero tangents: the (u,v) corner
                                                           // property below holds regardless
                                                           // of tangents (see FergusonPatch.h:
                                                           // at u=0 or 1 / v=0 or 1 the tangent
                                                           // basis weights vanish), but keeping
                                                           // them at zero makes the corner
                                                           // value trivially just P, C.
            v.C = {col * 0.11, row * 0.07, (row + col) * 0.03};
            v.Cu = {0, 0, 0}; v.Cv = {0, 0, 0}; v.Cuv = {0, 0, 0};
            mesh.vertices[mesh.idx(row, col)] = v;
        }
    }

    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            struct Corner { double u, v; int rowOff, colOff; };
            const Corner corners[4] = {
                {0.0, 0.0, 0, 0}, {1.0, 0.0, 0, 1}, {0.0, 1.0, 1, 0}, {1.0, 1.0, 1, 1},
            };
            for (const Corner& c : corners) {
                const MeshVertex& expected = mesh.at(pr + c.rowOff, pc + c.colOff);
                Vec2 pos = mesh.evalPos(pr, pc, c.u, c.v);
                Color col = mesh.evalColor(pr, pc, c.u, c.v);
                CHECK_NEAR(pos.x, expected.P.x, 1e-9);
                CHECK_NEAR(pos.y, expected.P.y, 1e-9);
                CHECK_NEAR(col.r, expected.C.r, 1e-9);
                CHECK_NEAR(col.g, expected.C.g, 1e-9);
                CHECK_NEAR(col.b, expected.C.b, 1e-9);
            }
        }
    }
}

// Paper fidelity regression: Sec. 3 says twist (muv) is "usually set to
// zero," and GradientMesh.h/geomCorner()'s header comment says Puv is
// deliberately NOT a free unknown -- it's fixed at {0,0} regardless of
// whatever happens to be stored in MeshVertex::Puv (kept only as inert
// serialization storage). An earlier pass in this project's history briefly
// promoted Puv to a free unknown "for completeness," then reverted after
// rereading the paper -- see git history and README's "Known
// simplifications." This test guards against that specific regression
// recurring silently: deliberately poison MeshVertex::Puv with a large
// nonzero value and confirm geomCorner()/evalPos() still behave as if it
// were {0,0}.
static void test_geometry_twist_stays_fixed_zero() {
    GradientMesh mesh;
    mesh.rows = 2; mesh.cols = 2;
    mesh.vertices.resize(4);
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 2; ++col) {
            MeshVertex v;
            v.P = {col * 10.0, row * 10.0};
            v.Pu = {5.0, 0.0}; v.Pv = {0.0, 5.0};
            v.Puv = {999.0, -999.0}; // poison value -- must be ignored
            mesh.vertices[mesh.idx(row, col)] = v;
        }
    }
    HermiteCorner<Vec2> hc = mesh.geomCorner(0, 0);
    CHECK_NEAR(hc.Puv.x, 0.0, 1e-12);
    CHECK_NEAR(hc.Puv.y, 0.0, 1e-12);

    // Cross-check at the patch level too: evalPos at an interior (u,v)
    // point with the poisoned mesh must equal evalPos on an otherwise
    // identical mesh whose Puv is left at {0,0} -- i.e. the poison value
    // has literally zero effect on the evaluated surface.
    GradientMesh clean = mesh;
    clean.vertices[clean.idx(0, 0)].Puv = {0, 0};
    clean.vertices[clean.idx(0, 1)].Puv = {0, 0};
    clean.vertices[clean.idx(1, 0)].Puv = {0, 0};
    clean.vertices[clean.idx(1, 1)].Puv = {0, 0};
    for (double u : {0.13, 0.5, 0.87}) {
        for (double v : {0.21, 0.5, 0.76}) {
            Vec2 a = mesh.evalPos(0, 0, u, v);
            Vec2 b = clean.evalPos(0, 0, u, v);
            CHECK_NEAR(a.x, b.x, 1e-12);
            CHECK_NEAR(a.y, b.y, 1e-12);
        }
    }
}

// Promotes the earlier ad hoc verification (~/gmtest/test_buffers.cpp, run
// once by hand during the GPU-preview work) into a permanent check: builds
// a real gmcore::GradientMesh (nonzero Pu/Pv/Cuv, so the color surface is
// genuinely bicubic, not accidentally degenerate), runs
// MeshRenderBuffers::build() on it, and confirms that reconstructing
// position/color from ONLY the packed buffer (the same corner-fetch +
// Hermite-basis-weight arithmetic GLShaderSources.h's GLSL vertex/fragment
// shaders perform) reproduces GradientMesh::evalPos/evalColor -- the REAL
// production functions, not a hand-ported copy -- to float32 precision.
// This is the CPU-side half of the GPU-preview's correctness guarantee
// (see GLShaderSources.h's header comment for the other half: a headless
// EGL/GL harness that checked the shader TEXT itself against a synthetic
// mesh); this test instead exercises the actual buffer-packing code
// against the actual mesh-evaluation code, every time core/ changes.
static void test_mesh_render_buffers_matches_eval() {
    GradientMesh mesh;
    mesh.rows = 4; mesh.cols = 5;
    mesh.vertices.resize(size_t(mesh.rows) * mesh.cols);
    for (int row = 0; row < mesh.rows; ++row) {
        for (int col = 0; col < mesh.cols; ++col) {
            MeshVertex v;
            v.P  = {col + 0.05 * std::sin(row + 2.0 * col), row + 0.04 * std::cos(2.0 * row + col)};
            v.Pu = {1.0 + 0.10 * row, 0.05 * col};
            v.Pv = {0.05 * row, 1.0 + 0.10 * col};
            v.C  = {0.30 + 0.10 * row + 0.05 * col, 0.20 + 0.07 * col - 0.03 * row, 0.50 - 0.03 * row * col};
            v.Cu = {0.15 - 0.02 * row, 0.05 + 0.01 * col, -0.10 + 0.02 * row};
            v.Cv = {-0.05 + 0.01 * col, 0.12 - 0.02 * row, 0.08 + 0.01 * col};
            v.Cuv = {0.20, -0.15, 0.10};
            mesh.vertices[mesh.idx(row, col)] = v;
        }
    }

    const int n = 6;
    MeshRenderBuffers buf = MeshRenderBuffers::build(mesh, n);
    CHECK(buf.vertexData.size() == size_t(mesh.rows) * mesh.cols * 18);
    CHECK(buf.rows == mesh.rows && buf.cols == mesh.cols);
    CHECK(buf.patchRows == mesh.rows - 1 && buf.patchCols == mesh.cols - 1);
    CHECK(buf.samplesPerEdge == n);
    CHECK(buf.uvTemplate.size() == size_t(n + 1) * (n + 1) * 2);
    CHECK(buf.indices.size() == size_t(n) * n * 6);

    auto hbasis = [](double t, double out[4]) {
        double t2 = t * t, t3 = t2 * t;
        out[0] = 2 * t3 - 3 * t2 + 1; out[1] = -2 * t3 + 3 * t2;
        out[2] = t3 - 2 * t2 + t;     out[3] = t3 - t2;
    };
    auto reconstructPos = [&](int pr, int pc, double u, double v) -> Vec2 {
        double Hu[4], Hv[4]; hbasis(u, Hu); hbasis(v, Hv);
        double Hu0[2] = {Hu[0], Hu[1]}, Hu1[2] = {Hu[2], Hu[3]}, Hv0[2] = {Hv[0], Hv[1]}, Hv1[2] = {Hv[2], Hv[3]};
        Vec2 pos{0, 0};
        for (int a = 0; a < 2; ++a) for (int b = 0; b < 2; ++b) {
            int row = pr + b, col = pc + a;
            const float* p = &buf.vertexData[size_t(mesh.idx(row, col)) * 18];
            Vec2 P{p[0], p[1]}, Pu{p[2], p[3]}, Pv{p[4], p[5]};
            pos = pos + P * (Hu0[a] * Hv0[b]) + Pu * (Hu1[a] * Hv0[b]) + Pv * (Hu0[a] * Hv1[b]);
        }
        return pos;
    };
    auto reconstructColor = [&](int pr, int pc, double u, double v) -> Color {
        double Hu[4], Hv[4]; hbasis(u, Hu); hbasis(v, Hv);
        double Hu0[2] = {Hu[0], Hu[1]}, Hu1[2] = {Hu[2], Hu[3]}, Hv0[2] = {Hv[0], Hv[1]}, Hv1[2] = {Hv[2], Hv[3]};
        Color c{0, 0, 0};
        for (int a = 0; a < 2; ++a) for (int b = 0; b < 2; ++b) {
            int row = pr + b, col = pc + a;
            const float* p = &buf.vertexData[size_t(mesh.idx(row, col)) * 18];
            Color C{p[6], p[7], p[8]}, Cu{p[9], p[10], p[11]}, Cv{p[12], p[13], p[14]}, Cuv{p[15], p[16], p[17]};
            c = c + C * (Hu0[a] * Hv0[b]) + Cu * (Hu1[a] * Hv0[b]) + Cv * (Hu0[a] * Hv1[b]) + Cuv * (Hu1[a] * Hv1[b]);
        }
        return c;
    };

    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            for (int i = 0; i <= n; ++i) {
                double v = double(i) / n;
                for (int j = 0; j <= n; ++j) {
                    double u = double(j) / n;
                    Vec2 refPos = mesh.evalPos(pr, pc, u, v);
                    Color refCol = mesh.evalColor(pr, pc, u, v);
                    Vec2 gotPos = reconstructPos(pr, pc, u, v);
                    Color gotCol = reconstructColor(pr, pc, u, v);
                    CHECK_NEAR(gotPos.x, refPos.x, 1e-4);
                    CHECK_NEAR(gotPos.y, refPos.y, 1e-4);
                    CHECK_NEAR(gotCol.r, refCol.r, 1e-4);
                    CHECK_NEAR(gotCol.g, refCol.g, 1e-4);
                    CHECK_NEAR(gotCol.b, refCol.b, 1e-4);
                }
            }
        }
    }
}

// SparseBlockSolver.h's block-sparse PCG (solveSPD_PCG) against an
// independent DENSE reference solve (smallMatInverse on the same system,
// materialized densely) for a small, well-conditioned block-tridiagonal
// SPD system -- exercises SparseBlockMatrix's block assembly
// (addBlock/reserveBlocks), multiply(), and blockJacobiInverse() together,
// completely independent of the mesh optimizer that's normally the only
// caller of this solver.
static void test_sparse_block_solver_matches_dense_reference() {
    const int N = 2;   // block size
    const int nb = 3;  // number of blocks -> 6x6 dense system

    SparseBlockMatrix H;
    H.init(N, nb);
    H.reserveBlocks({{0, 0}, {1, 1}, {2, 2}, {0, 1}, {1, 2}});
    H.addBlock(0, 0, {4, 0, 0, 4});
    H.addBlock(1, 1, {4, 0, 0, 4});
    H.addBlock(2, 2, {4, 0, 0, 4});
    H.addBlock(0, 1, {-1, 0, 0, -1});
    H.addBlock(1, 2, {-1, 0, 0, -1});

    const std::vector<double> b = {1, 2, 3, 4, 5, 6};

    // Independent dense reference: same system as one flat 6x6 SPD matrix,
    // inverted via SparseBlockSolver.h's own smallMatInverse (a completely
    // separate code path from solveSPD_PCG/SparseBlockMatrix -- Gauss-
    // Jordan elimination vs. block-Jacobi-preconditioned CG).
    std::vector<double> dense(36, 0.0);
    auto set = [&](int i, int j, double val) { dense[i * 6 + j] = val; dense[j * 6 + i] = val; };
    dense[0 * 6 + 0] = 4; dense[1 * 6 + 1] = 4;
    dense[2 * 6 + 2] = 4; dense[3 * 6 + 3] = 4;
    dense[4 * 6 + 4] = 4; dense[5 * 6 + 5] = 4;
    set(0, 2, -1); set(1, 3, -1); set(2, 4, -1); set(3, 5, -1);
    std::vector<double> denseInv = smallMatInverse(dense, 6);
    std::vector<double> expected(6, 0.0);
    smallMatMulVec(denseInv, 6, b.data(), expected.data());

    std::vector<double> x0(6, 0.0);
    std::vector<double> got = solveSPD_PCG(H, b, x0, /*maxIter=*/200, /*relTol=*/1e-12);

    CHECK(got.size() == 6);
    for (int i = 0; i < 6; ++i) CHECK_NEAR(got[i], expected[i], 1e-6);
}

// Regression for the "reconstruction preview looks washed out/mirrored"
// class of bugs this project has hit repeatedly (two separate vertical-
// flip bugs, see README) -- pins down Image's row-0-at-top addressing
// convention directly at the data-structure level, independent of any
// AppKit/CoreGraphics loading code (which is exactly where those bugs
// actually lived, and exactly what this sandbox cannot compile/test --
// this at least locks down the CONVENTION every core/ consumer must
// agree on).
static void test_image_row_convention_and_bilinear() {
    Image img(2, 2);
    img.set(0, 0, {1, 0, 0}); // top-left: red
    img.set(1, 0, {0, 1, 0}); // top-right: green
    img.set(0, 1, {0, 0, 1}); // bottom-left: blue
    img.set(1, 1, {1, 1, 1}); // bottom-right: white

    CHECK_NEAR(img.at(0, 0).r, 1.0, 1e-12);
    CHECK_NEAR(img.at(1, 0).g, 1.0, 1e-12);
    CHECK_NEAR(img.at(0, 1).b, 1.0, 1e-12);
    CHECK_NEAR(img.at(1, 1).r, 1.0, 1e-12);
    CHECK_NEAR(img.at(1, 1).g, 1.0, 1e-12);
    CHECK_NEAR(img.at(1, 1).b, 1.0, 1e-12);

    // Bilinear sample exactly at the shared center of all 4 pixels should
    // be their average, regardless of which "direction" is which -- a
    // sanity check that sampleBilinear's addressing matches at()/set()'s.
    Color center = img.sampleBilinear(1.0, 1.0);
    CHECK_NEAR(center.r, (1.0 + 0.0 + 0.0 + 1.0) / 4.0, 1e-9);
    CHECK_NEAR(center.g, (0.0 + 1.0 + 0.0 + 1.0) / 4.0, 1e-9);
    CHECK_NEAR(center.b, (0.0 + 0.0 + 1.0 + 1.0) / 4.0, 1e-9);

    // downsampled2x on a 4x4 image with a single bright marker pixel at
    // the top-left corner should keep that marker near the top-left of
    // the 2x2 result (blurred, not moved to the opposite corner) -- would
    // catch a transposed/flipped pyramid downsample.
    Image big(4, 4);
    for (int y = 0; y < 4; ++y) for (int x = 0; x < 4; ++x) big.set(x, y, {0, 0, 0});
    big.set(0, 0, {1, 1, 1});
    Image small = big.downsampled2x();
    CHECK(small.width == 2 && small.height == 2);
    double topLeft = small.at(0, 0).r;
    double bottomRight = small.at(1, 1).r;
    CHECK(topLeft > bottomRight);
}

// Basic optimizer sanity + two specific fixed-bug regressions:
//  (1) RMSE actually decreases (the solver isn't a no-op / doesn't diverge)
//      on a simple, deterministic synthetic image.
//  (2) The 4 mesh corners never move (see README "Fixed: the 4 mesh
//      corners are now hard-fixed (never move)") -- a real bug this
//      project hit and fixed; corners must equal their pre-optimize
//      positions exactly, bit for bit, since the code path that hard-fixes
//      them should skip touching P entirely rather than move-then-restore.
static void test_optimizer_reduces_rmse_and_keeps_corners_fixed() {
    const int W = 64, H = 64;
    Image target(W, H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            // A simple diagonal-ish gradient plus a hard vertical split at
            // the midline -- enough structure for geometry/color to have
            // real signal to fit, deterministic, no RNG needed.
            double u = double(x) / (W - 1), v = double(y) / (H - 1);
            Color c = (x < W / 2) ? Color{0.1 + 0.3 * v, 0.2, 0.2 + 0.2 * u}
                                   : Color{0.2, 0.2 + 0.5 * v, 0.6 + 0.2 * u};
            target.set(x, y, c);
        }
    }

    std::array<BezierSpline, 4> boundary;
    boundary[0].segments = {CubicBezier{Vec2{0, 0}, Vec2{W / 3.0, 0}, Vec2{2 * W / 3.0, 0}, Vec2{double(W), 0}}};             // top
    boundary[1].segments = {CubicBezier{Vec2{double(W), 0}, Vec2{double(W), H / 3.0}, Vec2{double(W), 2 * H / 3.0}, Vec2{double(W), double(H)}}}; // right
    boundary[2].segments = {CubicBezier{Vec2{double(W), double(H)}, Vec2{2 * W / 3.0, double(H)}, Vec2{W / 3.0, double(H)}, Vec2{0, double(H)}}}; // bottom
    boundary[3].segments = {CubicBezier{Vec2{0, double(H)}, Vec2{0, 2 * H / 3.0}, Vec2{0, H / 3.0}, Vec2{0, 0}}};              // left

    GradientMesh mesh = GradientMesh::buildInitial(5, 5, boundary, target);

    const Vec2 corner00Before = mesh.at(0, 0).P;
    const Vec2 corner04Before = mesh.at(0, 4).P;
    const Vec2 corner40Before = mesh.at(4, 0).P;
    const Vec2 corner44Before = mesh.at(4, 4).P;

    double rmseBefore = mesh.reconstructionRMSE(target);

    OptimizerOptions opts; // defaults, hand-rolled path (no Ceres dependency in this test)
    opts.outerIterationsPerLevel = 12;
    MeshOptimizer::optimizeCoarseToFine(mesh, target, {}, /*numPyramidLevels=*/2, opts);

    double rmseAfter = mesh.reconstructionRMSE(target);
    CHECK(rmseAfter < rmseBefore);
    // Not just "a little better" -- this is an easy, well-structured
    // synthetic case; require a real, substantial reduction so a
    // regression that makes the optimizer barely-functional (e.g. an
    // early-exit bug like README's "real bug in the early-exit itself")
    // fails this test instead of slipping through on a technicality.
    CHECK(rmseAfter < rmseBefore * 0.7);

    CHECK_NEAR(mesh.at(0, 0).P.x, corner00Before.x, 1e-12);
    CHECK_NEAR(mesh.at(0, 0).P.y, corner00Before.y, 1e-12);
    CHECK_NEAR(mesh.at(0, 4).P.x, corner04Before.x, 1e-12);
    CHECK_NEAR(mesh.at(0, 4).P.y, corner04Before.y, 1e-12);
    CHECK_NEAR(mesh.at(4, 0).P.x, corner40Before.x, 1e-12);
    CHECK_NEAR(mesh.at(4, 0).P.y, corner40Before.y, 1e-12);
    CHECK_NEAR(mesh.at(4, 4).P.x, corner44Before.x, 1e-12);
    CHECK_NEAR(mesh.at(4, 4).P.y, corner44Before.y, 1e-12);
}

// Regression for "Bug found and fixed: boundary vertices were effectively
// frozen in place" (README) -- after optimizing, every non-corner boundary
// vertex should still lie ON (very near) its assigned CubicBezier boundary
// segment: boundaryWeight pulls it back onto the spline in the NORMAL
// direction, but it should be free to have SLID along the curve (i.e. this
// checks "stayed on the spline," not "didn't move at all" -- the bug this
// guards against is a vertex drifting off the curve entirely, not a
// vertex that moved).
static void test_boundary_vertices_stay_on_spline_after_optimize() {
    const int W = 48, H = 48;
    Image target(W, H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            target.set(x, y, Color{double(x) / W, double(y) / H, 0.5});

    std::array<BezierSpline, 4> boundary;
    boundary[0].segments = {CubicBezier{Vec2{0, 0}, Vec2{16, 0}, Vec2{32, 0}, Vec2{double(W), 0}}};
    boundary[1].segments = {CubicBezier{Vec2{double(W), 0}, Vec2{double(W), 16}, Vec2{double(W), 32}, Vec2{double(W), double(H)}}};
    boundary[2].segments = {CubicBezier{Vec2{double(W), double(H)}, Vec2{32, double(H)}, Vec2{16, double(H)}, Vec2{0, double(H)}}};
    boundary[3].segments = {CubicBezier{Vec2{0, double(H)}, Vec2{0, 32}, Vec2{0, 16}, Vec2{0, 0}}};

    GradientMesh mesh = GradientMesh::buildInitial(4, 4, boundary, target);
    OptimizerOptions opts;
    opts.outerIterationsPerLevel = 10;
    MeshOptimizer::optimizeCoarseToFine(mesh, target, {}, /*numPyramidLevels=*/2, opts);

    int boundaryVertsChecked = 0;
    for (int row = 0; row < mesh.rows; ++row) {
        for (int col = 0; col < mesh.cols; ++col) {
            const MeshVertex& v = mesh.at(row, col);
            if (!v.isBoundary) continue;
            CHECK(v.boundarySide >= 0 && v.boundarySide < 4);
            Vec2 onSpline = mesh.boundary[v.boundarySide].eval(v.boundaryT);
            double dist = (v.P - onSpline).length();
            // A few pixels' slack (the boundaryWeight pull is soft, not a
            // hard constraint), but nowhere near the 16-24px control-point
            // spacing this mesh uses -- a vertex that had drifted
            // completely off its spline (the actual historical bug) would
            // fail this by a wide margin, not a narrow one.
            CHECK(dist < 3.0);
            ++boundaryVertsChecked;
        }
    }
    CHECK(boundaryVertsChecked > 0); // sanity: buildInitial actually marks boundary vertices
}

// Smoke test for SVGExporter: not a full XML validation, but confirms the
// exporter produces the expected SVG2 mesh-gradient structural elements
// for a small real mesh, and that both the sRGB and "source is CIELUV"
// code paths run without crashing and still emit valid-looking stop
// colors (a `#RRGGBB` hex, not e.g. a NaN or an unclamped/out-of-range
// literal leaking into the output).
static void test_svg_export_smoke() {
    Image target(16, 16);
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
            target.set(x, y, Color{double(x) / 16, double(y) / 16, 0.4});

    std::array<BezierSpline, 4> boundary;
    boundary[0].segments = {CubicBezier{Vec2{0, 0}, Vec2{5, 0}, Vec2{11, 0}, Vec2{16, 0}}};
    boundary[1].segments = {CubicBezier{Vec2{16, 0}, Vec2{16, 5}, Vec2{16, 11}, Vec2{16, 16}}};
    boundary[2].segments = {CubicBezier{Vec2{16, 16}, Vec2{11, 16}, Vec2{5, 16}, Vec2{0, 16}}};
    boundary[3].segments = {CubicBezier{Vec2{0, 16}, Vec2{0, 11}, Vec2{0, 5}, Vec2{0, 0}}};
    GradientMesh mesh = GradientMesh::buildInitial(3, 3, boundary, target);

    std::string svg = exportGradientMeshSVG(mesh, 16, 16, /*sourceIsCIELUV=*/false);
    CHECK(svg.find("<svg") != std::string::npos);
    CHECK(svg.find("<meshgradient") != std::string::npos);
    CHECK(svg.find("<meshpatch") != std::string::npos);
    CHECK(svg.find("#") != std::string::npos); // at least one hex stop color present
    CHECK(svg.find("nan") == std::string::npos && svg.find("NaN") == std::string::npos && svg.find("inf") == std::string::npos);

    GradientMesh meshLUV = mesh;
    for (auto& v : meshLUV.vertices) {
        v.C = srgbToCIELUV(v.C); v.Cu = srgbToCIELUV(v.Cu); v.Cv = srgbToCIELUV(v.Cv); v.Cuv = srgbToCIELUV(v.Cuv);
    }
    std::string svgLUV = exportGradientMeshSVG(meshLUV, 16, 16, /*sourceIsCIELUV=*/true);
    CHECK(svgLUV.find("<meshgradient") != std::string::npos);
    CHECK(svgLUV.find("nan") == std::string::npos && svgLUV.find("NaN") == std::string::npos && svgLUV.find("inf") == std::string::npos);
}

// Regression for Task 3's Lazy-Snapping-style segmentation tool (see
// README's "Known simplifications" -- this replaces plain click-tracing
// as the cutout tool). Three pieces, tested independently plus one
// integration test chaining them the way the real UI will: MaxFlowGraph
// (generic Dinic's max-flow/min-cut), segmentForeground (the graph-cut
// itself), and traceOuterContour/simplifyClosedPolygon (mask -> the same
// ordered polygon the existing corner-picking + per-side Bezier fit
// already consumes).
//
// MaxFlowGraph's own textbook case below caught nothing (it was correct
// on the first try), but segmentForeground's fg/bg cluster distances were
// initially swapped (an easy mistake: "cost of label L" must be
// proportional to distance from L's OWN cluster set, not the opposite
// one -- see LazySnapping.cpp's comment at the fix site) and
// traceOuterContour's stopping criterion was initially wrong (compared
// backtrack DIRECTION to an arbitrary initial guess instead of detecting
// the actual repeated s0->s1 transition -- see ContourTracing.cpp's
// comment). Both were caught by these tests before ever reaching the UI.
static void test_maxflow_textbook_graph() {
    // Classic max-flow textbook graph (CLRS-style), known max flow = 23.
    MaxFlowGraph g(6); // 0=s, 5=t
    g.addEdge(0, 1, 16); g.addEdge(0, 2, 13);
    g.addEdge(1, 3, 12);
    g.addEdge(2, 1, 4);
    g.addEdge(3, 2, 9);
    g.addEdge(2, 4, 14);
    g.addEdge(4, 3, 7);
    g.addEdge(3, 5, 20);
    g.addEdge(4, 5, 4);
    double flow = g.maxFlow(0, 5);
    CHECK_NEAR(flow, 23.0, 1e-9);
    CHECK(g.isSourceSide(0));
    CHECK(!g.isSourceSide(5));
}

static void test_maxflow_simple_cut() {
    // s->a(5)->t(3), s->b(2)->t(10): the a->t(3) and s->b(2) edges are the
    // bottleneck, so max flow should be exactly 3+2=5.
    MaxFlowGraph g(4); // 0=s,1=a,2=b,3=t
    g.addEdge(0, 1, 5);
    g.addEdge(1, 3, 3);
    g.addEdge(0, 2, 2);
    g.addEdge(2, 3, 10);
    CHECK_NEAR(g.maxFlow(0, 3), 5.0, 1e-9);
}

static Image makeTwoColorBlockImage(int w, int h, int splitX) {
    Image img(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            img.set(x, y, x < splitX ? Color{0.9, 0.1, 0.1} : Color{0.1, 0.1, 0.9});
    return img;
}

// A pixel-perfect two-color-block image with a handful of scribbles deep
// in each block should segment essentially exactly, given the huge color
// contrast and zero texture noise -- this is as easy a case as graph-cut
// segmentation ever sees, so ANY misclassification here (beyond a couple
// of boundary-column pixels) means something structural is wrong, not
// just "the algorithm did its honest best on a hard case."
static void test_segmentation_separates_two_color_blocks() {
    const int w = 40, h = 20, splitX = 20;
    Image img = makeTwoColorBlockImage(w, h, splitX);
    SegmentationScribbles scribbles;
    for (int y = 5; y < 15; y += 2) scribbles.foreground.push_back({5, y});
    for (int y = 5; y < 15; y += 2) scribbles.background.push_back({35, y});

    std::vector<uint8_t> mask = segmentForeground(img, scribbles);
    CHECK(mask.size() == (size_t)(w * h));

    int correctLeft = 0, totalLeft = 0, correctRight = 0, totalRight = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            bool isFg = mask[y * w + x] != 0;
            if (x < splitX) { ++totalLeft; if (isFg) ++correctLeft; }
            else { ++totalRight; if (!isFg) ++correctRight; }
        }
    }
    CHECK(correctLeft >= totalLeft - 2);
    CHECK(correctRight >= totalRight - 2);
}

// No scribbles of one colour yet -> documented "not enough information"
// behavior (an all-background mask), not a crash or a guess.
static void test_segmentation_returns_empty_mask_without_both_scribble_colors() {
    Image img = makeTwoColorBlockImage(10, 10, 5);
    SegmentationScribbles onlyFg;
    onlyFg.foreground.push_back({2, 2});
    std::vector<uint8_t> mask = segmentForeground(img, onlyFg);
    CHECK(mask.size() == 100);
    for (uint8_t m : mask) CHECK(m == 0);
}

static void test_contour_tracing_filled_square() {
    const int w = 10, h = 10;
    std::vector<uint8_t> mask(w * h, 0);
    for (int y = 2; y <= 6; ++y)
        for (int x = 2; x <= 6; ++x)
            mask[y * w + x] = 1;

    std::vector<Vec2> contour = traceOuterContour(mask, w, h);
    CHECK(!contour.empty());
    // A 5x5 block's 8-connected outer boundary should be on the order of
    // 16-20 points, not e.g. the ~866 an earlier, buggy stopping
    // criterion produced by looping around the perimeter dozens of times
    // without ever detecting closure.
    CHECK(contour.size() >= 12 && contour.size() <= 32);
    for (const Vec2& p : contour) {
        CHECK(p.x >= 2 && p.x <= 6 && p.y >= 2 && p.y <= 6);
        CHECK(p.x == 2 || p.x == 6 || p.y == 2 || p.y == 6); // every point IS on the block's edge
    }

    std::vector<Vec2> simplified = simplifyClosedPolygon(contour, 0.5);
    CHECK(simplified.size() >= 4 && simplified.size() <= 6); // should collapse close to the 4 true corners
}

// Chains segmentForeground -> traceOuterContour -> simplifyClosedPolygon
// exactly the way the real UI will (see DocumentModel's planned
// -segmentBoundaryWithScribbles: -- the result feeds the SAME
// -setBoundaryPolygonPoints:/-fitBoundaryWithCornerIndices: pipeline the
// old click-tracing tool already used).
static void test_segmentation_to_contour_integration() {
    const int w = 40, h = 20, splitX = 20;
    Image img = makeTwoColorBlockImage(w, h, splitX);
    SegmentationScribbles scribbles;
    for (int y = 5; y < 15; y += 2) scribbles.foreground.push_back({5, y});
    for (int y = 5; y < 15; y += 2) scribbles.background.push_back({35, y});

    std::vector<uint8_t> mask = segmentForeground(img, scribbles);
    std::vector<Vec2> contour = traceOuterContour(mask, w, h);
    CHECK(contour.size() >= 4);

    double minX = 1e9, maxX = -1e9;
    for (const Vec2& p : contour) { minX = std::min(minX, p.x); maxX = std::max(maxX, p.x); }
    CHECK(minX <= 1.0);
    CHECK(maxX >= splitX - 2 && maxX <= splitX + 2);

    std::vector<Vec2> simplified = simplifyClosedPolygon(contour, 1.0);
    CHECK(simplified.size() >= 4 && simplified.size() <= 10);
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
int main() {
    struct TestCase { const char* name; std::function<void()> fn; };
    const std::vector<TestCase> tests = {
        {"bezier_endpoints_and_closest", test_bezier_endpoints_and_closest},
        {"bezier_fit_straight_line", test_bezier_fit_straight_line},
        {"bezier_spline_no_split_for_smooth_input", test_bezier_spline_no_split_for_smooth_input},
        {"bezier_spline_splits_for_sharp_zigzag", test_bezier_spline_splits_for_sharp_zigzag},
        {"bezier_spline_closest_t_roundtrip", test_bezier_spline_closest_t_roundtrip},
        {"cieluv_roundtrip", test_cieluv_roundtrip},
        {"mesh_hermite_corner_exactness", test_mesh_hermite_corner_exactness},
        {"geometry_twist_stays_fixed_zero", test_geometry_twist_stays_fixed_zero},
        {"mesh_render_buffers_matches_eval", test_mesh_render_buffers_matches_eval},
        {"sparse_block_solver_matches_dense_reference", test_sparse_block_solver_matches_dense_reference},
        {"image_row_convention_and_bilinear", test_image_row_convention_and_bilinear},
        {"optimizer_reduces_rmse_and_keeps_corners_fixed", test_optimizer_reduces_rmse_and_keeps_corners_fixed},
        {"boundary_vertices_stay_on_spline_after_optimize", test_boundary_vertices_stay_on_spline_after_optimize},
        {"svg_export_smoke", test_svg_export_smoke},
        {"maxflow_textbook_graph", test_maxflow_textbook_graph},
        {"maxflow_simple_cut", test_maxflow_simple_cut},
        {"segmentation_separates_two_color_blocks", test_segmentation_separates_two_color_blocks},
        {"segmentation_returns_empty_mask_without_both_scribble_colors", test_segmentation_returns_empty_mask_without_both_scribble_colors},
        {"contour_tracing_filled_square", test_contour_tracing_filled_square},
        {"segmentation_to_contour_integration", test_segmentation_to_contour_integration},
    };

    int passed = 0, failed = 0;
    for (const TestCase& t : tests) {
        g_checksThisTest = 0;
        g_failuresThisTest = 0;
        std::printf("RUN  %s\n", t.name);
        t.fn();
        if (g_failuresThisTest == 0) {
            std::printf("PASS %s (%d checks)\n", t.name, g_checksThisTest);
            ++passed;
        } else {
            std::printf("FAIL %s (%d/%d checks failed)\n", t.name, g_failuresThisTest, g_checksThisTest);
            ++failed;
        }
    }

    std::printf("\n%d/%zu test cases passed (%d/%d individual checks passed)\n",
                passed, tests.size(), g_totalChecks - g_totalFailures, g_totalChecks);
    if (failed > 0) {
        std::printf("%d test case(s) FAILED\n", failed);
        return 1;
    }
    std::printf("ALL TESTS PASSED\n");
    return 0;
}
