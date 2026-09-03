// gmesh_cli — command-line test/reference harness for gmcore.
//
// This is the "automatic, no manual markup" mode the interactive macOS
// app also offers as a shortcut: it lays a plain rectangular boundary
// (with a small inset margin) and a regular initial grid over the whole
// input image, then runs the same coarse-to-fine optimizer the app uses.
// It exists so the core algorithm can be built, run and sanity-checked
// without any GUI (this Linux sandbox has no Xcode/AppKit) -- and it
// doubles as a handy regression tool on macOS too.
#include "gmcore/Image.h"
#include "gmcore/GradientMesh.h"
#include "gmcore/MeshOptimizer.h"
#include "gmcore/SVGExporter.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <array>
#include <cmath>
#include <random>

using namespace gmcore;

// Procedurally generates a shaded-sphere-like test image (smooth color
// gradients + a soft specular highlight + a little texture noise) so the
// optimizer can be exercised with zero external inputs.
static Image makeSyntheticTestImage(int w, int h) {
    Image img(w, h);
    double cx = w * 0.5, cy = h * 0.5, R = std::min(w, h) * 0.42;
    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> noise(-0.02, 0.02);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double dx = (x - cx) / R, dy = (y - cy) / R;
            double d2 = dx * dx + dy * dy;
            Color c;
            if (d2 <= 1.0) {
                double z = std::sqrt(std::max(0.0, 1.0 - d2));
                Vec2 lightDir{-0.5, -0.6};
                double diff = std::max(0.0, 0.6 - dx * lightDir.x - dy * lightDir.y + 0.4 * z);
                double spec = std::pow(std::max(0.0, z - 0.15 * (dx * dx + dy * dy)), 12.0);
                c.r = 0.15 + 0.55 * diff + 0.9 * spec;
                c.g = 0.10 + 0.35 * diff + 0.9 * spec;
                c.b = 0.55 + 0.40 * diff + 0.9 * spec;
            } else {
                double t = std::min(1.0, (y / (double)h));
                c.r = 0.92 - 0.10 * t;
                c.g = 0.92 - 0.06 * t;
                c.b = 0.88 - 0.02 * t;
            }
            c.r += noise(rng); c.g += noise(rng); c.b += noise(rng);
            img.set(x, y, c.clamped01());
        }
    }
    return img;
}

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s [--input path.ppm|.png] [--out-prefix name] [--rows N] [--cols N]\n"
        "          [--pyramid-levels N] [--margin px] [--width W --height H]\n"
        "          [--smooth-geom W] [--smooth-color W] [--color-ridge W] [--boundary-weight W]\n"
        "          [--tangent-prior W] [--edge-gain W] [--edge-min-factor W] [--vline \"x,y;x,y;...\"]\n"
        "          [--vector-line-weight W]\n"
        "          [--outer-iters N] [--outer-conv-tol X] [--outer-conv-patience N]\n"
        "          [--pyramid-restarts N]\n"
        "          [--gn-iters N] [--samples N] [--cg-iters N] [--use-ceres]\n"
        "          [--use-ceres-joint] [--ceres-threads N]\n"
        "  --use-ceres: replace the hand-rolled geometry Gauss-Newton solver with a\n"
        "  ceres::Problem solve (OptimizerOptions::useCeresGeometry) -- only has any\n"
        "  effect if this binary was built with Ceres found (see CMakeLists.txt); a\n"
        "  one-time warning is printed and the hand-rolled path is used otherwise.\n"
        "  --use-ceres-joint: replace BOTH the closed-form color solve and the geometry\n"
        "  solve with one fully-joint ceres::Problem (OptimizerOptions::useCeresJoint) --\n"
        "  wins over --use-ceres if both are given. Same graceful fallback if built\n"
        "  without Ceres.\n"
        "  --ceres-threads N: ceres::Solver::Options::num_threads for every Ceres solve\n"
        "  (OptimizerOptions::ceresNumThreads). 0 (default) = auto, every core; 1 forces\n"
        "  single-threaded -- useful for A/B-testing whether multithreading itself\n"
        "  changed a result (it shouldn't, beyond floating-point summation-order noise;\n"
        "  see that field's comment). No effect without --use-ceres/--use-ceres-joint.\n"
        "  With no --input, a synthetic shaded-sphere test image is generated so the\n"
        "  optimizer can be exercised without any external files.\n"
        "  The --smooth-geom/... flags override OptimizerOptions for quick experiments\n"
        "  without recompiling; also writes <out-prefix>_mesh_points.csv (row,col,x,y)\n"
        "  for visualizing/overlaying the control-point grid.\n",
        prog);
}

int main(int argc, char** argv) {
    std::string inputPath, outPrefix = "gmesh_out";
    int rows = 9, cols = 9, pyramidLevels = 4, margin = 6, synthW = 220, synthH = 220;
    OptimizerOptions opts; // defaults; may be overridden below for experimentation
    std::vector<VectorLine> vectorLines; // populated by --vline (testing/debugging aid, not in printUsage yet)

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() { return std::string(argv[++i]); };
        if (a == "--input") inputPath = next();
        else if (a == "--vline") {
            // "x0,y0;x1,y1;..." -- one guide polyline, for testing the
            // vector-line constraint (Sec 4.2) without the GUI.
            std::string spec = next();
            VectorLine line;
            size_t pos = 0;
            while (pos < spec.size()) {
                size_t semi = spec.find(';', pos);
                std::string pair = spec.substr(pos, semi == std::string::npos ? std::string::npos : semi - pos);
                size_t comma = pair.find(',');
                if (comma != std::string::npos) {
                    double x = std::stod(pair.substr(0, comma));
                    double y = std::stod(pair.substr(comma + 1));
                    line.points.push_back({x, y});
                }
                if (semi == std::string::npos) break;
                pos = semi + 1;
            }
            if (line.points.size() >= 2) vectorLines.push_back(std::move(line));
        }
        else if (a == "--out-prefix") outPrefix = next();
        else if (a == "--rows") rows = std::stoi(next());
        else if (a == "--cols") cols = std::stoi(next());
        else if (a == "--pyramid-levels") pyramidLevels = std::stoi(next());
        else if (a == "--margin") margin = std::stoi(next());
        else if (a == "--width") synthW = std::stoi(next());
        else if (a == "--height") synthH = std::stoi(next());
        else if (a == "--smooth-geom") opts.smoothWeightGeom = std::stod(next());
        else if (a == "--smooth-color") opts.smoothWeightColor = std::stod(next());
        else if (a == "--color-ridge") opts.colorDerivRidge = std::stod(next());
        else if (a == "--boundary-weight") opts.boundaryWeight = std::stod(next());
        else if (a == "--vector-line-weight") opts.vectorLineWeight = std::stod(next());
        else if (a == "--tangent-prior") opts.geomTangentPriorWeight = std::stod(next());
        else if (a == "--edge-gain") opts.smoothGeomEdgeGain = std::stod(next());
        else if (a == "--edge-min-factor") opts.smoothGeomMinFactor = std::stod(next());
        else if (a == "--outer-iters") opts.outerIterationsPerLevel = std::stoi(next());
        else if (a == "--outer-conv-tol") opts.outerConvergenceRelTol = std::stod(next());
        else if (a == "--outer-conv-patience") opts.outerConvergencePatience = std::stoi(next());
        else if (a == "--pyramid-restarts") opts.pyramidRestarts = std::stoi(next());
        else if (a == "--gn-iters") opts.geomGaussNewtonItersPerOuter = std::stoi(next());
        else if (a == "--samples") opts.samplesPerPatchEdge = std::stoi(next());
        else if (a == "--cg-iters") opts.cgMaxIterations = std::stoi(next());
        else if (a == "--use-ceres") opts.useCeresGeometry = true;
        else if (a == "--use-ceres-joint") opts.useCeresJoint = true;
        else if (a == "--ceres-threads") opts.ceresNumThreads = std::stoi(next());
        else if (a == "-h" || a == "--help") { printUsage(argv[0]); return 0; }
        else { std::fprintf(stderr, "Unknown arg: %s\n", a.c_str()); printUsage(argv[0]); return 1; }
    }

    Image target;
    if (!inputPath.empty()) {
        target = Image::load(inputPath);
        std::printf("Loaded %s (%dx%d)\n", inputPath.c_str(), target.width, target.height);
    } else {
        target = makeSyntheticTestImage(synthW, synthH);
        target.savePPM(outPrefix + "_synthetic_input.ppm");
        std::printf("No --input given: generated synthetic %dx%d test image -> %s\n", synthW, synthH,
                    (outPrefix + "_synthetic_input.ppm").c_str());
    }

    double x0 = margin, y0 = margin, x1 = target.width - margin, y1 = target.height - margin;
    std::array<CubicBezier, 4> boundary;
    // top: (x0,y0)->(x1,y0)
    boundary[0] = {Vec2{x0, y0}, Vec2{x0 + (x1 - x0) / 3, y0}, Vec2{x0 + 2 * (x1 - x0) / 3, y0}, Vec2{x1, y0}};
    // right: (x1,y0)->(x1,y1)
    boundary[1] = {Vec2{x1, y0}, Vec2{x1, y0 + (y1 - y0) / 3}, Vec2{x1, y0 + 2 * (y1 - y0) / 3}, Vec2{x1, y1}};
    // bottom: (x1,y1)->(x0,y1)
    boundary[2] = {Vec2{x1, y1}, Vec2{x0 + 2 * (x1 - x0) / 3, y1}, Vec2{x0 + (x1 - x0) / 3, y1}, Vec2{x0, y1}};
    // left: (x0,y1)->(x0,y0)
    boundary[3] = {Vec2{x0, y1}, Vec2{x0, y0 + 2 * (y1 - y0) / 3}, Vec2{x0, y0 + (y1 - y0) / 3}, Vec2{x0, y0}};

    GradientMesh mesh = GradientMesh::buildInitial(rows, cols, boundary, target);
    double rmseBefore = mesh.reconstructionRMSE(target, 6);
    std::printf("Initial mesh %dx%d control points, RMSE=%.5f\n", rows, cols, rmseBefore);

    MeshOptimizer::optimizeCoarseToFine(mesh, target, vectorLines, pyramidLevels, opts,
        [](const OptimizerProgress& p) {
            std::printf("  [level %d/%d] iter %d/%d  RMSE=%.5f\n", p.pyramidLevel, p.totalPyramidLevels - 1,
                        p.outerIteration, p.totalOuterIterations - 1, p.rmse);
        });

    double rmseAfter = mesh.reconstructionRMSE(target, 6);
    std::printf("Final RMSE=%.5f (%.1f%% reduction)\n", rmseAfter, 100.0 * (1.0 - rmseAfter / std::max(1e-9, rmseBefore)));

    Image recon = mesh.render(target.width, target.height, 8);
    recon.savePPM(outPrefix + "_reconstruction.ppm");
    std::string svg = exportGradientMeshSVG(mesh, target.width, target.height);
    FILE* f = std::fopen((outPrefix + "_mesh.svg").c_str(), "w");
    if (f) { std::fwrite(svg.data(), 1, svg.size(), f); std::fclose(f); }

    std::printf("Wrote %s_reconstruction.ppm and %s_mesh.svg\n", outPrefix.c_str(), outPrefix.c_str());

    // Diagnostic dump of control-point positions AND free tangents (for
    // overlay visualization / inspecting whether Pu,Pv actually moved away
    // from their initial finite-difference seed).
    FILE* mf = std::fopen((outPrefix + "_mesh_points.csv").c_str(), "w");
    if (mf) {
        std::fprintf(mf, "rows,%d,cols,%d\n", mesh.rows, mesh.cols);
        std::fprintf(mf, "row,col,x,y,pu_x,pu_y,pv_x,pv_y,puv_x,puv_y\n");
        for (int r = 0; r < mesh.rows; ++r)
            for (int c = 0; c < mesh.cols; ++c) {
                const MeshVertex& mv = mesh.at(r, c);
                std::fprintf(mf, "%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", r, c, mv.P.x, mv.P.y,
                             mv.Pu.x, mv.Pu.y, mv.Pv.x, mv.Pv.y, mv.Puv.x, mv.Puv.y);
            }
        std::fclose(mf);
    }
    return 0;
}
