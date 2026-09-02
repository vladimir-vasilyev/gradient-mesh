#include "gmcore/GradientMesh.h"
#include <algorithm>
#include <cmath>

namespace gmcore {

static Vec2 clampIndexDiff(const std::vector<MeshVertex>& v, int idxHi, int idxLo, double scale) {
    return (v[idxHi].P - v[idxLo].P) * scale;
}

Vec2 GradientMesh::tangentU(int row, int col) const {
    if (cols < 2) return {0, 0};
    if (col == 0) return (at(row, 1).P - at(row, 0).P);
    if (col == cols - 1) return (at(row, cols - 1).P - at(row, cols - 2).P);
    return (at(row, col + 1).P - at(row, col - 1).P) * 0.5;
}

Vec2 GradientMesh::tangentV(int row, int col) const {
    if (rows < 2) return {0, 0};
    if (row == 0) return (at(1, col).P - at(0, col).P);
    if (row == rows - 1) return (at(rows - 1, col).P - at(rows - 2, col).P);
    return (at(row + 1, col).P - at(row - 1, col).P) * 0.5;
}

Vec2 GradientMesh::twist(int row, int col) const {
    if (row == 0 || row == rows - 1 || col == 0 || col == cols - 1) return {0, 0};
    Vec2 a = at(row + 1, col + 1).P, b = at(row + 1, col - 1).P;
    Vec2 c = at(row - 1, col + 1).P, d = at(row - 1, col - 1).P;
    return (a - b - c + d) * 0.25;
}

HermiteCorner<Vec2> GradientMesh::geomCorner(int row, int col) const {
    HermiteCorner<Vec2> hc;
    const MeshVertex& v = at(row, col);
    hc.P = v.P;
    hc.Pu = v.Pu;         // free unknown (see MeshVertex comment)
    hc.Pv = v.Pv;         // free unknown
    hc.Puv = Vec2{0, 0};  // fixed at zero, per the paper's Sec 3: "In
                          // practice, the values of muv are usually set to
                          // zero" -- NOT read from v.Puv (that field is
                          // inert storage; see GradientMesh.h)
    return hc;
}

HermiteCorner<Color> GradientMesh::colorCorner(int row, int col) const {
    const MeshVertex& v = at(row, col);
    HermiteCorner<Color> hc;
    hc.P = v.C; hc.Pu = v.Cu; hc.Pv = v.Cv; hc.Puv = v.Cuv;
    return hc;
}

Vec2 GradientMesh::evalPos(int patchRow, int patchCol, double u, double v, Vec2* dU, Vec2* dV) const {
    HermiteCorner<Vec2> corners[2][2];
    for (int a = 0; a < 2; ++a)
        for (int b = 0; b < 2; ++b)
            corners[a][b] = geomCorner(patchRow + b, patchCol + a);
    return evalHermitePatch<Vec2>(corners, u, v, dU, dV);
}

Color GradientMesh::evalColor(int patchRow, int patchCol, double u, double v) const {
    HermiteCorner<Color> corners[2][2];
    for (int a = 0; a < 2; ++a)
        for (int b = 0; b < 2; ++b)
            corners[a][b] = colorCorner(patchRow + b, patchCol + a);
    return evalHermitePatch<Color>(corners, u, v);
}

GradientMesh GradientMesh::buildInitial(int rows, int cols, const std::array<CubicBezier, 4>& boundary,
                                         const Image& target) {
    GradientMesh mesh;
    mesh.rows = rows; mesh.cols = cols; mesh.boundary = boundary;
    mesh.vertices.resize(size_t(rows) * cols);

    const CubicBezier& top = boundary[0];
    const CubicBezier& right = boundary[1];
    const CubicBezier& bottom = boundary[2];
    const CubicBezier& left = boundary[3];

    Vec2 P00 = top.p0, P10 = top.p3, P11 = right.p3, P01 = bottom.p3;

    for (int r = 0; r < rows; ++r) {
        double v = double(r) / (rows - 1);
        for (int c = 0; c < cols; ++c) {
            double u = double(c) / (cols - 1);
            Vec2 C0u = top.eval(u);
            Vec2 C1u = bottom.eval(1.0 - u);
            Vec2 D0v = left.eval(1.0 - v);
            Vec2 D1v = right.eval(v);
            Vec2 S = C0u * (1 - v) + C1u * v + D0v * (1 - u) + D1v * u
                     - (P00 * ((1 - u) * (1 - v)) + P10 * (u * (1 - v)) + P01 * ((1 - u) * v) + P11 * (u * v));

            MeshVertex mv;
            mv.P = S;
            mv.C = target.sampleBilinear(S.x, S.y);
            if (r == 0) { mv.isBoundary = true; mv.boundarySide = 0; mv.boundaryT = u; }
            else if (c == cols - 1) { mv.isBoundary = true; mv.boundarySide = 1; mv.boundaryT = v; }
            else if (r == rows - 1) { mv.isBoundary = true; mv.boundarySide = 2; mv.boundaryT = 1.0 - u; }
            else if (c == 0) { mv.isBoundary = true; mv.boundarySide = 3; mv.boundaryT = 1.0 - v; }
            mesh.vertices[mesh.idx(r, c)] = mv;
        }
    }
    // Second pass: initialize color derivatives from neighboring sampled colors.
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            MeshVertex& mv = mesh.at(r, c);
            Color cu = (c == 0) ? (mesh.at(r, 1).C - mesh.at(r, 0).C)
                     : (c == cols - 1) ? (mesh.at(r, cols - 1).C - mesh.at(r, cols - 2).C)
                     : (mesh.at(r, c + 1).C - mesh.at(r, c - 1).C) * 0.5;
            Color cv = (r == 0) ? (mesh.at(1, c).C - mesh.at(0, c).C)
                     : (r == rows - 1) ? (mesh.at(rows - 1, c).C - mesh.at(rows - 2, c).C)
                     : (mesh.at(r + 1, c).C - mesh.at(r - 1, c).C) * 0.5;
            mv.Cu = cu; mv.Cv = cv; mv.Cuv = Color{0, 0, 0};
        }
    }
    // Third pass: seed the free geometry tangents Pu/Pv from the
    // position-implied finite-difference estimate (all positions are
    // already final at this point, so this can run in any order relative
    // to the color pass above). The optimizer is then free to move them
    // away from this starting value. Puv is left at its default {0,0} and
    // stays there -- it's fixed, not seeded/optimized (see GradientMesh.h).
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            MeshVertex& mv = mesh.at(r, c);
            mv.Pu = mesh.tangentU(r, c);
            mv.Pv = mesh.tangentV(r, c);
        }
    }
    return mesh;
}

void GradientMesh::scalePositions(double sx, double sy) {
    // Pu/Pv are position-derivative-scale quantities (pixels per unit
    // parametric u/v, over a patch always spanning u,v in [0,1]) -- they
    // must scale linearly with position when the mesh moves between
    // pyramid levels, exactly like P itself, or the surface would come out
    // badly distorted at the next resolution. Puv is fixed at {0,0} and
    // has no free value to rescale, so it's left untouched here (scaling
    // zero by anything is still zero, but there's no meaningful reason to
    // even touch the inert field).
    for (auto& v : vertices) {
        v.P.x *= sx; v.P.y *= sy;
        v.Pu.x *= sx; v.Pu.y *= sy;
        v.Pv.x *= sx; v.Pv.y *= sy;
    }
    for (auto& b : boundary) {
        b.p0.x *= sx; b.p0.y *= sy; b.p1.x *= sx; b.p1.y *= sy;
        b.p2.x *= sx; b.p2.y *= sy; b.p3.x *= sx; b.p3.y *= sy;
    }
}

// ---------------- rasterization ----------------

static void rasterizeTriangle(Image& img, Vec2 p0, Vec2 p1, Vec2 p2, Color c0, Color c1, Color c2) {
    double area = (p1 - p0).cross(p2 - p0);
    if (std::abs(area) < 1e-9) return;
    int minX = std::max(0, (int)std::floor(std::min({p0.x, p1.x, p2.x})));
    int maxX = std::min(img.width - 1, (int)std::ceil(std::max({p0.x, p1.x, p2.x})));
    int minY = std::max(0, (int)std::floor(std::min({p0.y, p1.y, p2.y})));
    int maxY = std::min(img.height - 1, (int)std::ceil(std::max({p0.y, p1.y, p2.y})));
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            Vec2 p{x + 0.5, y + 0.5};
            double w0 = (p1 - p0).cross(p - p0);
            double w1 = (p2 - p1).cross(p - p1);
            double w2 = (p0 - p2).cross(p - p2);
            bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;
            double b0 = w1 / area, b1 = w2 / area, b2 = w0 / area;
            Color c = c0 * b0 + c1 * b1 + c2 * b2;
            img.set(x, y, c);
        }
    }
}

Image GradientMesh::render(int outW, int outH, int samplesPerPatchEdge) const {
    Image img(outW, outH);
    int n = std::max(2, samplesPerPatchEdge);
    for (int pr = 0; pr < rows - 1; ++pr) {
        for (int pc = 0; pc < cols - 1; ++pc) {
            std::vector<Vec2> pos((n + 1) * (n + 1));
            std::vector<Color> col((n + 1) * (n + 1));
            for (int i = 0; i <= n; ++i) {
                double v = double(i) / n;
                for (int j = 0; j <= n; ++j) {
                    double u = double(j) / n;
                    pos[i * (n + 1) + j] = evalPos(pr, pc, u, v);
                    col[i * (n + 1) + j] = evalColor(pr, pc, u, v);
                }
            }
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    int i00 = i * (n + 1) + j, i10 = i * (n + 1) + j + 1;
                    int i01 = (i + 1) * (n + 1) + j, i11 = (i + 1) * (n + 1) + j + 1;
                    rasterizeTriangle(img, pos[i00], pos[i10], pos[i11], col[i00], col[i10], col[i11]);
                    rasterizeTriangle(img, pos[i00], pos[i11], pos[i01], col[i00], col[i11], col[i01]);
                }
            }
        }
    }
    return img;
}

double GradientMesh::reconstructionRMSE(const Image& target, int samplesPerPatchEdge) const {
    double sumSq = 0; long count = 0;
    int n = std::max(2, samplesPerPatchEdge);
    for (int pr = 0; pr < rows - 1; ++pr) {
        for (int pc = 0; pc < cols - 1; ++pc) {
            for (int i = 0; i <= n; ++i) {
                double v = double(i) / n;
                for (int j = 0; j <= n; ++j) {
                    double u = double(j) / n;
                    Vec2 pos = evalPos(pr, pc, u, v);
                    Color c = evalColor(pr, pc, u, v);
                    Color t = target.sampleBilinear(pos.x, pos.y);
                    Color d = c - t;
                    sumSq += d.lengthSq();
                    ++count;
                }
            }
        }
    }
    if (count == 0) return 0;
    return std::sqrt(sumSq / (count * 3));
}

double GradientMesh::reconstructionMAE(const Image& target, int samplesPerPatchEdge) const {
    double sumAbs = 0; long count = 0;
    int n = std::max(2, samplesPerPatchEdge);
    for (int pr = 0; pr < rows - 1; ++pr) {
        for (int pc = 0; pc < cols - 1; ++pc) {
            for (int i = 0; i <= n; ++i) {
                double v = double(i) / n;
                for (int j = 0; j <= n; ++j) {
                    double u = double(j) / n;
                    Vec2 pos = evalPos(pr, pc, u, v);
                    Color c = evalColor(pr, pc, u, v);
                    Color t = target.sampleBilinear(pos.x, pos.y);
                    Color d = c - t;
                    sumAbs += std::abs(d.r) + std::abs(d.g) + std::abs(d.b);
                    ++count;
                }
            }
        }
    }
    if (count == 0) return 0;
    return sumAbs / (count * 3);
}

} // namespace gmcore
