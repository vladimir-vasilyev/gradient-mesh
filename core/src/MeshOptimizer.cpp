#include "gmcore/MeshOptimizer.h"
#include "gmcore/SparseBlockSolver.h"
#include "gmcore/FergusonPatch.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace gmcore {

namespace {

struct RowEntry { int vertex; int sub; double coeff; };

// Accumulates one scalar Gauss-Newton residual row (r(x) ~ r0 + J*delta)
// into the normal equations H*delta = g (g holds +J^T*(-r0), i.e. the
// right-hand side such that solving gives the Gauss-Newton update step).
void accumulateGNRow(SparseBlockMatrix& H, std::vector<double>& g, const std::vector<RowEntry>& row,
                      double r0, double weight) {
    int n = (int)row.size();
    for (int i = 0; i < n; ++i) {
        const RowEntry& ei = row[i];
        g[ei.vertex * H.blockSize + ei.sub] += -weight * ei.coeff * r0;
        double diagVal = weight * ei.coeff * ei.coeff;
        H.addScalar(ei.vertex, ei.vertex, ei.sub, ei.sub, diagVal);
        for (int j = i + 1; j < n; ++j) {
            const RowEntry& ej = row[j];
            double val = weight * ei.coeff * ej.coeff;
            if (ei.vertex == ej.vertex) {
                H.addScalar(ei.vertex, ej.vertex, ei.sub, ej.sub, val);
                H.addScalar(ei.vertex, ej.vertex, ej.sub, ei.sub, val);
            } else {
                H.addScalar(ei.vertex, ej.vertex, ei.sub, ej.sub, val);
            }
        }
    }
}

double areaWeightAt(const GradientMesh& mesh, int pr, int pc, double u, double v, double duv) {
    Vec2 dU, dV;
    mesh.evalPos(pr, pc, u, v, &dU, &dV);
    double a = std::abs(dU.cross(dV)) * duv;
    return std::max(a, 1e-6);
}

Vec2 nearestVectorLineDir(const std::vector<VectorLine>& lines, const Vec2& p, double radius, bool& found) {
    found = false;
    double bestD2 = radius * radius;
    Vec2 bestDir{1, 0};
    for (const auto& line : lines) {
        for (size_t i = 0; i + 1 < line.points.size(); ++i) {
            Vec2 a = line.points[i], b = line.points[i + 1];
            Vec2 ab = b - a;
            double len2 = ab.lengthSq();
            if (len2 < 1e-9) continue;
            double t = (p - a).dot(ab) / len2;
            t = std::max(0.0, std::min(1.0, t));
            Vec2 proj = a + ab * t;
            double d2 = (p - proj).lengthSq();
            if (d2 < bestD2) { bestD2 = d2; bestDir = ab.normalized(); found = true; }
        }
    }
    return bestDir;
}

void addSmoothnessTerms(SparseBlockMatrix& H, std::vector<double>& g, const GradientMesh& mesh,
                         const std::vector<Vec2>& P, double weight) {
    // second differences along u (columns) and v (rows), on x and y separately.
    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
            double rx = P[i0].x - 2 * P[i1].x + P[i2].x;
            double ry = P[i0].y - 2 * P[i1].y + P[i2].y;
            accumulateGNRow(H, g, {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}}, rx, weight);
            accumulateGNRow(H, g, {{i0, 1, 1}, {i1, 1, -2}, {i2, 1, 1}}, ry, weight);
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
            double rx = P[i0].x - 2 * P[i1].x + P[i2].x;
            double ry = P[i0].y - 2 * P[i1].y + P[i2].y;
            accumulateGNRow(H, g, {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}}, rx, weight);
            accumulateGNRow(H, g, {{i0, 1, 1}, {i1, 1, -2}, {i2, 1, 1}}, ry, weight);
        }
    }
}

// Total weighted energy the geometry Gauss-Newton step actually minimizes:
// data term (area-weighted reconstruction error) + smoothness + boundary +
// vector-line terms, using the exact same residual formulas and sample
// density as gaussNewtonGeometryStep's H/g assembly. This -- not a cheap,
// differently-sampled RMSE proxy -- is what backtracking must check a step
// against; using a mismatched acceptance metric let the line search reject
// genuinely energy-decreasing steps (or accept non-decreasing ones), which
// starved geometry of any real movement after the first couple of outer
// iterations and left the mesh looking effectively rectangular.
double computeGeometryEnergy(const GradientMesh& mesh, const Image& target,
                              const std::vector<VectorLine>& vectorLines, const OptimizerOptions& opts) {
    int n = std::max(2, opts.samplesPerPatchEdge);
    double duv = 1.0 / (n * n);
    double energy = 0.0;

    for (int pr = 0; pr < mesh.rows - 1; ++pr) {
        for (int pc = 0; pc < mesh.cols - 1; ++pc) {
            for (int i = 0; i <= n; ++i) {
                double v = double(i) / n;
                for (int j = 0; j <= n; ++j) {
                    double u = double(j) / n;
                    Vec2 pos = mesh.evalPos(pr, pc, u, v);
                    Color cmesh = mesh.evalColor(pr, pc, u, v);
                    Color ctarget = target.sampleBilinear(pos.x, pos.y);
                    double w = areaWeightAt(mesh, pr, pc, u, v, duv);
                    Color d = cmesh - ctarget;
                    energy += w * d.lengthSq();
                }
            }
        }
    }

    std::vector<Vec2> P(mesh.vertices.size());
    for (size_t i = 0; i < P.size(); ++i) P[i] = mesh.vertices[i].P;

    for (int r = 0; r < mesh.rows; ++r) {
        for (int c = 1; c < mesh.cols - 1; ++c) {
            Vec2 d = P[mesh.idx(r, c - 1)] - P[mesh.idx(r, c)] * 2.0 + P[mesh.idx(r, c + 1)];
            energy += opts.smoothWeightGeom * (d.x * d.x + d.y * d.y);
        }
    }
    for (int c = 0; c < mesh.cols; ++c) {
        for (int r = 1; r < mesh.rows - 1; ++r) {
            Vec2 d = P[mesh.idx(r - 1, c)] - P[mesh.idx(r, c)] * 2.0 + P[mesh.idx(r + 1, c)];
            energy += opts.smoothWeightGeom * (d.x * d.x + d.y * d.y);
        }
    }

    for (const auto& mv : mesh.vertices) {
        if (!mv.isBoundary) continue;
        Vec2 t = mesh.boundary[mv.boundarySide].eval(mv.boundaryT);
        Vec2 d = mv.P - t;
        energy += opts.boundaryWeight * (d.x * d.x + d.y * d.y);
    }

    if (!vectorLines.empty()) {
        auto edgeEnergy = [&](int v1, int v2) {
            Vec2 mid = (P[v1] + P[v2]) * 0.5;
            bool found = false;
            Vec2 dir = nearestVectorLineDir(vectorLines, mid, opts.vectorLineInfluenceRadius, found);
            if (!found) return;
            double r0 = (P[v2] - P[v1]).cross(dir);
            energy += opts.vectorLineWeight * r0 * r0;
        };
        for (int r = 0; r < mesh.rows; ++r)
            for (int c = 0; c < mesh.cols - 1; ++c) edgeEnergy(mesh.idx(r, c), mesh.idx(r, c + 1));
        for (int c = 0; c < mesh.cols; ++c)
            for (int r = 0; r < mesh.rows - 1; ++r) edgeEnergy(mesh.idx(r, c), mesh.idx(r + 1, c));
    }

    return energy;
}

} // namespace

void MeshOptimizer::optimizeAtCurrentResolution(GradientMesh& mesh, const Image& target,
                                                 const std::vector<VectorLine>& vectorLines,
                                                 const OptimizerOptions& opts,
                                                 const OptimizerProgressCallback& cb, int level,
                                                 int totalLevels) {
    int numV = (int)mesh.vertices.size();
    int n = std::max(2, opts.samplesPerPatchEdge);
    double duv = 1.0 / (n * n);

    double lambda = opts.geomDampingInitial;

    for (int outer = 0; outer < opts.outerIterationsPerLevel; ++outer) {
        // 1) Re-project boundary vertices onto their spline (soft constraint target).
        for (auto& v : mesh.vertices) {
            if (!v.isBoundary) continue;
            v.boundaryT = mesh.boundary[v.boundarySide].closestT(v.P);
        }

        // 2) Solve for colors exactly (data term is linear in color unknowns).
        {
            SparseBlockMatrix H;
            H.init(4, numV);
            std::vector<double> gR(numV * 4, 0.0), gG(numV * 4, 0.0), gB(numV * 4, 0.0);
            std::vector<double> x0R(numV * 4), x0G(numV * 4), x0B(numV * 4);
            for (int i = 0; i < numV; ++i) {
                const MeshVertex& mv = mesh.vertices[i];
                Color c[4] = {mv.C, mv.Cu, mv.Cv, mv.Cuv};
                for (int k = 0; k < 4; ++k) {
                    x0R[i * 4 + k] = c[k].r; x0G[i * 4 + k] = c[k].g; x0B[i * 4 + k] = c[k].b;
                }
            }

            for (int pr = 0; pr < mesh.rows - 1; ++pr) {
                for (int pc = 0; pc < mesh.cols - 1; ++pc) {
                    for (int i = 0; i <= n; ++i) {
                        double v = double(i) / n;
                        for (int j = 0; j <= n; ++j) {
                            double u = double(j) / n;
                            Vec2 pos = mesh.evalPos(pr, pc, u, v);
                            Color cmesh = mesh.evalColor(pr, pc, u, v);
                            Color ctarget = target.sampleBilinear(pos.x, pos.y);
                            double w = areaWeightAt(mesh, pr, pc, u, v, duv);
                            PatchWeights pw = PatchWeights::at(u, v);

                            std::vector<RowEntry> row;
                            row.reserve(16);
                            for (int a = 0; a < 2; ++a) {
                                for (int b = 0; b < 2; ++b) {
                                    int base = (a * 2 + b) * 4;
                                    int vert = mesh.idx(pr + b, pc + a);
                                    for (int k = 0; k < 4; ++k)
                                        row.push_back({vert, k, pw.w[base + k]});
                                }
                            }
                            double r0R = cmesh.r - ctarget.r;
                            double r0G = cmesh.g - ctarget.g;
                            double r0B = cmesh.b - ctarget.b;
                            accumulateGNRow(H, gR, row, r0R, w);
                            accumulateGNRow(H, gG, row, r0G, w);
                            accumulateGNRow(H, gB, row, r0B, w);
                        }
                    }
                }
            }

            // Smoothness on base color (kind 0) + ridge on Cu,Cv,Cuv (kinds 1..3).
            for (int r = 0; r < mesh.rows; ++r) {
                for (int c = 1; c < mesh.cols - 1; ++c) {
                    int i0 = mesh.idx(r, c - 1), i1 = mesh.idx(r, c), i2 = mesh.idx(r, c + 1);
                    std::vector<RowEntry> row = {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}};
                    double r0r = mesh.vertices[i0].C.r - 2 * mesh.vertices[i1].C.r + mesh.vertices[i2].C.r;
                    double r0g = mesh.vertices[i0].C.g - 2 * mesh.vertices[i1].C.g + mesh.vertices[i2].C.g;
                    double r0b = mesh.vertices[i0].C.b - 2 * mesh.vertices[i1].C.b + mesh.vertices[i2].C.b;
                    accumulateGNRow(H, gR, row, r0r, opts.smoothWeightColor);
                    accumulateGNRow(H, gG, row, r0g, opts.smoothWeightColor);
                    accumulateGNRow(H, gB, row, r0b, opts.smoothWeightColor);
                }
            }
            for (int c = 0; c < mesh.cols; ++c) {
                for (int r = 1; r < mesh.rows - 1; ++r) {
                    int i0 = mesh.idx(r - 1, c), i1 = mesh.idx(r, c), i2 = mesh.idx(r + 1, c);
                    std::vector<RowEntry> row = {{i0, 0, 1}, {i1, 0, -2}, {i2, 0, 1}};
                    double r0r = mesh.vertices[i0].C.r - 2 * mesh.vertices[i1].C.r + mesh.vertices[i2].C.r;
                    double r0g = mesh.vertices[i0].C.g - 2 * mesh.vertices[i1].C.g + mesh.vertices[i2].C.g;
                    double r0b = mesh.vertices[i0].C.b - 2 * mesh.vertices[i1].C.b + mesh.vertices[i2].C.b;
                    accumulateGNRow(H, gR, row, r0r, opts.smoothWeightColor);
                    accumulateGNRow(H, gG, row, r0g, opts.smoothWeightColor);
                    accumulateGNRow(H, gB, row, r0b, opts.smoothWeightColor);
                }
            }
            for (int i = 0; i < numV; ++i) {
                for (int k = 1; k < 4; ++k) {
                    std::vector<RowEntry> row = {{i, k, 1}};
                    Color c[4] = {mesh.vertices[i].C, mesh.vertices[i].Cu, mesh.vertices[i].Cv, mesh.vertices[i].Cuv};
                    accumulateGNRow(H, gR, row, c[k].r, opts.colorDerivRidge);
                    accumulateGNRow(H, gG, row, c[k].g, opts.colorDerivRidge);
                    accumulateGNRow(H, gB, row, c[k].b, opts.colorDerivRidge);
                }
            }

            auto deltaR = solveSPD_PCG(H, gR, std::vector<double>(numV * 4, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);
            auto deltaG = solveSPD_PCG(H, gG, std::vector<double>(numV * 4, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);
            auto deltaB = solveSPD_PCG(H, gB, std::vector<double>(numV * 4, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);

            for (int i = 0; i < numV; ++i) {
                MeshVertex& mv = mesh.vertices[i];
                Color* slots[4] = {&mv.C, &mv.Cu, &mv.Cv, &mv.Cuv};
                for (int k = 0; k < 4; ++k) {
                    slots[k]->r = x0R[i * 4 + k] + deltaR[i * 4 + k];
                    slots[k]->g = x0G[i * 4 + k] + deltaG[i * 4 + k];
                    slots[k]->b = x0B[i * 4 + k] + deltaB[i * 4 + k];
                }
            }
        }

        // 3) Refine geometry (position only) with damped Gauss-Newton + backtracking.
        for (int gi = 0; gi < opts.geomGaussNewtonItersPerOuter; ++gi) {
            SparseBlockMatrix H;
            H.init(2, numV);
            std::vector<double> g(numV * 2, 0.0);
            std::vector<Vec2> P(numV);
            for (int i = 0; i < numV; ++i) P[i] = mesh.vertices[i].P;

            for (int pr = 0; pr < mesh.rows - 1; ++pr) {
                for (int pc = 0; pc < mesh.cols - 1; ++pc) {
                    for (int i = 0; i <= n; ++i) {
                        double v = double(i) / n;
                        for (int j = 0; j <= n; ++j) {
                            double u = double(j) / n;
                            Vec2 pos = mesh.evalPos(pr, pc, u, v);
                            Color cmesh = mesh.evalColor(pr, pc, u, v);
                            Color ctarget = target.sampleBilinear(pos.x, pos.y);
                            ColorGrad grad = target.sampleGradient(pos.x, pos.y);
                            double w = areaWeightAt(mesh, pr, pc, u, v, duv);
                            PatchWeights pw = PatchWeights::at(u, v);

                            std::vector<RowEntry> rowCorners; // corner vertex + Value-weight only
                            for (int a = 0; a < 2; ++a) {
                                for (int b = 0; b < 2; ++b) {
                                    int base = (a * 2 + b) * 4;
                                    int vert = mesh.idx(pr + b, pc + a);
                                    rowCorners.push_back({vert, -1, pw.w[base + 0]});
                                }
                            }
                            double r0r = cmesh.r - ctarget.r;
                            double r0g = cmesh.g - ctarget.g;
                            double r0b = cmesh.b - ctarget.b;

                            auto buildChanneled = [&](double gx, double gy) {
                                std::vector<RowEntry> row;
                                row.reserve(rowCorners.size() * 2);
                                for (auto& rc : rowCorners) {
                                    row.push_back({rc.vertex, 0, -gx * rc.coeff});
                                    row.push_back({rc.vertex, 1, -gy * rc.coeff});
                                }
                                return row;
                            };
                            accumulateGNRow(H, g, buildChanneled(grad.dx.r, grad.dy.r), r0r, w);
                            accumulateGNRow(H, g, buildChanneled(grad.dx.g, grad.dy.g), r0g, w);
                            accumulateGNRow(H, g, buildChanneled(grad.dx.b, grad.dy.b), r0b, w);
                        }
                    }
                }
            }

            addSmoothnessTerms(H, g, mesh, P, opts.smoothWeightGeom);

            for (int i = 0; i < numV; ++i) {
                const MeshVertex& mv = mesh.vertices[i];
                if (!mv.isBoundary) continue;
                Vec2 target_ = mesh.boundary[mv.boundarySide].eval(mv.boundaryT);
                accumulateGNRow(H, g, {{i, 0, 1}}, mv.P.x - target_.x, opts.boundaryWeight);
                accumulateGNRow(H, g, {{i, 1, 1}}, mv.P.y - target_.y, opts.boundaryWeight);
            }

            if (!vectorLines.empty()) {
                auto addEdge = [&](int v1, int v2) {
                    Vec2 mid = (P[v1] + P[v2]) * 0.5;
                    bool found = false;
                    Vec2 dir = nearestVectorLineDir(vectorLines, mid, opts.vectorLineInfluenceRadius, found);
                    if (!found) return;
                    Vec2 e = P[v2] - P[v1];
                    double r0 = e.cross(dir);
                    accumulateGNRow(H, g,
                                     {{v1, 0, -dir.y}, {v1, 1, dir.x}, {v2, 0, dir.y}, {v2, 1, -dir.x}},
                                     r0, opts.vectorLineWeight);
                };
                for (int r = 0; r < mesh.rows; ++r)
                    for (int c = 0; c < mesh.cols - 1; ++c) addEdge(mesh.idx(r, c), mesh.idx(r, c + 1));
                for (int c = 0; c < mesh.cols; ++c)
                    for (int r = 0; r < mesh.rows - 1; ++r) addEdge(mesh.idx(r, c), mesh.idx(r + 1, c));
            }

            // Levenberg damping (scale-aware: proportional to each diagonal entry).
            for (int i = 0; i < numV; ++i) {
                auto it = H.blocks.find(SparseBlockMatrix::key(i, i));
                double d0 = (it != H.blocks.end()) ? it->second[0] : 1.0;
                double d1 = (it != H.blocks.end()) ? it->second[3] : 1.0;
                H.addScalar(i, i, 0, 0, lambda * std::max(d0, 1e-6));
                H.addScalar(i, i, 1, 1, lambda * std::max(d1, 1e-6));
            }

            auto delta = solveSPD_PCG(H, g, std::vector<double>(numV * 2, 0.0), opts.cgMaxIterations, opts.cgRelTolerance);
#ifdef GMCORE_DEBUG_GEOM
            double maxDelta = 0; for (double d : delta) maxDelta = std::max(maxDelta, std::abs(d));
            double gNorm = 0; for (double v : g) gNorm += v*v; gNorm = std::sqrt(gNorm);
#endif

            // Backtracking must check the step against the SAME objective the
            // step was computed to reduce (data + smoothness + boundary +
            // vector-line energy, at the same sample density) -- not a
            // cheaper, differently-sampled RMSE proxy, which can disagree
            // with it and reject perfectly good steps. See
            // computeGeometryEnergy's comment above for why this matters.
            double baseEnergy = computeGeometryEnergy(mesh, target, vectorLines, opts);
            double alpha = 1.0;
            bool improved = false;
            for (int tries = 0; tries < 4; ++tries) {
                for (int i = 0; i < numV; ++i) {
                    mesh.vertices[i].P.x = P[i].x + alpha * delta[i * 2 + 0];
                    mesh.vertices[i].P.y = P[i].y + alpha * delta[i * 2 + 1];
                }
                double newEnergy = computeGeometryEnergy(mesh, target, vectorLines, opts);
                if (newEnergy <= baseEnergy) { improved = true; break; }
                alpha *= 0.4;
            }
#ifdef GMCORE_DEBUG_GEOM
            std::fprintf(stderr, "    [GN gi=%d] lambda=%.4g |g|=%.4g maxDelta=%.4g improved=%d alpha=%.3g\n",
                         gi, lambda, gNorm, maxDelta, (int)improved, alpha);
#endif
            if (!improved) {
                for (int i = 0; i < numV; ++i) mesh.vertices[i].P = P[i]; // revert
                lambda = std::min(lambda * 4.0, 1e6);
            } else {
                lambda = std::max(lambda * 0.6, 1e-8);
            }
        }

        double rmse = mesh.reconstructionRMSE(target, opts.samplesPerPatchEdge);
        if (cb) cb({level, totalLevels, outer, opts.outerIterationsPerLevel, rmse});
    }
}

void MeshOptimizer::optimizeCoarseToFine(GradientMesh& mesh, const Image& fullResTarget,
                                          const std::vector<VectorLine>& vectorLinesFullRes,
                                          int numPyramidLevels, const OptimizerOptions& opts,
                                          const OptimizerProgressCallback& cb) {
    auto levels = Image::buildPyramid(fullResTarget, std::max(1, numPyramidLevels));
    int L = (int)levels.size();

    // Move the (full-resolution) mesh down to the coarsest level.
    double sxDown = double(levels[L - 1].width) / double(levels[0].width);
    double syDown = double(levels[L - 1].height) / double(levels[0].height);
    mesh.scalePositions(sxDown, syDown);

    for (int li = L - 1; li >= 0; --li) {
        std::vector<VectorLine> scaledLines;
        double sx = double(levels[li].width) / double(levels[0].width);
        double sy = double(levels[li].height) / double(levels[0].height);
        scaledLines.reserve(vectorLinesFullRes.size());
        for (const auto& line : vectorLinesFullRes) {
            VectorLine sl;
            sl.points.reserve(line.points.size());
            for (const auto& p : line.points) sl.points.push_back({p.x * sx, p.y * sy});
            scaledLines.push_back(std::move(sl));
        }
        OptimizerOptions levelOpts = opts;
        levelOpts.vectorLineInfluenceRadius *= std::max(sx, sy);

        optimizeAtCurrentResolution(mesh, levels[li], scaledLines, levelOpts, cb, li, L);

        if (li > 0) {
            double sxUp = double(levels[li - 1].width) / double(levels[li].width);
            double syUp = double(levels[li - 1].height) / double(levels[li].height);
            mesh.scalePositions(sxUp, syUp);
        }
    }
}

} // namespace gmcore
