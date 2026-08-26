// SparseBlockSolver.h — a tiny, dependency-free sparse SPD linear solver.
//
// The gradient-mesh energy (data + smoothness + boundary + vector-line
// terms) has *local* support: each residual only touches the handful of
// control-point unknowns belonging to one patch or one mesh edge. That
// means both the color linear system and the Gauss-Newton normal
// equations for geometry are block-sparse and symmetric positive
// (semi-)definite -- ideal for block Jacobi-preconditioned Conjugate
// Gradient, which is what large-scale bundle-adjustment/mesh-fitting
// solvers (e.g. Ceres) use for exactly this kind of problem. Implementing
// this ourselves avoids pulling in an external linear-algebra dependency
// (e.g. Eigen) that the Xcode project would otherwise have to vendor.
#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace gmcore {

// Small fixed-size-at-runtime dense matrix helpers (row-major, size N*N).
inline void smallMatMulVec(const std::vector<double>& M, int N, const double* x, double* y) {
    for (int i = 0; i < N; ++i) {
        double s = 0;
        for (int j = 0; j < N; ++j) s += M[i * N + j] * x[j];
        y[i] = s;
    }
}
inline std::vector<double> smallMatTranspose(const std::vector<double>& M, int N) {
    std::vector<double> T(N * N);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) T[j * N + i] = M[i * N + j];
    return T;
}
// Gauss-Jordan inverse; falls back to a damped pseudo-inverse-ish identity
// scaling if singular (keeps the preconditioner well-defined even for
// degenerate blocks, e.g. an isolated vertex with no residuals yet).
inline std::vector<double> smallMatInverse(std::vector<double> M, int N) {
    std::vector<double> I(N * N, 0.0);
    for (int i = 0; i < N; ++i) I[i * N + i] = 1.0;
    for (int col = 0; col < N; ++col) {
        int piv = col;
        double best = std::abs(M[col * N + col]);
        for (int r = col + 1; r < N; ++r) {
            double v = std::abs(M[r * N + col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-12) { M[col * N + col] += 1e-6; best = 1e-6; piv = col; }
        if (piv != col) {
            for (int k = 0; k < N; ++k) { std::swap(M[col * N + k], M[piv * N + k]); std::swap(I[col * N + k], I[piv * N + k]); }
        }
        double d = M[col * N + col];
        for (int k = 0; k < N; ++k) { M[col * N + k] /= d; I[col * N + k] /= d; }
        for (int r = 0; r < N; ++r) {
            if (r == col) continue;
            double f = M[r * N + col];
            if (f == 0.0) continue;
            for (int k = 0; k < N; ++k) { M[r * N + k] -= f * M[col * N + k]; I[r * N + k] -= f * I[col * N + k]; }
        }
    }
    return I;
}

class SparseBlockMatrix {
public:
    int blockSize = 0;
    int numBlocks = 0;
    // key packs (i,j) with i<=j into one 64-bit value; value is a row-major N*N block.
    std::unordered_map<uint64_t, std::vector<double>> blocks;

    void init(int N, int nb) { blockSize = N; numBlocks = nb; blocks.clear(); }

    static uint64_t key(int i, int j) { return (uint64_t(uint32_t(i)) << 32) | uint32_t(j); }

    // Adds `local` (N*N, row-major, contribution to H[bi][bj]) into the matrix.
    void addBlock(int bi, int bj, const std::vector<double>& local) {
        int N = blockSize;
        if (bi > bj) {
            auto t = smallMatTranspose(local, N);
            addBlockOrdered(bj, bi, t);
        } else {
            addBlockOrdered(bi, bj, local);
        }
    }

    // Adds `val` into the scalar entry (si,sj) of the logical block (bi,bj),
    // handling the i<=j canonical storage (transposing indices when the
    // caller addresses the block in (row,col) order that is stored
    // transposed). Used to assemble Gauss-Newton normal equations one
    // residual row at a time (see MeshOptimizer.cpp).
    void addScalar(int bi, int bj, int si, int sj, double val) {
        int N = blockSize;
        int rbi = bi, rbj = bj, rsi = si, rsj = sj;
        if (rbi > rbj) { std::swap(rbi, rbj); std::swap(rsi, rsj); }
        auto& blk = blocks[key(rbi, rbj)];
        if (blk.empty()) blk.assign(N * N, 0.0);
        blk[rsi * N + rsj] += val;
    }

    void addToDiagonal(int bi, double lambda) {
        int N = blockSize;
        auto& blk = blocks[key(bi, bi)];
        if (blk.empty()) blk.assign(N * N, 0.0);
        for (int i = 0; i < N; ++i) blk[i * N + i] += lambda;
    }

    // y = H * x, x/y are numBlocks*blockSize long.
    std::vector<double> multiply(const std::vector<double>& x) const {
        int N = blockSize;
        std::vector<double> y(x.size(), 0.0);
        for (const auto& kv : blocks) {
            int bi = int(kv.first >> 32), bj = int(kv.first & 0xffffffffu);
            const std::vector<double>& M = kv.second;
            double tmp[16];
            smallMatMulVec(M, N, &x[bj * N], tmp);
            for (int k = 0; k < N; ++k) y[bi * N + k] += tmp[k];
            if (bi != bj) {
                auto Mt = smallMatTranspose(M, N);
                smallMatMulVec(Mt, N, &x[bi * N], tmp);
                for (int k = 0; k < N; ++k) y[bj * N + k] += tmp[k];
            }
        }
        return y;
    }

    std::vector<std::vector<double>> blockJacobiInverse() const {
        int N = blockSize;
        std::vector<std::vector<double>> inv(numBlocks);
        for (int i = 0; i < numBlocks; ++i) {
            auto it = blocks.find(key(i, i));
            std::vector<double> d = (it != blocks.end()) ? it->second : std::vector<double>(N * N, 0.0);
            if (it == blocks.end()) for (int k = 0; k < N; ++k) d[k * N + k] = 1.0;
            inv[i] = smallMatInverse(d, N);
        }
        return inv;
    }

private:
    void addBlockOrdered(int bi, int bj, const std::vector<double>& local) {
        auto& blk = blocks[key(bi, bj)];
        if (blk.empty()) blk.assign(local.size(), 0.0);
        for (size_t k = 0; k < local.size(); ++k) blk[k] += local[k];
    }
};

// Block-Jacobi preconditioned CG for SPD systems H x = b.
inline std::vector<double> solveSPD_PCG(const SparseBlockMatrix& H, const std::vector<double>& b,
                                         std::vector<double> x, int maxIter, double relTol = 1e-6) {
    int N = H.blockSize;
    auto Minv = H.blockJacobiInverse();
    auto applyM = [&](const std::vector<double>& r) {
        std::vector<double> z(r.size());
        double tmp[16];
        for (int i = 0; i < H.numBlocks; ++i) {
            smallMatMulVec(Minv[i], N, &r[i * N], tmp);
            for (int k = 0; k < N; ++k) z[i * N + k] = tmp[k];
        }
        return z;
    };
    std::vector<double> Ax = H.multiply(x);
    std::vector<double> r(b.size());
    for (size_t i = 0; i < b.size(); ++i) r[i] = b[i] - Ax[i];
    std::vector<double> z = applyM(r);
    std::vector<double> p = z;
    double rz = 0; for (size_t i = 0; i < r.size(); ++i) rz += r[i] * z[i];
    double b0 = 0; for (double v : b) b0 += v * v;
    double tol2 = relTol * relTol * std::max(b0, 1e-30);

    for (int it = 0; it < maxIter; ++it) {
        std::vector<double> Ap = H.multiply(p);
        double pAp = 0; for (size_t i = 0; i < p.size(); ++i) pAp += p[i] * Ap[i];
        if (std::abs(pAp) < 1e-300) break;
        double alpha = rz / pAp;
        for (size_t i = 0; i < x.size(); ++i) x[i] += alpha * p[i];
        for (size_t i = 0; i < r.size(); ++i) r[i] -= alpha * Ap[i];
        double rr = 0; for (double v : r) rr += v * v;
        if (rr < tol2) break;
        std::vector<double> znew = applyM(r);
        double rzNew = 0; for (size_t i = 0; i < r.size(); ++i) rzNew += r[i] * znew[i];
        double beta = rzNew / std::max(rz, 1e-300);
        for (size_t i = 0; i < p.size(); ++i) p[i] = znew[i] + beta * p[i];
        z = std::move(znew);
        rz = rzNew;
    }
    return x;
}

} // namespace gmcore
