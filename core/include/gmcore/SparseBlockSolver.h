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
//
// Block storage: a SORTED array of (bi,bj) keys (bi<=bj canonical, packed
// via key()) parallel to a flat `storage_` buffer, instead of the
// std::unordered_map<uint64_t, std::vector<double>> this class used to
// use. Every block-touching call (addScalar/addBlock/addToDiagonal)
// resolves its block via binary search over `keys_` -- for the mesh-
// topology-driven sparsity patterns this project actually builds (see
// MeshOptimizer.cpp's buildMeshBlockPattern), that's typically a few
// hundred entries at most, so a handful of comparisons over a contiguous,
// cache-friendly array beats hashing into a node-based map. Call
// reserveBlocks() with the full pattern once (right after init()), before
// any add* call, to avoid ever hitting the (still fully correct, just
// slower -- an insertion into the sorted array) not-yet-declared
// fallback in findOrCreate().
//
// IMPORTANT caveat, confirmed by an actual before/after harness (NOT just
// assumed): this changes the ORDER in which different blocks' contributions
// get summed in multiply() (now ascending sorted-key order; the old
// unordered_map's iteration order was hash-bucket order, effectively
// unspecified) -- floating-point addition isn't associative, so a result
// CAN and DOES differ from the pre-refactor one, even though the same
// terms, same weights, and the same per-residual accumulation order
// (accumulateGNRow itself is untouched) go into it. This is *not*
// bit-exact. A 4-configuration before/after harness (5x5 and 9x9 meshes,
// with and without vector lines, with and without pyramid restarts) found
// relative differences in final RMSE of ~1e-16 to ~1e-15 (i.e. ordinary
// floating-point noise floor) when no vector line is present, growing to
// ~1e-9 to ~1e-7 relative when a vector line term is active (that term's
// extra blocks perturb the summation order more). Every one of those
// differences is far below anything visible in the UI (RMSE is displayed
// to 4 decimal digits) or in any exported image, but callers that need
// literal bit-for-bit reproducibility across this refactor should not
// assume it holds.
#pragma once
#include <vector>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace gmcore {

// Small fixed-size-at-runtime dense matrix helpers (row-major, size N*N).
// Pointer-based (not std::vector<double>&) so callers can operate directly
// on a slice of SparseBlockMatrix's flat storage_ without materializing a
// temporary vector copy first.
inline void smallMatMulVec(const double* M, int N, const double* x, double* y) {
    for (int i = 0; i < N; ++i) {
        double s = 0;
        for (int j = 0; j < N; ++j) s += M[i * N + j] * x[j];
        y[i] = s;
    }
}
inline void smallMatMulVec(const std::vector<double>& M, int N, const double* x, double* y) {
    smallMatMulVec(M.data(), N, x, y);
}
// y = M^T * x, computed directly (without materializing the transpose) --
// same summation order (accumulates i=0..N-1 into each y[j]) as building
// smallMatTranspose(M,N) and then calling the plain smallMatMulVec on it
// would, so this is a pure allocation-avoidance change, not a
// reassociation of the sum.
inline void smallMatMulVecTransposed(const double* M, int N, const double* x, double* y) {
    for (int j = 0; j < N; ++j) {
        double s = 0;
        for (int i = 0; i < N; ++i) s += M[i * N + j] * x[i];
        y[j] = s;
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

    void init(int N, int nb) {
        blockSize = N; numBlocks = nb;
        keys_.clear();
        storage_.clear();
        cacheBi_ = cacheBj_ = -1;
        cacheSlot_ = -1;
    }

    // key packs (i,j) with i<=j into one 64-bit value (unchanged from the
    // old unordered_map version's convention -- still used to build the
    // pattern pairs passed to reserveBlocks).
    static uint64_t key(int i, int j) { return (uint64_t(uint32_t(i)) << 32) | uint32_t(j); }

    // Pre-declares the full set of (bi,bj) canonical (bi<=bj, either order
    // as given -- this sorts/canonicalizes/dedupes internally) block pairs
    // this matrix will be filled with, BEFORE any addScalar/addBlock/
    // addToDiagonal call. Purely a fast-path hint: every pair not declared
    // here still works correctly the first time it's touched (inserted
    // into keys_/storage_ in sorted position, an O(n) shift -- rare enough
    // after a correct reserveBlocks() call not to matter, but never a
    // silent drop). See MeshOptimizer.cpp's buildMeshBlockPattern for how
    // this project derives the pattern to pass in (it depends only on
    // mesh topology -- rows/cols -- never on vertex values, so it's the
    // same for every GN sub-iteration/outer iteration at a given
    // resolution level, and safe to build once and reuse).
    void reserveBlocks(std::vector<std::pair<int,int>> pairs) {
        for (auto& p : pairs) if (p.first > p.second) std::swap(p.first, p.second);
        std::sort(pairs.begin(), pairs.end());
        pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
        keys_.resize(pairs.size());
        for (size_t i = 0; i < pairs.size(); ++i) keys_[i] = key(pairs[i].first, pairs[i].second);
        storage_.assign(keys_.size() * (size_t)blockSize * blockSize, 0.0);
        // keys_ was just replaced wholesale -- any previously cached slot
        // index (from before this call) is meaningless now.
        cacheBi_ = cacheBj_ = -1;
        cacheSlot_ = -1;
    }

    // Adds `local` (N*N, row-major, contribution to H[bi][bj]) into the matrix.
    void addBlock(int bi, int bj, const std::vector<double>& local) {
        int N = blockSize;
        if (bi > bj) {
            auto t = smallMatTranspose(local, N);
            addBlockOrdered(bj, bi, t.data());
        } else {
            addBlockOrdered(bi, bj, local.data());
        }
    }

    // Adds `val` into the scalar entry (si,sj) of the logical block (bi,bj),
    // handling the i<=j canonical storage (transposing indices when the
    // caller addresses the block in (row,col) order that is stored
    // transposed). Used to assemble Gauss-Newton normal equations one
    // residual row at a time (see MeshOptimizer.cpp) -- this is the hot
    // path reserveBlocks() exists to speed up.
    void addScalar(int bi, int bj, int si, int sj, double val) {
        int N = blockSize;
        int rbi = bi, rbj = bj, rsi = si, rsj = sj;
        if (rbi > rbj) { std::swap(rbi, rbj); std::swap(rsi, rsj); }
        int slot = resolveSlot(rbi, rbj);
        storage_[(size_t)slot * N * N + rsi * N + rsj] += val;
    }

    void addToDiagonal(int bi, double lambda) {
        int N = blockSize;
        int slot = resolveSlot(bi, bi);
        double* blk = &storage_[(size_t)slot * N * N];
        for (int i = 0; i < N; ++i) blk[i * N + i] += lambda;
    }

    // Read-only lookup of an existing block's raw row-major N*N storage, or
    // nullptr if this (bi,bj) pair has never been touched. Only meaningful
    // as given for bi<=bj -- for bi>bj this returns the TRANSPOSED
    // canonical (bj,bi) block's storage (same as how this class stores
    // things internally), which every current caller avoids simply by
    // only ever querying bi==bj (there the transpose is moot).
    const double* findBlock(int bi, int bj) const {
        if (bi > bj) std::swap(bi, bj);
        uint64_t k = key(bi, bj);
        auto it = std::lower_bound(keys_.begin(), keys_.end(), k);
        if (it != keys_.end() && *it == k) return &storage_[(size_t)(it - keys_.begin()) * blockSize * blockSize];
        return nullptr;
    }

    // y = H * x, x/y are numBlocks*blockSize long.
    std::vector<double> multiply(const std::vector<double>& x) const {
        int N = blockSize;
        std::vector<double> y(x.size(), 0.0);
        double tmp[16];
        for (size_t s = 0; s < keys_.size(); ++s) {
            int bi = int(keys_[s] >> 32), bj = int(keys_[s] & 0xffffffffu);
            const double* M = &storage_[s * (size_t)N * N];
            smallMatMulVec(M, N, &x[bj * N], tmp);
            for (int k = 0; k < N; ++k) y[bi * N + k] += tmp[k];
            if (bi != bj) {
                smallMatMulVecTransposed(M, N, &x[bi * N], tmp);
                for (int k = 0; k < N; ++k) y[bj * N + k] += tmp[k];
            }
        }
        return y;
    }

    std::vector<std::vector<double>> blockJacobiInverse() const {
        int N = blockSize;
        std::vector<std::vector<double>> inv(numBlocks);
        for (int i = 0; i < numBlocks; ++i) {
            const double* d = findBlock(i, i);
            std::vector<double> dv;
            if (d) {
                dv.assign(d, d + (size_t)N * N);
            } else {
                dv.assign((size_t)N * N, 0.0);
                for (int k = 0; k < N; ++k) dv[k * N + k] = 1.0;
            }
            inv[i] = smallMatInverse(dv, N);
        }
        return inv;
    }

private:
    // Sorted (ascending) canonical keys, parallel to storage_'s blocks --
    // block s occupies storage_[s*blockSize*blockSize .. +blockSize*blockSize).
    std::vector<uint64_t> keys_;
    std::vector<double> storage_;

    // One-entry "last block touched" cache. Why this exists: profiling the
    // very first version of this refactor (binary search, no cache) against
    // the old unordered_map showed it was NOT a speedup -- it was ~1.6x MORE
    // instructions and measurably slower wall-clock on a real run. The
    // reason: replacing the hashmap with a sorted array didn't reduce the
    // NUMBER of block lookups, only changed the cost of each one, and an
    // O(log n) binary search over ~keys_.size() elements turned out to cost
    // more per call than the hashtable's O(1)-ish operator[] here. The
    // actual win was always available from the CALL PATTERN, not the data
    // structure: accumulateGNRow's nested loop over a RowEntry row (grouped
    // contiguously by originating vertex, up to 6 subs/vertex) calls
    // addScalar for every (sub_i,sub_j) pair, so the SAME (bi,bj) block gets
    // hit by up to blockSize*blockSize consecutive calls before the block
    // pair changes. Caching just the most recently resolved slot turns
    // nearly all of those into an O(1) integer compare + array index,
    // leaving a binary search only when the block pair actually changes --
    // O(number of distinct block pairs touched), not O(number of scalar
    // contributions). Confirmed empirically to fix the regression: see this
    // change's commit message for the before/after wall-clock numbers.
    int cacheBi_ = -1, cacheBj_ = -1;
    int cacheSlot_ = -1;

    // Returns the storage slot for canonical (bi<=bj), consulting/updating
    // the one-entry cache above first.
    int resolveSlot(int bi, int bj) {
        if (bi == cacheBi_ && bj == cacheBj_) return cacheSlot_;
        int slot = findOrCreate(bi, bj);
        cacheBi_ = bi; cacheBj_ = bj; cacheSlot_ = slot;
        return slot;
    }

    // Returns the storage slot for canonical (bi<=bj), creating (zero-
    // filled) and inserting it in sorted position if not already present.
    // O(log n) when already present (the common case after reserveBlocks);
    // O(n) (a vector insert) the first time an undeclared pair is touched.
    // An insert shifts every slot index at or after the insertion point, so
    // it invalidates the resolveSlot() cache above unconditionally (rather
    // than trying to reason about whether THIS particular cached pair moved)
    // -- reserveBlocks() means this path is not supposed to run at all in
    // the steady state, so paying a full cache-miss next call is fine.
    int findOrCreate(int bi, int bj) {
        uint64_t k = key(bi, bj);
        auto it = std::lower_bound(keys_.begin(), keys_.end(), k);
        size_t pos = (size_t)(it - keys_.begin());
        if (it != keys_.end() && *it == k) return (int)pos;
        keys_.insert(it, k);
        storage_.insert(storage_.begin() + pos * (size_t)blockSize * blockSize, (size_t)blockSize * blockSize, 0.0);
        cacheBi_ = cacheBj_ = -1;
        cacheSlot_ = -1;
        return (int)pos;
    }

    void addBlockOrdered(int bi, int bj, const double* local) {
        int N = blockSize;
        int slot = resolveSlot(bi, bj);
        double* blk = &storage_[(size_t)slot * N * N];
        for (int k = 0; k < N * N; ++k) blk[k] += local[k];
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
