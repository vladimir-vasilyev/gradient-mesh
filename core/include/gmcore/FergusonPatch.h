// FergusonPatch.h — the bicubic Hermite ("Ferguson") patch at the heart of
// a gradient mesh (Sun et al. 2007, Sec. 3). Each patch is defined by 4
// corners, each carrying a position/value P, two tangents Pu, Pv and a
// twist Puv; the same machinery is reused for the color surface (P -> C
// etc). Basis weights are exposed separately from evaluation so the
// optimizer can build an *analytic* Jacobian instead of differencing.
#pragma once
#include "gmcore/Vec2.h"
#include "gmcore/Color.h"
#include <array>

namespace gmcore {

// Cubic Hermite basis functions and derivatives, t in [0,1].
struct HermiteBasis {
    double h00, h01, h10, h11;       // value-basis[0], value-basis[1], tangent-basis[0], tangent-basis[1]
    double dh00, dh01, dh10, dh11;   // d/dt of the above

    static HermiteBasis eval(double t) {
        HermiteBasis b;
        double t2 = t * t, t3 = t2 * t;
        b.h00 = 2 * t3 - 3 * t2 + 1;
        b.h01 = -2 * t3 + 3 * t2;
        b.h10 = t3 - 2 * t2 + t;
        b.h11 = t3 - t2;
        b.dh00 = 6 * t2 - 6 * t;
        b.dh01 = -6 * t2 + 6 * t;
        b.dh10 = 3 * t2 - 4 * t + 1;
        b.dh11 = 3 * t2 - 2 * t;
        return b;
    }
};

// One "kind" of corner datum: value, u-tangent, v-tangent, twist.
enum class HermiteKind { Value = 0, TangentU = 1, TangentV = 2, Twist = 3 };

// Weight of unknown (corner a in {0,1} for u, corner b in {0,1} for v, kind)
// in the bicubic Hermite sum, and its parametric derivatives. Index scheme:
// slot = ((a*2 + b) * 4) + (int)kind, slot in [0,16).
struct PatchWeights {
    std::array<double, 16> w{};   // value weights
    std::array<double, 16> wu{};  // d(value)/du weights
    std::array<double, 16> wv{};  // d(value)/dv weights

    static PatchWeights at(double u, double v) {
        HermiteBasis Hu = HermiteBasis::eval(u), Hv = HermiteBasis::eval(v);
        double Hu0[2] = {Hu.h00, Hu.h01}, Hu1[2] = {Hu.h10, Hu.h11};
        double Hv0[2] = {Hv.h00, Hv.h01}, Hv1[2] = {Hv.h10, Hv.h11};
        double dHu0[2] = {Hu.dh00, Hu.dh01}, dHu1[2] = {Hu.dh10, Hu.dh11};
        double dHv0[2] = {Hv.dh00, Hv.dh01}, dHv1[2] = {Hv.dh10, Hv.dh11};

        PatchWeights out;
        for (int a = 0; a < 2; ++a) {
            for (int b = 0; b < 2; ++b) {
                int base = (a * 2 + b) * 4;
                // Value (P): Hu0[a]*Hv0[b]
                out.w[base + 0] = Hu0[a] * Hv0[b];
                out.wu[base + 0] = dHu0[a] * Hv0[b];
                out.wv[base + 0] = Hu0[a] * dHv0[b];
                // TangentU (Pu): Hu1[a]*Hv0[b]
                out.w[base + 1] = Hu1[a] * Hv0[b];
                out.wu[base + 1] = dHu1[a] * Hv0[b];
                out.wv[base + 1] = Hu1[a] * dHv0[b];
                // TangentV (Pv): Hu0[a]*Hv1[b]
                out.w[base + 2] = Hu0[a] * Hv1[b];
                out.wu[base + 2] = dHu0[a] * Hv1[b];
                out.wv[base + 2] = Hu0[a] * dHv1[b];
                // Twist (Puv): Hu1[a]*Hv1[b]
                out.w[base + 3] = Hu1[a] * Hv1[b];
                out.wu[base + 3] = dHu1[a] * Hv1[b];
                out.wv[base + 3] = Hu1[a] * dHv1[b];
            }
        }
        return out;
    }
};

template <typename T>
struct HermiteCorner {
    T P{}, Pu{}, Pv{}, Puv{};
    const T& get(HermiteKind k) const {
        switch (k) {
            case HermiteKind::Value: return P;
            case HermiteKind::TangentU: return Pu;
            case HermiteKind::TangentV: return Pv;
            default: return Puv;
        }
    }
};

// corners indexed corners[a][b], a=u-side(0/1), b=v-side(0/1)
template <typename T>
inline T evalHermitePatch(const HermiteCorner<T> corners[2][2], double u, double v,
                           T* dU = nullptr, T* dV = nullptr) {
    PatchWeights w = PatchWeights::at(u, v);
    T value{}, du{}, dv{};
    bool first = true;
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            int base = (a * 2 + b) * 4;
            const HermiteCorner<T>& c = corners[a][b];
            const T terms[4] = {c.P, c.Pu, c.Pv, c.Puv};
            for (int k = 0; k < 4; ++k) {
                if (first) {
                    value = terms[k] * w.w[base + k];
                    du = terms[k] * w.wu[base + k];
                    dv = terms[k] * w.wv[base + k];
                    first = false;
                } else {
                    value = value + terms[k] * w.w[base + k];
                    du = du + terms[k] * w.wu[base + k];
                    dv = dv + terms[k] * w.wv[base + k];
                }
            }
        }
    }
    if (dU) *dU = du;
    if (dV) *dV = dv;
    return value;
}

} // namespace gmcore
