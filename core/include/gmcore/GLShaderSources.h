// GLShaderSources.h -- portable GLSL 330-core source for the gradient-mesh
// GPU reconstruction renderer (Sun et al. 2007's bicubic Hermite / Ferguson
// patch surface, see FergusonPatch.h/GradientMesh.cpp). This file is pure
// data (C-string constants): no GL calls, no windowing/context dependency,
// so the SAME text is meant to be reused unmodified by every OpenGL
// backend this project ever adds (macOS via NSOpenGLView/CGL now; a future
// Windows/WGL backend later) -- only context creation and the draw-call
// plumbing around it are platform-specific, per the project's explicit
// "OpenGL+GLSL, because it needs to run on Windows eventually" decision.
//
// Design: one draw call, instanced once per mesh patch (gl_InstanceID),
// each instance rasterizing a fixed (u,v) tessellation template (see
// MeshRenderBuffers). The vertex shader computes the EXACT bicubic-Hermite
// POSITION at each tessellation node (same math as
// gmcore::evalHermitePatch<Vec2>, i.e. GradientMesh::evalPos). The
// fragment shader computes the EXACT bicubic-Hermite COLOR from the
// per-pixel INTERPOLATED (u,v) (same math as evalHermitePatch<Color>, i.e.
// GradientMesh::evalColor) -- this is deliberately more accurate than the
// CURRENT CPU renderer (GradientMesh::render()), which only evaluates
// color exactly at tessellation nodes and linearly interpolates
// (barycentric-lerps) between them via triangle rasterization. Both
// shaders read per-vertex data (18 floats/vertex -- see MeshRenderBuffers)
// from a GL_TEXTURE_BUFFER via texelFetch, since macOS's OpenGL is capped
// at 4.1 core and Shader Storage Buffer Objects (GL 4.3+) aren't
// available there, but Texture Buffer Objects (GL 3.1+) are.
//
// Empirically verified (not just derived): a standalone headless-EGL/GL
// harness (llvmpipe software rasterizer, since no GPU/display exists in
// the CI sandbox) ran this EXACT shader text against a synthetic
// deliberately-non-affine (Cuv != 0) test mesh and diffed the GPU's
// float32 output against a double-precision CPU port of
// evalHermitePatch/cieluvToSRGB: agreement was ~1e-7 (the float32 vs
// float64 noise floor) at both tessellation nodes and true interior
// (rasterized, non-node) pixels, in both the sRGB and CIELUV branches. See
// the dev harness / commit message for the exact numbers. NOT yet run
// through the real mac/GLReconstructionView.mm NSOpenGLView integration on
// actual hardware -- that needs an on-device Xcode build to confirm.
#pragma once

namespace gmcore {
namespace gpu {

inline const char* kMeshVertexShaderGLSL330 = R"GLSL(
#version 330 core
layout(location = 0) in vec2 uv;
uniform samplerBuffer vdata;
uniform int cols;
uniform int patchCols;
uniform vec4 xform; // scaleX, scaleY, centerX, centerY -- mesh space -> NDC: ndc = (mesh - xform.zw) * xform.xy

flat out int vPatchRow;
flat out int vPatchCol;
out vec2 vUV;

vec4 hbasis(float t) {
    float t2 = t * t, t3 = t2 * t;
    return vec4(2.0*t3 - 3.0*t2 + 1.0, -2.0*t3 + 3.0*t2, t3 - 2.0*t2 + t, t3 - t2);
}
int vidx(int row, int col) { return row * cols + col; }
vec2 fetchVec2(int base, int off) {
    return vec2(texelFetch(vdata, base + off).r, texelFetch(vdata, base + off + 1).r);
}

void main() {
    int patchIdx = gl_InstanceID;
    int patchRow = patchIdx / patchCols;
    int patchCol = patchIdx - patchRow * patchCols;
    vPatchRow = patchRow;
    vPatchCol = patchCol;
    vUV = uv;

    vec4 Hu = hbasis(uv.x);
    vec4 Hv = hbasis(uv.y);
    float Hu0[2] = float[2](Hu.x, Hu.y);
    float Hu1[2] = float[2](Hu.z, Hu.w);
    float Hv0[2] = float[2](Hv.x, Hv.y);
    float Hv1[2] = float[2](Hv.z, Hv.w);

    vec2 pos = vec2(0.0);
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            int row = patchRow + b;
            int col = patchCol + a;
            int base = vidx(row, col) * 18;
            vec2 P  = fetchVec2(base, 0);
            vec2 Pu = fetchVec2(base, 2);
            vec2 Pv = fetchVec2(base, 4);
            pos += P  * (Hu0[a] * Hv0[b]);
            pos += Pu * (Hu1[a] * Hv0[b]);
            pos += Pv * (Hu0[a] * Hv1[b]);
            // geometry Puv term omitted: always {0,0} per the paper (Sec 3
            // -- see GradientMesh.h's header comment / geomCorner()).
        }
    }

    vec2 ndc = (pos - xform.zw) * xform.xy;
    gl_Position = vec4(ndc, 0.0, 1.0);
    gl_PointSize = 1.0; // only used when the caller draws GL_POINTS (verification); ignored for GL_TRIANGLES
}
)GLSL";

inline const char* kMeshFragmentShaderGLSL330 = R"GLSL(
#version 330 core
uniform samplerBuffer vdata;
uniform int cols;
// 1 if the mesh's C/Cu/Cv/Cuv are this project's CIELUV *working*
// representation (DocumentModel.useCIELUVColorSpace / gmcore::ColorSpace.h)
// and must be converted back to sRGB per-pixel to display; 0 if they are
// already sRGB. Mirrors DocumentModel.mm's -renderReconstructionPreview,
// which interpolates in the mesh's native space and converts to sRGB only
// at the final raster -- done here per-FRAGMENT instead of per-output-
// pixel-after-the-fact, which is the same order of operations, just
// fused into one pass.
uniform int cieluv;

flat in int vPatchRow;
flat in int vPatchCol;
in vec2 vUV;
out vec4 fragColor;

vec4 hbasis(float t) {
    float t2 = t * t, t3 = t2 * t;
    return vec4(2.0*t3 - 3.0*t2 + 1.0, -2.0*t3 + 3.0*t2, t3 - 2.0*t2 + t, t3 - t2);
}
int vidx(int row, int col) { return row * cols + col; }
vec3 fetchVec3(int base, int off) {
    return vec3(texelFetch(vdata, base + off).r, texelFetch(vdata, base + off + 1).r, texelFetch(vdata, base + off + 2).r);
}

// Port of gmcore::cieluvToSRGB (core/src/ColorSpace.cpp) -- MUST stay in
// lockstep with that function; see its comment for the D65 /
// kCIELUVWorkingScale rationale. The gamma-encode step is sign-extended
// (sign(x)*f(abs(x))) to match ColorSpace.cpp's signedExtend(), since this
// project's LUV intermediate values go unclamped/out-of-gamut
// mid-optimization (see ColorSpace.h's header comment).
float srgbGammaEncodeSigned(float lin) {
    float a = abs(lin);
    float e = (a <= 0.0031308) ? (a * 12.92) : (1.055 * pow(a, 1.0/2.4) - 0.055);
    return sign(lin) * e;
}
vec3 cieluvToSRGB(vec3 luvScaled) {
    const float kCIELUVWorkingScale = 100.0;
    const float kXn = 0.95047, kYn = 1.00000, kZn = 1.08883;
    const float kWhiteDenom = kXn + 15.0*kYn + 3.0*kZn;
    const float kUn = 4.0*kXn/kWhiteDenom;
    const float kVn = 9.0*kYn/kWhiteDenom;
    const float kKappa = 24389.0/27.0;
    const float kEpsilon = 216.0/24389.0;

    vec3 luv = luvScaled * kCIELUVWorkingScale;
    float L = luv.x, u = luv.y, v = luv.z;

    float yr = (L > kKappa * kEpsilon) ? pow((L + 16.0) / 116.0, 3.0) : (L / kKappa);
    float Y = yr * kYn;

    float uPrime = (L != 0.0) ? (u / (13.0 * L) + kUn) : kUn;
    float vPrime = (L != 0.0) ? (v / (13.0 * L) + kVn) : kVn;

    float X, Z;
    if (vPrime != 0.0) {
        X = Y * 9.0 * uPrime / (4.0 * vPrime);
        Z = Y * (12.0 - 3.0 * uPrime - 20.0 * vPrime) / (4.0 * vPrime);
    } else {
        X = 0.0; Z = 0.0;
    }

    float rLin =  3.2404542*X - 1.5371385*Y - 0.4985314*Z;
    float gLin = -0.9692660*X + 1.8760108*Y + 0.0415560*Z;
    float bLin =  0.0556434*X - 0.2040259*Y + 1.0572252*Z;
    return vec3(srgbGammaEncodeSigned(rLin), srgbGammaEncodeSigned(gLin), srgbGammaEncodeSigned(bLin));
}

void main() {
    vec4 Hu = hbasis(vUV.x);
    vec4 Hv = hbasis(vUV.y);
    float Hu0[2] = float[2](Hu.x, Hu.y);
    float Hu1[2] = float[2](Hu.z, Hu.w);
    float Hv0[2] = float[2](Hv.x, Hv.y);
    float Hv1[2] = float[2](Hv.z, Hv.w);

    vec3 color = vec3(0.0);
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            int row = vPatchRow + b;
            int col = vPatchCol + a;
            int base = vidx(row, col) * 18;
            vec3 C   = fetchVec3(base, 6);
            vec3 Cu  = fetchVec3(base, 9);
            vec3 Cv  = fetchVec3(base, 12);
            vec3 Cuv = fetchVec3(base, 15);
            color += C   * (Hu0[a] * Hv0[b]);
            color += Cu  * (Hu1[a] * Hv0[b]);
            color += Cv  * (Hu0[a] * Hv1[b]);
            color += Cuv * (Hu1[a] * Hv1[b]);
        }
    }
    if (cieluv != 0) color = cieluvToSRGB(color);
    fragColor = vec4(color, 1.0);
}
)GLSL";

} // namespace gpu
} // namespace gmcore
