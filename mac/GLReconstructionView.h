// GLReconstructionView.h — the OpenGL/GLSL counterpart to CanvasView's CPU
// reconstruction preview (CanvasView.showReconstructionPreview /
// -refreshReconstructionPreview, backed by DocumentModel
// -renderReconstructionPreview / gmcore::GradientMesh::render()). Renders
// the SAME gradient mesh via the shader design in
// gmcore/GLShaderSources.h: EXACT bicubic-Hermite POSITION per
// tessellation node (vertex shader) and EXACT bicubic-Hermite COLOR per
// PIXEL from the interpolated (u,v) (fragment shader) — more accurate than
// the CPU renderer, which only evaluates color exactly at tessellation
// nodes and linearly interpolates between them via triangle rasterization.
// See GLShaderSources.h's header comment for the standalone verification
// that established this design (a headless EGL/GL harness run against
// this exact shader text: GPU float32 output agreed with a CPU
// double-precision reference to ~1e-7, both position and color, sRGB and
// CIELUV branches), and gmcore/MeshRenderBuffers.h's own test for the
// separate confirmation that the packed vertex buffer this view consumes
// reproduces GradientMesh::evalPos/evalColor exactly.
//
// This is an OPTIONAL, TOGGLEABLE alternative to the CPU path (see
// MainWindowController's "GPU Preview (OpenGL)" checkbox), deliberately
// kept side by side rather than replacing the CPU renderer, so the two can
// be compared directly. IMPORTANT CAVEAT: this class itself (the
// NSOpenGLView/CGL integration below) has NOT been compiled or run
// anywhere in this project's development — no macOS SDK is available in
// the sandbox environment this was written in. The shader math and the
// buffer-building it depends on WERE independently, empirically verified
// (see above); what has NOT been verified is this specific
// context-creation/view-integration glue. Needs a real Xcode build to
// confirm end-to-end. See the commit message for the full breakdown of
// what's verified vs. not.
//
// Platform note: only this file (NSOpenGLView / NSOpenGLPixelFormat /
// NSOpenGLContext) is macOS-specific. The GLSL shader text
// (GLShaderSources.h) and the CPU-side buffer building
// (MeshRenderBuffers.h/.cpp) are plain portable C++ with zero AppKit/GL
// dependency — reusable as-is by a future Windows/WGL OpenGL backend, per
// the project's explicit "OpenGL+GLSL, because it needs to run on Windows
// eventually" decision (this is also why the design targets OpenGL 3.3
// core, not something Apple-only like Metal).
#import <Cocoa/Cocoa.h>
#import "DocumentModel.h"  // for GMGPUMeshBuffers

NS_ASSUME_NONNULL_BEGIN

// NSOpenGLView/NSOpenGLContext/NSOpenGLPixelFormat have been deprecated
// (in favor of MTKView/Metal) since macOS 10.14, but remain fully
// functional -- this project deliberately still targets OpenGL (see the
// header comment above: needs to also run on Windows eventually, ruling
// out Metal). NOTE, corrected after an actual Xcode build: GL_SILENCE_DEPRECATION
// (defined in GLReconstructionView.mm) silences OpenGL.framework's OWN
// deprecated symbols but does NOT cover this separate AppKit-level
// deprecation tag on the NSOpenGLView class itself -- confirmed by a real
// build still emitting the warning even with that macro defined. Silenced
// explicitly here instead.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
@interface GLReconstructionView : NSOpenGLView

// Uploads (or replaces) the mesh data to render — see
// DocumentModel.gpuMeshBuffersWithSamplesPerPatchEdge:. Safe to call
// repeatedly (e.g. once per optimizer run, mirroring
// CanvasView.refreshReconstructionPreview).
- (void)uploadMeshBuffers:(GMGPUMeshBuffers*)buffers;

// Marks the view as having nothing to draw (clears to the same background
// color CanvasView uses when there's no image). Call this before the
// first mesh exists, or when switching away from the GPU preview.
- (void)clearMesh;

@end
#pragma clang diagnostic pop

NS_ASSUME_NONNULL_END
