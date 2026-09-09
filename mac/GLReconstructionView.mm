// GLReconstructionView.mm — see GLReconstructionView.h for the design and
// the important "not yet built/run" caveat on this specific file.
#define GL_SILENCE_DEPRECATION 1  // Apple deprecated OpenGL in macOS 10.14 but it remains fully functional; scoped to this file only, not project-wide, since only this file touches the GL API directly.
#import "GLReconstructionView.h"
#include "gmcore/GLShaderSources.h"
#import <OpenGL/gl3.h>
#import <CoreGraphics/CoreGraphics.h>  // CGImageRef/CGColorSpaceRef/CGDataProviderRef for -renderToImageWithWidth:height: -- explicit include rather than relying on it coming in transitively via Cocoa.h, matching DocumentModel.mm's own explicit CoreGraphics import for the same CG calls.
#include <vector>   // std::vector<uint8_t> pixel buffers in -renderToImageWithWidth:height:
#include <cstring>  // memcpy (row flip) in -renderToImageWithWidth:height:

// This whole file legitimately uses the NSOpenGLView/NSOpenGLContext/
// NSOpenGLPixelFormat family throughout -- see GLReconstructionView.h's
// header comment on why OpenGL (not Metal) is used here at all, and the
// note on GL_SILENCE_DEPRECATION above only covering OpenGL.framework
// itself, not this separate AppKit deprecation. Blanket-silenced for the
// whole file rather than at every individual call site.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

@interface GLReconstructionView () {
    GLuint _program;
    GLuint _vao;
    GLuint _uvBuffer;
    GLuint _indexBuffer;
    GLuint _tbo;         // GL_TEXTURE_BUFFER backing buffer (raw per-vertex floats)
    GLuint _tboTexture;  // the texture view of _tbo the shaders actually sample (samplerBuffer)
    GLsizei _indexCount;
    NSInteger _cols, _patchRows, _patchCols;
    BOOL _cieluv;
    BOOL _hasMesh;
    double _meshMinX, _meshMinY, _meshMaxX, _meshMaxY;
    GLint _uCols, _uPatchCols, _uXform, _uCieluv;
    BOOL _glSetUp;
}
@end

@implementation GLReconstructionView

- (instancetype)initWithFrame:(NSRect)frameRect {
    // NSOpenGLProfileVersion3_2Core requests the highest CORE-profile
    // context the driver supports (typically up to 4.1 on Intel/AMD Macs —
    // macOS's own OpenGL ceiling; see GLShaderSources.h's header comment on
    // why the shaders target 3.3 core rather than anything requiring more,
    // e.g. SSBOs (4.3+), which aren't available on macOS at all).
    NSOpenGLPixelFormatAttribute attrs[] = {
        NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersion3_2Core,
        NSOpenGLPFAColorSize, 32,
        NSOpenGLPFADepthSize, 0,
        NSOpenGLPFADoubleBuffer,
        NSOpenGLPFAAccelerated,
        0
    };
    NSOpenGLPixelFormat* pf = [[NSOpenGLPixelFormat alloc] initWithAttributes:attrs];
    if (!pf) {
        // Extremely unlikely (would mean no GL 3.2+ core-profile support
        // at all), but fall back rather than returning nil so the app
        // doesn't crash constructing the UI -- -drawRect: below still
        // renders nothing usable in that case, which at least fails
        // visibly instead of at construction time.
        pf = [NSOpenGLPixelFormat new];
    }
    if ((self = [super initWithFrame:frameRect pixelFormat:pf])) {
        self.wantsBestResolutionOpenGLSurface = YES;
    }
    return self;
}

- (GLuint)compileShaderOfType:(GLenum)type source:(const char*)src {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, NULL);
    glCompileShader(sh);
    GLint ok = 0;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096]; GLsizei len = 0;
        glGetShaderInfoLog(sh, sizeof(log), &len, log);
        NSLog(@"GLReconstructionView: shader compile failed:\n%s", log);
    }
    return sh;
}

- (GLuint)compileProgram {
    GLuint vs = [self compileShaderOfType:GL_VERTEX_SHADER source:gmcore::gpu::kMeshVertexShaderGLSL330];
    GLuint fs = [self compileShaderOfType:GL_FRAGMENT_SHADER source:gmcore::gpu::kMeshFragmentShaderGLSL330];
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096]; GLsizei len = 0;
        glGetProgramInfoLog(prog, sizeof(log), &len, log);
        NSLog(@"GLReconstructionView: program link failed:\n%s", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// -prepareOpenGL (NSOpenGLView's designated setup hook, called once the
// view has a live context) is where every one-time GL object is created --
// deliberately NOT in -initWithFrame:, since the context doesn't exist yet
// at that point.
- (void)prepareOpenGL {
    [super prepareOpenGL];
    [[self openGLContext] makeCurrentContext];

    GLint swapInterval = 1;
    [[self openGLContext] setValues:&swapInterval forParameter:NSOpenGLContextParameterSwapInterval];

    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_uvBuffer);
    glGenBuffers(1, &_indexBuffer);
    glGenBuffers(1, &_tbo);
    glGenTextures(1, &_tboTexture);

    _program = [self compileProgram];
    glUseProgram(_program);
    _uCols = glGetUniformLocation(_program, "cols");
    _uPatchCols = glGetUniformLocation(_program, "patchCols");
    _uXform = glGetUniformLocation(_program, "xform");
    _uCieluv = glGetUniformLocation(_program, "cieluv");
    glUniform1i(glGetUniformLocation(_program, "vdata"), 0);

    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _uvBuffer);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);

    // Kept as cheap belt-and-suspenders, but see -applySRGBWindowColorSpace
    // below for the fix that actually turned out to matter: this flag only
    // affects a framebuffer whose COLOR ATTACHMENT format is itself
    // GL_SRGB8_ALPHA8. The default framebuffer here is a plain 32-bit RGBA
    // surface (see the NSOpenGLPixelFormat request in -initWithFrame:,
    // NSOpenGLPFAColorSize 32 -- not an sRGB format), so this was very
    // likely a no-op for the on-screen path all along. Confirmed by
    // comparing the on-screen preview against a PNG saved via
    // -renderToImageWithWidth:height: (same shader, same draw call --
    // -drawMeshFittedToSize: -- same numeric bytes, just read straight out
    // of an offscreen FBO with no window compositor involved): the
    // exported file came out correctly saturated while the on-screen view
    // was still washed out, with this flag already disabled either way.
    // That rules the fragment shader and vertex data out a second, more
    // direct way and narrows the bug specifically to the "present the
    // swapped GL buffer to the screen" step -- which is a compositor-level
    // concern this flag never touches.
    glDisable(GL_FRAMEBUFFER_SRGB);

    // Matches CanvasView.mm drawRect:'s own background fill color
    // ([NSColor colorWithCalibratedWhite:0.16 alpha:1.0]) so switching
    // between the CPU and GPU previews doesn't visibly flash a different
    // background.
    glClearColor(0.16f, 0.16f, 0.16f, 1.0f);
    _glSetUp = YES;

    // See -applySRGBWindowColorSpace's comment. Usually redundant with the
    // -viewDidMoveToWindow call below (that fires first in the normal
    // "add view to window, then it gets a live GL context" order), but
    // cheap, idempotent, and guards the case where this view is already
    // in a window by the time -prepareOpenGL runs.
    [self applySRGBWindowColorSpace];
}

- (void)viewDidMoveToWindow {
    [super viewDidMoveToWindow];
    [self applySRGBWindowColorSpace];
}

// Explicitly tags this window's backing store as sRGB, rather than
// whatever color space the window/display defaults to (frequently Display
// P3 on modern Macs). This is the fix for the reported "GPU preview looks
// washed out ON SCREEN, but a PNG saved via -renderToImageWithWidth:height:
// from the exact same draw call looks correctly saturated": the fragment
// shader writes sRGB-encoded bytes (same convention as every other
// renderer in this app -- see DocumentModel.mm's -renderReconstructionPreview
// comment on the same underlying "ambiguous/generic colorspace -> ColorSync
// picks a flatter fallback" bug class this project has now hit three
// times), but the raw swapped OpenGL buffer the window compositor presents
// carries no per-pixel color-space tag the way a CGImage does -- so
// without this, the compositor can interpret those bytes as though they
// were already in the window's native (often wider-gamut) space, muting
// the on-screen result relative to what the identical bytes decode to as
// sRGB, exactly matching the reported symptom and exactly explaining why
// only the on-screen path (not the read-back/export path, which tags its
// CGImage sRGB explicitly) was affected.
- (void)applySRGBWindowColorSpace {
    if (self.window && ![self.window.colorSpace isEqual:[NSColorSpace sRGBColorSpace]]) {
        self.window.colorSpace = [NSColorSpace sRGBColorSpace];
    }
}

- (void)reshape {
    [super reshape];
    if (!_glSetUp) return;
    [[self openGLContext] makeCurrentContext];
    NSRect bounds = [self convertRectToBacking:self.bounds];
    glViewport(0, 0, (GLsizei)bounds.size.width, (GLsizei)bounds.size.height);
}

- (void)uploadMeshBuffers:(GMGPUMeshBuffers*)buffers {
    if (!_glSetUp) {
        // -prepareOpenGL hasn't fired yet (view not yet in a window with a
        // live context) -- force it so an upload requested very early
        // (e.g. right after construction) isn't silently dropped.
        [[self openGLContext] makeCurrentContext];
        [self prepareOpenGL];
    }
    [[self openGLContext] makeCurrentContext];

    glBindBuffer(GL_TEXTURE_BUFFER, _tbo);
    glBufferData(GL_TEXTURE_BUFFER, (GLsizeiptr)buffers.vertexData.length, buffers.vertexData.bytes, GL_STATIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, _tboTexture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, _tbo);

    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _uvBuffer);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)buffers.uvTemplate.length, buffers.uvTemplate.bytes, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _indexBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)buffers.indexData.length, buffers.indexData.bytes, GL_STATIC_DRAW);

    _indexCount = (GLsizei)(buffers.indexData.length / sizeof(uint32_t));
    _cols = buffers.cols;
    _patchRows = buffers.patchRows;
    _patchCols = buffers.patchCols;
    _cieluv = buffers.cieluv;
    _meshMinX = buffers.minX; _meshMinY = buffers.minY;
    _meshMaxX = buffers.maxX; _meshMaxY = buffers.maxY;
    _hasMesh = (_patchRows > 0 && _patchCols > 0);

    [self setNeedsDisplay:YES];
}

- (void)clearMesh {
    _hasMesh = NO;
    [self setNeedsDisplay:YES];
}

// Shared by -drawRect: and -renderToImageWithWidth:height: -- issues the
// actual mesh draw call (assumes the target framebuffer is already bound
// and glClear'd, and the viewport already matches `size`). Pulled out so
// "what gets exported" is provably the exact same code as "what's shown
// on screen", not a separate reimplementation that could quietly drift.
- (void)drawMeshFittedToSize:(NSSize)size {
    if (!_hasMesh) return;

    glUseProgram(_program);
    glBindVertexArray(_vao);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, _tboTexture);
    glUniform1i(_uCols, (GLint)_cols);
    glUniform1i(_uPatchCols, (GLint)_patchCols);
    glUniform1i(_uCieluv, _cieluv ? 1 : 0);

    // Fit the mesh's geometry bounding box into `size`, preserving aspect
    // ratio and centering -- mirrors CanvasView's -imageDisplayRect
    // letterboxing so the GPU preview lines up visually with the CPU one.
    // Y is flipped (mesh/image space has row 0 at the top; NDC +Y is up),
    // matching CanvasView.mm drawRect:'s own explicit vertical-flip fix
    // for the CPU image.
    double meshW = MAX(1e-6, _meshMaxX - _meshMinX);
    double meshH = MAX(1e-6, _meshMaxY - _meshMinY);
    double scale = MIN(size.width / meshW, size.height / meshH);
    double ndcScaleX = scale / (size.width * 0.5);
    double ndcScaleY = scale / (size.height * 0.5);
    double centerX = (_meshMinX + _meshMaxX) * 0.5;
    double centerY = (_meshMinY + _meshMaxY) * 0.5;
    glUniform4f(_uXform, (GLfloat)ndcScaleX, (GLfloat)(-ndcScaleY), (GLfloat)centerX, (GLfloat)centerY);

    glDrawElementsInstanced(GL_TRIANGLES, _indexCount, GL_UNSIGNED_INT, NULL, (GLsizei)(_patchRows * _patchCols));
}

- (void)drawRect:(NSRect)dirtyRect {
    (void)dirtyRect;
    if (!_glSetUp) { [self prepareOpenGL]; }
    [[self openGLContext] makeCurrentContext];
    glClear(GL_COLOR_BUFFER_BIT);
    [self drawMeshFittedToSize:self.bounds.size];
    [[self openGLContext] flushBuffer];
}

- (nullable NSImage*)renderToImageWithWidth:(NSInteger)width height:(NSInteger)height {
    if (!_hasMesh || width <= 0 || height <= 0) return nil;
    if (!_glSetUp) { [self prepareOpenGL]; }
    [[self openGLContext] makeCurrentContext];

    // One-shot offscreen FBO at the EXPORT resolution -- deliberately not
    // tied to the on-screen view's (window-dependent) pixel size, so the
    // exported PNG matches the original loaded image's resolution instead
    // (see this method's header comment).
    GLuint fbo = 0, colorTex = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenTextures(1, &colorTex);
    glBindTexture(GL_TEXTURE_2D, colorTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, (GLsizei)width, (GLsizei)height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex, 0);

    NSImage* result = nil;
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
        glViewport(0, 0, (GLsizei)width, (GLsizei)height);
        glClearColor(0.16f, 0.16f, 0.16f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        [self drawMeshFittedToSize:NSMakeSize(width, height)];
        glFinish();

        size_t rowBytes = (size_t)width * 4;
        std::vector<uint8_t> rows(rowBytes * (size_t)height);
        glReadPixels(0, 0, (GLsizei)width, (GLsizei)height, GL_RGBA, GL_UNSIGNED_BYTE, rows.data());

        // glReadPixels' row 0 is the BOTTOM of the image (GL's Y-up
        // convention); flip to row-0-at-top to match every other image in
        // this app (see MeshRenderBuffers.h/GradientMesh.h's own
        // row-0-at-top convention) before handing it to CoreGraphics.
        std::vector<uint8_t> flipped(rows.size());
        for (NSInteger y = 0; y < height; ++y) {
            memcpy(&flipped[(size_t)y * rowBytes], &rows[(size_t)(height - 1 - y) * rowBytes], rowBytes);
        }

        // Explicit sRGB tag, NOT a generic/device color space -- see
        // DocumentModel.mm's -renderReconstructionPreview comment on
        // exactly why an untagged raster looks washed out on screen
        // (the same class of bug this project already hit once).
        CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        CGDataProviderRef provider = CGDataProviderCreateWithData(NULL, flipped.data(), flipped.size(), NULL);
        CGImageRef cgImage = CGImageCreate((size_t)width, (size_t)height, 8, 32, rowBytes, cs,
                                            kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big,
                                            provider, NULL, false, kCGRenderingIntentDefault);
        CGDataProviderRelease(provider);
        CGColorSpaceRelease(cs);
        if (cgImage) {
            result = [[NSImage alloc] initWithCGImage:cgImage size:NSMakeSize(width, height)];
            CGImageRelease(cgImage);
        }
    }

    // Restore the default framebuffer and the on-screen viewport so a
    // subsequent normal -drawRect: isn't left drawing into (or sized for)
    // this one-shot offscreen target.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    NSRect backingBounds = [self convertRectToBacking:self.bounds];
    glViewport(0, 0, (GLsizei)backingBounds.size.width, (GLsizei)backingBounds.size.height);
    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &colorTex);

    return result;
}

- (void)dealloc {
    if (_glSetUp) {
        [[self openGLContext] makeCurrentContext];
        if (_program) glDeleteProgram(_program);
        if (_vao) glDeleteVertexArrays(1, &_vao);
        if (_uvBuffer) glDeleteBuffers(1, &_uvBuffer);
        if (_indexBuffer) glDeleteBuffers(1, &_indexBuffer);
        if (_tbo) glDeleteBuffers(1, &_tbo);
        if (_tboTexture) glDeleteTextures(1, &_tboTexture);
    }
}

@end
#pragma clang diagnostic pop
