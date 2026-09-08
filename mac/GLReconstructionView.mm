// GLReconstructionView.mm — see GLReconstructionView.h for the design and
// the important "not yet built/run" caveat on this specific file.
#define GL_SILENCE_DEPRECATION 1  // Apple deprecated OpenGL in macOS 10.14 but it remains fully functional; scoped to this file only, not project-wide, since only this file touches the GL API directly.
#import "GLReconstructionView.h"
#include "gmcore/GLShaderSources.h"
#import <OpenGL/gl3.h>

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

    // Matches CanvasView.mm drawRect:'s own background fill color
    // ([NSColor colorWithCalibratedWhite:0.16 alpha:1.0]) so switching
    // between the CPU and GPU previews doesn't visibly flash a different
    // background.
    glClearColor(0.16f, 0.16f, 0.16f, 1.0f);
    _glSetUp = YES;
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

- (void)drawRect:(NSRect)dirtyRect {
    (void)dirtyRect;
    if (!_glSetUp) { [self prepareOpenGL]; }
    [[self openGLContext] makeCurrentContext];
    glClear(GL_COLOR_BUFFER_BIT);

    if (_hasMesh) {
        glUseProgram(_program);
        glBindVertexArray(_vao);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_BUFFER, _tboTexture);
        glUniform1i(_uCols, (GLint)_cols);
        glUniform1i(_uPatchCols, (GLint)_patchCols);
        glUniform1i(_uCieluv, _cieluv ? 1 : 0);

        // Fit the mesh's geometry bounding box into the view, preserving
        // aspect ratio and centering -- mirrors CanvasView's
        // -imageDisplayRect letterboxing so the GPU preview lines up
        // visually with the CPU one. Y is flipped (mesh/image space has
        // row 0 at the top; NDC +Y is up), matching CanvasView.mm
        // drawRect:'s own explicit vertical-flip fix for the CPU image.
        NSRect bounds = self.bounds;
        double meshW = MAX(1e-6, _meshMaxX - _meshMinX);
        double meshH = MAX(1e-6, _meshMaxY - _meshMinY);
        double scale = MIN(bounds.size.width / meshW, bounds.size.height / meshH);
        double ndcScaleX = scale / (bounds.size.width * 0.5);
        double ndcScaleY = scale / (bounds.size.height * 0.5);
        double centerX = (_meshMinX + _meshMaxX) * 0.5;
        double centerY = (_meshMinY + _meshMaxY) * 0.5;
        glUniform4f(_uXform, (GLfloat)ndcScaleX, (GLfloat)(-ndcScaleY), (GLfloat)centerX, (GLfloat)centerY);

        glDrawElementsInstanced(GL_TRIANGLES, _indexCount, GL_UNSIGNED_INT, NULL, (GLsizei)(_patchRows * _patchCols));
    }

    [[self openGLContext] flushBuffer];
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
