#import "DocumentModel.h"
#import <ImageIO/ImageIO.h>
#import <CoreGraphics/CoreGraphics.h>
#include "gmcore/Image.h"
#include "gmcore/GradientMesh.h"
#include "gmcore/MeshOptimizer.h"
#include "gmcore/SVGExporter.h"
#include "gmcore/ColorSpace.h"
#include <vector>
#include <array>
#include <memory>
#include <cmath>
#include <cstdint>
#include <algorithm>

using gmcore::Vec2;
using gmcore::Color;
using gmcore::CubicBezier;
using gmcore::GradientMesh;
using gmcore::Image;
using gmcore::VectorLine;
using gmcore::OptimizerOptions;
using gmcore::OptimizerProgress;

static NSError* gmError(NSString* msg) {
    return [NSError errorWithDomain:@"GradientMeshStudio" code:1 userInfo:@{NSLocalizedDescriptionKey: msg}];
}

// --- Debug-data export helpers (see -exportDebugDataToURL:error:) ---

// Finds the git repo root by walking up from THIS SOURCE FILE'S OWN
// compile-time path (__FILE__) looking for a .git directory/file (a git
// worktree's .git is a file, not a directory, hence -fileExistsAtPath:
// rather than checking isDirectory). This works because Xcode compiles
// this .mm from its real location in the working copy and __FILE__
// captures whatever absolute path was actually passed to the compiler --
// it does NOT depend on the app's runtime bundle location (which, for a
// built .app, has nothing to do with where the source/repo lives) or on
// any build-time-generated version header. Returns nil (not a hard
// failure) if no .git is found within a handful of parent directories --
// e.g. if this binary was ever built from a source tree copied out from
// under its .git, or moved after building. Callers must handle nil.
static NSString* gmFindRepoRootFromSourceFile(void) {
    NSString* dir = [[NSString stringWithUTF8String:__FILE__] stringByDeletingLastPathComponent];
    NSFileManager* fm = [NSFileManager defaultManager];
    for (int i = 0; i < 8 && dir.length > 1; ++i) {
        if ([fm fileExistsAtPath:[dir stringByAppendingPathComponent:@".git"]]) return dir;
        NSString* parent = [dir stringByDeletingLastPathComponent];
        if ([parent isEqualToString:dir]) break;
        dir = parent;
    }
    return nil;
}

// Runs `git -C repoRoot <args>` and returns trimmed stdout, or nil on any
// failure (repoRoot nil, git not found, non-zero exit, ...) -- deliberately
// forgiving since this only feeds an informational debug dump, never
// something the app's own correctness depends on.
static NSString* gmRunGit(NSString* repoRoot, NSArray<NSString*>* args) {
    if (!repoRoot) return nil;
    NSTask* task = [[NSTask alloc] init];
    task.launchPath = @"/usr/bin/env";
    NSMutableArray<NSString*>* full = [NSMutableArray arrayWithObjects:@"git", @"-C", repoRoot, nil];
    [full addObjectsFromArray:args];
    task.arguments = full;
    NSPipe* outPipe = [NSPipe pipe];
    task.standardOutput = outPipe;
    task.standardError = [NSPipe pipe]; // discarded
    @try {
        [task launch];
    } @catch (NSException* __unused exc) {
        return nil;
    }
    NSData* data = [outPipe.fileHandleForReading readDataToEndOfFile];
    [task waitUntilExit];
    if (task.terminationStatus != 0) return nil;
    NSString* out = [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding];
    return [out stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
}

@interface DocumentModel () {
    gmcore::Image _target;
    // CIELUV conversion of _target, recomputed alongside it in
    // -loadImageAtURL:error: (see -workingTargetImage and
    // useCIELUVColorSpace's doc comment in DocumentModel.h). Computed
    // eagerly (a one-time, one-per-image-load cost) rather than lazily so
    // there's no first-use stall/branch to reason about.
    gmcore::Image _targetLUV;
    BOOL _hasImage;
    std::vector<Vec2> _boundaryPolygon;
    std::array<CubicBezier, 4> _boundary;
    BOOL _hasBoundary;
    std::unique_ptr<GradientMesh> _mesh;
    std::vector<VectorLine> _vectorLines;
    BOOL _isOptimizing;
    double _lastRMSE;
    // Which colour space _mesh's C/Cu/Cv/Cuv are actually stored in --
    // snapshotted from self.useCIELUVColorSpace at -buildInitialMeshRows:
    // cols: time (see that method) and used everywhere _mesh's colours are
    // read/written from then on, INSTEAD OF re-reading the live
    // useCIELUVColorSpace property. This is deliberate: it's what makes
    // toggling the property after a mesh already exists harmless (a no-op
    // until the next rebuild) rather than a silent colour-space mismatch
    // between the mesh and whatever -optimizeWithPyramidLevels:... would
    // otherwise feed it.
    BOOL _meshColorSpaceIsCIELUV;
    // State captured for -exportDebugDataToURL:error: (see DocumentModel.h)
    // -- the exact OptimizerOptions the most recent
    // -optimizeWithPyramidLevels:progress:completion: call used, its full
    // per-outer-iteration progress history, whether any run has happened
    // yet, and how long it took wall-clock. Snapshotted/reset at the START
    // of each optimize call (see that method), so a debug export mid-run
    // or right after reflects the run actually in flight/just finished.
    OptimizerOptions _lastOptsUsed;
    std::vector<OptimizerProgress> _lastRunHistory;
    BOOL _hasRunOptimize;
    double _lastRunWallClockSeconds;
    // URL of the currently loaded image (set in -loadImageAtURL:error:) --
    // used only to locate the "DebugOut" folder for -autoExportDebugData
    // (see DocumentModel.h), sibling to wherever the image actually lives.
    NSURL* _imageURL;
}
@property (nonatomic, strong, nullable) NSImage* displayImage;
@property (nonatomic, strong, nullable) NSString* lastDebugExportPath;
// The Image that -buildInitialMeshRows:cols:/-optimizeWithPyramidLevels:...
// should actually fit against: _target (sRGB) or _targetLUV (CIELUV),
// chosen by the LIVE useCIELUVColorSpace property. Only ever consulted at
// -buildInitialMeshRows:cols: time -- see that method and
// _meshColorSpaceIsCIELUV's comment above for why.
- (const gmcore::Image&)workingTargetImage;
@end

@implementation DocumentModel

- (BOOL)hasImage { return _hasImage; }
- (NSInteger)imageWidth { return _hasImage ? _target.width : 0; }
- (NSInteger)imageHeight { return _hasImage ? _target.height : 0; }
- (BOOL)hasBoundary { return _hasBoundary; }
- (BOOL)hasMesh { return _mesh != nullptr; }
- (BOOL)isOptimizing { return _isOptimizing; }
- (NSInteger)meshRows { return _mesh ? _mesh->rows : 0; }
- (NSInteger)meshCols { return _mesh ? _mesh->cols : 0; }
- (double)currentRMSE { return _lastRMSE; }
- (BOOL)meshColorSpaceIsCIELUV { return _mesh ? _meshColorSpaceIsCIELUV : NO; }

#pragma mark - Image loading

- (BOOL)loadImageAtURL:(NSURL*)url error:(NSError**)error {
    CGImageSourceRef src = CGImageSourceCreateWithURL((__bridge CFURLRef)url, NULL);
    if (!src) { if (error) *error = gmError(@"Could not open image file."); return NO; }
    CGImageRef cgImage = CGImageSourceCreateImageAtIndex(src, 0, NULL);
    CFRelease(src);
    if (!cgImage) { if (error) *error = gmError(@"Could not decode image."); return NO; }

    size_t w = CGImageGetWidth(cgImage), h = CGImageGetHeight(cgImage);
    if (w == 0 || h == 0) { CGImageRelease(cgImage); if (error) *error = gmError(@"Image has zero size."); return NO; }

    std::vector<uint8_t> buffer(w * h * 4, 0);
    // Explicit sRGB, NOT CGColorSpaceCreateDeviceRGB() -- see
    // -renderReconstructionPreview's comment for the full story (a
    // reported "reconstruction looks washed out on screen, but the
    // exported PNG looks correctly saturated" bug traced to exactly this
    // call using the ambiguous/legacy "device RGB" space instead of a
    // named, unambiguous one). Used here too for the same reason and for
    // consistency: this is the conversion that turns the source file's
    // bytes (whatever color space THEY were tagged in, e.g. a wide-gamut
    // Display P3 photo) into _target's raw numeric buffer, and every
    // consumer of that buffer (the optimizer's data term, and
    // -renderReconstructionPreview's render of the fitted mesh) should be
    // working in the same well-defined color space as what actually gets
    // displayed and exported, not an ambiguous one.
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef ctx = CGBitmapContextCreate(buffer.data(), w, h, 8, w * 4, cs,
                                              kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    CGColorSpaceRelease(cs);
    if (!ctx) { CGImageRelease(cgImage); if (error) *error = gmError(@"Could not create bitmap context."); return NO; }
    CGContextSetBlendMode(ctx, kCGBlendModeCopy);
    // NO flip here -- confirmed on-device via the "[GMCORE _target
    // orientation check]" NSLog below: a plain CGContextDrawImage into a
    // freshly created (non-view-backed) CGBitmapContext already puts row 0
    // of `buffer` at the image's true TOP row. An earlier version of this
    // code added a CGContextTranslateCTM/CGContextScaleCTM(1,-1) flip here,
    // reasoning (by analogy with the well-known "CGContextDrawImage draws
    // upside down" gotcha, which is real but applies to a DIFFERENT
    // scenario -- see below) that it was needed; the diagnostic log proved
    // that reasoning backwards -- loading gradient.png with that flip in
    // place printed TL=blue/TR=yellow/BL=red/BR=green, i.e. top and bottom
    // swapped relative to the file's real corners (TL=red/TR=green/
    // BL=blue/BR=yellow). Removing the flip is what actually matches.
    //
    // For anyone re-deriving this: the classic gotcha the earlier comment
    // invoked is real, but it's specifically about drawing into a context
    // that ALREADY has a flip applied to its CTM by something else (most
    // commonly an AppKit view with isFlipped=YES compensating for its own
    // top-left/y-down convention -- see CanvasView.mm's -drawRect:, which
    // had exactly that bug for its NSImage draw and needed exactly that
    // fix). A bare CGBitmapContextCreate context here has no such
    // pre-existing flip to compensate for, so adding one manually
    // over-corrected. Every other part of this codebase (Image::loadPNG/
    // loadPPM in core/src/Image.cpp, GradientMesh's boundary/vertex
    // construction, CanvasView's click mapping, SVGExporter) already uses
    // "row 0 / y=0 = top, y grows down" -- this now matches them too.
    CGContextDrawImage(ctx, CGRectMake(0, 0, w, h), cgImage);

    Image img((int)w, (int)h);
    for (size_t y = 0; y < h; ++y) {
        for (size_t x = 0; x < w; ++x) {
            const uint8_t* px = &buffer[(y * w + x) * 4];
            img.set((int)x, (int)y, Color(px[0] / 255.0, px[1] / 255.0, px[2] / 255.0));
        }
    }
    CGContextRelease(ctx);
    CGImageRelease(cgImage);

    _target = std::move(img);
    // See useCIELUVColorSpace's doc comment in DocumentModel.h: computed
    // eagerly here, once per image load, regardless of whether CIELUV mode
    // is currently on -- so flipping the checkbox and clicking "Build
    // Initial Mesh" never has to wait on (or forget to trigger) a
    // conversion pass.
    _targetLUV = gmcore::imageSRGBToCIELUV(_target);

    // Diagnostic: log _target's 4 corner colors so orientation can be
    // checked directly against a known test image (e.g. a corner-colored
    // gradient.png) rather than inferred indirectly from how the mesh looks
    // after optimizing -- added after a report that, even after fixing the
    // on-screen display flip (see CanvasView.mm's -drawRect:), the
    // OPTIMIZED mesh still looked wrong, raising the question of whether
    // THIS flip (the CGContextDrawImage one, below) is actually correct or
    // was itself backwards. _target.at(0,0) should equal the image's real
    // top-left pixel -- for gradient.png specifically, that's red
    // (~254,1,1); top-right ~green, bottom-left ~blue, bottom-right
    // ~yellow, per the file's own actual pixel data (confirmed with an
    // independent tool, outside this app). Check Xcode's debug console
    // after loading gradient.png and compare.
    {
        int iw = (int)w, ih = (int)h;
        Color tl = _target.at(0, 0);
        Color tr = _target.at(iw - 1, 0);
        Color bl = _target.at(0, ih - 1);
        Color br = _target.at(iw - 1, ih - 1);
        NSLog(@"[GMCORE _target orientation check] TL=(%.0f,%.0f,%.0f) TR=(%.0f,%.0f,%.0f) "
              @"BL=(%.0f,%.0f,%.0f) BR=(%.0f,%.0f,%.0f) -- expect TL=red TR=green BL=blue BR=yellow "
              @"for gradient.png",
              tl.r * 255, tl.g * 255, tl.b * 255, tr.r * 255, tr.g * 255, tr.b * 255,
              bl.r * 255, bl.g * 255, bl.b * 255, br.r * 255, br.g * 255, br.b * 255);
    }

    _imageURL = url; // see -autoExportDebugDataIfEnabled's use of this
    _hasImage = YES;
    _hasBoundary = NO;
    _mesh.reset();
    _vectorLines.clear();
    _boundaryPolygon.clear();
    self.displayImage = [[NSImage alloc] initWithContentsOfURL:url];
    return YES;
}

#pragma mark - Boundary

- (void)setBoundaryPolygonPoints:(NSArray<NSValue*>*)points {
    _boundaryPolygon.clear();
    for (NSValue* v in points) {
        NSPoint p = [v pointValue];
        _boundaryPolygon.push_back(Vec2(p.x, p.y));
    }
    _hasBoundary = NO;
}

- (NSArray<NSValue*>*)boundaryPolygonPoints {
    NSMutableArray* arr = [NSMutableArray arrayWithCapacity:_boundaryPolygon.size()];
    for (auto& p : _boundaryPolygon) [arr addObject:[NSValue valueWithPoint:NSMakePoint(p.x, p.y)]];
    return arr;
}

- (BOOL)fitBoundaryWithCornerIndices:(NSArray<NSNumber*>*)fourIndices {
    if (fourIndices.count != 4 || _boundaryPolygon.size() < 4) return NO;
    int idx[4];
    for (int i = 0; i < 4; ++i) idx[i] = [fourIndices[i] intValue];
    int n = (int)_boundaryPolygon.size();
    for (int side = 0; side < 4; ++side) {
        int a = idx[side], b = idx[(side + 1) % 4];
        std::vector<Vec2> pts;
        int i = a;
        while (true) {
            pts.push_back(_boundaryPolygon[i]);
            if (i == b) break;
            i = (i + 1) % n;
        }
        _boundary[side] = gmcore::fitCubicBezier(pts);
    }
    _hasBoundary = YES;
    return YES;
}

- (void)useRectangularBoundaryWithMargin:(double)marginPixels {
    if (!_hasImage) return;
    double x0 = marginPixels, y0 = marginPixels;
    double x1 = _target.width - marginPixels, y1 = _target.height - marginPixels;
    _boundary[0] = {Vec2(x0, y0), Vec2(x0 + (x1 - x0) / 3, y0), Vec2(x0 + 2 * (x1 - x0) / 3, y0), Vec2(x1, y0)};
    _boundary[1] = {Vec2(x1, y0), Vec2(x1, y0 + (y1 - y0) / 3), Vec2(x1, y0 + 2 * (y1 - y0) / 3), Vec2(x1, y1)};
    _boundary[2] = {Vec2(x1, y1), Vec2(x0 + 2 * (x1 - x0) / 3, y1), Vec2(x0 + (x1 - x0) / 3, y1), Vec2(x0, y1)};
    _boundary[3] = {Vec2(x0, y1), Vec2(x0, y0 + 2 * (y1 - y0) / 3), Vec2(x0, y0 + (y1 - y0) / 3), Vec2(x0, y0)};
    _hasBoundary = YES;
    _boundaryPolygon = {Vec2(x0, y0), Vec2(x1, y0), Vec2(x1, y1), Vec2(x0, y1)};
}

#pragma mark - Mesh

- (const gmcore::Image&)workingTargetImage {
    return self.useCIELUVColorSpace ? _targetLUV : _target;
}

- (void)buildInitialMeshRows:(NSInteger)rows cols:(NSInteger)cols {
    if (!_hasImage || !_hasBoundary) return;
    // Snapshot NOW, at build time -- everything downstream (this run's
    // -optimizeWithPyramidLevels:..., -renderReconstructionPreview,
    // -exportSVGToURL:, -exportDebugDataToURL:, the vertex color
    // swatch/picker) reads THIS, not the live property, so a later toggle
    // of useCIELUVColorSpace can never mismatch against what's actually
    // stored in _mesh until the next rebuild. See its declaration's comment.
    _meshColorSpaceIsCIELUV = self.useCIELUVColorSpace;
    const gmcore::Image& target = [self workingTargetImage];
    _mesh = std::make_unique<GradientMesh>(GradientMesh::buildInitial((int)rows, (int)cols, _boundary, target));
    _lastRMSE = _mesh->reconstructionRMSE(target, 6);
}

- (NSPoint)meshVertexPositionAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return NSZeroPoint;
    Vec2 p = _mesh->at((int)row, (int)col).P;
    return NSMakePoint(p.x, p.y);
}

- (NSColor*)meshVertexColorAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return [NSColor blackColor];
    Color c = _mesh->at((int)row, (int)col).C;
    // _mesh->C is only actually sRGB when _meshColorSpaceIsCIELUV is NO --
    // see that ivar's comment and ColorSpace.h. clamped01() must run AFTER
    // this conversion: it assumes an sRGB-range [0,1] triple, and would
    // silently mangle a raw CIELUV (L* up to 100, u*/v* often negative)
    // value if applied first.
    if (_meshColorSpaceIsCIELUV) c = gmcore::cieluvToSRGB(c);
    c = c.clamped01();
    return [NSColor colorWithCalibratedRed:c.r green:c.g blue:c.b alpha:1.0];
}

- (void)setMeshVertexPosition:(NSPoint)p atRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return;
    _mesh->at((int)row, (int)col).P = Vec2(p.x, p.y);
}

- (void)setMeshVertexColor:(NSColor*)color atRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return;
    NSColor* rgb = [color colorUsingColorSpace:[NSColorSpace deviceRGBColorSpace]];
    Color c(rgb.redComponent, rgb.greenComponent, rgb.blueComponent);
    // The color picker always hands back sRGB; convert INTO whichever space
    // _mesh actually stores (see _meshColorSpaceIsCIELUV's comment) so a
    // manual edit stays consistent with every other vertex's C field.
    if (_meshColorSpaceIsCIELUV) c = gmcore::srgbToCIELUV(c);
    _mesh->at((int)row, (int)col).C = c;
}

- (BOOL)findNearestVertexToPoint:(NSPoint)p maxDistance:(double)maxDist row:(NSInteger*)outRow col:(NSInteger*)outCol {
    if (!_mesh) return NO;
    double best = maxDist * maxDist;
    BOOL found = NO;
    for (int r = 0; r < _mesh->rows; ++r) {
        for (int c = 0; c < _mesh->cols; ++c) {
            Vec2 v = _mesh->at(r, c).P;
            double dx = v.x - p.x, dy = v.y - p.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best) { best = d2; *outRow = r; *outCol = c; found = YES; }
        }
    }
    return found;
}

- (NSArray<NSValue*>*)meshEdgeBezierFromRow:(NSInteger)r0 col:(NSInteger)c0
                                       toRow:(NSInteger)r1 col:(NSInteger)c1 {
    if (!_mesh) return @[];
    Vec2 P0 = _mesh->at((int)r0, (int)c0).P;
    Vec2 P1 = _mesh->at((int)r1, (int)c1).P;
    Vec2 T0, T1;
    if (r0 == r1 && c1 == c0 + 1) {
        // horizontal edge: position varies with col (u); the relevant
        // derivative is the FREE Pu at each end (Pu/Pv are optimized
        // unknowns now, not derived -- see GradientMesh.h/MeshOptimizer.h).
        T0 = _mesh->at((int)r0, (int)c0).Pu;
        T1 = _mesh->at((int)r1, (int)c1).Pu;
    } else if (c0 == c1 && r1 == r0 + 1) {
        // vertical edge: position varies with row (v); the free Pv at each end.
        T0 = _mesh->at((int)r0, (int)c0).Pv;
        T1 = _mesh->at((int)r1, (int)c1).Pv;
    } else {
        return @[]; // not a grid-adjacent pair
    }
    Vec2 B0 = P0;
    Vec2 B1 = P0 + T0 * (1.0 / 3.0);
    Vec2 B2 = P1 - T1 * (1.0 / 3.0);
    Vec2 B3 = P1;
    return @[
        [NSValue valueWithPoint:NSMakePoint(B0.x, B0.y)],
        [NSValue valueWithPoint:NSMakePoint(B1.x, B1.y)],
        [NSValue valueWithPoint:NSMakePoint(B2.x, B2.y)],
        [NSValue valueWithPoint:NSMakePoint(B3.x, B3.y)],
    ];
}

- (NSArray<NSArray<NSValue*>*>*)fittedBoundaryCurves {
    if (!_hasBoundary) return @[];
    NSMutableArray<NSArray<NSValue*>*>* out = [NSMutableArray arrayWithCapacity:4];
    for (int i = 0; i < 4; ++i) {
        const CubicBezier& b = _boundary[i];
        [out addObject:@[
            [NSValue valueWithPoint:NSMakePoint(b.p0.x, b.p0.y)],
            [NSValue valueWithPoint:NSMakePoint(b.p1.x, b.p1.y)],
            [NSValue valueWithPoint:NSMakePoint(b.p2.x, b.p2.y)],
            [NSValue valueWithPoint:NSMakePoint(b.p3.x, b.p3.y)],
        ]];
    }
    return out;
}

- (NSPoint)meshVertexTangentUAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return NSZeroPoint;
    Vec2 t = _mesh->at((int)row, (int)col).Pu; // free unknown, not derived
    return NSMakePoint(t.x, t.y);
}

- (NSPoint)meshVertexTangentVAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return NSZeroPoint;
    Vec2 t = _mesh->at((int)row, (int)col).Pv; // free unknown, not derived
    return NSMakePoint(t.x, t.y);
}

#pragma mark - Vector lines

- (void)addVectorLineWithPoints:(NSArray<NSValue*>*)points {
    if (points.count < 2) return;
    VectorLine line;
    for (NSValue* v in points) {
        NSPoint p = [v pointValue];
        line.points.push_back(Vec2(p.x, p.y));
    }
    _vectorLines.push_back(std::move(line));
}

- (void)removeLastVectorLine {
    if (!_vectorLines.empty()) _vectorLines.pop_back();
}

- (void)clearVectorLines { _vectorLines.clear(); }

- (NSArray<NSArray<NSValue*>*>*)vectorLinesPoints {
    NSMutableArray* lines = [NSMutableArray arrayWithCapacity:_vectorLines.size()];
    for (auto& line : _vectorLines) {
        NSMutableArray* pts = [NSMutableArray arrayWithCapacity:line.points.size()];
        for (auto& p : line.points) [pts addObject:[NSValue valueWithPoint:NSMakePoint(p.x, p.y)]];
        [lines addObject:pts];
    }
    return lines;
}

#pragma mark - Optimize

- (void)optimizeWithPyramidLevels:(NSInteger)levels
                          progress:(void (^)(double, NSInteger, NSInteger, NSInteger, NSInteger))progress
                        completion:(void (^)(void))completion {
    if (!_mesh || _isOptimizing) { if (completion) completion(); return; }
    _isOptimizing = YES;

    // MeshOptimizer mutates the mesh's std::vector storage; keep the mesh
    // pointer stable and avoid touching it from the main thread while this
    // runs (CanvasView checks isOptimizing before reading mesh geometry).
    GradientMesh* meshPtr = _mesh.get();
    // Must match the color space _mesh's C/Cu/Cv/Cuv were actually BUILT in
    // (_meshColorSpaceIsCIELUV, snapshotted at -buildInitialMeshRows:cols:
    // time), NOT the live useCIELUVColorSpace property -- see that ivar's
    // comment. Using the wrong one here would feed the optimizer's data
    // term a target in a different space than the mesh colors it's
    // comparing against, silently corrupting every run.
    Image targetCopy = _meshColorSpaceIsCIELUV ? _targetLUV : _target; // Image is a small value type wrapping a vector; a private copy for thread safety.
    std::vector<VectorLine> linesCopy = _vectorLines;
    OptimizerOptions opts;
    // See DocumentModel.h's comment on these two properties: "joint wins" if
    // both are set, mirroring MeshOptimizer.cpp's own jointSolvedByCeres
    // gating and the CLI's --use-ceres/--use-ceres-joint precedence.
    opts.useCeresGeometry = self.useCeresGeometry;
    opts.useCeresJoint = self.useCeresJoint;
    // 0/unset -> 1: see DocumentModel.h's comment on this property.
    opts.pyramidRestarts = (int)std::max((NSInteger)1, self.pyramidRestarts);

    // Snapshot the exact opts this run uses and reset the per-run progress
    // log/timer, for -exportDebugDataToURL:error: -- must happen here, on
    // the main thread, before the background dispatch below, not inside it,
    // so a debug export triggered mid-run (or right after) always reflects
    // THIS run's actual settings, not a stale opts struct left over from an
    // earlier call with different solver toggles.
    _lastOptsUsed = opts;
    _lastRunHistory.clear();
    _hasRunOptimize = YES;
    NSDate* runStart = [NSDate date];

    dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
        gmcore::MeshOptimizer::optimizeCoarseToFine(*meshPtr, targetCopy, linesCopy, (int)levels, opts,
            [progress, self](const OptimizerProgress& p) {
                dispatch_async(dispatch_get_main_queue(), ^{
                    // Recorded unconditionally, even if the caller passed a
                    // nil UI progress block -- this is the history
                    // -exportDebugDataToURL:error: dumps, independent of
                    // whether anything was listening for live UI updates.
                    self->_lastRunHistory.push_back(p);
                    if (progress) progress(p.rmse, p.pyramidLevel, p.totalPyramidLevels, p.outerIteration, p.totalOuterIterations);
                });
            });
        double finalRmse = meshPtr->reconstructionRMSE(targetCopy, 6);
        dispatch_async(dispatch_get_main_queue(), ^{
            self->_isOptimizing = NO;
            self->_lastRMSE = finalRmse;
            self->_lastRunWallClockSeconds = -[runStart timeIntervalSinceNow];
            // Must run AFTER the state above is updated (it dumps
            // _lastRMSE/_lastRunHistory/_lastRunWallClockSeconds) but
            // BEFORE completion(), so a completion handler that checks
            // -lastDebugExportPath (see MainWindowController's -optimize:)
            // sees this run's result, not a stale one from before it.
            [self autoExportDebugDataIfEnabled];
            if (completion) completion();
        });
    });
}

#pragma mark - Debug data auto-export

// Sibling "DebugOut" folder next to the currently loaded image, creating it
// if needed. Returns nil if there's no loaded image to be a sibling of, or
// if the folder couldn't be created (e.g. the image's folder is read-only).
- (nullable NSURL*)debugOutDirectoryURL {
    if (!_imageURL) return nil;
    NSURL* dir = [[_imageURL URLByDeletingLastPathComponent] URLByAppendingPathComponent:@"DebugOut" isDirectory:YES];
    NSError* mkdirErr = nil;
    if (![[NSFileManager defaultManager] createDirectoryAtURL:dir withIntermediateDirectories:YES
                                                     attributes:nil error:&mkdirErr]) {
        NSLog(@"[GMCORE] autoExportDebugData: could not create DebugOut folder at %@: %@", dir, mkdirErr);
        return nil;
    }
    return dir;
}

// "gm_debug_<solver>[_cieluv]_<rows>x<cols>_<timestamp>.json" -- same
// <solver>_<rows>x<cols> convention as the old manual export's default
// filename, plus a millisecond-resolution local timestamp (fixed
// yyyyMMdd-HHmmss-SSS format, en_US_POSIX locale so it can't come out
// looking different on a machine set to another locale/calendar) so
// repeated runs pile up in DebugOut side by side instead of each one
// silently overwriting the last -- the whole point of turning this into an
// always-on checkbox is comparing a SEQUENCE of runs after the fact. The
// "_cieluv" tag (see the JSON body's own "colorSpace" field, the actual
// source of truth) is deliberately visible in the filename too: a CIELUV
// run's lastRMSE is in different units than an sRGB run's (see
// useCIELUVColorSpace's doc comment) and this project has already once
// mixed up two RMSE numbers that looked comparable but weren't (see
// README) -- worth avoiding a repeat by making the units visible at a
// glance, not just inside the file.
- (NSString*)debugExportFilename {
    NSString* solverTag = self.useCeresJoint ? @"ceres_joint" : self.useCeresGeometry ? @"ceres_geom" : @"hand_rolled";
    NSString* colorSpaceTag = _meshColorSpaceIsCIELUV ? @"_cieluv" : @"";
    static NSDateFormatter* fmt;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        fmt = [[NSDateFormatter alloc] init];
        fmt.locale = [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"];
        fmt.dateFormat = @"yyyyMMdd-HHmmss-SSS";
    });
    NSString* stamp = [fmt stringFromDate:[NSDate date]];
    return [NSString stringWithFormat:@"gm_debug_%@%@_%ldx%ld_%@.json", solverTag, colorSpaceTag,
            (long)self.meshRows, (long)self.meshCols, stamp];
}

// See DocumentModel.h's comment on autoExportDebugData. Called at the end
// of every -optimizeWithPyramidLevels:progress:completion: run (regardless
// of outcome -- there's always a mesh by that point, since the method
// bails out early if there wasn't one to begin with).
- (void)autoExportDebugDataIfEnabled {
    if (!self.autoExportDebugData || !_mesh) return;
    NSURL* dir = [self debugOutDirectoryURL];
    if (!dir) { self.lastDebugExportPath = nil; return; }
    NSURL* fileURL = [dir URLByAppendingPathComponent:[self debugExportFilename]];
    NSError* error = nil;
    if ([self exportDebugDataToURL:fileURL error:&error]) {
        self.lastDebugExportPath = fileURL.path;
    } else {
        NSLog(@"[GMCORE] autoExportDebugData: export failed: %@", error);
        self.lastDebugExportPath = nil;
    }
}

#pragma mark - Output

- (NSImage*)renderReconstructionPreview {
    if (!_mesh || !_hasImage) return nil;
    Image rendered = _mesh->render((int)_target.width, (int)_target.height, 8);
    // _mesh->render() fully patch-interpolates (Sec 3's Ferguson patches),
    // producing per-pixel POINT VALUES throughout -- so, unlike Cu/Cv/Cuv,
    // the raster it returns is safe/correct to convert wholesale. Must run
    // before the clamped01() loop below (see that call's own note on why).
    if (_meshColorSpaceIsCIELUV) rendered = gmcore::imageCIELUVToSRGB(rendered);
    int w = rendered.width, h = rendered.height;
    std::vector<uint8_t> buffer(size_t(w) * h * 4, 255);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            Color c = rendered.at(x, y).clamped01();
            uint8_t* px = &buffer[(size_t(y) * w + x) * 4];
            px[0] = (uint8_t)std::lround(c.r * 255.0);
            px[1] = (uint8_t)std::lround(c.g * 255.0);
            px[2] = (uint8_t)std::lround(c.b * 255.0);
            px[3] = 255;
        }
    }
    // Explicit sRGB, NOT CGColorSpaceCreateDeviceRGB() -- fixes a reported
    // bug: the reconstruction preview looked visibly "washed out"/less
    // saturated on screen in CanvasView, while exporting it to a PNG via
    // -exportPNGToURL: (which rasterizes this SAME NSImage/CGImage) and
    // reopening that file showed the correct, fully-saturated colors. Both
    // paths draw the exact same numeric RGB bytes -- CGColorSpaceCreateDeviceRGB()
    // produces an untagged/"generic device" color space, which is legacy
    // CoreGraphics terminology, not a request to bypass color management.
    // On-screen, AppKit/ColorSync has to pick SOME interpretation for that
    // ambiguous tag when compositing into the window's (wide-gamut,
    // typically Display P3) backing store, and can fall back to an old
    // "Generic RGB" profile with a flatter, less saturated response than
    // sRGB; a PNG viewer opening the exported file, on the other hand,
    // treats an untagged/generic image as sRGB by convention (the modern
    // default assumption), which is why the file looked correct while the
    // live view did not. Explicitly naming the space as sRGB removes the
    // ambiguity at the source instead of relying on inconsistent fallback
    // behavior in two different code paths.
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef ctx = CGBitmapContextCreate(buffer.data(), w, h, 8, w * 4, cs,
                                              kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    CGColorSpaceRelease(cs);
    CGImageRef cgImage = CGBitmapContextCreateImage(ctx);
    CGContextRelease(ctx);
    NSImage* img = [[NSImage alloc] initWithCGImage:cgImage size:NSMakeSize(w, h)];
    CGImageRelease(cgImage);
    return img;
}

- (BOOL)exportPNGToURL:(NSURL*)url error:(NSError**)error {
    NSImage* preview = [self renderReconstructionPreview];
    if (!preview) { if (error) *error = gmError(@"No mesh to render yet."); return NO; }
    CGImageRef cgImage = [preview CGImageForProposedRect:NULL context:nil hints:nil];
    if (!cgImage) { if (error) *error = gmError(@"Could not rasterize preview."); return NO; }
    CGImageDestinationRef dest = CGImageDestinationCreateWithURL((__bridge CFURLRef)url, CFSTR("public.png"), 1, NULL);
    if (!dest) { if (error) *error = gmError(@"Could not create PNG file."); return NO; }
    CGImageDestinationAddImage(dest, cgImage, NULL);
    BOOL ok = CGImageDestinationFinalize(dest);
    CFRelease(dest);
    if (!ok && error) *error = gmError(@"Could not write PNG file.");
    return ok;
}

- (BOOL)exportSVGToURL:(NSURL*)url error:(NSError**)error {
    if (!_mesh) { if (error) *error = gmError(@"No mesh to export yet."); return NO; }
    std::string svg = gmcore::exportGradientMeshSVG(*_mesh, _target.width, _target.height, _meshColorSpaceIsCIELUV);
    NSString* str = [NSString stringWithUTF8String:svg.c_str()];
    NSError* writeErr = nil;
    BOOL ok = [str writeToURL:url atomically:YES encoding:NSUTF8StringEncoding error:&writeErr];
    if (!ok && error) *error = writeErr ?: gmError(@"Could not write SVG file.");
    return ok;
}

- (BOOL)exportDebugDataToURL:(NSURL*)url error:(NSError**)error {
    if (!_mesh) { if (error) *error = gmError(@"No mesh to export debug data for yet."); return NO; }

    NSMutableDictionary* root = [NSMutableDictionary dictionary];

    if (@available(macOS 10.12, *)) {
        NSISO8601DateFormatter* iso = [[NSISO8601DateFormatter alloc] init];
        root[@"exportedAt"] = [iso stringFromDate:[NSDate date]];
    }

    NSString* repoRoot = gmFindRepoRootFromSourceFile();
    NSString* commit = gmRunGit(repoRoot, @[@"rev-parse", @"HEAD"]) ?: @"unknown";
    NSString* describe = gmRunGit(repoRoot, @[@"describe", @"--always", @"--dirty", @"--long"]) ?: @"unknown";
    NSString* porcelain = gmRunGit(repoRoot, @[@"status", @"--porcelain"]);
    root[@"git"] = @{
        @"repoRootFound": @(repoRoot != nil),
        @"commit": commit,
        @"describe": describe,
        @"dirty": @(porcelain != nil && porcelain.length > 0),
    };

    // Which solver was REQUESTED (the UI toggle) vs. which one actually ran
    // -- these can differ silently: useCeresGeometry/useCeresJoint are
    // no-ops (with only a one-time stderr warning, easy to miss) in a
    // build without Ceres found. This is exactly the ambiguity that made
    // an earlier SVG-only analysis unreliable -- see gmcore::builtWithCeres().
    BOOL builtWithCeres = gmcore::builtWithCeres();
    NSString* requestedSolver = self.useCeresJoint ? @"ceresJoint"
                               : self.useCeresGeometry ? @"ceresGeometry" : @"handRolled";
    NSString* effectiveSolver = builtWithCeres ? requestedSolver : @"handRolled";
    root[@"solver"] = @{
        @"builtWithCeres": @(builtWithCeres),
        @"requested": requestedSolver,
        @"effective": effectiveSolver,
    };

    // Which colour space _mesh's C/Cu/Cv/Cuv (below) and lastRMSE are
    // actually in -- see useCIELUVColorSpace's doc comment in
    // DocumentModel.h and ColorSpace.h. Not present in JSONs exported
    // before this field existed; treat its absence there as "sRGB" (the
    // only space that existed at the time). lastRMSE from a "CIELUV" export
    // is measured in CIELUV units (L* roughly 0..100) and is NOT directly
    // comparable by raw number to an "sRGB" export's lastRMSE.
    root[@"colorSpace"] = _meshColorSpaceIsCIELUV ? @"CIELUV" : @"sRGB";

    root[@"image"] = @{ @"width": @(_target.width), @"height": @(_target.height) };
    // Same metric, same sample density (6) the UI's status label and the
    // filename-embedded RMSE convention already use -- see
    // -optimizeWithPyramidLevels:progress:completion:'s finalRmse.
    root[@"lastRMSE"] = @(_lastRMSE);
    root[@"pyramidRestarts"] = @(self.pyramidRestarts);
    root[@"hasRunOptimize"] = @(_hasRunOptimize);
    root[@"lastRunWallClockSeconds"] = @(_lastRunWallClockSeconds);

    NSMutableArray<NSDictionary*>* history = [NSMutableArray arrayWithCapacity:_lastRunHistory.size()];
    for (const auto& p : _lastRunHistory) {
        [history addObject:@{
            @"pyramidLevel": @(p.pyramidLevel),
            @"totalPyramidLevels": @(p.totalPyramidLevels),
            @"outerIteration": @(p.outerIteration),
            @"totalOuterIterations": @(p.totalOuterIterations),
            @"rmse": @(p.rmse),
        }];
    }
    root[@"lastRunHistory"] = history;

    const OptimizerOptions& o = _lastOptsUsed;
    root[@"optimizerOptions"] = @{
        @"samplesPerPatchEdge": @(o.samplesPerPatchEdge),
        @"smoothWeightGeom": @(o.smoothWeightGeom),
        @"smoothGeomEdgeGain": @(o.smoothGeomEdgeGain),
        @"smoothGeomMinFactor": @(o.smoothGeomMinFactor),
        @"smoothWeightColor": @(o.smoothWeightColor),
        @"colorDerivRidge": @(o.colorDerivRidge),
        @"boundaryWeight": @(o.boundaryWeight),
        @"vectorLineWeight": @(o.vectorLineWeight),
        @"vectorLineInfluenceRadius": @(o.vectorLineInfluenceRadius),
        @"geomTangentPriorWeight": @(o.geomTangentPriorWeight),
        @"outerIterationsPerLevel": @(o.outerIterationsPerLevel),
        @"outerConvergenceRelTol": @(o.outerConvergenceRelTol),
        @"outerConvergencePatience": @(o.outerConvergencePatience),
        @"pyramidRestarts": @(o.pyramidRestarts),
        @"geomGaussNewtonItersPerOuter": @(o.geomGaussNewtonItersPerOuter),
        @"cgMaxIterations": @(o.cgMaxIterations),
        @"cgRelTolerance": @(o.cgRelTolerance),
        @"geomDampingInitial": @(o.geomDampingInitial),
        @"useCeresGeometry": @(o.useCeresGeometry),
        @"useCeresJoint": @(o.useCeresJoint),
        // Added late -- omitted from the first round of exported debug
        // JSONs (they predate this OptimizerOptions field), so its
        // absence there means "this build's default (0.3), not recorded",
        // not "damping was off".
        @"jointGeomStepDampingWeight": @(o.jointGeomStepDampingWeight),
    };

    // Full mesh state -- the actual point of this export. Unlike the SVG
    // exporter (4 corner colors per patch, no Cu/Cv/Cuv, no explicit
    // geometry tangents) this carries every field GradientMesh::evalPos/
    // evalColor actually read, so an offline analysis can reproduce
    // reconstructionRMSE (and the render) exactly rather than approximate
    // it from Bezier control points.
    NSMutableArray<NSDictionary*>* vertsJSON = [NSMutableArray arrayWithCapacity:_mesh->vertices.size()];
    for (int r = 0; r < _mesh->rows; ++r) {
        for (int c = 0; c < _mesh->cols; ++c) {
            const gmcore::MeshVertex& mv = _mesh->at(r, c);
            [vertsJSON addObject:@{
                @"row": @(r), @"col": @(c),
                @"P": @[@(mv.P.x), @(mv.P.y)],
                @"Pu": @[@(mv.Pu.x), @(mv.Pu.y)],
                @"Pv": @[@(mv.Pv.x), @(mv.Pv.y)],
                @"Puv": @[@(mv.Puv.x), @(mv.Puv.y)], // always {0,0} -- see GradientMesh.h
                @"C": @[@(mv.C.r), @(mv.C.g), @(mv.C.b)],
                @"Cu": @[@(mv.Cu.r), @(mv.Cu.g), @(mv.Cu.b)],
                @"Cv": @[@(mv.Cv.r), @(mv.Cv.g), @(mv.Cv.b)],
                @"Cuv": @[@(mv.Cuv.r), @(mv.Cuv.g), @(mv.Cuv.b)],
                @"isBoundary": @(mv.isBoundary),
                @"boundarySide": @(mv.boundarySide),
                @"boundaryT": @(mv.boundaryT),
            }];
        }
    }
    root[@"mesh"] = @{
        @"rows": @(_mesh->rows),
        @"cols": @(_mesh->cols),
        @"vertices": vertsJSON,
    };

    NSMutableArray<NSArray*>* boundaryJSON = [NSMutableArray arrayWithCapacity:4];
    if (_hasBoundary) {
        for (int i = 0; i < 4; ++i) {
            const CubicBezier& b = _boundary[i];
            [boundaryJSON addObject:@[
                @[@(b.p0.x), @(b.p0.y)], @[@(b.p1.x), @(b.p1.y)],
                @[@(b.p2.x), @(b.p2.y)], @[@(b.p3.x), @(b.p3.y)],
            ]];
        }
    }
    root[@"boundary"] = boundaryJSON; // [top,right,bottom,left], each [p0,p1,p2,p3]

    NSMutableArray<NSArray*>* linesJSON = [NSMutableArray arrayWithCapacity:_vectorLines.size()];
    for (const auto& line : _vectorLines) {
        NSMutableArray<NSArray*>* pts = [NSMutableArray arrayWithCapacity:line.points.size()];
        for (const auto& p : line.points) [pts addObject:@[@(p.x), @(p.y)]];
        [linesJSON addObject:pts];
    }
    root[@"vectorLines"] = linesJSON;

    NSError* jsonErr = nil;
    NSData* data = [NSJSONSerialization dataWithJSONObject:root
                                                     options:(NSJSONWritingPrettyPrinted | NSJSONWritingSortedKeys)
                                                       error:&jsonErr];
    if (!data) { if (error) *error = jsonErr ?: gmError(@"Could not serialize debug data."); return NO; }
    BOOL ok = [data writeToURL:url options:NSDataWritingAtomic error:&jsonErr];
    if (!ok && error) *error = jsonErr ?: gmError(@"Could not write debug data file.");
    return ok;
}

@end
