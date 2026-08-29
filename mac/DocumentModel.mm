#import "DocumentModel.h"
#import <ImageIO/ImageIO.h>
#import <CoreGraphics/CoreGraphics.h>
#include "gmcore/Image.h"
#include "gmcore/GradientMesh.h"
#include "gmcore/MeshOptimizer.h"
#include "gmcore/SVGExporter.h"
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

@interface DocumentModel () {
    gmcore::Image _target;
    BOOL _hasImage;
    std::vector<Vec2> _boundaryPolygon;
    std::array<CubicBezier, 4> _boundary;
    BOOL _hasBoundary;
    std::unique_ptr<GradientMesh> _mesh;
    std::vector<VectorLine> _vectorLines;
    BOOL _isOptimizing;
    double _lastRMSE;
}
@property (nonatomic, strong, nullable) NSImage* displayImage;
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
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    CGContextRef ctx = CGBitmapContextCreate(buffer.data(), w, h, 8, w * 4, cs,
                                              kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    CGColorSpaceRelease(cs);
    if (!ctx) { CGImageRelease(cgImage); if (error) *error = gmError(@"Could not create bitmap context."); return NO; }
    CGContextSetBlendMode(ctx, kCGBlendModeCopy);
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

- (void)buildInitialMeshRows:(NSInteger)rows cols:(NSInteger)cols {
    if (!_hasImage || !_hasBoundary) return;
    _mesh = std::make_unique<GradientMesh>(GradientMesh::buildInitial((int)rows, (int)cols, _boundary, _target));
    _lastRMSE = _mesh->reconstructionRMSE(_target, 6);
}

- (NSPoint)meshVertexPositionAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return NSZeroPoint;
    Vec2 p = _mesh->at((int)row, (int)col).P;
    return NSMakePoint(p.x, p.y);
}

- (NSColor*)meshVertexColorAtRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return [NSColor blackColor];
    Color c = _mesh->at((int)row, (int)col).C.clamped01();
    return [NSColor colorWithCalibratedRed:c.r green:c.g blue:c.b alpha:1.0];
}

- (void)setMeshVertexPosition:(NSPoint)p atRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return;
    _mesh->at((int)row, (int)col).P = Vec2(p.x, p.y);
}

- (void)setMeshVertexColor:(NSColor*)color atRow:(NSInteger)row col:(NSInteger)col {
    if (!_mesh) return;
    NSColor* rgb = [color colorUsingColorSpace:[NSColorSpace deviceRGBColorSpace]];
    _mesh->at((int)row, (int)col).C = Color(rgb.redComponent, rgb.greenComponent, rgb.blueComponent);
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
    Image targetCopy = _target; // Image is a small value type wrapping a vector; a private copy for thread safety.
    std::vector<VectorLine> linesCopy = _vectorLines;
    OptimizerOptions opts;
    // See DocumentModel.h's comment on these two properties: "joint wins" if
    // both are set, mirroring MeshOptimizer.cpp's own jointSolvedByCeres
    // gating and the CLI's --use-ceres/--use-ceres-joint precedence.
    opts.useCeresGeometry = self.useCeresGeometry;
    opts.useCeresJoint = self.useCeresJoint;
    // 0/unset -> 1: see DocumentModel.h's comment on this property.
    opts.pyramidRestarts = (int)std::max((NSInteger)1, self.pyramidRestarts);

    dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
        gmcore::MeshOptimizer::optimizeCoarseToFine(*meshPtr, targetCopy, linesCopy, (int)levels, opts,
            [progress](const OptimizerProgress& p) {
                if (!progress) return;
                dispatch_async(dispatch_get_main_queue(), ^{
                    progress(p.rmse, p.pyramidLevel, p.totalPyramidLevels, p.outerIteration, p.totalOuterIterations);
                });
            });
        double finalRmse = meshPtr->reconstructionRMSE(targetCopy, 6);
        dispatch_async(dispatch_get_main_queue(), ^{
            self->_isOptimizing = NO;
            self->_lastRMSE = finalRmse;
            if (completion) completion();
        });
    });
}

#pragma mark - Output

- (NSImage*)renderReconstructionPreview {
    if (!_mesh || !_hasImage) return nil;
    Image rendered = _mesh->render((int)_target.width, (int)_target.height, 8);
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
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
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
    std::string svg = gmcore::exportGradientMeshSVG(*_mesh, _target.width, _target.height);
    NSString* str = [NSString stringWithUTF8String:svg.c_str()];
    NSError* writeErr = nil;
    BOOL ok = [str writeToURL:url atomically:YES encoding:NSUTF8StringEncoding error:&writeErr];
    if (!ok && error) *error = writeErr ?: gmError(@"Could not write SVG file.");
    return ok;
}

@end
