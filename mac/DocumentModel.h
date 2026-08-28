// DocumentModel.h — the Objective-C++ bridge between AppKit and gmcore.
// Owns the loaded image, the user-drawn boundary/vector-lines, the
// GradientMesh, and drives MeshOptimizer on a background queue.
#import <Cocoa/Cocoa.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, GMBoundarySide) {
    GMBoundarySideTop = 0,
    GMBoundarySideRight = 1,
    GMBoundarySideBottom = 2,
    GMBoundarySideLeft = 3
};

@interface DocumentModel : NSObject

@property (nonatomic, readonly) BOOL hasImage;
@property (nonatomic, readonly) NSInteger imageWidth;
@property (nonatomic, readonly) NSInteger imageHeight;
@property (nonatomic, readonly, nullable) NSImage* displayImage;

@property (nonatomic, readonly) BOOL hasBoundary;   // 4 fitted Bezier sides ready
@property (nonatomic, readonly) BOOL hasMesh;
@property (nonatomic, readonly) BOOL isOptimizing;

@property (nonatomic, readonly) NSInteger meshRows;
@property (nonatomic, readonly) NSInteger meshCols;

// Which geometry/color solver -[optimizeWithPyramidLevels:progress:completion:]
// uses next -- mirrors gmcore::OptimizerOptions::useCeresGeometry/useCeresJoint
// (see MeshOptimizer.h for what each does). Both default NO (the original,
// dependency-free hand-rolled path). Setting useCeresJoint also implies
// useCeresGeometry is ignored, same "joint wins" precedence as the CLI's
// --use-ceres/--use-ceres-joint flags and MeshOptimizer.cpp's own gating --
// see that file's `jointSolvedByCeres` comment. If this binary wasn't built
// with Ceres found (GMCORE_WITH_CERES), setting either has no effect beyond
// a one-time warning printed to stderr (visible in Xcode's debug console) --
// optimization silently falls back to the hand-rolled path.
@property (nonatomic, assign) BOOL useCeresGeometry;
@property (nonatomic, assign) BOOL useCeresJoint;

// --- Image ---
- (BOOL)loadImageAtURL:(NSURL*)url error:(NSError**)error;

// --- Boundary tracing ---
// Raw polygon points the user clicked (image pixel coordinates), open or closed.
- (void)setBoundaryPolygonPoints:(NSArray<NSValue*>*)points; // NSValue(NSPoint)
- (NSArray<NSValue*>*)boundaryPolygonPoints;
// Fits the 4 CubicBezier sides given 4 corner indices into the (closed) polygon,
// in order: top-left, top-right, bottom-right, bottom-left.
- (BOOL)fitBoundaryWithCornerIndices:(NSArray<NSNumber*>*)fourIndices;
// Convenience for the "automatic, no manual markup" mode: an inset rectangle.
- (void)useRectangularBoundaryWithMargin:(double)marginPixels;

// --- Mesh ---
- (void)buildInitialMeshRows:(NSInteger)rows cols:(NSInteger)cols;
- (NSPoint)meshVertexPositionAtRow:(NSInteger)row col:(NSInteger)col;
- (NSColor*)meshVertexColorAtRow:(NSInteger)row col:(NSInteger)col;
- (void)setMeshVertexPosition:(NSPoint)p atRow:(NSInteger)row col:(NSInteger)col;
- (void)setMeshVertexColor:(NSColor*)color atRow:(NSInteger)row col:(NSInteger)col;
- (BOOL)findNearestVertexToPoint:(NSPoint)p maxDistance:(double)maxDist
                              row:(NSInteger*)outRow col:(NSInteger*)outCol;

// Exact cubic-Bezier control points [B0,B1,B2,B3] (image pixel coords) of
// the Ferguson-patch-edge curve between two ADJACENT mesh vertices (same
// row, adjacent col -- or same col, adjacent row). This is the Hermite
// (position + free tangent) -> Bezier conversion: B0=P0, B1=P0+T0/3,
// B2=P1-T1/3, B3=P1, using whichever of Pu/Pv runs along that edge. It is
// exact, not an approximation -- the same math the optimizer's patch
// surface itself is built from (see FergusonPatch.h). Returns an empty
// array if the two vertices aren't grid-adjacent or there's no mesh.
- (NSArray<NSValue*>*)meshEdgeBezierFromRow:(NSInteger)r0 col:(NSInteger)c0
                                       toRow:(NSInteger)r1 col:(NSInteger)c1;

// The 4 fitted boundary CubicBezier splines (top/right/bottom/left), each
// as [p0,p1,p2,p3] control points in image pixel coords. Empty if
// hasBoundary is NO (i.e. only the raw traced polygon exists so far).
- (NSArray<NSArray<NSValue*>*>*)fittedBoundaryCurves;

// The free geometry tangents Pu/Pv at a control point -- independent
// optimization unknowns (see GradientMesh.h's MeshVertex comment), not
// derived from neighboring positions any more. Returned as (dx,dy)
// DISPLACEMENT vectors in image pixel units, not absolute points -- add to
// the vertex position yourself for an arrow endpoint. Pu runs along
// increasing column (u/horizontal), Pv along increasing row (v/vertical).
- (NSPoint)meshVertexTangentUAtRow:(NSInteger)row col:(NSInteger)col;
- (NSPoint)meshVertexTangentVAtRow:(NSInteger)row col:(NSInteger)col;

// --- Vector guide lines ---
- (void)addVectorLineWithPoints:(NSArray<NSValue*>*)points;
- (void)removeLastVectorLine;
- (void)clearVectorLines;
- (NSArray<NSArray<NSValue*>*>*)vectorLinesPoints;

// --- Optimization ---
// progress/completion blocks are always invoked on the main queue.
- (void)optimizeWithPyramidLevels:(NSInteger)levels
                          progress:(nullable void (^)(double rmse, NSInteger level, NSInteger totalLevels,
                                                       NSInteger iter, NSInteger totalIters))progress
                        completion:(nullable void (^)(void))completion;

// --- Output ---
- (nullable NSImage*)renderReconstructionPreview;
- (double)currentRMSE;
- (BOOL)exportPNGToURL:(NSURL*)url error:(NSError**)error;
- (BOOL)exportSVGToURL:(NSURL*)url error:(NSError**)error;

@end

NS_ASSUME_NONNULL_END
