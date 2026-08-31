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

// How many times -[optimizeWithPyramidLevels:progress:completion:] repeats
// the full coarse-to-fine sweep in one call -- mirrors
// gmcore::OptimizerOptions::pyramidRestarts (see MeshOptimizer.h for the
// full diagnosis of why this exists: a single sweep already reaches a real
// local optimum at every pyramid level, but re-descending to the coarsest
// level from an already-refined mesh can still find a marginally better one,
// which is exactly what clicking "Optimize" a second time was doing
// manually). 0 or unset is treated as 1 (the original single-sweep
// behavior, unchanged) -- see -optimizeWithPyramidLevels:progress:completion:.
@property (nonatomic, assign) NSInteger pyramidRestarts;

// When YES, every completed -optimizeWithPyramidLevels:progress:completion:
// run automatically writes a full debug-data JSON (same content
// -exportDebugDataToURL:error: below produces) to a "DebugOut" folder next
// to the currently loaded image -- created if it doesn't exist yet -- under
// a timestamped filename, e.g.
// "gm_debug_ceres_joint_9x9_20260901-143022-118.json". Timestamped rather
// than a fixed name specifically so repeated runs (trying different solver
// settings, or re-running after a code change) accumulate side by side
// instead of the newest one silently overwriting the last -- the whole
// point is comparing runs after the fact. Default NO (opt-in, since it
// writes files without an explicit per-run save dialog). Silently does
// nothing if there's no mesh yet or no image was ever loaded (no image
// means nowhere to put "next to the image"); check -lastDebugExportPath
// after a run to see whether/where it actually wrote, or watch the status
// bar in MainWindowController.
@property (nonatomic, assign) BOOL autoExportDebugData;

// Full path of the most recent successful auto-export (see
// autoExportDebugData above), or nil if none has happened yet in this
// session (auto-export disabled, no run completed since it was enabled, or
// the last attempt failed -- these aren't distinguished here; check the
// console log for a failure reason).
@property (nonatomic, readonly, nullable) NSString* lastDebugExportPath;

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

// Dumps everything needed to analyze a solver run OFFLINE, without a local
// Ceres build -- the actual motivation for this method: this project's
// Ceres-backed solvers can only be built/run on-device (Xcode + a real
// Ceres install), never in the sandbox used to develop and review this
// code, so this is the channel for getting real solver internals out of a
// run: the exact git commit this binary was built from, which solver was
// requested vs. which one ACTUALLY ran (see gmcore::builtWithCeres() --
// useCeresGeometry/useCeresJoint silently no-op back to hand-rolled, with
// only a one-time stderr warning, in a build without Ceres found), the full
// OptimizerOptions used, the mesh dimensions, the last reported RMSE, the
// per-outer-iteration RMSE history of the run that produced the current
// mesh, and -- most importantly -- the COMPLETE mesh state: every vertex's
// position P, free geometry tangents Pu/Pv, inert Puv, and full color
// Hermite data C/Cu/Cv/Cuv, plus the 4 boundary splines and any vector
// guide lines. This is strictly more than either the SVG or PNG export:
// SVG only carries 4 corner colors per patch (no Cu/Cv/Cuv, no geometry
// tangents beyond what's implied by the Bezier control points), and PNG
// carries no structured data at all -- neither lets an offline analysis
// exactly reproduce what GradientMesh::evalPos/evalColor/reconstructionRMSE
// actually compute. This does, by construction (same fields, same units).
// Written as pretty-printed, sorted-key JSON. Requires hasMesh; does NOT
// require having run -optimizeWithPyramidLevels:progress:completion: yet
// (an unoptimized initial mesh can still be dumped, just with an empty
// history and default-constructed optimizerOptions).
- (BOOL)exportDebugDataToURL:(NSURL*)url error:(NSError**)error;

@end

NS_ASSUME_NONNULL_END
