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

// GMGPUMeshBuffers -- the flat, GPU-renderer-ready data for the CURRENT
// mesh, produced by -gpuMeshBuffersWithSamplesPerPatchEdge: and consumed by
// GLReconstructionView (mac/GLReconstructionView.h). Deliberately a plain
// Objective-C value type (NSData, not a gmcore C++ type) so this
// (Objective-C-only) header and GLReconstructionView.h stay decoupled from
// gmcore's C++ types -- only DocumentModel.mm and GLReconstructionView.mm
// (both already Objective-C++) touch gmcore directly. See
// gmcore/MeshRenderBuffers.h for exactly what each NSData holds; the field
// names here match 1:1.
@interface GMGPUMeshBuffers : NSObject
@property (nonatomic, strong) NSData* vertexData;  // rows*cols*18 floats
@property (nonatomic, strong) NSData* uvTemplate;   // (samplesPerEdge+1)^2 * 2 floats
@property (nonatomic, strong) NSData* indexData;    // 2 triangles/cell, uint32 indices
@property (nonatomic, assign) NSInteger cols;
@property (nonatomic, assign) NSInteger patchRows;
@property (nonatomic, assign) NSInteger patchCols;
// Mirrors DocumentModel.meshColorSpaceIsCIELUV at the moment these buffers
// were built -- tells GLReconstructionView's fragment shader whether to
// run its cieluvToSRGB port (see GLShaderSources.h).
@property (nonatomic, assign) BOOL cieluv;
// Mesh geometry (P) bounding box, image/mesh-space -- lets
// GLReconstructionView fit + center the mesh the same way CanvasView's
// -imageDisplayRect letterboxes the CPU preview.
@property (nonatomic, assign) double minX, minY, maxX, maxY;
@end

@interface DocumentModel : NSObject

@property (nonatomic, readonly) BOOL hasImage;
@property (nonatomic, readonly) NSInteger imageWidth;
@property (nonatomic, readonly) NSInteger imageHeight;
@property (nonatomic, readonly, nullable) NSImage* displayImage;

@property (nonatomic, readonly) BOOL hasBoundary;   // 4 fitted Bezier sides ready
@property (nonatomic, readonly) BOOL hasMesh;
@property (nonatomic, readonly) BOOL isOptimizing;

// When YES, -optimizeWithPyramidLevels:progress:completion: takes a cheap
// snapshot COPY of the mesh once per outer iteration -- on the SAME
// background thread that's mutating the live mesh, right after that
// iteration's writes finish and before the next iteration's begin, so the
// copy itself is never racing a concurrent write -- and hands it to the
// main thread for -meshVertexPositionAtRow:col:/-meshVertexColorAtRow:col:/
// -meshEdgeBezierFromRow:col:toRow:col: to read from (see
// -hasLivePreviewMesh) instead of the live mesh those three would
// otherwise read. This is what lets CanvasView draw the in-progress mesh
// grid while optimizing, toggleable because it costs one small mesh copy
// per outer iteration (cheap for the mesh sizes this app targets, but
// nonzero) and a canvas redraw each time -- default NO, matching every
// other opt-in diagnostic/overlay toggle in this class. Read once into a
// local BEFORE the background dispatch in -optimizeWithPyramidLevels:...,
// same pattern as useCeresGeometry/useCeresJoint/the eight weight
// properties, so toggling it mid-run never half-applies.
@property (nonatomic, assign) BOOL livePreviewDuringOptimize;

// YES only while a run is in flight AND at least one post-iteration
// snapshot has arrived (see livePreviewDuringOptimize above) -- i.e.
// exactly when -meshVertexPositionAtRow:col:/-meshVertexColorAtRow:col:/
// -meshEdgeBezierFromRow:col:toRow:col: are reading the snapshot instead of
// the live (possibly-being-mutated) mesh. CanvasView checks this to decide
// whether it's safe to draw the mesh grid DURING an optimize run -- outside
// a run, or when livePreviewDuringOptimize was off for it, this is NO and
// CanvasView keeps its existing behavior of not drawing the mesh overlay
// while isOptimizing (reading the live mesh from the main thread while the
// background thread mutates it is a data race -- this property exists
// specifically so CanvasView never has to do that).
@property (nonatomic, readonly) BOOL hasLivePreviewMesh;

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

// Mirrors gmcore::OptimizerOptions::ceresNumThreads -- see that field's
// comment in MeshOptimizer.h for the full rationale. YES (default) lets
// every ceres::Solve call use every CPU core (ceresNumThreads=0, "auto");
// NO pins it to exactly 1 thread, for an apples-to-apples A/B comparison
// against a multithreaded run of the same weights, or on a machine you'd
// rather not saturate. Has no effect on the hand-rolled path (no threading
// of its own yet) or when this binary wasn't built with Ceres found.
@property (nonatomic, assign) BOOL ceresMultithreaded;

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

// --- Geometry/colour energy weights ---
// Mirror the corresponding gmcore::OptimizerOptions fields 1:1 (see
// MeshOptimizer.h for each one's full derivation/tuning history) -- read
// fresh into a new OptimizerOptions at the START of every
// -optimizeWithPyramidLevels:progress:completion: call, so a value typed
// into the UI takes effect on the NEXT "Optimize" click, same as the
// rows/cols fields already work. -init seeds these from a
// default-constructed gmcore::OptimizerOptions (i.e. this project's normal,
// sRGB-tuned defaults), not a second hardcoded copy of those numbers --
// see DocumentModel.mm. Exposed specifically because CIELUV mode
// (useCIELUVColorSpace) needs these RE-tuned for its own colour-magnitude
// scale (see ColorSpace.h's kCIELUVWorkingScale comment for why) and
// recompiling for every trial value isn't practical; MainWindowController's
// "Reset weights to defaults" button restores all eight at once via the
// same default-constructed OptimizerOptions.
@property (nonatomic, assign) double smoothWeightGeom;      // OptimizerOptions::smoothWeightGeom
@property (nonatomic, assign) double smoothWeightColor;     // OptimizerOptions::smoothWeightColor
@property (nonatomic, assign) double colorDerivRidge;       // OptimizerOptions::colorDerivRidge
@property (nonatomic, assign) double boundaryWeight;        // OptimizerOptions::boundaryWeight
@property (nonatomic, assign) double geomTangentPriorWeight; // OptimizerOptions::geomTangentPriorWeight
@property (nonatomic, assign) double vectorLineWeight;      // OptimizerOptions::vectorLineWeight
@property (nonatomic, assign) double geomDataWeight;        // OptimizerOptions::geomDataWeight -- multiplies
                                                              // ONLY the geometry step's photometric data
                                                              // term (see that field's comment in
                                                              // MeshOptimizer.h); 1.0 = unchanged/default.
                                                              // The direct, single-knob way to reproduce
                                                              // this project's pre-kCIELUVWorkingScale
                                                              // CIELUV behaviour (border artifacts + sharp
                                                              // edges) WITHOUT touching ColorSpace.cpp:
                                                              // raising this to ~10000 with the other six
                                                              // weights left at default reproduces the same
                                                              // data-term-vs-regularizer imbalance raw
                                                              // (unscaled) CIELUV had, more directly than
                                                              // shrinking boundaryWeight/smoothWeightGeom/
                                                              // geomTangentPriorWeight/vectorLineWeight by
                                                              // 10000x each by hand.
@property (nonatomic, assign) double smoothGeomEdgeGain;    // OptimizerOptions::smoothGeomEdgeGain --
                                                              // makes smoothWeightGeom ANISOTROPIC, relaxing
                                                              // it near a strong local image gradient (see
                                                              // that field's -- and edgeRelaxFactor's --
                                                              // comments in MeshOptimizer.h/.cpp); 0.0 =
                                                              // fully isotropic (edgeRelaxFactor degenerates
                                                              // to exactly 1.0 everywhere), matching Sec
                                                              // 4.1's original flat-weight smoothness term;
                                                              // 40.0 = this project's compiled-in default.
                                                              // A real, on-device sweep (see README's
                                                              // "smoothGeomEdgeGain sweep" section) found
                                                              // 150-200 snaps a coarse mesh to a sharp edge
                                                              // (Fig. 4-style) noticeably tighter than the
                                                              // default -- but the SAME sweep measurably
                                                              // REGRESSED a smooth synthetic-sphere test
                                                              // case (56.5% -> 52.6% RMSE reduction), so
                                                              // this is a genuine trade-off, not a strictly
                                                              // better value, and 40.0 stays the global
                                                              // default; exposed here so it can be raised
                                                              // per-image for a mesh that needs tighter
                                                              // edge-snapping without changing that default
                                                              // for everyone else. NOTE: because this
                                                              // property was added after geomDataWeight (and
                                                              // -loadPresetNamed:rows:cols:error: has no
                                                              // per-field presence guard, same as every
                                                              // other weight), a preset saved before this
                                                              // property existed will load it as 0.0 -- which
                                                              // happens to be the safe, paper-faithful
                                                              // isotropic fallback above, not a broken value,
                                                              // unlike a missing geomDataWeight (0.0 there
                                                              // would zero out the geometry data term
                                                              // entirely).

// Resets all eight weight properties above to gmcore::OptimizerOptions' own
// compiled-in defaults (the same ones -init seeds them with) -- does NOT
// touch anything else (solver picker, pyramidRestarts, useCIELUVColorSpace,
// the mesh itself). Wired to MainWindowController's "Reset weights to
// defaults" button.
- (void)resetWeightsToDefaults;

// When YES, the NEXT -buildInitialMeshRows:cols: call builds the mesh (and
// every subsequent -optimizeWithPyramidLevels:progress:completion: call
// fits it) entirely in CIELUV colour space instead of raw sRGB: the loaded
// image is converted once (see -workingTargetImage/_targetLUV), and the
// mesh's C/Cu/Cv/Cuv fields end up holding (L*,u*,v*) rather than (r,g,b).
// Everything downstream that needs an actual displayable colour (the
// on-screen/PNG raster, the SVG exporter's stop colors, the vertex color
// swatch/picker) converts back to sRGB automatically -- see ColorSpace.h and
// DocumentModel.mm for the full rationale (this follows a finding in
// Hogervorst 2017, "Colour Interpolation in Gradient Meshes": CIELUV is
// perceptually uniform, unlike sRGB, and -- unlike CIELAB -- doesn't show an
// unnatural colour artifact on some transitions).
//
// Default NO (sRGB, the original behavior). IMPORTANT: this property is
// only consulted at -buildInitialMeshRows:cols: time -- toggling it after a
// mesh already exists does NOT retroactively convert that mesh's stored
// colours; the mesh stays in whichever space it was built in (tracked
// internally) until rebuilt, precisely so a stray toggle between "build"
// and "optimize" can never silently feed the optimizer a colour-space
// mismatch (mesh colours in one space, target image in another).
// -currentRMSE after a CIELUV-mode run is measured in CIELUV units (L* is
// roughly 0..100), NOT directly comparable by raw number to an sRGB-mode
// RMSE -- see -exportDebugDataToURL:error:'s new "colorSpace" field, which
// records which space produced a given number.
@property (nonatomic, assign) BOOL useCIELUVColorSpace;

// The colour space the CURRENT mesh (hasMesh) was actually built in --
// snapshotted from useCIELUVColorSpace at the last -buildInitialMeshRows:
// cols: call, so this reflects reality even if useCIELUVColorSpace has been
// toggled since without rebuilding (see that property's doc comment). NO
// (sRGB) if there is no mesh yet.
@property (nonatomic, readonly) BOOL meshColorSpaceIsCIELUV;

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
// meshVertexPositionAtRow:col:/meshVertexColorAtRow:col: (and
// meshEdgeBezierFromRow:col:toRow:col: below) read the live mesh normally,
// but transparently read the livePreviewDuringOptimize snapshot instead
// whenever hasLivePreviewMesh is YES -- see that property's comment. Callers
// (CanvasView's -drawMesh) don't need to know or care which one they're
// getting; setMeshVertexPosition:/setMeshVertexColor:/findNearestVertexTo...
// below are unaffected and always read/write the live mesh (editing is not
// meant to run concurrently with an in-flight optimize).
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

// --- Scribble-based segmentation (Lazy Snapping, see Li/Sun/Shum 2004 as
// cited by the Sun et al. 2007 gradient-mesh paper for its cutout tool) ---
// Foreground/background scribble strokes, image pixel coordinates -- one
// entry per mouse drag, mirroring the vector-line strokes above. Unlike
// vector lines, a single-point "dab" (points.count == 1) is accepted: a
// tap is a valid scribble, not just a drag.
- (void)addForegroundScribbleWithPoints:(NSArray<NSValue*>*)points;
- (void)addBackgroundScribbleWithPoints:(NSArray<NSValue*>*)points;
- (void)removeLastForegroundScribble;
- (void)removeLastBackgroundScribble;
- (void)clearScribbles;
- (NSArray<NSArray<NSValue*>*>*)foregroundScribblePoints;
- (NSArray<NSArray<NSValue*>*>*)backgroundScribblePoints;
- (BOOL)hasForegroundScribbles;
- (BOOL)hasBackgroundScribbles;
// Runs the graph-cut segmentation (gmcore::segmentForeground) against the
// raw sRGB image (segmentation is a pre-mesh-building step, independent of
// useCIELUVColorSpace), traces the outer contour of the largest resulting
// foreground component (gmcore::traceOuterContour), simplifies it
// (gmcore::simplifyClosedPolygon), and -- on success -- feeds the result
// into the EXACT SAME -setBoundaryPolygonPoints: entry point manual
// click-tracing already uses, so corner-picking and boundary fitting work
// unchanged afterwards. Fails (returns NO, sets *error) if there aren't
// scribbles of both colors yet, or if the resulting contour is degenerate
// (fewer than 3 points -- e.g. scribbles that don't separate anything).
- (BOOL)segmentBoundaryFromScribblesWithError:(NSError**)error;

// --- Optimization ---
// progress/completion blocks are always invoked on the main queue.
- (void)optimizeWithPyramidLevels:(NSInteger)levels
                          progress:(nullable void (^)(double rmse, NSInteger level, NSInteger totalLevels,
                                                       NSInteger iter, NSInteger totalIters))progress
                        completion:(nullable void (^)(void))completion;

// --- Output ---
- (nullable NSImage*)renderReconstructionPreview;
// GPU (OpenGL) counterpart to -renderReconstructionPreview -- builds the
// same mesh's data as flat, shader-ready buffers instead of a rasterized
// NSImage (see GMGPUMeshBuffers and mac/GLReconstructionView.h). Returns
// nil if !hasMesh. samplesPerPatchEdge should normally be passed as 8 to
// match -renderReconstructionPreview's own hardcoded tessellation density,
// so the CPU and GPU previews are visually comparable at the same
// settings.
- (nullable GMGPUMeshBuffers*)gpuMeshBuffersWithSamplesPerPatchEdge:(NSInteger)samplesPerPatchEdge;
- (double)currentRMSE;
// Mean Absolute Error, same sampling/units convention as currentRMSE (see
// gmcore::GradientMesh::reconstructionMAE) -- computed and updated at
// exactly the same points currentRMSE is (-buildInitialMeshRows:cols: and
// after each -optimizeWithPyramidLevels:progress:completion: run), and
// carries the same CIELUV-units caveat when meshColorSpaceIsCIELUV is YES.
// Reported alongside RMSE because RMSE's squaring makes it disproportionately
// sensitive to a few large-error samples, while MAE weighs every sample
// equally -- comparing the two says something neither alone can.
- (double)currentMAE;
// Wall-clock duration of the most recently COMPLETED
// -optimizeWithPyramidLevels:progress:completion: run, in seconds (measured
// from just before the background dispatch to just after
// optimizeCoarseToFine returns -- see -optimizeWithPyramidLevels:...'s
// runStart/-lastRunWallClockSeconds assignment). 0 before any run has
// completed in this session. This is the same number already written into
// -exportDebugDataToURL:error:'s "lastRunWallClockSeconds" JSON field (and,
// when autoExportDebugData is on, every auto-exported log); exposed here
// too so MainWindowController can show it directly in the status bar
// without needing a debug export.
- (double)lastRunWallClockSeconds;
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

// --- Presets ---
// A preset captures exactly the settings the LAST completed
// -optimizeWithPyramidLevels:progress:completion: run used -- the same
// eight weight properties, solver choice, pyramidRestarts/ceresNumThreads,
// and colour space -exportDebugDataToURL:error:'s own "optimizerOptions"/
// "colorSpace" fields already carry (same source: _lastOptsUsed/
// _meshColorSpaceIsCIELUV, snapshotted at that run's start, not whatever
// may have been typed into the UI since) -- plus that run's mesh
// dimensions, as a small standalone JSON file that can be reloaded later,
// on this image or a different one, without retyping every field by hand.
// Deliberately does NOT capture the mesh itself, the boundary, or vector
// lines -- those are per-image content, not a reusable "setting". Requires
// a completed run (mirrors exportDebugDataToURL:error:'s _mesh guard, but
// on _hasRunOptimize instead -- an unoptimized initial mesh has no
// meaningful "settings that produced this result" yet); returns NO with an
// error otherwise.
- (BOOL)savePresetToURL:(NSURL*)url error:(NSError**)error;

// Sibling "Presets" folder next to the currently loaded image, creating it
// if needed -- same convention (and same nil cases: no loaded image, or
// the folder couldn't be created) as -debugOutDirectoryURL.
- (nullable NSURL*)presetsDirectoryURL;

// "gm_preset_<solver>[_cieluv]_<rows>x<cols>_<timestamp>.json" -- the exact
// same naming convention -debugExportFilename uses for its debug JSONs
// (see that method's comment for the full rationale), just "preset"
// instead of "debug", so a preset's filename is legible at a glance the
// same way a debug log's already is.
- (NSString*)presetExportFilename;

// Every ".json" file currently in -presetsDirectoryURL, newest first, as
// display names (WITHOUT the ".json" extension -- pass this same string to
// -loadPresetNamed:rows:cols:error: to load one back). Empty (not nil) if
// there's no image loaded yet or the folder doesn't exist/has nothing in
// it. Re-scans the folder every call -- presets are small and infrequent,
// and this is only ever called right before populating a menu, so there's
// no reason to cache.
- (NSArray<NSString*>*)availablePresetNames;

// Loads the preset previously saved as `name` (see -availablePresetNames,
// same string, no ".json") back onto this DocumentModel's live properties
// (the eight weights, solver choice, pyramidRestarts, ceresMultithreaded,
// useCIELUVColorSpace) -- exactly the settings -savePresetToURL:error:
// captured, applied the same way a user typing them in by hand would be:
// takes effect on the NEXT "Optimize" (and, for useCIELUVColorSpace, the
// next "Build Initial Mesh" -- see that property's own doc comment;
// loading a preset does NOT retroactively touch an already-built mesh).
// outRows/outCols (if non-NULL) receive the preset's saved mesh
// dimensions, since Rows/Cols are plain UI text fields, not DocumentModel
// properties -- for MainWindowController to copy them in itself, same
// pattern as -findNearestVertexToPoint:maxDistance:row:col:'s out-params.
// Returns NO with an error if `name` doesn't match a file in
// -presetsDirectoryURL or it couldn't be parsed.
- (BOOL)loadPresetNamed:(NSString*)name rows:(NSInteger*)outRows cols:(NSInteger*)outCols
                   error:(NSError**)error;

// Full path of the most recent successful -savePresetToURL:error: call, or
// nil if none has happened yet in this session -- same convention as
// lastDebugExportPath, for MainWindowController's status bar.
@property (nonatomic, readonly, nullable) NSString* lastPresetSavePath;

@end

NS_ASSUME_NONNULL_END
