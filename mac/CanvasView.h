// CanvasView.h — the interactive canvas: shows the loaded image and lets
// the user trace the object boundary (by hand, OR by painting foreground/
// background scribbles for a Lazy-Snapping-style graph-cut segmentation --
// see GMToolModeScribbleForeground/Background below and DocumentModel's
// -segmentBoundaryFromScribblesWithError:), mark its 4 corners, draw
// optional guide "vector lines", and (once a mesh exists) drag control
// points / double-click to recolor them. This is the hand-editing
// counterpart to Sun et al.'s "cutout tool" + corner segmentation +
// directional constraints (Sec. 4) -- the scribble tool modes are this
// project's implementation of the paper's own cited Lazy-Snapping cutout
// tool (see README "Known simplifications" -- manual click-tracing is
// kept alongside it as a fallback, not replaced).
#import <Cocoa/Cocoa.h>
#import "DocumentModel.h"

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, GMToolMode) {
    GMToolModeNone = 0,
    GMToolModeBoundary,   // click to add boundary polygon points
    GMToolModeCorners,    // click 4 existing boundary points to split it into 4 sides
    GMToolModeVectorLine,  // click-drag to draw one guide polyline
    // Click-drag (or click, for a single "dab") to paint a foreground/
    // background scribble stroke -- accumulated on DocumentModel (see
    // -addForegroundScribbleWithPoints:/-addBackgroundScribbleWithPoints:)
    // across as many separate strokes as needed, then run via
    // DocumentModel's -segmentBoundaryFromScribblesWithError:, which calls
    // -setBoundaryPolygonPoints: with the result -- so corner-picking
    // below (GMToolModeCorners) and everything after it works identically
    // whether the polygon came from these scribbles or manual tracing.
    GMToolModeScribbleForeground,
    GMToolModeScribbleBackground,
    GMToolModeEditMesh    // drag control points; double-click to recolor
};

@interface CanvasView : NSView

@property (nonatomic, weak, nullable) DocumentModel* documentModel;
@property (nonatomic, assign) GMToolMode toolMode;
@property (nonatomic, assign) BOOL showMeshOverlay;
@property (nonatomic, assign) BOOL showReconstructionPreview; // if YES, draws the rendered mesh instead of the source image
@property (nonatomic, assign) BOOL showTangents; // diagnostic overlay: derived Pu (orange) / Pv (cyan) arrows per vertex

@property (nonatomic, copy, nullable) void (^onBoundaryChanged)(void);
@property (nonatomic, copy, nullable) void (^onCornersPicked)(NSArray<NSNumber*>* fourIndices);
@property (nonatomic, copy, nullable) void (^onVectorLineFinished)(NSArray<NSValue*>* points);
@property (nonatomic, copy, nullable) void (^onMeshEdited)(void);
// Fired after EITHER scribble mode commits a completed stroke (drag or
// single dab) to DocumentModel -- lets MainWindowController update e.g. a
// status hint or enable a "Segment" button once both colours have at
// least one stroke.
@property (nonatomic, copy, nullable) void (^onScribblesChanged)(void);

- (void)resetBoundaryDrawing;
- (void)resetCornerPicking;
- (void)refreshReconstructionPreview; // re-renders the mesh->image preview shown when showReconstructionPreview is YES

@end

NS_ASSUME_NONNULL_END
