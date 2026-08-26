// CanvasView.h — the interactive canvas: shows the loaded image and lets
// the user trace the object boundary, mark its 4 corners, draw optional
// guide "vector lines", and (once a mesh exists) drag control points /
// double-click to recolor them. This is the hand-editing counterpart to
// Sun et al.'s "cutout tool" + corner segmentation + directional
// constraints (Sec. 4), deliberately simplified to plain click-tracing
// (see README "Known simplifications" -- the paper cites a separate
// Lazy-Snapping-style cutout tool for this step, which is out of scope
// here).
#import <Cocoa/Cocoa.h>
#import "DocumentModel.h"

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, GMToolMode) {
    GMToolModeNone = 0,
    GMToolModeBoundary,   // click to add boundary polygon points
    GMToolModeCorners,    // click 4 existing boundary points to split it into 4 sides
    GMToolModeVectorLine,  // click-drag to draw one guide polyline
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

- (void)resetBoundaryDrawing;
- (void)resetCornerPicking;
- (void)refreshReconstructionPreview; // re-renders the mesh->image preview shown when showReconstructionPreview is YES

@end

NS_ASSUME_NONNULL_END
