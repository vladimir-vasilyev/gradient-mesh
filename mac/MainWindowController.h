// MainWindowController.h — builds the single-window UI entirely in code
// (no nib/storyboard) and wires CanvasView's user-interaction callbacks to
// DocumentModel.
#import <Cocoa/Cocoa.h>

NS_ASSUME_NONNULL_BEGIN

@interface MainWindowController : NSWindowController
- (instancetype)init;

// File-menu actions (AppDelegate wires the menu items to these).
- (void)openImage:(nullable id)sender;
- (void)savePreset:(nullable id)sender;
- (void)exportPNG:(nullable id)sender;
- (void)exportSVG:(nullable id)sender;
- (void)exportGPUPNG:(nullable id)sender;

// Tool / Mesh / View menu actions (see AppDelegate -buildMainMenu).
- (void)selectTool:(NSMenuItem*)sender;       // sender.tag = tool segment index
- (void)autoMesh:(nullable id)sender;
- (void)buildMesh:(nullable id)sender;
- (void)optimize:(nullable id)sender;
- (void)toggleAnimateMesh:(nullable id)sender;
- (void)resetWeights:(nullable id)sender;
- (void)segmentBoundary:(nullable id)sender;
- (void)clearScribbles:(nullable id)sender;
- (void)clearLastLine:(nullable id)sender;
- (void)toggleInspector:(nullable id)sender;
- (void)toggleViewOption:(NSMenuItem*)sender; // sender.tag: 1 reconstruction, 2 GPU, 3 mesh, 4 tangents, 5 live preview
@end

NS_ASSUME_NONNULL_END
