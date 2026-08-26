// MainWindowController.h — builds the single-window UI entirely in code
// (no nib/storyboard) and wires CanvasView's user-interaction callbacks to
// DocumentModel.
#import <Cocoa/Cocoa.h>

NS_ASSUME_NONNULL_BEGIN

@interface MainWindowController : NSWindowController
- (instancetype)init;
@end

NS_ASSUME_NONNULL_END
