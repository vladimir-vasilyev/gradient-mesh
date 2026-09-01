#import "MainWindowController.h"
#import "CanvasView.h"
#import "DocumentModel.h"
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

@interface MainWindowController () <NSWindowDelegate>
@property (nonatomic, strong) DocumentModel* documentModel;
@property (nonatomic, strong) CanvasView* canvasView;
@property (nonatomic, strong) NSSegmentedControl* toolSegmented;
@property (nonatomic, strong) NSTextField* rowsField;
@property (nonatomic, strong) NSTextField* colsField;
@property (nonatomic, strong) NSTextField* statusLabel;
@property (nonatomic, strong) NSProgressIndicator* progressSpinner;
@property (nonatomic, strong) NSButton* previewCheckbox;
@property (nonatomic, strong) NSButton* tangentsCheckbox;
@property (nonatomic, strong) NSButton* buildMeshButton;
@property (nonatomic, strong) NSButton* optimizeButton;
@property (nonatomic, strong) NSButton* exportPNGButton;
@property (nonatomic, strong) NSButton* exportSVGButton;
// Solver picker: Hand-rolled (default) / Ceres (geometry) / Ceres (joint) --
// mirrors gmcore::OptimizerOptions::useCeresGeometry/useCeresJoint via
// DocumentModel's properties of the same name. See -solverChanged:.
@property (nonatomic, strong) NSPopUpButton* solverPopup;
// Mirrors DocumentModel.autoExportDebugData -- see -toggleAutoDebug:.
@property (nonatomic, strong) NSButton* autoDebugCheckbox;
// Mirrors DocumentModel.useCIELUVColorSpace -- see -toggleCIELUV:.
@property (nonatomic, strong) NSButton* cieluvCheckbox;
@end

@implementation MainWindowController

- (instancetype)init {
    NSRect frame = NSMakeRect(0, 0, 980, 760);
    NSWindow* window = [[NSWindow alloc] initWithContentRect:frame
                                                     styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                                                                NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable)
                                                       backing:NSBackingStoreBuffered
                                                         defer:NO];
    window.title = @"Gradient Mesh Studio";
    window.minSize = NSMakeSize(720, 560);
    [window center];

    if ((self = [super initWithWindow:window])) {
        window.delegate = self;
        self.documentModel = [[DocumentModel alloc] init];
        [self buildUI];
    }
    return self;
}

#pragma mark - UI construction

- (NSButton*)buttonTitled:(NSString*)title action:(SEL)action {
    NSButton* b = [NSButton buttonWithTitle:title target:self action:action];
    b.translatesAutoresizingMaskIntoConstraints = NO;
    return b;
}

- (void)buildUI {
    NSView* content = self.window.contentView;

    NSView* controlsRow1 = [self makeRow];
    NSView* controlsRow2 = [self makeRow];
    NSView* controlsRow3 = [self makeRow];

    // --- Row 1: file + tool selection ---
    NSButton* openBtn = [self buttonTitled:@"Open Image…" action:@selector(openImage:)];
    NSButton* autoBtn = [self buttonTitled:@"Auto (no markup)" action:@selector(autoMesh:)];

    self.toolSegmented = [[NSSegmentedControl alloc] init];
    self.toolSegmented.translatesAutoresizingMaskIntoConstraints = NO;
    self.toolSegmented.segmentCount = 4;
    NSArray* labels = @[@"1. Trace Boundary", @"2. Pick 4 Corners", @"3. Vector Line", @"4. Edit Mesh"];
    for (NSUInteger i = 0; i < labels.count; ++i) {
        [self.toolSegmented setLabel:labels[i] forSegment:i];
        [self.toolSegmented setWidth:120 forSegment:i];
    }
    self.toolSegmented.target = self;
    self.toolSegmented.action = @selector(toolChanged:);

    NSButton* clearLineBtn = [self buttonTitled:@"Clear Last Line" action:@selector(clearLastLine:)];

    for (NSView* v in @[openBtn, autoBtn, self.toolSegmented, clearLineBtn]) [controlsRow1 addSubview:v];

    // --- Row 2: mesh + optimize + export ---
    NSTextField* rowsLabel = [self makeLabel:@"Rows:"];
    self.rowsField = [self makeNumberFieldWithValue:@"9"];
    NSTextField* colsLabel = [self makeLabel:@"Cols:"];
    self.colsField = [self makeNumberFieldWithValue:@"9"];
    self.buildMeshButton = [self buttonTitled:@"Build Initial Mesh" action:@selector(buildMesh:)];
    self.optimizeButton = [self buttonTitled:@"Optimize" action:@selector(optimize:)];
    self.previewCheckbox = [NSButton checkboxWithTitle:@"Show reconstruction" target:self action:@selector(togglePreview:)];
    self.previewCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.tangentsCheckbox = [NSButton checkboxWithTitle:@"Show tangents" target:self action:@selector(toggleTangents:)];
    self.tangentsCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.exportPNGButton = [self buttonTitled:@"Export PNG…" action:@selector(exportPNG:)];
    self.exportSVGButton = [self buttonTitled:@"Export SVG…" action:@selector(exportSVG:)];
    self.progressSpinner = [[NSProgressIndicator alloc] init];
    self.progressSpinner.translatesAutoresizingMaskIntoConstraints = NO;
    self.progressSpinner.style = NSProgressIndicatorStyleSpinning;
    self.progressSpinner.controlSize = NSControlSizeSmall;
    self.progressSpinner.displayedWhenStopped = NO;
    self.statusLabel = [self makeLabel:@"Open an image to begin."];
    self.statusLabel.translatesAutoresizingMaskIntoConstraints = NO;

    for (NSView* v in @[rowsLabel, self.rowsField, colsLabel, self.colsField, self.buildMeshButton,
                         self.optimizeButton, self.progressSpinner, self.previewCheckbox, self.tangentsCheckbox,
                         self.exportPNGButton, self.exportSVGButton])
        [controlsRow2 addSubview:v];

    // --- Row 3: solver picker (hand-rolled vs Ceres geometry-only vs Ceres joint) ---
    NSTextField* solverLabel = [self makeLabel:@"Solver:"];
    self.solverPopup = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    self.solverPopup.translatesAutoresizingMaskIntoConstraints = NO;
    [self.solverPopup addItemsWithTitles:@[@"Hand-rolled", @"Ceres (geometry)", @"Ceres (joint)"]];
    [self.solverPopup selectItemAtIndex:0];
    self.solverPopup.target = self;
    self.solverPopup.action = @selector(solverChanged:);
    NSTextField* solverHint = [self makeLabel:@"(Ceres options are a no-op, with a console warning, in a build without Ceres found)"];
    solverHint.textColor = [NSColor secondaryLabelColor];
    solverHint.font = [NSFont systemFontOfSize:11];

    // Mirrors DocumentModel.autoExportDebugData -- see -toggleAutoDebug:
    // and that property's own comment for what gets written (a timestamped
    // JSON per run, in a "DebugOut" folder next to the loaded image) and
    // why (offline analysis of a solver run without a local Ceres build).
    self.autoDebugCheckbox = [NSButton checkboxWithTitle:@"Auto-export debug data"
                                                     target:self action:@selector(toggleAutoDebug:)];
    self.autoDebugCheckbox.translatesAutoresizingMaskIntoConstraints = NO;

    // Mirrors DocumentModel.useCIELUVColorSpace -- see -toggleCIELUV: and
    // that property's own doc comment (Hogervorst 2017's finding that
    // CIELUV is the best colour space for gradient-mesh colour
    // interpolation). Takes effect on the NEXT "Build Initial Mesh", not
    // retroactively -- see that comment for why.
    self.cieluvCheckbox = [NSButton checkboxWithTitle:@"CIELUV color space"
                                                  target:self action:@selector(toggleCIELUV:)];
    self.cieluvCheckbox.translatesAutoresizingMaskIntoConstraints = NO;

    for (NSView* v in @[solverLabel, self.solverPopup, solverHint, self.autoDebugCheckbox, self.cieluvCheckbox])
        [controlsRow3 addSubview:v];

    self.canvasView = [[CanvasView alloc] initWithFrame:NSZeroRect];
    self.canvasView.translatesAutoresizingMaskIntoConstraints = NO;
    self.canvasView.documentModel = self.documentModel;
    [self wireCanvasCallbacks];

    [content addSubview:controlsRow1];
    [content addSubview:controlsRow2];
    [content addSubview:controlsRow3];
    [content addSubview:self.canvasView];
    [content addSubview:self.statusLabel];

    NSDictionary* views = NSDictionaryOfVariableBindings(controlsRow1, controlsRow2, controlsRow3, _canvasView, _statusLabel);
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow1]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow2]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow3]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-0-[_canvasView]-0-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[_statusLabel]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:
        @"V:|-8-[controlsRow1(28)]-6-[controlsRow2(28)]-6-[controlsRow3(28)]-6-[_canvasView]-4-[_statusLabel(18)]-6-|"
                                                                    options:0 metrics:nil views:views]];

    [self layoutRowChildren:controlsRow1];
    [self layoutRowChildren:controlsRow2];
    [self layoutRowChildren:controlsRow3];
}

- (NSView*)makeRow {
    NSView* v = [[NSView alloc] initWithFrame:NSZeroRect];
    v.translatesAutoresizingMaskIntoConstraints = NO;
    return v;
}

- (NSTextField*)makeLabel:(NSString*)text {
    NSTextField* l = [NSTextField labelWithString:text];
    l.translatesAutoresizingMaskIntoConstraints = NO;
    return l;
}

- (NSTextField*)makeNumberFieldWithValue:(NSString*)value {
    NSTextField* f = [NSTextField textFieldWithString:value];
    f.translatesAutoresizingMaskIntoConstraints = NO;
    [f.widthAnchor constraintEqualToConstant:44].active = YES;
    return f;
}

// Lays out a row's children left-to-right with simple fixed spacing using
// their own leading/trailing/centerY anchors against the row view.
- (void)layoutRowChildren:(NSView*)row {
    NSView* prev = nil;
    for (NSView* v in row.subviews) {
        [v.centerYAnchor constraintEqualToAnchor:row.centerYAnchor].active = YES;
        if (prev) {
            [v.leadingAnchor constraintEqualToAnchor:prev.trailingAnchor constant:10].active = YES;
        } else {
            [v.leadingAnchor constraintEqualToAnchor:row.leadingAnchor].active = YES;
        }
        prev = v;
    }
}

#pragma mark - Canvas callbacks

- (void)wireCanvasCallbacks {
    __weak typeof(self) weakSelf = self;
    self.canvasView.onBoundaryChanged = ^{
        weakSelf.statusLabel.stringValue = @"Boundary traced. Switch to “Pick 4 Corners” and click the 4 corner points (in order).";
    };
    self.canvasView.onCornersPicked = ^(NSArray<NSNumber*>* indices) {
        BOOL ok = [weakSelf.documentModel fitBoundaryWithCornerIndices:indices];
        weakSelf.statusLabel.stringValue = ok ? @"4 boundary sides fitted. Set Rows/Cols and click “Build Initial Mesh”."
                                               : @"Could not fit boundary sides -- try tracing again.";
        [weakSelf.canvasView setNeedsDisplay:YES];
    };
    self.canvasView.onVectorLineFinished = ^(NSArray<NSValue*>* points) {
        [weakSelf.documentModel addVectorLineWithPoints:points];
        [weakSelf.canvasView setNeedsDisplay:YES];
    };
    self.canvasView.onMeshEdited = ^{
        weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"RMSE (unrefit): %.4f%@", weakSelf.documentModel.currentRMSE,
                                             weakSelf.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units)" : @""];
    };
}

#pragma mark - Actions

- (void)openImage:(id)sender {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.allowsMultipleSelection = NO;
    panel.canChooseDirectories = NO;
    if (@available(macOS 11.0, *)) {
        panel.allowedContentTypes = @[UTTypeImage];
    } else {
        panel.allowedFileTypes = @[@"png", @"jpg", @"jpeg", @"tiff", @"bmp", @"heic"];
    }
    __weak typeof(self) weakSelf = self;
    [panel beginSheetModalForWindow:self.window completionHandler:^(NSModalResponse result) {
        if (result != NSModalResponseOK) return;
        NSError* error = nil;
        if (![weakSelf.documentModel loadImageAtURL:panel.URL error:&error]) {
            [weakSelf presentError:error];
            return;
        }
        [weakSelf.canvasView resetBoundaryDrawing];
        [weakSelf.canvasView resetCornerPicking];
        weakSelf.canvasView.showReconstructionPreview = NO;
        weakSelf.canvasView.toolMode = GMToolModeBoundary;
        [weakSelf.toolSegmented setSelected:YES forSegment:0];
        weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"Loaded %ld×%ld image. Trace the object boundary (click points, double-click to close), or use “Auto (no markup)”.",
                                             (long)weakSelf.documentModel.imageWidth, (long)weakSelf.documentModel.imageHeight];
        [weakSelf.canvasView setNeedsDisplay:YES];
    }];
}

- (void)toolChanged:(id)sender {
    switch (self.toolSegmented.selectedSegment) {
        case 0: self.canvasView.toolMode = GMToolModeBoundary; break;
        case 1: self.canvasView.toolMode = GMToolModeCorners; break;
        case 2: self.canvasView.toolMode = GMToolModeVectorLine; break;
        case 3: self.canvasView.toolMode = GMToolModeEditMesh; break;
    }
}

- (void)clearLastLine:(id)sender {
    [self.documentModel removeLastVectorLine];
    [self.canvasView setNeedsDisplay:YES];
}

- (void)autoMesh:(id)sender {
    if (!self.documentModel.hasImage) { self.statusLabel.stringValue = @"Open an image first."; return; }
    // 0 margin: boundary exactly matches the image's own edges (0,0)-(w,h),
    // per explicit request -- was 6.0 (a small cosmetic inset with no
    // functional reason: Image::sampleBilinear/sampleGradient clamp to
    // valid pixel coordinates internally, so sampling exactly at the image
    // edge is safe, not an out-of-bounds risk).
    [self.documentModel useRectangularBoundaryWithMargin:0.0];
    NSInteger rows = MAX(3, self.rowsField.integerValue), cols = MAX(3, self.colsField.integerValue);
    [self.documentModel buildInitialMeshRows:rows cols:cols];
    self.canvasView.toolMode = GMToolModeEditMesh;
    [self.toolSegmented setSelected:YES forSegment:3];
    self.statusLabel.stringValue = [NSString stringWithFormat:@"Auto rectangular boundary + %ldx%ld grid built. RMSE=%.4f%@. Click Optimize.",
                                     (long)rows, (long)cols, self.documentModel.currentRMSE,
                                     self.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units)" : @""];
    [self.canvasView setNeedsDisplay:YES];
}

- (void)buildMesh:(id)sender {
    if (!self.documentModel.hasBoundary) { self.statusLabel.stringValue = @"Trace a boundary and pick its 4 corners first."; return; }
    NSInteger rows = MAX(3, self.rowsField.integerValue), cols = MAX(3, self.colsField.integerValue);
    [self.documentModel buildInitialMeshRows:rows cols:cols];
    self.canvasView.toolMode = GMToolModeEditMesh;
    [self.toolSegmented setSelected:YES forSegment:3];
    self.statusLabel.stringValue = [NSString stringWithFormat:@"Initial %ldx%ld mesh built. RMSE=%.4f%@. Optionally draw vector lines, then click Optimize.",
                                     (long)rows, (long)cols, self.documentModel.currentRMSE,
                                     self.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units)" : @""];
    [self.canvasView setNeedsDisplay:YES];
}

- (void)optimize:(id)sender {
    if (!self.documentModel.hasMesh) { self.statusLabel.stringValue = @"Build a mesh first."; return; }
    self.optimizeButton.enabled = NO;
    self.buildMeshButton.enabled = NO;
    [self.progressSpinner startAnimation:nil];
    __weak typeof(self) weakSelf = self;
    [self.documentModel optimizeWithPyramidLevels:4
        progress:^(double rmse, NSInteger level, NSInteger totalLevels, NSInteger iter, NSInteger totalIters) {
            // See DocumentModel.h's useCIELUVColorSpace comment: a CIELUV
            // run's RMSE is in different units (L* roughly 0..100) than an
            // sRGB run's -- flagged here so it's never mistaken for a huge
            // regression/improvement at a glance.
            NSString* unitTag = weakSelf.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units)" : @"";
            weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"Optimizing… pyramid level %ld/%ld, iteration %ld/%ld, RMSE=%.4f%@",
                                                 (long)level, (long)(totalLevels - 1), (long)iter, (long)(totalIters - 1), rmse, unitTag];
        }
        completion:^{
            [weakSelf.progressSpinner stopAnimation:nil];
            weakSelf.optimizeButton.enabled = YES;
            weakSelf.buildMeshButton.enabled = YES;
            NSString* unitTag = weakSelf.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units -- not comparable to sRGB-mode RMSE)" : @"";
            NSString* msg = [NSString stringWithFormat:@"Done. Final RMSE=%.4f%@.", weakSelf.documentModel.currentRMSE, unitTag];
            // -lastDebugExportPath is only non-nil right after a run that
            // both had autoExportDebugData ON and wrote successfully -- see
            // DocumentModel -autoExportDebugDataIfEnabled.
            if (weakSelf.documentModel.lastDebugExportPath) {
                msg = [msg stringByAppendingFormat:@" Debug log: %@", weakSelf.documentModel.lastDebugExportPath];
            }
            weakSelf.statusLabel.stringValue = msg;
            [weakSelf.canvasView refreshReconstructionPreview];
            [weakSelf.canvasView setNeedsDisplay:YES];
        }];
}

- (void)togglePreview:(id)sender {
    self.canvasView.showReconstructionPreview = (self.previewCheckbox.state == NSControlStateValueOn);
    if (self.canvasView.showReconstructionPreview) [self.canvasView refreshReconstructionPreview];
    [self.canvasView setNeedsDisplay:YES];
}

- (void)toggleTangents:(id)sender {
    self.canvasView.showTangents = (self.tangentsCheckbox.state == NSControlStateValueOn);
    [self.canvasView setNeedsDisplay:YES];
}

- (void)solverChanged:(id)sender {
    // Index 0 "Hand-rolled": both NO (the original, dependency-free path).
    // Index 1 "Ceres (geometry)": useCeresGeometry only.
    // Index 2 "Ceres (joint)": useCeresJoint (which alone implies "joint
    // wins" even if useCeresGeometry were also left on -- see
    // DocumentModel.h/MeshOptimizer.cpp's jointSolvedByCeres gating -- so
    // leaving useCeresGeometry off here is just the clearest way to express
    // "exactly one of these three modes" from this 3-item picker).
    NSInteger idx = self.solverPopup.indexOfSelectedItem;
    self.documentModel.useCeresGeometry = (idx == 1);
    self.documentModel.useCeresJoint = (idx == 2);
    NSArray* names = @[@"hand-rolled", @"Ceres (geometry)", @"Ceres (joint)"];
    self.statusLabel.stringValue = [NSString stringWithFormat:@"Solver set to %@. (Next “Optimize” run will use it.)",
                                     names[(NSUInteger)MAX(0, idx)]];
}

- (void)exportPNG:(id)sender {
    if (!self.documentModel.hasMesh) { self.statusLabel.stringValue = @"Build (and ideally optimize) a mesh first."; return; }
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"gradient-mesh-reconstruction.png";
    __weak typeof(self) weakSelf = self;
    [panel beginSheetModalForWindow:self.window completionHandler:^(NSModalResponse result) {
        if (result != NSModalResponseOK) return;
        NSError* error = nil;
        if (![weakSelf.documentModel exportPNGToURL:panel.URL error:&error]) [weakSelf presentError:error];
        else weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"Exported PNG to %@", panel.URL.path];
    }];
}

- (void)exportSVG:(id)sender {
    if (!self.documentModel.hasMesh) { self.statusLabel.stringValue = @"Build a mesh first."; return; }
    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"gradient-mesh.svg";
    __weak typeof(self) weakSelf = self;
    [panel beginSheetModalForWindow:self.window completionHandler:^(NSModalResponse result) {
        if (result != NSModalResponseOK) return;
        NSError* error = nil;
        if (![weakSelf.documentModel exportSVGToURL:panel.URL error:&error]) [weakSelf presentError:error];
        else weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"Exported SVG (mesh gradient) to %@", panel.URL.path];
    }];
}

- (void)toggleAutoDebug:(id)sender {
    // See DocumentModel.h's autoExportDebugData comment: when ON, every
    // completed "Optimize" run writes a timestamped debug JSON to a
    // "DebugOut" folder next to the loaded image, with no save dialog --
    // replaces the old one-shot "Export Debug Data…" button, which needed
    // a manual click (and manual filename bookkeeping to avoid overwriting
    // a previous run) after every single run you wanted to keep.
    self.documentModel.autoExportDebugData = (self.autoDebugCheckbox.state == NSControlStateValueOn);
    self.statusLabel.stringValue = self.documentModel.autoExportDebugData
        ? @"Auto-export debug data: ON. Each “Optimize” run will write a timestamped JSON to DebugOut/ next to the image."
        : @"Auto-export debug data: OFF.";
}

- (void)toggleCIELUV:(id)sender {
    // See DocumentModel.h's useCIELUVColorSpace comment: only takes effect
    // at the next "Build Initial Mesh" -- an already-built mesh keeps
    // whichever space it was built in regardless of this toggle, so the
    // status message says so explicitly rather than implying an immediate
    // effect the way -solverChanged:'s message does for the next Optimize.
    self.documentModel.useCIELUVColorSpace = (self.cieluvCheckbox.state == NSControlStateValueOn);
    self.statusLabel.stringValue = self.documentModel.useCIELUVColorSpace
        ? @"CIELUV color space: ON. Click “Build Initial Mesh” (or “Auto”) again to rebuild in CIELUV -- an existing mesh is unaffected until then."
        : @"CIELUV color space: OFF (sRGB). Click “Build Initial Mesh” (or “Auto”) again to rebuild in sRGB -- an existing mesh is unaffected until then.";
}

- (void)presentError:(NSError*)error {
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = @"Gradient Mesh Studio";
    alert.informativeText = error.localizedDescription ?: @"An unknown error occurred.";
    [alert beginSheetModalForWindow:self.window completionHandler:nil];
}

@end
