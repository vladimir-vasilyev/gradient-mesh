#import "MainWindowController.h"
#import "CanvasView.h"
#import "DocumentModel.h"
#import "GLReconstructionView.h"
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>
#import <CoreGraphics/CoreGraphics.h>  // CGImageDestinationRef etc. for -exportGPUPNG: (mirrors DocumentModel.mm's -exportPNGToURL:)
#import <ImageIO/ImageIO.h>            // CGImageDestinationCreateWithURL/AddImage/Finalize

@interface MainWindowController () <NSWindowDelegate, NSMenuDelegate>
@property (nonatomic, strong) DocumentModel* documentModel;
@property (nonatomic, strong) CanvasView* canvasView;
// GPU (OpenGL) counterpart to canvasView's CPU reconstruction preview --
// see GLReconstructionView.h and -refreshActivePreview. Stacked exactly
// on top of canvasView within canvasRow (see -buildUI); exactly one of the
// two is visible at a time.
@property (nonatomic, strong) GLReconstructionView* glReconstructionView;
@property (nonatomic, strong) NSButton* gpuPreviewCheckbox;
@property (nonatomic, strong) NSSegmentedControl* toolSegmented;
@property (nonatomic, strong) NSTextField* rowsField;
@property (nonatomic, strong) NSTextField* colsField;
@property (nonatomic, strong) NSTextField* statusLabel;
@property (nonatomic, strong) NSProgressIndicator* progressSpinner;
@property (nonatomic, strong) NSButton* previewCheckbox;
// Mirrors CanvasView.showMeshOverlay -- see -toggleShowMesh:. Checked by
// default, matching CanvasView's own default (_showMeshOverlay = YES in
// -initWithFrame:).
@property (nonatomic, strong) NSButton* meshCheckbox;
@property (nonatomic, strong) NSButton* tangentsCheckbox;
// Mirrors DocumentModel.livePreviewDuringOptimize -- see -toggleLivePreview:.
// When on, the mesh grid overlay redraws once per outer iteration during
// -optimize: instead of staying frozen until the run completes.
@property (nonatomic, strong) NSButton* livePreviewCheckbox;
@property (nonatomic, strong) NSButton* buildMeshButton;
@property (nonatomic, strong) NSButton* optimizeButton;
@property (nonatomic, strong) NSButton* exportPNGButton;
@property (nonatomic, strong) NSButton* exportSVGButton;
// Saves exactly what glReconstructionView renders (GPU/OpenGL, exact
// per-pixel Hermite color) rather than DocumentModel's CPU rasterization
// -- see -exportGPUPNG: and GLReconstructionView.renderToImageWithWidth:height:.
// Independent of whether the on-screen GPU preview toggle is currently on.
@property (nonatomic, strong) NSButton* exportGPUPNGButton;
// Solver picker: Hand-rolled (default) / Ceres (geometry) / Ceres (joint) --
// mirrors gmcore::OptimizerOptions::useCeresGeometry/useCeresJoint via
// DocumentModel's properties of the same name. See -solverChanged:.
@property (nonatomic, strong) NSPopUpButton* solverPopup;
// Mirrors DocumentModel.autoExportDebugData -- see -toggleAutoDebug:.
@property (nonatomic, strong) NSButton* autoDebugCheckbox;
// Mirrors DocumentModel.useCIELUVColorSpace -- see -toggleCIELUV:.
@property (nonatomic, strong) NSButton* cieluvCheckbox;
// Mirrors DocumentModel.ceresMultithreaded -- see -toggleCeresMultithreaded:.
@property (nonatomic, strong) NSButton* ceresMultithreadedCheckbox;
// Geometry/colour energy-weight fields -- mirror DocumentModel's eight
// properties of the same name 1:1 (see that header's comment). Read
// straight into the model at the start of -optimize:, same pattern as
// rowsField/colsField already use for -buildMesh:/-autoMesh:.
@property (nonatomic, strong) NSTextField* smoothWeightGeomField;
@property (nonatomic, strong) NSTextField* geomDataWeightField;
@property (nonatomic, strong) NSTextField* smoothGeomEdgeGainField;
@property (nonatomic, strong) NSTextField* boundaryWeightField;
@property (nonatomic, strong) NSTextField* geomTangentPriorWeightField;
@property (nonatomic, strong) NSTextField* vectorLineWeightField;
@property (nonatomic, strong) NSTextField* smoothWeightColorField;
@property (nonatomic, strong) NSTextField* colorDerivRidgeField;
// Small sidebar to the right of canvasView (see -buildUI's canvasRow) --
// "Save Preset…" snapshots the settings the last completed Optimize run
// used (see DocumentModel -savePresetToURL:error:); the popup below it
// lists+loads any preset previously saved next to the current image. See
// -savePreset:/-presetsPopupWillOpen:/-presetSelected:.
@property (nonatomic, strong) NSButton* savePresetButton;
@property (nonatomic, strong) NSPopUpButton* presetsPopup;
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
    NSView* controlsRow1b = [self makeRow];
    NSView* controlsRow2 = [self makeRow];
    NSView* controlsRow3 = [self makeRow];
    NSView* controlsRow4 = [self makeRow];
    NSView* controlsRow5 = [self makeRow];

    // --- Row 1: file + tool selection ---
    NSButton* openBtn = [self buttonTitled:@"Open Image…" action:@selector(openImage:)];
    NSButton* autoBtn = [self buttonTitled:@"Auto (no markup)" action:@selector(autoMesh:)];

    self.toolSegmented = [[NSSegmentedControl alloc] init];
    self.toolSegmented.translatesAutoresizingMaskIntoConstraints = NO;
    self.toolSegmented.segmentCount = 6;
    // Scribble FG/BG are the new Lazy-Snapping-style cutout tool (see
    // -segmentBoundary: below); Corners/Vector Line/Edit Mesh are
    // renumbered 3/4 to make room but are otherwise unchanged.
    NSArray* labels = @[@"1. Trace Boundary", @"1. Scribble FG", @"1. Scribble BG",
                        @"2. Pick 4 Corners", @"3. Vector Line", @"4. Edit Mesh"];
    for (NSUInteger i = 0; i < labels.count; ++i) {
        [self.toolSegmented setLabel:labels[i] forSegment:i];
        [self.toolSegmented setWidth:110 forSegment:i];
    }
    self.toolSegmented.target = self;
    self.toolSegmented.action = @selector(toolChanged:);

    NSButton* clearLineBtn = [self buttonTitled:@"Clear Last Line" action:@selector(clearLastLine:)];

    for (NSView* v in @[openBtn, autoBtn, self.toolSegmented, clearLineBtn]) [controlsRow1 addSubview:v];

    // --- Row 1b: Lazy-Snapping-style segmentation (scribble tools above) ---
    NSButton* segmentBtn = [self buttonTitled:@"Segment" action:@selector(segmentBoundary:)];
    NSButton* clearScribblesBtn = [self buttonTitled:@"Clear Scribbles" action:@selector(clearScribbles:)];
    NSTextField* scribbleHintLabel = [self makeLabel:@"(scribble foreground/background above, then Segment)"];
    for (NSView* v in @[segmentBtn, clearScribblesBtn, scribbleHintLabel]) [controlsRow1b addSubview:v];

    // --- Row 2: mesh + optimize + export ---
    NSTextField* rowsLabel = [self makeLabel:@"Rows:"];
    self.rowsField = [self makeNumberFieldWithValue:@"9"];
    NSTextField* colsLabel = [self makeLabel:@"Cols:"];
    self.colsField = [self makeNumberFieldWithValue:@"9"];
    self.buildMeshButton = [self buttonTitled:@"Build Initial Mesh" action:@selector(buildMesh:)];
    self.optimizeButton = [self buttonTitled:@"Optimize" action:@selector(optimize:)];
    self.previewCheckbox = [NSButton checkboxWithTitle:@"Show reconstruction" target:self action:@selector(togglePreview:)];
    self.previewCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    // Sub-toggle of previewCheckbox: when both are on, the GPU/OpenGL
    // renderer (GLReconstructionView) replaces the CPU one -- see
    // -refreshActivePreview. Meaningless (and left unused) while
    // previewCheckbox itself is off.
    self.gpuPreviewCheckbox = [NSButton checkboxWithTitle:@"GPU (OpenGL)" target:self action:@selector(toggleGPUPreview:)];
    self.gpuPreviewCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.meshCheckbox = [NSButton checkboxWithTitle:@"Show mesh" target:self action:@selector(toggleShowMesh:)];
    self.meshCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.meshCheckbox.state = NSControlStateValueOn;  // matches CanvasView.showMeshOverlay's default (YES)
    self.tangentsCheckbox = [NSButton checkboxWithTitle:@"Show tangents" target:self action:@selector(toggleTangents:)];
    self.tangentsCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.livePreviewCheckbox = [NSButton checkboxWithTitle:@"Live mesh preview" target:self action:@selector(toggleLivePreview:)];
    self.livePreviewCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.exportPNGButton = [self buttonTitled:@"Export PNG…" action:@selector(exportPNG:)];
    self.exportSVGButton = [self buttonTitled:@"Export SVG…" action:@selector(exportSVG:)];
    self.exportGPUPNGButton = [self buttonTitled:@"Export GPU PNG…" action:@selector(exportGPUPNG:)];
    self.progressSpinner = [[NSProgressIndicator alloc] init];
    self.progressSpinner.translatesAutoresizingMaskIntoConstraints = NO;
    self.progressSpinner.style = NSProgressIndicatorStyleSpinning;
    self.progressSpinner.controlSize = NSControlSizeSmall;
    self.progressSpinner.displayedWhenStopped = NO;
    self.statusLabel = [self makeLabel:@"Open an image to begin."];
    self.statusLabel.translatesAutoresizingMaskIntoConstraints = NO;

    for (NSView* v in @[rowsLabel, self.rowsField, colsLabel, self.colsField, self.buildMeshButton,
                         self.optimizeButton, self.progressSpinner, self.previewCheckbox, self.gpuPreviewCheckbox,
                         self.meshCheckbox, self.tangentsCheckbox, self.livePreviewCheckbox, self.exportPNGButton, self.exportSVGButton,
                         self.exportGPUPNGButton])
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

    // Mirrors DocumentModel.ceresMultithreaded -- see -toggleCeresMultithreaded:
    // and that property's own comment. Only affects the two Ceres solver
    // modes (has no effect on hand-rolled, and no effect at all without a
    // Ceres-enabled build) -- checked ON by default (every core), same as
    // the behavior this toggle was added to make optional.
    self.ceresMultithreadedCheckbox = [NSButton checkboxWithTitle:@"Multithread Ceres"
                                                             target:self action:@selector(toggleCeresMultithreaded:)];
    self.ceresMultithreadedCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.ceresMultithreadedCheckbox.state = NSControlStateValueOn;

    for (NSView* v in @[solverLabel, self.solverPopup, solverHint, self.autoDebugCheckbox, self.cieluvCheckbox, self.ceresMultithreadedCheckbox])
        [controlsRow3 addSubview:v];

    // --- Row 4: geometry energy weights -- see DocumentModel.h's comment
    // on these properties (mirrors gmcore::OptimizerOptions, MeshOptimizer.h
    // has each one's full derivation/tuning history). Read into the model
    // at the start of -optimize:, so a typed value takes effect on the next
    // "Optimize" click, same as Rows/Cols already work for "Build Initial
    // Mesh". Initial field text comes from documentModel's own -init
    // (which seeds them from OptimizerOptions' real compiled-in defaults),
    // not a second hardcoded copy of those numbers.
    NSTextField* geomWeightsLabel = [self makeLabel:@"Geometry weights —"];
    NSTextField* smoothGeomLabel = [self makeLabel:@"Smooth:"];
    self.smoothWeightGeomField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.smoothWeightGeom]];
    // "Data:" -- OptimizerOptions::geomDataWeight, the single-knob way to
    // strengthen/weaken the geometry step's photometric term against the
    // other four fields in this row (see MeshOptimizer.h's comment on that
    // field, and DocumentModel.h's comment on this property).
    NSTextField* geomDataLabel = [self makeLabel:@"Data:"];
    self.geomDataWeightField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.geomDataWeight]];
    // "Edge gain:" -- OptimizerOptions::smoothGeomEdgeGain, makes the
    // "Smooth:" field above ANISOTROPIC near strong local image gradients
    // instead of a single flat weight (see MeshOptimizer.h's comment on
    // that field and DocumentModel.h's comment on this property for the
    // real, on-device-measured trade-off: tighter edge-snapping on a
    // coarse mesh vs. a regression on smooth images -- NOT a strictly
    // better value, which is why it stays a separate opt-in field rather
    // than a change to the compiled-in default).
    NSTextField* edgeGainLabel = [self makeLabel:@"Edge gain:"];
    self.smoothGeomEdgeGainField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.smoothGeomEdgeGain]];
    NSTextField* boundaryLabel = [self makeLabel:@"Boundary:"];
    self.boundaryWeightField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.boundaryWeight]];
    NSTextField* tangentPriorLabel = [self makeLabel:@"Tangent prior:"];
    self.geomTangentPriorWeightField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.geomTangentPriorWeight]];
    NSTextField* vectorLineLabel = [self makeLabel:@"Vector line:"];
    self.vectorLineWeightField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.vectorLineWeight]];

    for (NSView* v in @[geomWeightsLabel, smoothGeomLabel, self.smoothWeightGeomField, geomDataLabel,
                         self.geomDataWeightField, edgeGainLabel, self.smoothGeomEdgeGainField, boundaryLabel,
                         self.boundaryWeightField, tangentPriorLabel, self.geomTangentPriorWeightField,
                         vectorLineLabel, self.vectorLineWeightField])
        [controlsRow4 addSubview:v];

    // --- Row 5: colour energy weights, + a shared reset for all eight ---
    NSTextField* colorWeightsLabel = [self makeLabel:@"Color weights —"];
    NSTextField* smoothColorLabel = [self makeLabel:@"Smooth:"];
    self.smoothWeightColorField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.smoothWeightColor]];
    NSTextField* colorRidgeLabel = [self makeLabel:@"Ridge:"];
    self.colorDerivRidgeField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.colorDerivRidge]];
    NSButton* resetWeightsBtn = [self buttonTitled:@"Reset weights to defaults" action:@selector(resetWeights:)];

    for (NSView* v in @[colorWeightsLabel, smoothColorLabel, self.smoothWeightColorField, colorRidgeLabel,
                         self.colorDerivRidgeField, resetWeightsBtn])
        [controlsRow5 addSubview:v];

    self.canvasView = [[CanvasView alloc] initWithFrame:NSZeroRect];
    self.canvasView.translatesAutoresizingMaskIntoConstraints = NO;
    self.canvasView.documentModel = self.documentModel;
    [self wireCanvasCallbacks];

    // GPU (OpenGL) preview view -- stacked exactly on top of canvasView
    // (see the canvasRow constraints below), hidden until
    // -refreshActivePreview turns it on. See GLReconstructionView.h.
    self.glReconstructionView = [[GLReconstructionView alloc] initWithFrame:NSZeroRect];
    self.glReconstructionView.translatesAutoresizingMaskIntoConstraints = NO;
    self.glReconstructionView.hidden = YES;

    // --- Preset sidebar, next to the canvas (see canvasRow below) ---
    // "Save Preset…" writes the LAST completed Optimize run's settings
    // (DocumentModel -savePresetToURL:error:) to a timestamped JSON in a
    // "Presets" folder next to the loaded image, named the same way
    // -autoExportDebugData's log files are (see -savePreset:). The popup
    // below it lists (newest first) + loads any preset already saved next
    // to the current image -- its items are rebuilt from
    // -availablePresetNames right before it opens (see -menuNeedsUpdate:),
    // so a preset saved a moment ago always shows up without relaunching.
    self.savePresetButton = [self buttonTitled:@"Save Preset…" action:@selector(savePreset:)];
    self.presetsPopup = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:YES];
    self.presetsPopup.translatesAutoresizingMaskIntoConstraints = NO;
    [self.presetsPopup addItemWithTitle:@"Load Preset…"];
    self.presetsPopup.target = self;
    self.presetsPopup.action = @selector(presetSelected:);
    self.presetsPopup.menu.delegate = self;

    NSView* presetSidebar = [[NSView alloc] initWithFrame:NSZeroRect];
    presetSidebar.translatesAutoresizingMaskIntoConstraints = NO;
    [presetSidebar addSubview:self.savePresetButton];
    [presetSidebar addSubview:self.presetsPopup];
    [self.savePresetButton.topAnchor constraintEqualToAnchor:presetSidebar.topAnchor constant:8].active = YES;
    [self.savePresetButton.leadingAnchor constraintEqualToAnchor:presetSidebar.leadingAnchor].active = YES;
    [self.savePresetButton.trailingAnchor constraintEqualToAnchor:presetSidebar.trailingAnchor].active = YES;
    [self.presetsPopup.topAnchor constraintEqualToAnchor:self.savePresetButton.bottomAnchor constant:6].active = YES;
    [self.presetsPopup.leadingAnchor constraintEqualToAnchor:presetSidebar.leadingAnchor].active = YES;
    [self.presetsPopup.trailingAnchor constraintEqualToAnchor:presetSidebar.trailingAnchor].active = YES;

    // canvasRow: canvasView + presetSidebar side by side, replacing
    // canvasView's old direct placement in the outer vertical stack below
    // -- everything about canvasView itself (documentModel, callbacks,
    // CanvasView's own drawing) is unchanged, it just now shares its row
    // with the sidebar instead of spanning the full window width.
    NSView* canvasRow = [[NSView alloc] initWithFrame:NSZeroRect];
    canvasRow.translatesAutoresizingMaskIntoConstraints = NO;
    [canvasRow addSubview:self.canvasView];
    [canvasRow addSubview:self.glReconstructionView];
    [canvasRow addSubview:presetSidebar];
    // glReconstructionView exactly overlays canvasView (not part of the
    // visual-format layout below, which only positions _canvasView/
    // presetSidebar within canvasRow) -- see -refreshActivePreview for how
    // the two are switched between.
    [self.glReconstructionView.leadingAnchor constraintEqualToAnchor:self.canvasView.leadingAnchor].active = YES;
    [self.glReconstructionView.trailingAnchor constraintEqualToAnchor:self.canvasView.trailingAnchor].active = YES;
    [self.glReconstructionView.topAnchor constraintEqualToAnchor:self.canvasView.topAnchor].active = YES;
    [self.glReconstructionView.bottomAnchor constraintEqualToAnchor:self.canvasView.bottomAnchor].active = YES;
    NSDictionary* canvasRowViews = NSDictionaryOfVariableBindings(_canvasView, presetSidebar);
    [canvasRow addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-0-[_canvasView]-8-[presetSidebar(150)]-8-|"
                                                                       options:0 metrics:nil views:canvasRowViews]];
    [self.canvasView.topAnchor constraintEqualToAnchor:canvasRow.topAnchor].active = YES;
    [self.canvasView.bottomAnchor constraintEqualToAnchor:canvasRow.bottomAnchor].active = YES;
    [presetSidebar.topAnchor constraintEqualToAnchor:canvasRow.topAnchor].active = YES;

    [content addSubview:controlsRow1];
    [content addSubview:controlsRow1b];
    [content addSubview:controlsRow2];
    [content addSubview:controlsRow3];
    [content addSubview:controlsRow4];
    [content addSubview:controlsRow5];
    [content addSubview:canvasRow];
    [content addSubview:self.statusLabel];

    NSDictionary* views = NSDictionaryOfVariableBindings(controlsRow1, controlsRow1b, controlsRow2, controlsRow3, controlsRow4,
                                                           controlsRow5, canvasRow, _statusLabel);
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow1]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow1b]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow2]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow3]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow4]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow5]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-0-[canvasRow]-0-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[_statusLabel]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:
        @"V:|-8-[controlsRow1(28)]-6-[controlsRow1b(28)]-6-[controlsRow2(28)]-6-[controlsRow3(28)]-6-[controlsRow4(28)]-6-[controlsRow5(28)]"
        "-6-[canvasRow]-4-[_statusLabel(18)]-6-|"
                                                                    options:0 metrics:nil views:views]];

    [self layoutRowChildren:controlsRow1];
    [self layoutRowChildren:controlsRow1b];
    [self layoutRowChildren:controlsRow2];
    [self layoutRowChildren:controlsRow3];
    [self layoutRowChildren:controlsRow4];
    [self layoutRowChildren:controlsRow5];
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

// Slightly wider than makeNumberFieldWithValue: (44pt, used for integer
// Rows/Cols) -- these show decimal weight values like "0.001" or "200",
// which need a touch more room. Kept as a separate helper rather than
// widening makeNumberFieldWithValue: itself so Rows/Cols' layout doesn't
// shift.
- (NSTextField*)makeWeightFieldWithValue:(NSString*)value {
    NSTextField* f = [NSTextField textFieldWithString:value];
    f.translatesAutoresizingMaskIntoConstraints = NO;
    f.font = [NSFont systemFontOfSize:11];
    [f.widthAnchor constraintEqualToConstant:54].active = YES;
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
    self.canvasView.onScribblesChanged = ^{
        weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"Scribbles: %lu foreground, %lu background. Click “Segment” when ready.",
                                             (unsigned long)weakSelf.documentModel.foregroundScribblePoints.count,
                                             (unsigned long)weakSelf.documentModel.backgroundScribblePoints.count];
        [weakSelf.canvasView setNeedsDisplay:YES];
    };
    self.canvasView.onMeshEdited = ^{
        weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"RMSE (unrefit): %.4f  MAE: %.4f%@",
                                             weakSelf.documentModel.currentRMSE, weakSelf.documentModel.currentMAE,
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
        weakSelf.canvasView.hidden = NO;
        weakSelf.previewCheckbox.state = NSControlStateValueOff;
        weakSelf.gpuPreviewCheckbox.state = NSControlStateValueOff;
        [weakSelf.glReconstructionView clearMesh];
        weakSelf.glReconstructionView.hidden = YES;
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
        case 1: self.canvasView.toolMode = GMToolModeScribbleForeground; break;
        case 2: self.canvasView.toolMode = GMToolModeScribbleBackground; break;
        case 3: self.canvasView.toolMode = GMToolModeCorners; break;
        case 4: self.canvasView.toolMode = GMToolModeVectorLine; break;
        case 5: self.canvasView.toolMode = GMToolModeEditMesh; break;
    }
}

- (void)clearLastLine:(id)sender {
    [self.documentModel removeLastVectorLine];
    [self.canvasView setNeedsDisplay:YES];
}

- (void)segmentBoundary:(id)sender {
    NSError* error = nil;
    if (![self.documentModel segmentBoundaryFromScribblesWithError:&error]) {
        [self presentError:error];
        return;
    }
    // Mirrors -wireCanvasCallbacks' onBoundaryChanged status message --
    // segmentBoundaryFromScribblesWithError: feeds the exact same
    // -setBoundaryPolygonPoints: entry point manual click-tracing does, so
    // the next step (corner-picking) is identical either way.
    self.statusLabel.stringValue = @"Boundary segmented from scribbles. Switch to “Pick 4 Corners” and click the 4 corner points (in order).";
    [self.canvasView setNeedsDisplay:YES];
}

- (void)clearScribbles:(id)sender {
    [self.documentModel clearScribbles];
    self.statusLabel.stringValue = @"Scribbles cleared.";
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
    [self.toolSegmented setSelected:YES forSegment:5];
    self.statusLabel.stringValue = [NSString stringWithFormat:@"Auto rectangular boundary + %ldx%ld grid built. RMSE=%.4f  MAE=%.4f%@. Click Optimize.",
                                     (long)rows, (long)cols, self.documentModel.currentRMSE, self.documentModel.currentMAE,
                                     self.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units)" : @""];
    [self.canvasView setNeedsDisplay:YES];
}

- (void)buildMesh:(id)sender {
    if (!self.documentModel.hasBoundary) { self.statusLabel.stringValue = @"Trace a boundary and pick its 4 corners first."; return; }
    NSInteger rows = MAX(3, self.rowsField.integerValue), cols = MAX(3, self.colsField.integerValue);
    [self.documentModel buildInitialMeshRows:rows cols:cols];
    self.canvasView.toolMode = GMToolModeEditMesh;
    [self.toolSegmented setSelected:YES forSegment:5];
    self.statusLabel.stringValue = [NSString stringWithFormat:@"Initial %ldx%ld mesh built. RMSE=%.4f  MAE=%.4f%@. Optionally draw vector lines, then click Optimize.",
                                     (long)rows, (long)cols, self.documentModel.currentRMSE, self.documentModel.currentMAE,
                                     self.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units)" : @""];
    [self.canvasView setNeedsDisplay:YES];
}

- (void)optimize:(id)sender {
    if (!self.documentModel.hasMesh) { self.statusLabel.stringValue = @"Build a mesh first."; return; }
    // Geometry/colour energy weights -- read straight from the fields into
    // the model, same pattern rowsField/colsField already use for -buildMesh:/
    // -autoMesh:, so whatever's currently typed takes effect on THIS run.
    self.documentModel.smoothWeightGeom = self.smoothWeightGeomField.doubleValue;
    self.documentModel.geomDataWeight = self.geomDataWeightField.doubleValue;
    self.documentModel.smoothGeomEdgeGain = self.smoothGeomEdgeGainField.doubleValue;
    self.documentModel.boundaryWeight = self.boundaryWeightField.doubleValue;
    self.documentModel.geomTangentPriorWeight = self.geomTangentPriorWeightField.doubleValue;
    self.documentModel.vectorLineWeight = self.vectorLineWeightField.doubleValue;
    self.documentModel.smoothWeightColor = self.smoothWeightColorField.doubleValue;
    self.documentModel.colorDerivRidge = self.colorDerivRidgeField.doubleValue;
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
            // Live mesh preview: when DocumentModel.livePreviewDuringOptimize
            // is on, a fresh snapshot lands in -hasLivePreviewMesh right before
            // this block runs (see -optimizeWithPyramidLevels:progress:completion:),
            // so redraw now to actually show it. A no-op cost when the toggle
            // is off, since CanvasView's guard then still gates the mesh out.
            [weakSelf.canvasView setNeedsDisplay:YES];
        }
        completion:^{
            [weakSelf.progressSpinner stopAnimation:nil];
            weakSelf.optimizeButton.enabled = YES;
            weakSelf.buildMeshButton.enabled = YES;
            NSString* unitTag = weakSelf.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units -- not comparable to sRGB-mode RMSE)" : @"";
            NSString* msg = [NSString stringWithFormat:@"Done in %.2fs. Final RMSE=%.4f  MAE=%.4f%@.",
                              weakSelf.documentModel.lastRunWallClockSeconds, weakSelf.documentModel.currentRMSE, weakSelf.documentModel.currentMAE, unitTag];
            // -lastDebugExportPath is only non-nil right after a run that
            // both had autoExportDebugData ON and wrote successfully -- see
            // DocumentModel -autoExportDebugDataIfEnabled.
            if (weakSelf.documentModel.lastDebugExportPath) {
                msg = [msg stringByAppendingFormat:@" Debug log: %@", weakSelf.documentModel.lastDebugExportPath];
            }
            weakSelf.statusLabel.stringValue = msg;
            [weakSelf refreshActivePreview];
        }];
}

- (void)togglePreview:(id)sender {
    [self refreshActivePreview];
}

- (void)toggleGPUPreview:(id)sender {
    [self refreshActivePreview];
}

// Shared refresh for BOTH reconstruction-preview backends -- canvasView
// (CPU, gmcore::GradientMesh::render()) and glReconstructionView (GPU,
// OpenGL/GLSL -- see GLReconstructionView.h). previewCheckbox gates
// whether ANY reconstruction preview shows at all; gpuPreviewCheckbox
// picks which of the two renders it when it does. Exactly one of the two
// views is ever visible; the other is hidden (and, for the GPU view,
// explicitly told it has nothing current to draw via -clearMesh) so
// switching back to it later never shows a stale render from before the
// last mesh change.
//
// v1 limitation, deliberate for now: unlike the CPU path, the GPU preview
// does not also draw the mesh-control-point/boundary/vector-line overlays
// canvasView draws on top of its own preview -- it is a plain side-by-side
// visual comparison of the two renderers, not (yet) an interactive editing
// surface. canvasView itself is hidden while the GPU preview is showing,
// so none of the click/drag tool interactions are reachable in that mode
// either.
- (void)refreshActivePreview {
    BOOL showPreview = (self.previewCheckbox.state == NSControlStateValueOn);
    BOOL useGPU = showPreview && (self.gpuPreviewCheckbox.state == NSControlStateValueOn);

    self.canvasView.showReconstructionPreview = showPreview && !useGPU;
    self.canvasView.hidden = useGPU;
    self.glReconstructionView.hidden = !useGPU;

    if (useGPU) {
        // 8 == the same samplesPerPatchEdge -renderReconstructionPreview
        // hardcodes for the CPU path (see DocumentModel.mm), so the two
        // previews are tessellated at a visually comparable density.
        GMGPUMeshBuffers* buffers = [self.documentModel gpuMeshBuffersWithSamplesPerPatchEdge:8];
        if (buffers) {
            [self.glReconstructionView uploadMeshBuffers:buffers];
        } else {
            [self.glReconstructionView clearMesh];
        }
    } else if (showPreview) {
        [self.canvasView refreshReconstructionPreview];
    }
    [self.canvasView setNeedsDisplay:YES];
}

- (void)toggleShowMesh:(id)sender {
    self.canvasView.showMeshOverlay = (self.meshCheckbox.state == NSControlStateValueOn);
    [self.canvasView setNeedsDisplay:YES];
}

- (void)toggleTangents:(id)sender {
    self.canvasView.showTangents = (self.tangentsCheckbox.state == NSControlStateValueOn);
    [self.canvasView setNeedsDisplay:YES];
}

- (void)toggleLivePreview:(id)sender {
    self.documentModel.livePreviewDuringOptimize = (self.livePreviewCheckbox.state == NSControlStateValueOn);
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

- (void)toggleCeresMultithreaded:(id)sender {
    self.documentModel.ceresMultithreaded = (self.ceresMultithreadedCheckbox.state == NSControlStateValueOn);
    self.statusLabel.stringValue = [NSString stringWithFormat:@"Ceres solves: %@. (Next “Optimize” run will use it; no effect on the hand-rolled solver.)",
                                     self.documentModel.ceresMultithreaded ? @"multithreaded (every core)" : @"single-threaded"];
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

// Saves what GLReconstructionView (GPU/OpenGL) actually draws, as opposed
// to -exportPNG: above (which always goes through DocumentModel's CPU
// rasterization -- see DocumentModel.exportPNGToURL:/renderReconstructionPreview
// -- regardless of which preview mode is currently shown on screen). Freshly
// builds and uploads GPU mesh buffers here rather than relying on
// glReconstructionView already having them, so this works even if the
// on-screen "GPU (OpenGL)" checkbox was never turned on.
- (void)exportGPUPNG:(id)sender {
    if (!self.documentModel.hasMesh) { self.statusLabel.stringValue = @"Build (and ideally optimize) a mesh first."; return; }

    // 8 == the same samplesPerPatchEdge used everywhere else in this app
    // for the GPU preview (see -refreshActivePreview) and the CPU preview
    // (DocumentModel.mm's -renderReconstructionPreview).
    GMGPUMeshBuffers* buffers = [self.documentModel gpuMeshBuffersWithSamplesPerPatchEdge:8];
    if (!buffers) {
        self.statusLabel.stringValue = @"Could not build GPU mesh buffers.";
        return;
    }
    [self.glReconstructionView uploadMeshBuffers:buffers];

    NSSavePanel* panel = [NSSavePanel savePanel];
    panel.nameFieldStringValue = @"gradient-mesh-gpu-reconstruction.png";
    __weak typeof(self) weakSelf = self;
    [panel beginSheetModalForWindow:self.window completionHandler:^(NSModalResponse result) {
        if (result != NSModalResponseOK) return;
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;

        // Export at the ORIGINAL loaded image's resolution -- matches
        // DocumentModel.exportPNGToURL:'s own (CPU) convention, so the two
        // export paths produce comparably-sized files regardless of
        // whatever size the on-screen view happens to be.
        NSInteger w = strongSelf.documentModel.imageWidth;
        NSInteger h = strongSelf.documentModel.imageHeight;
        NSImage* image = [strongSelf.glReconstructionView renderToImageWithWidth:w height:h];
        if (!image) {
            [strongSelf presentError:[NSError errorWithDomain:@"GradientMeshStudio" code:1
                                                       userInfo:@{NSLocalizedDescriptionKey: @"GPU render failed (no mesh uploaded, or invalid image size)."}]];
            return;
        }

        // Same CGImageForProposedRect: + CGImageDestination pattern as
        // DocumentModel.mm's -exportPNGToURL: -- renderToImageWithWidth:height:
        // already built `image` from an explicitly kCGColorSpaceSRGB-tagged
        // CGImage (see GLReconstructionView.mm), so this just writes those
        // exact bytes out; no extra color conversion happens here.
        CGImageRef cgImage = [image CGImageForProposedRect:NULL context:nil hints:nil];
        if (!cgImage) {
            [strongSelf presentError:[NSError errorWithDomain:@"GradientMeshStudio" code:1
                                                       userInfo:@{NSLocalizedDescriptionKey: @"Could not rasterize GPU render."}]];
            return;
        }
        CGImageDestinationRef dest = CGImageDestinationCreateWithURL((__bridge CFURLRef)panel.URL, CFSTR("public.png"), 1, NULL);
        if (!dest) {
            [strongSelf presentError:[NSError errorWithDomain:@"GradientMeshStudio" code:1
                                                       userInfo:@{NSLocalizedDescriptionKey: @"Could not create PNG file."}]];
            return;
        }
        CGImageDestinationAddImage(dest, cgImage, NULL);
        BOOL ok = CGImageDestinationFinalize(dest);
        CFRelease(dest);
        if (!ok) {
            [strongSelf presentError:[NSError errorWithDomain:@"GradientMeshStudio" code:1
                                                       userInfo:@{NSLocalizedDescriptionKey: @"Could not write PNG file."}]];
        } else {
            strongSelf.statusLabel.stringValue = [NSString stringWithFormat:@"Exported GPU-rendered PNG to %@", panel.URL.path];
        }
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

- (void)resetWeights:(id)sender {
    [self.documentModel resetWeightsToDefaults];
    self.smoothWeightGeomField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.smoothWeightGeom];
    self.geomDataWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.geomDataWeight];
    self.smoothGeomEdgeGainField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.smoothGeomEdgeGain];
    self.boundaryWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.boundaryWeight];
    self.geomTangentPriorWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.geomTangentPriorWeight];
    self.vectorLineWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.vectorLineWeight];
    self.smoothWeightColorField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.smoothWeightColor];
    self.colorDerivRidgeField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.colorDerivRidge];
    self.statusLabel.stringValue = @"Geometry/color weights reset to defaults. Takes effect on the next “Optimize” click.";
}

- (void)savePreset:(id)sender {
    if (!self.documentModel.hasImage) { self.statusLabel.stringValue = @"Open an image first."; return; }
    NSURL* dir = [self.documentModel presetsDirectoryURL];
    if (!dir) { self.statusLabel.stringValue = @"Could not create/find a Presets folder next to the image."; return; }
    NSURL* url = [dir URLByAppendingPathComponent:[self.documentModel presetExportFilename]];
    NSError* error = nil;
    if (![self.documentModel savePresetToURL:url error:&error]) {
        [self presentError:error];
        return;
    }
    self.statusLabel.stringValue = [NSString stringWithFormat:@"Saved preset: %@", url.lastPathComponent];
}

// NSMenuDelegate: rebuilds self.presetsPopup's item list from
// -availablePresetNames right before the popup opens, so a preset saved a
// moment ago (this session, or by hand outside it) always shows up without
// a relaunch. Keeps item 0 ("Load Preset…" -- a pull-down's always-shown
// label, never itself a loadable preset) and replaces everything after it.
- (void)menuNeedsUpdate:(NSMenu*)menu {
    if (menu != self.presetsPopup.menu) return;
    while (menu.numberOfItems > 1) [menu removeItemAtIndex:1];
    for (NSString* name in [self.documentModel availablePresetNames]) {
        [menu addItemWithTitle:name action:nil keyEquivalent:@""];
    }
}

// self.presetsPopup is a PULL-DOWN (pullsDown:YES, see -buildUI): item 0's
// title is always what's displayed, and clicking ANY item (including item
// 0 itself) fires this action with -indexOfSelectedItem telling us which
// one -- the standard AppKit idiom for a pull-down "menu of actions" button
// (same reason self.solverPopup -- a NORMAL, pullsDown:NO popup that DOES
// persist the picked item as its displayed title -- doesn't need this
// index<=0 guard in -solverChanged:).
- (void)presetSelected:(id)sender {
    NSInteger idx = self.presetsPopup.indexOfSelectedItem;
    if (idx <= 0) return; // the "Load Preset…" label itself, not a real preset
    NSString* name = [self.presetsPopup itemTitleAtIndex:idx];
    NSInteger rows = 0, cols = 0;
    NSError* error = nil;
    if (![self.documentModel loadPresetNamed:name rows:&rows cols:&cols error:&error]) {
        [self presentError:error];
        return;
    }
    if (rows > 0) self.rowsField.integerValue = rows;
    if (cols > 0) self.colsField.integerValue = cols;
    // Mirror every field/control this preset touched -- same pattern
    // -resetWeights: already uses for the eight weight fields.
    self.smoothWeightGeomField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.smoothWeightGeom];
    self.geomDataWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.geomDataWeight];
    self.smoothGeomEdgeGainField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.smoothGeomEdgeGain];
    self.boundaryWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.boundaryWeight];
    self.geomTangentPriorWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.geomTangentPriorWeight];
    self.vectorLineWeightField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.vectorLineWeight];
    self.smoothWeightColorField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.smoothWeightColor];
    self.colorDerivRidgeField.stringValue = [NSString stringWithFormat:@"%g", self.documentModel.colorDerivRidge];
    NSInteger solverIdx = self.documentModel.useCeresJoint ? 2 : self.documentModel.useCeresGeometry ? 1 : 0;
    [self.solverPopup selectItemAtIndex:solverIdx];
    self.ceresMultithreadedCheckbox.state = self.documentModel.ceresMultithreaded ? NSControlStateValueOn : NSControlStateValueOff;
    self.cieluvCheckbox.state = self.documentModel.useCIELUVColorSpace ? NSControlStateValueOn : NSControlStateValueOff;
    self.statusLabel.stringValue = [NSString stringWithFormat:
        @"Loaded preset “%@”. Takes effect on the next “Optimize” (and, for CIELUV, the next “Build Initial Mesh”).", name];
}

- (void)presentError:(NSError*)error {
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = @"Gradient Mesh Studio";
    alert.informativeText = error.localizedDescription ?: @"An unknown error occurred.";
    [alert beginSheetModalForWindow:self.window completionHandler:nil];
}

@end
