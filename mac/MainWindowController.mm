#import "MainWindowController.h"
#import "CanvasView.h"
#import "DocumentModel.h"
#import "GLReconstructionView.h"
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>
#import <CoreGraphics/CoreGraphics.h>  // CGImageDestinationRef etc. for -exportGPUPNG: (mirrors DocumentModel.mm's -exportPNGToURL:)
#import <ImageIO/ImageIO.h>            // CGImageDestinationCreateWithURL/AddImage/Finalize

@interface MainWindowController () <NSWindowDelegate, NSMenuDelegate, NSToolbarDelegate>
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
// "Animate Mesh" -- see DocumentModel.h's isAnimatingMesh/
// startMeshAnimationWithRedraw:/stopMeshAnimation and -toggleAnimateMesh:.
// animateMeshButton's title toggles between "Animate Mesh"/"Stop
// Animation"; animStylePopup mirrors DocumentModel.meshAnimationStyle
// (see GMMeshAnimationStyle in DocumentModel.h -- item order matches the
// enum's raw values exactly, 0=Jitter/1=Wave/2=Breathing/
// 3=SquashStretch, so -indexOfSelectedItem casts straight to the enum
// with no separate mapping table needed); the four text fields mirror
// DocumentModel.meshAnimationMaxAmplitude/meshAnimationTemperature/
// meshAnimationWaveDirectionDegrees/meshAnimationMinClearanceDistance,
// read (and clamped) into the model each time animation is (re)started,
// same "typed value takes effect on next start" convention as the
// geometry/colour weight fields above. Temperature only matters for
// Jitter and Direction only matters for Wave, but both fields stay
// visible regardless of the selected style (same "harmless if unused
// otherwise" treatment meshAnimationMinClearanceDistance's field
// already gets) -- simpler than wiring show/hide logic to the popup for
// two rarely-confusing, always-labeled fields.
@property (nonatomic, strong) NSButton* animateMeshButton;
@property (nonatomic, strong) NSPopUpButton* animStylePopup;
@property (nonatomic, strong) NSTextField* animAmplitudeField;
@property (nonatomic, strong) NSTextField* animTemperatureField;
@property (nonatomic, strong) NSTextField* animDirectionField;
@property (nonatomic, strong) NSTextField* animClearanceField;
// Mirrors DocumentModel.meshAnimationDebugDisableContinuousStages -- see
// that property's doc comment in DocumentModel.h. Read (like every other
// field on the Animate Mesh row) at "Animate Mesh" press time
// (-toggleAnimateMesh:), not live -- ticking it mid-animation has no
// effect until the animation is (re)started, same convention as every
// other control on this row.
@property (nonatomic, strong) NSButton* animDisableContinuousCheckbox;
// Bodies of the inspector's collapsible sections; a disclosure button's tag
// is its index here (see -sectionTitled:... and -toggleSection:).
@property (nonatomic, strong) NSMutableArray<NSView*>* sectionBodies;
// Toolbar item identifier -> the control shown in that item (see
// -toolbar:itemForItemIdentifier:willBeInsertedIntoToolbar: and -buildToolbar).
@property (nonatomic, strong) NSDictionary<NSString*, NSView*>* toolbarViews;
@end

// Sets the same tooltip on every given view (a control and its label, so
// hovering either one explains it). Tooltips show with zero delay -- see
// main.mm, which sets NSInitialToolTipDelay before the app starts.
static void GMTip(NSString* text, NSArray<NSView*>* views) {
    for (NSView* v in views) v.toolTip = text;
}

@implementation MainWindowController

- (instancetype)init {
    NSRect frame = NSMakeRect(0, 0, 1400, 820);
    NSWindow* window = [[NSWindow alloc] initWithContentRect:frame
                                                     styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                                                                NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable)
                                                       backing:NSBackingStoreBuffered
                                                         defer:NO];
    window.title = @"Gradient Mesh Studio";
    // The toolbar needs the width the title would take.
    window.titleVisibility = NSWindowTitleHidden;
    window.minSize = NSMakeSize(900, 560);
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

    NSView* controlsRow1b = [self makeRow];

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
        [self.toolSegmented setWidth:0 forSegment:i];  // 0 = fit the label
    }
    self.toolSegmented.target = self;
    self.toolSegmented.action = @selector(toolChanged:);

    NSButton* clearLineBtn = [self buttonTitled:@"Clear Last Line" action:@selector(clearLastLine:)];

    GMTip(@"Load an image to vectorize.", @[openBtn]);
    GMTip(@"Fit a mesh to the whole image without any boundary or corner markup.", @[autoBtn]);
    GMTip(@"Active tool. 1: trace the region boundary, or scribble foreground/background and press Segment; "
          "2: pick the 4 mesh corners; 3: draw vector lines the mesh must follow; 4: edit the mesh.", @[self.toolSegmented]);
    GMTip(@"Remove the most recently drawn vector line.", @[clearLineBtn]);
    // Row 1 became the window toolbar -- see -buildToolbar (called below, once Build/Optimize/Animate exist).

    // --- Row 1b: Lazy-Snapping-style segmentation (scribble tools above) ---
    NSButton* segmentBtn = [self buttonTitled:@"Segment" action:@selector(segmentBoundary:)];
    NSButton* clearScribblesBtn = [self buttonTitled:@"Clear Scribbles" action:@selector(clearScribbles:)];
    NSTextField* scribbleHintLabel = [self makeLabel:@"(scribble foreground/background above, then Segment)"];
    GMTip(@"Cut out the region marked by the foreground/background scribbles (Lazy Snapping).", @[segmentBtn]);
    GMTip(@"Discard all foreground and background scribbles.", @[clearScribblesBtn]);
    for (NSView* v in @[segmentBtn, clearScribblesBtn, clearLineBtn, scribbleHintLabel]) [controlsRow1b addSubview:v];

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

    GMTip(@"Number of mesh rows. Takes effect on the next Build Initial Mesh.", @[rowsLabel, self.rowsField]);
    GMTip(@"Number of mesh columns. Takes effect on the next Build Initial Mesh.", @[colsLabel, self.colsField]);
    GMTip(@"Build the initial mesh from the corners and boundary, before optimization.", @[self.buildMeshButton]);
    GMTip(@"Fit the mesh to the image (optimizer, Sec. 4 of the paper) using the solver and weights below.", @[self.optimizeButton]);
    GMTip(@"Draw the image reconstructed from the mesh instead of the original.", @[self.previewCheckbox]);
    GMTip(@"Render the reconstruction on the GPU with OpenGL instead of the CPU. Only used while Show reconstruction is on.", @[self.gpuPreviewCheckbox]);
    GMTip(@"Overlay the mesh grid on the canvas.", @[self.meshCheckbox]);
    GMTip(@"Overlay the tangent handles of the mesh vertices.", @[self.tangentsCheckbox]);
    GMTip(@"Redraw the in-progress mesh while Optimize runs. Costs one small mesh copy and a redraw per outer iteration.", @[self.livePreviewCheckbox]);
    GMTip(@"Export the reconstruction rendered on the CPU as a PNG.", @[self.exportPNGButton]);
    GMTip(@"Export the mesh as an SVG gradient mesh.", @[self.exportSVGButton]);
    GMTip(@"Export the reconstruction rendered on the GPU as a PNG.", @[self.exportGPUPNGButton]);

    // --- Row 3: solver picker (hand-rolled vs Ceres geometry-only vs Ceres joint) ---
    NSTextField* solverLabel = [self makeLabel:@"Solver:"];
    self.solverPopup = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    self.solverPopup.translatesAutoresizingMaskIntoConstraints = NO;
    [self.solverPopup addItemsWithTitles:@[@"Hand-rolled", @"Ceres (geometry)", @"Ceres (joint)"]];
    [self.solverPopup selectItemAtIndex:0];
    self.solverPopup.target = self;
    self.solverPopup.action = @selector(solverChanged:);

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

    GMTip(@"Optimizer: Hand-rolled (built in), Ceres (geometry) or Ceres (joint). The Ceres modes need a build with Ceres found; otherwise they do nothing and print a console warning.", @[solverLabel, self.solverPopup]);
    GMTip(@"After each Optimize run, write a timestamped JSON with the run's debug data into a DebugOut folder next to the loaded image.", @[self.autoDebugCheckbox]);
    GMTip(@"Build and fit the mesh in CIELUV instead of sRGB. Takes effect on the next Build Initial Mesh.", @[self.cieluvCheckbox]);
    GMTip(@"Let the Ceres solvers use every CPU core. No effect on the hand-rolled solver.", @[self.ceresMultithreadedCheckbox]);

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

    GMTip(@"Weights of the geometry optimization step. A typed value takes effect on the next Optimize.", @[geomWeightsLabel]);
    GMTip(@"Smoothness of the mesh geometry. Higher gives a smoother, less image-following mesh.", @[smoothGeomLabel, self.smoothWeightGeomField]);
    GMTip(@"Multiplier of the geometry step's photometric (image-fit) term against the other weights. 1 = default.", @[geomDataLabel, self.geomDataWeightField]);
    GMTip(@"Makes Smooth anisotropic: relaxes it near strong image edges so the mesh snaps to them tighter. 0 = isotropic; 40 = default.", @[edgeGainLabel, self.smoothGeomEdgeGainField]);
    GMTip(@"Weight keeping boundary vertices on their boundary curve.", @[boundaryLabel, self.boundaryWeightField]);
    GMTip(@"Weight of the prior on the tangent handles (Pu, Pv) in the geometry step.", @[tangentPriorLabel, self.geomTangentPriorWeightField]);
    GMTip(@"Weight pulling the mesh edges onto the vector lines drawn with tool 3.", @[vectorLineLabel, self.vectorLineWeightField]);

    // --- Row 5: colour energy weights, + a shared reset for all eight ---
    NSTextField* colorWeightsLabel = [self makeLabel:@"Color weights —"];
    NSTextField* smoothColorLabel = [self makeLabel:@"Smooth:"];
    self.smoothWeightColorField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.smoothWeightColor]];
    NSTextField* colorRidgeLabel = [self makeLabel:@"Ridge:"];
    self.colorDerivRidgeField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.colorDerivRidge]];
    NSButton* resetWeightsBtn = [self buttonTitled:@"Reset weights to defaults" action:@selector(resetWeights:)];

    GMTip(@"Weights of the color optimization step. A typed value takes effect on the next Optimize.", @[colorWeightsLabel]);
    GMTip(@"Smoothness of the vertex colors across the mesh.", @[smoothColorLabel, self.smoothWeightColorField]);
    GMTip(@"Ridge regularization on the color derivatives (Cu, Cv, Cuv).", @[colorRidgeLabel, self.colorDerivRidgeField]);
    GMTip(@"Restore all eight geometry and color weights to the compiled-in defaults.", @[resetWeightsBtn]);

    // --- Row 6: "Animate Mesh" -- a purely cosmetic, non-destructive
    // real-time wiggle of the current mesh's vertex positions, entirely
    // separate from "Optimize" (see DocumentModel.h's isAnimatingMesh
    // comment for the full design). Amplitude/temperature fields seeded
    // from documentModel's own -init defaults, same convention as Row 4/5's
    // weight fields.
    self.animateMeshButton = [self buttonTitled:@"Animate Mesh" action:@selector(toggleAnimateMesh:)];
    NSTextField* animAmplitudeLabel = [self makeLabel:@"Amplitude:"];
    self.animAmplitudeField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.meshAnimationMaxAmplitude]];
    NSTextField* animTemperatureLabel = [self makeLabel:@"Temperature:"];
    self.animTemperatureField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.meshAnimationTemperature]];
    // Minimum required separation (px) between two DIFFERENT mesh edges'
    // curves -- see meshAnimationMinClearanceDistance's doc comment in
    // DocumentModel.h. Same seed-from-model-default / read-and-clamp-on-
    // (re)start convention as the two fields above.
    NSTextField* animClearanceLabel = [self makeLabel:@"Min clearance:"];
    self.animClearanceField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.meshAnimationMinClearanceDistance]];

    GMTip(@"Start or stop a cosmetic wiggle of the fitted mesh. The mesh itself is not modified. The fields on this row and the next are read when you start it.", @[self.animateMeshButton]);
    GMTip(@"Maximum vertex displacement, in image pixels. 0 keeps every vertex in place.", @[animAmplitudeLabel, self.animAmplitudeField]);
    GMTip(@"Jitter only. Low = calm (few vertices move much); high = chaotic (nearly all swing close to full amplitude). Must be > 0.", @[animTemperatureLabel, self.animTemperatureField]);
    GMTip(@"Minimum distance, in pixels, kept between two different mesh edges. 0 checks only for literal crossings.", @[animClearanceLabel, self.animClearanceField]);

    // --- Row 7: animation STYLE (Jitter/Wave/Breathing/Squash & Stretch)
    // -- see GMMeshAnimationStyle in DocumentModel.h. A plain (pullsDown:
    // NO) popup, same pattern as self.solverPopup above: no target/action
    // wired up, since -- like every other field on this row -- it's only
    // ever read at "Animate Mesh" press time (-toggleAnimateMesh:), not
    // live. Item order matches the enum's raw values exactly.
    NSTextField* animStyleLabel = [self makeLabel:@"Style:"];
    self.animStylePopup = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    self.animStylePopup.translatesAutoresizingMaskIntoConstraints = NO;
    [self.animStylePopup addItemsWithTitles:@[@"Jitter", @"Wave", @"Breathing", @"Squash & Stretch"]];
    [self.animStylePopup selectItemAtIndex:(NSInteger)self.documentModel.meshAnimationStyle];
    // Direction only matters for Wave -- see
    // meshAnimationWaveDirectionDegrees's doc comment in DocumentModel.h
    // for the 0/90-degree convention and why there's no separate
    // wavelength control.
    NSTextField* animDirectionLabel = [self makeLabel:@"Direction, °:"];
    self.animDirectionField = [self makeWeightFieldWithValue:
        [NSString stringWithFormat:@"%g", self.documentModel.meshAnimationWaveDirectionDegrees]];

    GMTip(@"Animation style: Jitter, Wave, Breathing or Squash & Stretch.", @[animStyleLabel, self.animStylePopup]);
    GMTip(@"Wave only. Direction the ripple travels, in degrees: 0 = left to right, 90 = +Y.", @[animDirectionLabel, self.animDirectionField]);

    // --- Row 7b: debug-only "disable continuous stages" checkbox -- see
    // DocumentModel.h's meshAnimationDebugDisableContinuousStages doc
    // comment for what it does and why it exists (isolating a disputed
    // residual-unevenness report). No target/action wired up, same as
    // animStylePopup above -- only ever read at "Animate Mesh" press time
    // (-toggleAnimateMesh:), not live. Off by default unless
    // GM_ANIM_DISABLE_CONTINUOUS was set in the environment at launch --
    // see DocumentModel -init.
    self.animDisableContinuousCheckbox = [NSButton checkboxWithTitle:@"Disable damping/smoothing"
                                                                target:nil action:nil];
    self.animDisableContinuousCheckbox.translatesAutoresizingMaskIntoConstraints = NO;
    self.animDisableContinuousCheckbox.state = self.documentModel.meshAnimationDebugDisableContinuousStages
        ? NSControlStateValueOn : NSControlStateValueOff;

    GMTip(@"Debug. Isolates whether unevenness comes from the plain self-intersection guard or from damping/smoothing. Skips the continuous damping and the temporal smoothing, leaving only the exact self-intersection check.", @[self.animDisableContinuousCheckbox]);

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

    GMTip(@"Save the settings of the last completed Optimize run to a timestamped JSON in a Presets folder next to the loaded image.", @[self.savePresetButton]);
    GMTip(@"Load a preset saved next to the current image, newest first.", @[self.presetsPopup]);
    // --- Inspector (right-hand panel): collapsible sections, in the order
    // the work goes -- mesh, optimize, view, animate, presets, export.
    // Replaces the old stack of seven control rows and the preset sidebar.
    // Hint labels that used to sit in those rows now live in the widgets'
    // tooltips (see GMTip above).
    self.sectionBodies = [NSMutableArray array];

    NSTextField* geomHeading = [self makeLabel:@"Geometry"];
    geomHeading.font = [NSFont boldSystemFontOfSize:11];
    NSTextField* colorHeading = [self makeLabel:@"Color"];
    colorHeading.font = [NSFont boldSystemFontOfSize:11];
    NSGridView* geomGrid = [self gridWithLabels:@[smoothGeomLabel, geomDataLabel, edgeGainLabel, boundaryLabel, tangentPriorLabel, vectorLineLabel]
                                         fields:@[self.smoothWeightGeomField, self.geomDataWeightField, self.smoothGeomEdgeGainField,
                                                  self.boundaryWeightField, self.geomTangentPriorWeightField, self.vectorLineWeightField]];
    NSGridView* colorGrid = [self gridWithLabels:@[smoothColorLabel, colorRidgeLabel]
                                          fields:@[self.smoothWeightColorField, self.colorDerivRidgeField]];
    NSView* weightsSection = [self sectionTitled:@"Weights" expanded:NO nested:YES
                                         content:@[geomHeading, geomGrid, colorHeading, colorGrid, resetWeightsBtn]];

    NSView* meshSection = [self sectionTitled:@"Mesh" expanded:YES nested:NO content:@[
        [self hStack:@[rowsLabel, self.rowsField, colsLabel, self.colsField]]]];
    NSView* optimizeSection = [self sectionTitled:@"Optimize" expanded:YES nested:NO content:@[
        [self hStack:@[solverLabel, self.solverPopup]],
        self.autoDebugCheckbox, self.cieluvCheckbox, self.ceresMultithreadedCheckbox, weightsSection]];
    NSView* viewSection = [self sectionTitled:@"View" expanded:YES nested:NO content:@[
        self.previewCheckbox, self.gpuPreviewCheckbox, self.meshCheckbox, self.tangentsCheckbox, self.livePreviewCheckbox]];
    NSGridView* animGrid = [self gridWithLabels:@[animStyleLabel, animAmplitudeLabel, animTemperatureLabel, animClearanceLabel, animDirectionLabel]
                                         fields:@[self.animStylePopup, self.animAmplitudeField, self.animTemperatureField,
                                                  self.animClearanceField, self.animDirectionField]];
    NSView* animAdvanced = [self sectionTitled:@"Advanced" expanded:NO nested:YES content:@[self.animDisableContinuousCheckbox]];
    NSView* animateSection = [self sectionTitled:@"Animate" expanded:YES nested:NO content:@[
        animGrid, animAdvanced]];
    NSView* presetsSection = [self sectionTitled:@"Presets" expanded:YES nested:NO content:@[
        self.savePresetButton, self.presetsPopup]];
    NSView* exportSection = [self sectionTitled:@"Export" expanded:YES nested:NO content:@[
        self.exportPNGButton, self.exportSVGButton, self.exportGPUPNGButton]];

    NSStackView* inspector = [NSStackView stackViewWithViews:@[meshSection, optimizeSection, viewSection,
                                                               animateSection, presetsSection, exportSection]];
    inspector.orientation = NSUserInterfaceLayoutOrientationVertical;
    inspector.alignment = NSLayoutAttributeLeading;
    inspector.spacing = 14;
    inspector.edgeInsets = NSEdgeInsetsMake(10, 10, 10, 10);
    inspector.translatesAutoresizingMaskIntoConstraints = NO;
    for (NSView* section in inspector.arrangedSubviews)
        [section.widthAnchor constraintEqualToAnchor:inspector.widthAnchor constant:-20].active = YES;

    NSScrollView* inspectorScroll = [[NSScrollView alloc] initWithFrame:NSZeroRect];
    inspectorScroll.translatesAutoresizingMaskIntoConstraints = NO;
    inspectorScroll.hasVerticalScroller = YES;
    inspectorScroll.drawsBackground = NO;
    inspectorScroll.documentView = inspector;
    // Pin the document view to the clip view's top/sides so a short
    // inspector sits at the top instead of the bottom (stack views are not
    // flipped); its height stays intrinsic and scrolls when taller.
    [inspector.topAnchor constraintEqualToAnchor:inspectorScroll.contentView.topAnchor].active = YES;
    [inspector.leadingAnchor constraintEqualToAnchor:inspectorScroll.contentView.leadingAnchor].active = YES;
    [inspector.trailingAnchor constraintEqualToAnchor:inspectorScroll.contentView.trailingAnchor].active = YES;

    // canvasRow: canvasView + inspector side by side, replacing
    // canvasView's old direct placement in the outer vertical stack below
    // -- everything about canvasView itself (documentModel, callbacks,
    // CanvasView's own drawing) is unchanged, it just now shares its row
    // with the sidebar instead of spanning the full window width.
    NSView* canvasRow = [[NSView alloc] initWithFrame:NSZeroRect];
    canvasRow.translatesAutoresizingMaskIntoConstraints = NO;
    [canvasRow addSubview:self.canvasView];
    [canvasRow addSubview:self.glReconstructionView];
    [canvasRow addSubview:inspectorScroll];
    // glReconstructionView exactly overlays canvasView (not part of the
    // visual-format layout below, which only positions _canvasView/
    // presetSidebar within canvasRow) -- see -refreshActivePreview for how
    // the two are switched between.
    [self.glReconstructionView.leadingAnchor constraintEqualToAnchor:self.canvasView.leadingAnchor].active = YES;
    [self.glReconstructionView.trailingAnchor constraintEqualToAnchor:self.canvasView.trailingAnchor].active = YES;
    [self.glReconstructionView.topAnchor constraintEqualToAnchor:self.canvasView.topAnchor].active = YES;
    [self.glReconstructionView.bottomAnchor constraintEqualToAnchor:self.canvasView.bottomAnchor].active = YES;
    NSDictionary* canvasRowViews = NSDictionaryOfVariableBindings(_canvasView, inspectorScroll);
    [canvasRow addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-0-[_canvasView]-8-[inspectorScroll(320)]-0-|"
                                                                       options:0 metrics:nil views:canvasRowViews]];
    [self.canvasView.topAnchor constraintEqualToAnchor:canvasRow.topAnchor].active = YES;
    [self.canvasView.bottomAnchor constraintEqualToAnchor:canvasRow.bottomAnchor].active = YES;
    [inspectorScroll.topAnchor constraintEqualToAnchor:canvasRow.topAnchor].active = YES;
    [inspectorScroll.bottomAnchor constraintEqualToAnchor:canvasRow.bottomAnchor].active = YES;

    [content addSubview:controlsRow1b];
    [content addSubview:canvasRow];
    [content addSubview:self.statusLabel];

    NSDictionary* views = NSDictionaryOfVariableBindings(controlsRow1b, canvasRow, _statusLabel);
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[controlsRow1b]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-0-[canvasRow]-0-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:@"H:|-8-[_statusLabel]-8-|" options:0 metrics:nil views:views]];
    [content addConstraints:[NSLayoutConstraint constraintsWithVisualFormat:
        @"V:|-8-[controlsRow1b(28)]-6-[canvasRow]-4-[_statusLabel(18)]-6-|"
                                                                    options:0 metrics:nil views:views]];

    [self layoutRowChildren:controlsRow1b];
    self.toolbarViews = @{
        @"open": openBtn, @"auto": autoBtn, @"tools": self.toolSegmented,
        @"build": self.buildMeshButton, @"optimize": self.optimizeButton, @"spinner": self.progressSpinner,
        @"animate": self.animateMeshButton,
    };
    [self buildToolbar];
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

#pragma mark - Toolbar

// File/tool/run controls live in a native toolbar instead of the old
// "row 1" and the Mesh/Optimize/Animate buttons of the inspector. Order
// follows the workflow: open, tool, then build / optimize / animate.
- (NSArray<NSString*>*)toolbarIdentifiers {
    return @[@"open", @"auto", @"tools", NSToolbarFlexibleSpaceItemIdentifier,
             @"build", @"optimize", @"spinner", @"animate"];
}

- (void)buildToolbar {
    NSToolbar* toolbar = [[NSToolbar alloc] initWithIdentifier:@"GMMainToolbar"];
    toolbar.delegate = self;
    toolbar.displayMode = NSToolbarDisplayModeIconOnly;
    toolbar.allowsUserCustomization = NO;
    self.window.toolbar = toolbar;
}

- (NSArray<NSToolbarItemIdentifier>*)toolbarDefaultItemIdentifiers:(NSToolbar*)toolbar { return [self toolbarIdentifiers]; }
- (NSArray<NSToolbarItemIdentifier>*)toolbarAllowedItemIdentifiers:(NSToolbar*)toolbar { return [self toolbarIdentifiers]; }

- (NSToolbarItem*)toolbar:(NSToolbar*)toolbar itemForItemIdentifier:(NSToolbarItemIdentifier)identifier
 willBeInsertedIntoToolbar:(BOOL)flag {
    NSView* view = self.toolbarViews[identifier];
    if (!view) return nil;
    NSToolbarItem* item = [[NSToolbarItem alloc] initWithItemIdentifier:identifier];
    item.view = view;
    NSDictionary* names = @{@"open": @"Open Image", @"auto": @"Auto Mesh", @"tools": @"Tool", @"build": @"Build Initial Mesh",
                            @"optimize": @"Optimize", @"spinner": @"Progress", @"animate": @"Animate Mesh"};
    item.label = names[identifier] ?: identifier;
    return item;
}

// Horizontal row of controls for the inspector.
- (NSStackView*)hStack:(NSArray<NSView*>*)views {
    NSStackView* st = [NSStackView stackViewWithViews:views];
    st.orientation = NSUserInterfaceLayoutOrientationHorizontal;
    st.alignment = NSLayoutAttributeCenterY;
    st.spacing = 8;
    st.translatesAutoresizingMaskIntoConstraints = NO;
    return st;
}

// Two-column "label | field" table; keeps the columns aligned.
- (NSGridView*)gridWithLabels:(NSArray<NSView*>*)labels fields:(NSArray<NSView*>*)fields {
    NSMutableArray* rows = [NSMutableArray array];
    for (NSUInteger i = 0; i < labels.count; ++i) [rows addObject:@[labels[i], fields[i]]];
    NSGridView* g = [NSGridView gridViewWithViews:rows];
    g.translatesAutoresizingMaskIntoConstraints = NO;
    g.columnSpacing = 8;
    g.rowSpacing = 6;
    g.rowAlignment = NSGridRowAlignmentFirstBaseline;
    return g;
}

// A collapsible inspector section: a disclosure triangle + title, and a
// body that is hidden when collapsed (hidden arranged subviews of a
// NSStackView drop out of the layout). `nested` uses a smaller title.
- (NSView*)sectionTitled:(NSString*)title expanded:(BOOL)expanded nested:(BOOL)nested content:(NSArray<NSView*>*)content {
    NSButton* disclosure = [[NSButton alloc] initWithFrame:NSZeroRect];
    disclosure.bezelStyle = NSBezelStyleDisclosure;
    disclosure.buttonType = NSButtonTypePushOnPushOff;
    disclosure.title = @"";
    disclosure.state = expanded ? NSControlStateValueOn : NSControlStateValueOff;
    disclosure.target = self;
    disclosure.action = @selector(toggleSection:);
    disclosure.translatesAutoresizingMaskIntoConstraints = NO;
    NSTextField* label = [self makeLabel:title];
    label.font = nested ? [NSFont boldSystemFontOfSize:11] : [NSFont boldSystemFontOfSize:13];
    NSStackView* header = [self hStack:@[disclosure, label]];
    header.spacing = 4;

    NSStackView* body = [NSStackView stackViewWithViews:content];
    body.orientation = NSUserInterfaceLayoutOrientationVertical;
    body.alignment = NSLayoutAttributeLeading;
    body.spacing = 8;
    body.edgeInsets = NSEdgeInsetsMake(0, 18, 0, 0);
    body.translatesAutoresizingMaskIntoConstraints = NO;
    body.hidden = !expanded;
    disclosure.tag = (NSInteger)self.sectionBodies.count;
    [self.sectionBodies addObject:body];

    NSStackView* section = [NSStackView stackViewWithViews:@[header, body]];
    section.orientation = NSUserInterfaceLayoutOrientationVertical;
    section.alignment = NSLayoutAttributeLeading;
    section.spacing = 8;
    section.translatesAutoresizingMaskIntoConstraints = NO;
    return section;
}

- (void)toggleSection:(NSButton*)sender {
    self.sectionBodies[(NSUInteger)sender.tag].hidden = (sender.state != NSControlStateValueOn);
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
        // A fresh boundary just replaced whatever was there before (if
        // anything) -- clear any corner picks and in-progress trace state
        // left over from a PREVIOUS boundary, so they don't linger stuck
        // against the new one (a previous set of 4 picked corners would
        // otherwise still show, at the wrong positions, and further clicks
        // would silently do nothing since the picker already thinks it has
        // its 4 points). DocumentModel's own stale mesh (if any) is
        // dropped inside -setBoundaryPolygonPoints: itself, which already
        // ran by the time this callback fires.
        [weakSelf.canvasView resetBoundaryDrawing];
        [weakSelf.canvasView resetCornerPicking];
        weakSelf.statusLabel.stringValue = @"Boundary traced. Switch to “Pick 4 Corners” and click the 4 corner points (in order).";
        [weakSelf.canvasView setNeedsDisplay:YES];
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
    // The deployment target is macOS 12 (CMakeLists.txt), so the pre-11
    // -allowedFileTypes: fallback (deprecated since 12) was dead code.
    panel.allowedContentTypes = @[UTTypeImage];
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
        // A new image drops the old mesh, which -loadImageAtURL:error:
        // already stops any running animation of (see DocumentModel.mm's
        // -stopMeshAnimation callers) -- reset this button's own title/
        // enabled state to match, since "Open Image…" isn't disabled while
        // animating the way Optimize/Build Mesh are (see -toggleAnimateMesh:).
        weakSelf.animateMeshButton.title = @"Animate Mesh";
        weakSelf.animateMeshButton.enabled = YES;
        weakSelf.optimizeButton.enabled = YES;
        weakSelf.buildMeshButton.enabled = YES;
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
    // Same reset as -wireCanvasCallbacks' onBoundaryChanged does for the
    // manual-trace path -- segmentBoundaryFromScribblesWithError: feeds
    // the exact same -setBoundaryPolygonPoints: entry point, so it needs
    // the exact same cleanup: drop CanvasView's leftover corner picks (and
    // any stale in-progress trace draft) from before, so re-running
    // Segment doesn't get stuck showing 4 old corner markers and refusing
    // new clicks. DocumentModel's stale mesh (if any) was already dropped
    // inside -setBoundaryPolygonPoints: itself.
    [self.canvasView resetBoundaryDrawing];
    [self.canvasView resetCornerPicking];
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
    self.animateMeshButton.enabled = NO; // DocumentModel also stops any running animation itself -- see -stopMeshAnimation's callers
    [self.progressSpinner startAnimation:nil];
    __weak typeof(self) weakSelf = self;
    [self.documentModel optimizeWithPyramidLevels:4
        progress:^(double rmse, NSInteger level, NSInteger totalLevels, NSInteger iter, NSInteger totalIters) {
            // See DocumentModel.h's useCIELUVColorSpace comment: a CIELUV
            // run's RMSE is in different units (L* roughly 0..100) than an
            // sRGB run's -- flagged here so it's never mistaken for a huge
            // regression/improvement at a glance.
            NSString* unitTag = weakSelf.documentModel.meshColorSpaceIsCIELUV ? @" (CIELUV units)" : @"";
            // totalLevels==1 means this run skipped the coarse pyramid
            // levels entirely -- see DocumentModel.mm's
            // _meshHasHadFullPyramidPass: only the FIRST "Optimize" click on
            // a given mesh does the full coarse-to-fine sweep, every later
            // click refines at full resolution only. "pyramid level 0/0"
            // would read as if nothing were happening, so phrase that case
            // as a plain refinement pass instead.
            NSString* stageDesc = (totalLevels > 1)
                ? [NSString stringWithFormat:@"pyramid level %ld/%ld", (long)level, (long)(totalLevels - 1)]
                : @"refining at full resolution";
            weakSelf.statusLabel.stringValue = [NSString stringWithFormat:@"Optimizing… %@, iteration %ld/%ld, RMSE=%.4f%@",
                                                 stageDesc, (long)iter, (long)(totalIters - 1), rmse, unitTag];
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
            weakSelf.animateMeshButton.enabled = YES;
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

// Toggles "Animate Mesh" on/off -- see DocumentModel.h's isAnimatingMesh/
// startMeshAnimationWithRedraw:/stopMeshAnimation for the actual mechanics
// (temperature-controlled per-vertex amplitude, sin(t) motion, boundary
// vertices pinned). This method only owns the UI side: reading+clamping
// the two fields, flipping the button title, and disabling Optimize/Build
// Mesh while an animation is running (mirroring how -optimize: disables
// them for an in-flight run) so the two features can't stack confusingly.
- (void)toggleAnimateMesh:(id)sender {
    if (self.documentModel.isAnimatingMesh) {
        [self.documentModel stopMeshAnimation];
        self.animateMeshButton.title = @"Animate Mesh";
        self.optimizeButton.enabled = YES;
        self.buildMeshButton.enabled = YES;
        self.statusLabel.stringValue = @"Animation stopped.";
        // -meshForReading now reads the real (never-touched) _mesh again --
        // refresh the reconstruction preview back to it, or "Show
        // reconstruction" would keep showing the last animated frame's
        // stale raster/GPU buffers until some OTHER action refreshed it.
        [self refreshActivePreview];
        return;
    }
    if (!self.documentModel.hasMesh) { self.statusLabel.stringValue = @"Build a mesh first."; return; }
    // MAX(0, ...) for amplitude (0 is a valid "don't move" value); MAX with
    // a small positive floor for temperature since it divides an exponent
    // (see meshAnimationTemperature's doc comment in DocumentModel.h) --
    // same clamp DocumentModel.mm's -startMeshAnimationWithRedraw: itself
    // applies again defensively, so a stray 0 typed here can never crash.
    self.documentModel.meshAnimationStyle = (GMMeshAnimationStyle)self.animStylePopup.indexOfSelectedItem;
    self.documentModel.meshAnimationMaxAmplitude = MAX(0.0, self.animAmplitudeField.doubleValue);
    self.documentModel.meshAnimationTemperature = MAX(0.01, self.animTemperatureField.doubleValue);
    // No clamping here -- any real degree value is meaningful (cos/sin
    // wrap around on their own), and this only affects Wave -- see
    // meshAnimationWaveDirectionDegrees's doc comment in DocumentModel.h.
    self.documentModel.meshAnimationWaveDirectionDegrees = self.animDirectionField.doubleValue;
    // 0 is a valid value here (disables the clearance check entirely,
    // falling back to literal-crossing-only) -- see
    // meshAnimationMinClearanceDistance's doc comment in DocumentModel.h.
    self.documentModel.meshAnimationMinClearanceDistance = MAX(0.0, self.animClearanceField.doubleValue);
    // See meshAnimationDebugDisableContinuousStages' doc comment in
    // DocumentModel.h -- same "read once at (re)start" convention as
    // every other field on this row, so ticking the box takes effect the
    // next time "Animate Mesh" is (re)started, no rebuild required.
    self.documentModel.meshAnimationDebugDisableContinuousStages =
        (self.animDisableContinuousCheckbox.state == NSControlStateValueOn);
    __weak typeof(self) weakSelf = self;
    [self.documentModel startMeshAnimationWithRedraw:^{
        // -refreshActivePreview is a cheap no-op when "Show reconstruction"
        // is off (just a couple of property assignments); when it's on, it
        // re-rasterizes (CPU) or re-uploads mesh buffers (GPU) from the
        // CURRENT animated mesh every tick, so the reconstruction preview
        // wiggles along with the mesh grid, not just the wireframe overlay.
        // NOTE: the CPU raster path re-rasterizes the full target image
        // every tick and can feel choppy at 60fps on a large photo --
        // switch on "GPU (OpenGL)" for a smooth real-time reconstruction
        // preview while animating.
        [weakSelf refreshActivePreview];
    }];
    self.animateMeshButton.title = @"Stop Animation";
    self.optimizeButton.enabled = NO;
    self.buildMeshButton.enabled = NO;
    self.statusLabel.stringValue = @"Animating mesh… click “Stop Animation” to return to the fitted mesh.";
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

// Overrides -[NSResponder presentError:], which returns BOOL (the old `void`
// signature triggered -Wmismatched-return-types). YES: the error was presented.
- (BOOL)presentError:(NSError*)error {
    NSAlert* alert = [[NSAlert alloc] init];
    alert.messageText = @"Gradient Mesh Studio";
    alert.informativeText = error.localizedDescription ?: @"An unknown error occurred.";
    [alert beginSheetModalForWindow:self.window completionHandler:nil];
    return YES;
}

@end
