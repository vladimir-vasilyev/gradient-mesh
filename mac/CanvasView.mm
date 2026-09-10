#import "CanvasView.h"
#include <cmath>

@interface CanvasView () {
    NSMutableArray<NSValue*>* _boundaryDraft;   // points while tracing (image coords)
    NSMutableArray<NSNumber*>* _cornerDraft;    // indices into documentModel.boundaryPolygonPoints
    NSMutableArray<NSValue*>* _vectorLineDraft; // points while drawing the current guide line
    NSMutableArray<NSValue*>* _scribbleDraft;   // points while painting the current scribble stroke
    BOOL _draggingVertex;
    NSInteger _dragRow, _dragCol;
    NSInteger _editingRow, _editingCol; // for the color panel callback
    NSImage* _reconstructionPreview;
}
- (void)appendCurveInto:(NSBezierPath*)path from:(NSPoint)p0 dm:(DocumentModel*)dm
                      r0:(NSInteger)r0 c0:(NSInteger)c0 r1:(NSInteger)r1 c1:(NSInteger)c1;
- (void)drawArrowFromImagePoint:(NSPoint)aImg toImagePoint:(NSPoint)bImg color:(NSColor*)color;
- (void)drawScribbles;
- (void)strokeScribble:(NSArray<NSValue*>*)pts color:(NSColor*)color;
@end

@implementation CanvasView

- (instancetype)initWithFrame:(NSRect)frameRect {
    if ((self = [super initWithFrame:frameRect])) {
        _boundaryDraft = [NSMutableArray array];
        _cornerDraft = [NSMutableArray array];
        _vectorLineDraft = [NSMutableArray array];
        _scribbleDraft = [NSMutableArray array];
        _showMeshOverlay = YES;
    }
    return self;
}

- (BOOL)isFlipped { return YES; } // origin top-left, matches image pixel coordinates

#pragma mark - Coordinate mapping

- (NSRect)imageDisplayRect {
    DocumentModel* dm = self.documentModel;
    if (!dm.hasImage) return self.bounds;
    double iw = dm.imageWidth, ih = dm.imageHeight;
    NSRect b = self.bounds;
    double scale = MIN(b.size.width / iw, b.size.height / ih);
    double w = iw * scale, h = ih * scale;
    double x = b.origin.x + (b.size.width - w) * 0.5;
    double y = b.origin.y + (b.size.height - h) * 0.5;
    return NSMakeRect(x, y, w, h);
}

- (double)currentScale {
    DocumentModel* dm = self.documentModel;
    if (!dm.hasImage) return 1.0;
    NSRect r = [self imageDisplayRect];
    return r.size.width / MAX(1.0, dm.imageWidth);
}

- (NSPoint)imagePointFromViewPoint:(NSPoint)vp {
    NSRect r = [self imageDisplayRect];
    double scale = [self currentScale];
    if (scale < 1e-9) return NSZeroPoint;
    return NSMakePoint((vp.x - r.origin.x) / scale, (vp.y - r.origin.y) / scale);
}

- (NSPoint)viewPointFromImagePoint:(NSPoint)ip {
    NSRect r = [self imageDisplayRect];
    double scale = [self currentScale];
    return NSMakePoint(r.origin.x + ip.x * scale, r.origin.y + ip.y * scale);
}

#pragma mark - Public API

- (void)resetBoundaryDrawing {
    [_boundaryDraft removeAllObjects];
    [self setNeedsDisplay:YES];
}

- (void)resetCornerPicking {
    [_cornerDraft removeAllObjects];
    [self setNeedsDisplay:YES];
}

- (void)refreshReconstructionPreview {
    _reconstructionPreview = [self.documentModel renderReconstructionPreview];
    [self setNeedsDisplay:YES];
}

- (void)setToolMode:(GMToolMode)toolMode {
    _toolMode = toolMode;
    [_vectorLineDraft removeAllObjects];
    [_scribbleDraft removeAllObjects];
    [self setNeedsDisplay:YES];
}

#pragma mark - Drawing

- (void)drawRect:(NSRect)dirtyRect {
    [[NSColor colorWithCalibratedWhite:0.16 alpha:1.0] setFill];
    NSRectFill(self.bounds);

    DocumentModel* dm = self.documentModel;
    if (!dm.hasImage) return;
    NSRect r = [self imageDisplayRect];

    NSImage* shown = (self.showReconstructionPreview && _reconstructionPreview) ? _reconstructionPreview : dm.displayImage;
    // -[NSImage drawInRect:fromRect:operation:fraction:] does NOT automatically
    // compensate for -isFlipped==YES on the destination view -- despite that
    // being the commonly assumed behavior for this "modern" (post-10.6)
    // drawing API, it draws the image as-is in the CURRENT graphics-state
    // coordinate system, so in this view (isFlipped=YES, origin top-left,
    // matching every other coordinate in this file) the image comes out
    // vertically mirrored. Found after an on-device report that the loaded
    // photo displays upside-down. Fixed by reflecting just this one draw call
    // vertically within its own rect (save/concat/restore scoped to only this
    // call, so drawBoundary/drawMesh/drawTangents/drawVectorLines below --
    // which already correctly assume a right-side-up image via
    // viewPointFromImagePoint: -- are unaffected and stay correctly aligned
    // now that the image itself displays correctly).
    [NSGraphicsContext saveGraphicsState];
    NSAffineTransform* flip = [NSAffineTransform transform];
    [flip translateXBy:0 yBy:(r.origin.y * 2 + r.size.height)];
    [flip scaleXBy:1.0 yBy:-1.0];
    [flip concat];
    [shown drawInRect:r fromRect:NSZeroRect operation:NSCompositingOperationCopy fraction:1.0];
    [NSGraphicsContext restoreGraphicsState];

    [self drawBoundary];
    // -drawMesh reads -meshVertexPositionAtRow:col:/-meshVertexColorAtRow:col:,
    // which read the LIVE mesh while !dm.isOptimizing (safe, nothing else
    // touches it then) and the livePreviewDuringOptimize snapshot while
    // dm.isOptimizing && dm.hasLivePreviewMesh (also safe -- see that
    // property's comment in DocumentModel.h). Either way this call never
    // reads the live mesh concurrently with the background optimizer thread
    // mutating it -- there is no third case where drawing here would race.
    if (self.showMeshOverlay && dm.hasMesh && (!dm.isOptimizing || dm.hasLivePreviewMesh)) [self drawMesh];
    // Tangent-arrow overlay stays optimize-only-when-idle -- it isn't part
    // of this request and doesn't need the preview snapshot to stay safe.
    if (self.showTangents && dm.hasMesh && !dm.isOptimizing) [self drawTangents];
    [self drawVectorLines];
    [self drawScribbles];
}

- (void)drawBoundary {
    DocumentModel* dm = self.documentModel;
    if (dm.hasBoundary) {
        // Once fitted, draw the REAL 4 cubic Bezier boundary splines (Sec.
        // 4: "each boundary consists of one or more cubic Bezier splines"),
        // not a straight approximation of the originally-traced polygon.
        NSBezierPath* path = [NSBezierPath bezierPath];
        path.lineWidth = 1.5;
        BOOL first = YES;
        for (NSArray<NSValue*>* ctrl in [dm fittedBoundaryCurves]) {
            NSPoint p0 = [self viewPointFromImagePoint:[ctrl[0] pointValue]];
            NSPoint p1 = [self viewPointFromImagePoint:[ctrl[1] pointValue]];
            NSPoint p2 = [self viewPointFromImagePoint:[ctrl[2] pointValue]];
            NSPoint p3 = [self viewPointFromImagePoint:[ctrl[3] pointValue]];
            if (first) { [path moveToPoint:p0]; first = NO; }
            [path curveToPoint:p3 controlPoint1:p1 controlPoint2:p2];
        }
        [path closePath];
        [[NSColor systemYellowColor] setStroke];
        [path stroke];
    } else if (_boundaryDraft.count > 0 || dm.boundaryPolygonPoints.count > 0) {
        // Falls back to dm.boundaryPolygonPoints when _boundaryDraft is
        // empty -- the same fallback the corner-picking mouseDown: handler
        // below already used (so clicking corners already worked on a
        // segmentation-produced polygon), but -drawBoundary itself didn't
        // draw anything in that case until now: a polygon set via
        // -segmentBoundaryFromScribblesWithError: (instead of manual
        // click-tracing, which populates _boundaryDraft as it goes) would
        // otherwise be invisible right up until the user started clicking
        // corners blind.
        NSArray<NSValue*>* polyPts = _boundaryDraft.count > 0 ? _boundaryDraft : dm.boundaryPolygonPoints;
        NSBezierPath* path = [NSBezierPath bezierPath];
        path.lineWidth = 1.5;
        NSPoint first = [self viewPointFromImagePoint:[polyPts[0] pointValue]];
        [path moveToPoint:first];
        for (NSUInteger i = 1; i < polyPts.count; ++i)
            [path lineToPoint:[self viewPointFromImagePoint:[polyPts[i] pointValue]]];
        [[NSColor systemYellowColor] setStroke];
        [path stroke];
        for (NSValue* v in polyPts) {
            NSPoint p = [self viewPointFromImagePoint:[v pointValue]];
            NSRect dot = NSMakeRect(p.x - 3, p.y - 3, 6, 6);
            [[NSColor systemYellowColor] setFill];
            [[NSBezierPath bezierPathWithOvalInRect:dot] fill];
        }
    }
    // corner picks (only meaningful before hasBoundary is finalized) --
    // NOTE: indices are into whichever polygon GMToolModeCorners' own
    // mouseDown: snapped against (_boundaryDraft if non-empty, else
    // dm.boundaryPolygonPoints -- see that switch case below), so this
    // lookup must use the SAME fallback, not _boundaryDraft unconditionally.
    NSArray<NSValue*>* cornerLookupPts = _boundaryDraft.count > 0 ? _boundaryDraft : dm.boundaryPolygonPoints;
    NSInteger ci = 0;
    for (NSNumber* idx in _cornerDraft) {
        NSInteger i = idx.integerValue;
        if (i < 0 || i >= (NSInteger)cornerLookupPts.count) continue;
        NSPoint p = [self viewPointFromImagePoint:[cornerLookupPts[(NSUInteger)i] pointValue]];
        NSRect dot = NSMakeRect(p.x - 5, p.y - 5, 10, 10);
        [[NSColor systemRedColor] setStroke];
        NSBezierPath* ring = [NSBezierPath bezierPathWithOvalInRect:dot];
        ring.lineWidth = 2;
        [ring stroke];
        NSString* label = [NSString stringWithFormat:@"%ld", (long)(++ci)];
        [label drawAtPoint:NSMakePoint(p.x + 6, p.y - 6)
            withAttributes:@{NSForegroundColorAttributeName: [NSColor systemRedColor]}];
    }
}

- (void)drawMesh {
    DocumentModel* dm = self.documentModel;
    NSInteger rows = dm.meshRows, cols = dm.meshCols;
    NSBezierPath* grid = [NSBezierPath bezierPath];
    grid.lineWidth = 1.0;
    // Each mesh-line segment between grid-adjacent control points is drawn
    // as the EXACT cubic Bezier curve of that Ferguson-patch edge (see
    // -[DocumentModel meshEdgeBezierFromRow:col:toRow:col:]), not a
    // straight line between the two points. Previously this overlay always
    // connected control points with straight lines, so even a mesh whose
    // geometry *had* curved to follow an internal color boundary looked
    // perfectly rectangular on screen -- a pure rendering artifact,
    // unrelated to whether the optimizer actually bent the surface.
    for (NSInteger r = 0; r < rows; ++r) {
        for (NSInteger c = 0; c < cols; ++c) {
            NSPoint p0 = [self viewPointFromImagePoint:[dm meshVertexPositionAtRow:r col:c]];
            if (c + 1 < cols) [self appendCurveInto:grid from:p0 dm:dm r0:r c0:c r1:r c1:c + 1];
            if (r + 1 < rows) [self appendCurveInto:grid from:p0 dm:dm r0:r c0:c r1:r + 1 c1:c];
        }
    }
    [[NSColor colorWithCalibratedWhite:1.0 alpha:0.55] setStroke];
    [grid stroke];

    for (NSInteger r = 0; r < rows; ++r) {
        for (NSInteger c = 0; c < cols; ++c) {
            NSPoint p = [self viewPointFromImagePoint:[dm meshVertexPositionAtRow:r col:c]];
            NSRect dot = NSMakeRect(p.x - 3, p.y - 3, 6, 6);
            [[dm meshVertexColorAtRow:r col:c] setFill];
            [[NSBezierPath bezierPathWithOvalInRect:dot] fill];
            [[NSColor blackColor] setStroke];
            NSBezierPath* ring = [NSBezierPath bezierPathWithOvalInRect:dot];
            ring.lineWidth = 0.5;
            [ring stroke];
        }
    }
}

// Appends one mesh-edge segment (r0,c0)->(r1,c1), which must be
// grid-adjacent, to `path` as a cubic Bezier curve in view coordinates,
// starting from the already-converted view point `p0`.
- (void)appendCurveInto:(NSBezierPath*)path from:(NSPoint)p0 dm:(DocumentModel*)dm
                      r0:(NSInteger)r0 c0:(NSInteger)c0 r1:(NSInteger)r1 c1:(NSInteger)c1 {
    NSArray<NSValue*>* ctrl = [dm meshEdgeBezierFromRow:r0 col:c0 toRow:r1 col:c1];
    [path moveToPoint:p0];
    if (ctrl.count == 4) {
        NSPoint b1 = [self viewPointFromImagePoint:[ctrl[1] pointValue]];
        NSPoint b2 = [self viewPointFromImagePoint:[ctrl[2] pointValue]];
        NSPoint b3 = [self viewPointFromImagePoint:[ctrl[3] pointValue]];
        [path curveToPoint:b3 controlPoint1:b1 controlPoint2:b2];
    } else {
        // Shouldn't happen for a grid-adjacent pair, but fail safe to a
        // straight segment rather than dropping the edge.
        NSPoint p1 = [self viewPointFromImagePoint:[dm meshVertexPositionAtRow:r1 col:c1]];
        [path lineToPoint:p1];
    }
}

// Diagnostic overlay: draws the FREE tangent unknowns Pu (orange) and Pv
// (cyan) at every control point as arrows. These are independently
// optimized (not derived from position any more) and directly determine
// each patch edge's curvature (see appendCurveInto:...) -- visualizing them
// shows, e.g., a vertex whose tangent has swung to align with a nearby
// edge, versus one still close to its axis-aligned initial estimate.
// Scaled down (0.35x) so arrows don't just retrace the neighbor-to-neighbor
// mesh-grid spacing.
- (void)drawTangents {
    DocumentModel* dm = self.documentModel;
    NSInteger rows = dm.meshRows, cols = dm.meshCols;
    const double scale = 0.35;
    for (NSInteger r = 0; r < rows; ++r) {
        for (NSInteger c = 0; c < cols; ++c) {
            NSPoint pImg = [dm meshVertexPositionAtRow:r col:c];
            NSPoint tu = [dm meshVertexTangentUAtRow:r col:c];
            NSPoint tv = [dm meshVertexTangentVAtRow:r col:c];
            NSPoint uEndImg = NSMakePoint(pImg.x + tu.x * scale, pImg.y + tu.y * scale);
            NSPoint vEndImg = NSMakePoint(pImg.x + tv.x * scale, pImg.y + tv.y * scale);
            [self drawArrowFromImagePoint:pImg toImagePoint:uEndImg color:[NSColor systemOrangeColor]];
            [self drawArrowFromImagePoint:pImg toImagePoint:vEndImg color:[NSColor systemCyanColor]];
        }
    }
}

- (void)drawArrowFromImagePoint:(NSPoint)aImg toImagePoint:(NSPoint)bImg color:(NSColor*)color {
    NSPoint a = [self viewPointFromImagePoint:aImg];
    NSPoint b = [self viewPointFromImagePoint:bImg];
    double dx = b.x - a.x, dy = b.y - a.y;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.5) return; // too short to read as an arrow at this zoom; skip

    NSBezierPath* line = [NSBezierPath bezierPath];
    line.lineWidth = 1.5;
    [line moveToPoint:a];
    [line lineToPoint:b];
    [color setStroke];
    [line stroke];

    double ux = dx / len, uy = dy / len;
    double headLen = MIN(7.0, len * 0.5), headWidth = 3.5;
    NSPoint back = NSMakePoint(b.x - ux * headLen, b.y - uy * headLen);
    NSPoint left = NSMakePoint(back.x - uy * headWidth, back.y + ux * headWidth);
    NSPoint right = NSMakePoint(back.x + uy * headWidth, back.y - ux * headWidth);
    NSBezierPath* head = [NSBezierPath bezierPath];
    [head moveToPoint:b];
    [head lineToPoint:left];
    [head lineToPoint:right];
    [head closePath];
    [color setFill];
    [head fill];
}

- (void)drawVectorLines {
    DocumentModel* dm = self.documentModel;
    NSColor* c = [NSColor systemCyanColor];
    CGFloat dashPattern[2] = {6, 4};
    for (NSArray<NSValue*>* line in dm.vectorLinesPoints) [self strokeLine:line color:c dashed:NO dashPattern:dashPattern];
    if (_vectorLineDraft.count > 1) [self strokeLine:_vectorLineDraft color:[NSColor systemOrangeColor] dashed:YES dashPattern:dashPattern];
}

- (void)strokeLine:(NSArray<NSValue*>*)pts color:(NSColor*)color dashed:(BOOL)dashed dashPattern:(CGFloat[2])dash {
    if (pts.count < 2) return;
    NSBezierPath* path = [NSBezierPath bezierPath];
    path.lineWidth = 2.0;
    if (dashed) [path setLineDash:dash count:2 phase:0];
    NSPoint first = [self viewPointFromImagePoint:[pts[0] pointValue]];
    [path moveToPoint:first];
    for (NSUInteger i = 1; i < pts.count; ++i) [path lineToPoint:[self viewPointFromImagePoint:[pts[i] pointValue]]];
    [color setStroke];
    [path stroke];
}

// Foreground scribbles draw green, background magenta/pink -- deliberately
// far apart on the color wheel (and from the yellow boundary/red corner-
// pick markers already used elsewhere in this view) so they stay legible
// on top of any source photo. The in-progress stroke (still being dragged)
// draws in the same color at reduced alpha, so it's visually distinct from
// already-committed strokes without needing a second color.
- (void)drawScribbles {
    DocumentModel* dm = self.documentModel;
    NSColor* fgColor = [NSColor systemGreenColor];
    NSColor* bgColor = [NSColor systemPinkColor];
    for (NSArray<NSValue*>* stroke in dm.foregroundScribblePoints) [self strokeScribble:stroke color:fgColor];
    for (NSArray<NSValue*>* stroke in dm.backgroundScribblePoints) [self strokeScribble:stroke color:bgColor];
    if (_scribbleDraft.count > 0) {
        NSColor* liveColor = (self.toolMode == GMToolModeScribbleBackground) ? bgColor : fgColor;
        [self strokeScribble:_scribbleDraft color:[liveColor colorWithAlphaComponent:0.55]];
    }
}

// Deliberately thicker (6pt) than strokeLine:'s 2pt vector-line stroke --
// scribbles are meant to read as a "paint brush" mark, not a thin guide
// line, and DocumentModel rasterizes each stored point into a matching-
// radius filled disc when building the actual segmentation scribble pixel
// set (see -addForegroundScribbleWithPoints:'s comment in DocumentModel.mm)
// -- the on-screen width here is a cosmetic echo of that, not read back
// from it, so the two are kept visually close but not algorithmically
// coupled to the exact same constant.
- (void)strokeScribble:(NSArray<NSValue*>*)pts color:(NSColor*)color {
    if (pts.count == 0) return;
    if (pts.count == 1) {
        // A single click with no drag -- a "dab" -- has no line to stroke;
        // draw it as a filled dot so it's still visible and still counts.
        NSPoint p = [self viewPointFromImagePoint:[pts[0] pointValue]];
        NSRect dot = NSMakeRect(p.x - 4, p.y - 4, 8, 8);
        [color setFill];
        [[NSBezierPath bezierPathWithOvalInRect:dot] fill];
        return;
    }
    NSBezierPath* path = [NSBezierPath bezierPath];
    path.lineWidth = 6.0;
    path.lineCapStyle = NSLineCapStyleRound;
    path.lineJoinStyle = NSLineJoinStyleRound;
    NSPoint first = [self viewPointFromImagePoint:[pts[0] pointValue]];
    [path moveToPoint:first];
    for (NSUInteger i = 1; i < pts.count; ++i) [path lineToPoint:[self viewPointFromImagePoint:[pts[i] pointValue]]];
    [color setStroke];
    [path stroke];
}

#pragma mark - Mouse handling

- (NSPoint)clampedImagePointForEvent:(NSEvent*)event {
    NSPoint vp = [self convertPoint:event.locationInWindow fromView:nil];
    NSPoint ip = [self imagePointFromViewPoint:vp];
    DocumentModel* dm = self.documentModel;
    ip.x = MAX(0, MIN(dm.imageWidth, ip.x));
    ip.y = MAX(0, MIN(dm.imageHeight, ip.y));
    return ip;
}

- (void)mouseDown:(NSEvent*)event {
    DocumentModel* dm = self.documentModel;
    if (!dm.hasImage) return;
    NSPoint ip = [self clampedImagePointForEvent:event];

    switch (self.toolMode) {
        case GMToolModeBoundary: {
            if (event.clickCount >= 2) {
                if (_boundaryDraft.count >= 3) {
                    [dm setBoundaryPolygonPoints:_boundaryDraft];
                    if (self.onBoundaryChanged) self.onBoundaryChanged();
                }
            } else {
                [_boundaryDraft addObject:[NSValue valueWithPoint:ip]];
            }
            [self setNeedsDisplay:YES];
            break;
        }
        case GMToolModeCorners: {
            // snap to nearest already-traced boundary point
            NSArray<NSValue*>* pts = _boundaryDraft.count > 0 ? _boundaryDraft : dm.boundaryPolygonPoints;
            NSInteger best = -1; double bestD2 = 1e18;
            for (NSUInteger i = 0; i < pts.count; ++i) {
                NSPoint p = [pts[i] pointValue];
                double dx = p.x - ip.x, dy = p.y - ip.y, d2 = dx * dx + dy * dy;
                if (d2 < bestD2) { bestD2 = d2; best = (NSInteger)i; }
            }
            if (best >= 0 && _cornerDraft.count < 4) {
                [_cornerDraft addObject:@(best)];
                if (_cornerDraft.count == 4 && self.onCornersPicked) self.onCornersPicked([_cornerDraft copy]);
            }
            [self setNeedsDisplay:YES];
            break;
        }
        case GMToolModeVectorLine: {
            [_vectorLineDraft removeAllObjects];
            [_vectorLineDraft addObject:[NSValue valueWithPoint:ip]];
            break;
        }
        case GMToolModeScribbleForeground:
        case GMToolModeScribbleBackground: {
            [_scribbleDraft removeAllObjects];
            [_scribbleDraft addObject:[NSValue valueWithPoint:ip]];
            [self setNeedsDisplay:YES];
            break;
        }
        case GMToolModeEditMesh: {
            NSInteger row = 0, col = 0;
            double thresholdImageSpace = 10.0 / MAX(0.001, [self currentScale]);
            if ([dm findNearestVertexToPoint:ip maxDistance:thresholdImageSpace row:&row col:&col]) {
                if (event.clickCount >= 2) {
                    _editingRow = row; _editingCol = col;
                    NSColorPanel* panel = [NSColorPanel sharedColorPanel];
                    panel.showsAlpha = NO;
                    panel.color = [dm meshVertexColorAtRow:row col:col];
                    panel.target = self;
                    panel.action = @selector(colorPanelChanged:);
                    [panel orderFront:nil];
                } else {
                    _draggingVertex = YES; _dragRow = row; _dragCol = col;
                }
            }
            break;
        }
        default: break;
    }
}

- (void)mouseDragged:(NSEvent*)event {
    DocumentModel* dm = self.documentModel;
    if (!dm.hasImage) return;
    NSPoint ip = [self clampedImagePointForEvent:event];
    if (self.toolMode == GMToolModeVectorLine) {
        NSPoint last = _vectorLineDraft.count > 0 ? [_vectorLineDraft.lastObject pointValue] : ip;
        double dx = last.x - ip.x, dy = last.y - ip.y;
        if (dx * dx + dy * dy > 16) [_vectorLineDraft addObject:[NSValue valueWithPoint:ip]];
        [self setNeedsDisplay:YES];
    } else if (self.toolMode == GMToolModeScribbleForeground || self.toolMode == GMToolModeScribbleBackground) {
        // Denser sampling (3px) than vector lines' 4px -- DocumentModel
        // stamps a filled disc around each stored point to build the
        // actual scribble pixel set (see -addForegroundScribbleWithPoints:'s
        // comment), and consecutive discs need to overlap for the painted
        // stroke to read as one continuous region rather than a dotted
        // line; see that comment for the matching disc radius.
        NSPoint last = _scribbleDraft.count > 0 ? [_scribbleDraft.lastObject pointValue] : ip;
        double dx = last.x - ip.x, dy = last.y - ip.y;
        if (dx * dx + dy * dy > 9) [_scribbleDraft addObject:[NSValue valueWithPoint:ip]];
        [self setNeedsDisplay:YES];
    } else if (self.toolMode == GMToolModeEditMesh && _draggingVertex) {
        [dm setMeshVertexPosition:ip atRow:_dragRow col:_dragCol];
        [self setNeedsDisplay:YES];
    }
}

- (void)mouseUp:(NSEvent*)event {
    if (self.toolMode == GMToolModeVectorLine) {
        if (_vectorLineDraft.count >= 2 && self.onVectorLineFinished) self.onVectorLineFinished([_vectorLineDraft copy]);
        [_vectorLineDraft removeAllObjects];
        [self setNeedsDisplay:YES];
    } else if (self.toolMode == GMToolModeScribbleForeground || self.toolMode == GMToolModeScribbleBackground) {
        // >=1 (not >=2 like vector lines): a single click with no drag is a
        // valid one-pixel "dab", useful for touching up a small mistake
        // without a full stroke.
        if (_scribbleDraft.count >= 1) {
            DocumentModel* dm = self.documentModel;
            if (self.toolMode == GMToolModeScribbleForeground) [dm addForegroundScribbleWithPoints:_scribbleDraft];
            else [dm addBackgroundScribbleWithPoints:_scribbleDraft];
            if (self.onScribblesChanged) self.onScribblesChanged();
        }
        [_scribbleDraft removeAllObjects];
        [self setNeedsDisplay:YES];
    } else if (self.toolMode == GMToolModeEditMesh && _draggingVertex) {
        _draggingVertex = NO;
        if (self.onMeshEdited) self.onMeshEdited();
    }
}

- (void)colorPanelChanged:(id)sender {
    NSColorPanel* panel = (NSColorPanel*)sender;
    [self.documentModel setMeshVertexColor:panel.color atRow:_editingRow col:_editingCol];
    [self setNeedsDisplay:YES];
    if (self.onMeshEdited) self.onMeshEdited();
}

@end
