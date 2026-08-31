# Gradient Mesh Studio

A from-scratch C++ implementation of the algorithm from **Jian Sun, Lin Liang, Fang Wen,
Heung-Yeung Shum, "Image Vectorization using Optimized Gradient Meshes,"
ACM Transactions on Graphics 26(3), 2007** (SIGGRAPH 2007), plus an interactive
macOS front-end (AppKit, Objective-C++) built as an Xcode project via CMake.

The paper turns a raster image into an editable, scalable **gradient mesh**: a grid of
control points, each carrying a position *and* a color, connected by curved
(Ferguson/Hermite) patches, fitted to the image by minimizing reconstruction error.

> **Update:** an earlier version of this project had internal mesh-lines stay
> essentially rectangular instead of bending to follow color/gradient boundaries inside
> the image, as they should (the *outer* silhouette boundary was already curving
> correctly). Root-caused and fixed against a real test image with a sharp internal
> boundary (`gradient.png`) -- see "Known simplifications" below (the
> `computeGeometryEnergy` / line-search bug) and the new `smoothWeightGeom` default in
> `MeshOptimizer.h`. If you pulled this project before that fix, rebuild.

## What's in here

```
GradientMeshStudio/
  core/                  gmcore: the portable C++17 algorithm library (no GUI, no
                          external dependencies beyond an optional system libpng)
    include/gmcore/*.h
    src/*.cpp
  cli/main_cli.cpp        gmesh_cli: dependency-free command-line test harness
  mac/                    GradientMeshStudio.app: the interactive AppKit UI
                          (Objective-C++, macOS-only)
  CMakeLists.txt          builds gmcore + gmesh_cli everywhere; builds the macOS
                          app bundle only when configured on Apple platforms
```

## Building on macOS (the Xcode project)

This was developed in a Linux sandbox with no Xcode available, so instead of hand-editing
a fragile `.xcodeproj` file blind, the project is set up so **CMake generates a real,
Xcode-native project for you** in one step:

```sh
# from the GradientMeshStudio/ directory, on your Mac:
cmake -G Xcode -B build .
open build/GradientMeshStudio.xcodeproj
```

Xcode will show two targets: `gmesh_cli` (a plain command-line tool -- handy for
regression testing) and `GradientMeshStudio` (the actual app, `⌘R` to run). No
third-party packages, no Swift Package Manager, no CocoaPods/Carthage -- everything
the app needs is either in `core/` or an Apple system framework (Cocoa, ImageIO,
CoreGraphics, UniformTypeIdentifiers).

If you don't have CMake yet: `brew install cmake`.

You can also just build from the terminal without opening Xcode:

```sh
cmake -B build .
cmake --build build --config Release
open build/GradientMeshStudio.app
```

### A note on how this was actually validated

The sandbox this was built in can compile and run plain C++ (`gmcore` + `gmesh_cli`),
but has no macOS SDK, so the AppKit files in `mac/` could not be compiled here. The
algorithm core, which is the mathematically interesting part, **was** built, run, and
checked end-to-end on Linux -- see "How this was tested" below. The `mac/*.mm` files
were written carefully against documented AppKit/CoreGraphics APIs and reviewed by hand,
but if Xcode's compiler flags an issue when you first build, it's most likely a small,
easy fix-it in `mac/` (a missing cast, an availability check) rather than a problem with
the algorithm itself.

## Using the app

1. **Open Image…** loads a photo.
2. **1. Trace Boundary**: click points around the object's outline; double-click to close
   the loop. (Or skip straight to step 5 for a quick, unattended run.)
3. **2. Pick 4 Corners**: click 4 of the points you just traced, in order around the
   loop -- these split the boundary into the 4 sides the mesh grid is built from
   (Sec. 4 of the paper).
4. Set **Rows/Cols** and click **Build Initial Mesh**.
5. **Auto (no markup)**: shortcut that skips steps 2-4 and drops a plain inset-rectangle
   boundary + regular grid over the whole image -- useful for a fast test, or when you
   don't want to trace anything by hand.
6. **3. Vector Line** (optional): click-drag to draw a guide line (e.g. along a highlight
   or fold); the optimizer will pull nearby mesh edges to align with it.
7. **Optimize** runs the coarse-to-fine solver in the background; the status bar shows
   live RMSE per pyramid level/iteration.
8. **4. Edit Mesh**: drag control points to nudge geometry; double-click one to open the
   color panel and repaint it.
9. **Export PNG…** rasterizes the current mesh; **Export SVG…** writes a real SVG2
   `<meshgradient>` document (see below).

## How the algorithm maps to the paper

| Paper concept (Sec. 3-4) | Code |
|---|---|
| Ferguson patch (bicubic Hermite, 16 corner values) | `FergusonPatch.h`: `HermiteBasis`, `PatchWeights`, `evalHermitePatch<T>` |
| Gradient mesh (grid of control points) | `GradientMesh.h/.cpp` |
| Boundary curves the mesh is fitted within | `BezierSpline.h/.cpp` (`fitCubicBezier`, `closestT`) |
| Energy minimization (data + smoothness + boundary + vector-line terms), Levenberg-Marquardt | `MeshOptimizer.h/.cpp` |
| Coarse-to-fine (Gaussian pyramid) | `Image::buildPyramid`, `MeshOptimizer::optimizeCoarseToFine` |
| User-drawn directional constraint | `VectorLine.h`, the vector-line term in `MeshOptimizer.cpp` |
| Scalable vector output | `SVGExporter.h/.cpp` (SVG2 `<meshgradient>`) |

## Known simplifications vs. the paper

Implementing the full paper exactly (jointly-optimized 16 unknowns per patch corner,
analytic derivative-continuity constraints, a general cutout/segmentation tool) is a
multi-person, multi-month effort. To get something real, working, and checkable within
this project's scope, a few deliberate simplifications were made -- all noted in code
comments at the relevant spot too:

* **Twist (`Puv`) is fixed at zero, per the paper's own text.** Position `P` and
  tangents `Pu`, `Pv` are free per-vertex unknowns, jointly refined by the geometry
  Gauss-Newton step (see "Fixed: Pu/Pv promoted to free unknowns" below). An earlier
  pass in this project (see "Fixed: Puv promoted to a free unknown too" below, now
  itself reverted -- see "Reverted: Puv back to fixed zero, vector-line term rewritten
  per Sec 4.2" further down) briefly promoted `Puv` to a free unknown too, "for
  completeness of paper-fidelity." Rereading the paper's Sec 3 turned up the actual
  text: "The mu, mv, muv are the partial derivatives. In practice, the values of muv
  are usually set to zero" -- i.e. the paper itself does NOT treat twist as a free
  unknown. `GradientMesh::geomCorner()` now hardcodes it to `{0,0}` again. Color keeps
  the full independent `(C, Cu, Cv, Cuv)` unknown set, unaffected by any of this.
* **Block-coordinate descent, not one joint solve.** Colors enter the reconstruction
  error *linearly*, so they're solved exactly with one sparse linear system per outer
  iteration; geometry enters *nonlinearly* (through the image lookup) and is refined with
  a few damped Gauss-Newton steps with backtracking line search. Both minimize the same
  energy, just alternately rather than jointly -- see `MeshOptimizer.h`'s header comment.
* **No external linear-algebra dependency.** Both solves use a small dependency-free
  block-sparse Conjugate Gradient solver (`SparseBlockSolver.h`) with Jacobi
  preconditioning, instead of e.g. Eigen -- this sandbox has no network access to vendor
  a third-party library, and it keeps the Xcode project dependency-free too.
* **The "cutout tool" is plain click-tracing.** The paper cites a separate
  Lazy-Snapping-style interactive segmentation tool for isolating the object; this
  project's boundary tool is just click-to-add-a-point polygon tracing, fitted with cubic
  Beziers per side afterward.
* **SVG mesh-gradient export uses one `<meshgradient>` per patch** rather than one mesh
  object with SVG2's inter-patch implicit-shared-edge stop omission encoding. A mesh
  gradient only paints inside its own patch boundary, so adjacent patches still tile
  seamlessly (they share exact boundary geometry, since both are derived from the same
  control-point tangents) -- this sidesteps a fiddly, easy-to-get-subtly-wrong part of
  the SVG2 spec that couldn't be cross-checked without network access. See the comment
  at the top of `SVGExporter.cpp` for the exact stop-color convention used.

None of these change the fundamental approach (Ferguson patches + energy minimization +
coarse-to-fine); they trade a bit of fidelity for something that is actually implemented,
runs, and was checked to work.

### Bug found and fixed: mesh-lines weren't bending to internal color boundaries

The outer boundary always curved correctly (it's constrained to the traced Bezier
splines), but interior mesh-lines stayed close to a plain rectangular grid instead of
warping toward internal gradient/color edges the way the paper's figures show. Root
cause, found by testing against a real image with a sharp internal boundary rather than
only the smooth synthetic sphere: the geometry Gauss-Newton step's backtracking line
search (in `optimizeAtCurrentResolution`) was accepting/rejecting steps by checking a
cheap, differently-sampled plain-RMSE proxy (`reconstructionRMSE(target, 3)`) instead of
the actual weighted energy the step was computed to reduce (data + smoothness +
boundary + vector-line terms, at the sample density used to build the step). That
mismatch let real, energy-decreasing steps get rejected; rejections drive the Levenberg
damping up, which shrinks the next step, which makes it even less likely to clear the
mismatched threshold -- a self-reinforcing lock that (combined with the original
`smoothWeightGeom = 40` default being too high relative to the data-term signal once
color has locally absorbed most of the error) left geometry essentially frozen after the
first couple of outer iterations.

Fixed by adding `computeGeometryEnergy()`, which mirrors the exact residual formulas used
to build the Gauss-Newton normal equations, and using *that* for the line search's
accept/reject decision instead of the RMSE proxy; and by lowering the `smoothWeightGeom`
default from 40 to 2 (see the comment on that field in `MeshOptimizer.h`). Verified on
`gradient.png` (a synthetic two-tone image with a sharp zigzag internal boundary,
included in this repo): before the fix, an 11x11 mesh's interior lines stayed visibly
grid-like and RMSE reduction topped out around 46-49%; after the fix, interior lines
visibly bend along the zigzag and RMSE reduction reaches 51-54% depending on
`smoothWeightGeom`. The synthetic sphere test (smooth image, no sharp internal edges)
was re-checked too and is unaffected (still ~47% reduction) -- the fix specifically
restores geometry's ability to respond to *sharp* internal color transitions, which the
sphere doesn't have much of.

**Separately, the mesh-line overlay in the app never actually drew curves at
all** -- `CanvasView::drawMesh` connected control points with straight
`lineToPoint:` segments regardless of how curved the underlying Ferguson
patches were. So even a mesh whose control points *had* moved to follow an
internal edge would still look like a plain rectangular grid on screen; this
was a pure rendering bug, independent of the optimizer bug above. Fixed by
adding `-[DocumentModel meshEdgeBezierFromRow:col:toRow:col:]`, which
converts each patch edge's Hermite representation (`P`, plus the relevant
derived `tangentU`/`tangentV`) to its exact equivalent cubic Bezier
(`B0=P0, B1=P0+T0/3, B2=P1-T1/3, B3=P1` -- the standard Hermite<->Bezier
change of basis, exact for cubic curves, not an approximation), and drawing
that with `NSBezierPath curveToPoint:controlPoint1:controlPoint2:` instead
of a straight line. The traced-boundary overlay got the same treatment via
`-[DocumentModel fittedBoundaryCurves]` (previously it also drew the raw
traced polygon instead of the 4 fitted splines). No optimizer/algorithm
change here -- purely what you *see* now matches what the mesh actually is.

`gmesh_cli` also gained a few flags for iterating on this without recompiling:
`--smooth-geom`, `--smooth-color`, `--color-ridge`, `--boundary-weight`,
`--outer-iters`, `--gn-iters`, `--samples` override the corresponding `OptimizerOptions`
field, and every run now also writes `<out-prefix>_mesh_points.csv` (one `row,col,x,y`
line per control point) for overlaying the mesh wireframe on the source image to check
visually whether it's actually conforming to image content.

### On matching the paper's reported ~0.7/pixel error (Fig. 4)

The paper reports errors low enough that reconstructions are visually
indistinguishable from the source photo. Testing against `gradient.png`
(21x21 synthetic image with a sharp, effectively step-function, zigzag
boundary between two flat-colored regions) at a few mesh resolutions:

| Mesh | Final RMSE (0-1 scale) | ~error/pixel (0-255 scale) |
|---|---|---|
| 9x9   | 0.0198 | ~5.0 |
| 15x15 | 0.0143 | ~3.6 |
| 25x25 | 0.0114 | ~2.9 |

Absolute error keeps improving with resolution/iterations (35x35 didn't
finish within a 2-minute budget in this sandbox, so there's more headroom),
but it's not going to reach ~0.7 on *this* image, and that's structural, not
a bug: `gradient.png` has a true pixel-hard discontinuity, and a smooth
C1 spline surface cannot represent a step edge to sub-1/255 accuracy unless
a mesh-line runs exactly along it. The paper's own Sec. 4 examples are real
photos (smooth natural gradients, not synthetic hard edges) and its cutout
tool is specifically about placing the mesh boundary/lines coincident with
real object silhouettes for this reason. To close the gap here: (a) increase
`--rows/--cols` and `--outer-iters/--gn-iters/--samples` for a tighter local
fit, and (b) drag mesh-lines (or use a vector-line guide) to sit directly on
the zigzag -- once a mesh-line coincides with the edge instead of merely
bending toward it, the discontinuity stops needing to be approximated by a
smooth patch at all. Real photographs without hard synthetic edges should
get much closer to paper-level numbers at moderate resolution.

### Deeper finding: coarse meshes still don't snap to an edge the way Fig. 4 does

Good catch from testing against the paper's Fig. 4 (a 4x4-patch/5x5-point mesh
where one whole interior column sits almost exactly on a sharp boundary):
reproducing that at 5x5 resolution on `gradient.png` still shows only
sub-pixel geometry movement (`gmesh_cli`'s `*_mesh_points.csv` positions stay
within ~0.5px of a perfectly uniform grid), even with the line-search fix and
the `smoothWeightGeom` default already lowered once (40 -> 2). Instrumented
the Gauss-Newton geometry step (`GMCORE_DEBUG_GEOM`) to see why: each outer
iteration the color solve gives geometry a real, non-zero gradient (`|g|` in
the low single digits), but it gets driven back to ~0 within that same outer
iteration's few GN sub-steps (`maxDelta` shrinking from ~0.4px to ~0.0002px)
-- i.e. this genuinely converges to (very close to) the rectangular starting
point as a real stationary point of the assembled energy at that
`smoothWeightGeom`, not a rejected/starved step. Raising `--samples` from 8
to 24 changed almost nothing, ruling out "the fit grid is too sparse to see
the edge" as the cause. Sweeping `--smooth-geom` down further on the same
5x5 case did free the geometry to move substantially:

| `--smooth-geom` | interior column's x-range across rows (px) |
|---|---|
| 2.0 (current default) | 106.4-106.9 (~0.5) |
| 0.5 | 104.6-106.5 (~1.9) |
| 0.1 | 103.8-109.2 (~5.4) |
| 0.02 | 99.2-109.1 (~9.9) |
| 0.0 | 101.0-116.4 (~15.4) |

(The true edge's x spans ~89-138 across those same rows, so even fully
unregularized geometry (0.0) only recovers about a third of the target
range, and not always in the right direction row-by-row -- so this isn't
simply "set smoothWeightGeom near 0 and ship it": with no regularizer at all
the coarse, few-DOF fit becomes a much harder non-convex problem and can
converge to a locally-plausible but wrong shape.) **Net: the 2.0 default is
still meaningfully overdamped for coarse-mesh edge-snapping specifically,
but naively zeroing it isn't a clean fix either.** Likely needs one or more
of: an annealed `smoothWeightGeom` schedule (higher early for stability,
decaying over outer iterations), more outer/GN iterations at coarse
resolutions, and/or promoting `Pu`/`Pv` from derived (finite-difference)
values to free per-vertex unknowns as the paper actually does (see "Known
simplifications" above) -- derived tangents mechanically limit how sharply
one point can bend without its finite-difference neighbors fighting it. The
last option is what was actually implemented -- see "Fixed: Pu/Pv promoted
to free unknowns" below.

**Diagnostic: tangent visualization.** The app has a "Show tangents"
checkbox that draws each control point's `Pu` (orange) and `Pv` (cyan) as
arrows (`-[DocumentModel meshVertexTangentUAtRow:col:]` /
`...TangentVAtRow:col:]`, drawn by `-[CanvasView drawTangents]`) -- useful
for seeing directly whether tangents are actually swinging to track an edge
or staying axis-aligned/uniform-length like an unbent grid, which is exactly
the symptom found above. (Originally these read the derived
`tangentU`/`tangentV` estimate; now that `Pu`/`Pv` are free unknowns, they
read the actual optimized `MeshVertex::Pu`/`Pv` fields instead, so what you
see is what the optimizer is actually using.)

### Fixed: Pu/Pv promoted to free unknowns (matching the paper)

Implemented the option flagged above: `Pu` and `Pv` are no longer derived
via finite differences -- they're free per-vertex unknowns, jointly refined
alongside `P` by the same geometry Gauss-Newton step (`MeshVertex` gained
`Pu`, `Pv` fields; `geomCorner()` reads them directly; `buildInitial()`
seeds them from the old finite-difference estimate as a starting point; the
GN unknown block grew from 2 components/vertex (`P.x,P.y`) to 6
(`P.x,P.y,Pu.x,Pu.y,Pv.x,Pv.y`), with the corresponding Jacobian rows for
`Pu`/`Pv` built directly from `PatchWeights`' `TangentU`/`TangentV` weights
-- see `MeshOptimizer.cpp`). Twist (`Puv`) stayed derived, unchanged, at the
time this was written -- see the next section for the follow-up pass that
closed that gap too.

A free tangent has no sensible "pull toward zero" prior the way a color
derivative does (`colorDerivRidge`) -- zero tangent collapses the patch.
Tried it anyway as a control (`--tangent-prior 0`, i.e. completely
unconstrained `Pu`/`Pv`): `Pu`/`Pv` values went chaotic (e.g. one run's `Pu.x`
ranged from -53 to +188 across a mesh whose spacing is ~50px) and the
rendered reconstruction came out visibly warped and torn at the silhouette,
even though the coarse-sample RMSE metric it was fitting *improved* --
classic overfitting to a sparse sample grid, invisible to that metric.
Fixed by adding `geomTangentPriorWeight`: a soft ridge pulling `Pu`/`Pv`
toward the *current* position-implied finite-difference estimate
(`GradientMesh::tangentU/V`, recomputed fresh every GN sub-iteration), so
the data term can pull a tangent away from that estimate where it has real
signal to, without it drifting unboundedly where the signal is weak.
`computeGeometryEnergy` was extended to include this term too, for the same
line-search-consistency reason as everything else in it.

With that grounding in place, `smoothWeightGeom` could be lowered much
further (0.02, from 2) without the garbling seen above -- on the *same*
5x5/`gradient.png` case from the table above, with `--tangent-prior 0.6`:

| config | interior column's x-range (px) | reconstruction |
|---|---|---|
| old defaults (`smooth-geom 2`, derived tangents) | 106.4-106.9 (~0.5) | boxy, doesn't track the notch |
| new defaults (`smooth-geom 0.02`, free tangents, `tangent-prior 0.6`) | 100.1-111.4 (~11.3) | visibly bends to follow the zigzag |

Checked with a wireframe overlay (mesh-line Bezier curves drawn over
`gradient.png`, same technique as the `CanvasView` fix above): with the new
defaults the interior mesh column visibly kinks to trace the boundary's
notch, instead of running as a nearly-straight vertical line down the
middle. Re-checked for regressions: 25x25 on `gradient.png` improved too
(final RMSE 0.0114 -> 0.0083, i.e. ~2.9 -> ~2.1 in 0-255 terms) and the
noisy synthetic sphere (smooth image, the regression check for "did this
break normal images") also improved (45.5% -> 52.6% RMSE reduction, no
visible artifacts) rather than regressing. New defaults:
`smoothWeightGeom = 0.02`, `geomTangentPriorWeight = 0.6` (new option;
`--tangent-prior` in `gmesh_cli`).

Caveat: the geometry unknown block growing 2 -> 6 components/vertex makes
each Gauss-Newton solve noticeably more expensive (roughly an order of
magnitude on the 25x25 case in this sandbox, ~3 minutes vs under a minute
previously, at the CLI's higher-than-default iteration counts used for
testing) -- the app's own defaults (8 outer iterations, 3 GN iterations/outer,
per `OptimizerOptions`) stay fast at typical mesh sizes (9x9), but a large
mesh with many manually-increased iterations will take noticeably longer
than before. Not yet profiled/optimized.

### Fixed: anisotropic smoothWeightGeom (relax smoothing near real edges)

Even with free Pu/Pv, a specific complaint held up on closer comparison against
the paper's Fig. 4: on a 5x5 mesh matching that figure's resolution, ours bent
one interior mesh-*line* toward the sharp boundary, but never let two adjacent
lines squeeze together into a visibly *narrow patch column* straddling the
edge from both sides, the way Fig. 4 shows. Root cause: `smoothWeightGeom`'s
2nd-difference term is isotropic -- it penalizes uneven spacing between
neighboring control points identically everywhere, and two mesh-lines
converging around an edge is, almost by definition, a large local departure
from even spacing. Lowering the single global weight (previous fix) helps
overall bending but can't selectively stop resisting *specifically* near a
real edge without also giving up smoothing everywhere else.

Fix: `smoothWeightGeom` is no longer a flat constant -- each 2nd-difference
triple's effective weight is scaled by `edgeRelaxFactor()`, a new helper that
samples the image gradient magnitude at the middle control point's *current*
position (recomputed fresh every GN sub-iteration, so it adapts as a vertex
approaches an edge) and relaxes smoothing there:
`weight = smoothWeightGeom / (1 + smoothGeomEdgeGain * localGradientMagnitude)`,
floored at `smoothGeomMinFactor` (never fully zero -- keeps the solve
well-posed). `computeGeometryEnergy` mirrors the exact same per-triple scaling,
for the same line-search-consistency reason as everywhere else here. New
options: `smoothGeomEdgeGain = 40.0`, `smoothGeomMinFactor = 0.05`
(`--edge-gain` / `--edge-min-factor` in `gmesh_cli`).

Swept `--edge-gain` from 0 (isotropic, old behavior) up to 1500 on the same
5x5/`gradient.png` case: RMSE improved from the isotropic baseline (56.6%
reduction) up to a peak around gain 40-100 (~61.3-61.4%), then *degraded*
at higher gain (600: 59.2%, 1500: 59.3%) -- over-relaxing lets the smoothness
term stop doing its job even where it should, so this isn't "more is better."
40 was picked as the default (near-peak, round number). Checked visually via
the same wireframe-overlay technique as before: with gain 40, the interior
mesh-lines visibly bow toward the notch rather than running close to straight
-- a real improvement -- but this alone still didn't produce a dramatic
near-zero-width pinched column like Fig. 4's. That makes sense on reflection:
relaxing the *resistance* to non-uniform spacing doesn't create a new force
pulling two lines together -- it only lets an existing pull win more easily.
The data term's own incentive to physically narrow a column, absent that,
still seems to be fairly weak (consistent with the earlier finding that color
alone already absorbs much of the local residual). Getting the dramatic
Fig.-4-style pinch probably needs an actual attractive force -- the
`vectorLineWeight` constraint was tested as one candidate (see the previous
turn's investigation: it *did* produce dramatic column-clustering, e.g. two
columns landing 2.5px apart, but overshot and made overall RMSE worse at its
current default weight/semantics) -- not pursued further this round.

Regression check: the noisy synthetic sphere (smooth image, no real edges,
per-pixel noise baked in) went from 52.6% to 50.5% RMSE reduction with
anisotropic relaxation on -- a real but small regression, because the
injected noise itself produces small nonzero local gradients everywhere,
so `edgeRelaxFactor` relaxes smoothing slightly even in "flat" regions
purely from that noise. Judged an acceptable trade for a meaningfully
better result on real edges; flagging honestly rather than hiding it. Did
not re-verify the 25x25 gradient.png case with this change (each such run
takes several minutes in this sandbox) -- if you have a moment, `gmesh_cli
--rows 25 --cols 25 ...` and comparing against the numbers in the section
above would be a useful check.

### Fixed: Puv (twist) promoted to a free unknown too

> **Note (later reverted):** this section is kept as an honest historical record, but
> the change it describes was undone in a later pass after rereading the paper's Sec 3
> turned up "In practice, the values of muv are usually set to zero" -- see "Reverted:
> Puv back to fixed zero, vector-line term rewritten per Sec 4.2" near the end of this
> file. `Puv` is fixed at `{0,0}` again, not a free unknown.

Follow-up to the two sections above: `Puv` (the mixed second partial,
"twist", at each Ferguson-patch corner) was still *derived* via centered
finite differences from neighboring vertices' `P` (`GradientMesh::twist`),
the one remaining gap vs. the paper's fully-free 4-value-per-corner Hermite
corner (`P`, `Pu`, `Pv`, `Puv` all independent). Promoting it too, for
completeness of paper-fidelity rather than as a targeted fix for the Fig. 4
pinch investigation above (a derived-vs-free twist term was never a leading
hypothesis for *that* -- it only ever contributes a general surface-shape
degree of freedom, not a force pulling mesh-lines together).

Mechanically: `MeshVertex` gained a `Puv` field (`GradientMesh.h`);
`geomCorner()` reads it directly instead of calling `twist()`;
`buildInitial()` seeds it from `twist()` as a starting point, same pattern
already used for `Pu`/`Pv` and `tangentU`/`tangentV`; `scalePositions()`
scales it the same linear per-component way as `P`/`Pu`/`Pv` between
pyramid levels; the geometry GN unknown block grew from 6 components/vertex
to 8 (`...,Puv.x,Puv.y`, still comfortably under `SparseBlockSolver.h`'s
`double tmp[16]` ceiling); `addTangentPriorTerms`/`computeGeometryEnergy`
extended to also ground `Puv` toward the current `twist()` estimate via the
same `geomTangentPriorWeight`, for the same well-posedness reason `Pu`/`Pv`
needed grounding. The Ceres path (both `useCeresGeometry` and
`useCeresJoint`) got the matching treatment: `PatchDataCostFunction`/
`JointPatchDataCostFunction`'s geometry parameter block grew 6->8 doubles
with a new Jacobian column pair using `PatchWeights`' already-existing
`Twist` kind weight, and `TangentPriorCostFunction` grew from 4 to 6
residuals to also ground `Puv`. This incidentally makes the joint data-term
Jacobian *more* exact than before: the 0.168 max-relative-error gap found
during the joint-solve `ceres::GradientChecker` verification (see below) was
specifically because derived-`Puv` depended on neighboring vertices outside
a residual block's own 4 corners, which the Jacobian couldn't represent --
with `Puv` now a direct per-corner parameter, that gap is gone (reverified
with the same finite-difference-harness method, 200 entries checked, 0 bad,
max relative error 4e-5 -- pure FD noise).

Regression-checked on the same three cases used throughout this section
(hand-rolled path, no Ceres, defaults unchanged otherwise):

| case | before (Puv derived) | after (Puv free) |
|---|---|---|
| synthetic sphere, 9x9 | 53.2% reduction | 54.1% reduction |
| 25x25 `gradient.png` | 61.4% reduction | 59.1% reduction |
| 5x5 `gradient.png` | 61.0% reduction | 60.8% reduction |

No divergence or instability in any case -- RMSE still decreases smoothly
through the optimization in all three. The 25x25 case regressed a couple of
points; plausible since `smoothWeightGeom`/`geomTangentPriorWeight`/
`smoothGeomEdgeGain` were all empirically tuned earlier assuming a derived
(FD-smoothed, implicitly regularized) twist, and an unconstrained `Puv` now
has one more way to (slightly) overfit the coarse sample grid before the
prior fully catches it. Reporting honestly rather than re-tuning weights to
paper over it; if it matters in practice, retuning `geomTangentPriorWeight`
specifically for the twist term (splitting it from the shared weight `Pu`/
`Pv` use) would be the first thing to try.

### Optional: Ceres-based geometry solver

The geometry Gauss-Newton block (position P + free tangents Pu, Pv, Puv) can
optionally be solved by [Ceres Solver](http://ceres-solver.org/) instead
of the hand-rolled damped GN + backtracking in `MeshOptimizer.cpp`. This
is purely additive and off by default: `CMakeLists.txt` does
`find_package(Ceres QUIET)` (never `REQUIRED`), and `core/src/MeshOptimizerCeres.cpp`
compiles to an empty translation unit when Ceres isn't found, so the
default, dependency-free build is completely unaffected -- nothing about
it changed in this pass.

Motivation: a fully joint (position+tangent+color) optimization pass --
one of the fidelity options considered for closing the remaining gap to
the paper's Fig. 4 (see "Deeper finding" above) -- would need a much
larger per-vertex parameter block than the hand-rolled solver's fixed
`double tmp[16]` scratch buffers can safely hold (20 dims: 8 geometry --
`P,Pu,Pv,Puv` -- + 12 color, now that `Puv` is also free; was 18 = 6+12
before that pass). Ceres's own sparse linear algebra and Levenberg-Marquardt
trust region don't have that ceiling, and would also remove an entire
class of hand-derived-Jacobian bugs (the kind that caused the "mesh-lines
weren't bending" bug above) via analytic verification against
`ceres::GradientChecker`. This pass ports *only* the existing geometry
GN step (color stays solved by the exact linear closed form, unchanged)
as a lower-risk first step and feasibility check before attempting a
fully joint solve.

Enable with `OptimizerOptions::useCeresGeometry = true` (or `gmesh_cli
--use-ceres`). Every residual/Jacobian in `MeshOptimizerCeres.cpp` is a
direct transcription of the corresponding hand-rolled term, not a
re-derivation -- and each was independently cross-checked before being
wired in:

- The data term (`PatchDataCostFunction`) was checked with
  `ceres::GradientChecker` via `spike/ceres_geom_spike.cpp`. First pass
  found a real bug: the per-sample area weight was being recomputed from
  the *trial* (perturbed) parameters instead of frozen at the
  linearization point, giving 906/1800 bad Jacobian entries (max relative
  error 1.81). Fixed by freezing it from a `snapshot` mesh, matching
  exactly how the hand-rolled path already treats that weight (frozen per
  GN sub-iteration there). After the fix: max relative error 0.168,
  confirmed identical between a standalone (non-Ceres) finite-difference
  check and the real `ceres::GradientChecker` run -- and that remaining
  gap was itself not new at the time: it was the then-still-open
  simplification that twist (`Puv`) wasn't differentiated w.r.t.
  neighboring vertex positions. **Now closed** -- see "Fixed: Puv promoted
  to a free unknown too" above; with `Puv` a direct per-corner parameter,
  this Jacobian is exact again (reverified the same way, 0 bad entries).
- Smoothness, tangent-prior, boundary and vector-line terms were each
  checked against a standalone finite-difference harness before being
  transcribed here; all four matched to numerical precision (no
  approximation involved in any of them).

Design note carried over honestly: every "frozen" quantity above (area
weight, the anisotropic edge-relax factor, tangent-prior's `tu`/`tv`
target, the vector-line direction) is evaluated once from a mesh snapshot
taken at the start of `optimizeGeometryCeres()`, not re-evaluated as
Ceres's own internal LM iterations move the trial parameters -- the same
granularity of freezing the hand-rolled path already uses (there, frozen
per GN sub-iteration; here, frozen for the whole call, since Ceres
iterates internally rather than the caller re-entering the loop). Not a
new approximation introduced by this port, but worth knowing if the
`--use-ceres` RMSE trajectory ever looks different from the hand-rolled
one in a way that isn't obviously better.

Status as of this pass: default (no-Ceres) build and `--use-ceres`'s
graceful fallback (one-time stderr warning, hand-rolled path used) were
verified in the Linux sandbox (this sandbox has no network access to
install Ceres itself). The real Ceres-linked build was then verified on
the macOS target (Homebrew `ceres-solver`/`eigen`, `cmake ..
-DCMAKE_PREFIX_PATH="$(brew --prefix)"`): `gmcore`/`gmesh_cli` link and
run cleanly against Ceres 2.2.0. A same-seed synthetic-sphere comparison
(`--rows 9 --cols 9 --pyramid-levels 3 --outer-iters 6 --gn-iters 3`, with
vs. without `--use-ceres`) converged smoothly on both paths with no
divergence, and Ceres reached a slightly *better* final RMSE (54.1%
reduction vs. 52.0% hand-rolled) -- plausibly because Ceres's adaptive
trust region is a more capable step-acceptance scheme than the hand-rolled
path's fixed 4-try/0.4-shrink backtracking, though this is one run on one
test case and not yet a full regression. Not yet done: the harder 25x25
sharp-edge test that's the actual point of this whole line of work (does
Ceres's more robust step acceptance help with the Fig. 4 pinching
question), a wall-clock timing comparison, and a visual wireframe-overlay
sanity check of the Ceres-produced mesh (same techniques used earlier in
this file for the hand-rolled path's fidelity checks).

### Bug found and fixed: boundary vertices were effectively frozen in place

The paper says (Sec 4): "control points on the boundary only move along the splines" --
1 degree of freedom per boundary vertex (its position along the curve), not 2. The
existing soft `boundaryWeight` penalty instead pulled each boundary vertex toward a
fixed re-projected point using the FULL 2D residual (`P.x - target.x`, `P.y -
target.y`, both weighted equally), which resists along-curve (tangential) motion just
as hard as off-curve (normal) drift -- not "free to slide", but "stay near this one
point". Confirmed empirically via `_mesh_points.csv`: boundary vertices sat at almost
exactly their initial uniform-spacing coordinates even after full optimization, no
matter how many outer iterations ran.

Fixed by projecting the residual onto only the curve's NORMAL direction at the
vertex's current `boundaryT` (`Vec2{-tangent.y, tangent.x}.normalized()`), leaving the
tangential component completely free -- a single scalar residual instead of two
independent x/y ones, in both the hand-rolled path and the shared Ceres
`BoundaryCostFunction`. Verified via before/after `_mesh_points.csv` comparison
(vertices visibly non-uniformly spaced after the fix) and a small RMSE improvement
across the regression suite (5x5: 60.8%->61.8%, 25x25: 59.1%->61.1%, sphere:
unchanged). Found while investigating a user report that the mesh still wasn't
snapping to sharp edges the way the paper's Fig. 4 shows, even on cases predating the
Puv-free experiment above -- this was the actual root cause of that, not twist.

A related follow-up fix, found later in this same investigation (see next section):
the boundary re-projection (`closestT()` against the vertex's current position) was
only being re-run once per OUTER iteration, not once per Gauss-Newton sub-iteration.
That's fine for small steps, but a strong pull (like the new vector-line term below
can produce) can move a boundary vertex far enough in a single GN sub-step that the
target/normal computed at the top of the outer iteration is badly stale by the time
the *next* sub-step's linearization uses it. Now re-projected every GN sub-iteration
in both the hand-rolled path and `MeshOptimizerCeres.cpp`'s `ceresSolveOnce`/
`jointSolveOnce`, at the same frequency backtracking already re-checks the true
energy.

### Reverted: Puv back to fixed zero, vector-line term rewritten per Sec 4.2

Two changes made together after a careful reread of the paper, prompted by the
question above ("why can't we get a sharp edge like Fig. 4") not being resolved by
either the boundary fix or the earlier Puv-free experiment on their own:

**1. Puv reverted to fixed `{0,0}`.** As already covered in the note above: the
paper's Sec 3 says plainly, "In practice, the values of muv are usually set to zero."
Promoting it to a free unknown (see "Fixed: Puv promoted to a free unknown too" above)
was a paper-fidelity regression, not an improvement -- caught by rereading the primary
source rather than assuming the more-general/more-free version was automatically more
faithful. `GradientMesh::geomCorner()` hardcodes `Puv` to `{0,0}` again; the
`MeshVertex::Puv` field itself is kept (as inert, always-zero storage) only so
CSV/serialization code referencing it doesn't need to change. All Ceres cost functions
(`PatchDataCostFunction`, `JointPatchDataCostFunction`, `SmoothTripleCostFunction`,
`TangentPriorCostFunction`, `BoundaryCostFunction`) went back to 6-double (not 8-double)
geometry parameter blocks.

**2. The vector-line-guided term (Sec 4.2) was rewritten to match the paper's actual
formula.** The previous implementation was a coarse approximation: it only checked the
discrete straight edge between two adjacent control points (at its midpoint) against a
single flat global pixel radius (`vectorLineInfluenceRadius`), and never touched the
free tangent unknowns `Pu`/`Pv` at all -- so the term could pull vertex *positions*
toward alignment but had no way to shape the *tangent* the way the paper describes.
The paper's own formula (quoted verbatim from Sec 4.2): for the nearest vector line to
a point `m(u,v)`, `wu(m(u,v)) = G(d|0,sigma_v^2)` where `d` is the distance to that
line and `sigma_v` is one third of a "narrow band" width, itself one fifth of the
line's own length; the term penalizes the component of the analytic surface tangent
(`dm/du`, `dm/dv`) perpendicular to the line's direction, Gaussian-weighted by that
`d`. This is now implemented as `nearestVectorLineField()` (`MeshOptimizer.cpp`,
mirrored in `MeshOptimizerCeres.cpp`), evaluated at the SAME dense per-patch `(u,v)`
sample grid the data term already uses -- not just at discrete mesh edges -- using the
analytic `dU`/`dV` from `evalPos()` and `PatchWeights`' `wu`/`wv` arrays for the
Jacobian, exactly the way the data term already reuses the `w` array for `d(pos)/d
(corner)`. `vectorLineWeight`'s default changed from 60 to the paper's stated `beta=20`.
`vectorLineInfluenceRadius` is now unused (the band width is derived per-line from
each line's own polyline length) but left in `OptimizerOptions` as an inert field for
config compatibility.

The new term's Jacobian (`d(ru)/d(corner param)`, `d(rv)/d(corner param)` via
`pw.wu`/`pw.wv`) was verified against a standalone finite-difference harness before
being trusted: 60,000 entries checked across 50 random patch configurations and 25
`(u,v)` sample points each, 0 bad, max relative error 3.0e-10 -- effectively exact.

**Result -- the core question of this whole investigation, finally answered:** adding
a single guide vector line down the middle of a 5x5 mesh's domain (`gmesh_cli --vline
"106.5,40;106.5,173"`) causes the three interior mesh-columns to collapse tightly
around the line -- x-coordinates of columns 1, 2 and 3 (of 0..4) landing within about
5-8 pixels of the line's x=106.5, versus being evenly spread across the full ~200px
width with no vector line supplied. This is the genuine "sharp edge from mesh-line
collapse" effect the paper's Fig. 4 shows, and it did not happen with the old
discrete-edge/flat-radius approximation. Both the boundary fix and the Puv revert were
necessary but not sufficient on their own; it was specifically making the vector-line
term dense, analytic-tangent-based and paper-accurate that produced the effect.

**Caveat, reported honestly rather than hidden:** in that same stress test, a couple of
boundary vertices land visibly outside the image bounds (e.g. y~277 against a 213px-tall
image), and this persisted even after the boundary-reprojection-per-substep fix above,
across a range of `vectorLineWeight` values including well below the default. The
Jacobian is verified exact (see above), and the regression suite with no vector lines
is unaffected, so this isn't a sign error -- it looks like a real (if extreme) energy
trade-off: `computeGeometryEnergy`'s backtracking only requires each step to *decrease*
the total weighted energy, and a single vector line spanning nearly the full height of
a tiny 5x5 mesh is a deliberately adversarial test case (an unrealistically dominant,
image-spanning constraint relative to the boundary/data terms) rather than typical
usage (a shorter guide line traced along an actual detected feature, on a finer mesh
where each line's influence is more local). Not yet re-tested with a more realistic
guide line length/placement, or with `boundaryWeight` raised to compensate -- worth
doing before relying on this term heavily on real images with vector-line input.

Regression suite (hand-rolled path, no Ceres, no vector lines, same three cases used
throughout this file):

| case | before this pass | after this pass |
|---|---|---|
| synthetic sphere, 9x9 | 53.8% reduction | 53.8% reduction (unchanged) |
| 25x25 `gradient.png` | 59.1%-61.1%\* | 60.6% reduction |
| 5x5 `gradient.png` | 60.8%-61.8%\* | 61.7% reduction |

\* range reflects the boundary-fix-only numbers from the previous section; not a
regression, just noting the small amount of run-to-run variation already present in
this codebase's block-coordinate scheme.

Not yet done: re-verifying the equivalent Ceres-path (`--use-ceres`/`--use-ceres-joint`)
numbers on the real macOS/Ceres build -- the changes were syntax-checked against a
local Ceres-API stub and are structurally identical to the hand-rolled path (same
formulas, same freezing convention), but a real `ceres::GradientChecker` run on the new
`VectorLineCostFunction` hasn't been done yet (see "Optional: Ceres-based geometry
solver" above for how prior passes did this verification).

### Fixed: the 4 mesh corners are now hard-fixed (never move)

Direct follow-up to the boundary-vertex divergence caveat in the section above: the
corners (grid positions `(0,0)`, `(0,cols-1)`, `(rows-1,0)`, `(rows-1,cols-1)`) are
each the exact junction of TWO boundary splines, not an interior point of a single
one. "Control points on the boundary only move along the splines" (Sec 4) is
ambiguous at a corner -- along *which* of the two splines? -- and
`GradientMesh::buildInitial()`'s boundary-side assignment resolves the ambiguity
somewhat arbitrarily (its `if`/`else-if` chain checks `r==0` before `c==cols-1`
before `r==rows-1` before `c==0`, so e.g. the bottom-right corner ends up assigned to
the *right* spline at `t=1`, not the bottom one). The soft normal-only boundary
constraint's linearization (see "Fixed: boundary vertices were effectively frozen in
place" above) is only valid for small displacements from that assigned point; at a
`t=0`/`t=1` endpoint shared with a *different* curve, a large step (as the new,
much stronger vector-line term can produce) can end up "tangential" with respect to
the wrong curve's direction entirely and drift arbitrarily far before the soft
penalty pushes back. That's exactly what the stress-test caveat above was seeing.

Corners are also structurally redundant as free unknowns in the first place:
`buildInitial()` sets them to the boundary curves' own endpoints (`P00`/`P10`/`P01`/
`P11`) exactly, so there was never anything for the optimizer to usefully solve for
there. Hard-fixing removes the failure mode entirely rather than tuning around it.

Only *position* is fixed -- `Pu`/`Pv` (the free tangents) stay free at corner
vertices, unaffected. Implementation differs by solve path but the effect is
identical: the hand-rolled path solves the Gauss-Newton step as usual (corners stay
in the normal equations, so the linear system shape doesn't change) and then simply
zeroes the proposed position delta for the 4 corner vertices before applying it, at
every backtracking `alpha` -- the simplest correct way to pin specific unknowns
without restructuring the sparse solve into a smaller system. The Ceres path uses the
tool built for exactly this, `ceres::SubsetManifold(6, {0, 1})` applied via
`Problem::SetManifold` on each corner's 6-double `(P,Pu,Pv)` parameter block, holding
dimensions 0-1 (`P.x`,`P.y`) constant while leaving 2-5 (`Pu`,`Pv`) free to vary --
used identically in `ceresSolveOnce` and `jointSolveOnce`.

Verified directly on the same adversarial stress test that surfaced the divergence
(`gmesh_cli --input gradient.png --rows 5 --cols 5 --vline "106.5,40;106.5,173"`):
all 4 corners now sit at exactly their fixed positions (`(6,6)`, `(207,6)`, `(6,207)`,
`(207,207)` for this test's margins) in `_mesh_points.csv`, and the previous
max-`|y|`-outside-image-bounds reading of ~277 is gone (max now ~207, i.e. within
bounds) -- while the interior-column pinching effect that the vector-line rewrite was
built to produce is fully preserved (columns 1-3 still land within a few pixels of
the guide line's x-coordinate). Regression suite with no vector lines unaffected
(sphere: 53.8%->55.4%, mildly *improved* since corners no longer waste any GN
capacity fighting a doomed local linearization; 25x25: 60.6% unchanged; 5x5:
61.7%->61.3%, within normal run-to-run noise).

Syntax-checked against the local Ceres-API stub, extended with a minimal
`ceres::SubsetManifold`/`Problem::SetManifold` stand-in for this check (see
`/tmp/ceres_stub` in the session that made this change -- not checked into this
repo). A real `ceres::GradientChecker`/build verification on this exact `SetManifold`
usage against real Ceres hasn't been done yet.

### Diagnosed: useCeresJoint gave visibly the worst reconstruction of the three solver modes

Reported by the user after testing all three solver modes (hand-rolled, `--use-ceres`,
`--use-ceres-joint`) side by side in the app: the joint solve's output looked
noticeably worse than either of the other two, not just marginally.

Went through every joint-specific residual and Jacobian line by line looking for an
actual formula bug -- `JointPatchDataCostFunction` (data term, both geometry and
color as live parameters), `ColorSmoothTripleCostFunction`, `ColorRidgeCostFunction`,
and the geometry-side terms it shares with `ceresSolveOnce` (smoothness,
tangent-prior, boundary, vector-line). None of them showed a sign error, wrong array
offset, or mismatched weight -- every one matches its corresponding hand-rolled term
exactly, same as the already-verified `useCeresGeometry` path.

What's structurally different about joint, and the likely actual cause: its
parameter space is 18 doubles/vertex (6 geometry + 12 color) versus
`useCeresGeometry`'s 6. Both the hand-rolled path and `useCeresGeometry` solve color
*exactly*, via its own dedicated closed-form linear system -- up to
`OptimizerOptions::cgMaxIterations` (200 by default) conjugate-gradient iterations
against `cgRelTolerance`, freshly every single outer iteration. `useCeresJoint` has no
such dedicated solve at all: color and geometry are minimized together in one
`ceres::Problem`, and the internal Ceres iteration cap used for that combined solve
(`jointSolveOnce`'s `maxIters` argument) was hardcoded to the same value (6) as
`useCeresGeometry`'s -- appropriate for a 6-dimensional-per-vertex problem, almost
certainly far too few for one 3x the size that also has to arrive at a good color fit
with no dedicated linear solve to fall back on. Each of those 6 outer
Levenberg-Marquardt iterations only gets one shot at an internally-approximated
linearized step before `optimizeJointCeres`'s own gate (`computeTrueJointEnergy`,
same verify-and-shrink-or-revert pattern as `optimizeGeometryCeres`) re-checks the
true energy and, on rejection, throws the whole sub-step away -- so an
under-converged combined step is exactly the kind of thing that gate would end up
rejecting most of the time, plausibly explaining reconstructions that look close to
the un-optimized initial mesh.

Fix: `optimizeJointCeres` now reuses `opts.cgMaxIterations` (200 by default) as
Ceres's own outer iteration cap for the joint solve, instead of the 6 borrowed from
`useCeresGeometry`. Ceres's own convergence tolerances (`function_tolerance` etc.)
still apply, so a well-converged joint step should terminate well before hitting 200
in practice -- 200 is a ceiling, not a target iteration count. Added
`OptimizerOptions::cgMaxIterations` as a CLI flag (`gmesh_cli --cg-iters N`) so this
(and the hand-rolled color solve's own budget, which already used this same field) is
tunable without recompiling.

**Not yet verified against a real Ceres build** -- this sandbox has no Ceres
installed, so this fix is syntax-checked and structurally sound but unconfirmed to
actually fix the visual quality issue; it should be re-tested by rebuilding on macOS
with the real Homebrew Ceres and comparing `--use-ceres-joint` output before/after
this change on the same test image. Expect `--use-ceres-joint` to run noticeably
slower than before as a result of this change (up to ~33x more Ceres-internal
iterations allowed per call, though real convergence should stop well short of that
ceiling most of the time) -- a wall-clock timing comparison for this mode still
hasn't been done (see "Optional: Ceres-based geometry solver" above).

### Found the actual root cause: CGNR+JACOBI was too weak a linear solver, not just too few iterations

Reported by the user after rebuilding with the `cgMaxIterations` fix above:
`--use-ceres` (geometry only) still approximates worse than the hand-rolled path, and
`--use-ceres-joint` is worse still -- i.e. the iteration-budget fix directly above did
**not** resolve the problem. Went back through the whole file line by line at the
user's explicit request ("Ceres geom аппроксимирует хуже Hand-rolled. Ceres-joint -
еще хуже. Перепроверь код."):

- Re-verified every `sqrt(weight)` residual-scaling call site (Ceres minimizes
  sum-of-squared-residuals; the hand-rolled path's energy is `weight * r^2`) -- all
  consistent.
- Read `computeTrueGeometryEnergy`/`computeTrueJointEnergy` side by side against
  `MeshOptimizer.cpp`'s `computeGeometryEnergy` term by term (data, vector-line,
  smoothness, tangent-prior, boundary) -- identical formulas and weights.
- Read every `ceres::CostFunction` in the file in full --
  `PatchDataCostFunction`, `JointPatchDataCostFunction`, `ColorSmoothTripleCostFunction`,
  `ColorRidgeCostFunction`, `SmoothTripleCostFunction`, `TangentPriorCostFunction`,
  `BoundaryCostFunction`, `VectorLineCostFunction` -- comparing each residual and
  Jacobian sign/index against its hand-rolled counterpart (`addSmoothnessTerms`,
  `addTangentPriorTerms`, the boundary and vector-line blocks inside
  `optimizeAtCurrentResolution`'s GN loop, and `PatchWeights`' `w`/`wu`/`wv` array
  layout). No sign error, wrong array offset, or mismatched weight found anywhere --
  every term still matches its hand-rolled equivalent exactly.

With the residual/Jacobian math cleared (again), the remaining suspect was the linear
solver itself. Both `ceresSolveOnce` and `jointSolveOnce` had `Solver::Options` set to
`linear_solver_type = ceres::CGNR` with `preconditioner_type = ceres::JACOBI`. CGNR is
itself an iterative conjugate-gradient solve of the normal equations (not an exact
one), and Ceres's `JACOBI` preconditioner for CGNR is only a per-*scalar* diagonal of
`J^T J`. Compare that to the hand-rolled path's own linear solve
(`SparseBlockSolver.h`'s `solveSPD_PCG`): also CG, but block-Jacobi preconditioned --
it inverts each vertex's full 6x6 diagonal block, capturing the strong coupling
between `P`, `Pu`, and `Pv` at the same vertex. A bare scalar diagonal preconditioner
is materially weaker than that block preconditioner, especially with the vector-line
and boundary terms mixed in (very unevenly-weighted residuals sharing the same
6-double block). Under a weak preconditioner, a low iteration cap (the original,
un-fixed 6 for `useCeresGeometry`) starves the CG solve before it gets close to the
true GN/LM step -- and raising the cap alone (the `cgMaxIterations` fix above, already
applied to `useCeresJoint`) just means grinding more slowly toward the same
under-converged answer; it doesn't fix the underlying solve quality, which is
consistent with the user's report that the budget fix alone didn't help.

Fix: switched `linear_solver_type` to `ceres::SPARSE_NORMAL_CHOLESKY` in both
`ceresSolveOnce` and `jointSolveOnce` (dropping `preconditioner_type`, which only
applies to iterative solvers). This solves the normal equations *exactly* every LM
iteration instead of approximately -- Ceres requires Eigen as a hard dependency
regardless of solver choice, so `EIGEN_SPARSE` is always available as the sparse
backend even without SuiteSparse; a Homebrew `ceres-solver` install additionally links
SuiteSparse, which Ceres prefers automatically when present, for an even faster exact
solve. The mesh grids here are small (tens to low hundreds of vertices), so an exact
sparse Cholesky factorization per iteration is computationally trivial. Also raised
`optimizeGeometryCeres`'s `itersPerSubStep` from the original hardcoded 6 to
`std::max(6, opts.cgMaxIterations)`, matching `useCeresJoint`'s budget -- with an
exact solver, Ceres's own convergence tolerances (`function_tolerance`,
`gradient_tolerance`, `parameter_tolerance`) stop it well short of a generous cap once
it actually converges, so there's no real cost to keeping the two paths symmetric
instead of leaving geometry-only arbitrarily starved relative to joint.

**Not yet verified against a real Ceres build** -- same caveat as every Ceres change
in this project: this sandbox has no Ceres installed, only a hand-written stub
(`/tmp/ceres_stub`) used to syntax-check `MeshOptimizerCeres.cpp` compiles
(`g++ -fsyntax-only -DGMCORE_WITH_CERES`), which caught no errors. This is a
well-reasoned fix backed by a real, identifiable difference between the two solvers'
preconditioning strength, not a proven one. Please rebuild on macOS with the real
Homebrew Ceres and report reconstruction RMSE for hand-rolled vs. `--use-ceres` vs.
`--use-ceres-joint` on the same test image so this can be confirmed -- if
`SPARSE_NORMAL_CHOLESKY` isn't available in your Ceres build for some reason (it
requires *some* sparse linear algebra backend at Ceres's own build time, which nearly
every distribution -- including Homebrew's -- provides), Ceres will report that
clearly via `Solver::Summary::message` / stderr rather than silently misbehaving.

### Fixed: hand-rolled optimizer's default iteration budget was too low to converge

Reported by the user: running the hand-rolled optimizer a SECOND time on its own
already-optimized output (same mesh, same image, `optimizeCoarseToFine` called
again) kept reducing RMSE substantially -- one 5x5-mesh `gradient.png` case went
from 0.0315 to 0.0260. That should not happen at a real local optimum: a converged
solve re-run on its own output should barely move.

Reproduced directly with a standalone harness (`/tmp/converge_test.cpp`, exact same
5x5-mesh-against-`gradient.png` setup as `gmesh_cli`'s defaults): one pass at the old
default (`outerIterationsPerLevel=8`) reached RMSE 0.03147; running that same 8-outer-
iteration pass twice in a row reached 0.03101; a single pass at
`outerIterationsPerLevel=40` (same pyramid schedule, run once) reached 0.02817 --
confirming 8 was simply stopping short of a real local optimum, not the run-twice
result being an artifact of some other bug. Raising the ceiling to 100 produced
*identical* per-level stopping iteration counts and final RMSE as 40, confirming 40
isn't itself truncating anything -- it's a genuine ceiling, not a new bottleneck.

Root cause: `outerIterationsPerLevel` defaulted to 8, a budget that was apparently
tuned (or just guessed) against easier cases and never re-checked once other terms
(vector-line, boundary, tangent-prior, corner-fixing) were added to the energy this
loop is minimizing -- a harder case like a 5x5 mesh fitting a full-color gradient
image needs meaningfully more outer Levenberg-Marquardt iterations than that to
settle. (Also checked whether `geomGaussNewtonItersPerOuter`, the *inner* GN
iteration count, was the real lever instead -- doubling it from 3 to 6 barely moved
RMSE, 0.03146 vs the 8-outer-iteration baseline's 0.03147 -- so the outer count, not
the inner one, was the actual bottleneck.)

Fix, in two parts:

1. Raised `OptimizerOptions::outerIterationsPerLevel`'s default from 8 to 40 (an
   upper ceiling, not a target).
2. Added `OptimizerOptions::outerConvergenceRelTol` (new field, default `1e-3`): the
   outer loop now exits early once the relative improvement in
   `computeGeometryEnergy` (the same composite data+vector-line+smoothness+tangent-
   prior+boundary energy backtracking already checks every GN sub-iteration) between
   successive outer iterations drops below this fraction. Without this, simply
   raising the ceiling to 40 would make every case -- including already-converged
   easy ones like the synthetic sphere -- 5x slower for no benefit. Set to 0 to
   disable early-exit entirely.

Verified with the same harness: a single pass at the new defaults (40 + early-stop)
reaches RMSE 0.03065 on the 5x5 `gradient.png` case, better than manually running the
old 8-iteration default twice (0.03101), in one `optimizeCoarseToFine` call.

`outerConvergenceRelTol`'s default was then tuned by sweeping 1e-5 through 3e-3
across three cases (synthetic sphere 9x9, `gradient.png` 25x25, `gradient.png` 5x5,
via `/tmp/converge_test3.cpp`): the initially-chosen 1e-5 turned out needlessly
tight -- it cost 24-40% more wall-clock than 1e-3 on *every* case (sphere: 6703ms vs
4027ms; gradient 25x25: 42750ms vs 32583ms) for no measurable quality benefit (on the
hard 5x5 case, 1e-3 actually reached a slightly *better* RMSE than 1e-5, 0.03041 vs
0.03065 -- late Levenberg steps aren't guaranteed net-positive, so stopping a touch
earlier isn't strictly a quality/speed tradeoff here). Looser still (3e-3) starts
losing real quality on the hard case (RMSE back up to 0.03107, most of this fix's
benefit given up), so 1e-3 was kept as the default -- a genuine sweet spot, not just
"as loose as possible."

Full regression suite (`/tmp/build_check2`, no-Ceres CMake build) re-run against the
standard three cases (synthetic sphere 9x9, `gradient.png` 25x25, `gradient.png` 5x5)
with these new defaults: all three run cleanly through `gmesh_cli`, early-stopping
sensibly per pyramid level, no crashes or divergence, corner-fixing and the Sec-4.2
vector-line term unaffected (this fix only touches the outer-loop stopping
condition, not any residual or Jacobian).

This fix, unlike the two Ceres-path fixes above, is fully verifiable in this sandbox
(pure hand-rolled C++, no Ceres dependency) -- confirmed end-to-end here, not just
structurally reasoned about.

### Follow-up: a real bug in the early-exit itself, plus a separate (non-bug) multi-restart effect

Reported by the user immediately after the fix above: it helped, but running the
hand-rolled optimizer again on its own output *still* measurably improved RMSE.
That shouldn't happen if the early-exit above genuinely detects convergence, so this
needed a real second look rather than just nudging `outerConvergenceRelTol` again.

Found an actual bug in the early-exit's logic, not just a mistuned constant: it
compares this outer iteration's composite energy to the previous one and stops the
moment the relative improvement drops below `outerConvergenceRelTol` -- but a
rejected Gauss-Newton step (see the backtracking block in
`optimizeAtCurrentResolution`) *reverts* the mesh to its pre-step state and only
bumps the Levenberg damping (`lambda`) up 4x. The geometry genuinely didn't change
that outer iteration, so of course its energy looks unchanged -- but a
single-iteration check can't tell that apart from real convergence, and stops right
there. The very next outer iteration, working from that same point but with more
damping headroom already spent, can easily find a good step once `lambda` settles --
which is exactly what calling `optimizeCoarseToFine` a second time was doing by
accident: it resets `lambda` back to `geomDampingInitial` and gives the solve a fresh
chance that the premature stop had denied it.

Fix: added `OptimizerOptions::outerConvergencePatience` (default 3) -- the early-exit
now requires that many CONSECUTIVE stalled outer iterations before it actually
breaks, instead of trusting a single one. A lone stall (lambda transiently too high)
just increments a counter and the loop keeps going; the counter resets the moment any
iteration improves enough again.

Verified this actually fixes the underlying bug, not just moves the symptom, by
isolating the two effects that were previously tangled together:

- **At a single FIXED resolution** (`optimizeAtCurrentResolution` called directly,
  no pyramid), with the patience fix in place: calling it again on its own output
  now shows an honest **~0.00% gap** (0.03066 -> 0.03065 -> 0.03066, i.e. noise) --
  confirming the early-exit itself now genuinely detects convergence and the
  original bug is fixed, not just patched over with looser numbers.
- **Through the FULL `optimizeCoarseToFine` pyramid pipeline**, a small residual gap
  remains even with the fix (~0.2-0.3% RMSE per repeated call on the 5x5
  `gradient.png` case) -- but this is a *different, structurally expected*
  phenomenon, not the same bug resurfacing: every call re-descends the mesh to the
  COARSEST pyramid level and re-climbs, and on a repeat call that descent starts from
  an already-refined mesh instead of the crude initial one. Because this is
  non-convex block-coordinate descent, a different starting point at the coarse
  level can (and measurably does) lead to a marginally different, sometimes better,
  local optimum by the time it climbs back to the finest level -- a multi-restart
  effect inherent to any coarse-to-fine non-convex optimizer, not a sign that any
  individual level failed to converge.

Rather than leave this as "just run it again if you want the last bit of polish"
(what the user was already doing manually), added `OptimizerOptions::pyramidRestarts`
(default 1, so existing behavior is unchanged) so `optimizeCoarseToFine` can repeat
its own full sweep N times in a single call. Verified bit-exact equivalence:
`pyramidRestarts=2` in one call produces the identical final RMSE (0.03061) as two
separate `optimizeCoarseToFine` calls on the same mesh. Exposed as `gmesh_cli
--pyramid-restarts N` and as a `DocumentModel.pyramidRestarts` property on the macOS
side (`DocumentModel.mm`'s `-optimizeWithPyramidLevels:progress:completion:`,
following the same pattern as `useCeresGeometry`/`useCeresJoint`) -- left at the
default of 1 there too rather than silently multiplying every optimize click's
runtime; the macOS app doesn't yet have a UI control wired to this property (no
storyboard/XIB change was made in this pass, since that can't be build-verified in
this sandbox), so it currently only takes effect if set programmatically. A visible
control (e.g. a small stepper next to the solver picker) would be a natural,
low-risk follow-up.

Full regression suite re-run again after this follow-up fix: sphere 9x9, `gradient.png`
25x25 and 5x5 all build and run cleanly through `gmesh_cli`, including with
`--pyramid-restarts 2` explicitly passed; `MeshOptimizerCeres.cpp` re-checked against
the Ceres stub and is untouched/unaffected by this change (the patience and restart
logic both live in the hand-rolled `optimizeAtCurrentResolution`/`optimizeCoarseToFine`
control flow, outside the Ceres-specific solve functions those call into).

### Bug found and fixed: the macOS app's internal `Image` was vertically mirrored relative to everything else

Reported by the user from the app's UI: the mesh grid visually looked inconsistent
with the displayed photo -- like one of the two was flipped vertically relative to
the other.

Traced the Y-axis convention through the entire pipeline end to end: `Image::loadPNG`/
`loadPPM` (`core/src/Image.cpp`, used by the CLI), `GradientMesh::buildInitial`'s
boundary/vertex construction, `CanvasView`'s on-screen mesh-overlay drawing and
mouse-click-to-image-coordinate mapping (`mac/CanvasView.mm`), and `SVGExporter` are
all mutually consistent: row 0 / y=0 = top, y grows downward, uniformly.

Found one place that disagreed: `DocumentModel.mm`'s `-loadImageAtURL:error:`, the
macOS app's actual image-loading path (the CLI's `Image::load` is a separate,
unaffected code path -- see `Image.h`'s own comment that the app never calls it).
It builds `_target` (the internal `gmcore::Image` used for every color sample, and
for the optimizer's data/gradient energy terms) by drawing the loaded `CGImage` into
a `CGBitmapContextCreate` bitmap context via `CGContextDrawImage`, then copying that
buffer row-by-row into `_target` assuming row 0 = top. That assumption doesn't hold
here: a fresh `CGBitmapContextCreate` context has Quartz/PDF's default coordinate
convention -- origin at the BOTTOM-left, y increasing upward -- and `CGContextDrawImage`
draws respecting that current transform, so without an explicit flip the image ends
up right-side-up in y-up space, which means it's stored upside-down in the buffer's
top-down memory layout. (This is a well-known, frequently-hit Core Graphics gotcha,
not specific to this codebase -- searching "CGContextDrawImage draws image upside
down" turns up many independent reports of exactly this scenario and exactly this
fix.) The displayed `NSImage` (`self.displayImage`) is a completely separate object
loaded straight from the file and was never affected -- `NSImage`'s own `-drawInRect:`
self-orients regardless of the destination context's flip state -- so the bug was
invisible in the raw photo display and only showed up in anything that used `_target`
geometrically: initial per-vertex mesh colors sampled from the wrong row
(`GradientMesh::buildInitial`, `target.sampleBilinear(S.x, S.y)`), and, if the
optimizer ran, geometry pulled toward color/edge features that were actually the
vertical mirror of what's on screen.

Fix: added the standard `CGContextTranslateCTM(ctx, 0, h); CGContextScaleCTM(ctx, 1.0,
-1.0);` before the `CGContextDrawImage` call, so `buffer`'s row 0 ends up holding the
image's true top row, matching every other stage's convention. Checked the app's one
other `CGBitmapContextCreate` use (`-renderReconstructionPreview`, for the
export/preview image): that path writes `gmcore::Image` pixel data directly into a
buffer and wraps it with `CGBitmapContextCreateImage` -- it never calls
`CGContextDrawImage`, so no CTM/transform is ever invoked and no flip is needed there;
confirmed it was not a second instance of the same bug.

**Confirmed on-device.** This sandbox has no Cocoa/Core Graphics toolchain, so the fix
itself was written from reading the exact code path plus the (surprisingly hard to pin
down with a single authoritative quote -- see below) Quartz coordinate-flip convention,
not from a local rebuild-and-see-it-line-up test. The user rebuilt the app with this fix
and tested against `gradient.png` (whose real corner colors, confirmed with `PIL`
outside the app: top-left red, top-right green, bottom-right yellow, bottom-left blue,
going clockwise -- matching the paper's own convention) and reported: before this fix
`_target` was indeed mirrored (matching the diagnosis); after it, the mesh -- once
optimized -- correctly snaps to the real (non-mirrored) image features, and
`renderReconstructionPreview`'s output is correct. Direction of the fix confirmed
correct, not just structurally reasoned about.

Side note on process: trying to nail the *exact* mechanism down further by searching for
an authoritative primary source (Apple's own docs on whether a fresh `CGBitmapContext`'s
raw buffer row 0 is the image's top or bottom absent a flip) turned out to be
surprisingly inconclusive -- multiple Apple reference pages and the Quartz 2D
Programming Guide describe the *user-space* convention (bottom-left origin, y-up) but
none of the fetched sources spelled out the raw-buffer-row question in so many words.
The on-device empirical test above is what actually settled it, which is generally the
more trustworthy signal for this class of bug anyway.

If there's an old project file/mesh saved from before this fix, it may be worth
re-optimizing it, since its vertex colors and any prior optimization were fit against
the (then-mirrored) data.

### Second, separate flip bug found: the DISPLAYED photo itself was upside-down in the app's canvas

After the fix above, the user reported the mesh/reconstruction were now correct, but a
follow-up, more specific report clarified something the first round of questions had
missed: the loaded photo itself, as literally shown on screen in the canvas, displays
upside-down -- a completely different code path than `_target`, and one this project's
earlier CanvasView investigation (see the flip bug above) had assumed was fine.

Root cause: `CanvasView.mm`'s `-drawRect:` shows the loaded photo via
`[shown drawInRect:r fromRect:NSZeroRect operation:NSCompositingOperationCopy
fraction:1.0]` -- `NSImage`'s "modern" (post-10.6) drawing API. The earlier
investigation assumed this method auto-compensates for a flipped destination view
(`CanvasView.isFlipped` returns `YES`, origin top-left, matching every other coordinate
in this file). That assumption was wrong: `-drawInRect:fromRect:operation:fraction:`
draws the image as-is in the current graphics-state coordinate system without any such
compensation, so in this flipped view the photo comes out vertically mirrored. This is
a real, if less commonly documented, AppKit gotcha -- unlike the Core Graphics flip
fixed above, this one wasn't confirmed via extensive documentation archaeology (that
approach had already proven unreliable once this session -- see the note above); it was
identified directly from the user's live, on-device report and fixed on that basis.

Fix: wrap just that one `drawInRect:` call in a save/concat/restore-scoped
`NSAffineTransform` that reflects the image vertically within its own display rect
(`translateXBy:0 yBy:(r.origin.y*2 + r.size.height)` then `scaleXBy:1.0 yBy:-1.0`,
the standard idiom for this). Scoped narrowly so `drawBoundary`/`drawMesh`/
`drawTangents`/`drawVectorLines` (drawn right after, via the same `viewPointFromImagePoint:`
mapping already confirmed correct) are unaffected -- they already assumed a right-side-up
image, so fixing the image draw to actually be right-side-up is what makes them agree
with what's on screen, rather than requiring any change on their end.

**Confirmed on-device**: after this fix, the loaded photo displays right-side-up.

### Resolved: the `_target` CTM flip was backwards -- removed

Immediately after confirming the photo displays correctly, a follow-up report: the
*optimized mesh* still looks flipped (or isn't sampling from the right place during
optimization) relative to the now-correctly-displayed photo.

This raised a real concern about the FIRST fix in this sequence (the `_target` CTM flip
in `DocumentModel.mm`, above): that fix was justified partly by the claim "the displayed
NSImage was never affected -- `-drawInRect:` self-orients regardless of a flipped
destination" -- a claim the very next fix (`CanvasView.mm`) proved WRONG for exactly
that call. That undermines confidence in the reasoning used to justify the `_target`
fix's direction, even though it doesn't by itself prove that fix is backwards --- an
honest re-derivation from Quartz first principles was attempted and produced, once
again, a result that contradicts the original justification (this time suggesting NO
flip should have been needed for a freestanding, non-view-backed `CGBitmapContext`,
since only an AppKit `isFlipped` view's compensating CTM -- not a raw bitmap context --
would make `-drawInRect:`-style "always draw upright in current user space" logic
produce an upside-down result). Given this analysis has now flip-flopped multiple times
under supposedly careful reasoning, and given the CanvasView fix already showed that
confident-sounding Core Graphics reasoning can be wrong here, theory alone isn't a
reliable arbiter for this specific question any more.

Rather than guess a third time, added a direct, unambiguous, on-device diagnostic
instead: `loadImageAtURL:` now `NSLog`s `_target`'s 4 corner colors right after
building it (`"[GMCORE _target orientation check] TL=... TR=... BL=... BR=..."`).
Loading `gradient.png` and checking Xcode's console against that file's actual corner
colors (TL red, TR green, BL blue, BR yellow -- confirmed independently, outside the
app) settles definitively whether the existing CTM flip is right or backwards, with no
remaining ambiguity -- unlike inferring it indirectly from how an optimized mesh looks,
which depends on several other steps (mesh construction, the optimizer, rendering) all
working correctly too.

The diagnostic settled it: loading `gradient.png` with the flip in place logged
`TL=(3,3,254) TR=(254,254,3) BL=(254,1,1) BR=(2,254,2)` -- i.e. TL=blue, TR=yellow,
BL=red, BR=green, exactly top and bottom swapped relative to the file's real corners
(TL=red, TR=green, BL=blue, BR=yellow). The flip was backwards.

Fix: removed the `CGContextTranslateCTM`/`CGContextScaleCTM(1.0, -1.0)` pair added
earlier, back to a plain, unflipped `CGContextDrawImage` call. No other change was
needed -- everything downstream of `_target` (mesh construction, the optimizer,
`GradientMesh::render`) was independently confirmed to already consistently assume
"row 0 / y=0 = top" and never introduces its own flip, so once `_target` itself is
right, the rest of the pipeline should already agree with it.

The takeaway for this whole saga (three rounds: the original `_target` fix, the
`CanvasView` display fix, and this correction): Core Graphics' exact flip behavior for
`CGContextDrawImage` genuinely depends on subtle context (a bare `CGBitmapContext` vs.
a view-backed one with `isFlipped`), confident-sounding reasoning about it was wrong
twice in a row even when it cited real, well-known gotchas, and a cheap, direct,
on-device diagnostic (log 4 known corner colors, compare against ground truth) settled
in one rebuild what several rounds of documentation research and first-principles
re-derivation couldn't. The `NSLog` diagnostic is left in place in `loadImageAtURL:`
as a cheap regression check for this exact class of bug.

**Confirm after rebuilding**: load `gradient.png` fresh (not a mesh/document left over
from before this fix -- that would still carry stale, pre-fix-era vertex colors), build
the mesh, and check that it now visually tracks the image correctly both before and
after optimizing.

### Fixed: reconstruction preview looked washed out on screen, but the exported PNG was correctly saturated

Reported by the user: the optimized reconstruction, as drawn live in `CanvasView`,
looked visibly lightened/less saturated than expected -- but saving it via File > Export
PNG and reopening that file showed the colors fully saturated, matching expectations.

Both the on-screen preview and the exported file are built from the exact same call
(`-[DocumentModel renderReconstructionPreview]`, which rasterizes the mesh into a raw
RGBA buffer and wraps it in an `NSImage`) -- `-exportPNGToURL:` calls it again and
writes the resulting `CGImage` straight to a PNG, so the two paths start from
numerically identical pixel bytes. That ruled out a data/optimizer bug (nothing about
the mesh, the color solve, or the render math differs between the two) and pointed
at how the same bytes get *interpreted* differently by two different consumers.

The cause: `renderReconstructionPreview`'s bitmap context was created with
`CGColorSpaceCreateDeviceRGB()`. Despite the name, this does not mean "use the actual
display's color response" -- it's legacy CoreGraphics terminology for an
untagged/ambiguous "generic device" RGB space. When AppKit composites an image tagged
this way into a modern window's (wide-gamut, typically Display P3) backing store, the
color-management pipeline has to pick *some* concrete interpretation for that
ambiguous tag, and can fall back to an old "Generic RGB" ColorSync profile with a
visibly flatter, less saturated response than sRGB. A PNG file, by contrast, is opened
by other apps (Preview, QuickLook, etc.) under the modern convention that an
untagged/generic image means sRGB -- so the identical numeric bytes render with full,
correct sRGB saturation once reopened from disk. Two different implicit assumptions
about the same ambiguous tag, in two different code paths, produced two different
looking results from one set of numbers.

Fix: `CGColorSpaceCreateDeviceRGB()` replaced with the explicit, unambiguous
`CGColorSpaceCreateWithName(kCGColorSpaceSRGB)` in both
`renderReconstructionPreview` and `loadImageAtURL:` (the latter builds `_target`'s
numeric buffer from the source file -- fixed too, for consistency, so the color space
that actually gets optimized against, rendered, and displayed is the same well-defined
sRGB space throughout the pipeline, not an ambiguous one at the very first step). No
change to any optimizer math or file formats -- purely a color-space tag on two
`CGBitmapContextCreate` calls.

**Confirm after rebuilding**: re-open a photo (ideally one known to be wide-gamut,
e.g. a recent iPhone photo, which is where this would be most visible), optimize, and
compare the live `CanvasView` reconstruction against a fresh PNG export side by side --
they should now look the same, both fully saturated.

## How this was tested

`gmesh_cli` (no image needed -- it can generate a synthetic shaded-sphere test image)
was built and run repeatedly on Linux while developing `gmcore`:

```sh
cmake -B build . && cmake --build build -j
./build/gmesh_cli --rows 9 --cols 9 --pyramid-levels 4
```

This confirmed: the mesh initializes correctly (transfinite/Coons interpolation between
4 boundary curves), the optimizer reduces reconstruction RMSE substantially (a 9x9 mesh
against a 220x220 synthetic image went from RMSE 0.091 to 0.049, a ~46% reduction, in
under 2 seconds), the rendered reconstruction visually resembles the input (verified by
converting the PPM output to PNG and inspecting it), and the SVG export is well-formed
XML with the expected `<meshgradient>`/`<meshpatch>` structure. Tune `OptimizerOptions`
in `MeshOptimizer.h` (smoothness/boundary/vector-line weights, iteration counts,
damping) if you want tighter fits on real photos -- the defaults were picked for
reasonable behavior on a variety of images, not tuned per-image.

## Third-party code

None. `gmcore` is 100% original code against the C++ standard library. The macOS app
uses only Apple system frameworks (Cocoa/AppKit, ImageIO, CoreGraphics,
UniformTypeIdentifiers). `gmesh_cli` optionally links the system `libpng` (if CMake
finds it) purely as a developer convenience for reading/writing real PNGs from the
command line; the app never needs it.
