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

* ~~Twist (`Puv`) is derived, not free.~~ **No longer a simplification -- fixed.**
  Position `P`, tangents `Pu`, `Pv` AND the twist `Puv` are now all free per-vertex
  unknowns, jointly refined by the geometry Gauss-Newton step (see "Fixed: Pu/Pv
  promoted to free unknowns" and "Fixed: Puv promoted to a free unknown too" below),
  exactly the 4-value-per-corner Hermite corner the paper describes.
  `Puv` used to be computed from neighboring positions via centered finite differences
  (Catmull-Rom style) instead of being a free unknown; that gap is closed. Color keeps
  the full independent `(C, Cu, Cv, Cuv)` unknown set, which is what actually gives a
  gradient mesh its shading expressiveness -- geometry now matches that same shape.
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
