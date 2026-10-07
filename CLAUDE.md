# CLAUDE.md

Guidance for Claude Code when working in this repository. Read this first; the long history is in
`docs/DEVELOPMENT_LOG.md` (138 KB — grep it by heading, do not load it whole).

## What this project is

**Gradient Mesh Studio**: a from-scratch C++17 implementation of Sun, Liang, Wen & Shum,
*"Image Vectorization using Optimized Gradient Meshes"* (ACM TOG 26(3), SIGGRAPH 2007), plus an
interactive macOS app (AppKit, Objective-C++). The goal is **maximum fidelity to the paper**; every
deviation is documented in the development log under "Known simplifications vs. the paper".

| Path | What | Builds where |
|---|---|---|
| `core/` (`gmcore`) | Portable algorithm library: mesh, optimiser, Bézier fitting, Lazy-Snapping segmentation, SVG export | macOS and Linux |
| `core/tests/test_main.cpp` | `gmcore_tests`, dependency-free regression suite | macOS and Linux |
| `cli/main_cli.cpp` | `gmesh_cli` command-line harness | macOS and Linux |
| `mac/*.mm`, `mac/*.h` | The AppKit app (`DocumentModel`, `MainWindowController`, `CanvasView`, `GLReconstructionView`) | macOS only (Xcode toolchain) |
| `core/src/MeshOptimizerCeres.cpp` | Optional Ceres solvers | Only if CMake finds Ceres (an empty translation unit otherwise) |
| `spike/` | Ceres experiments and probes | n/a |

## Working with the user

- The user (Vladimir) writes in **Russian**. Reply in Russian; keep code, comments, commit messages
  and docs in English.
- **Standing priority order** (his words: *"порядок 1 - regression-тесты. 2 - Точность по статье.
  3 - Настоящий инструмент выделения. Делаем, отлаживаем и комитим по одной таске за раз."*):
  1. regression tests, 2. accuracy against the paper, 3. a real selection tool. **One task at a
  time: implement, debug, commit.** Items 1–3 are done; the user has since been steering work on
  "Animate Mesh" directly. Do not start unrequested side-quests.
- You run on his MacBook and can build; but **you cannot see the running app**. Visual judgement (is the
  animation smooth? does the preview look right?) is his, usually reported with exported debug JSON.
  **When what he sees contradicts your analysis, believe the observation** and build an experiment that settles it
  (see the debug toggle below) instead of arguing from numeric proxies.
- Be honest about what was and was not verified. Say so in the reply and in the commit message.

## Environment, build and test

This project runs in **Claude Code on the user's MacBook**, which has Xcode. Confirm at the start of a
session with `xcodebuild -version` and `cmake --version`. (Earlier work was done in a Linux sandbox
with no Xcode, which is why much of the history says "unverified until a real Xcode build" and why
the log describes a "literal extraction" test method. **That limitation no longer applies here:
compile every `mac/` change.**)

```sh
# library, CLI and regression tests
cmake -B build . && cmake --build build -j --target gmcore_tests gmesh_cli
./build/gmcore_tests        # expect: ALL TESTS PASSED (20 cases / 3387 checks at the time of writing)
./build/gmesh_cli --input gradient.png --rows 9 --cols 9 --pyramid-levels 4 --margin 0 --out-prefix demo

# the macOS app (CMake generates the Xcode project; app is only configured on Apple platforms)
cmake -B build . && cmake --build build --config Release --target GradientMeshStudio
open build/GradientMeshStudio.app
# or: cmake -G Xcode -B build_xcode . && open build_xcode/GradientMeshStudio.xcodeproj   (⌘R)
```

(The app commands were verified by the user in Xcode; the `cmake --build ... --target GradientMeshStudio`
line has not been run from a Claude Code session yet — if it needs adjusting, fix this file.)

- Ceres is optional. If it is installed (e.g. via Homebrew), CMake prints "Ceres found" and the Ceres
  solver modes become available; otherwise `MeshOptimizerCeres.cpp` compiles to nothing.
- **Run `gmcore_tests` after every `core/` change** and add a test for new core behaviour. Also rebuild
  `gmesh_cli` and the app: `gmesh_cli` silently stopped compiling once (`buildInitial` changed to take
  `BezierSpline`) because nothing builds it automatically. There is no CI yet; adding one would be a
  good task if asked.
- For fast numeric experiments on Animate Mesh logic, it is still handy to extract the pure C++ part of
  `-startMeshAnimationWithRedraw:` into a standalone harness compiled against `core/include` (copy the
  exact text with a script, never retype it) — but a real Xcode build is now the source of truth.
- When patching with scripts, use `assert text.count(old) == 1` per replacement so a stale anchor fails
  loudly. Put the full explanation (root cause, why this fix, what was measured) **in the code comment**
  of the shipped file, not only in a script or commit message.

## Git

Repo root is the project folder (on the user's Mac: `~/Downloads/GradientMeshStudio`). Local branch
`master`; GitHub remote `origin` is `https://github.com/vladimir-vasilyev/gradient-mesh` with its branch
`main` (`git push origin master:main`). One task per commit; the message explains *why* and states what
was and was not verified; follow the attribution trailer the harness gives. **Do not push unless asked.**
Never commit generated output (`build/`, `build_xcode/`, `out/`, `DebugOut/` are ignored or untracked).

## Code conventions

- C++17; `gmcore` has **no required third-party dependencies** (libpng and Ceres are optional,
  detected by CMake). Do not add dependencies to `core/`.
- Comments are long and explain *why* and the history of the decision; keep that style, and cite
  paper sections (e.g. "Sec. 4"). Preserve existing paper-fidelity decisions: geometry twist `Puv` is
  fixed at `{0,0}` (paper Sec. 3); `Pu`/`Pv` are free unknowns; the four mesh corners are hard-fixed;
  boundary vertices slide along their spline; each mesh side is a `BezierSpline` (one or more cubic
  segments), not a single cubic.
- Do not quote numbers from memory or from old docs: re-run and use real output.

## Animate Mesh (the active area; `mac/DocumentModel.mm`, `-startMeshAnimationWithRedraw:`)

A cosmetic, non-destructive wiggle of the fitted mesh (styles: Jitter, Wave, Breathing, Squash &
Stretch). The full loop (~377 frames, periodic) is **precomputed once** before playback. Per frame:

1. **Continuous damping**: per-vertex clearance `margin` (`gmMeshComputeVertexClearanceMargins`)
   → `damping = smoothstep((margin − minClearance) / transitionWidth)` scales displacement.
2. **Exact checker + fallback**: `gmMeshHasCurvedSelfIntersection` (zero false positives); on a hit,
   conflict-set growth + shared-λ bisection, scoped to the involved vertices.
3. **Post-process temporal smoothing** (after all frames exist): circular moving average,
   `kAnimSmoothingHalfWindow = 8` (17-tap), boundary vertices pinned, every smoothed frame
   re-verified by the exact checker and reverted to the original if it would introduce a violation.

Lessons already paid for:

- The "jerky, not smooth" motion came from `margin` being a **min() over several smooth distances**:
  its derivative has corners where the closest edge switches, and damping multiplies displacement by
  it. Snap/reversal detectors do not see this; use central-difference acceleration on raw (x,y)
  (compare against `maxAmplitude·(2π/frameCount)²`; flag > 10×).
- **Narrowing `kAnimMarginTransitionWidthFactor` makes it worse** (`d damping/d margin ∝ 1/width`).
- The smoothing window was first tuned on synthetic meshes (`halfWindow=2`) and was too small for the
  user's real, irregular mesh. **Validate against the user's real mesh**: `gm_debug_*` exports contain
  full per-vertex geometry (`mesh.vertices[]`: `P, Pu, Pv, isBoundary, …`); extract it into a harness.
- **Debug toggle**: a checkbox "Debug: disable damping/smoothing (self-intersection check only)"
  (property `meshAnimationDebugDisableContinuousStages`, seeded from env `GM_ANIM_DISABLE_CONTINUOUS`)
  turns off stages 1 and 3, leaving only the original stage 2. It is read when "Animate Mesh" is
  (re)started. The exported trace records `continuousStagesDisabledForDebug`.

### Analysing the user's exports

The app writes JSON into a `DebugOut/` folder next to the loaded image (the user has used `~/Documents/DebugOut`
and `~/Downloads/DebugOut`) — read them straight from disk, or he may point you at a file: `gm_meshanim_<r>x<c>_<ts>.json` (animation trace:
`animation` summary, per-frame `frames`, `git`, `mesh` = rows/cols only) and
`gm_debug_<solver>[_cieluv]_<r>x<c>_<ts>.json` (optimiser debug dump, **with** vertex geometry).
**Check `git.commit` / `git.dirty` first** to know which code produced a trace. Note that the actual
`minClearanceDistance` used is *not* in the trace (the default is 2.0) — recording it there would
remove an assumption; consider adding it.

## Open threads (as of 2026-10-07)

- **Waiting on the user's visual A/B**: does the real mesh still look uneven with the debug checkbox
  ON (original discrete-only behaviour)? He disputed the "vertex 13 margin reaches ~0 against fixed
  boundary edge (5,6)" diagnosis, which was computed with an *assumed* clearance. Do not re-assert or
  retract it until his result arrives.
- **First task for a new session: build the app.** The last Animate Mesh commits (the checkbox and a new
  "Row 7b" in `MainWindowController.mm`, plus the `GM_ANIM_DISABLE_CONTINUOUS`/property plumbing in
  `DocumentModel`) were written without ever being compiled. Build with Xcode/CMake, fix any errors,
  and note that `window.minSize` (720×560) was not adjusted for the extra row — the user checks the layout.
- Housekeeping done recently: README rewritten (old one is `docs/DEVELOPMENT_LOG.md`), MIT `LICENSE`
  added, `.gitignore` extended. Still to do on GitHub by the user: repo description/topics and the
  social-preview image (`docs/media/social-preview.png`).
