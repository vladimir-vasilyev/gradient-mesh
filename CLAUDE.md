# CLAUDE.md

Guidance for Claude Code when working in this repository. Read this first; the long history is in
`docs/DEVELOPMENT_LOG.md` (138 KB — grep it by heading, do not load it whole).

## What this project is

**Gradient Mesh Studio**: a from-scratch C++17 implementation of Sun, Liang, Wen & Shum,
*"Image Vectorization using Optimized Gradient Meshes"* (ACM TOG 26(3), SIGGRAPH 2007), plus an
interactive macOS app (AppKit, Objective-C++). The goal is **maximum fidelity to the paper**; every
deviation is documented in the development log under "Known simplifications vs. the paper".

| Path | What | Compiles in a Linux cloud session? |
|---|---|---|
| `core/` (`gmcore`) | Portable algorithm library: mesh, optimiser, Bézier fitting, Lazy-Snapping segmentation, SVG export | **Yes** |
| `core/tests/test_main.cpp` | `gmcore_tests`, dependency-free regression suite | **Yes** |
| `cli/main_cli.cpp` | `gmesh_cli` command-line harness | **Yes** |
| `mac/*.mm`, `mac/*.h` | The AppKit app (`DocumentModel`, `MainWindowController`, `CanvasView`, `GLReconstructionView`) | **No** — needs macOS + Xcode |
| `core/src/MeshOptimizerCeres.cpp` | Optional Ceres solvers | Only if Ceres is installed (it is not by default; an empty translation unit otherwise) |
| `spike/` | Ceres experiments and probes | n/a |

## Working with the user

- The user (Vladimir) writes in **Russian**. Reply in Russian; keep code, comments, commit messages
  and docs in English.
- **Standing priority order** (his words: *"порядок 1 - regression-тесты. 2 - Точность по статье.
  3 - Настоящий инструмент выделения. Делаем, отлаживаем и комитим по одной таске за раз."*):
  1. regression tests, 2. accuracy against the paper, 3. a real selection tool. **One task at a
  time: implement, debug, commit.** Items 1–3 are done; the user has since been steering work on
  "Animate Mesh" directly. Do not start unrequested side-quests.
- He tests on his own Mac in Xcode and reports back, often with exported debug JSON. **When what he
  sees contradicts your analysis, believe the observation** and build an experiment that settles it
  (see the debug toggle below) instead of arguing from numeric proxies.
- Be honest about what was and was not verified. Say so in the reply and in the commit message.

## Build, test, run (verified on Linux, GCC 13)

```sh
cmake -B build . && cmake --build build -j --target gmcore_tests gmesh_cli
./build/gmcore_tests                      # expect: ALL TESTS PASSED (20 cases / 3387 checks at the time of writing)
./build/gmesh_cli --input gradient.png --rows 9 --cols 9 --pyramid-levels 4 --margin 0 --out-prefix demo
```

- The cloud image has CMake, g++ and libpng; **Ceres is not installed**.
- macOS app (user's machine only): `cmake -G Xcode -B build_xcode . && open build_xcode/GradientMeshStudio.xcodeproj`.
- **Run `gmcore_tests` after every `core/` change** and add a test for new core behaviour. Nothing
  else builds `gmesh_cli` automatically, and it silently stopped compiling once already
  (`buildInitial` changed to take `BezierSpline`); rebuild it too when you change `GradientMesh`/
  `MeshOptimizer` signatures. There is no CI yet — adding one would be a good first task if asked.

## The hard constraint: you cannot compile `mac/*.mm`

There is no Objective-C++/AppKit/OpenGL toolchain in the cloud. Therefore:

1. Keep algorithmic logic in `core/` (pure C++) whenever possible so it can be compiled and tested.
2. For logic that must live in `mac/*.mm` (e.g. the Animate Mesh loop in `DocumentModel.mm`), use
   **literal extraction**: copy the *exact* patched text out of the `.mm` with a script (anchor-based
   string slicing — never retype it) into a small `FakeDocumentModel`-style harness (ivars as
   struct members, `typedef long NSInteger;`), compile it with `g++ -std=c++17 -Icore/include`
   against the real `gmcore` headers, and run it on realistic meshes **before** committing.
3. For pure UI/AppKit edits, review the diff carefully and check `{}`/`()`/`[]` balance. Say in the
   commit message that it is uncompiled and ask the user to build in Xcode.
4. When patching, use a Python string-replacement script with `assert text.count(old) == 1` per
   replacement, so a stale anchor fails loudly instead of editing the wrong place.
5. Put the full explanation (root cause, why this fix, what was measured) **in the code comment**
   in the shipped file — once the essay ended up only in a patch script and never in the code.

## Delivering changes (Code project, no device bridge)

You cannot touch the user's Mac files from here. Work on a branch and open a PR (or push as
instructed); the user pulls and builds. The local branch on his Mac is `master` and the
remote branch is `origin/main` (`git push origin master:main`); he does his own pushes unless access is granted.
Commits: one task per commit, message explains *why*, follow the attribution trailer the harness gives.

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

He uploads JSON from his `DebugOut/` folder: `gm_meshanim_<r>x<c>_<ts>.json` (animation trace:
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
- A new "Row 7b" was added to the Animate Mesh UI and `window.minSize` (720×560) was not adjusted;
  the user should check the layout at minimum size.
- Housekeeping done recently: README rewritten (old one is `docs/DEVELOPMENT_LOG.md`), MIT `LICENSE`
  added, `.gitignore` extended. Still to do on GitHub by the user: repo description/topics and the
  social-preview image (`docs/media/social-preview.png`).
