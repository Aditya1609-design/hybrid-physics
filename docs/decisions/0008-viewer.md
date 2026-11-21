# 0008: Live viewer

Status: proposed (Phase 6)

## Goal

Watch any scenario file run live: orbit camera, solid and wireframe modes, colliders, coloring
by the per-vertex solver owner and region arrays, and a stats overlay with FPS and energies.
Without touching the engine, and without changing what the simulation computes.

## Decision

- **raylib 5.5**, fetched like the other dependencies (tag archive, SHA256, cached in
  `HP_DEPS_CACHE_DIR`, about 42 MB). It bundles GLFW and builds with CMake alone, and it has
  text, cameras and immediate-mode drawing built in. Audio is left on (the default): turning it
  off needs `CUSTOMIZE_BUILD`, which makes raylib 5.5 warn about redefined macros.
- **Optional, on by default.** `HP_BUILD_VIEWER` (ON) adds `cmake/ViewerDependencies.cmake`
  and enables C (raylib is C); with it OFF nothing is downloaded and the rest builds as before.
  `ci.sh` builds the viewer in release and asan.
- **Two targets.** `hp_viewer_core` holds everything that needs no graphics library (orbit
  camera, flat surface mesh, color modes, collider geometry, stepping and pacing) and is
  unit-tested in every build. `hp_view` adds raylib, drawing and input. The engine's dependency
  guard is unchanged.
- **Same physics as `hp_run`.** `SimDriver` builds the scene with `build_scene` and steps it with
  `RegionMask::all` and `kFrameDt`. Wall time only decides how many steps run per render frame
  (`FixedStepPacer`: leftover time carried over, at most 4 steps per frame, backlog dropped when
  the solver cannot keep up). So frame N in the viewer is bit-identical to frame N in `hp_run`:
  `hp_view --frames N` prints the same final state hash (tested in code and on all six examples).
- **Slower than real time is measured, not guessed.** The overlay shows sim time / wall time
  over half-second windows and warns below 0.9. The reference preset (about 40 ms per frame for
  `cube_drop`) shows 0.4x.
- **Shading on the CPU.** The surface is an unindexed mesh (three vertices per triangle) with
  flat Lambert shading baked into the vertex colors each frame. No shader files, so nothing
  depends on GLSL versions. Wireframes draw surface edges or all tet edges.
- **Coloring reads `SimState`.** Owner mode reads `solver_owner`, region mode `region_id`, body
  mode `body_id`. Until regions exist (Phase 9) every vertex is region 0 and owned by XPBD, so a
  display-only preview (P) paints four alternating slabs along x into the viewer's color buffer.
  It never writes to the simulation state, and the overlay labels it as a preview.
- **Colliders** are drawn from `solver.colliders()`: planes as a grid patch under the camera
  target (snapped to the grid spacing so lines do not swim), spheres and boxes as shaded
  translucent solids with back faces culled and no depth writes, plus outlines.

## Alternatives considered

- **sokol** (header-only, Metal on macOS): no dependence on Apple's deprecated OpenGL, but no
  text or camera, so more of our own code. The fallback if OpenGL goes away.
- **GLFW + OpenGL by hand:** the same OpenGL dependency as raylib with more boilerplate.
- **One simulation step per render frame:** simpler, but sim speed would follow the display
  rate (this machine runs at 120 Hz, so twice real time).

## Consequences and limits

- OpenGL has been deprecated on macOS since 10.14. It still works on macOS 27.0.1 (Apple runs it
  on Metal), but could be removed in a future release.
- raylib's `Mesh` and `Material` must be written `::Mesh`/`::Material` inside `hp::viewer`:
  otherwise `hp::Material` (the physics material) wins name lookup in files that include engine
  headers first, and the class gets two layouts. AddressSanitizer caught this during the phase.
- `--screenshot` reads the frame back before the buffer swap. On this machine about half of the
  readbacks have the rightmost 112 pixels black even though the viewport covers the whole
  framebuffer; this looks like an OpenGL-on-Metal readback quirk. The live window does not show
  it (checked in the Phase 6 review).
- Linux would need the X11 and OpenGL development packages for raylib; configure with
  `-DHP_BUILD_VIEWER=OFF` where they are missing.
- The viewer is not exercised by `ctest`: it needs a window. Its logic is, through
  `hp_viewer_core`, and `ci.sh --with-viewer` runs the app itself on every scenario in the
  release and asan builds.
