# Phase 6: Viewer

Status: approved 2026-11-21 (tag `phase-06-approved`). Branch `phase/06-viewer`.

## What was built

- **raylib 5.5** as an optional dependency (`cmake/ViewerDependencies.cmake`, pinned tag archive
  with SHA256, cached like the others). `HP_BUILD_VIEWER` is ON by default; with it OFF nothing
  is downloaded and C is not enabled. The engine and its dependency guard are unchanged.
- **`hp_viewer_core`** (no graphics, unit-tested in release and asan):
  - `OrbitCamera`: yaw/pitch/distance around a target, pitch clamped short of the poles, zoom
    limits, pan that follows the cursor, framing a box inside the view for any aspect.
  - `build_surface` / `surface_edges`: surface triangles unrolled for flat shading with outward
    normals; unique wire edges.
  - Color modes reading `solver_owner`, `region_id` and `body_id`; a fixed palette; CPU Lambert
    shading; the display-only owner/region preview (`make_owner_preview`).
  - Collider geometry: plane basis, box corners, outward-wound faces and edges, a unit sphere
    mesh.
  - `SimDriver` and `FixedStepPacer`: the scene stepped exactly like `hp_run`, wall time turned
    into 0–4 fixed steps per render frame, reset, a frame limit, metrics and the state hash.
- **`hp_view`**: runs a scenario live. Solid, solid + wire and wire modes, optional tet edges,
  ground grid, translucent sphere and box colliders, stats panel, color legend, controls panel,
  follow camera, reset, reload from disk, pause and single step. `--frames N` stops exactly on
  frame N and prints the same final state hash as `hp_run --frames N`; `--screenshot` saves that
  frame.
- Docs: ADR 0008, architecture (viewer section, target diagram), README (viewer usage,
  `HP_BUILD_VIEWER=OFF`, layout, phase table).

## How to run

```sh
./scripts/ci.sh                                         # release + asan, 117 tests each
./build/release/tests/hp_tests -sf="*viewer*"           # the 20 viewer-core test cases
V=./build/release/apps/viewer/hp_view
$V scenarios/beam_drape_box.json                        # press H for the controls
$V scenarios/cube_drop.json --preset reference          # "slower than real time" case
$V scenarios/cantilever_beam.json --paused              # space runs, '.' steps
$V scenarios/cube_drop.json --frames 120                # compare with hp_run --frames 120
```

Exit codes: 0 ok, 1 invalid scenario or unwritable screenshot, 2 usage error.

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh` before every commit (7 commits) | PASS, release and asan, no warnings in our code |
| `hp_view --frames 90` and `--frames 120` vs `hp_run`, all six scenarios | identical final state hashes |
| `SimDriver` vs direct `XpbdSolver` stepping, 90 frames (test) | byte-identical positions and velocities, identical energies |
| Pacer: carry-over, cap of 4, backlog dropped, 600 frames at 60 Hz (test) | as designed |
| Reset (test) | state and hash identical to frame 0 |
| Camera framing, 4 aspects x 3 yaws (test) | every box corner inside the view |
| All six scenarios in the asan build of `hp_view` | no sanitizer reports (after the fix below) |
| Real-time preset, 3 s per scenario, 120 Hz display | 110–131 render FPS, step 0.65–1.7 ms, sim speed 1.00x |
| `cube_drop` on the reference preset | about 40 ms per step, 0.40x, warning shown |
| Screenshots of every display mode, colliders, preview, panels | checked by eye |

Final state hashes after 120 frames on my M1 (release, Apple clang 21.0.0), the same for
`hp_view --frames 120` and `hp_run --frames 120`:

| Scenario | Hash |
|----------|------|
| `ball_on_sphere` | `60e9249da7d258a9` |
| `beam_drape_box` | `fd7f65ec060e7cdb` |
| `cantilever_beam` | `c04db1eb21463a9f` |
| `cube_drop` | `6edfe562d1e9783a` |
| `cube_slide_slope` | `ea3c4cc353d1554c` |
| `cube_throw_spin` | `f41f7c13be72f854` |

## Bug found and fixed during the phase

AddressSanitizer reported a heap overflow when `hp_view` started. `render.hpp` declared
`Material material_` inside `hp::viewer`. In `render.cpp` that is raylib's `::Material`
(40 bytes); in `main.cpp`, which includes engine headers first, name lookup found
`hp::Material`, the physics material (16 bytes). The class had two layouts, and the release
build was silently writing past the object. raylib types are now written `::Mesh` and
`::Material` in that header, with a comment saying why. CI did not catch it because it never
opens a window (see decision 2).

## Changes from the approved plan

- **C is enabled with `enable_language(C)` only when the viewer is built**, instead of adding it
  to `project()`, so builds without the viewer are unchanged.
- **`tests/CMakeLists.txt` also sets policy CMP0156** (CMake 3.29+), which removes the repeated
  engine library from `hp_tests`' link line; without it Apple's linker warns about duplicates.
- **Boxes are drawn from computed corners** (`box_corners`, `box_faces`, tested), not with
  `rlMultMatrixf`, so the geometry is unit-tested.
- **Added:** `--screenshot` (for checking the rendering and for report figures), V to follow
  the body (`ball_on_sphere` and `cube_slide_slope` leave the starting view), and a frame limit
  on `SimDriver::advance` so `--frames N` never overshoots.
- **"Slower than real time"** is measured as sim time / wall time over half-second windows
  (warning below 0.9), not taken from the pacer's per-frame flag: window start-up made the first
  frames look like a stall.
- **Framing pads the scene to at least 1.5 m across**, so a 0.4 m cube does not fill the view.
- **Elastic note**: the overlay says the real-time preset's elastic value is unconverged and
  not physical energy, not only "during impacts": `cantilever_beam` shows 2.6 kJ with no impact.

## Known limitations

- **Screenshot readback strip.** About half of the `--screenshot` readbacks on this machine have
  the rightmost 112 pixels (56 points) black. The GL viewport covers the full 2560x1600
  framebuffer and no resize happens, so I believe it is a quirk of Apple's OpenGL-on-Metal
  readback, but I could not see the live window to confirm. See decision 1.
- OpenGL is deprecated on macOS (works on 27.0.1); sokol is the fallback (ADR 0008).
- Owner and region coloring is uniform until Phase 9 (all region 0, all XPBD); the P preview
  shows the hook works. The preview only changes the viewer's colors.
- No picking or dragging of vertices, no recording, one scenario per window.
- The reference preset runs at about 0.4x real time on the 729-vertex cube.
- `hp_run`'s state hash is duplicated in `SimDriver` (agreed; candidate for Phase 7/8).

## What to evaluate

- Run all six scenarios and try every key (H lists them). Check the rotated box in
  `beam_drape_box`, the slope in `cube_slide_slope`, and P in owner and region modes.
- Whether the right edge of the **live window** ever shows a black strip (decision 1).
- `./scripts/ci.sh` on your teammate's machine, and the 120-frame hashes above with
  `hp_view --frames 120` and `hp_run --frames 120`.
- The overlay contents and wording, especially the energy note.

## Decisions needed

1. **Black strip.** If you see it in the live window, I will investigate further (or try the
   sokol fallback for comparison). If only screenshots have it, I will retry the readback until
   the right edge is not black. Recommendation: check first; no code change until then.
2. **Viewer smoke run in CI.** `ci.sh` never opens a window, so the overflow above was found by
   hand. Recommendation: an opt-in `ci.sh --with-viewer` that runs every scenario for 30 frames
   in the release and asan viewers (a few windows open and close, about 15 s), used before each
   phase review rather than on every commit.
3. **Teammate's OS.** If it is Linux, raylib needs the X11 and OpenGL dev packages, or the
   build can use `-DHP_BUILD_VIEWER=OFF`. Tell me which, and I will document the packages.

## Review outcome

- Approved.
- Black strip: not visible in the live window, so it only affects screenshot readback. No code
  change.
- Viewer smoke run: added as the opt-in `ci.sh --with-viewer` (every scenario, 30 frames, release
  and asan viewers, final hash compared with `hp_run`). It passed before the merge.
- Teammate's machine is macOS (M4), so no Linux packages are documented.
