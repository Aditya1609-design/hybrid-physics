# Phase 5: Scenario system

Status: in review. Branch `phase/05-scenario-system`.

## What was built

- `scene` module (new files only, no change to approved engine code):
  - `hp/scene/scenario.hpp`: a plain `Scenario` struct (mesh params, material, pose, initial
    linear and angular velocity, pin boxes, solver settings, colliders, seed, frames), with
    `parse_scenario(text)` and `load_scenario(path)`. Unknown keys are errors, and every error
    names its key's path (`colliders[1].radius: must be positive`).
  - `hp/scene/scene.hpp`: `build_scene(scenario)` returns `Scene { TetMesh, Topology,
    SimState, XpbdSolver, warnings }`. It rejects bodies that start inside a collider (pinned
    vertices included), rejects pin boxes that select no vertex and more than one body, and
    warns about small sphere colliders and colliders thin enough to tunnel through.
  - nlohmann/json is linked PRIVATE and used only in `scenario.cpp`; no public header exposes it.
- `apps/headless_runner` → `hp_run`: runs a scenario and prints kinetic, gravitational and
  elastic energy separately, edge and volume error, max penetration and step time, then a
  final-state hash. It can also write a per-frame metrics CSV and the final surface as an OBJ,
  or only validate the file (`--check`), and can override the frame count and preset.
- Six example scenarios in `scenarios/`, all real-time preset, 336–823 vertices.
- Docs: `docs/scenario_format.md` (every key, unit, default and check), ADR 0007,
  architecture and README updated.

## How to run

```sh
./scripts/ci.sh                                        # release + asan, 97 tests each
./build/release/tests/hp_tests -sf="*scene*,*scenario*" -s
R=./build/release/apps/headless_runner/hp_run
for f in scenarios/*.json; do $R "$f" | tail -2; done  # all six, about 10 s in total
$R scenarios/cube_drop.json --preset reference --frames 60
$R scenarios/cube_drop.json --csv out.csv --surface-obj final.obj
$R some_file.json --check                              # validate only; exit 1 with the key path
```

Exit codes: 0 ok, 1 invalid scenario or unwritable output, 2 usage error, 3 non-finite state.

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh` before every commit (6 commits) | PASS, release and asan, no warnings |
| Scenario built from JSON vs the same scene built through the C++ API, 60 frames with plane and sphere | byte-identical positions, velocities and masses |
| Parser: every field, defaults, 29 error cases | each message has the expected key path |
| Absent material keys | `Material` defaults bit for bit |
| Rotated and moved beam | tet volumes equal to 1e-4, edge lengths to 1e-5, centroid at `position` to 1e-5 m |
| Initial velocity `v + ω × r` | per vertex to 1e-5 m/s; mean momentum equals `v` |
| Body touching the ground (distance 0) | accepted |
| Body 5 cm into the ground, also with those vertices pinned; body overlapping a sphere | rejected with collider index, vertex count and depth |
| Sphere radius 0.05 m on a mesh with 0.15 m mean edges | warning; 0.5 m: none |
| Box 2 cm thick at 20 m/s: real-time warns, 20 cm box or reference preset does not | yes |
| All six examples, 60 frames, in the test suite | finite, max penetration ≤ 1e-5 m, no warnings, two runs byte-identical; adds about 3 s to the asan run |
| All six examples, full length with `hp_run` | exit 0, max penetration ≤ 1.2e-7 m |
| `cube_slide_slope` (25°, μ = 0.5, 1 m/s) against `g(sinθ − μcosθ)` | stops at 3.25 s after 1.69 m; analytic 3.34 s, 1.67 m |
| `hp_run` hash, two runs of each scenario, and with `--preset reference` | identical |

Final-state hashes on my M1 (release, Apple clang 21.0.0), for comparing on your teammate's Mac.
They should match only with the same compiler version (ADR 0002):

| Scenario | Hash |
|----------|------|
| `ball_on_sphere` | `f0d5d9de22874ea6` |
| `beam_drape_box` | `fe791da0c528f3c0` |
| `cantilever_beam` | `6fca4d991b88a9e7` |
| `cube_drop` | `b612e6591e49e988` |
| `cube_slide_slope` | `f480ffc77d02e8e4` |
| `cube_throw_spin` | `866eed1738befe54` |

## Changes from the approved plan

- `Scene` has no separate collider list: the solver already holds the colliders and exposes
  them through `solver.colliders()`, so a second copy could only drift out of sync.
- **Tunnelling threshold.** The warning uses *half* a collider's thickness per substep, not the
  full thickness. A vertex pushed past the middle is projected out through the far side, so
  half is the real limit. ADR 0006 (Phase 4) states the full thickness; I have not edited it,
  but ADR 0007 records the correction. Tell me if you want ADR 0006 amended.
- **Speed used for that warning:** `max initial speed + |g| · frames · dt`. The plan said
  "initial max speed", but that never warns for a dropped body, which starts at rest.
- `cube_throw_spin`: the wall was raised from 1 m to 2 m after the first run showed the
  cube bouncing over it, so the wall was never hit.
- Material `damping` must be ≥ 0. That check is `Material::validate`'s, so its error names
  `bodies[0].material` rather than the `damping` key.

## Known limitations

- **Elastic energy spikes on the real-time preset** (the Phase 4 limitation): `cube_throw_spin`
  (30 MPa) shows 2.3e4 J of "elastic" energy in a single impact frame, and `ball_on_sphere`
  2.3e3 J, while kinetic + gravitational energy keeps falling. `hp_run` prints a note saying
  so; do not read the real-time elastic column as physical energy during impacts.
- `cantilever_beam` (10 MPa, damping 0.5) is still swinging after 5 s, and the real-time
  preset is known to sag too far at this stiffness (ADR 0005). It is a demo, not a reference.
- The voxel sphere in `ball_on_sphere` rolls off and keeps rolling (no rolling resistance), at
  about 3.6 m/s at 4 s.
- The tunnelling warning's speed bound is loose: long runs with sphere or box colliders thinner
  than about `|g| · T · dt / substeps` will warn even if the body never gets that fast. None of
  the examples trigger it.
- One body, a fixed timestep, no includes between scenario files, and no way to write a
  perfectly stiff (zero-compliance) material.
- `seed` is carried but unused until randomized scenarios exist.

## What to evaluate

- Run `./scripts/ci.sh` on your teammate's Mac, and compare the `hp_run` hashes above.
- The format in `docs/scenario_format.md`: key names, units, defaults, and whether the six
  examples cover the scenes you want for the viewer and the dataset.
- The two warnings and their thresholds (ADR 0007), including the half-thickness correction.
- `hp_run` output and CSV columns; the CSV is a convenience, not the Phase 7 trajectory format.

## Decisions needed

1. **ADR 0006 wording on tunnelling** (full vs half thickness). Recommendation: add a one-line
   note to ADR 0006 pointing to ADR 0007, without rewriting it.
