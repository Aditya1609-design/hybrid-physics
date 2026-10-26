# Phase 4: Collisions

Status: approved 2026-10-27 (tag `phase-04-approved`). Branch `phase/04-collisions`.

## What was built

- `collision` module (new, depends only on `core`): `Collider` (plane, sphere, oriented box, one
  friction coefficient each) with validating factories, and `signed_distance<S>`, a header
  template that returns the exact signed distance and the outward normal, in float or double.
- `solver`: hard one-sided contacts and positional Coulomb friction inside the XPBD kernel.
  Every iteration projects edges, then tets, then contacts. These are additive API changes,
  approved before implementation: a second `XpbdSolver` constructor taking colliders,
  `XpbdSolver::colliders()`, and a defaulted trailing collider argument on
  `detail::XpbdKernel<S>::step`. With no colliders, results are bit-identical to Phase 3.
- `metrics`: `penetration(state, colliders)` returns max and RMS depth, the number of
  penetrating vertices and the number of vertices in contact, computed in double.
- `apps/benchmark/hp_xpbd_speed --colliders`: adds `<preset>+colliders` rows; the CSV columns
  are unchanged.
- ADR 0006 (colliders, contacts, friction, limits); overhead results in
  `results/phase-04-collisions/`; architecture doc and README updated.

## How to run

```sh
./scripts/ci.sh                                               # release + asan, 82 tests each
./build/release/tests/hp_tests -sf="*contact*,*friction*,*collider*,*penetration*" -s \
    | grep MESSAGE                                            # prints the measured numbers
./build/release/apps/benchmark/hp_xpbd_speed --quick --colliders   # about 20 s
```

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh` before every commit (6 code commits) | PASS, release and asan, no warnings |
| No colliders, empty list, or a collider never touched, vs Phase 3 solver | byte-identical |
| Max penetration at any frame end: drops on plane, sphere, rotated box; 20 m/s impacts; friction | 6.0e-8 m (rt), 5.9e-8 m (ref); bound 1e-5 m |
| Cube resting on the ground, 2000 frames (rt) | drift 4e-7 m, max RMS speed 1.3e-5 m/s, no penetration |
| Rest height of a 0.5 m cube | 0.2466 m (rt), 0.2469 m (ref) |
| 20 m/s impacts on a plane and a thick box | no tunnelling, finite |
| No energy gain, kinetic + gravitational, with and without friction | max/start 1.0 (rt), 1.00001 (ref) |
| Full energy incl. elastic, 20 m/s impact | ref 1.00001× start (asserted); rt 2.34× (metric artefact, see limitations) |
| 35° slope, μ = 0.5: acceleration / `g(sinθ − μcosθ)` | 0.992 (rt), 0.998 (ref) |
| 35° slope, μ = 0 | 1.0000 (rt), 0.9999 (ref) |
| Flat ground, μ = 0, 2 m/s for 1 s | speed 2.00001 (rt), 1.99988 (ref) |
| Flat ground, μ = 0.5: stopping distance / `v²/2μg` | 1.001 (rt), 1.015 (ref); contact face then still (< 22 µm) |
| 20° slope, μ = 0.5, `Y = 10 MPa`: contact slip over 300 frames | 0.24 mm (rt), 0.31 mm (ref); bound 1 mm |
| 20° slope, μ = 0.5, `Y = 1 MPa`: start-up slip, then slip after frame 30 | 2.1 / 2.6 mm, then < 1 nm / 4.6 µm |
| Byte-identical rerun and second solver, three colliders with friction | yes |
| float32 vs double kernel with contacts and friction, 45 frames | 4e-6 (rt), 5e-5 (ref) of body size |
| Overhead of three colliders, 729 to 50k vertices | about 4–8% of frame time |

## Changes from the approved plan

- **Stick test material** (your decision during the phase). The planned "< 1 mm over 300
  frames" check uses a 10 MPa block, and stays measured from frame 0. A separate 1 MPa test
  asserts < 5 mm start-up slip and < 0.1 mm after frame 30.
- **Energy checks** (your instruction during the phase). No-gain checks use kinetic +
  gravitational energy. A new test asserts that the reference preset's full energy, including
  elastic, does not rise during a 20 m/s impact; the real-time peak is only reported.
- **"Stopped" check on flat ground.** I first added a check of my own (centre-of-mass speed
  < 1e-2 m/s), which was not in the plan. It failed at 0.013 m/s on the real-time preset
  because the undamped block keeps rocking elastically after it stops (contact face fixed to
  five digits from frame 36). It now checks that the contact face moves < 0.1 mm between
  frames 40 and 60. The planned stopping-distance check (10%) is unchanged.
- **Collider unit-normal check.** The tolerance went from 1e-9 to 1e-6, because normals and
  rotations are stored in float32. It was wrong in kind, not loosened to make physics pass.
- **Benchmark output.** The collider rows are labelled `realtime+colliders` and
  `reference+colliders` instead of adding a column, so `gnn_cost_estimate.py` reads the CSV
  unchanged. The plan did not mention a results folder; `results/phase-04-collisions/` was
  added, following Phase 3.
- The planned bounding-box pre-check was not needed (overhead within budget).

## Known limitations

- **The elastic-energy metric is not trustworthy mid-impact on the real-time preset.**
  `C²/(2α)` assumes converged constraints. In a 20 m/s impact the measured total peaks at
  2.34× its start while the body rebounds lower than it started; on the reference preset it
  peaks at 1.00001×. The recording and metrics phase must not report real-time elastic or total
  energy during impacts as physical energy.
- **Start-up slip of soft bodies on slopes** near the friction limit: 2–2.6 mm for a 1 MPa
  block at 20° with μ = 0.5, then it sticks. It scales with softness and with `tanθ / μ`. This is
  plausibly physical partial slip, but it has not been compared against a continuum model.
- **Vertex-only contact.** Collider edges and corners, or spheres smaller than about two
  mesh edge lengths, can push into mesh faces between vertices undetected.
- **Initial overlap becomes velocity** (`depth / h`). Bodies must start outside colliders; the
  scenario loader will reject overlaps (agreed).
- **Thin sphere and box colliders can tunnel** when a vertex moves further than their thickness
  in one substep (3.3 cm at 20 m/s on the real-time preset). The plane cannot be tunnelled.
- Static colliders only, no restitution, and one μ for sticking and sliding (all agreed).
- With several colliders on one vertex, projection is in list order. All tests stayed within
  6e-8 m, but arbitrary overlaps are not proven.
- The asan CI run grew from about 10 s to about 76 s, mostly from the reference-preset drop and
  slide tests (256 substeps). The release run takes about 17 s.

## What to evaluate

- Run `./scripts/ci.sh` on the teammate's Mac (`CI PASS`), and
  `hp_xpbd_speed --quick --colliders` to compare overhead.
- The contact and friction model in ADR 0006, and whether vertex-only contact is enough for
  the scenarios you have in mind.
- The two limitations that affect later phases: the elastic-energy metric mid-impact (metrics
  and recording), and start-up slip of soft bodies on slopes (any slope scenarios in the dataset).
- The friction checks against analytic results (slope acceleration, stopping distance).
- The additive changes to `XpbdSolver` and `XpbdKernel` (approved in the plan) in
  `hp/solver/xpbd.hpp` and `hp/solver/detail/xpbd_kernel.hpp`.

## Decisions needed

1. **asan CI time** (about 76 s, from about 10 s). Recommendation: keep it, because the
   reference-preset contact tests are the strongest evidence for the bound. If it slows you
   down, I can run the reference cases only in the release preset.
2. **Energy metric for later phases.** Recommendation: in the metrics and recording phase,
   record kinetic, gravitational and elastic energy separately (not just the total), and
   document that elastic energy is meaningful only for converged states (reference preset or at
   rest). This needs no engine change now.

## Review outcome

- Approved.
- asan CI time: kept as it is, including the reference-preset contact tests.
- Energy in the metrics and recording phase: record kinetic, gravitational and elastic energy
  separately, and document that elastic energy is only meaningful near convergence (reference
  preset, or at rest).
- Requirements for the scenario loader (Phase 5):
  - Reject a scenario whose body starts overlapping a collider (any vertex with signed
    distance < 0), since XPBD would turn the overlap into a large velocity.
  - Warn when a sphere collider's radius is smaller than about two mesh edge lengths, since
    vertex-only contact can let it push into faces between vertices.
