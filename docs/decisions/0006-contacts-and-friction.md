# 0006: Colliders, contacts and friction

Status: proposed (Phase 4)

## Goal

Keep the soft body out of static colliders (ground plane, sphere, box) with a hard bound on
penetration, add simple Coulomb friction, and do both inside the existing XPBD substep loop
so the real-time and reference presets still differ only in their counts.

## Decision

### Colliders (`hp/collision/collider.hpp`)

- One plain struct `Collider` with a kind tag: `Plane` (point + unit normal, a half-space),
  `Sphere` (centre + radius) and `Box` (centre, half extents, rotation matrix; an oriented box).
  Each collider has one friction coefficient `friction` (μ). Factories `Collider::plane/sphere/box`
  validate their input.
- A tagged struct rather than a virtual class: the kernel evaluates it inline in float and in
  double, nothing is allocated, and it maps one-to-one to JSON in the scenario phase.
- `signed_distance(collider, p, normal)` is a header template returning the exact signed
  distance (negative inside) and the outward unit normal. Inside a box it uses the nearest face;
  ties go to the lower axis. A point exactly at a sphere's centre gets +y.
- Colliders are **static**. They live in their own module (`collision` depends only on `core`),
  not in the solver, because the GNN solver will also need them as inputs later.

### Contact constraint

- Every free vertex is tested against every collider: `C = sdf(p) ≥ 0`, unilateral, zero
  compliance. With an immovable collider the XPBD update is simply `p −= sdf(p)·n` when
  `sdf(p) < 0`. Pinned vertices are skipped.
- No contact list. In each Gauss–Seidel iteration the kernel projects edges, then tets, then
  **contacts last**, re-evaluating the SDF each time. A vertex touching a single collider
  therefore ends every substep exactly on or outside it, up to float rounding.
- All vertices are tested, not only surface ones. It costs one SDF per vertex per collider
  per iteration, which is simpler and also catches interior vertices pushed through a thin
  surface layer.
- Contacts are fully inelastic (no restitution). Bounce comes only from the body's elasticity.

### Friction

- Positional Coulomb friction (Macklin et al. 2014, "Unified particle physics"). After the
  normal push-out of depth `d`, take the tangential part `t` of the vertex's displacement over
  the current substep (the kernel already tracks it as `dx`). If `|t| ≤ μ·d` it is removed
  (stick); otherwise it is shortened by `μ·d` (slide). For a resting or sliding vertex `d` is
  the normal acceleration times `h²`, so this gives a sliding deceleration of `μ·a_n`.
- One μ per collider, used for both sticking and sliding.
- Velocity is still `dx / h`, so the Phase 3 cancellation-free velocity is kept.

### API changes to Phase 3 code (approved before implementation)

- `XpbdSolver` gets a second constructor taking `std::vector<Collider>`, plus `colliders()`.
  The original constructor means "no colliders".
- `detail::XpbdKernel<S>::step` gets a trailing `const std::vector<Collider>& = {}`.
- With no colliders, or colliders that are never touched, results are bit-identical to Phase 3
  (tested).

## Measured behaviour

0.5 m soft rubber cube (125 vertices) unless noted. "rt" is the real-time preset (10 × 1),
"ref" the reference preset (256 × 1).

| Check | rt | ref |
|-------|----|-----|
| Max penetration at any frame end, all drop/impact/slide tests (bound 1e-5 m) | 6.0e-8 m | 5.9e-8 m |
| Rest height of the cube on the ground (half size 0.25 m) | 0.2466 m | 0.2469 m |
| Resting cube, 2000 frames: centre drift / max RMS speed | 4e-7 m / 1.3e-5 m/s | (not run) |
| 20 m/s impact: rebound energy, kinetic + gravitational, max / start | 1.0 | 1.00001 |
| Sliding on a 35° slope, μ = 0.5: acceleration / `g(sinθ − μcosθ)` | 0.992 | 0.998 |
| Same, μ = 0 | 1.0000 | 0.9999 |
| Stopping distance on flat ground, μ = 0.5, 2 m/s: measured / `v²/2μg` | 1.001 | 1.015 |
| 20° slope, μ = 0.5, `Y = 10 MPa` block: contact slip over 300 frames | 0.24 mm | 0.31 mm |
| 20° slope, μ = 0.5, `Y = 1 MPa` block: slip in the first 30 frames, then after | 2.1 mm, < 1 nm | 2.6 mm, 5 µm |
| float32 vs double kernel with contacts and friction, 45 frames (fraction of size) | 4e-6 | 5e-5 |

Overhead of three colliders (plane, sphere, box; 22–39 of 729 vertices in contact) on the
pinned-box benchmark is about 4–8% of frame time from 729 to 50k vertices, on both presets
(`results/phase-04-collisions/README.md`). That is within the 10% budget, so there is no broad
phase.

## Consequences and known limits

- **The elastic-energy metric is not reliable mid-impact on the real-time preset.** The
  metric computes `C²/(2α)` per constraint, which assumes converged constraints. During a
  20 m/s impact the real-time preset is far from converged, and the measured total (kinetic +
  gravitational + elastic) peaks at **2.34×** its start, even though the body rebounds lower
  than it started. On the reference preset the same total peaks at 1.00001×. So no-energy-gain
  checks use kinetic + gravitational energy, which leaves out a non-negative term and can only
  rise if the solver adds energy. The recording and metrics phase must not report the
  real-time elastic or total energy during impacts as physical energy.
- **Start-up slip of soft bodies on slopes.** A 1 MPa block placed at rest on a 20° slope
  (μ = 0.5, tan 20° = 0.36) slips 2–2.6 mm in its first ~6 frames, then sticks. The amount
  grows with softness and as `tanθ` approaches μ: 0.04 mm at μ = 1, 0.24 mm at 10 MPa, 0 on
  flat ground. As the block takes its weight it spreads sideways, and the lightly loaded contact
  edges slip until the load settles. This is plausible physically (partial slip), but it has
  not been compared against a continuum model.
- **Vertex-only contact.** Only vertices are tested against colliders. A box edge or corner,
  or a sphere smaller than about two mesh edge lengths, can push into a mesh face between
  vertices without being detected. Scenario colliders should be larger than the mesh spacing.
  Edge and face contact would need a different contact model.
- **Initial overlap is resolved as a velocity.** XPBD turns a push-out of depth `p` into
  velocity `p / h`. A body that starts 5 cm inside a collider leaves at about 30 m/s on the
  real-time preset. The solver is unchanged; the scenario loader will reject bodies that start
  inside a collider.
- **Thin colliders can tunnel** (sphere and box; the plane is a half-space and cannot). A vertex
  that moves more than the collider's thickness within one substep skips it. At 20 m/s the
  real-time substep moves 3.3 cm, so sphere and box colliders should be thicker than
  `speed × dt / substeps` plus a margin.
- With several colliders touching one vertex, they are projected in list order. Every
  multi-collider test still stayed within the bound (≤ 6e-8 m), but there is no proof for
  arbitrary overlaps.
- The cube resting on the ground with only damping and contact settles fully. Undamped bodies
  keep vibrating elastically; contact itself removes only the normal impact energy and
  friction work.
