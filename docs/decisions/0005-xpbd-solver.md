# 0005: XPBD solver and presets

Status: proposed (Phase 3)

## Goal

A classical soft-body solver that (a) gives near-converged results for training data and error
measurement, (b) runs in real time as the normal engine and the pure-classical baseline, and
(c) uses the same code for both, so the only difference between them is a few counts.

## Decision

### Solver interface

- `ISolver::step(SimState&, const RegionMask&, dt)` (`hp/solver/solver.hpp`). `RegionMask`
  currently only supports `all()`; `XpbdSolver` throws on anything else. Partial masks arrive
  with mixed ownership, without changing the signature.

### Constraints and material

- One distance constraint per mesh edge, `C = |x_i − x_j| − L`, and one volume constraint per
  tet, `C = V − V0` (signed volume, so it also pushes inverted tets back).
- `Material` holds compliances **per unit volume** (1/Pa) and scales them per element:
  - edge: `α_e = edge_compliance · L² / V_e`, where `V_e` is the edge's share of volume (each tet
    gives a sixth of its volume to each of its six edges). This is the energy `½·Y·V_e·ε²` of a
    strain `ε = ΔL/L` with `Y = 1/edge_compliance`.
  - tet: `α_t = volume_compliance · V0`, the energy `½·K·V0·(ΔV/V0)²` with `K = 1/volume_compliance`.
  - A test shows that a uniformly stretched box stores the same elastic energy at 3, 6 and 12
    cells per side (511.14 J each), so the material does not change with mesh resolution.
  - `Y` and `K` are moduli of an edge network, not of a continuum model. They are only
    approximately Young's and bulk moduli, and the effective Poisson ratio is not set directly.
    Defaults: `Y = 1 MPa`, `K = 10 MPa` (soft rubber), density 1000 kg/m³, no damping.

### Time stepping

- Substepped XPBD ("small steps", Macklin et al. 2019): the frame `dt` (fixed, `kFrameDt =
  1/60 s`) is split into `substeps`. In each substep: predict with explicit gravity, reset the
  multipliers, run `iterations` Gauss–Seidel sweeps (all edges in sorted order, then all tets in
  mesh order), update velocity, then apply damping as `v /= 1 + damping·h`, which is stable for any `h`.
- **No cancellation in velocity.** Projections update a per-substep displacement array `dx`, with
  positions read as `x + dx`, and velocity is `dx / h`. The usual `(x_new − x_old)/h` subtracts two
  nearly equal float32 values; at 256 substeps the per-substep gravity displacement
  `g·h² ≈ 1.5e-7 m` is about one float32 spacing at 1 m.
- The kernel is a template (`hp/solver/detail/xpbd_kernel.hpp`) built for `float` (the engine)
  and `double` (only to measure float32 error in tests).
- Single-threaded with a fixed order, so results are bit-identical on rerun (tested).

### Presets (`XpbdSettings::realtime()` and `::reference()`)

Chosen by measurement on the default beam (756 vertices) and a smaller test beam:

| Preset    | Substeps × iterations | Frame time, 729 verts | Role |
|-----------|-----------------------|-----------------------|------|
| real-time | 10 × 1                | 1.55 ms               | normal engine, pure-classical baseline |
| reference | 256 × 1               | 41 ms (26×)           | training data, error reference |

- At equal cost, more substeps usually beat more iterations. For example, 128×1 against 64×2 is
  3× more accurate on the soft beam and is better in every damped (quasi-static) case. The
  exception is the undamped stiff beam, where 64×2 was slightly better (4.3e-3 against 6.0e-3).
  Both presets use one iteration.
- Reference criterion: doubling its substeps changes positions by less than 0.1% of body size.
  Damped stiff cantilever (`Y = 100 MPa`): 256 → 512 moves the lowest point by 0.3 mm on a 2 m
  beam (0.015%). Undamped swinging cantilever (`Y = 1 MPa`): 256 against 512 differs by 3e-5 of
  the length.
- Real-time: every setting from 4 substeps up was stable on all test scenes. 10 × 1 leaves room in
  a 16 ms frame up to about 5k vertices (12 ms) and is a common XPBD default.

### Precision

- Same kernel, float32 against double, 30 frames on the test cantilever: max position difference
  5e-7 (real-time) and 9e-6 (reference) of the beam length. At 256 substeps the max velocity
  difference is 1e-3 m/s at 5 m/s speed. float32 is kept; no double scratch arrays are needed.
- Positions on the default beam agree with a 2048-substep double run to about 2e-4 of the length.
  Below that, float32 rounding dominates, so adding substeps beyond about 512 does not help.

## Consequences and known limits

- **Real-time accuracy depends strongly on the material.** Real-time XPBD is effectively softer
  than the material it is given. Tip deflection of the default 2 m beam at equilibrium, real-time
  against reference:

  | Material | Real-time | Reference | Real-time / reference |
  |----------|-----------|-----------|-----------------------|
  | `Y = 1 MPa` | 1.525 m | 1.498 m | +1.8% |
  | `Y = 10 MPa` | 0.493 m | 0.313 m | +58% |
  | `Y = 100 MPa` | 0.208 m | 0.033 m | 6.4× |
  | perfectly stiff | 0.168 m | 0.000 m | all of it is error |

  So "a bit stretchy" holds only for soft materials. For stiff ones the real-time preset is a
  poor approximation (the reference is still converged there). This matters for the material
  range in the dataset and for how the classical baseline is reported.
- For compliant materials, edge strain is not a pure error measure, because the true solution
  also stretches. The preset tests use zero compliance (ratio 650×) and `Y = 100 MPa` (ratio 2.3×).
  Position error against the reference is the better measure for compliant materials.
- **A fully mirrored body does not recover.** It un-inverts every tet but gets stuck in a tangled
  state (edges up to 225% stretched) that is a local minimum of distance-plus-volume energy.
  Local inversions (a vertex pushed through its neighbours) recover fully. An inversion-robust
  model (such as stable Neo-Hookean) would fix this, but is outside the current scope.
- In that stuck state, conflicting constraints leave a small constant velocity that damping does
  not remove, because velocity is derived from the solver's corrections. It has not been seen in
  any other scene.
- Linear momentum is conserved to float precision. Angular momentum is only approximately
  conserved: the error shrinks with the substep size and is visible only in violent scenes.
- Gauss–Seidel order introduces a small directional bias and is serial. Multi-threading would
  need graph colouring (ADR 0002), which changes results bit-wise (expected).
- Cost is linear: about 25 ns per constraint projection on one M1 performance core, from 729 to
  50k vertices. No optimization has been done yet; it is deferred until a later phase needs it.
