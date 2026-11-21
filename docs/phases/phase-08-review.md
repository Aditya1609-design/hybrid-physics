# Phase 8: Engine correctness review

Review of the engine code from Phases 2–7 (mesh generators, XPBD solver, colliders and
friction, scenario loader and scene builder, metrics, recorder and reader, core), riskiest
first. Each area was read line by line against its header comments and ADR. Anything doubtful
was checked with a probe program or a new test. Bugs were reported with evidence and fixed only
after approval.

## Summary

| # | Area | Finding | Severity | Status |
|---|------|---------|----------|--------|
| 1 | Trajectory reader | Bounds check could overflow; corrupt files read out of bounds (C++) or opened with wrong-length arrays (Python) | Medium | Fixed, `7bd68ab` |
| 2 | Mesh generators | Cell counts past the int32 range overflowed the grid arithmetic (undefined behaviour) | Low | Fixed, `350e862` |
| 3 | Topology / graph | Edge counts of meshes above ~1.8e8 cells would exceed int32 | Note | Not fixed; documented (see below) |

No bugs were found in the XPBD kernel, contacts, friction, scene builder, scenario loader,
metrics, RNG, state hash or recorder.

## 1. XPBD kernel, contacts and friction

- The edge and tet updates match small-steps XPBD (Macklin et al. 2016, 2019): multipliers reset
  every substep, `α / h²` scaling, tet gradients `g1 = (e2 × e3)/6` etc. with `g0 = −Σ gi`, and
  `C = e1 · g1 − V0`. The damping `v /= 1 + damping·h` and the velocity `dx / h` are as ADR 0005
  says.
- **Friction with more than one iteration.** Friction runs inside every iteration, with its limit
  taken from that iteration's push-out, so it could have been applied too often. A probe slid a
  stiff block down a 35° slope for 1 s (10 substeps):

  | μ | Analytic v | 1 it | 2 it | 4 it | 8 it |
  |---|-----------|------|------|------|------|
  | 0.0 | 5.627 | 5.627 | 5.627 | 5.627 | 5.627 |
  | 0.3 | 3.216 | 3.217 | 3.217 | 3.216 | 3.216 |
  | 0.5 | 1.609 | 1.632 | 1.624 | 1.618 | 1.615 |

  More iterations move the result toward the analytic value: once a vertex is pushed out, later
  iterations skip it. Not a bug.
- **A vertex touching two colliders** (ground plane plus a box wall, cube thrown into the
  corner, 120 frames): worst penetration 1.5e-8 m with either collider order.
- **Angular momentum** is not exactly conserved. Each projection conserves it about the
  predicted positions, but the `dx / h` velocity update loses some every substep. For a spinning
  dumbbell, one substep keeps exactly `1 / sqrt(1 + ω²h²)` of it, so the drift is first order in
  `h`. Measured on a spinning, jittered beam without gravity, over 120 frames:

  | Substeps | 5 | 10 | 20 | 40 | 80 | 160 | 256 |
  |----------|---|----|----|----|----|-----|-----|
  | Drift | 2.7e-2 | 1.5e-2 | 8.6e-3 | 4.4e-3 | 2.2e-3 | 1.2e-3 | 7.2e-4 |

  ADR 0005 already lists this. It is a property of the method, now covered by a test (below).

## 2. Scene builder

No bugs. The spin is applied before pinning (so the centre of mass uses every vertex's
mass), the overlap check covers pinned and free vertices as documented, a distance of exactly 0
is allowed, and the two warning formulas match ADR 0006 (with its Phase 5 note on half
thickness).

## 3. Mesh generators and topology

- The existing tests already checked edge-manifold closed surfaces and Euler characteristic for 9
  sphere sizes. **New test:** every surface vertex has a single loop of triangles around it,
  for sphere sizes 1–40 and a sweep of box and beam shapes. Box and beam surfaces lie exactly on
  the box faces, and their area equals the box area. All pass. The voxelized sphere has no
  corner or edge pinches at any size tested. A probe confirmed the check rejects two cubes
  touching at one corner.
- **Finding 2.** `CellGrid::cell()` and the grid point index were computed in int32 with no
  upper limit on cell counts, so `"cells": [1300, 1300, 1300]` passed the loader and overflowed.
  Fix: the generators compute the grid point count and the tet count in int64 and throw
  `std::invalid_argument` before allocating if either exceeds the int32 range. For the sphere,
  the kept cells are counted exactly, so no valid sphere is rejected. Side effect: a beam with
  non-positive `cells_along` / `cells_across` now reports `beam cells_along must be positive`
  instead of `box cells_x must be positive`, still as `std::invalid_argument`.
- **Note 3 (not fixed).** Within the new limits, a box of more than about 1.8e8 cells has more
  than 2³⁰ edges. The graph's `2E` directed edges, and above about 3.6e8 cells `E` itself,
  would then exceed int32. Such a mesh needs well over 100 GB of memory just for its tets and
  faces, so this is documented as a limit of the int32 `Index` (in `docs/api.md`) rather than
  guarded in code.

## 4. Collider signed distance

No bugs. Plane, sphere and box distances and normals were re-derived. The existing test
"normals are unit length and match the distance gradient" already compares normals with
finite-difference gradients at 300 random points per collider kind, including inside the
rotated box. **The planned "collider normals" test commit was dropped** as a duplicate.

## 5. Metrics

No bugs. Elastic energy uses the per-constraint `α` (not `α / h²`), so it is the
`½ C² / α` energy the header describes. Gravitational energy is `−m g · x`. Constraint errors
are relative to rest values computed in double. Penetration skips pinned vertices, and
`num_in_contact` uses the 1e-4 m tolerance as documented.

## 6. Scenario loader

No bugs. Unknown keys, wrong types, out-of-range integers and non-finite numbers all give
`std::invalid_argument` naming the key path. Every default in `docs/scenario_format.md`
matches the code (mesh parameters, material moduli, solver preset, friction).
`seed` must be a non-negative integer and is still unused by the physics.

## 7. Recorder and reader

**Finding 1.** The reader checked `off + e.bytes() > limit` in uint64 and accepted negative
counts. Evidence, with a real `cube_drop` recording and two damaged copies, read by the reader
built with AddressSanitizer and UndefinedBehaviorSanitizer:

| File | Damage | Before the fix |
|------|--------|----------------|
| `bad_offset` | `rest_positions` offset 2⁶⁴ − 16 | UBSan "addition of unsigned offset overflowed"; a normal build would copy from 16 bytes before the buffer |
| `bad_count` | `counts.edges = −1`, matching shapes −1 | `std::length_error` instead of the documented `std::runtime_error` |

The Python loader rejected the first file but opened the second, returning `edges` with 16,184
rows instead of 3,672 (numpy treats −1 as "infer").

Fix: the C++ reader limits counts to `[0, 2³¹ − 1]` and checks `off > limit ||
size > limit − off`. The Python loader rejects negative or non-integer offsets and shapes, and
requires every array sized by a count to match `counts`. A C++ test and check 8 of
`check_trajectory.py` build seven damaged files themselves. Without the fix, the C++ test failed
4 of its 6 cases and the Python check stopped on a raw `ValueError`. The file format is
unchanged.

## 8. Core

No bugs. `Rng` seeding follows `pcg32_srandom_r`, and the existing test already compares its
output with the official pcg32-demo values. **The planned "RNG reference values" test commit
was dropped** as a duplicate. `split` is independent of call order (tested). `state_hash` is
FNV-1a over positions then velocities, matching the Python copy.

## Determinism notes

- Release (-O2) and asan (-O1) builds give identical hashes for all twelve golden runs.
- Two inputs go through libm, whose results are not guaranteed bit-identical across platforms:
  scenario rotations (`Eigen::AngleAxisd`, `sin`/`cos` in double) and `Rng::normal` (`log`). On
  one macOS version every Apple Silicon machine uses the same libm, so the M1 and M4 should agree.
  A different OS or libm could change the last bit of a rotated scenario
  (`beam_drape_box`, `cube_slide_slope`) or of a normal sample. The teammate's M4 run of
  `ci.sh` checks the first case. Phase 10 should keep this in mind if it draws normals.

## Tests added

| Commit | Test |
|--------|------|
| `4980c94` | Vertex-manifold surfaces (sphere 1–40, box and beam sweep), box face coverage |
| `7bd68ab` | Reader rejects seven kinds of corrupt header (C++ test and Python check 8) |
| `350e862` | Generators reject cell counts just past the int32 range; `build_scene` reports it |
| `e7c9483` | Angular-momentum drift is first order in the substep (bounds 3e-2 real-time, 1.5e-3 reference) |
| `dc15527` | Golden reference results for all six scenarios on both presets |
| `95f58bb` | `all_finite` flags inf and NaN |

The planned graph-symmetry test was not added either: the existing "graph has paired directed
edges with consistent rest geometry" test already checks that directed edge `k + E` reverses
edge `k`, that `rel_rest_pos` flips sign and that rest lengths match.
