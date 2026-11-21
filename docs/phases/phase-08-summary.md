# Phase 8: Engine correctness and API freeze proposal

Status: approved 2026-11-22 (tag `phase-08-approved`). Branch `phase/08-engine-correctness`.

## What was built

- **Correctness review** of Phases 2–7, riskiest area first, in `phase-08-review.md`. Two bugs
  were found, reported with evidence and fixed after approval:
  1. `7bd68ab` **Trajectory reader.** The bounds check could overflow, so a corrupt header made
     the C++ reader copy from outside its buffer or throw the wrong exception, and made the
     Python loader return wrong-length arrays. Both readers now validate counts, offsets and
     shapes.
  2. `350e862` **Mesh generators.** Cell counts past the int32 range overflowed the grid
     arithmetic. They now throw `std::invalid_argument` before anything is allocated.
- **Regression tests:** vertex-manifold surfaces for sphere sizes 1–40 and a box/beam sweep
  (`4980c94`); angular-momentum drift, first order in the substep (`e7c9483`); corrupt-header and
  generator-limit tests with the two fixes.
- **Golden reference results** (`dc15527`). `tests/regression/golden.json` holds, for all six
  scenarios on the real-time preset (120 frames) and the reference preset (10 frames), a mesh,
  constraint and graph fingerprint, state hashes at frames 0/1/30/60/120 (real-time) or 0/1/5/10
  (reference), and 15 final metrics. Three `hp_tests` cases compare them exactly. `hp_golden`
  checks the file or rewrites it with `--write`.
- **Cleanup (a)** (`95f58bb`). `hp::all_finite(const SimState&)` is now in the engine, used by
  `hp_run` and `SimDriver`.
- **Docs** (`34b9b5b`): `docs/api.md` (API reference with freeze status and change log),
  ADR 0010 (freeze and golden policy), the review, architecture, README, and a
  reader-validation paragraph in `data_format.md` (in `7bd68ab`).

## How to run

```sh
./scripts/ci.sh                                  # release + asan: 135 tests each + trajectory check
./scripts/ci.sh --with-viewer                    # also the viewer smoke run
./build/release/tests/hp_tests -tc="golden*"     # the 3 golden test cases
./build/release/tests/hp_golden                  # prints each run's final hash vs golden.json
./build/release/tests/hp_tests -tc="*manifold*,*angular*,*corrupt*,*int32*"
```

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh` before every commit (8 commits) | PASS, release and asan |
| `ci.sh --with-viewer` after the `all_finite` change | PASS; viewer hashes equal hp_run's |
| Six-scenario 120-frame hashes before and after each code change (both fixes, `all_finite`) | unchanged, release and asan, equal to the Phase 6 table |
| Golden real-time hashes at frame 120 vs `hp_run --frames 120` | identical (= Phase 6 table) |
| Golden reference hashes vs `hp_run --preset reference --frames 10` | identical |
| `hp_golden` from the asan build | all 12 runs match the file written by the release build |
| Golden test with one stored hash changed by one bit | fails, names the scenario and frame, prints the metric table |
| Reader tests with the fix removed | C++ test fails 4 of 6 cases; Python check stops with `ValueError` |
| Vertex-manifold check on two cubes sharing a corner | rejected, so the check is not vacuous |

Golden state hashes after 120 frames (real-time) and 10 frames (reference), M1, Apple clang 21:

| Scenario | Real-time, frame 120 | Reference, frame 10 |
|----------|----------------------|---------------------|
| `ball_on_sphere` | `60e9249da7d258a9` | `412a9cbb0d196757` |
| `beam_drape_box` | `fd7f65ec060e7cdb` | `7889346a0bb64cd2` |
| `cantilever_beam` | `c04db1eb21463a9f` | `5210d8816f3ab6a3` |
| `cube_drop` | `6edfe562d1e9783a` | `55e499393c8c4f50` |
| `cube_slide_slope` | `ea3c4cc353d1554c` | `b45c6824ccf33da0` |
| `cube_throw_spin` | `f41f7c13be72f854` | `63ad56f8f7884bbd` |

## Changes from the approved plan

- **Dropped as duplicates:** the RNG reference-value test, the collider normal/gradient test and
  the graph-symmetry test. All three already existed with the planned checks (see the review).
- **Golden checkpoints include frame 0**, so changes to scene building (rotation, spin, pins)
  show up separately from solver changes. The fingerprint also covers topology, masses and the
  graph view, not only the constraint arrays.
- **Angular momentum is a drift test, not a conservation test**, since XPBD loses angular
  momentum at first order in `h` (as ADR 0005 already said). Bounds 3e-2 (real-time) and 1.5e-3
  (reference), measured 1.5e-2 and 7.2e-4. The halving ratio must be in [0.4, 0.65]; measured 0.56.
- **Beam error message:** non-positive beam cell counts now report `beam cells_along ...` instead
  of `box cells_x ...` (still `std::invalid_argument`). It is a side effect of the generator fix.
- The sphere generator now counts kept cells in a first pass, so the tet limit is exact. The
  extra cost is negligible at our sizes.

## Known limitations

- Golden hashes are exact for Apple clang on Apple Silicon. Scenario rotations and
  `Rng::normal` go through libm, so a different OS may differ in the last bit (review,
  determinism notes). The M4 check has not been run yet.
- Meshes are limited by int32 indices. The generators guard grid points and tets. The graph's
  directed edge count would overflow above about 1.8e8 cells, which is documented but not
  guarded (it needs over 100 GB).
- The golden file covers the six example scenarios only. A new scenario needs `hp_golden --write`
  to add its records.
- No profiling, optimization or scaling benchmark, as the roadmap defers them.

## What to evaluate

1. **The API freeze proposal in `docs/api.md`** (separate sign-off): the frozen and provisional
   lists, the conventions section, and ADR 0010's rule on what counts as additive.
2. **On the teammate's M4:** `./scripts/ci.sh`. The golden tests must pass, and
   `./build/release/tests/hp_golden` must print "all runs match". Any difference is a
   determinism finding.
3. `phase-08-review.md`: whether the depth of each area is enough, especially XPBD and
   contacts.
4. The two fixes (`7bd68ab`, `350e862`) and the golden test's failure output (break one hash in
   a copy of `golden.json` to see it).

## Decisions needed

1. **API freeze sign-off** on `docs/api.md` and ADR 0010 as written.
2. **Graph edge-count limit (note 3 in the review).** Recommendation: document only, as done.
   The alternative is a guard in `build_topology` / `build_graph`, which is a small code change
   for a case that cannot fit in memory.

## Review outcome

- Approved.
- API freeze: signed off as written in `docs/api.md` and ADR 0010. Everything listed as frozen
  is frozen from now on; `solver/solver.hpp` and `scene/scene.hpp` stay provisional until the
  end of Phase 9, and the graph node features until the end of Phase 11.
- Graph edge-count limit: documented only (in `docs/api.md` and the review); no guard in
  `build_topology` or `build_graph`.
