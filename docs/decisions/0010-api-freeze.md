# 0010: Engine API freeze and golden reference results

Status: proposed (Phase 8)

## Goal

From Phase 9 on, the dataset pipeline, the GNN and the C++ inference bridge are built on the
engine. They need its interfaces, file formats and physics results to stay fixed, and any
change to them to be deliberate and visible.

## Decision

- **Three levels**, listed per header in `docs/api.md`:
  - *Frozen*: the declarations, their documented behaviour (units, ordering, exceptions,
    determinism) and, where stated, their output (mesh vertex and tet order, RNG sequence,
    state hash algorithm).
  - *Provisional*: expected to change in a named later phase, then frozen. `solver/solver.hpp`
    (`RegionMask`, `ISolver`) and `scene/scene.hpp` (`Scene`, `build_scene`) are frozen at the
    end of Phase 9. The node-feature part of `mesh/graph.hpp` is frozen at the end of Phase 11.
  - *Internal*: `hp/solver/detail/` and everything under `engine/src/`. These are public only
    because a frozen class holds them by value; they may change freely.
- **Formats are part of the freeze.** Scenario format version 1 (`docs/scenario_format.md`) and
  trajectory format version 1 (`docs/data_format.md`). Their own versioning rules still apply:
  appending header keys or layout fields keeps version 1, anything else bumps the version.
- **Allowed without a request (additive):** new functions, types and headers; new struct fields
  added at the end with a default that keeps old behaviour; new enum values; new overloads that
  do not make existing calls ambiguous. Each one gets a line in the change log in
  `docs/api.md`. Behaviour of frozen items for inputs they already accepted must not change.
- **Everything else is a change request.** Removing or renaming anything, changing a signature,
  default, unit, ordering or exception type, or changing results for valid inputs needs an
  explicit request with a justification, and approval before it is made.
- **Golden reference results** (`tests/regression/golden.json`) pin the physics. For each example
  scenario, on the real-time preset (120 frames) and the reference preset (10 frames), the file
  holds a mesh and constraint fingerprint, state hashes at several frames, and the final metrics.
  The test compares hashes exactly. On a mismatch it prints every metric's old and new value, so
  the report says what moved. The file is rewritten only with `hp_golden --write`, after the
  physics change has been approved, and the diff is committed with that change. Adding a
  scenario also needs a rewrite, which adds records without changing the existing ones.
- **Exact hashes, no tolerance mode.** The supported toolchain is Apple clang on Apple
  Silicon. A hash difference on a supported machine is a determinism bug to investigate, not a
  threshold to loosen.

## Alternatives considered

- **Freeze everything now, including `ISolver` and `Scene`.** Phase 9 exists to add partial
  masks and mixed ownership, so it would start with change requests against a fresh freeze.
- **Golden metrics with tolerances instead of hashes.** They would survive compiler changes, but
  they miss small physics changes that hashes catch, and choosing tolerances is guesswork.
  The metrics are stored anyway, as the explanation on a mismatch.
- **Golden trajectory files.** Every frame would be covered, but at about 2 MB per run they are
  too large to track in git, and a hash per checkpoint already detects any change.

## Consequences and limits

- Profiling and optimization (deferred to Phase 12 or later) must keep every golden hash. A change
  that alters float rounding, for example vectorizing a sum, is a physics change and needs approval.
- Hashes depend on libm for scenario rotations (`sin`/`cos`) and for `Rng::normal` (`log`).
  A different OS or libm may need its own investigation. See the determinism notes in
  `docs/phases/phase-08-review.md`.
- The golden tests add about 2.5 s to the release test run and about 10 s to asan.
