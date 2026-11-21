# Phase 7: Metrics and recording

Status: approved 2026-11-21 (tag `phase-07-approved`). Branch `phase/07-metrics-recording`.

## What was built

- **Git info in the build.** `cmake/GitInfo.cmake` runs on every build and writes the commit and
  a dirty flag into a generated source (rewritten only when they change). Exposed as
  `hp::git_commit()` and `hp::git_dirty()`.
- **Shared metrics and hash in the engine.** `hp::measure_frame` (energy, constraint error,
  penetration in one `FrameMetrics`) and `hp::state_hash` (the FNV-1a final-state hash).
  `hp_run` and the viewer's `SimDriver` now use them instead of their own copies.
- **Energy reported in three parts**: kinetic, gravitational and elastic, in every output (CSV,
  trajectory, overlay). The caveat that elastic energy is only meaningful near convergence is
  stated in `energy.hpp`, in every trajectory header, in `docs/data_format.md` and in the
  README.
- **Recorder.** `hp::IRecorder` (begin / write_frame / end) and `hp::TrajectoryWriter`, which
  streams a `.hptraj` file: a 64-byte preamble, a JSON header (scenario text and the settings
  actually used, seed, colliders, `build_info`, git commit and dirty flag, machine, layout
  tables), a static mesh block (rest positions, masses, region/body IDs, edges, tets, rest
  lengths and volumes, compliances, surface), then one fixed-size record per frame (metrics,
  positions, velocities, solver owner). `hp::TrajectoryReader` reads it back in C++.
- **`hp_run --record run.hptraj`**: frame 0 (the initial state) through frame F, plus machine
  info collected by the app (chip name and core split from sysctl behind `#ifdef __APPLE__`).
- **Python** (numpy only): `ml/data/trajectory.py` (memory-mapped loader, `pairs(k)` for
  t → t+k training pairs as views, `edge_index()`, `state_hash`, `energies`) and
  `ml/tools/check_trajectory.py` (cross-check against hp_run, seven checks).
- **CI**: a "trajectory" step per preset when `.venv` has numpy, otherwise "skipped".
- Docs: `docs/data_format.md` (full spec), ADR 0009, architecture (metrics and recording
  section), README.

## How to run

```sh
./scripts/ci.sh                                   # release + asan: 125 tests each + trajectory check
./scripts/ci.sh --with-viewer                     # also the viewer smoke run
./build/release/tests/hp_tests -sf="*trajectory*" # the 5 recorder/reader test cases
R=./build/release/apps/headless_runner/hp_run
$R scenarios/cube_drop.json --record run.hptraj
.venv/bin/python ml/tools/check_trajectory.py --hp-run $R scenarios/cube_drop.json
.venv/bin/python ml/tools/check_trajectory.py --hp-run $R scenarios/cantilever_beam.json \
    --preset reference --frames 60
```

`hp_run` exit codes are unchanged (0 ok, 1 invalid scenario or unwritable output, 2 usage,
3 non-finite). The status stored in the file is the same code.

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh` before every commit (9 commits) | PASS, release and asan |
| `ci.sh --with-viewer` after the viewer change and at the end | PASS; viewer hashes equal hp_run's |
| `hp_run --frames 120` hashes, six scenarios, with and without `--record` | identical to the Phase 6 table |
| `check_trajectory.py`, all six scenarios (full length) + reference `cantilever_beam` | all 7 checks PASS |
| Python hash of the last recorded frame vs hp_run's printed hash | equal in every run (bit-exact storage) |
| Energies recomputed in numpy vs recorded | worst difference 3.9e-15 of the frame's energy (limit 1e-9) |
| Two recordings of one run (C++ test and Python check) | identical except `step_ms` / `created_utc` |
| Negative checks: flipped bit, truncated file, unfinished file, bad magic (C++ tests; Python by hand) | detected / rejected / read with a warning |
| Git info | a file recorded at `add35e9` says `add35e9…`, `git_dirty: false`; an unchanged rebuild compiles nothing |

**Measured frame record size** (`25 N + 112` bytes, padded to 8):

| Scenario | Vertices | Bytes per frame | File, 120 frames |
|----------|----------|-----------------|------------------|
| `cube_drop` | 729 | 18,344 | 2.37 MB |
| `ball_on_sphere` | 823 | 20,688 | 2.67 MB |
| `cantilever_beam` | 400 | 10,112 | 1.30 MB |
| `beam_drape_box` | 336 | 8,512 | 1.10 MB |
| `cube_slide_slope`, `cube_throw_spin` | 343 | 8,688 | 1.12 MB |

600 frames of `cube_drop` come to 11.2 MB, matching your 18 KB/frame figure (my 36 KB in the
plan was wrong). Recording costs about 4% of run time on `cube_drop` realtime (600 frames:
0.97 s without, 1.01 s with, three runs each).

## Changes from the approved plan

- **`measure_frame(state, constraints, colliders, gravity)`** instead of
  `measure(state, XpbdSolver)`, so the metrics module does not depend on the XPBD class and
  works unchanged for the GNN solver.
- **CSV tolerance per column, not a flat 1e-8.** hp_run prints energies with `%.9g` but errors
  and penetration with `%.6g`, so a flat 1e-8 cannot pass for those columns. The check allows
  half a unit in the last printed digit: 5e-9 relative for energies (stricter than planned),
  5e-7 relative for errors and penetration, 5e-7 s for time, 5e-5 ms for step time, exact for
  integers. The tight test of stored values is the bit-exact hash and the 1e-9 energy recompute.
  See decision 1.
- **The preamble also stores `static_offset`**, since static array offsets are relative to it.
- **Two more checks in `check_trajectory.py`**: a repeat recording must be identical, and the
  numpy snippet in `data_format.md` must read the same positions. The CI step runs on the asan
  build too (13 s), so the writer is sanitizer-checked on real runs.
- **`git_dirty` ignores untracked files** (`machine_info.sh` counts them), so local output
  files do not mark a build dirty.
- **Machine info** adds `logical_cores`; `os` is taken from the product version ("macOS 27.0.1").
- The non-finite frame itself is not recorded: the file ends at the last valid frame with
  status 3.

## Known limitations

- Only `hp_run` records; the viewer does not.
- Field views from the memmap are strided (one record per frame). Copy a window with
  `np.ascontiguousarray` before turning it into a tensor.
- `git_dirty` reflects the tree at build time, not when the binary runs.
- `step_ms` times the solver step only, not metrics or recording.
- No compression (it would break memory mapping); about 25 bytes per vertex per frame.
- Little-endian only; the writer refuses big-endian hosts.
- `rms_penetration_m` is in the trajectory but not in hp_run's CSV (the CSV is unchanged).
- `all_finite` is still duplicated in `hp_run` and `SimDriver` (small; Phase 8 candidate).

## What to evaluate

- `docs/data_format.md`: it is the contract Phases 10–11 build on. Check that the field list,
  units, frame-0 convention and the elastic-energy section say what you expect.
- On your teammate's machine (M4): create `.venv`, run `./scripts/ci.sh`, and confirm the
  trajectory step runs rather than skips.
- Open a recording yourself (README snippet) and look at `traj.header`, especially `machine`
  and `engine`.
- The viewer change (commit `b67e5d5`): small, behaviour-preserving, verified with
  `--with-viewer`.

## Decisions needed

1. **CSV comparison tolerance.** Confirm the per-column print-precision bounds above instead of
   the plan's flat 1e-8 (impossible for the `%.6g` columns). Recommendation: keep them. The
   other option is to print every CSV column with `%.17g`, which changes hp_run's CSV output
   from Phase 5.
2. **Recording from the viewer.** Recommendation: not now; datasets come from `hp_run` (Phase 10
   runs headless). Can be added later behind the same `IRecorder`.

## Review outcome

- Approved.
- CSV tolerance: kept at half a unit in the last printed digit per column; hp_run's CSV output
  is unchanged.
- Viewer recording: not added; datasets are recorded with `hp_run`.
