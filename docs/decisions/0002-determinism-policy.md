# 0002: Determinism policy

Status: proposed (Phase 1)

## Goal

Given the same scenario and seed, a `release` build produces bit-identical results on every run,
on Apple Silicon with the same compiler. Datasets, regression tests and error measurements all
rely on this.

## Decision

- All our targets compile with `-ffp-contract=off -fno-fast-math` (through `hp::build_flags`).
  Clang's default on arm64 fuses `a*b+c` into one FMA instruction. Whether that happens depends
  on optimization and surrounding code, so results could change when unrelated code changes.
- A test (`tests/core/test_determinism_flags.cpp`) uses inputs where fused and unfused results
  differ, so it fails if the flag is ever lost. It has been checked to fail when the file is
  compiled without the flag.
- No wall-clock time, global state or unseeded randomness in physics code. The physics will be
  single-threaded at first; if threads are added, constraint graph coloring keeps the order fixed.

## Known limits

- **Eigen's own NEON code uses explicit FMA.** Eigen 3.4's vectorized multiply-add (`pmadd`)
  calls `vfmaq_f32` on arm64. `-ffp-contract=off` does not change explicit intrinsics. This is
  still deterministic run to run, but Eigen's vectorized paths can differ in the last bit from
  the same maths written as plain scalar code. Do not compare results across the two.
- Bit-exactness is guaranteed only for the same compiler version and flags. A different Apple
  clang version may produce different (but individually deterministic) results.
  `scripts/machine_info.sh` and `hp::build_info()` record the toolchain so such differences can
  be spotted.
- The `asan` and `debug` presets use different optimization levels. They are for catching bugs,
  not for producing reference numbers.
