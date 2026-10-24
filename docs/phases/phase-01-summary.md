# Phase 1: Repo foundation

Status: in review. Branch `phase/01-foundation`. Date: 2026-10-25.
Machine: Apple M1 (4P + 4E cores, 8 GB), macOS 27.0.1, Apple clang 21.0.0, CMake 4.4.4, Ninja 1.13.2.

## What was built

- CMake project with `release`, `debug` and `asan` presets (Ninja, `build/<preset>/`).
- Pinned dependencies (Eigen 3.4.0, doctest 2.4.12, nlohmann/json 3.11.3) from tag archives with
  SHA256 hashes, cached in `.cache/deps/`. See `docs/decisions/0001-build-and-dependencies.md`.
- `hp::build_flags`: `-ffp-contract=off -fno-fast-math`, strict warnings, optional `-Werror`
  and sanitizers. See `docs/decisions/0002-determinism-policy.md`.
- `hp_engine` static library (version and build info only) with a configure-time guard that
  allows only Eigen, nlohmann/json and the build flags.
- doctest test suite: version, dependency smoke tests, FMA-contraction guard (float and double).
- `scripts/ci.sh` (release + asan, `-Werror`, tracked-file checks) and `scripts/machine_info.sh`.
- README with phase status table, architecture skeleton, data format placeholder, data manifest,
  results rules.

## How to run

```sh
./scripts/ci.sh --clean        # full local CI, about 15 s once dependencies are cached
cmake --preset release && cmake --build --preset release && ctest --preset release
```

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh --clean` with system bash 3.2 | PASS: release and asan, 5/5 tests each, no warnings |
| Clean reconfigure after `rm -rf build` | 2 s, no downloads (cache used) |
| FMA guard compiled without `-ffp-contract=off` | fails as intended (results -2^-26 and -2^-54 instead of 0) |
| Engine linked to a dummy library | configure stops with the guard's error; change reverted |
| ASan flags on an out-of-bounds read | heap-buffer-overflow reported |
| `ci.sh` with a 1 KB size limit | reports the oversized files and exits 1 |

## Known limitations

- Eigen's NEON multiply-add uses explicit FMA, which the compiler flag does not affect. Still
  deterministic run to run (ADR 0002).
- Bit-exactness holds for the same compiler version and flags only.
- The dependency guard checks the engine's direct links, not what those libraries link in turn,
  and not raw include paths.
- LeakSanitizer is not available with Apple clang on arm64; the asan preset runs ASan + UBSan only.
- First configure needs network (about 13 MB; Eigen comes from gitlab.com).
- After `ci.sh`, the build folders keep `HP_WARNINGS_AS_ERRORS=ON` in their CMake cache.
- `machine_info.sh` is written for macOS; on other systems some fields read `unknown`.

## What to evaluate

- On the teammate's Mac: clone, run `./scripts/ci.sh`, and check for `CI PASS`.
- Whether the folder layout and the preset names suit you.
- Whether 1 MB is the right limit for tracked files (`HP_MAX_TRACKED_KB`).
