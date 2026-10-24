# 0001: Build system and dependencies

Status: proposed (Phase 1)

## Decision

- CMake (minimum 3.20) with Ninja, driven by `CMakePresets.json` (`release`, `debug`, `asan`).
- Dependencies come from FetchContent, each pinned to a tag archive and verified by SHA256:

  | Dependency    | Version | Source                                   | Use                  |
  |---------------|---------|------------------------------------------|----------------------|
  | Eigen         | 3.4.0   | gitlab.com tag archive (Eigen is hosted on GitLab) | math (engine) |
  | doctest       | 2.4.12  | github.com tag archive                   | unit tests           |
  | nlohmann/json | 3.11.3  | github.com tag archive                   | scenario configs     |

- Tag archives (`/archive/refs/tags/...`) are used instead of release-asset downloads, which are
  very slow on some networks.
- Archives are stored in `HP_DEPS_CACHE_DIR` (default `.cache/deps/`, git-ignored) through
  FetchContent's `DOWNLOAD_DIR`. Only the archives are shared; each build directory extracts and
  builds its own copy, so presets with different flags never share build output.
- Eigen is download-only and wrapped in our own `hp::eigen` INTERFACE target. Its own CMake
  project configures tests and BLAS checks we do not need.
- doctest was chosen over GoogleTest: a single header, faster to compile, and enough for our needs.
- raylib is added in the viewer phase, not earlier.
- Third-party include directories are marked as system includes, so our strict warnings
  (`-Wconversion` and others, `-Werror` in CI) apply only to our code.

## Consequences

- The first configure needs network access and downloads about 13 MB (the json tag archive is
  about 8 MB because it includes test data). Later configures, including after `rm -rf build`,
  use the cache.
- doctest 2.4.12 declares `cmake_minimum_required(3.5)`, which CMake 4 reports as deprecated.
  `CMAKE_POLICY_VERSION_MINIMUM=3.10` is set around the dependency fetch only. If a future CMake
  stops accepting this, update doctest or patch its minimum.
- Release builds use `-O2` (CMake's default is `-O3`). This is set for the whole project, but all
  dependencies are currently header-only, so in practice it only affects our code.
