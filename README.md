# Hybrid Physics

Uncertainty-gated hybrid simulation of soft-body dynamics. A classical XPBD solver and a
graph neural network (MeshGraphNets-style) run side by side, and a controller decides per mesh
region and per timestep which of the two computes the physics, based on the network's
predicted uncertainty. B.Tech final-year research project, Delhi Technological University.

The engine is a headless C++17 static library with no graphics or ML dependencies. The viewer,
command-line tools and the ML bridge are separate targets built on top of it.

## Prerequisites

macOS on Apple Silicon, with:

- Xcode Command Line Tools (Apple clang)
- CMake 3.20 or newer and Ninja: `brew install cmake ninja`

Everything else (Eigen, doctest, nlohmann/json, and raylib for the viewer) is downloaded by
CMake on the first configure and cached in `.cache/deps/` (about 55 MB). After that, the build
works offline. To build without the viewer (for example on a machine without a desktop OpenGL
stack), configure with `-DHP_BUILD_VIEWER=OFF`.

## Build and test

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
```

Presets: `release` (-O2, deterministic floating point), `debug`, and `asan`
(AddressSanitizer + UndefinedBehaviorSanitizer). Build output goes to `build/<preset>/`.

Full local CI (release + asan, warnings as errors, tracked-file checks):

```sh
./scripts/ci.sh            # add --clean for a fresh build, --with-debug to add the debug preset
```

To record the machine a result came from:

```sh
HP_MACHINE_LABEL=my-m1 ./scripts/machine_info.sh -o results/<run>/machine_info.json
```

## Running a scenario

Scenarios are JSON files in `scenarios/` (format: `docs/scenario_format.md`). The headless
runner prints energy, constraint error, penetration and step time, and ends with a hash of the
final state; the same build on the same compiler gives the same hash on every run.

```sh
./build/release/apps/headless_runner/hp_run scenarios/cube_drop.json
./build/release/apps/headless_runner/hp_run scenarios/cube_drop.json --preset reference --frames 60
./build/release/apps/headless_runner/hp_run scenarios/cube_drop.json --csv out.csv --surface-obj final.obj
./build/release/apps/headless_runner/hp_run my_scenario.json --check     # validate only
```

## Viewer

Runs a scenario live in a window, with the same stepping as `hp_run` (frame N is identical):

```sh
./build/release/apps/viewer/hp_view scenarios/beam_drape_box.json
./build/release/apps/viewer/hp_view scenarios/cube_drop.json --preset reference --paused
./build/release/apps/viewer/hp_view scenarios/cube_drop.json --frames 120   # prints the hash
./build/release/apps/viewer/hp_view scenarios/cube_drop.json --frames 120 --screenshot shot.png
```

Mouse: left drag orbits, right drag (or shift + left drag) pans, wheel zooms. Press H in the
window for all keys: space pause, `.` step, R reset, L reload the file, W solid/wire, T tet
edges, C color mode (solver owner, region, body, plain), P display-only owner preview,
V follow the body, F frame the scene, Tab stats panel, F12 screenshot.

## Benchmarks

XPBD frame time for the real-time and reference presets across mesh sizes (release build):

```sh
./build/release/apps/benchmark/hp_xpbd_speed --quick           # three small meshes, about 10 s
./build/release/apps/benchmark/hp_xpbd_speed --csv results/raw/xpbd_speed.csv
./build/release/apps/benchmark/hp_xpbd_speed --quick --colliders   # adds rows with 3 colliders
```

## Python tools

Python 3.11+ in a local, git-ignored virtual environment:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r ml/requirements.txt
.venv/bin/python ml/tools/gnn_cost_estimate.py --flops-only    # GNN cost estimate, no PyTorch needed
```

## Repository layout

```
engine/     headless static library (include/hp/ public headers, src/ by module)
apps/       benchmark (XPBD speed), headless_runner (hp_run), viewer (hp_view, raylib)
tests/      unit and regression tests, mirroring engine/ (plus tests/viewer)
scenarios/  JSON scenario files (six examples)
ml/         Python tools (GNN cost estimate); training and evaluation in later phases
cmake/      compiler settings, dependency fetching, engine dependency guard
scripts/    local CI and machine info
docs/       architecture, design decisions, data format, per-phase summaries
data/       generated datasets (git-ignored except MANIFEST.md)
results/    benchmark summaries, plots and tables
```

## Phase status

| #  | Phase                                   | Status      |
|----|-----------------------------------------|-------------|
| 1  | Repo foundation                         | Approved    |
| 2  | Core and mesh                           | Approved    |
| 3  | XPBD soft body                          | Approved    |
| 4  | Collisions                              | Approved    |
| 5  | Scenario system                         | Approved    |
| 6  | Viewer                                  | In review   |
| 7  | Metrics and recording                   | Not started |
| 8  | Engine correctness and API freeze       | Not started |
| 9  | Regions and solver interface            | Not started |
| 10 | Dataset generation                      | Not started |
| 11 | GNN training                            | Not started |
| 12 | Speed-crossover check                   | Not started |
| 13 | Uncertainty signals                     | Not started |
| 14 | C++ inference bridge                    | Not started |
| 15 | Switching controller                    | Not started |
| 16 | Benchmark and analysis                  | Not started |
| 17 | Paper support                           | Not started |
