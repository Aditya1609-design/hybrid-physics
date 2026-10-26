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

Everything else (Eigen, doctest, nlohmann/json) is downloaded by CMake on the first configure
and cached in `.cache/deps/`. After that, the build works offline.

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

## Benchmarks

XPBD frame time for the real-time and reference presets across mesh sizes (release build):

```sh
./build/release/apps/benchmark/hp_xpbd_speed --quick           # three small meshes, about 10 s
./build/release/apps/benchmark/hp_xpbd_speed --csv results/raw/xpbd_speed.csv
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
apps/       benchmark (XPBD speed); viewer and headless_runner in later phases
tests/      unit and regression tests, mirroring engine/
scenarios/  JSON scenario files
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
| 3  | XPBD soft body                          | In review   |
| 4  | Collisions                              | Not started |
| 5  | Scenario system                         | Not started |
| 6  | Viewer                                  | Not started |
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
