# Architecture

This document grows as phases land. Sections marked *(planned)* are placeholders.

## Targets and dependency rule

```
                 +------------------+
                 |   hp_engine      |  static library, headless
                 |  Eigen, json     |  no graphics, no ML
                 +--------+---------+
                          ^
        +-----------------+------------------+-----------------+
        |                 |                  |                 |
   hp_tests        headless_runner        viewer          ML bridge
   (doctest)         (planned)       (planned, raylib)  (planned, ONNX
                                                         Runtime or custom)
```

- Dependencies point one way: apps, tests and the ML bridge depend on the engine, never the
  reverse.
- `cmake/DependencyGuard.cmake` enforces this. Configuring fails if `hp_engine` links anything
  other than Eigen, nlohmann/json and `hp::build_flags`.
- `hp::build_flags` carries the deterministic floating-point flags and warnings
  (see `decisions/0002-determinism-policy.md`). The engine links it PUBLIC so inline engine code
  compiled inside other targets follows the same rules.

## Engine modules

| Module      | Responsibility                                          | Status    |
|-------------|---------------------------------------------------------|-----------|
| `core`      | types, math helpers, RNG, version and build info        | version only |
| `mesh`      | tetrahedral mesh, generators, graph export, partitioner | planned   |
| `solver`    | `ISolver` interface, XPBD, later the GNN solver         | planned   |
| `collision` | ground plane, sphere and box colliders, contacts        | planned   |
| `scene`     | scenario loading, `SimState` setup                      | planned   |
| `metrics`   | energy, constraint error, penetration, timing           | planned   |
| `io`        | trajectory recorder and reader                          | planned   |

## Simulation state *(planned)*

Struct-of-arrays, float32. Per-vertex region ID, solver owner and body ID.

## Solver interface *(planned)*

`ISolver::step(SimState&, const RegionMask&, dt)`, so XPBD and the GNN solver are interchangeable
per region.

## Graph view for the GNN *(planned)*

Node features, edge index list, rest edge lengths, relative rest positions.

## Data flow *(planned)*

Scenario JSON -> scene loader -> SimState -> solver step(s) -> metrics + recorder -> trajectory files
-> Python dataset loader.
