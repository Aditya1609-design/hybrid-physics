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
| `core`      | types, RNG, `SimState`, version and build info          | Phase 2   |
| `mesh`      | tet mesh, generators, topology, masses, graph export    | Phase 2 (partitioner planned) |
| `solver`    | `ISolver` interface, XPBD, later the GNN solver         | Phase 3 (XPBD) |
| `collision` | plane, sphere and oriented box colliders, signed distance | Phase 4 |
| `scene`     | `SimState` setup (`make_state`, `pin_vertices`); scenario loading later | Phase 2–3 (partial) |
| `metrics`   | constraint error, energy, penetration; recording later  | Phase 3–4 (partial) |
| `io`        | surface OBJ writer; trajectory recorder and reader later | Phase 2 (partial) |

## Mesh and simulation state

See `decisions/0003-data-layout.md` and `decisions/0004-tet-mesh-generation.md`.

```
make_box / make_beam / make_sphere --> TetMesh (rest_positions [N,3], tets [T,4])
                                          |
                    +---------------------+----------------------+
                    v                     v                      v
             build_topology          lumped_masses          make_state
        (edges, CSR neighbours,      (density * vol / 4)    (scene module)
         surface triangles)                 |                    |
                    |                       +------->  SimState  <+
                    v                                     |
               build_graph  --> MeshGraph            node_features --> [N,5]
                                (edge_index, rest lengths, rel. rest positions)
```

- `TetMesh`: rest positions and positively oriented tets. Immutable once generated.
- `Topology`: sorted unique edges (`i < j`), CSR vertex neighbours, outward-wound surface
  triangles. Built by sorting, so the order is deterministic.
- `SimState`: struct-of-arrays, float32, each `Vec3` field a contiguous `[N,3]` block. Per-vertex
  positions, velocities, rest positions, inverse mass (0 = pinned), region ID, solver owner and
  body ID, plus contiguous body ranges. `validate()` checks all of it.
- `core` depends on nothing else in the engine; `mesh` depends on `core`; `scene` and `io` depend
  on both. `collision` depends only on `core`. `solver` depends on `core`, `mesh` and
  `collision`; `metrics` on `core`, `mesh`, `collision` and the solver's constraint data.

## Solver

See `decisions/0005-xpbd-solver.md`.

```
TetMesh + Topology + Material --> build_constraints --> XpbdConstraints
                                                      (edges, rest lengths, compliances;
                                                       tets, rest volumes, compliances)
                                                              |
XpbdSettings::realtime() / ::reference() ----------> XpbdSolver : ISolver
std::vector<Collider> (plane, sphere, box) ---------->        |
       SimState --step(state, RegionMask::all(n), kFrameDt)--> SimState
                                                              |
            constraint_error, energy, penetration  <----------+
```

- `ISolver::step(SimState&, const RegionMask&, dt)`, so XPBD and the GNN solver are
  interchangeable per region. `RegionMask` only supports "all vertices" for now.
- `XpbdSolver` owns its constraints and scratch arrays; it reads masses from `SimState` and never
  allocates after the first step. The kernel is a template so the same code can run in double
  for precision checks.
- Each Gauss–Seidel iteration projects edges, then tets, then collider contacts (hard,
  one-sided, with positional Coulomb friction). Contacts go last, so every substep ends with
  vertices outside the colliders. See `decisions/0006-contacts-and-friction.md`.
- Colliders are static and live in `collision`, not in the solver, so the GNN solver can read
  the same list later.
- The two presets differ only in substep and iteration counts. Both use the fixed frame step
  `kFrameDt = 1/60 s`.
- Timing is measured only in apps (`apps/benchmark/hp_xpbd_speed`), never inside the engine.

## Graph view for the GNN

`MeshGraph` (MeshGraphNets layout): every edge in both directions, forward edges first and
their reverses second. `senders`/`receivers` form `edge_index [2, 2E]`; `rest_lengths [2E]` and
`rel_rest_pos [2E, 3]` (rest[sender] − rest[receiver]) are the static edge features.
`node_features` writes `[N, 5]` = velocity + one-hot (free, pinned) into a caller buffer. The
node feature layout is provisional until the GNN phase.

## Data flow *(planned)*

Scenario JSON -> scene loader -> SimState -> solver step(s) -> metrics + recorder -> trajectory files
-> Python dataset loader.
