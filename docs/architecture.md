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
   hp_tests        headless_runner   hp_viewer_core       ML bridge
   (doctest)         (hp_run)        (no graphics)      (planned, ONNX
                                          ^              Runtime or custom)
                                          |
                                       hp_view
                                       (raylib)
```

- Dependencies point one way: apps, tests and the ML bridge depend on the engine, never the
  reverse.
- `cmake/DependencyGuard.cmake` enforces this. Configuring fails if `hp_engine` links anything
  other than Eigen, nlohmann/json and `hp::build_flags`.
- The viewer is optional (`HP_BUILD_VIEWER`, on by default). raylib is linked by `hp_view`
  only; `hp_viewer_core` is plain C++ on the engine, so its tests run in every build.
- `hp::build_flags` carries the deterministic floating-point flags and warnings
  (see `decisions/0002-determinism-policy.md`). The engine links it PUBLIC so inline engine code
  compiled inside other targets follows the same rules.

## Engine modules

| Module      | Responsibility                                          | Status    |
|-------------|---------------------------------------------------------|-----------|
| `core`      | types, RNG, `SimState`, state hash, version, build and git info | Phase 2, 7 |
| `mesh`      | tet mesh, generators, topology, masses, graph export    | Phase 2 (partitioner planned) |
| `solver`    | `ISolver` interface, XPBD, later the GNN solver         | Phase 3 (XPBD) |
| `collision` | plane, sphere and oriented box colliders, signed distance | Phase 4 |
| `scene`     | `SimState` setup (`make_state`, `pin_vertices`), scenario parsing and `build_scene` | Phase 2–5 |
| `metrics`   | constraint error, energy, penetration, `measure_frame`  | Phase 3–4, 7 |
| `io`        | surface OBJ writer, `IRecorder`, trajectory writer and reader | Phase 2, 7 |

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
  The scenario loader in `scene` also uses `collision` and `solver`, and is the only engine code
  that includes nlohmann/json.

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

## Scenarios

See `scenario_format.md` and `decisions/0007-scenario-format.md`.

```
scenario.json --load_scenario--> Scenario (plain struct: mesh params, material, pose,
                                  velocities, pin boxes, solver settings, colliders, seed)
                                      |
                                 build_scene
       generator -> rotate/place -> make_state -> v + w x r -> pins -> overlap check
                                      |                                 (throws)
                                      v
            Scene { TetMesh, Topology, SimState, XpbdSolver, warnings }
                                      |
     hp_run: step, metrics, optional CSV, OBJ and trajectory, final state hash
```

- Parsing checks everything that needs no mesh. `build_scene` does the geometric checks:
  initial overlap with a collider is an error, and small or thin colliders produce warnings.
- The engine never prints. Warnings go back in `Scene::warnings` for the app to show.

## Viewer

See `decisions/0008-viewer.md`.

```
scenario.json -> load_scenario -> SimDriver (owns the Scene, same stepping as hp_run)
                                      |  advance(wall dt): FixedStepPacer -> 0..4 steps of kFrameDt
                                      v
   SimState -> vertex_colors(owner | region | body | plain, or display-only preview)
            -> build_surface -> CPU Lambert shading -> raylib mesh update -> draw
   solver.colliders() -> ground grid, translucent spheres and boxes
   measure_frame -> stats overlay
```

- Wall time decides only how many fixed steps run, so the viewer's frame N is bit-identical to
  `hp_run`'s; `hp_view --frames N` prints the same final state hash.
- Nothing in the viewer writes to `SimState`; the owner/region preview lives in the viewer's
  color buffer.

## Metrics and recording

See `decisions/0009-trajectory-format.md` and `data_format.md`.

```
Scene --step--> SimState --measure_frame--> FrameMetrics {energy, error, penetration}
                    |                              |
                    |       (app) step_ms, machine info, timestamp
                    v                              v
            IRecorder::write_frame(frame, state, metrics, step_ms)
                    |
            TrajectoryWriter --> run.hptraj
                                 preamble | header JSON | static block | frame 0 | frame 1 | ...
                                     |
            ml/data/trajectory.py (np.memmap) --> positions [F,N,3], pairs(k), edge_index
            ml/tools/check_trajectory.py      --> cross-check against hp_run (ci.sh)
```

- `measure_frame` takes constraint data, colliders and gravity rather than a solver, so it
  works the same for any solver that produced the state. `hp_run`, the viewer and the recorder
  all use it, and `state_hash` gives all of them the same final-state hash.
- The engine never reads a clock or the machine: step times, the timestamp and machine info
  come from the app. The git commit is compiled in (`cmake/GitInfo.cmake`, refreshed on every
  build).
- Frame 0 is the initial state. A run that ends non-finite stops at the last valid frame with
  status 3; a crashed run leaves an "unfinished" file that readers can still open.

## Data flow

Scenario JSON -> scene loader -> SimState -> solver step(s) -> metrics + recorder -> trajectory
files -> Python loader. Everything up to the Python loader exists; datasets (Phase 10) and the
GNN (Phase 11) build on it.
