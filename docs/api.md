# Engine API

The public API of `hp_engine` is the set of headers under `engine/include/hp/`. This page lists
every one with its freeze status (see `decisions/0010-api-freeze.md`) and the guarantees that
are part of the contract. Each header's comments give the details; this page does not repeat
every exception.

**Status:** *frozen* (changes only by approved request; additive changes allowed and logged
below), *provisional* (frozen at the end of the named phase), *internal* (no guarantees).

## Conventions (frozen)

- **Units:** SI throughout: metres, seconds, kilograms, pascals, joules. Angles in scenario files
  are degrees.
- **Types:** `Real` is `float` (float32) and `Index` is `std::int32_t` everywhere. `Vec3` is three
  packed floats, so a `std::vector<Vec3>` is a contiguous `[N, 3]` float32 block.
  Every mesh is limited by int32 indices: the generators reject meshes whose grid points or tets
  exceed 2³¹ − 1, and meshes above about 1.8e8 cells would also overflow the graph's directed
  edge count. Both are far beyond any memory budget.
- **Errors:** invalid arguments throw `std::invalid_argument`, I/O failures throw
  `std::runtime_error`, and misuse of call order throws `std::logic_error`. Nothing returns error
  codes and nothing aborts.
- **Determinism:** no global state, no clocks and no threads in the engine. The same inputs give
  bit-identical results on the supported toolchain. The golden reference results pin this.
- **Ordering:** every list the engine builds (edges, neighbours, surface triangles, vertices
  and tets of the generators) has a fixed, documented order. Downstream code may rely on it.

## Headers

### `hp/core/types.hpp`: frozen
`Real`, `Index`, `Vec3`, `SolverId { Xpbd = 0, Gnn = 1 }` (the stored byte values are part of
the trajectory format).

### `hp/core/rng.hpp`: frozen
`Rng(seed, stream = 0)`, `next_u32`, `uniform()`, `uniform(lo, hi)`, `normal()`,
`split(stream)`. The output sequence is frozen: PCG32 seeded like the reference
`pcg32_srandom_r`. `normal()` uses libm `log`, so its last bit may differ on another libm.

### `hp/core/sim_state.hpp`: frozen
`BodyRange`, `SimState` (all fields: `positions`, `velocities`, `rest_positions`, `inv_mass`,
`region_id`, `solver_owner`, `body_id`, `bodies`), `num_vertices`, `validate`, `all_finite`.
`inv_mass == 0` means pinned.

### `hp/core/state_hash.hpp`: frozen
`state_hash(state)`: FNV-1a over the bytes of `positions`, then `velocities`. The algorithm is
frozen because `ml/data/trajectory.py` reimplements it.

### `hp/version.hpp`: frozen
`version`, `build_info`, `git_commit`, `git_dirty`.

### `hp/mesh/tet_mesh.hpp`: frozen
`Tet`, `TetMesh` (`rest_positions`, `tets`, `num_vertices`, `num_tets`, `validate`), `tet_volume`,
`total_volume`. Tets are positively oriented.

### `hp/mesh/topology.hpp`: frozen
`Edge`, `Triangle`, `Topology` (`edges`, `neighbor_offsets`, `neighbors`, `surface_triangles`,
`num_edges`), `build_topology`. Edges are unique, `i < j`, sorted. Neighbours are CSR and
ascending. Surface triangles are wound outward.

### `hp/mesh/generators.hpp`: frozen
`BoxParams`, `BeamParams`, `SphereParams` and their defaults; `make_box`, `make_beam`,
`make_sphere`. The vertex order (grid order, x fastest) and tet order (cell order, five per cell)
are frozen, since meshes, constraints and recorded datasets all depend on them.

### `hp/mesh/mass.hpp`: frozen
`lumped_masses(mesh, density)`: a quarter of each tet's mass to each of its vertices.

### `hp/mesh/graph.hpp`: frozen, except node features (provisional until Phase 11)
Frozen: `MeshGraph` (`num_nodes`, `senders`, `receivers`, `rest_lengths`, `rel_rest_pos`,
`num_directed_edges`) and `build_graph`. Directed edge `k` is `edges[k]` (`i → j`) and edge
`k + E` is its reverse; `rel_rest_pos = rest[sender] − rest[receiver]`.
Provisional: `NodeType`, `kNumNodeTypes`, `kNodeFeatureDim`, `node_features`. Phase 11 may
add features.

### `hp/collision/collider.hpp`: frozen
`Mat3`, `ColliderKind { Plane = 0, Sphere = 1, Box = 2 }`, `Collider` (fields, `plane`, `sphere`,
`box`, `validate`), `signed_distance<S>`: negative inside, outward unit normal, with the
documented tie rules for points inside a box.

### `hp/solver/xpbd_constraints.hpp`: frozen
`Material` (`density`, `edge_compliance`, `volume_compliance`, `damping`, `validate`),
`XpbdConstraints`, `build_constraints`. Despite the name, `XpbdConstraints` is plain rest-shape
constraint data (edges and tets with rest lengths, rest volumes and compliances). The metrics
use it, and later the GNN residual will too, whichever solver produced the state.

### `hp/solver/xpbd.hpp`: frozen
`kFrameDt` (1/60 s), `XpbdSettings` (`substeps`, `iterations`, `gravity`, `reference()` = 256×1,
`realtime()` = 10×1, `validate`), `XpbdSolver` (both constructors, `step`, `constraints`,
`settings`, `colliders`). `step` currently requires a mask that covers every vertex.

### `hp/solver/solver.hpp`: provisional until Phase 9
`RegionMask` (`all`, `num_vertices`, `num_owned`, `owns`, `covers_all`) and
`ISolver::step(SimState&, const RegionMask&, Real dt)`. Phase 9 adds partial masks and
mixed-ownership stepping.

### `hp/solver/detail/xpbd_kernel.hpp`: internal
The templated kernel. It is public only because `XpbdSolver` holds it by value. Tests use it to
compare float and double.

### `hp/metrics/energy.hpp`, `constraint_error.hpp`, `penetration.hpp`, `frame_metrics.hpp`: frozen
`Energy` + `energy`, `ConstraintError` + `constraint_error`, `Penetration` + `penetration`
(`contact_tolerance` default 1e-4 m), `FrameMetrics` + `measure_frame`. All sums are in double.
Pinned vertices are excluded from kinetic and gravitational energy and from penetration.
Elastic energy is only physical near convergence (see `energy.hpp`).

### `hp/scene/scenario.hpp`: frozen
`kScenarioFormatVersion` (1), `MeshSpec`, `PinBox`, `ScenarioBody`, `Scenario`, `parse_scenario`,
`load_scenario`. `Scenario::seed` is stored and recorded but not yet used. It is reserved for
Phase 10 randomization. The file format itself is in `scenario_format.md`.

### `hp/scene/make_state.hpp`, `hp/scene/pin.hpp`: frozen
`make_state(mesh, density)` (single body at rest, region 0, owned by XPBD) and
`pin_vertices(state, select)`.

### `hp/scene/scene.hpp`: provisional until Phase 9
`Scene` (`mesh`, `topology`, `state`, `solver`, `warnings`) and `build_scene`. `Scene` holds an
`XpbdSolver` by value, and the metrics read constraints and colliders from it. Phase 9 will
likely separate these so a second solver can own regions.

### `hp/io/recorder.hpp`: frozen
`kRecordingOk`, `kRecordingUnfinished`, `RunInfo`, `IRecorder` (`begin`, `write_frame`, `end`).
Frame 0 is the initial state; frames arrive in order without gaps.

### `hp/io/trajectory.hpp`: frozen
`kTrajectoryFormatVersion` (1), `TrajectoryWriter`, `TrajectoryStatic`, `TrajectoryFrame`,
`TrajectoryReader`. The byte layout is in `data_format.md`.

### `hp/io/obj.hpp`: frozen
`write_surface_obj(path, positions, triangles)`.

## Change log

Additive changes to frozen headers, newest first. Start date: the Phase 8 freeze.

| Phase | Header | Change |
|-------|--------|--------|
| 8 (before the freeze) | `core/sim_state.hpp` | Added `all_finite(const SimState&)`, replacing copies in `hp_run` and the viewer |
| 8 (before the freeze) | `mesh/generators.hpp` | Generators also throw for cell counts past the int32 range |
