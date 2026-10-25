# 0003: Simulation data layout

Status: proposed (Phase 2)

## Goal

The engine's arrays should go to the GNN (Python now, C++ inference later) without conversion
or copying, and later features (regions, a second solver, more bodies) should not force a change
to the core data structures.

## Decision

- **float32 everywhere** (`hp::Real = float`). The GNN works in float32, so engine data maps
  directly onto tensors. Sums over many elements (volume, mass) are accumulated in double and
  then stored as float.
- **int32 indices** (`hp::Index = std::int32_t`). Half the memory of int64 and plenty for our
  mesh sizes. PyTorch's `edge_index` wants int64; widening on load costs very little.
- **Vector fields are interleaved `[N,3]` arrays**: `std::vector<hp::Vec3>`, where `Vec3` is a
  packed 3-float Eigen vector (12 bytes, no alignment padding; a `static_assert` checks it).
  `data()` is a row-major `[N,3]` float block, the same layout as a tensor. Each field
  (positions, velocities, rest positions) is its own array, so the layout is struct-of-arrays at
  the field level.
- **SimState holds only per-vertex state**: positions, velocities, rest positions, inverse mass
  (0 = pinned), region ID, solver owner (`SolverId`, one byte), body ID, plus contiguous
  `BodyRange`s. Topology (tets, edges, surface) lives in `TetMesh` and `Topology`. Solver scratch
  (such as predicted positions) lives inside the solver.
- **Region, owner and body are first-class from the start.** With one body and only XPBD they are
  all zero, but every later phase can rely on them being present and validated.
- **Multiple bodies** use consecutive vertex ranges. `SimState::validate()` checks that the ranges
  tile the vertices in order and that `body_id` matches. Adding a body later means appending a
  range, not changing the structure.
- **Graph view (GNN input)** follows MeshGraphNets: every undirected edge appears in both
  directions, forward edges first and their reverses second (directed edge `k + E` is the reverse
  of `k`). `senders` and `receivers` together form `edge_index [2, 2E]`; edge data is
  `rest_lengths [2E]` and `rel_rest_pos [2E, 3]` = rest[sender] − rest[receiver].
- **Node features** are written row-major `[N, 5]`: velocity (3) and a one-hot node type
  (free, pinned). This layout is provisional; the GNN phase may extend it.

## Consequences

- Python can read engine arrays with `numpy.frombuffer(..., dtype=float32).reshape(-1, 3)`.
- Pure SoA (separate x, y, z arrays) would allow slightly easier SIMD loops but needs a copy or
  transpose for every tensor hand-off. We chose tensor compatibility.
- Index counts are limited to about 2 billion, far beyond our mesh sizes.
