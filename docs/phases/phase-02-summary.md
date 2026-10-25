# Phase 2: Core and mesh

Status: approved 2026-10-26 (tag `phase-02-approved`). Branch `phase/02-core-mesh`.

## What was built

- `core`: `Real` (float32), `Index` (int32), packed `Vec3`, `SolverId`; PCG32 `Rng` with its own
  `uniform`, `normal` and order-independent `split`; `SimState` (struct-of-arrays with region ID,
  solver owner, body ID and body ranges) with `validate()`.
- `mesh`: `TetMesh` with `validate()`, `tet_volume`, `total_volume`; `make_box`, `make_beam`,
  `make_sphere` (5-tet alternating grid split; voxelized sphere); `build_topology` (sorted unique
  edges, CSR neighbours, outward surface triangles); `lumped_masses`; `build_graph` and
  `node_features` for GNN input.
- `scene`: `make_state(mesh, density)` builds a single-body SimState at rest.
- `io`: `write_surface_obj` for checking meshes in an external viewer.
- ADRs: 0003 data layout, 0004 mesh generation; ADR 0002 now covers the RNG.

## Example meshes (default parameters)

| Mesh   | Vertices | Tets | Edges | Surface tris | Directed graph edges | Volume |
|--------|---------:|-----:|------:|-------------:|---------------------:|-------:|
| box 1×1×1, 8³ cells | 729 | 2560 | 3672 | 768 | 7344 | 1.0000 |
| beam 2×0.5×0.5, 20×5×5 | 756 | 2500 | 3705 | 900 | 7410 | 0.5000 |
| sphere r=0.5, 10 cells | 823 | 2760 | 4062 | 960 | 8124 | 0.5520 (exact 0.5236) |

Topology plus graph construction takes about 3.5 ms per mesh (one-time setup, release build).

## How to run

```sh
./scripts/ci.sh                    # release + asan, 31 tests each
ctest --preset release             # after building
./build/release/tests/hp_tests -tc="*sphere*" -s   # prints sphere sizes and volume errors
```

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh` before every commit (7 commits) | PASS, release and asan, no warnings |
| RNG against the official pcg32-demo output (seed 42, stream 54) | identical |
| Interior faces shared by exactly two tets; boundary face counts | as expected for box, beam, sphere |
| Solid Euler characteristic V − E + F − T = 1 | holds for box, beam, 9 sphere resolutions |
| Surface closed, consistently wound, Euler characteristic 2 | holds for all of the above |
| Outward winding: surface-enclosed volume = tet volume sum | within 1e-4 relative |
| Graph: reverse pairs, antisymmetric offsets, lengths match positions | holds |
| Generators and graph export run twice | byte-identical |

## Changes from the approved plan

- `make_state` is in `scene/`, not `core/`, so `core` does not depend on `mesh`.
- `lumped_masses` returns masses; `make_state` inverts them (the plan named `lumped_inv_mass`).
- Default sphere resolution is 10 cells per diameter (823 vertices) instead of 12, which gives
  1285 vertices. My earlier estimate of about 700 at 12 cells was wrong.
- `types.hpp` includes `Eigen/Geometry` so `Vec3::cross` links wherever `Vec3` is used.
- The non-manifold fallback for the sphere was not needed; ADR 0004 explains why it cannot occur
  for edge contacts, and tests confirm it at nine resolutions.

## Known limitations

- Voxel sphere is blocky (about 5% volume error at the default resolution).
- Node feature layout `[N, 5]` is provisional; the GNN phase will likely add fields.
- `Rng::normal` uses `std::log`, which may differ in the last bit on another OS's maths library.
- Only one body is created by `make_state`; there is no function to append a second body yet
  (the data layout supports it).
- `TetMesh::validate` checks orientation and indices but not degenerate (near-zero volume) tets.

## What to evaluate

- Open the meshes in Preview or Blender: write them with `hp::write_surface_obj` (as in
  `tests/io/test_obj.cpp`) and check the box, beam and sphere surfaces look right.
- Whether the data layout in ADR 0003 (interleaved `[N,3]`, int32, forward-then-reverse edge
  order) suits the planned Python loader.
- Whether `make_state` belongs in `scene/` and the default mesh sizes suit you.
- On the teammate's Mac: `./scripts/ci.sh` shows `CI PASS`.

## Review outcome

- Approved, including the three changes from the plan (`make_state` in `scene/`, sphere default
  of 10 cells per diameter, `lumped_masses` returning masses).
