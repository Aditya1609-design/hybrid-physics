# 0004: Procedural tetrahedral mesh generation

Status: proposed (Phase 2)

## Goal

Generate box, beam and sphere tet meshes in code, with no external mesher, so that meshes are
reproducible from a few parameters and dataset generation can vary them freely.

## Decision

- **Regular grid, 5 tets per cell, alternating split.** Corners are numbered
  `dx + 2*dy + 4*dz`. Cells with even `i+j+k` use a central tet on corners {0, 3, 5, 6} plus four
  corner tets; odd cells use the mirrored split with a central tet on {1, 2, 4, 7}. Neighbouring
  cells then cut their shared face along the same diagonal, so the mesh is conforming.
- The 6-tet (Kuhn) split was rejected: every cell's main diagonal points the same way, which makes
  a distance-constraint material slightly stiffer along that direction. The alternating 5-tet
  split has no single preferred diagonal and uses fewer tets.
- **Orientation**: each tet's signed volume is checked at creation, and two vertices are swapped
  if it is negative. All tets have positive volume (`TetMesh::validate()`).
- **Vertex numbering**: grid order (x fastest), keeping only grid points used by some cell. The
  output depends only on the parameters.
- **Sphere**: keep the cells whose centre lies strictly inside the sphere. The test uses exact
  half-integer offsets in units of the cell size, so the result is mirror-symmetric. The surface
  is blocky. The default of 10 cells per diameter gives 823 vertices and about 5% volume error;
  at 32 cells the error is under 1%.
- **No non-manifold surfaces**: if two kept cells share only an edge, consider the four cells
  around that edge. They form a square, and for any square the sums of squared distances to the
  two pairs of opposite corners are equal. So at least one of the other two cells is also inside
  the sphere. Tests check a closed, manifold surface (Euler characteristic 2) at nine
  resolutions. A vertex-only contact would change the Euler characteristic, so the same test
  would catch it. No cell-removal fallback was needed.

## Known limits

- The voxel sphere's stair-step surface is a poor fit for smooth contact and rolling. If a scenario
  needs a smooth sphere, the option is to snap boundary vertices onto the sphere with an
  inversion check. That is a scope change to request first.
- All cells are axis-aligned and the same size; there is no local refinement.
