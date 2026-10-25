#pragma once

#include <array>
#include <vector>

#include "hp/core/types.hpp"
#include "hp/mesh/tet_mesh.hpp"

namespace hp {

using Edge = std::array<Index, 2>;
using Triangle = std::array<Index, 3>;

// Connectivity derived from a TetMesh. Every list is sorted, so the result is deterministic.
struct Topology {
    // Unique undirected edges with edge[0] < edge[1], sorted lexicographically.
    std::vector<Edge> edges;

    // Vertex neighbours in CSR form: the neighbours of v are
    // neighbors[neighbor_offsets[v] .. neighbor_offsets[v+1]), in increasing order.
    std::vector<Index> neighbor_offsets;
    std::vector<Index> neighbors;

    // Boundary faces (used by exactly one tet), wound counter-clockwise seen from outside,
    // so (b-a) x (c-a) points out of the body.
    std::vector<Triangle> surface_triangles;

    Index num_edges() const { return static_cast<Index>(edges.size()); }
};

// Builds edges, neighbours and the surface. Expects a valid, positively oriented mesh.
// Throws std::invalid_argument if a face is shared by more than two tets.
Topology build_topology(const TetMesh& mesh);

}  // namespace hp
