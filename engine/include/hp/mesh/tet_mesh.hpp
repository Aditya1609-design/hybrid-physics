#pragma once

#include <array>
#include <vector>

#include "hp/core/types.hpp"

namespace hp {

using Tet = std::array<Index, 4>;

// Rest-shape tetrahedral mesh: vertex positions plus tets as 4 vertex indices each.
// Tets are positively oriented: (b-a) x (c-a) . (d-a) > 0.
struct TetMesh {
    std::vector<Vec3> rest_positions;
    std::vector<Tet> tets;

    Index num_vertices() const { return static_cast<Index>(rest_positions.size()); }
    Index num_tets() const { return static_cast<Index>(tets.size()); }

    // Throws std::invalid_argument if an index is out of range or repeated within a tet,
    // a tet has non-positive volume, or a vertex is not used by any tet.
    void validate() const;
};

// Signed volume of tet t (positive for a correctly oriented tet).
Real tet_volume(const TetMesh& mesh, Index t);

// Sum of all tet volumes, accumulated in double.
double total_volume(const TetMesh& mesh);

}  // namespace hp
