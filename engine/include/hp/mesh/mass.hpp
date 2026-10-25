#pragma once

#include <vector>

#include "hp/core/types.hpp"
#include "hp/mesh/tet_mesh.hpp"

namespace hp {

// Lumped vertex masses: each tet's mass (density * volume) is split equally over its four
// vertices. Throws std::invalid_argument if density is not positive.
std::vector<Real> lumped_masses(const TetMesh& mesh, Real density);

}  // namespace hp
