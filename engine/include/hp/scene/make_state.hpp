#pragma once

#include "hp/core/sim_state.hpp"
#include "hp/mesh/tet_mesh.hpp"

namespace hp {

// SimState for a single body at rest: positions = rest positions, zero velocity, lumped
// inverse masses from `density`, region 0, owned by XPBD, body 0.
// Throws std::invalid_argument if the mesh is invalid or density is not positive.
SimState make_state(const TetMesh& mesh, Real density);

}  // namespace hp
