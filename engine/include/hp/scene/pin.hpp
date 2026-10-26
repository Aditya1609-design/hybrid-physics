#pragma once

#include <functional>

#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"

namespace hp {

// Pins every vertex whose rest position satisfies `select`: inverse mass becomes 0 and
// velocity is zeroed, so solvers never move it. Returns the number of vertices pinned.
Index pin_vertices(SimState& state, const std::function<bool(const Vec3&)>& select);

}  // namespace hp
