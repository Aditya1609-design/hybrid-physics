#pragma once

#include <vector>

#include "hp/collision/collider.hpp"
#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"

namespace hp {

// How far free vertices sit inside colliders, in metres. A vertex's depth is its deepest
// penetration over all colliders. Pinned vertices are skipped, since no solver moves them.
// Distances are evaluated in double.
struct Penetration {
    double max_depth = 0.0;
    double rms_depth = 0.0;      // over penetrating vertices only; 0 if there are none
    Index num_penetrating = 0;   // vertices with signed distance < 0
    Index num_in_contact = 0;    // vertices with signed distance <= contact_tolerance
};

Penetration penetration(const SimState& state, const std::vector<Collider>& colliders,
                        double contact_tolerance = 1e-4);

}  // namespace hp
