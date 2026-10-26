#pragma once

#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"
#include "hp/solver/xpbd_constraints.hpp"

namespace hp {

// Energy terms in joules, accumulated in double. Pinned vertices (infinite mass) are left out
// of the kinetic and gravitational terms, and zero-compliance constraints out of the elastic one.
struct Energy {
    double kinetic = 0.0;
    double gravitational = 0.0;  // -m g.x, zero at the origin
    double elastic = 0.0;        // sum of C^2 / (2 alpha)

    double total() const { return kinetic + gravitational + elastic; }
};

Energy energy(const SimState& state, const XpbdConstraints& constraints, const Vec3& gravity);

}  // namespace hp
