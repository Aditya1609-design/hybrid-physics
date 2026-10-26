#pragma once

#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"
#include "hp/solver/xpbd_constraints.hpp"

namespace hp {

// How far the current positions are from the rest shape, as relative errors:
// edges |l - L| / L, tets |V - V0| / V0. Sums are taken in double.
struct ConstraintError {
    double edge_rms = 0.0;
    double edge_max = 0.0;
    double volume_rms = 0.0;
    double volume_max = 0.0;
    Index inverted_tets = 0;  // tets with V <= 0
};

ConstraintError constraint_error(const SimState& state, const XpbdConstraints& constraints);

}  // namespace hp
