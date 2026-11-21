#pragma once

#include <vector>

#include "hp/collision/collider.hpp"
#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"
#include "hp/metrics/constraint_error.hpp"
#include "hp/metrics/energy.hpp"
#include "hp/metrics/penetration.hpp"
#include "hp/solver/xpbd_constraints.hpp"

namespace hp {

// Every per-frame metric of one state: what hp_run prints, the viewer shows and the
// trajectory recorder stores. Step time is not here; the engine never reads a clock.
struct FrameMetrics {
    Energy energy;
    ConstraintError error;
    Penetration penetration;
};

// Evaluates all metrics. Takes the constraint data and colliders rather than a solver, so it
// works the same whichever solver produced the state.
FrameMetrics measure_frame(const SimState& state, const XpbdConstraints& constraints,
                           const std::vector<Collider>& colliders, const Vec3& gravity);

}  // namespace hp
