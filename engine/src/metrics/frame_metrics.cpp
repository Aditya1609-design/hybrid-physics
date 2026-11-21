#include "hp/metrics/frame_metrics.hpp"

namespace hp {

FrameMetrics measure_frame(const SimState& state, const XpbdConstraints& constraints,
                           const std::vector<Collider>& colliders, const Vec3& gravity) {
    return {energy(state, constraints, gravity), constraint_error(state, constraints),
            penetration(state, colliders)};
}

}  // namespace hp
