#include "hp/scene/pin.hpp"

namespace hp {

Index pin_vertices(SimState& state, const std::function<bool(const Vec3&)>& select) {
    Index count = 0;
    for (size_t v = 0; v < state.rest_positions.size(); ++v) {
        if (!select(state.rest_positions[v])) continue;
        state.inv_mass[v] = 0.0f;
        state.velocities[v] = Vec3::Zero();
        ++count;
    }
    return count;
}

}  // namespace hp
