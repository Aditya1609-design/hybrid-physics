#include "hp/metrics/penetration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hp {

Penetration penetration(const SimState& state, const std::vector<Collider>& colliders,
                        double contact_tolerance) {
    Penetration out;
    if (colliders.empty()) return out;
    double sum_sq = 0.0;
    for (size_t v = 0; v < state.positions.size(); ++v) {
        if (state.inv_mass[v] == 0.0f) continue;
        const Eigen::Vector3d p = state.positions[v].cast<double>();
        // Smallest signed distance over all colliders.
        double d = std::numeric_limits<double>::infinity();
        for (const Collider& c : colliders) {
            Eigen::Vector3d n;
            d = std::min(d, signed_distance(c, p, n));
        }
        if (d <= contact_tolerance) ++out.num_in_contact;
        if (d < 0.0) {
            ++out.num_penetrating;
            out.max_depth = std::max(out.max_depth, -d);
            sum_sq += d * d;
        }
    }
    if (out.num_penetrating > 0) out.rms_depth = std::sqrt(sum_sq / out.num_penetrating);
    return out;
}

}  // namespace hp
