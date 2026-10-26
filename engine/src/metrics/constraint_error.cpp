#include "hp/metrics/constraint_error.hpp"

#include <algorithm>
#include <cmath>

#include "geometry.hpp"

namespace hp {

ConstraintError constraint_error(const SimState& state, const XpbdConstraints& c) {
    ConstraintError out;
    const auto& x = state.positions;

    double sum_sq = 0.0;
    for (size_t e = 0; e < c.edges.size(); ++e) {
        const double length = edge_length(x, c.edges[e]);
        const double rel = std::abs(length - c.edge_rest_length[e]) / c.edge_rest_length[e];
        sum_sq += rel * rel;
        out.edge_max = std::max(out.edge_max, rel);
    }
    if (!c.edges.empty()) out.edge_rms = std::sqrt(sum_sq / static_cast<double>(c.edges.size()));

    sum_sq = 0.0;
    for (size_t t = 0; t < c.tets.size(); ++t) {
        const double volume = signed_volume(x, c.tets[t]);
        const double rel = std::abs(volume - c.tet_rest_volume[t]) / c.tet_rest_volume[t];
        sum_sq += rel * rel;
        out.volume_max = std::max(out.volume_max, rel);
        if (volume <= 0.0) ++out.inverted_tets;
    }
    if (!c.tets.empty()) out.volume_rms = std::sqrt(sum_sq / static_cast<double>(c.tets.size()));
    return out;
}

}  // namespace hp
