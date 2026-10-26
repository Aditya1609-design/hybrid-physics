#include "hp/metrics/energy.hpp"

#include "geometry.hpp"

namespace hp {

Energy energy(const SimState& state, const XpbdConstraints& c, const Vec3& gravity) {
    Energy out;
    const Eigen::Vector3d g = gravity.cast<double>();
    for (size_t v = 0; v < state.positions.size(); ++v) {
        if (state.inv_mass[v] == 0.0f) continue;
        const double m = 1.0 / static_cast<double>(state.inv_mass[v]);
        out.kinetic += 0.5 * m * state.velocities[v].cast<double>().squaredNorm();
        out.gravitational -= m * g.dot(state.positions[v].cast<double>());
    }

    for (size_t e = 0; e < c.edges.size(); ++e) {
        if (c.edge_compliance[e] == 0.0f) continue;
        const double C = edge_length(state.positions, c.edges[e]) - c.edge_rest_length[e];
        out.elastic += 0.5 * C * C / static_cast<double>(c.edge_compliance[e]);
    }
    for (size_t t = 0; t < c.tets.size(); ++t) {
        if (c.tet_compliance[t] == 0.0f) continue;
        const double C = signed_volume(state.positions, c.tets[t]) - c.tet_rest_volume[t];
        out.elastic += 0.5 * C * C / static_cast<double>(c.tet_compliance[t]);
    }
    return out;
}

}  // namespace hp
