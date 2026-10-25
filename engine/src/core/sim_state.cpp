#include "hp/core/sim_state.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace hp {

void SimState::validate() const {
    const size_t n = positions.size();
    if (velocities.size() != n || rest_positions.size() != n || inv_mass.size() != n ||
        region_id.size() != n || solver_owner.size() != n || body_id.size() != n) {
        throw std::invalid_argument("SimState per-vertex arrays have different lengths");
    }

    // Bodies must cover [0, n) back to back, and body_id must match the ranges.
    Index next = 0;
    for (size_t b = 0; b < bodies.size(); ++b) {
        const BodyRange& r = bodies[b];
        if (r.first_vertex != next || r.num_vertices <= 0) {
            throw std::invalid_argument("SimState body " + std::to_string(b) +
                                        " does not follow the previous body");
        }
        next += r.num_vertices;
        if (static_cast<size_t>(next) > n) {
            throw std::invalid_argument("SimState body ranges exceed the vertex count");
        }
        for (Index v = r.first_vertex; v < next; ++v) {
            if (body_id[static_cast<size_t>(v)] != static_cast<Index>(b)) {
                throw std::invalid_argument("SimState body_id of vertex " + std::to_string(v) +
                                            " does not match its body range");
            }
        }
    }
    if (static_cast<size_t>(next) != n) {
        throw std::invalid_argument("SimState bodies do not cover every vertex");
    }

    for (size_t v = 0; v < n; ++v) {
        if (!std::isfinite(inv_mass[v]) || inv_mass[v] < 0.0f) {
            throw std::invalid_argument("SimState inverse mass of vertex " + std::to_string(v) +
                                        " is negative or not finite");
        }
    }
}

}  // namespace hp
