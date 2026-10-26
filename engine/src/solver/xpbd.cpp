#include "hp/solver/xpbd.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace hp {

// Preset counts were chosen by measurement (see the XPBD ADR). More substeps beat more
// iterations at equal cost, so both presets use one iteration.
XpbdSettings XpbdSettings::reference() {
    XpbdSettings s;
    s.substeps = 256;
    s.iterations = 1;
    return s;
}

XpbdSettings XpbdSettings::realtime() {
    XpbdSettings s;
    s.substeps = 10;
    s.iterations = 1;
    return s;
}

void XpbdSettings::validate() const {
    if (substeps < 1 || iterations < 1) {
        throw std::invalid_argument("substeps and iterations must be at least 1");
    }
    if (!gravity.allFinite()) throw std::invalid_argument("gravity must be finite");
}

XpbdSolver::XpbdSolver(XpbdConstraints constraints, const Material& material,
                       const XpbdSettings& settings)
    : constraints_(std::move(constraints)), settings_(settings), damping_(material.damping) {
    material.validate();
    settings_.validate();
    const auto& c = constraints_;
    if (c.edge_rest_length.size() != c.edges.size() ||
        c.edge_compliance.size() != c.edges.size() ||
        c.tet_rest_volume.size() != c.tets.size() || c.tet_compliance.size() != c.tets.size()) {
        throw std::invalid_argument("constraint arrays have mismatched lengths");
    }
    for (const Edge& e : c.edges) min_vertices_ = std::max({min_vertices_, e[0] + 1, e[1] + 1});
    for (const Tet& t : c.tets) {
        for (const Index v : t) min_vertices_ = std::max(min_vertices_, v + 1);
    }
}

void XpbdSolver::step(SimState& state, const RegionMask& mask, Real dt) {
    if (!(dt > 0.0f)) throw std::invalid_argument("dt must be positive");
    if (mask.num_vertices() != state.num_vertices() || !mask.covers_all()) {
        throw std::invalid_argument("XpbdSolver supports only masks covering every vertex");
    }
    if (state.num_vertices() < min_vertices_ ||
        state.velocities.size() != state.positions.size() ||
        state.inv_mass.size() != state.positions.size()) {
        throw std::invalid_argument("state does not match the solver's constraints");
    }
    kernel_.step(constraints_, state.positions, state.velocities, state.inv_mass, dt,
                 settings_.substeps, settings_.iterations, settings_.gravity, damping_);
}

}  // namespace hp
