#pragma once

#include <cstdint>
#include <vector>

#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"

namespace hp {

// The set of vertices a solver is responsible for in one step. Only full masks exist for now;
// partial masks (mixed XPBD/GNN ownership) are built from SimState::solver_owner later.
class RegionMask {
public:
    // Every vertex of a state with `num_vertices` vertices.
    static RegionMask all(Index num_vertices);

    Index num_vertices() const { return static_cast<Index>(owned_.size()); }
    Index num_owned() const { return num_owned_; }
    bool owns(Index v) const { return owned_[static_cast<size_t>(v)] != 0; }
    bool covers_all() const { return num_owned_ == num_vertices(); }

private:
    std::vector<std::uint8_t> owned_;  // one byte per vertex, 1 = owned
    Index num_owned_ = 0;
};

// One physics solver. XPBD and the GNN solver implement this so they can be swapped per region.
class ISolver {
public:
    virtual ~ISolver() = default;

    // Advances the vertices in `mask` by `dt` seconds, updating positions and velocities.
    virtual void step(SimState& state, const RegionMask& mask, Real dt) = 0;
};

}  // namespace hp
