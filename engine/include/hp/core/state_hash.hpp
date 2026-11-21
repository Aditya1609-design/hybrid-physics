#pragma once

#include <cstdint>

#include "hp/core/sim_state.hpp"

namespace hp {

// FNV-1a (64-bit) over the raw bytes of positions, then velocities. Equal hashes mean
// bit-identical states, so runs on two machines can be compared with one number. The Python
// trajectory checker computes the same hash from a recorded frame.
std::uint64_t state_hash(const SimState& state);

}  // namespace hp
