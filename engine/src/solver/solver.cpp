#include "hp/solver/solver.hpp"

#include <stdexcept>

namespace hp {

RegionMask RegionMask::all(Index num_vertices) {
    if (num_vertices < 0) throw std::invalid_argument("vertex count must not be negative");
    RegionMask mask;
    mask.owned_.assign(static_cast<size_t>(num_vertices), 1);
    mask.num_owned_ = num_vertices;
    return mask;
}

}  // namespace hp
