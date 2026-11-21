#include "hp/core/state_hash.hpp"

#include <vector>

namespace hp {

namespace {

// Folds the bytes of one array into the running hash.
template <typename T>
void fnv1a(std::uint64_t& h, const std::vector<T>& data) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(data.data());
    for (size_t i = 0; i < data.size() * sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 0x100000001b3ULL;
    }
}

}  // namespace

std::uint64_t state_hash(const SimState& state) {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    fnv1a(h, state.positions);
    fnv1a(h, state.velocities);
    return h;
}

}  // namespace hp
