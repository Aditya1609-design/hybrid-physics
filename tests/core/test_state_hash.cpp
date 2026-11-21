#include <doctest/doctest.h>

#include "hp/core/state_hash.hpp"

TEST_CASE("state hash is FNV-1a over positions then velocities") {
    hp::SimState s;
    s.positions = {hp::Vec3(1.0f, 2.0f, 3.0f), hp::Vec3(-0.5f, 0.25f, 4.0f)};
    s.velocities = {hp::Vec3::Zero(), hp::Vec3(1.0f, 0.0f, 0.0f)};
    // Reference value computed independently from the same 48 little-endian bytes.
    CHECK(hp::state_hash(s) == 0x4e08052670091be2ULL);

    // Any bit change gives a different hash.
    hp::SimState t = s;
    t.velocities[0].z() = -0.0f;
    CHECK(hp::state_hash(t) != hp::state_hash(s));
}
