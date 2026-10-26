#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include "hp/metrics/penetration.hpp"
#include "hp/mesh/generators.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"

namespace {

// Unit box (centred at the origin) with 2 cells per side: vertex rows at y = -0.5, 0, 0.5.
hp::SimState unit_box() {
    hp::BoxParams params;
    params.cells_x = params.cells_y = params.cells_z = 2;
    return hp::make_state(hp::make_box(params), 1000.0f);
}

}  // namespace

TEST_CASE("no colliders or no overlap gives zero penetration") {
    const hp::SimState s = unit_box();
    const hp::Penetration none = hp::penetration(s, {});
    CHECK(none.max_depth == 0.0);
    CHECK(none.num_in_contact == 0);

    const std::vector<hp::Collider> far = {
        hp::Collider::plane(hp::Vec3(0, -2, 0), hp::Vec3::UnitY(), 0.0f)};
    const hp::Penetration p = hp::penetration(s, far);
    CHECK(p.max_depth == 0.0);
    CHECK(p.num_penetrating == 0);
    CHECK(p.num_in_contact == 0);
}

TEST_CASE("ground plane through the box counts depths and contacts") {
    hp::SimState s = unit_box();
    // Plane at y = -0.2: the 9 bottom vertices are 0.3 inside, the 9 middle ones 0.2 above.
    const std::vector<hp::Collider> ground = {
        hp::Collider::plane(hp::Vec3(0, -0.2f, 0), hp::Vec3::UnitY(), 0.0f)};
    hp::Penetration p = hp::penetration(s, ground);
    CHECK(p.num_penetrating == 9);
    CHECK(p.num_in_contact == 9);
    CHECK(p.max_depth == doctest::Approx(0.3).epsilon(1e-6));
    CHECK(p.rms_depth == doctest::Approx(0.3).epsilon(1e-6));

    // A wide tolerance also counts the middle row as in contact.
    CHECK(hp::penetration(s, ground, 0.25).num_in_contact == 18);

    // Pinned vertices are ignored.
    hp::pin_vertices(s, [](const hp::Vec3& x) { return x.y() < -0.4f && x.x() < -0.4f; });
    p = hp::penetration(s, ground);
    CHECK(p.num_penetrating == 6);
}

TEST_CASE("deepest collider wins when colliders overlap") {
    const hp::SimState s = unit_box();
    // The centre vertex (origin) is 0.1 inside the plane and 0.4 inside the sphere.
    const std::vector<hp::Collider> colliders = {
        hp::Collider::plane(hp::Vec3(0, 0.1f, 0), hp::Vec3::UnitY(), 0.0f),
        hp::Collider::sphere(hp::Vec3::Zero(), 0.4f, 0.0f)};
    const hp::Penetration p = hp::penetration(s, colliders);
    // 18 vertices under y = 0.1 (bottom and middle rows), deepest is the bottom row at 0.6.
    CHECK(p.num_penetrating == 18);
    CHECK(p.max_depth == doctest::Approx(0.6).epsilon(1e-6));
    // 9 at 0.6, the centre at 0.4 (sphere, not 0.1 from the plane), 8 at 0.1.
    const double expected_rms = std::sqrt((9 * 0.36 + 0.16 + 8 * 0.01) / 18);
    CHECK(p.rms_depth == doctest::Approx(expected_rms).epsilon(1e-6));
}
