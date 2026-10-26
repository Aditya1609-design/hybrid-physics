#include <doctest/doctest.h>

#include "hp/mesh/generators.hpp"
#include "hp/metrics/constraint_error.hpp"
#include "hp/metrics/energy.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"

namespace {

struct Setup {
    hp::TetMesh mesh;
    hp::XpbdConstraints constraints;
    hp::SimState state;
};

Setup make_setup(hp::Index cells, const hp::Material& material = {}) {
    hp::BoxParams params;
    params.cells_x = params.cells_y = params.cells_z = cells;
    Setup s{hp::make_box(params), {}, {}};
    s.constraints = hp::build_constraints(s.mesh, hp::build_topology(s.mesh), material);
    s.state = hp::make_state(s.mesh, material.density);
    return s;
}

// Scales every position along x by `sx` about the origin.
void stretch_x(hp::SimState& state, float sx) {
    for (hp::Vec3& p : state.positions) p.x() *= sx;
}

}  // namespace

TEST_CASE("rest state has zero constraint error and zero energy") {
    const Setup s = make_setup(6);
    const hp::ConstraintError err = hp::constraint_error(s.state, s.constraints);
    // Only float32 rounding of the stored rest lengths and volumes remains.
    CHECK(err.edge_max < 1e-6);
    CHECK(err.volume_max < 1e-6);
    CHECK(err.inverted_tets == 0);

    const hp::Energy en = hp::energy(s.state, s.constraints, hp::Vec3::Zero());
    CHECK(en.kinetic == 0.0);
    CHECK(en.elastic < 1e-6);  // rounding only; a 1% stretch is around 1e2 J here
}

TEST_CASE("constraint error measures relative stretch and inversion") {
    Setup s = make_setup(6);
    for (hp::Vec3& p : s.state.positions) p *= 1.01f;
    hp::ConstraintError err = hp::constraint_error(s.state, s.constraints);
    CHECK(err.edge_rms == doctest::Approx(0.01).epsilon(1e-4));
    CHECK(err.edge_max == doctest::Approx(0.01).epsilon(1e-4));
    CHECK(err.volume_rms == doctest::Approx(1.01 * 1.01 * 1.01 - 1.0).epsilon(1e-3));

    // Mirroring the body inverts every tet.
    s.state.positions = s.mesh.rest_positions;
    stretch_x(s.state, -1.0f);
    err = hp::constraint_error(s.state, s.constraints);
    CHECK(err.inverted_tets == s.mesh.num_tets());
    CHECK(err.volume_max == doctest::Approx(2.0).epsilon(1e-4));
}

TEST_CASE("kinetic and gravitational energy skip pinned vertices") {
    Setup s = make_setup(4);
    s.state.velocities.assign(s.state.velocities.size(), hp::Vec3(2.0f, 0.0f, 0.0f));
    const hp::Vec3 g(0.0f, -9.81f, 0.0f);
    for (hp::Vec3& p : s.state.positions) p.y() += 3.0f;

    // Unit box with density 1000: mass 1000 kg, centre at y = 3.
    hp::Energy en = hp::energy(s.state, s.constraints, g);
    CHECK(en.kinetic == doctest::Approx(0.5 * 1000.0 * 4.0).epsilon(1e-5));
    CHECK(en.gravitational == doctest::Approx(1000.0 * 9.81 * 3.0).epsilon(1e-5));

    const hp::Index pinned = hp::pin_vertices(s.state, [](const hp::Vec3&) { return true; });
    CHECK(pinned == s.state.num_vertices());
    en = hp::energy(s.state, s.constraints, g);
    CHECK(en.kinetic == 0.0);
    CHECK(en.gravitational == 0.0);
}

TEST_CASE("volume energy matches the analytic value") {
    hp::Material material;
    material.edge_compliance = 0.0f;  // leave only the volume term
    material.volume_compliance = 1e-6f;
    Setup s = make_setup(5, material);
    stretch_x(s.state, 1.02f);
    // Every tet scales to 1.02 V0: E = sum (0.02 V0)^2 / (2 c V0) = 0.02^2 V / (2 c).
    const double expected = 0.02 * 0.02 * hp::total_volume(s.mesh) / (2.0 * 1e-6);
    CHECK(hp::energy(s.state, s.constraints, hp::Vec3::Zero()).elastic ==
          doctest::Approx(expected).epsilon(2e-3));
}

TEST_CASE("elastic energy of a uniform stretch does not depend on resolution") {
    double previous = 0.0;
    for (const hp::Index cells : {3, 6, 12}) {
        Setup s = make_setup(cells);
        stretch_x(s.state, 1.01f);
        const double e = hp::energy(s.state, s.constraints, hp::Vec3::Zero()).elastic;
        CAPTURE(cells);
        if (previous > 0.0) CHECK(e == doctest::Approx(previous).epsilon(1e-3));
        previous = e;
    }
}
