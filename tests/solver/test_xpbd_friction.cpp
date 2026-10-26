#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include "hp/collision/collider.hpp"
#include "hp/mesh/generators.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/solver/xpbd.hpp"

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kG = 9.81f;
constexpr float kHalfHeight = 0.125f;

// A 0.5 x 0.25 x 0.5 m block (75 vertices) resting with its base on a plane through the
// origin, tilted by `degrees` about z. `down` points down the slope.
struct Slope {
    hp::TetMesh mesh;
    hp::Material material;
    hp::SimState state;
    hp::Collider plane;
    hp::Vec3 down;
    std::vector<hp::Index> base;  // vertices on the contact face
};

Slope make_slope(float degrees, float friction, float edge_compliance, float damping) {
    hp::BoxParams params;
    params.size = hp::Vec3(0.5f, 2.0f * kHalfHeight, 0.5f);
    params.cells_x = 4;
    params.cells_y = 2;
    params.cells_z = 4;
    Slope s{hp::make_box(params), {}, {}, {}, {}, {}};

    const hp::Mat3 r =
        Eigen::AngleAxis<float>(degrees * kPi / 180.0f, hp::Vec3::UnitZ()).toRotationMatrix();
    const hp::Vec3 n = r * hp::Vec3::UnitY();
    s.down = -(r * hp::Vec3::UnitX());
    for (hp::Vec3& p : s.mesh.rest_positions) p = r * p + kHalfHeight * n;
    s.plane = hp::Collider::plane(hp::Vec3::Zero(), n, friction);

    s.material.edge_compliance = edge_compliance;
    s.material.damping = damping;
    s.state = hp::make_state(s.mesh, s.material.density);
    for (hp::Index v = 0; v < s.mesh.num_vertices(); ++v) {
        if (s.mesh.rest_positions[static_cast<size_t>(v)].dot(n) < 1e-4f) s.base.push_back(v);
    }
    return s;
}

hp::XpbdSolver make_solver(const hp::TetMesh& mesh, const hp::Material& material,
                           const hp::XpbdSettings& settings, const hp::Collider& collider) {
    return hp::XpbdSolver(hp::build_constraints(mesh, hp::build_topology(mesh), material),
                          material, settings, {collider});
}

// Mean displacement of the contact-face vertices along `dir` since the start: their slip.
double base_slip(const Slope& s, const hp::SimState& state) {
    double sum = 0.0;
    for (const hp::Index v : s.base) {
        const auto i = static_cast<size_t>(v);
        sum += (state.positions[i] - s.mesh.rest_positions[i]).cast<double>().dot(
            s.down.cast<double>());
    }
    return sum / static_cast<double>(s.base.size());
}

// Mass-weighted (centre of mass) velocity; internal vibration cancels out of it.
Eigen::Vector3d com_velocity(const hp::SimState& s) {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double m = 0.0;
    for (size_t v = 0; v < s.positions.size(); ++v) {
        const double mv = 1.0 / static_cast<double>(s.inv_mass[v]);
        p += mv * s.velocities[v].cast<double>();
        m += mv;
    }
    return p / m;
}

Eigen::Vector3d com_position(const hp::SimState& s) {
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    double m = 0.0;
    for (size_t v = 0; v < s.positions.size(); ++v) {
        const double mv = 1.0 / static_cast<double>(s.inv_mass[v]);
        c += mv * s.positions[v].cast<double>();
        m += mv;
    }
    return c / m;
}

const hp::XpbdSettings kPresets[] = {hp::XpbdSettings::realtime(),
                                     hp::XpbdSettings::reference()};

}  // namespace

TEST_CASE("a stiff block on a 20 degree slope with friction 0.5 sticks") {
    // tan(20) = 0.36 < 0.5. Y = 10 MPa; softer blocks are covered by the next test.
    for (const hp::XpbdSettings& settings : kPresets) {
        Slope s = make_slope(20.0f, 0.5f, 1e-7f, 2.0f);
        hp::XpbdSolver solver = make_solver(s.mesh, s.material, settings, s.plane);
        const hp::RegionMask mask = hp::RegionMask::all(s.state.num_vertices());
        double max_slip = 0.0;
        for (int f = 0; f < 300; ++f) {
            solver.step(s.state, mask, hp::kFrameDt);
            max_slip = std::max(max_slip, std::abs(base_slip(s, s.state)));
        }
        MESSAGE(settings.substeps << " substeps, Y = 10 MPa: max contact slip " << max_slip
                                  << " m over 300 frames");
        CHECK(max_slip < 1e-3);
    }
}

TEST_CASE("a soft block on a 20 degree slope slips briefly while loading, then sticks") {
    // Y = 1 MPa: the block spreads sideways as it takes its weight, and the lightly loaded
    // contact edges slip a little in the first frames. After that it must not move.
    for (const hp::XpbdSettings& settings : kPresets) {
        Slope s = make_slope(20.0f, 0.5f, 1e-6f, 2.0f);
        hp::XpbdSolver solver = make_solver(s.mesh, s.material, settings, s.plane);
        const hp::RegionMask mask = hp::RegionMask::all(s.state.num_vertices());
        for (int f = 0; f < 30; ++f) solver.step(s.state, mask, hp::kFrameDt);
        const double startup = base_slip(s, s.state);
        double max_later = 0.0;
        for (int f = 30; f < 300; ++f) {
            solver.step(s.state, mask, hp::kFrameDt);
            max_later = std::max(max_later, std::abs(base_slip(s, s.state) - startup));
        }
        MESSAGE(settings.substeps << " substeps, Y = 1 MPa: start-up slip " << startup
                                  << " m, slip after frame 30 " << max_later << " m");
        CHECK(std::abs(startup) < 5e-3);
        CHECK(max_later < 1e-4);
    }
}

TEST_CASE("a block slides down a 35 degree slope at g (sin - mu cos)") {
    for (const hp::XpbdSettings& settings : kPresets) {
        for (const float mu : {0.5f, 0.0f}) {
            // Undamped, so damping does not slow the slide.
            Slope s = make_slope(35.0f, mu, 1e-6f, 0.0f);
            hp::XpbdSolver solver = make_solver(s.mesh, s.material, settings, s.plane);
            const hp::RegionMask mask = hp::RegionMask::all(s.state.num_vertices());
            // Measure between frames 60 and 180, after the start-up transient.
            double v60 = 0.0, v180 = 0.0;
            for (int f = 1; f <= 180; ++f) {
                solver.step(s.state, mask, hp::kFrameDt);
                if (f == 60) v60 = com_velocity(s.state).dot(s.down.cast<double>());
            }
            v180 = com_velocity(s.state).dot(s.down.cast<double>());
            const double accel = (v180 - v60) / (120.0 * hp::kFrameDt);
            const double theta = 35.0 * kPi / 180.0;
            const double expected = kG * (std::sin(theta) - mu * std::cos(theta));
            MESSAGE(settings.substeps << " substeps, mu = " << mu << ": acceleration " << accel
                                      << " m/s^2, expected " << expected << " (ratio "
                                      << accel / expected << ")");
            CHECK(accel == doctest::Approx(expected).epsilon(0.05));
        }
    }
}

TEST_CASE("sliding on flat ground: no friction keeps speed, friction stops it") {
    const float v0 = 2.0f;
    for (const hp::XpbdSettings& settings : kPresets) {
        for (const float mu : {0.0f, 0.5f}) {
            Slope s = make_slope(0.0f, mu, 1e-6f, 0.0f);
            for (hp::Vec3& v : s.state.velocities) v.x() = v0;
            hp::XpbdSolver solver = make_solver(s.mesh, s.material, settings, s.plane);
            const hp::RegionMask mask = hp::RegionMask::all(s.state.num_vertices());
            const Eigen::Vector3d start = com_position(s.state);
            double base_at_40 = 0.0;
            for (int f = 1; f <= 60; ++f) {
                solver.step(s.state, mask, hp::kFrameDt);
                if (f == 40) base_at_40 = base_slip(s, s.state);
            }
            const double distance = com_position(s.state).x() - start.x();
            const double speed = com_velocity(s.state).x();
            if (mu == 0.0f) {
                MESSAGE(settings.substeps << " substeps, mu = 0: speed after 1 s " << speed
                                          << " m/s (start " << v0 << ")");
                CHECK(speed == doctest::Approx(v0).epsilon(1e-3));
            } else {
                // Stops after v0 / (mu g) = 0.41 s (frame 25). The undamped block keeps rocking
                // elastically afterwards, so "stopped" is checked on the contact face.
                const double expected = v0 * v0 / (2.0 * mu * kG);
                const double base_moved = std::abs(base_slip(s, s.state) - base_at_40);
                MESSAGE(settings.substeps << " substeps, mu = " << mu << ": stopping distance "
                                          << distance << " m, expected " << expected
                                          << " (ratio " << distance / expected
                                          << "), contact face moved " << base_moved
                                          << " m from frame 40 to 60");
                CHECK(distance == doctest::Approx(expected).epsilon(0.10));
                CHECK(base_moved < 1e-4);
            }
        }
    }
}
