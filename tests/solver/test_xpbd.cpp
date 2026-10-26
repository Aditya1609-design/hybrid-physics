#include <doctest/doctest.h>

#include <cstring>
#include <stdexcept>
#include <vector>

#include "hp/core/rng.hpp"
#include "hp/mesh/generators.hpp"
#include "hp/metrics/constraint_error.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"
#include "hp/solver/xpbd.hpp"

namespace {

// Small beam so the tests stay fast in the sanitizer build.
hp::TetMesh small_beam() {
    hp::BeamParams params;
    params.cells_along = 8;
    params.cells_across = 2;
    return hp::make_beam(params);
}

hp::XpbdSolver make_solver(const hp::TetMesh& mesh, const hp::Material& material,
                           const hp::XpbdSettings& settings) {
    return hp::XpbdSolver(hp::build_constraints(mesh, hp::build_topology(mesh), material),
                          material, settings);
}

void run(hp::XpbdSolver& solver, hp::SimState& state, int frames) {
    const hp::RegionMask mask = hp::RegionMask::all(state.num_vertices());
    for (int f = 0; f < frames; ++f) solver.step(state, mask, hp::kFrameDt);
}

bool same_bytes(const std::vector<hp::Vec3>& a, const std::vector<hp::Vec3>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(a[0])) == 0;
}

Eigen::Vector3d linear_momentum(const hp::SimState& s) {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    for (size_t v = 0; v < s.positions.size(); ++v) {
        p += s.velocities[v].cast<double>() / static_cast<double>(s.inv_mass[v]);
    }
    return p;
}

}  // namespace

TEST_CASE("presets and settings are valid") {
    const hp::XpbdSettings ref = hp::XpbdSettings::reference();
    const hp::XpbdSettings rt = hp::XpbdSettings::realtime();
    CHECK_NOTHROW(ref.validate());
    CHECK_NOTHROW(rt.validate());
    CHECK(ref.substeps * ref.iterations > rt.substeps * rt.iterations);
    CHECK(ref.gravity == rt.gravity);

    hp::XpbdSettings bad = rt;
    bad.substeps = 0;
    CHECK_THROWS_AS(bad.validate(), std::invalid_argument);
    const hp::TetMesh mesh = small_beam();
    CHECK_THROWS_AS(make_solver(mesh, {}, bad), std::invalid_argument);
}

TEST_CASE("step rejects bad masks, timesteps and states") {
    const hp::TetMesh mesh = small_beam();
    hp::XpbdSolver solver = make_solver(mesh, {}, hp::XpbdSettings::realtime());
    hp::SimState s = hp::make_state(mesh, 1000.0f);
    const hp::Index n = s.num_vertices();
    CHECK_THROWS_AS(solver.step(s, hp::RegionMask::all(n - 1), hp::kFrameDt),
                    std::invalid_argument);
    CHECK_THROWS_AS(solver.step(s, hp::RegionMask::all(n), 0.0f), std::invalid_argument);

    hp::BoxParams one_cell;
    one_cell.cells_x = one_cell.cells_y = one_cell.cells_z = 1;
    hp::SimState smaller = hp::make_state(hp::make_box(one_cell), 1000.0f);
    CHECK_THROWS_AS(
        solver.step(smaller, hp::RegionMask::all(smaller.num_vertices()), hp::kFrameDt),
        std::invalid_argument);
}

TEST_CASE("a body at rest without gravity stays at rest") {
    const hp::TetMesh mesh = small_beam();
    hp::XpbdSettings settings = hp::XpbdSettings::realtime();
    settings.gravity = hp::Vec3::Zero();
    hp::XpbdSolver solver = make_solver(mesh, {}, settings);
    hp::SimState s = hp::make_state(mesh, 1000.0f);
    run(solver, s, 30);
    for (size_t v = 0; v < s.positions.size(); ++v) {
        CHECK((s.positions[v] - mesh.rest_positions[v]).norm() < 1e-6f);
        CHECK(s.velocities[v].norm() < 1e-5f);
    }
}

TEST_CASE("free fall matches the discrete integrator and keeps the shape") {
    const hp::TetMesh mesh = small_beam();
    const hp::XpbdSettings settings = hp::XpbdSettings::realtime();
    hp::XpbdSolver solver = make_solver(mesh, {}, settings);
    hp::SimState s = hp::make_state(mesh, 1000.0f);
    const int frames = 60;
    run(solver, s, frames);

    // Per substep v += g h, then x += h v, so after n substeps x = x0 + g h^2 n (n + 1) / 2.
    const double n = static_cast<double>(frames * settings.substeps);
    const double h = static_cast<double>(hp::kFrameDt) / static_cast<double>(settings.substeps);
    const double drop = -9.81 * h * h * n * (n + 1.0) / 2.0;
    for (size_t v = 0; v < s.positions.size(); ++v) {
        CHECK(s.positions[v].y() - mesh.rest_positions[v].y() == doctest::Approx(drop).epsilon(1e-4));
        CHECK(s.velocities[v].y() == doctest::Approx(-9.81 * h * n).epsilon(1e-4));
    }
    const hp::ConstraintError err = hp::constraint_error(s, solver.constraints());
    CHECK(err.edge_max < 1e-5);
    CHECK(err.volume_max < 1e-4);
}

TEST_CASE("linear momentum is conserved without gravity") {
    const hp::TetMesh mesh = small_beam();
    hp::XpbdSettings settings = hp::XpbdSettings::realtime();
    settings.gravity = hp::Vec3::Zero();
    hp::Material material;
    material.edge_compliance = 1e-5f;
    hp::XpbdSolver solver = make_solver(mesh, material, settings);
    hp::SimState s = hp::make_state(mesh, material.density);

    // Translation plus spin plus random jitter, so the body deforms while it moves.
    hp::Rng rng(7);
    const hp::Vec3 omega(0.5f, 2.0f, -1.0f);
    for (size_t v = 0; v < s.positions.size(); ++v) {
        const hp::Vec3 jitter(rng.normal(), rng.normal(), rng.normal());
        s.velocities[v] = hp::Vec3(1.0f, -0.5f, 0.25f) + omega.cross(s.positions[v]) + jitter;
    }
    const Eigen::Vector3d p0 = linear_momentum(s);
    run(solver, s, 120);
    const Eigen::Vector3d p1 = linear_momentum(s);
    CHECK((p1 - p0).norm() < 1e-4 * p0.norm());
    for (const hp::Vec3& x : s.positions) CHECK(x.allFinite());
}

TEST_CASE("pinned vertices never move") {
    const hp::TetMesh mesh = small_beam();
    hp::Material material;
    material.edge_compliance = 1e-5f;
    hp::XpbdSolver solver = make_solver(mesh, material, hp::XpbdSettings::realtime());
    hp::SimState s = hp::make_state(mesh, material.density);
    const hp::Index pinned =
        hp::pin_vertices(s, [](const hp::Vec3& p) { return p.x() < -1.0f + 1e-4f; });
    REQUIRE(pinned == 9);
    run(solver, s, 100);

    bool moved = false;
    for (size_t v = 0; v < s.positions.size(); ++v) {
        if (s.inv_mass[v] == 0.0f) {
            CHECK(s.positions[v] == mesh.rest_positions[v]);
            CHECK(s.velocities[v] == hp::Vec3::Zero());
        } else if ((s.positions[v] - mesh.rest_positions[v]).norm() > 1e-2f) {
            moved = true;
        }
    }
    CHECK(moved);  // the free end sags
}

TEST_CASE("stepping is bit-for-bit deterministic") {
    const hp::TetMesh mesh = small_beam();
    hp::Material material;
    material.edge_compliance = 1e-5f;
    material.damping = 0.5f;
    hp::SimState start = hp::make_state(mesh, material.density);
    hp::pin_vertices(start, [](const hp::Vec3& p) { return p.x() < -1.0f + 1e-4f; });

    // Two separate solvers, and the first solver reused from the same start.
    hp::XpbdSolver a = make_solver(mesh, material, hp::XpbdSettings::realtime());
    hp::XpbdSolver b = make_solver(mesh, material, hp::XpbdSettings::realtime());
    hp::SimState sa = start, sb = start, sc = start;
    run(a, sa, 90);
    run(b, sb, 90);
    run(a, sc, 90);
    CHECK(same_bytes(sa.positions, sb.positions));
    CHECK(same_bytes(sa.velocities, sb.velocities));
    CHECK(same_bytes(sa.positions, sc.positions));
    CHECK(same_bytes(sa.velocities, sc.velocities));
}
