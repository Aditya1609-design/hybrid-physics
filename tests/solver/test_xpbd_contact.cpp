#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "hp/collision/collider.hpp"
#include "hp/mesh/generators.hpp"
#include "hp/metrics/energy.hpp"
#include "hp/metrics/penetration.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"
#include "hp/solver/xpbd.hpp"

namespace {

// Bound on penetration at the end of any frame: 10 um on a 0.5 m body.
constexpr double kMaxPenetration = 1e-5;
constexpr float kSize = 0.5f;

// A 0.5 m soft rubber cube (4 cells per side, 125 vertices) and a solver for it.
struct Drop {
    hp::TetMesh mesh;
    hp::Material material;
    hp::SimState state;
};

Drop make_drop(const hp::Vec3& center, float damping, float speed_y = 0.0f) {
    hp::BoxParams params;
    params.size = hp::Vec3(kSize, kSize, kSize);
    params.cells_x = params.cells_y = params.cells_z = 4;
    params.center = center;
    Drop d{hp::make_box(params), {}, {}};
    d.material.damping = damping;
    d.state = hp::make_state(d.mesh, d.material.density);
    for (hp::Vec3& v : d.state.velocities) v.y() = speed_y;
    return d;
}

hp::XpbdSolver make_solver(const Drop& d, const hp::XpbdSettings& settings,
                           std::vector<hp::Collider> colliders) {
    return hp::XpbdSolver(hp::build_constraints(d.mesh, hp::build_topology(d.mesh), d.material),
                          d.material, settings, std::move(colliders));
}

hp::Vec3 center_of_mass(const hp::SimState& s) {
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    double m = 0.0;
    for (size_t v = 0; v < s.positions.size(); ++v) {
        const double mv = 1.0 / static_cast<double>(s.inv_mass[v]);
        c += mv * s.positions[v].cast<double>();
        m += mv;
    }
    return (c / m).cast<float>();
}

double rms_speed(const hp::SimState& s) {
    double sum = 0.0;
    for (const hp::Vec3& v : s.velocities) sum += v.cast<double>().squaredNorm();
    return std::sqrt(sum / static_cast<double>(s.velocities.size()));
}

// Kinetic plus gravitational energy. It leaves out elastic energy, which is never negative,
// so it can only rise above its start if the solver adds energy.
double mechanical_energy(const hp::SimState& s, const hp::XpbdSolver& solver) {
    const hp::Energy e = hp::energy(s, solver.constraints(), solver.settings().gravity);
    return e.kinetic + e.gravitational;
}

double total_energy(const hp::SimState& s, const hp::XpbdSolver& solver) {
    return hp::energy(s, solver.constraints(), solver.settings().gravity).total();
}

// What a run records frame by frame.
struct Trace {
    double max_penetration = 0.0;
    double max_mechanical = 0.0;
    double start_mechanical = 0.0;
    double max_total = 0.0;  // kinetic + gravitational + elastic
    double start_total = 0.0;
    bool finite = true;
};

Trace simulate(hp::XpbdSolver& solver, hp::SimState& s, int frames) {
    const hp::RegionMask mask = hp::RegionMask::all(s.num_vertices());
    Trace t;
    t.start_mechanical = t.max_mechanical = mechanical_energy(s, solver);
    t.start_total = t.max_total = total_energy(s, solver);
    for (int f = 0; f < frames; ++f) {
        solver.step(s, mask, hp::kFrameDt);
        t.max_penetration =
            std::max(t.max_penetration, hp::penetration(s, solver.colliders()).max_depth);
        t.max_mechanical = std::max(t.max_mechanical, mechanical_energy(s, solver));
        t.max_total = std::max(t.max_total, total_energy(s, solver));
        for (const hp::Vec3& p : s.positions) t.finite = t.finite && p.allFinite();
    }
    return t;
}

// Allows float rounding in the energy sum (about 1e-4 of the start) but no real gain.
void check_no_energy_gain(const Trace& t) {
    CHECK(t.max_mechanical <= t.start_mechanical + 1e-4 * std::abs(t.start_mechanical));
}

hp::Collider ground(float friction = 0.0f) {
    return hp::Collider::plane(hp::Vec3::Zero(), hp::Vec3::UnitY(), friction);
}

std::string kind_name(hp::ColliderKind kind) {
    switch (kind) {
        case hp::ColliderKind::Plane: return "plane";
        case hp::ColliderKind::Sphere: return "sphere";
        case hp::ColliderKind::Box: return "box";
    }
    return "?";
}

bool same_bytes(const std::vector<hp::Vec3>& a, const std::vector<hp::Vec3>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(a[0])) == 0;
}

}  // namespace

TEST_CASE("colliders that are never touched leave results bit-identical") {
    // Sagging cantilever, as in the solver tests, with a plane far below it.
    hp::BeamParams params;
    params.cells_along = 8;
    params.cells_across = 2;
    const hp::TetMesh mesh = hp::make_beam(params);
    const hp::Material material;
    hp::SimState a = hp::make_state(mesh, material.density);
    hp::pin_vertices(a, [](const hp::Vec3& p) { return p.x() < -1.0f + 1e-4f; });
    hp::SimState b = a, c = a;

    const hp::XpbdConstraints constraints =
        hp::build_constraints(mesh, hp::build_topology(mesh), material);
    const hp::XpbdSettings settings = hp::XpbdSettings::realtime();
    hp::XpbdSolver plain(constraints, material, settings);
    hp::XpbdSolver empty(constraints, material, settings, {});
    hp::XpbdSolver far(constraints, material, settings,
                       {hp::Collider::plane(hp::Vec3(0, -100, 0), hp::Vec3::UnitY(), 0.5f)});
    const hp::RegionMask mask = hp::RegionMask::all(a.num_vertices());
    for (int f = 0; f < 60; ++f) {
        plain.step(a, mask, hp::kFrameDt);
        empty.step(b, mask, hp::kFrameDt);
        far.step(c, mask, hp::kFrameDt);
    }
    CHECK(same_bytes(a.positions, b.positions));
    CHECK(same_bytes(a.velocities, b.velocities));
    CHECK(same_bytes(a.positions, c.positions));
    CHECK(same_bytes(a.velocities, c.velocities));
}

TEST_CASE("invalid colliders are rejected by the solver") {
    const Drop d = make_drop(hp::Vec3(0, 1, 0), 0.0f);
    hp::Collider bad = ground();
    bad.friction = -1.0f;
    CHECK_THROWS_AS(make_solver(d, hp::XpbdSettings::realtime(), {bad}), std::invalid_argument);
}

TEST_CASE("a box dropped on the ground stays above it and comes to rest") {
    for (const hp::XpbdSettings& settings :
         {hp::XpbdSettings::realtime(), hp::XpbdSettings::reference()}) {
        // Undamped: checks the penetration bound and that contact never adds energy.
        Drop d = make_drop(hp::Vec3(0, 1, 0), 0.0f);
        hp::XpbdSolver solver = make_solver(d, settings, {ground()});
        const Trace t = simulate(solver, d.state, 300);
        MESSAGE(settings.substeps << " substeps, undamped: max penetration " << t.max_penetration
                                  << " m, mechanical energy max " << t.max_mechanical
                                  << " J (start " << t.start_mechanical << ")");
        CHECK(t.finite);
        CHECK(t.max_penetration < kMaxPenetration);
        check_no_energy_gain(t);

        // Damped: settles on its base at about half its height.
        Drop dd = make_drop(hp::Vec3(0, 1, 0), 2.0f);
        hp::XpbdSolver damped = make_solver(dd, settings, {ground()});
        const Trace td = simulate(damped, dd.state, 300);
        const hp::Vec3 com = center_of_mass(dd.state);
        const hp::Penetration p = hp::penetration(dd.state, damped.colliders());
        MESSAGE(settings.substeps << " substeps, damped: rest height " << com.y()
                                  << " m, rms speed " << rms_speed(dd.state) << " m/s, "
                                  << p.num_in_contact << " vertices in contact");
        CHECK(td.max_penetration < kMaxPenetration);
        CHECK(com.y() > 0.5f * kSize - 0.01f);
        CHECK(com.y() < 0.5f * kSize);
        CHECK(std::abs(com.x()) < 1e-4f);
        CHECK(std::abs(com.z()) < 1e-4f);
        CHECK(p.num_in_contact == 25);  // the 5 x 5 bottom face
        CHECK(rms_speed(dd.state) < 1e-2);
    }
}

TEST_CASE("a resting box stays put for 2000 frames") {
    // Real-time preset only: 2000 reference frames would be too slow for the sanitizer build.
    Drop d = make_drop(hp::Vec3(0, 0.5f * kSize + 0.01f, 0), 2.0f);
    hp::XpbdSolver solver = make_solver(d, hp::XpbdSettings::realtime(), {ground()});
    simulate(solver, d.state, 300);
    const hp::Vec3 settled = center_of_mass(d.state);

    double max_speed = 0.0;
    float max_drift = 0.0f;
    double max_penetration = 0.0;
    const hp::RegionMask mask = hp::RegionMask::all(d.state.num_vertices());
    for (int f = 0; f < 2000; ++f) {
        solver.step(d.state, mask, hp::kFrameDt);
        max_speed = std::max(max_speed, rms_speed(d.state));
        max_drift = std::max(max_drift, (center_of_mass(d.state) - settled).norm());
        max_penetration =
            std::max(max_penetration, hp::penetration(d.state, solver.colliders()).max_depth);
    }
    MESSAGE("2000 frames at rest: max drift " << max_drift << " m, max rms speed " << max_speed
                                              << " m/s, max penetration " << max_penetration);
    CHECK(max_drift < 1e-4f);
    CHECK(max_speed < 1e-3);
    CHECK(max_penetration < kMaxPenetration);
}

TEST_CASE("fast impacts do not tunnel through the ground or a thick box") {
    // 20 m/s downwards, 0.5 m above the surface: contact happens within the second frame.
    const std::vector<hp::Collider> targets[] = {
        {ground()},
        {hp::Collider::box(hp::Vec3(0, -0.5f, 0), hp::Vec3(1, 0.5f, 1), hp::Mat3::Identity(),
                           0.0f)},
    };
    for (const hp::XpbdSettings& settings :
         {hp::XpbdSettings::realtime(), hp::XpbdSettings::reference()}) {
        for (const std::vector<hp::Collider>& colliders : targets) {
            Drop d = make_drop(hp::Vec3(0, 0.75f, 0), 0.0f, -20.0f);
            hp::XpbdSolver solver = make_solver(d, settings, colliders);
            const Trace t = simulate(solver, d.state, 120);
            MESSAGE(settings.substeps << " substeps, " << kind_name(colliders[0].kind)
                                      << ": max penetration " << t.max_penetration
                                      << " m, max/start energy: mechanical "
                                      << t.max_mechanical / t.start_mechanical
                                      << ", total with elastic " << t.max_total / t.start_total);
            CHECK(t.finite);
            CHECK(t.max_penetration < kMaxPenetration);
            check_no_energy_gain(t);
        }
    }
}

TEST_CASE("full energy with elastic never rises during impact on the reference preset") {
    // The elastic term C^2 / (2 alpha) assumes converged constraints. Mid-impact the real-time
    // preset is far from converged, so its measured total can spike above the start even though
    // the body rebounds lower; that peak is only reported. The reference preset must stay below.
    for (const hp::XpbdSettings& settings :
         {hp::XpbdSettings::realtime(), hp::XpbdSettings::reference()}) {
        Drop d = make_drop(hp::Vec3(0, 0.75f, 0), 0.0f, -20.0f);
        hp::XpbdSolver solver = make_solver(d, settings, {ground()});
        const Trace t = simulate(solver, d.state, 120);
        MESSAGE(settings.substeps << " substeps, 20 m/s onto the ground: total energy with "
                                  << "elastic, max/start " << t.max_total / t.start_total);
        if (settings.substeps == hp::XpbdSettings::reference().substeps) {
            CHECK(t.max_total <= t.start_total * (1.0 + 1e-4));
        }
    }
}

TEST_CASE("bodies slide off a sphere and a rotated box onto the ground") {
    // Frictionless, so the body slides off the curved or tilted top and lands on the ground.
    // Checks the bound with two colliders in play and contact against the curved surfaces.
    const hp::Mat3 tilt = Eigen::AngleAxis<float>(0.3f, hp::Vec3::UnitZ()).toRotationMatrix();
    const std::vector<hp::Collider> scenes[] = {
        {ground(), hp::Collider::sphere(hp::Vec3(0, 0.5f, 0), 0.5f, 0.0f)},
        {ground(), hp::Collider::box(hp::Vec3(0, 0.4f, 0), hp::Vec3(0.5f, 0.25f, 0.5f), tilt,
                                     0.0f)},
    };
    for (const hp::XpbdSettings& settings :
         {hp::XpbdSettings::realtime(), hp::XpbdSettings::reference()}) {
        for (const std::vector<hp::Collider>& colliders : scenes) {
            Drop d = make_drop(hp::Vec3(0.1f, 1.6f, 0), 2.0f);
            hp::XpbdSolver solver = make_solver(d, settings, colliders);
            const Trace t = simulate(solver, d.state, 300);
            const hp::Penetration p = hp::penetration(d.state, solver.colliders());
            const hp::Vec3 com = center_of_mass(d.state);
            MESSAGE(settings.substeps << " substeps, " << kind_name(colliders[1].kind)
                                      << ": max penetration " << t.max_penetration
                                      << " m, final centre (" << com.x() << ", " << com.y()
                                      << ")");
            CHECK(t.finite);
            CHECK(t.max_penetration < kMaxPenetration);
            check_no_energy_gain(t);
            // Ended up on the ground beside the obstacle, and something touches.
            CHECK(com.y() < 0.5f * kSize + 0.05f);
            CHECK(std::abs(com.x()) > 0.5f);
            CHECK(p.num_in_contact > 0);
        }
    }
}
