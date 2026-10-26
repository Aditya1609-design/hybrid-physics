#include <doctest/doctest.h>

#include <algorithm>
#include <vector>

#include "hp/mesh/generators.hpp"
#include "hp/metrics/constraint_error.hpp"
#include "hp/metrics/energy.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"
#include "hp/solver/detail/xpbd_kernel.hpp"
#include "hp/solver/xpbd.hpp"

namespace {

constexpr float kBeamLength = 2.0f;

// Cantilever: small beam along x, pinned at the x = min end, bending under gravity.
struct Cantilever {
    hp::TetMesh mesh;
    hp::Material material;
    hp::SimState state;
};

Cantilever make_cantilever(float edge_compliance, float volume_compliance, float damping) {
    hp::BeamParams params;
    params.length = kBeamLength;
    params.cells_along = 8;
    params.cells_across = 2;
    Cantilever c{hp::make_beam(params), {}, {}};
    c.material.edge_compliance = edge_compliance;
    c.material.volume_compliance = volume_compliance;
    c.material.damping = damping;
    c.state = hp::make_state(c.mesh, c.material.density);
    hp::pin_vertices(c.state, [](const hp::Vec3& p) { return p.x() < -1.0f + 1e-4f; });
    return c;
}

hp::XpbdSolver make_solver(const hp::TetMesh& mesh, const hp::Material& material,
                           const hp::XpbdSettings& settings) {
    return hp::XpbdSolver(hp::build_constraints(mesh, hp::build_topology(mesh), material),
                          material, settings);
}

hp::XpbdSettings with_substeps(hp::Index substeps) {
    hp::XpbdSettings s;
    s.substeps = substeps;
    s.iterations = 1;
    return s;
}

// Runs `frames` frames and returns the final state.
hp::SimState simulate(const Cantilever& c, const hp::XpbdSettings& settings, int frames,
                      hp::ConstraintError* error = nullptr) {
    hp::XpbdSolver solver = make_solver(c.mesh, c.material, settings);
    hp::SimState s = c.state;
    const hp::RegionMask mask = hp::RegionMask::all(s.num_vertices());
    for (int f = 0; f < frames; ++f) solver.step(s, mask, hp::kFrameDt);
    if (error) *error = hp::constraint_error(s, solver.constraints());
    return s;
}

float lowest_y(const hp::SimState& s) {
    float y = s.positions[0].y();
    for (const hp::Vec3& p : s.positions) y = std::min(y, p.y());
    return y;
}

double max_distance(const std::vector<hp::Vec3>& a, const std::vector<hp::Vec3>& b) {
    double d = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        d = std::max(d, static_cast<double>((a[i] - b[i]).norm()));
    }
    return d;
}

}  // namespace

TEST_CASE("constraint error and sag converge as substeps increase") {
    // Stiff, damped cantilever settling towards its static equilibrium.
    const Cantilever c = make_cantilever(1e-8f, 1e-9f, 8.0f);
    double prev_error = 1e9;
    float prev_sag = -1e9f;
    for (const hp::Index substeps : {2, 4, 8, 16, 32, 64, 128, 256}) {
        hp::ConstraintError err;
        const hp::SimState s = simulate(c, with_substeps(substeps), 120, &err);
        const float sag = lowest_y(s);
        MESSAGE("substeps " << substeps << ": edge_rms " << err.edge_rms << ", volume_rms "
                            << err.volume_rms << ", lowest y " << sag);
        CHECK(err.edge_rms < prev_error);
        CHECK(sag > prev_sag);  // less stretch, so less sag
        CHECK(err.inverted_tets == 0);
        prev_error = err.edge_rms;
        prev_sag = sag;
    }
}

TEST_CASE("reference preset is much more accurate than the real-time preset") {
    // With zero compliance the exact answer has no stretch at all, so all error is solver
    // error. A compliant beam really stretches under load, which puts a floor under the error
    // of any solver, so the ratio there is smaller.
    struct Case {
        float compliance;
        double min_ratio;
    };
    for (const Case& k : {Case{0.0f, 100.0}, Case{1e-8f, 2.0}}) {
        CAPTURE(k.compliance);
        const Cantilever c = make_cantilever(k.compliance, 0.1f * k.compliance, 8.0f);
        hp::ConstraintError rt_err, ref_err;
        const hp::SimState rt = simulate(c, hp::XpbdSettings::realtime(), 120, &rt_err);
        const hp::SimState ref = simulate(c, hp::XpbdSettings::reference(), 120, &ref_err);
        MESSAGE("compliance " << k.compliance << ": edge_rms real-time " << rt_err.edge_rms
                              << ", reference " << ref_err.edge_rms << " (ratio "
                              << rt_err.edge_rms / ref_err.edge_rms << "); volume_rms "
                              << rt_err.volume_rms << " vs " << ref_err.volume_rms
                              << "; lowest y " << lowest_y(rt) << " vs " << lowest_y(ref));
        CHECK(ref_err.edge_rms * k.min_ratio < rt_err.edge_rms);
        CHECK(ref_err.volume_rms * k.min_ratio < rt_err.volume_rms);
        CHECK(lowest_y(rt) < lowest_y(ref));  // real-time is stretchier, so it sags further
    }
}

TEST_CASE("reference preset is converged: doubling its substeps barely moves the result") {
    // Dynamic case, no damping: half a second of the beam swinging down from rest.
    const Cantilever c = make_cantilever(1e-6f, 1e-7f, 0.0f);
    const hp::XpbdSettings ref = hp::XpbdSettings::reference();
    const hp::SimState a = simulate(c, ref, 30);
    const hp::SimState b = simulate(c, with_substeps(2 * ref.substeps), 30);
    const double rel = max_distance(a.positions, b.positions) / kBeamLength;
    MESSAGE("reference vs 2x substeps: max distance / length = " << rel);
    CHECK(rel < 1e-3);
}

TEST_CASE("float32 kernel stays close to the same kernel in double") {
    const Cantilever c = make_cantilever(1e-6f, 1e-7f, 0.0f);
    hp::Topology topo = hp::build_topology(c.mesh);
    const hp::XpbdConstraints constraints = hp::build_constraints(c.mesh, topo, c.material);

    for (const hp::XpbdSettings& settings :
         {hp::XpbdSettings::realtime(), hp::XpbdSettings::reference()}) {
        const int frames = 30;
        const hp::SimState f = simulate(c, settings, frames);

        using V3d = hp::detail::XpbdKernel<double>::V3;
        std::vector<V3d> x, v;
        std::vector<double> w;
        for (size_t i = 0; i < c.state.positions.size(); ++i) {
            x.push_back(c.state.positions[i].cast<double>());
            v.push_back(c.state.velocities[i].cast<double>());
            w.push_back(c.state.inv_mass[i]);
        }
        hp::detail::XpbdKernel<double> kernel;
        for (int i = 0; i < frames; ++i) {
            kernel.step(constraints, x, v, w, static_cast<double>(hp::kFrameDt),
                        settings.substeps, settings.iterations, settings.gravity.cast<double>(),
                        static_cast<double>(c.material.damping));
        }
        double dx = 0.0, dv = 0.0, vmax = 0.0;
        for (size_t i = 0; i < x.size(); ++i) {
            dx = std::max(dx, (f.positions[i].cast<double>() - x[i]).norm());
            dv = std::max(dv, (f.velocities[i].cast<double>() - v[i]).norm());
            vmax = std::max(vmax, v[i].norm());
        }
        MESSAGE(settings.substeps << " substeps: float vs double max position diff / length "
                                  << dx / kBeamLength << ", max velocity diff " << dv
                                  << " m/s (max speed " << vmax << ")");
        CHECK(dx / kBeamLength < 1e-3);
        CHECK(dv < 1e-2 * vmax);
    }
}

TEST_CASE("volume constraints limit volume change under load") {
    // Soft edges, so the edge network alone would let the volume change noticeably.
    const Cantilever c = make_cantilever(1e-5f, 1e-9f, 8.0f);
    const hp::XpbdSettings settings = hp::XpbdSettings::realtime();

    hp::XpbdSolver with_volume = make_solver(c.mesh, c.material, settings);
    hp::XpbdConstraints edges_only = with_volume.constraints();
    edges_only.tets.clear();
    edges_only.tet_rest_volume.clear();
    edges_only.tet_compliance.clear();
    hp::XpbdSolver without_volume(edges_only, c.material, settings);

    hp::SimState a = c.state, b = c.state;
    const hp::RegionMask mask = hp::RegionMask::all(a.num_vertices());
    for (int f = 0; f < 120; ++f) {
        with_volume.step(a, mask, hp::kFrameDt);
        without_volume.step(b, mask, hp::kFrameDt);
    }
    // Measure both against the full constraint set so volumes are compared the same way.
    const hp::ConstraintError ea = hp::constraint_error(a, with_volume.constraints());
    const hp::ConstraintError eb = hp::constraint_error(b, with_volume.constraints());
    MESSAGE("volume_rms with volume constraints " << ea.volume_rms << ", without "
                                                  << eb.volume_rms);
    CHECK(ea.volume_rms * 5.0 < eb.volume_rms);
    CHECK(ea.inverted_tets == 0);
}

TEST_CASE("a squashed box recovers its rest volume") {
    hp::BoxParams params;
    params.cells_x = params.cells_y = params.cells_z = 4;
    const hp::TetMesh mesh = hp::make_box(params);
    hp::Material material;
    material.edge_compliance = 1e-5f;
    material.volume_compliance = 1e-6f;
    material.damping = 4.0f;
    hp::XpbdSettings settings = hp::XpbdSettings::realtime();
    settings.gravity = hp::Vec3::Zero();
    hp::XpbdSolver solver = make_solver(mesh, material, settings);

    hp::SimState s = hp::make_state(mesh, material.density);
    for (hp::Vec3& p : s.positions) p.y() *= 0.5f;
    const hp::ConstraintError before = hp::constraint_error(s, solver.constraints());
    const hp::RegionMask mask = hp::RegionMask::all(s.num_vertices());
    for (int f = 0; f < 180; ++f) solver.step(s, mask, hp::kFrameDt);
    const hp::ConstraintError after = hp::constraint_error(s, solver.constraints());
    MESSAGE("volume_rms before " << before.volume_rms << ", after " << after.volume_rms
                                 << ", max " << after.volume_max);
    CHECK(before.volume_rms == doctest::Approx(0.5).epsilon(1e-3));
    CHECK(after.volume_max < 1e-2);
    CHECK(after.inverted_tets == 0);
}

TEST_CASE("locally inverted tets recover") {
    // Push the centre vertex 1.5 cells sideways, through its neighbours, inverting the tets
    // around it. (A fully mirrored body does not recover; see the XPBD ADR.)
    hp::BoxParams params;
    params.cells_x = params.cells_y = params.cells_z = 4;
    const hp::TetMesh mesh = hp::make_box(params);
    hp::Material material;
    material.edge_compliance = 1e-5f;
    material.volume_compliance = 1e-6f;
    material.damping = 4.0f;
    hp::XpbdSettings settings = hp::XpbdSettings::realtime();
    settings.gravity = hp::Vec3::Zero();
    hp::XpbdSolver solver = make_solver(mesh, material, settings);

    hp::SimState s = hp::make_state(mesh, material.density);
    size_t centre = 0;
    for (size_t v = 0; v < s.positions.size(); ++v) {
        if (s.positions[v].norm() < s.positions[centre].norm()) centre = v;
    }
    s.positions[centre] += hp::Vec3(1.5f * 0.25f, 1.5f * 0.1f, 0.0f);
    REQUIRE(hp::constraint_error(s, solver.constraints()).inverted_tets > 0);

    const hp::RegionMask mask = hp::RegionMask::all(s.num_vertices());
    for (int f = 0; f < 180; ++f) solver.step(s, mask, hp::kFrameDt);
    const hp::ConstraintError after = hp::constraint_error(s, solver.constraints());
    MESSAGE("after 180 frames: inverted " << after.inverted_tets << ", edge_max "
                                          << after.edge_max << ", volume_max "
                                          << after.volume_max);
    CHECK(after.inverted_tets == 0);
    CHECK(after.edge_max < 1e-3);
    CHECK(after.volume_max < 1e-4);
}

TEST_CASE("real-time preset is stable and never gains energy") {
    // Undamped default material: XPBD only loses energy, so the total must not rise beyond
    // small fluctuations. The tolerance is relative to the energy exchanged (the largest
    // drop in potential), since the total itself starts near zero.
    const Cantilever c = make_cantilever(1e-6f, 1e-7f, 0.0f);
    hp::XpbdSolver solver = make_solver(c.mesh, c.material, hp::XpbdSettings::realtime());
    hp::SimState s = c.state;
    const hp::Vec3 g = solver.settings().gravity;
    const hp::Energy e0 = hp::energy(s, solver.constraints(), g);
    const hp::RegionMask mask = hp::RegionMask::all(s.num_vertices());
    double highest = e0.total(), exchanged = 0.0;
    for (int f = 0; f < 1000; ++f) {
        solver.step(s, mask, hp::kFrameDt);
        const hp::Energy e = hp::energy(s, solver.constraints(), g);
        highest = std::max(highest, e.total());
        exchanged = std::max(exchanged, e0.gravitational - e.gravitational);
    }
    MESSAGE("initial energy " << e0.total() << " J, highest " << highest
                              << " J, energy exchanged " << exchanged << " J");
    REQUIRE(exchanged > 100.0);
    CHECK(highest <= e0.total() + 1e-3 * exchanged);
    for (const hp::Vec3& p : s.positions) REQUIRE(p.allFinite());
    CHECK(hp::constraint_error(s, solver.constraints()).inverted_tets == 0);
}
