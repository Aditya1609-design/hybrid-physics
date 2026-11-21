#include <doctest/doctest.h>

#include "hp/metrics/frame_metrics.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/scene/scene.hpp"

TEST_CASE("measure_frame matches the separate metric functions") {
    hp::Scene scene = hp::build_scene(hp::load_scenario(HP_SCENARIO_DIR "/cube_drop.json"));
    const hp::RegionMask mask = hp::RegionMask::all(scene.state.num_vertices());
    for (int f = 0; f < 30; ++f) scene.solver.step(scene.state, mask, hp::kFrameDt);

    const hp::XpbdSolver& s = scene.solver;
    const hp::FrameMetrics m =
        hp::measure_frame(scene.state, s.constraints(), s.colliders(), s.settings().gravity);
    const hp::Energy e = hp::energy(scene.state, s.constraints(), s.settings().gravity);
    const hp::ConstraintError c = hp::constraint_error(scene.state, s.constraints());
    const hp::Penetration p = hp::penetration(scene.state, s.colliders());

    CHECK(m.energy.kinetic == e.kinetic);
    CHECK(m.energy.gravitational == e.gravitational);
    CHECK(m.energy.elastic == e.elastic);
    CHECK(m.error.edge_rms == c.edge_rms);
    CHECK(m.error.volume_max == c.volume_max);
    CHECK(m.error.inverted_tets == c.inverted_tets);
    CHECK(m.penetration.max_depth == p.max_depth);
    CHECK(m.penetration.num_in_contact == p.num_in_contact);
    CHECK(m.energy.kinetic > 0.0);  // the cube is falling, so the check is not trivially 0 == 0
}
