#include "hp/scene/make_state.hpp"

#include "hp/mesh/mass.hpp"

namespace hp {

SimState make_state(const TetMesh& mesh, Real density) {
    mesh.validate();
    const std::vector<Real> mass = lumped_masses(mesh, density);
    const auto n = mesh.rest_positions.size();

    SimState s;
    s.positions = mesh.rest_positions;
    s.velocities.assign(n, Vec3::Zero());
    s.rest_positions = mesh.rest_positions;
    s.inv_mass.resize(n);
    for (size_t v = 0; v < n; ++v) s.inv_mass[v] = Real(1) / mass[v];
    s.region_id.assign(n, 0);
    s.solver_owner.assign(n, SolverId::Xpbd);
    s.body_id.assign(n, 0);
    s.bodies.push_back({0, mesh.num_vertices()});
    return s;
}

}  // namespace hp
