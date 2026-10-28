#pragma once

#include <string>
#include <vector>

#include "hp/core/sim_state.hpp"
#include "hp/mesh/tet_mesh.hpp"
#include "hp/mesh/topology.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/solver/xpbd.hpp"

namespace hp {

// A scenario turned into objects that can be stepped. The solver holds the constraints,
// settings and colliders, so metrics can read them from it.
struct Scene {
    TetMesh mesh;  // rest shape in the initial pose
    Topology topology;
    SimState state;
    XpbdSolver solver;
    std::vector<std::string> warnings;  // setups that run but may misbehave; for the caller to show
};

// Builds the mesh, places and spins it, pins vertices and creates the solver.
// Throws std::invalid_argument if the scenario has more than one body, a pin box selects no
// vertex, or any vertex (pinned or free) starts inside a collider. Adds a warning for sphere
// colliders smaller than about two mesh edge lengths, and for sphere or box colliders thin
// enough that a vertex could cross half of them in one substep.
Scene build_scene(const Scenario& scenario);

}  // namespace hp
