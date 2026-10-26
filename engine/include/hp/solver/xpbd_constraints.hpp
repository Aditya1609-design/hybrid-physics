#pragma once

#include <vector>

#include "hp/core/types.hpp"
#include "hp/mesh/tet_mesh.hpp"
#include "hp/mesh/topology.hpp"

namespace hp {

// Soft-body material. Compliances are per unit volume (1/Pa), so the same material behaves
// about the same at any mesh resolution; 0 means perfectly stiff.
struct Material {
    Real density = 1000.0f;           // kg/m^3, used to build masses
    Real edge_compliance = 1e-6f;     // 1 / Young's-like modulus of the edge network
    Real volume_compliance = 1e-7f;   // 1 / bulk-like modulus of the volume constraints
    Real damping = 0.0f;              // linear velocity damping, 1/s

    // Throws std::invalid_argument for non-positive density or negative/non-finite values.
    void validate() const;
};

// Constraint data for XPBD, as flat float32/int32 arrays in a fixed order (edges in Topology
// order, then tets in mesh order). Clearing a list disables that constraint type.
struct XpbdConstraints {
    // Distance constraints: C = |x_i - x_j| - L.
    std::vector<Edge> edges;
    std::vector<Real> edge_rest_length;
    std::vector<Real> edge_compliance;  // alpha_e = edge_compliance * L^2 / V_e

    // Volume constraints: C = V - V0.
    std::vector<Tet> tets;
    std::vector<Real> tet_rest_volume;
    std::vector<Real> tet_compliance;  // alpha_t = volume_compliance * V0

    Index num_edges() const { return static_cast<Index>(edges.size()); }
    Index num_tets() const { return static_cast<Index>(tets.size()); }
};

// Builds one distance constraint per mesh edge and one volume constraint per tet. Each tet
// gives a sixth of its volume to each of its edges; V_e is the sum an edge receives.
XpbdConstraints build_constraints(const TetMesh& mesh, const Topology& topo,
                                  const Material& material);

}  // namespace hp
