#include "hp/solver/xpbd_constraints.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hp {

void Material::validate() const {
    if (!(density > 0.0f) || !std::isfinite(density)) {
        throw std::invalid_argument("material density must be positive and finite");
    }
    for (const Real value : {edge_compliance, volume_compliance, damping}) {
        if (!(value >= 0.0f) || !std::isfinite(value)) {
            throw std::invalid_argument("material compliances and damping must be >= 0");
        }
    }
}

namespace {

// Position of edge (a, b) in the sorted unique edge list.
size_t edge_index(const Topology& topo, Index a, Index b) {
    const Edge key = a < b ? Edge{a, b} : Edge{b, a};
    const auto it = std::lower_bound(topo.edges.begin(), topo.edges.end(), key);
    if (it == topo.edges.end() || *it != key) {
        throw std::invalid_argument("topology does not match the mesh");
    }
    return static_cast<size_t>(it - topo.edges.begin());
}

}  // namespace

XpbdConstraints build_constraints(const TetMesh& mesh, const Topology& topo,
                                  const Material& material) {
    material.validate();
    const auto num_edges = topo.edges.size();
    const auto num_tets = mesh.tets.size();

    XpbdConstraints c;
    c.edges = topo.edges;
    c.tets = mesh.tets;
    c.tet_rest_volume.resize(num_tets);
    c.tet_compliance.resize(num_tets);

    // Volume share of each edge, accumulated in double.
    static constexpr std::pair<int, int> kTetEdges[6] = {{0, 1}, {0, 2}, {0, 3},
                                                         {1, 2}, {1, 3}, {2, 3}};
    std::vector<double> edge_volume(num_edges, 0.0);
    for (size_t t = 0; t < num_tets; ++t) {
        const Real v0 = tet_volume(mesh, static_cast<Index>(t));
        c.tet_rest_volume[t] = v0;
        c.tet_compliance[t] = material.volume_compliance * v0;
        const Tet& tet = mesh.tets[t];
        for (const auto& [i, j] : kTetEdges) {
            edge_volume[edge_index(topo, tet[static_cast<size_t>(i)],
                                   tet[static_cast<size_t>(j)])] += v0 / 6.0;
        }
    }

    c.edge_rest_length.resize(num_edges);
    c.edge_compliance.resize(num_edges);
    for (size_t e = 0; e < num_edges; ++e) {
        const Vec3 d = mesh.rest_positions[static_cast<size_t>(c.edges[e][0])] -
                       mesh.rest_positions[static_cast<size_t>(c.edges[e][1])];
        const double length = static_cast<double>(d.norm());
        c.edge_rest_length[e] = static_cast<Real>(length);
        c.edge_compliance[e] = static_cast<Real>(static_cast<double>(material.edge_compliance) *
                                                 length * length / edge_volume[e]);
    }
    return c;
}

}  // namespace hp
