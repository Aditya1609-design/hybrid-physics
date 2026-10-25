#include "hp/mesh/graph.hpp"

namespace hp {

MeshGraph build_graph(const TetMesh& mesh, const Topology& topo) {
    const size_t e = topo.edges.size();
    MeshGraph g;
    g.num_nodes = mesh.num_vertices();
    g.senders.resize(2 * e);
    g.receivers.resize(2 * e);
    g.rest_lengths.resize(2 * e);
    g.rel_rest_pos.resize(2 * e);

    // Forward edges fill the first half, their reverses the second half.
    for (size_t k = 0; k < e; ++k) {
        const Index i = topo.edges[k][0], j = topo.edges[k][1];
        const Vec3 d = mesh.rest_positions[static_cast<size_t>(i)] -
                       mesh.rest_positions[static_cast<size_t>(j)];
        const Real len = d.norm();
        g.senders[k] = i;
        g.receivers[k] = j;
        g.rel_rest_pos[k] = d;
        g.rest_lengths[k] = len;
        g.senders[k + e] = j;
        g.receivers[k + e] = i;
        g.rel_rest_pos[k + e] = -d;
        g.rest_lengths[k + e] = len;
    }
    return g;
}

void node_features(const SimState& state, Real* out) {
    const auto n = static_cast<size_t>(state.num_vertices());
    const auto dim = static_cast<size_t>(kNodeFeatureDim);
    for (size_t v = 0; v < n; ++v) {
        Real* row = out + v * dim;
        row[0] = state.velocities[v].x();
        row[1] = state.velocities[v].y();
        row[2] = state.velocities[v].z();
        const bool pinned = state.inv_mass[v] == Real(0);
        row[3 + static_cast<size_t>(NodeType::Free)] = pinned ? Real(0) : Real(1);
        row[3 + static_cast<size_t>(NodeType::Pinned)] = pinned ? Real(1) : Real(0);
    }
}

std::vector<Real> node_features(const SimState& state) {
    std::vector<Real> out(static_cast<size_t>(state.num_vertices()) *
                          static_cast<size_t>(kNodeFeatureDim));
    node_features(state, out.data());
    return out;
}

}  // namespace hp
