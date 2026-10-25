#pragma once

#include <vector>

#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"
#include "hp/mesh/tet_mesh.hpp"
#include "hp/mesh/topology.hpp"

namespace hp {

// Static graph view of a mesh for GNN input (MeshGraphNets layout).
// Each undirected edge k = (i, j), i < j, appears twice: directed edge k is i -> j and
// directed edge k + E is j -> i. senders/receivers together form PyTorch's edge_index [2, 2E].
struct MeshGraph {
    Index num_nodes = 0;
    std::vector<Index> senders;      // [2E]
    std::vector<Index> receivers;    // [2E]
    std::vector<Real> rest_lengths;  // [2E], |rel_rest_pos|
    std::vector<Vec3> rel_rest_pos;  // [2E, 3], rest[sender] - rest[receiver]

    Index num_directed_edges() const { return static_cast<Index>(senders.size()); }
};

MeshGraph build_graph(const TetMesh& mesh, const Topology& topo);

// Node types, one-hot encoded in the node features.
enum class NodeType : Index { Free = 0, Pinned = 1 };
constexpr Index kNumNodeTypes = 2;

// Per-node features, row-major [N, kNodeFeatureDim]: velocity (3), then the one-hot node
// type (pinned when inv_mass == 0). Provisional layout; the GNN phase may add fields.
constexpr Index kNodeFeatureDim = 3 + kNumNodeTypes;

// Writes into `out`, which must hold num_vertices * kNodeFeatureDim values. Lets callers
// reuse one buffer every step.
void node_features(const SimState& state, Real* out);

// Same, allocating the result.
std::vector<Real> node_features(const SimState& state);

}  // namespace hp
