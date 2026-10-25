#pragma once

#include <vector>

#include "hp/core/types.hpp"

namespace hp {

// Vertices of one body occupy a contiguous range of the per-vertex arrays.
struct BodyRange {
    Index first_vertex = 0;
    Index num_vertices = 0;
};

// Per-vertex simulation state, struct-of-arrays. Every per-vertex array has the same length,
// and each Vec3 array is a contiguous [N,3] float32 block that can be handed to Python as is.
// Topology (tets, edges) lives in TetMesh/Topology; solver scratch data lives in the solver.
struct SimState {
    std::vector<Vec3> positions;
    std::vector<Vec3> velocities;
    std::vector<Vec3> rest_positions;
    std::vector<Real> inv_mass;          // 0 means pinned (infinite mass)
    std::vector<Index> region_id;        // mesh region, used by the solver switching later
    std::vector<SolverId> solver_owner;  // which solver computes this vertex
    std::vector<Index> body_id;          // index into `bodies`
    std::vector<BodyRange> bodies;

    Index num_vertices() const { return static_cast<Index>(positions.size()); }

    // Throws std::invalid_argument if array lengths differ, bodies do not tile the vertices
    // in order, body_id disagrees with the ranges, or an inverse mass is negative or not finite.
    void validate() const;
};

}  // namespace hp
