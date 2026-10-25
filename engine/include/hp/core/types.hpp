#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>  // cross()
#include <cstdint>

namespace hp {

// float32 everywhere, so engine arrays match GNN tensors without conversion.
using Real = float;

// Vertex, edge and element indices. PyTorch widens them to int64 on load.
using Index = std::int32_t;

// 12 bytes, no alignment requirement: a std::vector<Vec3> is a contiguous [N,3] float array.
using Vec3 = Eigen::Matrix<Real, 3, 1>;

// Which solver computes a vertex this step. Stored per vertex in SimState.
enum class SolverId : std::uint8_t { Xpbd = 0, Gnn = 1 };

static_assert(sizeof(Vec3) == 3 * sizeof(Real), "Vec3 must be tightly packed");

}  // namespace hp
