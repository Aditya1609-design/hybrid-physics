#pragma once

#include <array>

#include "hp/collision/collider.hpp"
#include "hp/core/types.hpp"

namespace hp::viewer {

// Two unit tangents of a plane with unit normal n, such that u x v = n.
struct PlaneBasis {
    Vec3 u;
    Vec3 v;
};
PlaneBasis plane_basis(const Vec3& n);

// World-space corners of a box collider. Corner i takes +half extent on box axis k when
// bit k of i is set, and -half extent otherwise.
std::array<Vec3, 8> box_corners(const Collider& box);

// The 6 faces as corner indices, wound counter-clockwise seen from outside the box.
const std::array<std::array<int, 4>, 6>& box_faces();

// The 12 edges as corner index pairs.
const std::array<std::array<int, 2>, 12>& box_edges();

}  // namespace hp::viewer
