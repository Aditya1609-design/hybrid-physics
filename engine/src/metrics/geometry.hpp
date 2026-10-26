#pragma once

#include <vector>

#include "hp/core/types.hpp"
#include "hp/mesh/tet_mesh.hpp"
#include "hp/mesh/topology.hpp"

namespace hp {

// Double-precision edge length and signed tet volume, so metrics do not add float rounding.
inline double edge_length(const std::vector<Vec3>& x, const Edge& e) {
    const Eigen::Vector3d d = (x[static_cast<size_t>(e[0])] - x[static_cast<size_t>(e[1])])
                                  .cast<double>();
    return d.norm();
}

inline double signed_volume(const std::vector<Vec3>& x, const Tet& t) {
    const Eigen::Vector3d a = x[static_cast<size_t>(t[0])].cast<double>();
    const Eigen::Vector3d b = x[static_cast<size_t>(t[1])].cast<double>();
    const Eigen::Vector3d c = x[static_cast<size_t>(t[2])].cast<double>();
    const Eigen::Vector3d d = x[static_cast<size_t>(t[3])].cast<double>();
    return (b - a).cross(c - a).dot(d - a) / 6.0;
}

}  // namespace hp
