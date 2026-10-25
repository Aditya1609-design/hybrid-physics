#pragma once

#include <string>
#include <vector>

#include "hp/core/types.hpp"
#include "hp/mesh/topology.hpp"

namespace hp {

// Writes a triangle surface as a Wavefront OBJ file, for checking meshes in an external
// viewer. Only vertices used by the triangles are written, renumbered in increasing order.
// Throws std::runtime_error if the file cannot be written.
void write_surface_obj(const std::string& path, const std::vector<Vec3>& positions,
                       const std::vector<Triangle>& triangles);

}  // namespace hp
