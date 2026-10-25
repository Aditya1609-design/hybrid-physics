#pragma once

#include "hp/core/types.hpp"
#include "hp/mesh/tet_mesh.hpp"

namespace hp {

// Axis-aligned box centred at `center`, split into cells_x * cells_y * cells_z grid cells
// of 5 tets each (alternating split, so neighbouring cells share face diagonals).
struct BoxParams {
    Vec3 size = Vec3(1.0f, 1.0f, 1.0f);
    Index cells_x = 8;
    Index cells_y = 8;
    Index cells_z = 8;
    Vec3 center = Vec3::Zero();
};

// Beam along the x axis with a square cross-section.
struct BeamParams {
    Real length = 2.0f;
    Real thickness = 0.5f;
    Index cells_along = 20;
    Index cells_across = 5;
    Vec3 center = Vec3::Zero();
};

// Voxelized sphere: the grid cells of a cube around the sphere whose centres lie inside it.
// The surface is blocky (stair-stepped); its volume approaches 4/3 pi r^3 as cells shrink.
struct SphereParams {
    Real radius = 0.5f;
    Index cells_per_diameter = 10;
    Vec3 center = Vec3::Zero();
};

// The generators throw std::invalid_argument for non-positive sizes or cell counts.
TetMesh make_box(const BoxParams& params);
TetMesh make_beam(const BeamParams& params);
TetMesh make_sphere(const SphereParams& params);

}  // namespace hp
