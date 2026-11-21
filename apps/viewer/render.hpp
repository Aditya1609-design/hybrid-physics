#pragma once

#include <vector>

#include "core/colors.hpp"
#include "core/orbit_camera.hpp"
#include "core/surface_mesh.hpp"
#include "hp/collision/collider.hpp"
#include "hp/core/sim_state.hpp"
#include "hp/mesh/topology.hpp"
#include "raylib.h"

namespace hp::viewer {

enum class DisplayMode { Solid, SolidWire, Wire };

const char* display_mode_name(DisplayMode mode);
DisplayMode next_display_mode(DisplayMode mode);

// Draws the deformable body: the surface as a GPU mesh refreshed every frame with flat,
// CPU-shaded vertex colors, plus optional wireframes. Needs an open window; create it after
// InitWindow and destroy it before CloseWindow.
class BodyRenderer {
public:
    explicit BodyRenderer(const Topology& topology);
    ~BodyRenderer();
    BodyRenderer(const BodyRenderer&) = delete;
    BodyRenderer& operator=(const BodyRenderer&) = delete;

    // Rebuilds the surface from the current positions and per-vertex colors.
    void update(const SimState& state, const std::vector<Rgb>& vertex_colors,
                const OrbitCamera& camera);
    void draw(DisplayMode mode, bool tet_edges, const OrbitCamera& camera) const;

private:
    std::vector<Triangle> triangles_;
    std::vector<Edge> surface_edges_;
    std::vector<Edge> tet_edges_;
    SurfaceMesh surface_;
    std::vector<Vec3> positions_;  // copy of the last positions, for the wireframes
    std::vector<Rgb> colors_;      // copy of the last per-vertex colors
    // Qualified with :: because hp::Material (the physics material) would otherwise win name
    // lookup wherever engine headers are included first, giving this class two layouts.
    ::Mesh mesh_{};
    ::Material material_{};
};

// Draws planes as a large tiled ground around the camera target, and spheres and boxes as
// shaded translucent solids with outlines. Call after drawing the body.
class ColliderRenderer {
public:
    ColliderRenderer();
    void draw(const std::vector<Collider>& colliders, const OrbitCamera& camera) const;

private:
    std::vector<Vec3> unit_sphere_;
};

// Key light direction shared by the body and collider shading (towards the light).
Vec3 key_light_dir();

}  // namespace hp::viewer
