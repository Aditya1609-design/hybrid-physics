#include "render.hpp"

#include <cmath>
#include <cstring>

#include "core/collider_shapes.hpp"
#include "raymath.h"
#include "rlgl.h"

namespace hp::viewer {

namespace {

Vector3 rl(const Vec3& v) { return {v.x(), v.y(), v.z()}; }

Color rl(Rgb c, unsigned char alpha = 255) { return {c.r, c.g, c.b, alpha}; }

constexpr Rgb kColliderColor{196, 160, 120};
constexpr Rgb kGroundColor{58, 62, 70};
constexpr Rgb kGridColor{82, 87, 97};
constexpr Rgb kWireOnSolid{24, 26, 30};
constexpr unsigned char kColliderAlpha = 110;

// Shaded color for a collider face with unit normal n.
Color collider_face(const Vec3& n, const OrbitCamera& cam) {
    return rl(shade(kColliderColor, n, key_light_dir(), -cam.forward()), kColliderAlpha);
}

}  // namespace

Vec3 key_light_dir() { return Vec3(0.35f, 1.0f, 0.55f).normalized(); }

const char* display_mode_name(DisplayMode mode) {
    switch (mode) {
        case DisplayMode::Solid: return "solid";
        case DisplayMode::SolidWire: return "solid + wire";
        case DisplayMode::Wire: return "wire";
    }
    return "?";
}

DisplayMode next_display_mode(DisplayMode mode) {
    switch (mode) {
        case DisplayMode::Solid: return DisplayMode::SolidWire;
        case DisplayMode::SolidWire: return DisplayMode::Wire;
        case DisplayMode::Wire: return DisplayMode::Solid;
    }
    return DisplayMode::Solid;
}

// The mesh is unindexed (three vertices per triangle) so every triangle has its own flat
// color; raylib frees the arrays in UnloadMesh.
BodyRenderer::BodyRenderer(const Topology& topology)
    : triangles_(topology.surface_triangles),
      surface_edges_(surface_edges(topology.surface_triangles)),
      tet_edges_(topology.edges) {
    const int vertex_count = static_cast<int>(3 * triangles_.size());
    mesh_.vertexCount = vertex_count;
    mesh_.triangleCount = static_cast<int>(triangles_.size());
    const auto n = static_cast<unsigned>(vertex_count);
    mesh_.vertices = static_cast<float*>(MemAlloc(n * 3u * static_cast<unsigned>(sizeof(float))));
    mesh_.colors = static_cast<unsigned char*>(MemAlloc(n * 4u));
    UploadMesh(&mesh_, true);
    material_ = LoadMaterialDefault();
}

BodyRenderer::~BodyRenderer() {
    UnloadMesh(mesh_);
    UnloadMaterial(material_);
}

void BodyRenderer::update(const SimState& state, const std::vector<Rgb>& vertex_colors,
                          const OrbitCamera& camera) {
    positions_ = state.positions;
    colors_ = vertex_colors;
    build_surface(state.positions, triangles_, surface_);

    const Vec3 key = key_light_dir();
    const Vec3 view = -camera.forward();
    for (size_t t = 0; t < triangles_.size(); ++t) {
        const Vec3& n = surface_.normals[t];
        for (size_t k = 0; k < 3; ++k) {
            const size_t c = 3 * t + k;
            const Vec3& p = surface_.corners[c];
            std::memcpy(mesh_.vertices + 3 * c, p.data(), 3 * sizeof(float));
            const Rgb base = colors_[static_cast<size_t>(surface_.source[c])];
            const Rgb shaded = shade(base, n, key, view);
            mesh_.colors[4 * c + 0] = shaded.r;
            mesh_.colors[4 * c + 1] = shaded.g;
            mesh_.colors[4 * c + 2] = shaded.b;
            mesh_.colors[4 * c + 3] = 255;
        }
    }
    // Buffer slots follow raylib's layout: 0 positions, 3 colors.
    const int position_bytes = mesh_.vertexCount * 3 * static_cast<int>(sizeof(float));
    UpdateMeshBuffer(mesh_, 0, mesh_.vertices, position_bytes, 0);
    UpdateMeshBuffer(mesh_, 3, mesh_.colors, mesh_.vertexCount * 4, 0);
}

void BodyRenderer::draw(DisplayMode mode, bool tet_edges, const OrbitCamera& camera) const {
    if (mode != DisplayMode::Wire) DrawMesh(mesh_, material_, MatrixIdentity());

    // Lines over a solid surface are nudged towards the camera so they do not z-fight.
    const Vec3 nudge = mode == DisplayMode::Wire
                           ? Vec3(Vec3::Zero())
                           : Vec3(-camera.forward() * (0.002f * camera.distance));
    auto line_color = [&](Index v) {
        return mode == DisplayMode::Wire ? rl(colors_[static_cast<size_t>(v)]) : rl(kWireOnSolid);
    };
    auto draw_edges = [&](const std::vector<Edge>& edges, unsigned char alpha) {
        for (const Edge& e : edges) {
            Color c = line_color(e[0]);
            c.a = alpha;
            DrawLine3D(rl(positions_[static_cast<size_t>(e[0])] + nudge),
                       rl(positions_[static_cast<size_t>(e[1])] + nudge), c);
        }
    };
    if (tet_edges) {
        draw_edges(tet_edges_, mode == DisplayMode::Solid ? 255 : 120);
    } else if (mode != DisplayMode::Solid) {
        draw_edges(surface_edges_, 255);
    }
}

ColliderRenderer::ColliderRenderer() : unit_sphere_(unit_sphere_triangles(18, 32)) {}

void ColliderRenderer::draw(const std::vector<Collider>& colliders,
                            const OrbitCamera& camera) const {
    // Planes first, opaque: a ground patch centred under the camera target, snapped to the
    // grid spacing so the lines do not swim as the camera pans.
    constexpr float kHalf = 25.0f;
    constexpr float kSpacing = 0.5f;
    for (const Collider& c : colliders) {
        if (c.kind != ColliderKind::Plane) continue;
        const PlaneBasis b = plane_basis(c.normal);
        const Vec3 rel = camera.target - c.center;
        const float su = std::round(rel.dot(b.u) / kSpacing) * kSpacing;
        const float sv = std::round(rel.dot(b.v) / kSpacing) * kSpacing;
        const Vec3 o = c.center + su * b.u + sv * b.v;
        const Vec3 p0 = o - kHalf * b.u - kHalf * b.v;
        const Vec3 p1 = o + kHalf * b.u - kHalf * b.v;
        const Vec3 p2 = o + kHalf * b.u + kHalf * b.v;
        const Vec3 p3 = o - kHalf * b.u + kHalf * b.v;
        const Color top = rl(kGroundColor);
        const Color under = rl(Rgb{40, 42, 48});
        // u x v = n, so (p0, p1, p2) is counter-clockwise seen from the normal side.
        DrawTriangle3D(rl(p0), rl(p1), rl(p2), top);
        DrawTriangle3D(rl(p0), rl(p2), rl(p3), top);
        DrawTriangle3D(rl(p0), rl(p2), rl(p1), under);
        DrawTriangle3D(rl(p0), rl(p3), rl(p2), under);
        const Vec3 lift = 0.001f * c.normal;
        const int lines = static_cast<int>(2.0f * kHalf / kSpacing);
        for (int i = 0; i <= lines; ++i) {
            const float s = -kHalf + kSpacing * static_cast<float>(i);
            const Color g = rl(kGridColor);
            DrawLine3D(rl(o + s * b.u - kHalf * b.v + lift),
                       rl(o + s * b.u + kHalf * b.v + lift), g);
            DrawLine3D(rl(o - kHalf * b.u + s * b.v + lift),
                       rl(o + kHalf * b.u + s * b.v + lift), g);
        }
    }

    // Translucent solids: back faces culled (convex shapes need no sorting) and no depth
    // writes, so the body and other colliders stay visible behind them.
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (const Collider& c : colliders) {
        if (c.kind == ColliderKind::Sphere) {
            const std::vector<Vec3>& u = unit_sphere_;
            for (size_t t = 0; t < u.size(); t += 3) {
                const Vec3 n = (u[t] + u[t + 1] + u[t + 2]).normalized();
                DrawTriangle3D(rl(c.center + c.radius * u[t]), rl(c.center + c.radius * u[t + 1]),
                               rl(c.center + c.radius * u[t + 2]), collider_face(n, camera));
            }
        } else if (c.kind == ColliderKind::Box) {
            const auto corners = box_corners(c);
            for (const auto& f : box_faces()) {
                const Vec3& a = corners[static_cast<size_t>(f[0])];
                const Vec3& b = corners[static_cast<size_t>(f[1])];
                const Vec3& cc = corners[static_cast<size_t>(f[2])];
                const Vec3& d = corners[static_cast<size_t>(f[3])];
                const Color col = collider_face((b - a).cross(cc - a).normalized(), camera);
                DrawTriangle3D(rl(a), rl(b), rl(cc), col);
                DrawTriangle3D(rl(a), rl(cc), rl(d), col);
            }
        }
    }
    rlDrawRenderBatchActive();
    rlEnableDepthMask();

    // Outlines on top of the translucent faces.
    const Color outline = rl(Rgb{120, 96, 70});
    for (const Collider& c : colliders) {
        if (c.kind == ColliderKind::Box) {
            const auto corners = box_corners(c);
            for (const auto& e : box_edges()) {
                DrawLine3D(rl(corners[static_cast<size_t>(e[0])]),
                           rl(corners[static_cast<size_t>(e[1])]), outline);
            }
        } else if (c.kind == ColliderKind::Sphere) {
            // Three great circles: enough to read the size and position.
            constexpr int kSegments = 64;
            const Vec3 axes[3][2] = {{Vec3::UnitX(), Vec3::UnitY()},
                                     {Vec3::UnitY(), Vec3::UnitZ()},
                                     {Vec3::UnitZ(), Vec3::UnitX()}};
            for (const auto& ax : axes) {
                for (int i = 0; i < kSegments; ++i) {
                    const float a0 = 2.0f * PI * static_cast<float>(i) / kSegments;
                    const float a1 = 2.0f * PI * static_cast<float>(i + 1) / kSegments;
                    const Vec3 q0 =
                        c.center + c.radius * (std::cos(a0) * ax[0] + std::sin(a0) * ax[1]);
                    const Vec3 q1 =
                        c.center + c.radius * (std::cos(a1) * ax[0] + std::sin(a1) * ax[1]);
                    DrawLine3D(rl(q0), rl(q1), outline);
                }
            }
        }
    }
}

}  // namespace hp::viewer
