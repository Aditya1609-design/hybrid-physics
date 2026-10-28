#include "hp/scene/scene.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include "hp/mesh/generators.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"

namespace hp {

namespace {

// printf-style formatting for short messages.
template <typename... Args>
std::string format(const char* fmt, Args... args) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, args...);
    return buf;
}

const char* kind_name(ColliderKind kind) {
    switch (kind) {
        case ColliderKind::Plane: return "plane";
        case ColliderKind::Sphere: return "sphere";
        case ColliderKind::Box: return "box";
    }
    return "collider";
}

// Without a rotation the generator places the mesh itself, so a scenario gives exactly the
// mesh the C++ API gives for the same centre. With one, the mesh is built at the origin,
// rotated (a proper rotation keeps tets positively oriented) and then moved.
TetMesh make_body_mesh(const ScenarioBody& body) {
    const bool rotated = body.rotation != Mat3::Identity();
    const Vec3 center = rotated ? Vec3::Zero() : body.position;
    TetMesh mesh = std::visit(
        [&](auto params) {
            params.center = center;
            using P = decltype(params);
            if constexpr (std::is_same_v<P, BoxParams>) {
                return make_box(params);
            } else if constexpr (std::is_same_v<P, BeamParams>) {
                return make_beam(params);
            } else {
                return make_sphere(params);
            }
        },
        body.mesh);
    if (rotated) {
        for (Vec3& p : mesh.rest_positions) p = body.rotation * p + body.position;
    }
    return mesh;
}

// v + w x (x - c) per vertex, with c the centre of mass. Computed in double; a pure
// translation stays exact.
void set_velocities(SimState& state, const ScenarioBody& body) {
    if (body.angular_velocity == Vec3::Zero()) {
        state.velocities.assign(state.velocities.size(), body.velocity);
        return;
    }
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    double total = 0.0;
    for (size_t v = 0; v < state.positions.size(); ++v) {
        const double m = 1.0 / static_cast<double>(state.inv_mass[v]);
        com += m * state.positions[v].cast<double>();
        total += m;
    }
    com /= total;
    const Eigen::Vector3d w = body.angular_velocity.cast<double>();
    const Eigen::Vector3d v0 = body.velocity.cast<double>();
    for (size_t v = 0; v < state.positions.size(); ++v) {
        const Eigen::Vector3d r = state.positions[v].cast<double>() - com;
        state.velocities[v] = (v0 + w.cross(r)).cast<Real>();
    }
}

// XPBD turns an initial overlap into a large velocity (depth / substep), so it is an error.
void reject_overlaps(const SimState& state, const std::vector<Collider>& colliders) {
    for (size_t c = 0; c < colliders.size(); ++c) {
        Index count = 0;
        size_t deepest = 0;
        double depth = 0.0;
        for (size_t v = 0; v < state.positions.size(); ++v) {
            Eigen::Vector3d n;
            const double d =
                signed_distance<double>(colliders[c], state.positions[v].cast<double>(), n);
            if (d >= 0.0) continue;
            ++count;
            if (-d > depth) {
                depth = -d;
                deepest = v;
            }
        }
        if (count > 0) {
            throw std::invalid_argument(format(
                "scenario: bodies[0] starts inside colliders[%zu] (%s): %d vertices penetrate, "
                "deepest is vertex %zu at %.3g m; bodies must start outside colliders",
                c, kind_name(colliders[c].kind), count, deepest, depth));
        }
    }
}

double mean_edge_length(const SimState& state, const Topology& topo) {
    double sum = 0.0;
    for (const Edge& e : topo.edges) {
        sum += (state.positions[static_cast<size_t>(e[0])] -
                state.positions[static_cast<size_t>(e[1])])
                   .cast<double>()
                   .norm();
    }
    return topo.edges.empty() ? 0.0 : sum / static_cast<double>(topo.edges.size());
}

// Upper bound on any vertex speed during the run: the fastest initial speed plus what
// gravity can add over all frames. Loose on purpose; it only decides whether to warn.
double speed_bound(const SimState& state, const Scenario& s) {
    double v_max = 0.0;
    for (size_t v = 0; v < state.velocities.size(); ++v) {
        if (state.inv_mass[v] == 0.0f) continue;
        v_max = std::max(v_max, state.velocities[v].cast<double>().norm());
    }
    return v_max + s.solver.gravity.cast<double>().norm() * static_cast<double>(s.frames) *
                       static_cast<double>(kFrameDt);
}

// Collider sizes that contact handling cannot cope with (see the contacts ADR).
std::vector<std::string> collider_warnings(const Scenario& s, const SimState& state,
                                           const Topology& topo) {
    std::vector<std::string> out;
    const double edge = mean_edge_length(state, topo);
    const double travel = speed_bound(state, s) * static_cast<double>(kFrameDt) /
                          static_cast<double>(s.solver.substeps);
    for (size_t c = 0; c < s.colliders.size(); ++c) {
        const Collider& col = s.colliders[c];
        if (col.kind == ColliderKind::Plane) continue;  // a half-space cannot be crossed
        if (col.kind == ColliderKind::Sphere && col.radius < 2.0 * edge) {
            out.push_back(format(
                "colliders[%zu]: sphere radius %.3g m is less than two mesh edge lengths "
                "(mean edge %.3g m); with vertex-only contact it can push into faces",
                c, static_cast<double>(col.radius), edge));
        }
        // A vertex pushed past the middle of a collider leaves through the far side.
        const double half_thickness = col.kind == ColliderKind::Sphere
                                          ? static_cast<double>(col.radius)
                                          : static_cast<double>(col.half_extents.minCoeff());
        if (travel > half_thickness) {
            out.push_back(format(
                "colliders[%zu]: %s is thin enough to tunnel through: half thickness %.3g m, "
                "but a vertex may move up to %.3g m per substep (speed bound %.3g m/s)",
                c, kind_name(col.kind), half_thickness, travel, speed_bound(state, s)));
        }
    }
    return out;
}

}  // namespace

Scene build_scene(const Scenario& scenario) {
    if (scenario.bodies.size() != 1) {
        throw std::invalid_argument(format(
            "scenario: bodies: exactly one body is supported for now, got %zu",
            scenario.bodies.size()));
    }
    const ScenarioBody& body = scenario.bodies[0];

    TetMesh mesh = make_body_mesh(body);
    Topology topology = build_topology(mesh);
    SimState state = make_state(mesh, body.material.density);
    set_velocities(state, body);

    // Pinned last, so pinned vertices end with zero velocity.
    for (size_t i = 0; i < body.pins.size(); ++i) {
        const PinBox& pin = body.pins[i];
        const Index n = pin_vertices(state, [&](const Vec3& p) {
            return (p.array() >= pin.min.array()).all() && (p.array() <= pin.max.array()).all();
        });
        if (n == 0) {
            throw std::invalid_argument(
                format("scenario: bodies[0].pins[%zu]: selects no vertex", i));
        }
    }

    reject_overlaps(state, scenario.colliders);
    std::vector<std::string> warnings = collider_warnings(scenario, state, topology);

    XpbdSolver solver(build_constraints(mesh, topology, body.material), body.material,
                      scenario.solver, scenario.colliders);
    return Scene{std::move(mesh), std::move(topology), std::move(state), std::move(solver),
                 std::move(warnings)};
}

}  // namespace hp
