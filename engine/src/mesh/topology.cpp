#include "hp/mesh/topology.hpp"

#include <algorithm>
#include <stdexcept>

namespace hp {

namespace {

// A tet face: sorted vertices identify it, `oriented` keeps the outward winding.
struct Face {
    Triangle key;
    Triangle oriented;
};

// Outward-wound faces of a positively oriented tet (a, b, c, d).
constexpr int kOutwardFaces[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};

std::vector<Edge> unique_edges(const TetMesh& mesh) {
    std::vector<Edge> edges;
    edges.reserve(mesh.tets.size() * 6);
    for (const Tet& t : mesh.tets) {
        for (size_t a = 0; a < 4; ++a)
            for (size_t b = a + 1; b < 4; ++b) {
                edges.push_back({std::min(t[a], t[b]), std::max(t[a], t[b])});
            }
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    return edges;
}

// Faces that appear once after sorting by key are on the boundary.
std::vector<Triangle> boundary_faces(const TetMesh& mesh) {
    std::vector<Face> faces;
    faces.reserve(mesh.tets.size() * 4);
    for (const Tet& t : mesh.tets) {
        for (const auto& f : kOutwardFaces) {
            Face face;
            face.oriented = {t[static_cast<size_t>(f[0])], t[static_cast<size_t>(f[1])],
                             t[static_cast<size_t>(f[2])]};
            face.key = face.oriented;
            std::sort(face.key.begin(), face.key.end());
            faces.push_back(face);
        }
    }
    std::sort(faces.begin(), faces.end(),
              [](const Face& x, const Face& y) { return x.key < y.key; });

    std::vector<Triangle> surface;
    for (size_t i = 0; i < faces.size();) {
        size_t j = i + 1;
        while (j < faces.size() && faces[j].key == faces[i].key) ++j;
        if (j - i > 2) throw std::invalid_argument("a face is shared by more than two tets");
        if (j - i == 1) surface.push_back(faces[i].oriented);
        i = j;
    }
    return surface;
}

}  // namespace

Topology build_topology(const TetMesh& mesh) {
    Topology topo;
    topo.edges = unique_edges(mesh);
    topo.surface_triangles = boundary_faces(mesh);

    // CSR neighbours: count degrees, prefix-sum, then fill. Edges are sorted, so each
    // vertex's neighbour list comes out in increasing order.
    const auto n = static_cast<size_t>(mesh.num_vertices());
    topo.neighbor_offsets.assign(n + 1, 0);
    for (const Edge& e : topo.edges) {
        ++topo.neighbor_offsets[static_cast<size_t>(e[0]) + 1];
        ++topo.neighbor_offsets[static_cast<size_t>(e[1]) + 1];
    }
    for (size_t v = 0; v < n; ++v) topo.neighbor_offsets[v + 1] += topo.neighbor_offsets[v];

    topo.neighbors.resize(topo.edges.size() * 2);
    std::vector<Index> cursor(topo.neighbor_offsets.begin(), topo.neighbor_offsets.end() - 1);
    // Lower neighbours first (from edges where v is the larger end), then higher ones.
    for (const Edge& e : topo.edges) {
        topo.neighbors[static_cast<size_t>(cursor[static_cast<size_t>(e[1])]++)] = e[0];
    }
    for (const Edge& e : topo.edges) {
        topo.neighbors[static_cast<size_t>(cursor[static_cast<size_t>(e[0])]++)] = e[1];
    }
    return topo;
}

}  // namespace hp
