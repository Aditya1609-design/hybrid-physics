#include "hp/mesh/tet_mesh.hpp"

#include <stdexcept>
#include <string>

namespace hp {

void TetMesh::validate() const {
    const Index n = num_vertices();
    std::vector<char> used(rest_positions.size(), 0);
    for (Index t = 0; t < num_tets(); ++t) {
        const Tet& tet = tets[static_cast<size_t>(t)];
        for (size_t a = 0; a < 4; ++a) {
            if (tet[a] < 0 || tet[a] >= n) {
                throw std::invalid_argument("tet " + std::to_string(t) +
                                            " has an out-of-range vertex index");
            }
            for (size_t b = a + 1; b < 4; ++b) {
                if (tet[a] == tet[b]) {
                    throw std::invalid_argument("tet " + std::to_string(t) +
                                                " repeats a vertex");
                }
            }
            used[static_cast<size_t>(tet[a])] = 1;
        }
        if (!(tet_volume(*this, t) > 0.0f)) {
            throw std::invalid_argument("tet " + std::to_string(t) +
                                        " has non-positive volume");
        }
    }
    for (size_t v = 0; v < used.size(); ++v) {
        if (!used[v]) {
            throw std::invalid_argument("vertex " + std::to_string(v) + " is not used by any tet");
        }
    }
}

Real tet_volume(const TetMesh& mesh, Index t) {
    const Tet& tet = mesh.tets[static_cast<size_t>(t)];
    const Vec3& a = mesh.rest_positions[static_cast<size_t>(tet[0])];
    const Vec3& b = mesh.rest_positions[static_cast<size_t>(tet[1])];
    const Vec3& c = mesh.rest_positions[static_cast<size_t>(tet[2])];
    const Vec3& d = mesh.rest_positions[static_cast<size_t>(tet[3])];
    return (b - a).cross(c - a).dot(d - a) / Real(6);
}

double total_volume(const TetMesh& mesh) {
    double sum = 0.0;
    for (Index t = 0; t < mesh.num_tets(); ++t) sum += tet_volume(mesh, t);
    return sum;
}

}  // namespace hp
