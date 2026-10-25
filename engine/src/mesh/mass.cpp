#include "hp/mesh/mass.hpp"

#include <stdexcept>

namespace hp {

std::vector<Real> lumped_masses(const TetMesh& mesh, Real density) {
    if (!(density > 0.0f)) throw std::invalid_argument("density must be positive");

    // Accumulate in double to keep rounding small at vertices shared by many tets.
    std::vector<double> mass(mesh.rest_positions.size(), 0.0);
    for (Index t = 0; t < mesh.num_tets(); ++t) {
        const double quarter = static_cast<double>(density) * tet_volume(mesh, t) / 4.0;
        for (const Index v : mesh.tets[static_cast<size_t>(t)]) {
            mass[static_cast<size_t>(v)] += quarter;
        }
    }
    std::vector<Real> out(mass.size());
    for (size_t v = 0; v < mass.size(); ++v) out[v] = static_cast<Real>(mass[v]);
    return out;
}

}  // namespace hp
