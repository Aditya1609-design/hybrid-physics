#include "hp/mesh/generators.hpp"

#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace hp {

namespace {

// Cube corners are numbered dx + 2*dy + 4*dz. Each parity class uses one central tet on four
// alternate corners plus four corner tets. Cells with odd i+j+k use the mirrored split, so
// face diagonals match between neighbouring cells and the mesh is conforming.
constexpr int kEvenSplit[5][4] = {{0, 3, 5, 6}, {1, 0, 3, 5}, {2, 0, 3, 6}, {4, 0, 5, 6},
                                  {7, 3, 5, 6}};
constexpr int kOddSplit[5][4] = {{1, 2, 4, 7}, {0, 1, 2, 4}, {3, 1, 2, 7}, {5, 1, 4, 7},
                                 {6, 2, 4, 7}};

// Regular grid of cells, of which only the cells flagged in `keep` become tets.
struct CellGrid {
    Vec3 origin;
    Vec3 cell_size;
    Index nx, ny, nz;
    std::vector<char> keep;  // nx*ny*nz flags, x fastest

    size_t cell(Index i, Index j, Index k) const {
        return static_cast<size_t>(i + nx * (j + ny * k));
    }
};

// Turns the kept cells into tets. Only grid points used by a kept cell become vertices,
// numbered in grid order, so the output is deterministic.
TetMesh build_from_cells(const CellGrid& g) {
    const Index px = g.nx + 1, py = g.ny + 1, pz = g.nz + 1;
    auto point = [&](Index i, Index j, Index k) {
        return static_cast<size_t>(i + px * (j + py * k));
    };

    std::vector<Index> vertex_of(static_cast<size_t>(px) * static_cast<size_t>(py) *
                                     static_cast<size_t>(pz),
                                 -1);
    for (Index k = 0; k < g.nz; ++k)
        for (Index j = 0; j < g.ny; ++j)
            for (Index i = 0; i < g.nx; ++i) {
                if (!g.keep[g.cell(i, j, k)]) continue;
                for (Index c = 0; c < 8; ++c) {
                    vertex_of[point(i + (c & 1), j + ((c >> 1) & 1), k + ((c >> 2) & 1))] = 0;
                }
            }

    TetMesh mesh;
    for (Index k = 0; k < pz; ++k)
        for (Index j = 0; j < py; ++j)
            for (Index i = 0; i < px; ++i) {
                Index& v = vertex_of[point(i, j, k)];
                if (v < 0) continue;
                v = mesh.num_vertices();
                mesh.rest_positions.push_back(
                    g.origin + Vec3(static_cast<Real>(i), static_cast<Real>(j),
                                    static_cast<Real>(k))
                                   .cwiseProduct(g.cell_size));
            }

    for (Index k = 0; k < g.nz; ++k)
        for (Index j = 0; j < g.ny; ++j)
            for (Index i = 0; i < g.nx; ++i) {
                if (!g.keep[g.cell(i, j, k)]) continue;
                Index corner[8];
                for (Index c = 0; c < 8; ++c) {
                    corner[c] =
                        vertex_of[point(i + (c & 1), j + ((c >> 1) & 1), k + ((c >> 2) & 1))];
                }
                const int(*split)[4] = ((i + j + k) % 2 == 0) ? kEvenSplit : kOddSplit;
                for (int t = 0; t < 5; ++t) {
                    const int* s = split[t];
                    mesh.tets.push_back({corner[s[0]], corner[s[1]], corner[s[2]], corner[s[3]]});
                    // Fix orientation by swapping two vertices if the volume is negative.
                    if (tet_volume(mesh, mesh.num_tets() - 1) < 0.0f) {
                        std::swap(mesh.tets.back()[2], mesh.tets.back()[3]);
                    }
                }
            }
    return mesh;
}

void require_positive(Real value, const char* what) {
    if (!(value > 0.0f)) throw std::invalid_argument(std::string(what) + " must be positive");
}

void require_positive(Index value, const char* what) {
    if (value <= 0) throw std::invalid_argument(std::string(what) + " must be positive");
}

// Product of positive factors, or -1 once it passes the int32 index range. Each factor is at
// most 2^31 and the running product stays below 2^31, so int64 never overflows here.
std::int64_t checked_product(std::initializer_list<std::int64_t> factors) {
    constexpr std::int64_t kMax = std::numeric_limits<Index>::max();
    std::int64_t product = 1;
    for (const std::int64_t f : factors) {
        product *= f;
        if (product > kMax) return -1;
    }
    return product;
}

// Grid points and tets are numbered with int32 Index, so both counts must fit. Checked before
// anything is allocated, so oversized parameters fail fast instead of overflowing.
// `cells` is the number of cells that become tets, or -1 if that count is already too large.
void require_index_range(const char* what, Index nx, Index ny, Index nz, std::int64_t cells) {
    const std::int64_t points = checked_product({std::int64_t{nx} + 1, std::int64_t{ny} + 1,
                                                 std::int64_t{nz} + 1});
    if (points < 0) {
        throw std::invalid_argument(std::string(what) +
                                    ": too many cells: the grid points exceed the int32 index range");
    }
    if (cells < 0 || checked_product({cells, 5}) < 0) {
        throw std::invalid_argument(std::string(what) +
                                    ": too many cells: the tets exceed the int32 index range");
    }
}

}  // namespace

TetMesh make_box(const BoxParams& p) {
    require_positive(p.size.x(), "box size x");
    require_positive(p.size.y(), "box size y");
    require_positive(p.size.z(), "box size z");
    require_positive(p.cells_x, "box cells_x");
    require_positive(p.cells_y, "box cells_y");
    require_positive(p.cells_z, "box cells_z");
    require_index_range("box", p.cells_x, p.cells_y, p.cells_z,
                        checked_product({p.cells_x, p.cells_y, p.cells_z}));

    CellGrid g;
    g.nx = p.cells_x;
    g.ny = p.cells_y;
    g.nz = p.cells_z;
    g.origin = p.center - p.size / Real(2);
    g.cell_size = p.size.cwiseQuotient(
        Vec3(static_cast<Real>(g.nx), static_cast<Real>(g.ny), static_cast<Real>(g.nz)));
    g.keep.assign(static_cast<size_t>(g.nx) * static_cast<size_t>(g.ny) *
                      static_cast<size_t>(g.nz),
                  1);
    return build_from_cells(g);
}

TetMesh make_beam(const BeamParams& p) {
    require_positive(p.length, "beam length");
    require_positive(p.thickness, "beam thickness");
    require_positive(p.cells_along, "beam cells_along");
    require_positive(p.cells_across, "beam cells_across");
    require_index_range("beam", p.cells_along, p.cells_across, p.cells_across,
                        checked_product({p.cells_along, p.cells_across, p.cells_across}));
    BoxParams box;
    box.size = Vec3(p.length, p.thickness, p.thickness);
    box.cells_x = p.cells_along;
    box.cells_y = p.cells_across;
    box.cells_z = p.cells_across;
    box.center = p.center;
    return make_box(box);
}

TetMesh make_sphere(const SphereParams& p) {
    require_positive(p.radius, "sphere radius");
    require_positive(p.cells_per_diameter, "sphere cells_per_diameter");

    // The point check comes first, so the cell count below loops over a grid that fits.
    const Index n = p.cells_per_diameter;
    require_index_range("sphere", n, n, n, 0);
    const double half_n = static_cast<double>(n) / 2.0;
    // Cell centres measured from the sphere centre in units of h, so the test is exact and
    // symmetric: the offsets are half-integers and the comparison is done in double.
    auto inside = [half_n](Index i, Index j, Index k) {
        const double x = i + 0.5 - half_n, y = j + 0.5 - half_n, z = k + 0.5 - half_n;
        return x * x + y * y + z * z < half_n * half_n;
    };
    std::int64_t kept = 0;
    for (Index k = 0; k < n; ++k)
        for (Index j = 0; j < n; ++j)
            for (Index i = 0; i < n; ++i) kept += inside(i, j, k) ? 1 : 0;
    require_index_range("sphere", n, n, n, kept);

    CellGrid g;
    g.nx = g.ny = g.nz = n;
    const Real h = Real(2) * p.radius / static_cast<Real>(p.cells_per_diameter);
    g.cell_size = Vec3(h, h, h);
    g.origin = p.center - Vec3(p.radius, p.radius, p.radius);
    g.keep.assign(static_cast<size_t>(g.nx) * static_cast<size_t>(g.ny) *
                      static_cast<size_t>(g.nz),
                  0);

    for (Index k = 0; k < g.nz; ++k)
        for (Index j = 0; j < g.ny; ++j)
            for (Index i = 0; i < g.nx; ++i) g.keep[g.cell(i, j, k)] = inside(i, j, k) ? 1 : 0;

    // The centre cell(s) always pass the test, so the mesh is never empty.
    return build_from_cells(g);
}

}  // namespace hp
