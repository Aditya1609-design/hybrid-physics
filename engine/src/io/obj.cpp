#include "hp/io/obj.hpp"

#include <fstream>
#include <locale>
#include <stdexcept>

namespace hp {

void write_surface_obj(const std::string& path, const std::vector<Vec3>& positions,
                       const std::vector<Triangle>& triangles) {
    // Map used vertices to 1-based OBJ indices, keeping their original order.
    std::vector<Index> obj_index(positions.size(), 0);
    for (const Triangle& t : triangles) {
        for (const Index v : t) {
            if (v < 0 || static_cast<size_t>(v) >= positions.size()) {
                throw std::runtime_error("triangle vertex index out of range");
            }
            obj_index[static_cast<size_t>(v)] = 1;
        }
    }

    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot open " + path + " for writing");
    // Classic locale and 9 significant digits: a '.' decimal point and exact float round trip.
    out.imbue(std::locale::classic());
    out.precision(9);

    Index next = 1;
    for (size_t v = 0; v < positions.size(); ++v) {
        if (!obj_index[v]) continue;
        obj_index[v] = next++;
        const Vec3& x = positions[v];
        out << "v " << x.x() << ' ' << x.y() << ' ' << x.z() << '\n';
    }
    for (const Triangle& t : triangles) {
        out << "f " << obj_index[static_cast<size_t>(t[0])] << ' '
            << obj_index[static_cast<size_t>(t[1])] << ' '
            << obj_index[static_cast<size_t>(t[2])] << '\n';
    }
    if (!out) throw std::runtime_error("failed while writing " + path);
}

}  // namespace hp
