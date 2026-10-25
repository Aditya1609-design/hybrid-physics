#include <doctest/doctest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "hp/io/obj.hpp"
#include "hp/mesh/generators.hpp"

TEST_CASE("surface obj round-trips positions and faces") {
    hp::BoxParams p;
    p.cells_x = 3;
    p.cells_y = 2;
    p.cells_z = 2;
    p.center = hp::Vec3(0.1f, 0.2f, 0.3f);
    const hp::TetMesh mesh = hp::make_box(p);
    const hp::Topology topo = hp::build_topology(mesh);

    const auto path = (std::filesystem::temp_directory_path() / "hp_test_surface.obj").string();
    hp::write_surface_obj(path, mesh.rest_positions, topo.surface_triangles);

    std::vector<hp::Vec3> verts;
    std::vector<std::vector<long>> faces;
    std::ifstream in(path);
    REQUIRE(in);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag == "v") {
            hp::Vec3 x;
            ls >> x.x() >> x.y() >> x.z();
            verts.push_back(x);
        } else if (tag == "f") {
            std::vector<long> f(3);
            ls >> f[0] >> f[1] >> f[2];
            faces.push_back(f);
        }
    }
    in.close();
    std::remove(path.c_str());

    // 4*3*3 grid points, of which 2 are interior and must not be written.
    CHECK(verts.size() == 34);
    REQUIRE(faces.size() == topo.surface_triangles.size());

    // Faces point at the same positions as the original triangles, bit for bit.
    for (size_t i = 0; i < faces.size(); ++i) {
        for (size_t a = 0; a < 3; ++a) {
            REQUIRE(faces[i][a] >= 1);
            REQUIRE(faces[i][a] <= static_cast<long>(verts.size()));
            const hp::Vec3& got = verts[static_cast<size_t>(faces[i][a] - 1)];
            const hp::Vec3& want =
                mesh.rest_positions[static_cast<size_t>(topo.surface_triangles[i][a])];
            CHECK(got == want);
        }
    }
}
