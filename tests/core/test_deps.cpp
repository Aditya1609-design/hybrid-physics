#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

// Smoke tests: the fetched dependencies compile, link and behave as expected.

TEST_CASE("Eigen float math works") {
    const Eigen::Vector3f x(1.0f, 0.0f, 0.0f);
    const Eigen::Vector3f y(0.0f, 1.0f, 0.0f);
    CHECK(x.cross(y).isApprox(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));

    Eigen::Matrix3f m;
    m << 2, 0, 0, 0, 4, 0, 0, 0, 8;
    CHECK((m * m.inverse()).isApprox(Eigen::Matrix3f::Identity()));
}

TEST_CASE("nlohmann/json parses a scenario-like document") {
    const auto doc = nlohmann::json::parse(R"({"seed": 42, "material": {"youngs": 1.0e5}})");
    CHECK(doc.at("seed").get<int>() == 42);
    CHECK(doc.at("material").at("youngs").get<double>() == doctest::Approx(1.0e5));
}
