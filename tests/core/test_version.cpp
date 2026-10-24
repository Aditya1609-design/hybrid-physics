#include <doctest/doctest.h>

#include <string>

#include "hp/version.hpp"

TEST_CASE("engine reports its version and build info") {
    const std::string v = hp::version();
    const std::string info = hp::build_info();
    CHECK(v == "0.1.0");
    CHECK(info.find(v) != std::string::npos);
    CHECK(info.find("fp-contract=off") != std::string::npos);
}
