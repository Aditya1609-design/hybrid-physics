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

TEST_CASE("engine reports the git commit it was built from") {
    const std::string commit = hp::git_commit();
    // Either a full SHA-1 or "unknown" when built outside a git checkout.
    if (commit != "unknown") {
        CHECK(commit.size() == 40);
        CHECK(commit.find_first_not_of("0123456789abcdef") == std::string::npos);
    }
    (void)hp::git_dirty();  // links and returns; its value depends on the working tree
}
