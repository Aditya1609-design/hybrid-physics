#include <doctest/doctest.h>

#include <cstdint>
#include <vector>

#include "hp/core/rng.hpp"

TEST_CASE("rng matches the reference pcg32 output") {
    // Values printed by the official pcg32-demo for pcg32_srandom(42, 54).
    hp::Rng rng(42u, 54u);
    const std::uint32_t expected[] = {0xa15c02b7u, 0x7b47f409u, 0xba1d3330u,
                                      0x83d2f293u, 0xbfa4784bu, 0xcbed606eu};
    for (const std::uint32_t e : expected) {
        CHECK(rng.next_u32() == e);
    }
}

TEST_CASE("rng is reproducible and streams differ") {
    hp::Rng a(7u), b(7u), c(7u, 1u);
    int same_as_c = 0;
    for (int i = 0; i < 1000; ++i) {
        const std::uint32_t x = a.next_u32();
        CHECK(x == b.next_u32());
        if (x == c.next_u32()) ++same_as_c;
    }
    CHECK(same_as_c < 5);
}

TEST_CASE("rng split is order independent and differs from the parent") {
    hp::Rng parent(123u);
    const hp::Rng child_before = parent.split(3u);
    parent.next_u32();  // advancing the parent must not change its children
    hp::Rng child_after = parent.split(3u);
    hp::Rng child_copy = child_before;
    hp::Rng other = parent.split(4u);
    hp::Rng fresh(123u);
    for (int i = 0; i < 100; ++i) {
        const std::uint32_t x = child_after.next_u32();
        CHECK(x == child_copy.next_u32());
        CHECK(x != other.next_u32());
        CHECK(x != fresh.next_u32());
    }
}

TEST_CASE("rng uniform stays in range with the right mean and variance") {
    hp::Rng rng(2024u);
    const int n = 200000;
    double sum = 0.0, sum_sq = 0.0;
    for (int i = 0; i < n; ++i) {
        const float u = rng.uniform();
        REQUIRE(u >= 0.0f);
        REQUIRE(u < 1.0f);
        sum += u;
        sum_sq += static_cast<double>(u) * u;
    }
    const double mean = sum / n;
    CHECK(mean == doctest::Approx(0.5).epsilon(0.01));
    CHECK(sum_sq / n - mean * mean == doctest::Approx(1.0 / 12.0).epsilon(0.01));

    for (int i = 0; i < 1000; ++i) {
        const float x = rng.uniform(-2.0f, 3.0f);
        REQUIRE(x >= -2.0f);
        REQUIRE(x < 3.0f);
    }
}

TEST_CASE("rng normal has mean 0 and variance 1") {
    hp::Rng rng(99u);
    const int n = 200000;
    double sum = 0.0, sum_sq = 0.0;
    for (int i = 0; i < n; ++i) {
        const double x = rng.normal();
        sum += x;
        sum_sq += x * x;
    }
    CHECK(sum / n == doctest::Approx(0.0).epsilon(0.01));
    CHECK(sum_sq / n == doctest::Approx(1.0).epsilon(0.02));
}

TEST_CASE("rng uniform and normal give fixed values for a fixed seed") {
    // Golden values: if these change, every recorded dataset and seeded test changes too.
    hp::Rng rng(1u);
    std::vector<float> got;
    for (int i = 0; i < 3; ++i) got.push_back(rng.uniform());
    for (int i = 0; i < 3; ++i) got.push_back(rng.normal());
    const std::vector<float> expected = {0x1.c4726p-1f,   0x1.112fp-8f,   0x1.a6a13ap-1f,
                                         -0x1.32fae6p-1f, 0x1.99f39cp-1f, 0x1.aa0932p-2f};
    CHECK(got == expected);
}
