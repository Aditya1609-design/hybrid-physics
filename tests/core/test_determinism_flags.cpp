#include <doctest/doctest.h>

#include <cmath>

// These inputs give a different answer when a*b+c is fused into one FMA instruction:
// the unfused product rounds to exactly 1, so a*b+c is 0, while the FMA keeps the tiny
// remainder. If -ffp-contract=off goes missing, the compiler fuses and these tests fail.
// volatile stops the compiler from folding the arithmetic at compile time.

TEST_CASE("float a*b+c is not contracted into an FMA") {
    volatile float a = 1.0f + 0x1p-13f;
    volatile float b = 1.0f - 0x1p-13f;
    volatile float c = -1.0f;
    const float x = a, y = b, z = c;

    const float result = x * y + z;
    CHECK(std::fma(x, y, z) != 0.0f);  // sanity: the fused result really differs
    CHECK(result == 0.0f);
}

TEST_CASE("double a*b+c is not contracted into an FMA") {
    volatile double a = 1.0 + 0x1p-27;
    volatile double b = 1.0 - 0x1p-27;
    volatile double c = -1.0;
    const double x = a, y = b, z = c;

    const double result = x * y + z;
    CHECK(std::fma(x, y, z) != 0.0);
    CHECK(result == 0.0);
}
