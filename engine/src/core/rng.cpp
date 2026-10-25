#include "hp/core/rng.hpp"

#include <cmath>

namespace hp {

namespace {

// SplitMix64 finalizer: spreads nearby inputs over the whole 64-bit range.
std::uint64_t mix64(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

}  // namespace

// Seeding follows the reference pcg32_srandom_r, so outputs match the published demo values.
Rng::Rng(std::uint64_t seed, std::uint64_t stream)
    : seed_(seed), stream_(stream), inc_((stream << 1u) | 1u) {
    next_u32();
    state_ += seed;
    next_u32();
}

// XSH-RR output on a 64-bit LCG state.
std::uint32_t Rng::next_u32() {
    const std::uint64_t old = state_;
    state_ = old * 6364136223846793005ULL + inc_;
    const auto xorshifted = static_cast<std::uint32_t>(((old >> 18u) ^ old) >> 27u);
    const auto rot = static_cast<std::uint32_t>(old >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((32u - rot) & 31u));
}

float Rng::uniform() { return static_cast<float>(next_u32() >> 8u) * 0x1p-24f; }

float Rng::uniform(float lo, float hi) { return lo + (hi - lo) * uniform(); }

// Polar method in double: needs only log and sqrt (no sin/cos), and the spare value is
// dropped so the generator state stays just the PCG state.
float Rng::normal() {
    double u = 0.0, v = 0.0, s = 0.0;
    do {
        u = static_cast<double>(next_u32()) * 0x1p-31 - 1.0;
        v = static_cast<double>(next_u32()) * 0x1p-31 - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);
    return static_cast<float>(u * std::sqrt(-2.0 * std::log(s) / s));
}

Rng Rng::split(std::uint64_t stream) const {
    return Rng(mix64(seed_ ^ mix64(stream_)), mix64(stream));
}

}  // namespace hp
