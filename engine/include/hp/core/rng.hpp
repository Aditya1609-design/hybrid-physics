#pragma once

#include <cstdint>

namespace hp {

// PCG32 random generator (O'Neill, pcg-random.org) with its own distributions.
// std:: distributions are avoided because their output differs between standard libraries.
class Rng {
public:
    // Same (seed, stream) always gives the same sequence. Different streams are independent.
    explicit Rng(std::uint64_t seed, std::uint64_t stream = 0);

    std::uint32_t next_u32();

    // Uniform in [0, 1), 24 random bits.
    float uniform();

    // Uniform in [lo, hi).
    float uniform(float lo, float hi);

    // Standard normal (mean 0, variance 1), Marsaglia polar method.
    float normal();

    // Independent generator for a sub-task, derived from this one's seed and stream.
    // Does not advance this generator, so the result does not depend on call order.
    Rng split(std::uint64_t stream) const;

private:
    std::uint64_t seed_;
    std::uint64_t stream_;
    std::uint64_t state_ = 0;
    std::uint64_t inc_;
};

}  // namespace hp
