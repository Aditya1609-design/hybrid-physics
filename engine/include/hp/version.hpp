#pragma once

namespace hp {

// Engine version, e.g. "0.1.0".
const char* version();

// One-line description of how the engine was built (version, build type, compiler,
// floating-point mode, sanitizers). Benchmarks and recordings store it with their results.
const char* build_info();

}  // namespace hp
