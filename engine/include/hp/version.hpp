#pragma once

namespace hp {

// Engine version, e.g. "0.1.0".
const char* version();

// One-line description of how the engine was built (version, build type, compiler,
// floating-point mode, sanitizers). Benchmarks and recordings store it with their results.
const char* build_info();

// Git commit the engine was built from (40 hex digits), or "unknown" outside a git checkout.
// Refreshed on every build.
const char* git_commit();

// True if tracked files differed from that commit at build time. Untracked files are ignored.
bool git_dirty();

}  // namespace hp
