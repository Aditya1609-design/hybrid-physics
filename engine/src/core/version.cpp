#include "hp/version.hpp"

namespace hp {

const char* version() { return HP_VERSION_STRING; }

const char* build_info() {
    // Assembled at compile time from the definitions set in engine/CMakeLists.txt.
    return "hybrid-physics " HP_VERSION_STRING " | " HP_BUILD_TYPE " | " __VERSION__
           " | fp-contract=off"
#if HP_SANITIZED
           " | sanitizers=address,undefined";
#else
           " | sanitizers=none";
#endif
}

}  // namespace hp
