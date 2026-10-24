# Keeps the engine headless: configuring fails if the engine library links anything
# other than math, JSON and our own build flags.

set(HP_ENGINE_ALLOWED_LINKS
    hp_eigen hp::eigen
    nlohmann_json nlohmann_json::nlohmann_json
    hp_build_flags hp::build_flags)

function(hp_check_engine_dependencies target)
    get_target_property(_links ${target} LINK_LIBRARIES)
    get_target_property(_iface_links ${target} INTERFACE_LINK_LIBRARIES)
    set(_all "")
    foreach(_lib IN LISTS _links _iface_links)
        # Private links of a static library show up as $<LINK_ONLY:name>; check the name.
        string(REGEX REPLACE "^\\$<LINK_ONLY:(.*)>$" "\\1" _lib "${_lib}")
        if(_lib AND NOT _lib MATCHES "-NOTFOUND$")
            list(APPEND _all "${_lib}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES _all)

    set(_bad "")
    foreach(_lib IN LISTS _all)
        if(NOT _lib IN_LIST HP_ENGINE_ALLOWED_LINKS)
            list(APPEND _bad "${_lib}")
        endif()
    endforeach()

    if(_bad)
        message(FATAL_ERROR
            "${target} must stay free of graphics and ML dependencies, but it links: ${_bad}\n"
            "Allowed: ${HP_ENGINE_ALLOWED_LINKS}\n"
            "Link these from an app or bridge target instead.")
    endif()
endfunction()
