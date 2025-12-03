# FindTinyXML2.cmake - Find TinyXML2 library
#
# This module defines:
#  TinyXML2_FOUND - True if TinyXML2 is found
#  TinyXML2_INCLUDE_DIRS - Include directories for TinyXML2
#  TinyXML2_LIBRARIES - Libraries to link against TinyXML2
#  TinyXML2::TinyXML2 - Imported target for TinyXML2

# First, try a CMake config package (common when built from source)
set(_tinyxml2_config_found FALSE)
find_package(tinyxml2 QUIET CONFIG)
if(tinyxml2_FOUND)
    # Try common target names
    if(TARGET tinyxml2::tinyxml2)
        add_library(TinyXML2::TinyXML2 INTERFACE IMPORTED)
        target_link_libraries(TinyXML2::TinyXML2 INTERFACE tinyxml2::tinyxml2)
        set(_tinyxml2_config_found TRUE)
    elseif(TARGET tinyxml2)
        add_library(TinyXML2::TinyXML2 INTERFACE IMPORTED)
        target_link_libraries(TinyXML2::TinyXML2 INTERFACE tinyxml2)
        set(_tinyxml2_config_found TRUE)
    endif()
endif()

if(NOT _tinyxml2_config_found)
    # Fallback to pkg-config and manual discovery (typical on Ubuntu/Debian)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(PC_TinyXML2 QUIET tinyxml2)
    endif()

    find_path(TinyXML2_INCLUDE_DIR
        NAMES tinyxml2.h
        PATHS ${PC_TinyXML2_INCLUDE_DIRS}
        PATH_SUFFIXES include
    )

    find_library(TinyXML2_LIBRARY
        NAMES tinyxml2
        PATHS ${PC_TinyXML2_LIBRARY_DIRS}
        PATH_SUFFIXES lib lib64
    )

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(TinyXML2
        REQUIRED_VARS TinyXML2_LIBRARY TinyXML2_INCLUDE_DIR
        VERSION_VAR PC_TinyXML2_VERSION
    )

    if(TinyXML2_FOUND)
        set(TinyXML2_LIBRARIES ${TinyXML2_LIBRARY})
        set(TinyXML2_INCLUDE_DIRS ${TinyXML2_INCLUDE_DIR})

        if(NOT TARGET TinyXML2::TinyXML2)
            add_library(TinyXML2::TinyXML2 UNKNOWN IMPORTED)
            set_target_properties(TinyXML2::TinyXML2 PROPERTIES
                IMPORTED_LOCATION "${TinyXML2_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${TinyXML2_INCLUDE_DIR}"
            )
        endif()
    endif()

    mark_as_advanced(TinyXML2_INCLUDE_DIR TinyXML2_LIBRARY)
else()
    # Config-based discovery succeeded
    set(TinyXML2_FOUND TRUE)
    # TinyXML2_INCLUDE_DIRS and TinyXML2_LIBRARIES are not strictly needed when using targets,
    # but set minimal placeholders for compatibility.
    set(TinyXML2_INCLUDE_DIRS "")
    set(TinyXML2_LIBRARIES "")
endif()
