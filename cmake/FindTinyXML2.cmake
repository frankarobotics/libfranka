# FindTinyXML2.cmake - Find TinyXML2 library
#
# This module defines:
#  TinyXML2_FOUND - True if TinyXML2 is found
#  TinyXML2_INCLUDE_DIRS - Include directories for TinyXML2
#  TinyXML2_LIBRARIES - Libraries to link against TinyXML2
#  TinyXML2::TinyXML2 - Imported target for TinyXML2

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