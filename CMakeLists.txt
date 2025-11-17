cmake_minimum_required(VERSION 3.11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(libfranka_VERSION 0.18.0)

project(libfranka
  VERSION ${libfranka_VERSION}
  LANGUAGES CXX
)

option(BUILD_DOCUMENTATION "Build documentation" OFF)
option(SKIP_CXX_BUILD "Skips the c++ build" OFF)
option(GENERATE_PYLIBFRANKA "Generate the python bindings for libfranka" OFF)
if(BUILD_DOCUMENTATION)
  add_subdirectory(doc)

  if(SKIP_CXX_BUILD)
    return()
  endif()
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set a default build type if none was specified
set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
    STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(MSVC)
  add_compile_options(/W0)
else()
  add_compile_options(-Wall -Wextra)
endif()

set(THIRDPARTY_SOURCES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" CACHE PATH
  "Directory for third-party sources")

## Dependencies
find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(pinocchio REQUIRED)
find_package(tinyxml2 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

## Build options
option(STRICT "Treat warnings as errors" OFF)
if(STRICT)
  if(MSVC)
    add_compile_options(/WX)
  else()
    add_compile_options(-Werror)
  endif()
endif()

option(BUILD_COVERAGE "Build with code coverage" OFF)
if(BUILD_COVERAGE)
  add_compile_options(--coverage)
  # Unfortunately, there is no add_link_options...
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

## Load dependencies
include(FetchFMT)

## Submodules
add_subdirectory(common)

## Library
add_library(franka SHARED
  src/active_control.cpp
  src/active_motion_generator.cpp
  src/active_torque_control.cpp
  src/control_loop.cpp
  src/control_tools.cpp
  src/control_types.cpp
  src/duration.cpp
  src/errors.cpp
  src/exception.cpp
  src/gripper.cpp
  src/gripper_state.cpp
  src/joint_velocity_limits.cpp
  src/load_calculations.cpp
  src/lowpass_filter.cpp
  src/model.cpp
  src/network.cpp
  src/rate_limiting.cpp
  src/robot.cpp
  src/robot_impl.cpp
  src/robot_state.cpp
  src/vacuum_gripper.cpp
  src/vacuum_gripper_state.cpp
  src/robot_model.cpp

  src/async_control/async_position_control_handler.cpp

  src/logging/cout_logging_sink.cpp
  src/logging/logger.cpp
  src/logging/robot_state_log.cpp
  src/logging/robot_state_logger.cpp
)
add_library(Franka::Franka ALIAS franka)

set_target_properties(franka PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
  VERSION ${libfranka_VERSION}
  SOVERSION ${libfranka_VERSION_MAJOR}.${libfranka_VERSION_MINOR} # Use minor version while we're at 0.x
  EXPORT_NAME Franka
)
target_compile_features(franka INTERFACE
  cxx_attribute_deprecated
  cxx_constexpr
  cxx_defaulted_functions
  cxx_deleted_functions
  cxx_generalized_initializers
  cxx_noexcept
  cxx_uniform_initialization
)
if(MSVC)
  target_compile_definitions(franka PUBLIC
    _USE_MATH_DEFINES # for M_PI in cmath
    NOMINMAX # avoid conflict with std::min
  )
endif()

target_include_directories(franka PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(franka PRIVATE
  tinyxml2
  Poco::Foundation
  Poco::Net
  Eigen3::Eigen3
  Threads::Threads
  pinocchio::pinocchio
)

target_link_libraries(franka PUBLIC
  libfranka-common
  fmt::fmt
)

## Compile pylibfranka
if (GENERATE_PYLIBFRANKA)
  add_subdirectory(pylibfranka)
endif()

## Installation
include(GNUInstallDirs)
set(INSTALL_CMAKE_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/Franka)

install(TARGETS franka libfranka-common
  EXPORT FrankaTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ common/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  USE_SOURCE_PERMISSIONS
)

export(EXPORT FrankaTargets
  NAMESPACE Franka::
  FILE ${CMAKE_CURRENT_BINARY_DIR}/FrankaTargets.cmake
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/FrankaConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(cmake/FrankaConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfigVersion.cmake
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)
install(EXPORT FrankaTargets
  NAMESPACE Franka::
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

## Subprojects

# Ignore find_package(Franka) in subprojects.
set(FRANKA_IS_FOUND TRUE)

option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_EXAMPLES "Build example code" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

## Packaging
include(SetVersionFromGit)
set_version_from_git(PACKAGE_VERSION PACKAGE_TAG)
execute_process(COMMAND dpkg --print-architecture OUTPUT_VARIABLE PACKAGE_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CPACK_PACKAGE_VENDOR "Franka Robotics GmbH")
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_PACKAGE_VERSION ${libfranka_VERSION})
set(CPACK_PACKAGE_FILE_NAME ${PROJECT_NAME}_${PACKAGE_TAG}_${PACKAGE_ARCH})
set(CPACK_PACKAGE_VERSION ${PACKAGE_TAG})
set(CPACK_PACKAGE_ARCHITECTURE ${PACKAGE_ARCH})
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})

# Debian versions require a dash
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Franka Robotics GmbH")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libpoco-dev, libfmt-dev, libeigen3-dev")
set(CPACK_DEBIAN_PACKAGE_CONFLICTS "ros-kinetic-libfranka, ros-melodic-libfranka, ros-noetic-libfranka, ros-foxy-libfranka, ros-humble-libfranka, ros-jazzy-libfranka")
include(CPack)

## Tools
file(GLOB_RECURSE SOURCES
  src/*.cpp
  examples/*.cpp
)
file(GLOB_RECURSE HEADERS
  include/*.h
  src/*.h
  examples/*.h
)
file(GLOB_RECURSE TEST_FILES
  test/*.h
  test/*.cpp
)
file(GLOB_RECURSE TIDY_IGNORED_FILES
)

find_program(CLANG_FORMAT_PROG clang-format DOC "'clang-format' executable")
if(CLANG_FORMAT_PROG)
  add_custom_target(format
    COMMAND ${CLANG_FORMAT_PROG} -i ${SOURCES} ${HEADERS} ${TEST_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Formatting source code with clang-format"
    VERBATIM
  )
  add_custom_target(check-format
    COMMAND scripts/format-check.sh ${CLANG_FORMAT_PROG} -output-replacements-xml ${SOURCES} ${HEADERS} ${TEST_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Checking code formatting with clang-format"
    VERBATIM
  )
else(CLANG_FORMAT_PROG)
  MESSAGE(WARNING "Could not find clang-format")
endif()

find_program(CLANG_TIDY_PROG clang-tidy DOC "'clang-tidy' executable")
if(CLANG_TIDY_PROG)
  # Build JSON string for clang-tidy's line filter
  set(TIDY_FILES ${SOURCES} ${HEADERS})
  string(REPLACE ";" "\"},{\"name\":\"" TIDY_LINE_FILTER "${TIDY_FILES}")
  set(TIDY_LINE_FILTER "[{\"name\":\"${TIDY_LINE_FILTER}\"}]")

  add_custom_target(tidy
    COMMAND ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR}
                               -line-filter=${TIDY_LINE_FILTER} ${SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running clang-tidy"
    VERBATIM
  )
  add_custom_target(check-tidy
    COMMAND scripts/fail-on-output.sh ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR}
                                      -line-filter=${TIDY_LINE_FILTER} ${SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running clang-tidy"
    VERBATIM
  )
else(CLANG_TIDY_PROG)
  MESSAGE(WARNING "Could not find clang-tidy")
endif()
