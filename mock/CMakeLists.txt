# This is pretty clunky, as it is built as a subdirectory but also from colcon
# When imported in the simulation repo. I need to learn better CMake to clean this up

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(aescape_mock_server
  VERSION 0.0.1
  LANGUAGES CXX
  DESCRIPTION "Mock Franka arm server code"
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(fmt REQUIRED)
find_package(magic_enum REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Tracy REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net)

if(NOT CMAKE_PROJECT_NAME STREQUAL "libfranka")
  find_package(Franka REQUIRED)
endif()

set(MOCK_DEPENDENCIES
  Poco::Net
  Eigen3::Eigen
  Threads::Threads
  franka
  libfranka-common
  pinocchio::pinocchio
  fmt::fmt
  magic_enum::magic_enum
  Tracy::TracyClient
)

set(MOCK_BIN_DIR AESCAPE_MOCK_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
set(MOCK_SRC_DIR AESCAPE_MOCK_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
set(MOCK_COMPILE_DEFINITIONS ${MOCK_BIN_DIR} ${MOCK_SRC_DIR})

set(MOCK_PUBLIC_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include>
)

set(MOCK_PRIVATE_INCLUDES
  ${MOCK_PUBLIC_INCLUDES}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

## FCI Model
add_library(aescape_fcimodels SHARED
  src/libfcimodels_stub.cpp
)

target_include_directories(aescape_fcimodels PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src ${MOCK_PRIVATE_INCLUDES})

list(APPEND MOCK_DEPENDENCIES aescape_fcimodels)


## Aescape Backend Library
add_library(aescape_backend SHARED
  src/backend/forward_dynamics.cpp
  src/backend/forward_dynamics_backend.cpp
  src/backend/proportional_controller_backend.cpp
)

target_link_libraries(aescape_backend PUBLIC ${MOCK_DEPENDENCIES})
target_include_directories(aescape_backend PRIVATE ${MOCK_PRIVATE_INCLUDES})

## Aescape Mock Server

add_library(${PROJECT_NAME} SHARED
  src/mock_server.cpp
  src/aescape_mock_server.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${MOCK_DEPENDENCIES} aescape_backend)
target_include_directories(${PROJECT_NAME} PUBLIC ${MOCK_PUBLIC_INCLUDES})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${MOCK_SRC_DIR} ${MOCK_BIN_DIR})


install(TARGETS ${PROJECT_NAME} aescape_backend aescape_fcimodels
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# for build tree / in workspace
export(EXPORT ${PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::)

set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake)

# exported targets
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${ConfigPackageLocation})

  write_basic_package_version_file(${PROJECT_NAME}Config-version.cmake
    VERSION ${CMAKE_PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
  )

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_NAME}Config.cmake @ONLY
  )

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config-version.cmake
    DESTINATION ${ConfigPackageLocation}
  )


add_executable(aescape_mock_server_example
  src/example/mock_server_example.cpp
)
target_include_directories(aescape_mock_server_example PRIVATE ${MOCK_PRIVATE_INCLUDES})
target_link_libraries(aescape_mock_server_example PRIVATE ${MOCK_DEPENDENCIES} aescape_backend aescape_mock_server)
target_compile_definitions(aescape_mock_server_example PRIVATE ${MOCK_COMPILE_DEFINITIONS})

add_executable(aescape_mock_client_example
  src/example/mock_client_example.cpp
)
target_include_directories(aescape_mock_client_example PRIVATE ${MOCK_PRIVATE_INCLUDES})
target_link_libraries(aescape_mock_client_example PRIVATE ${MOCK_DEPENDENCIES})
target_compile_definitions(aescape_mock_client_example PRIVATE ${MOCK_COMPILE_DEFINITIONS})

# Google Test
if(BUILD_TESTS)

if(CMAKE_PROJECT_NAME STREQUAL "libfranka")
  target_compile_options(gtest PUBLIC -Wno-deprecated-copy -fPIC)
else()
  find_package(GTest REQUIRED)
endif()

  include(GoogleTest)

  add_executable(forward_dynamics_tests
    src/backend/forward_dynamics_tests.cpp
  )

  target_link_libraries(forward_dynamics_tests PRIVATE ${MOCK_DEPENDENCIES} aescape_backend gtest)
  target_include_directories(forward_dynamics_tests PRIVATE ${MOCK_PRIVATE_INCLUDES})
  target_compile_definitions(forward_dynamics_tests PRIVATE ${MOCK_SRC_DIR})
  add_test(forward_dynamics_tests forward_dynamics_tests --gtest_output=xml:${TEST_OUTPUT_DIR}/default.xml)

endif()