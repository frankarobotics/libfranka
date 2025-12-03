set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

## Google Test
set(GTEST_LIBRARIES gtest gmock gmock_main)

include(SetupGoogleTest)

foreach(lib ${GTEST_LIBRARIES})
  target_compile_options(${lib} PUBLIC -Wno-deprecated-copy)
endforeach()

set(TEST_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test_results)
set(TEST_DEPENDENCIES
  ${GTEST_LIBRARIES}
  TinyXML2::TinyXML2
  Poco::Foundation
  Poco::Net
  Eigen3::Eigen3
  Threads::Threads
  pinocchio::pinocchio
  franka
  libfranka-common
)
set(TEST_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../src)

## Test runner
add_executable(run_all_tests
  active_motion_generator_tests.cpp
  active_torque_control_tests.cpp
  calculations_tests.cpp
  control_loop_tests.cpp
  control_types_tests.cpp
  duration_tests.cpp
  errors_tests.cpp
  gripper_command_tests.cpp
  gripper_tests.cpp
  helpers.cpp
  lowpass_filter_tests.cpp
  mock_server.cpp
  model_tests.cpp
  rate_limiting_tests.cpp
  robot_command_tests.cpp
  robot_impl_tests.cpp
  robot_model_tests.cpp
  robot_model_fk_tests.cpp
  robot_jacobian_tests.cpp
  robot_state_tests.cpp
  robot_tests.cpp
  vacuum_gripper_tests.cpp
  vacuum_gripper_command_tests.cpp
  logging/logging_tests.cpp
  logging/robot_state_logger_tests.cpp
  joint_velocity_limits_tests.cpp

  async_control/async_position_control_tests.cpp
)

set(TEST_COMPILE_DEFINITIONS FRANKA_TEST_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(run_all_tests PRIVATE ${TEST_COMPILE_DEFINITIONS})
if(WIN32)
    target_compile_options(run_all_tests PRIVATE /bigobj)
endif(WIN32)

target_include_directories(run_all_tests PRIVATE ${TEST_INCLUDE_DIRECTORIES})
target_link_libraries(run_all_tests PUBLIC ${TEST_DEPENDENCIES})

add_test(Default run_all_tests --gtest_output=xml:${TEST_OUTPUT_DIR}/default.xml)

if(BUILD_COVERAGE)
  find_program(LCOV_PROG lcov)
  if(NOT LCOV_PROG)
    message(FATAL_ERROR "lcov not found.")
  endif()

  find_program(GENHTML_PROG genhtml)
  if(NOT GENHTML_PROG)
    message(FATAL_ERROR "genhtml not found.")
  endif()

  set(COVERAGE_REPORT_DIR "coverage")
  set(COVERAGE_FILE "all_tests.coverage")

  add_custom_target(coverage
    COMMAND ${LCOV_PROG} --directory . --zerocounters
    COMMAND ${CMAKE_CTEST_COMMAND} -R Default .
    COMMAND ${LCOV_PROG} --directory . --capture --output-file ${COVERAGE_FILE}
    COMMAND ${LCOV_PROG} --remove ${COVERAGE_FILE} 'test/*' '/usr/*' --output-file ${COVERAGE_FILE}
    COMMAND ${GENHTML_PROG} -o ${COVERAGE_REPORT_DIR} ${COVERAGE_FILE}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS run_all_tests
    COMMENT "Generating code coverage"
  )
  return()
endif()

## Sanitizers
include(CheckCXXCompilerFlag)
get_target_property(TESTS run_all_tests SOURCES)

set(ASan_FLAGS -g -O1 -fsanitize=address)
set(UBSan_FLAGS -g -O1 -fsanitize=undefined -fno-sanitize=alignment -fno-sanitize-recover=undefined)
set(TSan_FLAGS -g -O1 -fsanitize=thread)
set(SANITIZERS ASan UBSan TSan)

foreach(sanitizer ${SANITIZERS})
  set(sanitizer_flags "${${sanitizer}_FLAGS}")

  set(CMAKE_REQUIRED_LIBRARIES "${sanitizer_flags}")
  check_cxx_compiler_flag("${sanitizer_flags}" HAVE_${sanitizer})
  unset(CMAKE_REQUIRED_LIBRARIES)

  if(HAVE_${sanitizer})
    string(TOLOWER "${sanitizer}" sanitizer_lc)
    set(target_name "run_all_tests_${sanitizer_lc}")

    add_executable(${target_name} ${TESTS})
    target_compile_definitions(${target_name} PRIVATE ${TEST_COMPILE_DEFINITIONS})
    target_include_directories(${target_name} PRIVATE ${TEST_INCLUDE_DIRECTORIES})
    target_link_libraries(${target_name} PUBLIC ${TEST_DEPENDENCIES} ${sanitizer_flags})
    target_compile_options(${target_name} PRIVATE ${sanitizer_flags})
    add_test(NAME ${sanitizer}
      COMMAND ${target_name} --gtest_output=xml:${TEST_OUTPUT_DIR}/${sanitizer_lc}.xml
    )
  endif()
endforeach()

## Valgrind
find_program(VALGRIND_PROG valgrind)
if(VALGRIND_PROG)
  message(STATUS "Found Valgrind: ${VALGRIND_PROG}")
  add_test(NAME Valgrind
    COMMAND ${VALGRIND_PROG} --error-exitcode=1 --track-origins=yes --
            ./run_all_tests --gtest_output=xml:${TEST_OUTPUT_DIR}/valgrind.xml
  )
endif()
