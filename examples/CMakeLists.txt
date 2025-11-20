cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Load dependencies
include(FetchFMT)

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)
target_include_directories(examples_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set(EXAMPLES
  async_position_control
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_pose_motion_external_control_loop
  generate_cartesian_velocity_motion
  generate_cartesian_velocity_motion_external_control_loop
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_position_motion_external_control_loop
  generate_joint_velocity_motion
  generate_joint_velocity_motion_external_control_loop
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  motion_with_control_external_control_loop
  print_joint_poses
  vacuum_object
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_include_directories(${example} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)
target_link_libraries(motion_with_control_external_control_loop Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Further examples
add_subdirectory(utility_examples)
