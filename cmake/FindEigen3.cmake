find_package(Eigen3 REQUIRED CONFIG)

mark_as_advanced(FORCE Eigen3_DIR)

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "Eigen3 was found, but target Eigen3::Eigen is not available")
endif()
