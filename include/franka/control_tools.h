// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cmath>
#include <string>

/**
 * @file control_tools.h
 * Contains helper functions for writing control loops.
 */

namespace franka {

/**
 * Determines whether the given elbow configuration is valid or not.
 *
 * @param[in] elbow Elbow configuration.
 *
 * @return True if valid, otherwise false.
 */
inline bool isValidElbow(const std::array<double, 2>& elbow) noexcept {
  return elbow[1] == -1.0 || elbow[1] == 1.0;
}

/**
 * Determines whether the given array represents a valid homogeneous transformation matrix.
 *
 * @param[in] transform 4x4 matrix in column-major format.
 *
 * @return True if the array represents a homogeneous transformation matrix, otherwise false.
 */
inline bool isHomogeneousTransformation(const std::array<double, 16>& transform) noexcept {
  constexpr double kOrthonormalThreshold = 1e-5;

  if (transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 || transform[15] != 1.0) {
    return false;
  }
  for (size_t j = 0; j < 3; ++j) {  // i..column
    if (std::abs(std::sqrt(std::pow(transform[j * 4 + 0], 2) + std::pow(transform[j * 4 + 1], 2) +
                           std::pow(transform[j * 4 + 2], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {  // j..row
    if (std::abs(std::sqrt(std::pow(transform[0 * 4 + i], 2) + std::pow(transform[1 * 4 + i], 2) +
                           std::pow(transform[2 * 4 + i], 2)) -
                 1.0) > kOrthonormalThreshold) {
      return false;
    }
  }
  return true;
}

/**
 * Determines whether the current OS kernel is a realtime kernel.
 *
 * On Linux, this checks for the existence of `/sys/kernel/realtime`.
 * On Windows, this always returns true.
 *
 * @return True if running a realtime kernel, false otherwise.
 */
bool hasRealtimeKernel();

/**
 * Sets the current thread to the highest possible scheduler priority.
 *
 * @param[out] error_message Contains an error message if the scheduler priority
 * cannot be set successfully.
 *
 * @return True if successful, false otherwise.
 */
bool setCurrentThreadToHighestSchedulerPriority(std::string* error_message);

}  // namespace franka
