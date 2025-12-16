// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <stdexcept>
#include <string>

#include <franka/logging/robot_state_log.hpp>

/**
 * @file exception.h
 * Contains exception definitions.
 */

namespace franka {

/**
 * Base class for all exceptions used by `libfranka`.
 */
struct Exception : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

/**
 * ModelException is thrown if an error occurs when loading the model library.
 */
struct ModelException : public Exception {
  using Exception::Exception;
};

/**
 * NetworkException is thrown if a connection to the robot cannot be established, or when a timeout
 * occurs.
 */
struct NetworkException : public Exception {
  using Exception::Exception;
};

/**
 * ProtocolException is thrown if the robot returns an incorrect message.
 */
struct ProtocolException : public Exception {
  using Exception::Exception;
};

/**
 * IncompatibleVersionException is thrown if the robot does not support this version of libfranka.
 */
struct IncompatibleVersionException : public Exception {
  /**
   * Creates the exception using the two different protocol versions.
   * @param[in] server_version Protocol version on the Control side.
   * @param[in] library_version Protocol version of libfranka.
   */
  IncompatibleVersionException(uint16_t server_version, uint16_t library_version) noexcept;

  /**
   * Control's protocol version.
   */
  const uint16_t server_version;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  /**
   * libfranka protocol version.
   */
  const uint16_t library_version;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};

/**
 * ControlException is thrown if an error occurs during motion generation or torque control.
 * The exception holds a vector with the last received robot states. The number of recorded
 * states can be configured in the Robot constructor.
 *
 */
struct ControlException : public Exception {
  /**
   * Creates the exception with an explanatory string and a Log object.
   *
   * @param[in] what Explanatory string.
   * @param[in] log Vector of last received states and commands.
   */
  explicit ControlException(const std::string& what, std::vector<franka::Record> log = {}) noexcept;

  /**
   * Vector of states and commands logged just before the exception occurred.
   */
  const std::vector<franka::Record>
      log;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
};

/**
 * CommandException is thrown if an error occurs during command execution.
 */
struct CommandException : public Exception {
  using Exception::Exception;
};

/**
 * RealtimeException is thrown if realtime priority cannot be set.
 */
struct RealtimeException : public Exception {
  using Exception::Exception;
};

/**
 * InvalidOperationException is thrown if an operation cannot be performed.
 */
struct InvalidOperationException : public Exception {
  using Exception::Exception;
};

}  // namespace franka
