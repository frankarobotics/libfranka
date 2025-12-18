// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <franka/vacuum_gripper_state.h>

/**
 * @file vacuum_gripper.h
 * Contains the franka::VacuumGripper type.
 */

namespace franka {

class Network;

/**
 * Maintains a network connection to the vacuum gripper, provides the current vacuum gripper state,
 * and allows the execution of commands.
 *
 * @note
 * The members of this class are threadsafe.
 */
class VacuumGripper {
 public:
  /**
   * Version of the vacuum gripper server.
   */
  using ServerVersion = uint16_t;

  /**
   * Vacuum production setup profile.
   */
  enum class ProductionSetupProfile { kP0, kP1, kP2, kP3 };  // NOLINT(performance-enum-size)

  /**
   * Establishes a connection with a vacuum gripper connected to a robot.
   *
   * @param[in] franka_address IP/hostname of the robot the vacuum gripper is connected to.
   *
   * @throw NetworkException if the connection is unsuccessful.
   * @throw IncompatibleVersionException if this version of `libfranka` is not supported.
   */
  explicit VacuumGripper(const std::string& franka_address);

  /**
   * Move-constructs a new VacuumGripper instance.
   *
   * @param[in] vacuum_gripper Other VacuumGripper instance.
   */
  VacuumGripper(VacuumGripper&& vacuum_gripper) noexcept;

  /**
   * Move-assigns this VacuumGripper from another VacuumGripper instance.
   *
   * @param[in] vacuum_gripper Other VacuumGripper instance.
   *
   * @return Model instance.
   */
  VacuumGripper& operator=(VacuumGripper&& vacuum_gripper) noexcept;

  /**
   * Closes the connection.
   */
  ~VacuumGripper() noexcept;

  /**
   * Vacuums an object.
   *
   * @param[in] vacuum Setpoint for control mode. Unit: \f$[10*mbar]\f$.
   * @param[in] timeout Vacuum timeout. Unit: \f$[ms]\f$.
   * @param[in] profile Production setup profile P0 to P3. Default: P0.
   *
   * @return True if the vacuum has been established, false otherwise.
   *
   * @throw CommandException if an error occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  bool vacuum(uint8_t vacuum,
              std::chrono::milliseconds timeout,
              ProductionSetupProfile profile = ProductionSetupProfile::kP0) const;

  /**
   * Drops the grasped object off.
   *
   * @param[in] timeout Dropoff timeout. Unit: \f$[ms]\f$.
   *
   * @return True if command was successful, false otherwise.
   *
   * @throw CommandException if an error occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  bool dropOff(std::chrono::milliseconds timeout) const;

  /**
   * Stops a currently running vacuum gripper vacuum or drop off operation.
   *
   * @return True if command was successful, false otherwise.
   *
   * @throw CommandException if an error occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  bool stop() const;

  /**
   * Waits for a vacuum gripper state update and returns it.
   *
   * @return Current vacuum gripper state.
   *
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw InvalidOperationException if another readOnce is already running.
   */
  VacuumGripperState readOnce() const;

  /**
   * Returns the software version reported by the connected server.
   *
   * @return Software version of the connected server.
   */
  ServerVersion serverVersion() const noexcept;

  /// @cond DO_NOT_DOCUMENT
  VacuumGripper(const VacuumGripper&) = delete;
  VacuumGripper& operator=(const VacuumGripper&) = delete;
  /// @endcond

 private:
  std::unique_ptr<Network> network_;

  uint16_t ri_version_;
};

}  // namespace franka
