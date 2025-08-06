// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka/active_control_base.h>
#include <franka/exception.h>

#include "robot.h"

/**
 * @file active_control.h
 * Implements the ActiveControlBase abstract class. Contains the `franka::ActiveControl`,
 * `franka::ActiveTorqueControl` and `franka::ActiveMotionGenerator` type.
 */

namespace franka {

/**
 * Documented in ActiveControlBase
 */
class ActiveControl : public ActiveControlBase {
 public:
  ~ActiveControl() override;

  std::pair<RobotState, Duration> readOnce() override;

  void writeOnce(const Torques& /* control_input */) override {
    throw franka::ControlException(wrong_write_once_method_called);
  };

  void writeOnce(const JointPositions& /* motion_generator_input */,
                 const std::optional<const Torques>& /*control_input*/) override {
    throw franka::ControlException(wrong_write_once_method_called);
  };

  void writeOnce(const JointVelocities& /* motion_generator_input */,
                 const std::optional<const Torques>& /* control_input */) override {
    throw franka::ControlException(wrong_write_once_method_called);
  };

  void writeOnce(const CartesianPose& /* motion_generator_input */,
                 const std::optional<const Torques>& /* control_input */) override {
    throw franka::ControlException(wrong_write_once_method_called);
  };

  void writeOnce(const CartesianVelocities& /* motion_generator_input */,
                 const std::optional<const Torques>& /* control_input */) override {
    throw franka::ControlException(wrong_write_once_method_called);
  };

  void writeOnce(const JointPositions& motion_generator_input) override {
    writeOnce(motion_generator_input, std::optional<const Torques>());
  };

  void writeOnce(const JointVelocities& motion_generator_input) override {
    writeOnce(motion_generator_input, std::optional<const Torques>());
  };

  void writeOnce(const CartesianPose& motion_generator_input) override {
    writeOnce(motion_generator_input, std::optional<const Torques>());
  };

  void writeOnce(const CartesianVelocities& motion_generator_input) override {
    writeOnce(motion_generator_input, std::optional<const Torques>());
  };

 protected:
  /**
   * Construct a new ActiveControl object
   *
   * @param robot_impl shared_ptr to the Robot::Impl in the Robot
   * @param motion_id id of the managed motion
   * @param control_lock of the Robot, preventing other read and write accesses during the active
   * control
   */
  ActiveControl(std::shared_ptr<Robot::Impl> robot_impl,
                uint32_t motion_id,
                std::unique_lock<std::mutex> control_lock);

  /// shared pointer to Robot::Impl instance for read and write accesses
  std::shared_ptr<Robot::Impl> robot_impl;

  /// motion id of running motion
  uint32_t motion_id;

  /// control-lock preventing parallel control processes
  std::unique_lock<std::mutex> control_lock;

  /// flag indicating if control process is finished
  bool control_finished{false};

  /// duration to last read access
  std::optional<Duration> last_read_access;

 private:
  const std::string wrong_write_once_method_called{
      "Wrong writeOnce method called for currently active control!"};
};

}  // namespace franka
