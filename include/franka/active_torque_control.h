// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include "active_control.h"

/**
 * @file active_torque_control.h
 * Contains the `franka::ActiveTorqueControl` type.
 */

namespace franka {

/**
 * Allows the user to read the state of a Robot and to send new torque control commands after
 * starting a control process of a Robot.
 *
 * hint: To create an ActiveTorqueControl, see franka::Robot
 *
 */
class ActiveTorqueControl : public ActiveControl {
 public:
  ~ActiveTorqueControl() override = default;

  /**
   * Updates the joint-level based torque commands of an active joint effort control
   *
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred, or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const Torques& control_input) override;

  /**
   * franka::Robot as friend to allow construction of ActiveTorqueControl in
   * startTorqueControl methods
   *
   */
  friend class Robot;

 private:
  /**
   * Construct a new ActiveTorqueControl object
   *
   * @param robot shared_ptr to the Robot::Impl in the Robot
   * @param motion_id id of the managed motion
   * @param control_lock of the Robot, preventing other read and write accesses during the active
   * control
   */
  ActiveTorqueControl(std::shared_ptr<Robot::Impl> robot_impl,
                      uint32_t motion_id,
                      std::unique_lock<std::mutex> control_lock)
      : ActiveControl(std::move(robot_impl), motion_id, std::move(control_lock)){};
};

}  // namespace franka
