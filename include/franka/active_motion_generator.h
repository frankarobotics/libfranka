// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include "active_control.h"

/**
 * @file active_motion_generator.h
 * Contains the `franka::ActiveMotionGenerator` type.
 */

namespace franka {

/**
 * Allows the user to read the state of a Robot and to send new motion generator commands after
 * starting a control process of a Robot.
 *
 * hint: To create an ActiveMotionGenerator, see franka::Robot
 *
 */
template <typename MotionGeneratorType>
class ActiveMotionGenerator : public ActiveControl {
 public:
  ~ActiveMotionGenerator() override = default;

  /**
   * Updates the motion generator commands of an active control
   *
   * @param motion_generator_input the new motion generator input
   * @param control_input optional: the external control input for each joint, if an external
   * controller is used
   *
   * @throw ControlException if an error related to torque control or motion generation occurred,
   or
   * the motion was already finished.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  void writeOnce(const MotionGeneratorType& motion_generator_input,
                 const std::optional<const Torques>& control_input) override;
  /**
   * franka::Robot as friend to allow construction of ActiveMotionGenerator<MotionGeneratorType> in
   * start<MotionGeneratorType>Control methods
   *
   */
  friend class Robot;

 private:
  /**
   * Construct a new ActiveMotionGenerator object
   *
   * @param robot shared_ptr to the Robot::Impl in the Robot
   * @param motion_id id of the managed motion
   * @param control_lock of the Robot, preventing other read and write accesses during the active
   * control
   * @param controller_type defining which controller shall be used
   */
  ActiveMotionGenerator(std::shared_ptr<Robot::Impl> robot_impl,
                        uint32_t motion_id,
                        std::unique_lock<std::mutex> control_lock,
                        research_interface::robot::Move::ControllerMode controller_type)
      : ActiveControl(robot_impl, motion_id, std::move(control_lock)),
        controller_type_(controller_type){};

  bool isTorqueControlFinished(const std::optional<const Torques>& control_input);

  research_interface::robot::Move::ControllerMode controller_type_;
};
}  // namespace franka
