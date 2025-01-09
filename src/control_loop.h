// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cmath>
#include <functional>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>

#include "robot_control.h"

namespace franka {

/**
 * Provides the functionality to control the robot via a callback function which receives the robot
 * state as input and returns the desired commands.
 *
 * @tparam MotionControlType The type of the motion/control command.
 */
template <typename MotionControlType>
class ControlLoop {
 public:
  static constexpr research_interface::robot::Move::Deviation kDefaultDeviation{10.0, 3.12,
                                                                                2 * M_PI};

  using ControlCallback = std::function<Torques(const RobotState&, franka::Duration)>;
  using MotionGeneratorCallback =
      std::function<MotionControlType(const RobotState&, franka::Duration)>;

  /**
   * Construct a control loop with the given robot, control callback and motion generator callback.
   *
   * @param robot The robot to control.
   * @param control_callback The control callback to use.
   * @param motion_callback The motion generator callback to use.
   * @param limit_rate If the rate should be limited.
   * @param cutoff_frequency The cutoff frequency for the low-pass filter.
   */
  ControlLoop(RobotControl& robot,
              ControlCallback control_callback,
              MotionGeneratorCallback motion_callback,
              bool limit_rate,
              double cutoff_frequency);

  /**
   * Construct a control loop with the given robot and control callback.
   *
   * @param robot The robot to control.
   * @param control_callback The control callback to use.
   * @param limit_rate If the rate should be limited.
   * @param cutoff_frequency The cutoff frequency for the low-pass filter.
   */
  ControlLoop(RobotControl& robot,
              ControlCallback control_callback,
              bool limit_rate,
              double cutoff_frequency);

  /**
   * Construct a control loop with the given robot, controller mode and motion generator callback.
   *
   * @param robot The robot to control.
   * @param controller_mode The controller mode to use.
   * @param motion_callback The motion generator callback to use.
   * @param limit_rate If the rate should be limited.
   * @param cutoff_frequency The cutoff frequency for the low-pass filter.
   */
  ControlLoop(RobotControl& robot,
              ControllerMode controller_mode,
              MotionGeneratorCallback motion_callback,
              bool limit_rate,
              double cutoff_frequency);

  /**
   * Blocks and runs the control loop until the motion or control is finished.
   */
  auto loop() -> void;

 protected:
  /**
   * Creates a control command based on the given robot state and time step.
   *
   * @param robot_state The current robot state.
   * @param time_step The time step since the last call.
   * @param command The control command to create.
   * @return true if control is not finished, false otherwise.
   */
  auto createControlCommand(const RobotState& robot_state,
                            franka::Duration time_step,
                            research_interface::robot::ControllerCommand& control_command) -> bool;

  /**
   * Creates a motion command based on the given robot state and time step.
   *
   * @param robot_state The current robot state.
   * @param time_step The time step since the last call.
   * @param command The motion command to create.
   * @return true if motion is not finished, false otherwise.
   */
  auto createMotionCommand(
      const RobotState& robot_state,
      franka::Duration time_step,
      research_interface::robot::MotionGeneratorCommand& motion_generation_command) -> bool;

 private:
  RobotControl& robot_;
  const MotionGeneratorCallback motion_callback_;  // NOLINT(readability-identifier-naming)
  const ControlCallback control_callback_;         // NOLINT(readability-identifier-naming)
  const bool limit_rate_;                          // NOLINT(readability-identifier-naming)
  const double cutoff_frequency_;                  // NOLINT(readability-identifier-naming)
  uint32_t motion_id_ = 0;
  bool initialized_filter_{false};

  void convertMotion(const MotionControlType& motion,
                     const RobotState& robot_state,
                     research_interface::robot::MotionGeneratorCommand& command);
};

}  // namespace franka
