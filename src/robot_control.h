// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <optional>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

namespace franka {

/**
 * Interface for controlling the robot either via motion commands (e.g. joint position, joint
 * velocity, Cartesian pose, Cartesian velocity) or torque commands.
 */
class RobotControl {
 public:
  static constexpr size_t kNumJoints = 7;
  virtual ~RobotControl() = default;

  /**
   * Starts a new motion.
   *
   * @param controller_mode The controller mode to use for the motion.
   * @param motion_generator_mode The motion generator mode to use for the motion.
   * @param maximum_path_deviation The maximum allowed deviation from the path.
   * @param maximum_goal_pose_deviation The maximum allowed deviation from the goal pose.
   * @param maximum_velocities Optional maximum velocities for the motion generator.
   *   Only supported for:
   *   - VariableRateJointPositionControl
   * @return uint32_t The ID of the started motion.
   */
  virtual uint32_t startMotion(
      research_interface::robot::Move::ControllerMode controller_mode,
      research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
      const research_interface::robot::Move::Deviation& maximum_path_deviation,
      const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation,
      bool use_async_motion_generator,
      const std::optional<std::vector<double>>& maximum_velocities) = 0;

  /**
   * Receives the new motion/control command to send to the robot while updating the robot state.
   *
   * @param motion_command The new motion command (either valid motion command or nullptr).
   * @param control_command The new control command (either valid control command or nullptr).
   * @return RobotState The updated robot state.
   */
  virtual RobotState updateMotion(
      const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
      const std::optional<research_interface::robot::ControllerCommand>& control_command) = 0;

  /**
   * Finishes the motion.
   *
   * @param motion_id The ID of the motion to finish.
   * @param motion_command The final motion command.
   * @param control_command The final control command.
   */
  virtual void finishMotion(
      uint32_t motion_id,
      const std::optional<research_interface::robot::MotionGeneratorCommand>& motion_command,
      const std::optional<research_interface::robot::ControllerCommand>& control_command) = 0;

  /**
   * Cancels the motion with the given ID.
   *
   * @param motion_id The ID of the motion to cancel.
   */
  virtual void cancelMotion(uint32_t motion_id) = 0;

  /**
   * Throws an exception if a motion error occurred.
   *
   * @param robot_state The current robot state.
   * @param motion_id The ID of the currently active motion.
   */
  virtual void throwOnMotionError(const RobotState& robot_state, uint32_t motion_id) = 0;

  /**
   * @return RealtimeConfig The realtime configuration.
   */
  virtual RealtimeConfig realtimeConfig() const noexcept = 0;

  /**
   * Computes the upper joint velocity limits based on current joint positions.
   *
   * @param joint_positions Current joint positions.
   * @return std::array<double, kNumJoints> Upper joint velocity limits.
   */
  virtual auto getUpperJointVelocityLimits(const std::array<double, kNumJoints>& joint_positions)
      const -> std::array<double, kNumJoints> = 0;

  /**
   * Computes the lower joint velocity limits based on current joint positions.
   *
   * @param joint_positions Current joint positions.
   * @return std::array<double, kNumJoints> Lower joint velocity limits.
   */
  virtual auto getLowerJointVelocityLimits(const std::array<double, kNumJoints>& joint_positions)
      const -> std::array<double, kNumJoints> = 0;
};

}  // namespace franka
