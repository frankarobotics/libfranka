// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <franka/active_control_base.h>
#include <franka/robot.h>
#include <franka/async_control/target_status.hpp>

namespace franka {

/**
 * Handler for asynchronous joint position control.
 *
 * The defined structures will allow us to further extend the interface in the future.
 */
class AsyncPositionControlHandler {
 public:
  // This is the input structure for the initialization. It contains parameters required for
  // the control.
  struct Configuration {
    // Maximum joint velocities for the position control. If no values are provided, the maximum
    // velocities of the robot will be used.
    std::optional<std::vector<double>> maximum_joint_velocities{};
    std::optional<double> goal_tolerance{};
  };

  // This is the output structure for the initialization.
  // It contains either the handler or an error message.
  struct ConfigurationResult {
    std::shared_ptr<AsyncPositionControlHandler> handler{};

    std::optional<std::string> error_message{};
  };

  // This defines the structure to send the next control target to the robot.
  struct JointPositionTarget {
    std::array<double, Robot::kNumJoints> joint_positions{};
  };

  // This structure holds the result after sending the next target.
  struct CommandResult {
    std::optional<std::string> motion_uuid{};

    bool was_successful{};
    std::optional<std::string> error_message{};
  };

  // This structure holds feedback information about the current target state.
  struct TargetFeedback {
    TargetStatus status{TargetStatus::kIdle};

    std::optional<std::string> error_message{};
  };

  virtual ~AsyncPositionControlHandler();

  /**
   * Constructs the async control handler.
   *
   * @param robot Shared pointer to the robot instance.
   * @param configuration Input parameters for the initialization.
   * @return ConfigurationResult containing either the handler or an error message.
   */
  static auto configure(const std::shared_ptr<Robot>& robot, const Configuration& configuration)
      -> ConfigurationResult;

  /**
   * Sets the next joint positions for the asynchronous control.
   *
   * @param target The target joint positions to set.
   * @return CommandResult containing the result for the proposed next target.
   */
  auto setJointPositionTarget(const JointPositionTarget& target) -> CommandResult;

  /**
   * Retrieves feedback about the current control state.
   *
   * @return TargetFeedback containing the current status and joint positions.
   */
  auto getTargetFeedback() -> TargetFeedback;

  /**
   * Stops the asynchronous position control.
   */
  auto stopControl() -> void;

 private:
  /**
   * Constructs an AsyncPositionControlHandler with the given active robot control.
   *
   * @param active_robot_control Unique pointer to the active control base for robot communication.
   * @param goal_tolerance The goal tolerance for reaching the target position.
   */
  explicit AsyncPositionControlHandler(std::unique_ptr<ActiveControlBase> active_robot_control,
                                       std::optional<double> goal_tolerance);

  std::shared_ptr<Robot> robot_;
  std::unique_ptr<ActiveControlBase> active_robot_control_;
  TargetStatus control_status_{TargetStatus::kIdle};

  std::array<double, Robot::kNumJoints> target_position_{};

  double goal_tolerance_;  // [rad]
};

}  // namespace franka
