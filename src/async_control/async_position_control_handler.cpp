// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <Eigen/Core>

#include <franka/async_control/async_position_control_handler.hpp>
#include <franka/logging/logger.hpp>

constexpr double kDefaultGoalTolerance = 0.001;           // [rad]
constexpr double kTargetReachedVelocityThreshold = 0.01;  // [rad/s]

using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace franka {

AsyncPositionControlHandler::AsyncPositionControlHandler(
    std::unique_ptr<ActiveControlBase> active_robot_control,
    std::optional<double> goal_tolerance)
    : active_robot_control_(std::move(active_robot_control)),
      goal_tolerance_(goal_tolerance.value_or(kDefaultGoalTolerance)) {}

AsyncPositionControlHandler::~AsyncPositionControlHandler() {
  if (active_robot_control_ != nullptr) {
    active_robot_control_.reset();
  }
}

auto AsyncPositionControlHandler::configure(const std::shared_ptr<Robot>& robot,
                                            const Configuration& configuration)
    -> ConfigurationResult {
  ConfigurationResult result;

  try {
    auto active_robot_control = robot->startAsyncJointPositionControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance,  // make this configurable
        configuration.maximum_joint_velocities);

    result.handler = std::shared_ptr<AsyncPositionControlHandler>(new AsyncPositionControlHandler(
        std::move(active_robot_control), configuration.goal_tolerance));

    return result;
  } catch (const std::exception& e) {
    result.error_message = e.what();
    logging::logError("Error while constructing AsyncPositionControlHandler: {}",
                      result.error_message.value());

    return result;
  }

  logging::logInfo("Async position control initialized successfully.");

  return result;
}

auto AsyncPositionControlHandler::setJointPositionTarget(const JointPositionTarget& target)
    -> CommandResult {
  CommandResult result{};

  if (control_status_ == TargetStatus::kAborted || active_robot_control_ == nullptr) {
    result.error_message = "Control interface is in aborted state.";
    logging::logError("Error while setting next joint position: {}", result.error_message.value());

    return result;
  }

  try {
    auto next_joint_position = JointPositions(target.joint_positions);
    next_joint_position.motion_finished = false;
    active_robot_control_->writeOnce(next_joint_position);

    result.was_successful = true;
    result.motion_uuid = std::string{"test_uuid"};
  } catch (const std::exception& e) {
    result.error_message = e.what();
    result.was_successful = false;
    logging::logError("Error while setting next joint positions: {}", result.error_message.value());

    control_status_ = TargetStatus::kAborted;

    return result;
  }

  target_position_ = target.joint_positions;

  return result;
}

auto AsyncPositionControlHandler::getTargetFeedback() -> TargetFeedback {
  TargetFeedback feedback{};

  if (control_status_ == TargetStatus::kAborted || active_robot_control_ == nullptr) {
    feedback.status = TargetStatus::kAborted;

    feedback.error_message = "Control interface is in aborted state.";

    logging::logError("Error while receiving feedback: {}", feedback.error_message.value());

    return feedback;
  }

  try {
    auto [robot_state, duration] = active_robot_control_->readOnce();

    switch (robot_state.robot_mode) {
      case RobotMode::kMove: {
        Eigen::Map<const Vector7d> current_position(robot_state.q_d.data(), Robot::kNumJoints);
        Eigen::Map<const Vector7d> target_position(target_position_.data(), Robot::kNumJoints);
        Eigen::Map<const Vector7d> current_velocity(robot_state.dq_d.data(), Robot::kNumJoints);
        auto position_error = (current_position - target_position).norm();
        auto current_speed = current_velocity.norm();

        if (position_error < goal_tolerance_ && current_speed < kTargetReachedVelocityThreshold) {
          feedback.status = TargetStatus::kTargetReached;
        } else {
          feedback.status = TargetStatus::kExecuting;
        }
        break;
      }
      case RobotMode::kIdle:
        feedback.status = TargetStatus::kIdle;
        break;
      case RobotMode::kReflex:
        feedback.status = TargetStatus::kAborted;
        feedback.error_message = "Control aborted due to reflex.";
        break;
      case RobotMode::kUserStopped:
        feedback.status = TargetStatus::kAborted;
        feedback.error_message = "Control aborted due to user stop.";
        break;
      default:
        break;
    }
  } catch (const std::exception& e) {
    feedback.status = TargetStatus::kAborted;

    feedback.error_message = e.what();

    logging::logError("Error while receiving feedback: {}", feedback.error_message.value());
  }

  return feedback;
}

auto AsyncPositionControlHandler::stopControl() -> void {
  try {
    // ToDo(kuhn_an): Stop motion gracefully
    // auto motion_finished = JointPositions{target_position_};
    // motion_finished.motion_finished = true;
    // active_robot_control_->writeOnce(motion_finished);

    active_robot_control_.reset();
    logging::logInfo("Async position control stopped successfully.");
  } catch (const std::exception& e) {
    logging::logError("Error while stopping async position control: {}", e.what());
  }

  control_status_ = TargetStatus::kIdle;
}

}  // namespace franka
