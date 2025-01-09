// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <research_interface/robot/rbk_types.h>

#include "robot_impl.h"

namespace franka {

template <typename MotionGeneratorType>
bool ActiveMotionGenerator<MotionGeneratorType>::isTorqueControlFinished(
    const std::optional<const Torques>& control_input) {
  if (!control_input.has_value()) {
    return false;
  }
  return control_input.value().motion_finished;
}

template <typename MotionGeneratorType>
void ActiveMotionGenerator<MotionGeneratorType>::writeOnce(
    const MotionGeneratorType& motion_generator_input,
    const std::optional<const Torques>& control_input) {
  if (control_finished) {
    throw franka::ControlException("writeOnce must not be called after the motion has finished.");
  }

  if (control_input.has_value() &&
      controller_type_ != research_interface::robot::Move::ControllerMode::kExternalController) {
    throw franka::ControlException("Torques can only be commanded in kExternalController mode.");
  }

  if (!control_input.has_value() &&
      controller_type_ == research_interface::robot::Move::ControllerMode::kExternalController) {
    throw franka::ControlException(
        "Torque command missing, please use writeOnce(const MotionGeneratorType& "
        "motion_generator_input, const Torques& control_input) for external controllers.");
  }

  if (motion_generator_input.motion_finished || isTorqueControlFinished(control_input)) {
    auto motion_command = robot_impl->createMotionCommand(motion_generator_input);
    if (!control_input.has_value()) {
      robot_impl->finishMotion(motion_id, motion_command, std::nullopt);
    } else {
      auto control_command = robot_impl->createControllerCommand(control_input.value());
      robot_impl->finishMotion(motion_id, motion_command, control_command);
    }

    control_finished = true;
    control_lock.unlock();
    return;
  }

  if (!control_input.has_value()) {
    robot_impl->writeOnce(motion_generator_input);
    return;
  }

  robot_impl->writeOnce(motion_generator_input, control_input.value());
}

template class ActiveMotionGenerator<JointPositions>;
template class ActiveMotionGenerator<JointVelocities>;
template class ActiveMotionGenerator<CartesianPose>;
template class ActiveMotionGenerator<CartesianVelocities>;

}  // namespace franka
