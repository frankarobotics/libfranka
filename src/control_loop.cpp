// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "control_loop.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/rate_limiting.h>
#include <franka/logging/logger.hpp>

#include "motion_generator_traits.h"

namespace franka {

template <typename MotionControlType>
ControlLoop<MotionControlType>::ControlLoop(RobotControl& robot,
                                            ControlCallback control_callback,
                                            MotionGeneratorCallback motion_callback,
                                            bool limit_rate,
                                            double cutoff_frequency)
    : robot_(robot),
      motion_callback_(std::move(motion_callback)),
      control_callback_(std::move(control_callback)),
      limit_rate_(limit_rate),
      cutoff_frequency_(cutoff_frequency) {
  if (!motion_callback_ || !control_callback_) {
    logging::logError("libfranka: Got no motion or control callback.");
    throw std::invalid_argument("libfranka: Got no motion or control callback.");
  }

  // Don't use the variable rate motion generator for control loops
  motion_id_ =
      robot.startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                        MotionGeneratorTraits<MotionControlType>::kMotionGeneratorMode,
                        kDefaultDeviation, kDefaultDeviation, false, std::nullopt);
}

template <typename MotionControlType>
ControlLoop<MotionControlType>::ControlLoop(RobotControl& robot,
                                            ControlCallback control_callback,
                                            bool limit_rate,
                                            double cutoff_frequency)
    : robot_(robot),
      control_callback_(std::move(control_callback)),
      limit_rate_(limit_rate),
      cutoff_frequency_(cutoff_frequency) {
  if (!control_callback_) {
    logging::logError("libfranka: Invalid control callback given.");
    throw std::invalid_argument("libfranka: Invalid control callback given.");
  }

  // Don't use the variable rate motion generator for control loops
  motion_id_ =
      robot.startMotion(research_interface::robot::Move::ControllerMode::kExternalController,
                        research_interface::robot::Move::MotionGeneratorMode::kNone,
                        kDefaultDeviation, kDefaultDeviation, false, std::nullopt);
}

template <typename MotionControlType>
ControlLoop<MotionControlType>::ControlLoop(RobotControl& robot,
                                            ControllerMode controller_mode,
                                            MotionGeneratorCallback motion_callback,
                                            bool limit_rate,
                                            double cutoff_frequency)
    : robot_(robot),
      motion_callback_(std::move(motion_callback)),
      limit_rate_(limit_rate),
      cutoff_frequency_(cutoff_frequency) {
  if (!motion_callback_) {
    logging::logError("libfranka: Invalid motion callback given.");
    throw std::invalid_argument("libfranka: Invalid motion callback given.");
  }

  research_interface::robot::Move::ControllerMode mode{};
  switch (controller_mode) {
    case ControllerMode::kJointImpedance:
      mode = decltype(mode)::kJointImpedance;
      break;
    case ControllerMode::kCartesianImpedance:
      mode = decltype(mode)::kCartesianImpedance;
      break;
    default:
      logging::logError("libfranka: Invalid controller mode given.");
      throw std::invalid_argument("libfranka: Invalid controller mode given.");
  }

  // Don't use the variable rate motion generator for control loops
  motion_id_ =
      robot.startMotion(mode, MotionGeneratorTraits<MotionControlType>::kMotionGeneratorMode,
                        kDefaultDeviation, kDefaultDeviation, false, std::nullopt);
}

template <typename MotionControlType>
auto ControlLoop<MotionControlType>::loop() -> void try {
  // Initialize the robot state
  auto robot_state = robot_.updateMotion(std::nullopt, std::nullopt);
  robot_.throwOnMotionError(robot_state, motion_id_);

  auto control_command = std::optional<research_interface::robot::ControllerCommand>{};
  auto motion_command = std::optional<research_interface::robot::MotionGeneratorCommand>{};

  if (control_callback_) {
    control_command = research_interface::robot::ControllerCommand();
  }

  if (motion_callback_) {
    motion_command = research_interface::robot::MotionGeneratorCommand();
  }

  auto previous_time = robot_state.time;
  auto is_not_finished = true;
  while (is_not_finished) {
    if (control_callback_) {
      is_not_finished = createControlCommand(robot_state, robot_state.time - previous_time,
                                             control_command.value());
    }

    if (motion_callback_) {
      is_not_finished =
          is_not_finished && createMotionCommand(robot_state, robot_state.time - previous_time,
                                                 motion_command.value());
    }

    if (!is_not_finished) {
      // Completed the motion or control, continue with finishing
      break;
    }

    previous_time = robot_state.time;
    robot_state = robot_.updateMotion(motion_command, control_command);
    robot_.throwOnMotionError(robot_state, motion_id_);
  }

  // Done with the motion and control
  robot_.finishMotion(motion_id_, motion_command, control_command);
} catch (...) {
  try {
    robot_.cancelMotion(motion_id_);
  } catch (...) {
  }
  throw;
}

template <typename MotionControlType>
bool ControlLoop<MotionControlType>::createControlCommand(
    const RobotState& robot_state,
    franka::Duration time_step,
    research_interface::robot::ControllerCommand& control_command) {
  Torques control_output = control_callback_(robot_state, time_step);
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 7; i++) {
      control_output.tau_J[i] = lowpassFilter(kDeltaT, control_output.tau_J[i],
                                              robot_state.tau_J_d[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    control_output.tau_J = limitRate(kMaxTorqueRate, control_output.tau_J, robot_state.tau_J_d);
  }
  control_command.tau_J_d = control_output.tau_J;
  checkFinite(control_command.tau_J_d);
  return !control_output.motion_finished;
}

template <typename MotionControlType>
bool ControlLoop<MotionControlType>::createMotionCommand(
    const RobotState& robot_state,
    franka::Duration time_step,
    research_interface::robot::MotionGeneratorCommand& motion_generation_command) {
  MotionControlType motion_output = motion_callback_(robot_state, time_step);
  convertMotion(motion_output, robot_state, motion_generation_command);
  return !motion_output.motion_finished;
}

template <>
void ControlLoop<Torques>::convertMotion(
    const Torques& /* torques */,
    const RobotState& /* robot_state */,
    research_interface::robot::MotionGeneratorCommand& /* command */) {
  throw std::logic_error("libfranka: Torques cannot be converted to a motion command.");
}

template <>
void ControlLoop<JointPositions>::convertMotion(
    const JointPositions& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand& command) {
  command.q_c = motion.q;
  std::array<double, 7> reference_position{};
  if (!initialized_filter_) {
    reference_position = command.q_c;
    initialized_filter_ = true;
  } else {
    reference_position = robot_state.q_d;
  }
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 7; i++) {
      command.q_c[i] =
          lowpassFilter(kDeltaT, command.q_c[i], reference_position[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    command.q_c = limitRate(robot_.getUpperJointVelocityLimits(reference_position),
                            robot_.getLowerJointVelocityLimits(reference_position),
                            kMaxJointAcceleration, kMaxJointJerk, command.q_c, reference_position,
                            robot_state.dq_d, robot_state.ddq_d);
  }
  checkFinite(command.q_c);
}

template <>
void ControlLoop<JointVelocities>::convertMotion(
    const JointVelocities& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand& command) {
  command.dq_c = motion.dq;
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 7; i++) {
      command.dq_c[i] =
          lowpassFilter(kDeltaT, command.dq_c[i], robot_state.dq_d[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    command.dq_c =
        limitRate(robot_.getUpperJointVelocityLimits(robot_state.q_d),
                  robot_.getLowerJointVelocityLimits(robot_state.q_d), kMaxJointAcceleration,
                  kMaxJointJerk, command.dq_c, robot_state.dq_d, robot_state.ddq_d);
  }
  checkFinite(command.dq_c);
}

template <>
void ControlLoop<CartesianPose>::convertMotion(
    const CartesianPose& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand& command) {
  command.O_T_EE_c = motion.O_T_EE;
  std::array<double, 16> reference_cartesian_pose{};
  std::array<double, 2> reference_elbow_pose{};
  if (!initialized_filter_) {
    reference_cartesian_pose = command.O_T_EE_c;
    if (motion.hasElbow()) {
      reference_elbow_pose = motion.elbow;
    }
    initialized_filter_ = true;
  } else {
    reference_cartesian_pose = robot_state.O_T_EE_c;
    if (motion.hasElbow()) {
      reference_elbow_pose = robot_state.elbow_c;
    }
  }

  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    command.O_T_EE_c = cartesianLowpassFilter(kDeltaT, command.O_T_EE_c, reference_cartesian_pose,
                                              cutoff_frequency_);
  }
  if (limit_rate_) {
    command.O_T_EE_c = limitRate(
        kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
        kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk, command.O_T_EE_c,
        reference_cartesian_pose, robot_state.O_dP_EE_c, robot_state.O_ddP_EE_c);
  }
  checkMatrix(command.O_T_EE_c);

  if (motion.hasElbow()) {
    command.valid_elbow = true;
    command.elbow_c = motion.elbow;
    if (cutoff_frequency_ < kMaxCutoffFrequency) {
      command.elbow_c[0] =
          lowpassFilter(kDeltaT, command.elbow_c[0], reference_elbow_pose[0], cutoff_frequency_);
    }
    if (limit_rate_) {
      command.elbow_c[0] = limitRate(kMaxElbowVelocity, kMinElbowVelocity, kMaxElbowAcceleration,
                                     kMaxElbowJerk, command.elbow_c[0], reference_elbow_pose[0],
                                     robot_state.delbow_c[0], robot_state.ddelbow_c[0]);
    }
    checkElbow(command.elbow_c);
  } else {
    command.valid_elbow = false;
    command.elbow_c = {};
  }
}

template <>
void ControlLoop<CartesianVelocities>::convertMotion(
    const CartesianVelocities& motion,
    const RobotState& robot_state,
    research_interface::robot::MotionGeneratorCommand& command) {
  command.O_dP_EE_c = motion.O_dP_EE;
  if (cutoff_frequency_ < kMaxCutoffFrequency) {
    for (size_t i = 0; i < 6; i++) {
      command.O_dP_EE_c[i] =
          lowpassFilter(kDeltaT, command.O_dP_EE_c[i], robot_state.O_dP_EE_c[i], cutoff_frequency_);
    }
  }
  if (limit_rate_) {
    command.O_dP_EE_c =
        limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
                  kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk,
                  command.O_dP_EE_c, robot_state.O_dP_EE_c, robot_state.O_ddP_EE_c);
  }
  checkFinite(command.O_dP_EE_c);

  if (motion.hasElbow()) {
    command.valid_elbow = true;
    command.elbow_c = motion.elbow;
    if (cutoff_frequency_ < kMaxCutoffFrequency) {
      command.elbow_c[0] =
          lowpassFilter(kDeltaT, command.elbow_c[0], robot_state.elbow_c[0], cutoff_frequency_);
    }
    if (limit_rate_) {
      command.elbow_c[0] = limitRate(kMaxElbowVelocity, kMinElbowVelocity, kMaxElbowAcceleration,
                                     kMaxElbowJerk, command.elbow_c[0], robot_state.elbow_c[0],
                                     robot_state.delbow_c[0], robot_state.ddelbow_c[0]);
    }
    checkElbow(command.elbow_c);
  } else {
    command.valid_elbow = false;
    command.elbow_c = {};
  }
}

template class ControlLoop<JointPositions>;
template class ControlLoop<JointVelocities>;
template class ControlLoop<CartesianPose>;
template class ControlLoop<CartesianVelocities>;
template class ControlLoop<Torques>;

}  // namespace franka
