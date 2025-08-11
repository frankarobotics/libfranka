// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/errors.h>

#include <algorithm>
#include <iterator>
#include <sstream>

#include <research_interface/robot/error.h>

using Error = research_interface::robot::Error;

namespace franka {

Errors::Errors() : Errors(std::array<bool, 41>{}) {}

Errors::Errors(const Errors& other) : Errors(other.errors_) {}

Errors& Errors::operator=(Errors other) {
  std::swap(errors_, other.errors_);
  return *this;
}

Errors::Errors(const std::array<bool, 41>& errors)  // NOLINT(modernize-pass-by-value)
    : errors_(errors),
      joint_position_limits_violation(
          errors_[static_cast<size_t>(Error::kJointPositionLimitsViolation)]),
      cartesian_position_limits_violation(
          errors_[static_cast<size_t>(Error::kCartesianPositionLimitsViolation)]),
      self_collision_avoidance_violation(
          errors_[static_cast<size_t>(Error::kSelfcollisionAvoidanceViolation)]),
      joint_velocity_violation(errors_[static_cast<size_t>(Error::kJointVelocityViolation)]),
      cartesian_velocity_violation(
          errors_[static_cast<size_t>(Error::kCartesianVelocityViolation)]),
      force_control_safety_violation(
          errors_[static_cast<size_t>(Error::kForceControlSafetyViolation)]),
      joint_reflex(errors_[static_cast<size_t>(Error::kJointReflex)]),
      cartesian_reflex(errors_[static_cast<size_t>(Error::kCartesianReflex)]),
      max_goal_pose_deviation_violation(
          errors_[static_cast<size_t>(Error::kMaxGoalPoseDeviationViolation)]),
      max_path_pose_deviation_violation(
          errors_[static_cast<size_t>(Error::kMaxPathPoseDeviationViolation)]),
      cartesian_velocity_profile_safety_violation(
          errors_[static_cast<size_t>(Error::kCartesianVelocityProfileSafetyViolation)]),
      joint_position_motion_generator_start_pose_invalid(
          errors_[static_cast<size_t>(Error::kJointPositionMotionGeneratorStartPoseInvalid)]),
      joint_motion_generator_position_limits_violation(
          errors_[static_cast<size_t>(Error::kJointMotionGeneratorPositionLimitsViolation)]),
      joint_motion_generator_velocity_limits_violation(
          errors_[static_cast<size_t>(Error::kJointMotionGeneratorVelocityLimitsViolation)]),
      joint_motion_generator_velocity_discontinuity(
          errors_[static_cast<size_t>(Error::kJointMotionGeneratorVelocityDiscontinuity)]),
      joint_motion_generator_acceleration_discontinuity(
          errors_[static_cast<size_t>(Error::kJointMotionGeneratorAccelerationDiscontinuity)]),
      cartesian_position_motion_generator_start_pose_invalid(
          errors_[static_cast<size_t>(Error::kCartesianPositionMotionGeneratorStartPoseInvalid)]),
      cartesian_motion_generator_elbow_limit_violation(
          errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorElbowLimitViolation)]),
      cartesian_motion_generator_velocity_limits_violation(
          errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorVelocityLimitsViolation)]),
      cartesian_motion_generator_velocity_discontinuity(
          errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorVelocityDiscontinuity)]),
      cartesian_motion_generator_acceleration_discontinuity(
          errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorAccelerationDiscontinuity)]),
      cartesian_motion_generator_elbow_sign_inconsistent(
          errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorElbowSignInconsistent)]),
      cartesian_motion_generator_start_elbow_invalid(
          errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorStartElbowInvalid)]),
      cartesian_motion_generator_joint_position_limits_violation(errors_[static_cast<size_t>(
          Error::kCartesianMotionGeneratorJointPositionLimitsViolation)]),
      cartesian_motion_generator_joint_velocity_limits_violation(errors_[static_cast<size_t>(
          Error::kCartesianMotionGeneratorJointVelocityLimitsViolation)]),
      cartesian_motion_generator_joint_velocity_discontinuity(
          errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorJointVelocityDiscontinuity)]),
      cartesian_motion_generator_joint_acceleration_discontinuity(errors_[static_cast<size_t>(
          Error::kCartesianMotionGeneratorJointAccelerationDiscontinuity)]),
      cartesian_position_motion_generator_invalid_frame(
          errors_[static_cast<size_t>(Error::kCartesianPositionMotionGeneratorInvalidFrame)]),
      force_controller_desired_force_tolerance_violation(
          errors_[static_cast<size_t>(Error::kForceControllerDesiredForceToleranceViolation)]),
      controller_torque_discontinuity(
          errors_[static_cast<size_t>(Error::kControllerTorqueDiscontinuity)]),
      start_elbow_sign_inconsistent(
          errors_[static_cast<size_t>(Error::kStartElbowSignInconsistent)]),
      communication_constraints_violation(
          errors_[static_cast<size_t>(Error::kCommunicationConstraintsViolation)]),
      power_limit_violation(errors_[static_cast<size_t>(Error::kPowerLimitViolation)]),
      joint_p2p_insufficient_torque_for_planning(
          errors_[static_cast<size_t>(Error::kJointP2PInsufficientTorqueForPlanning)]),
      tau_j_range_violation(errors_[static_cast<size_t>(Error::kTauJRangeViolation)]),
      instability_detected(errors_[static_cast<size_t>(Error::kInstabilityDetection)]),
      joint_move_in_wrong_direction(
          errors_[static_cast<size_t>(Error::kJointMoveInWrongDirection)]),
      cartesian_spline_motion_generator_violation(
          errors_[static_cast<size_t>(Error::kCartesianSplineViolation)]),
      joint_via_motion_generator_planning_joint_limit_violation(
          errors_[static_cast<size_t>(Error::kJointViaPlanLimitViolation)]),
      base_acceleration_initialization_timeout(
          errors_[static_cast<size_t>(Error::kBaseAccelerationInitializationTimeout)]),
      base_acceleration_invalid_reading(
          errors_[static_cast<size_t>(Error::kBaseAccelerationInvalidReading)]) {}

Errors::operator bool() const noexcept {
  return std::any_of(errors_.cbegin(), errors_.cend(), [](bool is_error) { return is_error; });
}

Errors::operator std::string() const {
  std::string error_string = "[";

  for (size_t i = 0; i < errors_.size(); i++) {
    if (errors_[i]) {
      error_string += "\"";
      error_string += getErrorName(static_cast<Error>(i));
      error_string += "\", ";
    }
  }

  if (error_string.size() > 1) {
    error_string.erase(error_string.end() - 2, error_string.end());
  }

  error_string += "]";

  return error_string;
}

std::ostream& operator<<(std::ostream& ostream, const Errors& errors) {
  ostream << static_cast<std::string>(errors);
  return ostream;
}

}  // namespace franka
