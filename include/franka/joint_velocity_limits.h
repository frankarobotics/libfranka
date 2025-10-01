// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>

/**
 * @file joint_velocity_limits.h
 * Contains types and utilities for managing position-based joint velocity limits.
 */

namespace franka {

/**
 * Structure to hold position-based joint velocity limit constants for a single joint from URDF.
 * These parameters define velocity limits that depend on the current joint position.
 */
struct PositionBasedJointVelocityLimitConstants {
  double max_velocity;
  double velocity_offset;
  double deceleration_limit;
  double upper_position_limit;
  double lower_position_limit;

  PositionBasedJointVelocityLimitConstants()
      : max_velocity(0),
        velocity_offset(0),
        deceleration_limit(0),
        upper_position_limit(0),
        lower_position_limit(0) {}

  /**
   * Constructor with specified parameters
   * @param max_velocity Maximum velocity
   * @param velocity_offset Velocity offset parameter
   * @param deceleration_limit Deceleration limit parameter
   * @param upper_position_limit Upper position limit
   * @param lower_position_limit Lower position limit
   */
  PositionBasedJointVelocityLimitConstants(double max_velocity,
                                           double velocity_offset,
                                           double deceleration_limit,
                                           double upper_position_limit,
                                           double lower_position_limit)
      : max_velocity(max_velocity),
        velocity_offset(velocity_offset),
        deceleration_limit(deceleration_limit),
        upper_position_limit(upper_position_limit),
        lower_position_limit(lower_position_limit) {}
};

/**
 * Configuration class that holds position-based joint velocity limit parameters for all 7 joints.
 * This class handles parsing URDF data, storing joint configurations, and computing velocity
 * limits.
 */
class JointVelocityLimitsConfig {
 public:
  // Robot configuration constants
  static constexpr int kNumJoints = 7;

  JointVelocityLimitsConfig() = default;

  /**
   * Constructor that parses URDF string and initializes joint parameters
   * @param urdf_string The URDF string to parse for joint velocity limit parameters
   */
  explicit JointVelocityLimitsConfig(const std::string& urdf_string) { parseFromURDF(urdf_string); }

  /**
   * Get joint parameters for a specific joint
   * @param joint_index Index of the joint (0-6)
   * @return Joint velocity limit parameters for the specified joint
   */
  const PositionBasedJointVelocityLimitConstants& operator[](size_t joint_index) const {
    return joint_params_[joint_index];
  }

  /**
   * Get all joint parameters
   * @return Array of joint velocity limit parameters for all joints
   */
  const std::array<PositionBasedJointVelocityLimitConstants, kNumJoints>& getJointParams() const {
    return joint_params_;
  }

  /**
   * Compute upper joint velocity limits based on current joint positions
   * @param joint_positions Current joint positions
   * @return Array of upper velocity limits for each joint
   */
  std::array<double, kNumJoints> getUpperJointVelocityLimits(
      const std::array<double, kNumJoints>& joint_positions) const;

  /**
   * Compute lower joint velocity limits based on current joint positions
   * @param joint_positions Current joint positions
   * @return Array of lower velocity limits for each joint
   */
  std::array<double, kNumJoints> getLowerJointVelocityLimits(
      const std::array<double, kNumJoints>& joint_positions) const;

 private:
  // XML element and attribute name constants
  static constexpr const char* kRobotElementName = "robot";
  static constexpr const char* kJointElementName = "joint";
  static constexpr const char* kPositionBasedVelocityLimitsElementName =
      "position_based_velocity_limits";
  static constexpr const char* kLimitElementName = "limit";
  static constexpr const char* kNameAttributeName = "name";
  static constexpr const char* kVelocityAttributeName = "velocity";
  static constexpr const char* kUpperAttributeName = "upper";
  static constexpr const char* kLowerAttributeName = "lower";
  static constexpr const char* kVelocityOffsetAttributeName = "velocity_offset";
  static constexpr const char* kDecelerationLimitAttributeName = "deceleration_limit";

  // Joint name pattern constants
  static constexpr const char* kJoint1Name = "joint1";
  static constexpr const char* kJoint2Name = "joint2";
  static constexpr const char* kJoint3Name = "joint3";
  static constexpr const char* kJoint4Name = "joint4";
  static constexpr const char* kJoint5Name = "joint5";
  static constexpr const char* kJoint6Name = "joint6";
  static constexpr const char* kJoint7Name = "joint7";

  /**
   * Parse joint velocity limit parameters from URDF string
   * @param urdf_string The URDF string to parse
   */
  void parseFromURDF(const std::string& urdf_string);

  /**
   * Get joint index from joint name using efficient lookup
   * @param joint_name The name of the joint to find index for
   * @return Joint index (0-6) or -1 if not found
   */
  static int getJointIndex(const std::string& joint_name);

  std::array<PositionBasedJointVelocityLimitConstants, kNumJoints> joint_params_;
};

}  // namespace franka