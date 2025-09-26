// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <tinyxml2.h>
#include <array>
#include <bitset>
#include <stdexcept>
#include <string>
#include <unordered_map>

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
  double max_base_velocity;           // Maximum base velocity for the joint
  double velocity_offset;             // Velocity offset parameter from URDF
  double deceleration_limit;          // Deceleration limit parameter from URDF
  double upper_joint_position_limit;  // Reference position for upper limit calculation
  double lower_joint_position_limit;  // Reference position for lower limit calculation

  PositionBasedJointVelocityLimitConstants()
      : max_base_velocity(0),
        velocity_offset(0),
        deceleration_limit(0),
        upper_joint_position_limit(0),
        lower_joint_position_limit(0) {}

  /**
   * Constructor with specified parameters
   * @param max_vel Maximum base velocity
   * @param velocity_offset Velocity offset parameter
   * @param deceleration_limit Deceleration limit parameter
   * @param upper_joint_position_limit Upper reference position
   * @param lower_joint_position_limit Lower reference position
   */
  PositionBasedJointVelocityLimitConstants(double max_vel,
                                           double velocity_offset,
                                           double deceleration_limit,
                                           double upper_joint_position_limit,
                                           double lower_joint_position_limit)
      : max_base_velocity(max_vel),
        velocity_offset(velocity_offset),
        deceleration_limit(deceleration_limit),
        upper_joint_position_limit(upper_joint_position_limit),
        lower_joint_position_limit(lower_joint_position_limit) {}
};

/**
 * Configuration class that holds position-based joint velocity limit parameters for all 7 joints.
 * This class handles parsing URDF data and storing the resulting joint configurations.
 */
class JointVelocityLimitsConfig {
 public:
  JointVelocityLimitsConfig() = default;

  /**
   * Constructor that parses URDF string and initializes joint parameters
   * @param urdf_string The URDF string to parse for joint velocity limit parameters
   */
  explicit JointVelocityLimitsConfig(const std::string& urdf_string) { parseFromURDF(urdf_string); }

  /**
   * Parse joint velocity limit parameters from URDF string
   * @param urdf_string The URDF string to parse
   */
  void parseFromURDF(const std::string& urdf_string);

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
  const std::array<PositionBasedJointVelocityLimitConstants, 7>& getJointParams() const {
    return joint_params_;
  }

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
   * Get joint index from joint name using efficient lookup
   * @param joint_name The name of the joint to find index for
   * @return Joint index (0-6) or -1 if not found
   */
  static int getJointIndex(const std::string& joint_name);

  std::array<PositionBasedJointVelocityLimitConstants, 7> joint_params_;
};

inline void JointVelocityLimitsConfig::parseFromURDF(const std::string& urdf_string) {
  // Reset to defaults
  joint_params_ = {};

  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_string.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to parse URDF for joint velocity limits");
  }

  auto* robot = doc.FirstChildElement(kRobotElementName);
  if (!robot) {
    throw std::runtime_error(
        "Failed to parse URDF: no <robot> element exists for joint velocity limits");
  }

  std::bitset<7> found_joints;
  
  auto parseRequiredDouble = [](const tinyxml2::XMLElement* elem, const char* attr, 
                               const std::string& joint_name, const std::string& element_name) -> double {
    if (!elem) {
      throw std::runtime_error("Missing <" + element_name + "> element for joint: " + joint_name);
    }
    const char* value = elem->Attribute(attr);
    if (!value) {
      throw std::runtime_error("Missing '" + std::string(attr) + "' attribute in <" + 
                               element_name + "> for joint: " + joint_name);
    }
    return std::stod(value);
  };

  for (auto* joint = robot->FirstChildElement(kJointElementName); joint;
       joint = joint->NextSiblingElement(kJointElementName)) {
    const char* name = joint->Attribute(kNameAttributeName);
    if (!name)
      continue;

    int idx = getJointIndex(name);
    if (idx == -1)
      continue;

    std::string joint_name_str(name);

    auto* position_based_limits = joint->FirstChildElement(kPositionBasedVelocityLimitsElementName);
    if (!position_based_limits) {
      throw std::runtime_error("Missing <" + std::string(kPositionBasedVelocityLimitsElementName) + 
                               "> element for joint: " + joint_name_str);
    }

    auto* limit = joint->FirstChildElement(kLimitElementName);
    if (!limit) {
      throw std::runtime_error("Missing <" + std::string(kLimitElementName) + 
                               "> element for joint: " + joint_name_str);
    }

    found_joints[idx] = true;

    joint_params_[idx] = {
        parseRequiredDouble(limit, kVelocityAttributeName, joint_name_str, kLimitElementName),
        parseRequiredDouble(position_based_limits, kVelocityOffsetAttributeName, 
                           joint_name_str, kPositionBasedVelocityLimitsElementName),
        parseRequiredDouble(position_based_limits, kDecelerationLimitAttributeName, 
                           joint_name_str, kPositionBasedVelocityLimitsElementName),
        parseRequiredDouble(limit, kUpperAttributeName, joint_name_str, kLimitElementName),
        parseRequiredDouble(limit, kLowerAttributeName, joint_name_str, kLimitElementName)
    };
  }

  if (!found_joints.all()) {
    std::string missing_joints = "Missing required joints: ";
    for (int i = 0; i < 7; ++i) {
      if (!found_joints[i]) {
        missing_joints += "joint" + std::to_string(i + 1) + " ";
      }
    }
    throw std::runtime_error(missing_joints);
  }
}

inline int JointVelocityLimitsConfig::getJointIndex(const std::string& joint_name) {
  static const std::unordered_map<std::string, int> joint_name_to_index = {
      {kJoint1Name, 0}, {kJoint2Name, 1}, {kJoint3Name, 2}, {kJoint4Name, 3},
      {kJoint5Name, 4}, {kJoint6Name, 5}, {kJoint7Name, 6}};

  auto exact_match = joint_name_to_index.find(joint_name);
  if (exact_match != joint_name_to_index.end()) {
    return exact_match->second;
  }

  for (const auto& [joint_pattern, index] : joint_name_to_index) {
    if (joint_name.find(joint_pattern) != std::string::npos) {
      return index;
    }
  }

  return -1;
}

}  // namespace franka
