// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/joint_velocity_limits.h>
#include <franka/rate_limiting.h>

#include <tinyxml2.h>

#include <algorithm>
#include <bitset>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace franka {

void JointVelocityLimitsConfig::parseFromURDF(const std::string& urdf_string) {
  // Reset to defaults
  joint_params_ = {};

  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_string.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to parse URDF for joint velocity limits");
  }

  auto* robot = doc.FirstChildElement(kRobotElementName);
  if (robot == nullptr) {
    throw std::runtime_error(
        "Failed to parse URDF: no <robot> element exists for joint velocity limits");
  }

  std::bitset<kNumJoints> found_joints;

  auto parse_required_double = [](const tinyxml2::XMLElement* elem, const char* attr,
                                  const std::string& joint_name,
                                  const std::string& element_name) -> double {
    if (elem == nullptr) {
      throw std::runtime_error("Missing <" + element_name + "> element for joint: " + joint_name);
    }
    const char* value = elem->Attribute(attr);
    if (value == nullptr) {
      throw std::runtime_error("Missing '" + std::string(attr) + "' attribute in <" + element_name +
                               "> for joint: " + joint_name);
    }
    return std::stod(value);
  };

  for (auto* joint = robot->FirstChildElement(kJointElementName); joint != nullptr;
       joint = joint->NextSiblingElement(kJointElementName)) {
    const char* name = joint->Attribute(kNameAttributeName);
    if (name == nullptr) {
      continue;
    }

    int idx = getJointIndex(name);
    if (idx == -1) {
      continue;
    }

    std::string joint_name_str(name);

    auto* position_based_limits = joint->FirstChildElement(kPositionBasedVelocityLimitsElementName);
    if (position_based_limits == nullptr) {
      throw std::runtime_error("Missing <" + std::string(kPositionBasedVelocityLimitsElementName) +
                               "> element for joint: " + joint_name_str);
    }

    auto* limit = joint->FirstChildElement(kLimitElementName);
    if (limit == nullptr) {
      throw std::runtime_error("Missing <" + std::string(kLimitElementName) +
                               "> element for joint: " + joint_name_str);
    }

    found_joints[idx] = true;

    joint_params_[idx] = {
        parse_required_double(limit, kVelocityAttributeName, joint_name_str, kLimitElementName),
        parse_required_double(position_based_limits, kVelocityOffsetAttributeName, joint_name_str,
                              kPositionBasedVelocityLimitsElementName),
        parse_required_double(position_based_limits, kDecelerationLimitAttributeName,
                              joint_name_str, kPositionBasedVelocityLimitsElementName),
        parse_required_double(limit, kUpperAttributeName, joint_name_str, kLimitElementName),
        parse_required_double(limit, kLowerAttributeName, joint_name_str, kLimitElementName)};
  }

  if (!found_joints.all()) {
    std::string missing_joints = "Missing required joints: ";
    for (int i = 0; i < kNumJoints; ++i) {
      if (!found_joints[i]) {
        missing_joints += "joint" + std::to_string(i + 1) + " ";
      }
    }
    throw std::runtime_error(missing_joints);
  }
}

int JointVelocityLimitsConfig::getJointIndex(const std::string& joint_name) {
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

std::array<double, JointVelocityLimitsConfig::kNumJoints>
JointVelocityLimitsConfig::getUpperJointVelocityLimits(
    const std::array<double, kNumJoints>& joint_positions) const {
  std::array<double, kNumJoints> result;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto& params = joint_params_[i];
    result[i] =
        std::min(params.max_velocity,
                 std::max(0.0, -params.velocity_offset +
                                   std::sqrt(std::max(0.0, 2.0 * params.deceleration_limit *
                                                               (params.upper_position_limit -
                                                                joint_positions[i]))))) -
        kJointVelocityLimitsTolerance[i];
  }
  return result;
}

std::array<double, JointVelocityLimitsConfig::kNumJoints>
JointVelocityLimitsConfig::getLowerJointVelocityLimits(
    const std::array<double, kNumJoints>& joint_positions) const {
  std::array<double, kNumJoints> result;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto& params = joint_params_[i];
    result[i] =
        std::max(-params.max_velocity,
                 std::min(0.0, params.velocity_offset -
                                   std::sqrt(std::max(0.0, 2.0 * params.deceleration_limit *
                                                               (-params.lower_position_limit +
                                                                joint_positions[i]))))) +
        kJointVelocityLimitsTolerance[i];
  }
  return result;
}

}  // namespace franka
