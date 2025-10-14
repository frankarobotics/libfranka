// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <franka/gripper_state.h>
#include <franka/robot_state.h>
#include <franka/vacuum_gripper_state.h>
#include <research_interface/gripper/types.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>
#include <franka/logging/robot_state_log.hpp>

/**
 * @brief Computes the reference lower limits for joint velocity calculation for testing only.
 */
constexpr std::array<double, 7> referenceLowerLimitsJointVelocityCalculation(
    const std::array<double, 7>& q) {
  return std::array<double, 7>{
      std::max(-2.62, std::min(0.0, 0.30 - std::sqrt(std::max(0.0, 12.0 * (2.750100 + q[0]))))) +
          0.001,
      std::max(-2.62, std::min(0.0, 0.20 - std::sqrt(std::max(0.0, 5.17 * (1.791800 + q[1]))))) +
          0.001,
      std::max(-2.62, std::min(0.0, 0.20 - std::sqrt(std::max(0.0, 7.00 * (2.906500 + q[2]))))) +
          0.001,
      std::max(-2.62, std::min(0.0, 0.30 - std::sqrt(std::max(0.0, 8.00 * (3.048100 + q[3]))))) +
          0.001,
      std::max(-5.26, std::min(0.0, 0.35 - std::sqrt(std::max(0.0, 34.0 * (2.810100 + q[4]))))) +
          0.001,
      std::max(-4.18, std::min(0.0, 0.35 - std::sqrt(std::max(0.0, 11.0 * (-0.54092 + q[5]))))) +
          0.001,
      std::max(-5.26, std::min(0.0, 0.35 - std::sqrt(std::max(0.0, 34.0 * (3.019600 + q[6]))))) +
          0.001,
  };
}

/**
 * @brief Computes the reference upper limits for joint velocity calculation for testing only.
 */
constexpr std::array<double, 7> referenceUpperLimitsJointVelocityCalculation(
    const std::array<double, 7>& q) {
  return std::array<double, 7>{
      std::min(2.62, std::max(0.0, -0.30 + std::sqrt(std::max(0.0, 12.0 * (2.75010 - q[0]))))) -
          0.001,
      std::min(2.62, std::max(0.0, -0.20 + std::sqrt(std::max(0.0, 5.17 * (1.79180 - q[1]))))) -
          0.001,
      std::min(2.62, std::max(0.0, -0.20 + std::sqrt(std::max(0.0, 7.00 * (2.90650 - q[2]))))) -
          0.001,
      std::min(2.62, std::max(0.0, -0.30 + std::sqrt(std::max(0.0, 8.00 * (-0.1458 - q[3]))))) -
          0.001,
      std::min(5.26, std::max(0.0, -0.35 + std::sqrt(std::max(0.0, 34.0 * (2.81010 - q[4]))))) -
          0.001,
      std::min(4.18, std::max(0.0, -0.35 + std::sqrt(std::max(0.0, 11.0 * (4.52050 - q[5]))))) -
          0.001,
      std::min(5.26, std::max(0.0, -0.35 + std::sqrt(std::max(0.0, 34.0 * (3.01960 - q[6]))))) -
          0.001,
  };
}

bool stringContains(const std::string& actual, const std::string& expected);

std::vector<std::string> splitAt(const std::string& s, char delimiter);

template <typename T>
std::vector<T> findDuplicates(const std::vector<T>& xs) {
  // Create a histogram of the elements
  std::map<T, int> counts;
  for (auto& x : xs) {
    // Insert 0 if first occurrence, otherwise return current pair, then increment by 1
    counts.insert(std::make_pair(x, 0)).first->second++;
  }

  // Fold histogram into a list where f(x) > 1
  std::vector<T> dups;
  for (auto pair : counts) {
    if (pair.second > 1) {
      dups.push_back(pair.first);
    }
  }
  return dups;
}

void randomRobotState(franka::RobotState& robot_state);
void randomRobotState(research_interface::robot::RobotState& robot_state);
void testRobotStateIsZero(const franka::RobotState& actual);
void testRobotStatesAreEqual(const research_interface::robot::RobotState& expected,
                             const franka::RobotState& actual);
void testRobotStatesAreEqual(const franka::RobotState& expected, const franka::RobotState& actual);

std::tuple<std::optional<research_interface::robot::MotionGeneratorCommand>,
           std::optional<research_interface::robot::ControllerCommand>>
randomRobotCommand();
void testMotionGeneratorCommandsAreEqual(
    const research_interface::robot::MotionGeneratorCommand& expected,
    const research_interface::robot::MotionGeneratorCommand& actual);
void testControllerCommandsAreEqual(const research_interface::robot::ControllerCommand& expected,
                                    const research_interface::robot::ControllerCommand& actual);
void testRobotCommandsAreEqual(const research_interface::robot::RobotCommand& expected,
                               const research_interface::robot::RobotCommand& actual);
void testRobotCommandsAreEqual(const research_interface::robot::RobotCommand& expected,
                               const franka::RobotCommand actual);
std::array<double, 16> identityMatrix();
franka::RobotState generateValidRobotState();
std::array<double, 6> differentiateOneSample(std::array<double, 16> value,
                                             std::array<double, 16> last_value,
                                             double delta_t);

namespace research_interface {
namespace robot {

bool operator==(const Move::Deviation& left, const Move::Deviation& right);
}  // namespace robot
}  // namespace research_interface

namespace franka {

bool operator==(const Errors& lhs, const Errors& rhs);
bool operator==(const RobotState& lhs, const RobotState& rhs);

}  // namespace franka
