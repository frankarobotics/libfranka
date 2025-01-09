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
