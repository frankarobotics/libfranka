// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/logging/robot_state_log.hpp>

#include <iterator>
#include <sstream>

using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace franka {

namespace {

template <typename T, size_t N>
std::string csvName(const std::array<T, N>& /*unused*/, const std::string& name) {
  std::ostringstream ostream;
  for (size_t i = 0; i < N - 1; i++) {
    ostream << name << "[" << i << "],";
  }
  ostream << name << "[" << N - 1 << "]";
  return ostream.str();
}

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream /*unused*/, const std::array<T, N>& array) {
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  return ostream;
}

std::string csvRobotStateHeader() {
  RobotState robot_state;
  std::ostringstream ostream;
  ostream << "time,success_rate," << csvName(robot_state.q, "state.q") << ","
          << csvName(robot_state.q_d, "state.q_d") << "," << csvName(robot_state.dq, "state.dq")
          << "," << csvName(robot_state.dq_d, "state.dq_d") << ","
          << csvName(robot_state.tau_J, "state.tau_J") << ","
          << csvName(robot_state.tau_ext_hat_filtered, "state.tau_ext_hat_filtered");
  return ostream.str();
}

std::string csvRobotCommandHeader() {
  franka::RobotCommand command;
  std::ostringstream ostream;
  ostream << csvName(command.joint_positions.q, "cmd.q_d") << ","
          << csvName(command.joint_velocities.dq, "cmd.dq_d") << ","
          << csvName(command.cartesian_pose.O_T_EE, "cmd.O_T_EE_d") << ","
          << csvName(command.cartesian_velocities.O_dP_EE, "cmd.O_dP_EE_d") << ","
          << csvName(command.torques.tau_J, "cmd.tau_J_d");
  return ostream.str();
}

std::string csvLine(const franka::RobotState& robot_state) {
  std::ostringstream ostream;
  ostream << robot_state.time.toMSec() << "," << robot_state.control_command_success_rate << ","
          << robot_state.q << "," << robot_state.q_d << "," << robot_state.dq << ","
          << robot_state.dq_d << "," << robot_state.tau_J << ","
          << robot_state.tau_ext_hat_filtered;
  return ostream.str();
}

std::string csvLine(const franka::RobotCommand& command) {
  std::ostringstream ostream;
  ostream << command.joint_positions.q << "," << command.joint_velocities.dq << ","
          << command.cartesian_pose.O_T_EE << "," << command.cartesian_velocities.O_dP_EE << ","
          << command.torques.tau_J;
  return ostream.str();
}

}  // anonymous namespace

std::string logToCSV(const std::vector<Record>& log) {
  if (log.empty()) {
    return "";
  }
  std::ostringstream ostream;

  ostream << csvRobotStateHeader() << "," << csvRobotCommandHeader() << std::endl;
  for (const Record& record : log) {
    ostream << csvLine(record.state) << "," << csvLine(record.command) << std::endl;
  }

  return ostream.str();
}

}  // namespace franka
