// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_state.h"

#include <algorithm>
#include <cstring>
#include <iterator>

namespace franka {

namespace {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

template <class T, size_t M, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<std::array<T, M>, N>& array) {
  ostream << "[";
  // TODO (wink_ma): check if for loop can be replaced using iterators
  for (size_t i = 0; i < N; ++i) {
    if (i > 0) {
      ostream << ", ";
    }
    ostream << "[";
    std::copy(array[i].cbegin(), array[i].cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array[i].cend() - 1, array[i].cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
  }
  ostream << "]";
  return ostream;
}

}  // anonymous namespace

std::ostream& operator<<(std::ostream& ostream, const RobotMode robot_mode) {
  ostream << "\"";
  switch (robot_mode) {
    case (RobotMode::kUserStopped):
      ostream << "User stopped";
      break;
    case (RobotMode::kIdle):
      ostream << "Idle";
      break;
    case (RobotMode::kMove):
      ostream << "Move";
      break;
    case (RobotMode::kGuiding):
      ostream << "Guiding";
      break;
    case (RobotMode::kReflex):
      ostream << "Reflex";
      break;
    case (RobotMode::kAutomaticErrorRecovery):
      ostream << "Automatic error recovery";
      break;
    case (RobotMode::kOther):
      ostream << "Other";
      break;
  }
  ostream << "\"";
  return ostream;
}

std::ostream& operator<<(std::ostream& ostream, const franka::RobotState& robot_state) {
  ostream << "{\"O_T_EE\": " << robot_state.O_T_EE << ", \"O_T_EE_d\": " << robot_state.O_T_EE_d
          << ", \"F_T_NE\": " << robot_state.F_T_NE << ", \"NE_T_EE\": " << robot_state.NE_T_EE
          << ", \"F_T_EE\": " << robot_state.F_T_EE << ", \"EE_T_K\": " << robot_state.EE_T_K
          << ", \"m_ee\": " << robot_state.m_ee << ", \"F_x_Cee\": " << robot_state.F_x_Cee
          << ", \"I_ee\": " << robot_state.I_ee << ", \"m_load\": " << robot_state.m_load
          << ", \"F_x_Cload\": " << robot_state.F_x_Cload << ", \"I_load\": " << robot_state.I_load
          << ", \"m_total\": " << robot_state.m_total
          << ", \"F_x_Ctotal\": " << robot_state.F_x_Ctotal
          << ", \"I_total\": " << robot_state.I_total << ", \"elbow\": " << robot_state.elbow
          << ", \"elbow_d\": " << robot_state.elbow_d << ", \"elbow_c\": " << robot_state.elbow_c
          << ", \"delbow_c\": " << robot_state.delbow_c
          << ", \"ddelbow_c\": " << robot_state.ddelbow_c << ", \"tau_J\": " << robot_state.tau_J
          << ", \"tau_J_d\": " << robot_state.tau_J_d << ", \"dtau_J\": " << robot_state.dtau_J
          << ", \"q\": " << robot_state.q << ", \"dq\": " << robot_state.dq
          << ", \"q_d\": " << robot_state.q_d << ", \"dq_d\": " << robot_state.dq_d
          << ", \"ddq_d\": " << robot_state.ddq_d
          << ", \"joint_contact\": " << robot_state.joint_contact
          << ", \"cartesian_contact\": " << robot_state.cartesian_contact
          << ", \"joint_collision\": " << robot_state.joint_collision
          << ", \"cartesian_collision\": " << robot_state.cartesian_collision
          << ", \"tau_ext_hat_filtered\": " << robot_state.tau_ext_hat_filtered
          << ", \"O_F_ext_hat_K\": " << robot_state.O_F_ext_hat_K
          << ", \"K_F_ext_hat_K\": " << robot_state.K_F_ext_hat_K
          << ", \"O_dP_EE_d\": " << robot_state.O_dP_EE_d
          << ", \"O_ddP_O\": " << robot_state.O_ddP_O << ", \"O_T_EE_c\": " << robot_state.O_T_EE_c
          << ", \"O_dP_EE_c\": " << robot_state.O_dP_EE_c
          << ", \"O_ddP_EE_c\": " << robot_state.O_ddP_EE_c << ", \"theta\": " << robot_state.theta
          << ", \"dtheta\": " << robot_state.dtheta
          << ", \"accelerometer_top\": " << robot_state.accelerometer_top
          << ", \"accelerometer_bottom\": " << robot_state.accelerometer_bottom
          << ", \"current_errors\": " << robot_state.current_errors
          << ", \"last_motion_errors\": " << robot_state.last_motion_errors
          << ", \"control_command_success_rate\": " << robot_state.control_command_success_rate
          << ", \"robot_mode\": " << robot_state.robot_mode
          << ", \"time\": " << robot_state.time.toMSec() << "}";
  return ostream;
}

}  // namespace franka
