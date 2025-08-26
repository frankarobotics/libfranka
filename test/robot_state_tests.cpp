// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <sstream>

#include <gtest/gtest.h>

#include <franka/robot_state.h>

#include "helpers.h"
#include "robot_impl.h"

using namespace franka;

TEST(RobotState, IsInitializedToZero) {
  RobotState robot_state;

  testRobotStateIsZero(robot_state);
}

TEST(RobotState, CanBeStreamed) {
  RobotState robot_state;

  std::stringstream ss;
  ss << robot_state;
  std::string output(ss.str());

  EXPECT_PRED2(stringContains, output, "O_T_EE");
  EXPECT_PRED2(stringContains, output, "F_T_NE");
  EXPECT_PRED2(stringContains, output, "NE_T_EE");
  EXPECT_PRED2(stringContains, output, "O_T_EE_d");
  EXPECT_PRED2(stringContains, output, "O_T_EE_c");
  EXPECT_PRED2(stringContains, output, "EE_T_K");
  EXPECT_PRED2(stringContains, output, "F_T_EE");
  EXPECT_PRED2(stringContains, output, "m_ee");
  EXPECT_PRED2(stringContains, output, "F_x_Cee");
  EXPECT_PRED2(stringContains, output, "I_ee");
  EXPECT_PRED2(stringContains, output, "m_load");
  EXPECT_PRED2(stringContains, output, "F_x_Cload");
  EXPECT_PRED2(stringContains, output, "I_load");
  EXPECT_PRED2(stringContains, output, "m_total");
  EXPECT_PRED2(stringContains, output, "F_x_Ctotal");
  EXPECT_PRED2(stringContains, output, "I_total");
  EXPECT_PRED2(stringContains, output, "elbow");
  EXPECT_PRED2(stringContains, output, "elbow_d");
  EXPECT_PRED2(stringContains, output, "elbow_c");
  EXPECT_PRED2(stringContains, output, "delbow_c");
  EXPECT_PRED2(stringContains, output, "ddelbow_c");
  EXPECT_PRED2(stringContains, output, "tau_J");
  EXPECT_PRED2(stringContains, output, "tau_J_d");
  EXPECT_PRED2(stringContains, output, "dtau_J");
  EXPECT_PRED2(stringContains, output, "q");
  EXPECT_PRED2(stringContains, output, "dq");
  EXPECT_PRED2(stringContains, output, "q_d");
  EXPECT_PRED2(stringContains, output, "dq_d");
  EXPECT_PRED2(stringContains, output, "ddq_d");
  EXPECT_PRED2(stringContains, output, "joint_contact");
  EXPECT_PRED2(stringContains, output, "cartesian_contact");
  EXPECT_PRED2(stringContains, output, "joint_collision");
  EXPECT_PRED2(stringContains, output, "cartesian_collision");
  EXPECT_PRED2(stringContains, output, "tau_ext_hat_filtered");
  EXPECT_PRED2(stringContains, output, "O_F_ext_hat_K");
  EXPECT_PRED2(stringContains, output, "K_F_ext_hat_K");
  EXPECT_PRED2(stringContains, output, "O_dP_EE_d");
  EXPECT_PRED2(stringContains, output, "O_dP_EE_c");
  EXPECT_PRED2(stringContains, output, "O_ddP_EE_c");
  EXPECT_PRED2(stringContains, output, "theta");
  EXPECT_PRED2(stringContains, output, "dtheta");
  EXPECT_PRED2(stringContains, output, "accelerometer_top");
  EXPECT_PRED2(stringContains, output, "accelerometer_bottom");
  EXPECT_PRED2(stringContains, output, "current_errors");
  EXPECT_PRED2(stringContains, output, "last_motion_errors");
  EXPECT_PRED2(stringContains, output, "control_command_success_rate");
  EXPECT_PRED2(stringContains, output, "robot_mode");
  EXPECT_PRED2(stringContains, output, "time");
}

TEST(RobotState, CanCopyConstruct) {
  RobotState robot_state;
  randomRobotState(robot_state);

  RobotState robot_state2(robot_state);

  testRobotStatesAreEqual(robot_state, robot_state2);
}

TEST(RobotState, CanAssign) {
  RobotState robot_state;
  randomRobotState(robot_state);

  RobotState robot_state2;
  robot_state2 = robot_state;

  testRobotStatesAreEqual(robot_state, robot_state2);
}

TEST(RobotState, CanConvert) {
  research_interface::robot::RobotState original;
  randomRobotState(original);

  RobotState converted = convertRobotState(original);
  testRobotStatesAreEqual(original, converted);
}
