// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_model.h"
#include "gtest/gtest.h"
#include "test_utils.h"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>

constexpr double kDeltaCoriolis = 0.2;
constexpr double kDeltaInertia = 0.1;
constexpr double kDeltaGravity = 0.2;

constexpr double kDeltaGravityDot = 0.4;
constexpr double kDeltaInertiaDot = 0.03;
constexpr double kDeltaCoriolisDot = 0.3;

class RobotModelTest : public ::testing::Test {
 protected:
  std::unique_ptr<franka::RobotModel> model;
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> dq = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> unit_dq = {1, 1, 1, 1, 1, 1, 1};
  std::array<double, 9> i_total = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  std::array<double, 3> f_x_ctotal = {0, 0, 0};
  double m_total = 0;
  std::array<double, 3> g_earth = {0, 0, -9.81};
  std::array<double, 7> default_joint_configuration{0, 0, 0, -0.75 * M_PI, 0, 0.75 * M_PI, 0};
  std::array<double, 7> moved_joint_configuration{0.0010, 0.0010, 0.0010, -2.3552,
                                                  0.0010, 2.3572, 0.0010};

  // Franka end-effector mechanical properties
  std::array<double, 9> ee_i_total = {0.001, 0, 0, 0, 0.0025, 0, 0, 0, 0.0017};
  std::array<double, 3> ee_f_x_ctotal = {0.01, 0, 0.03};
  double ee_m_total = 0.73;

  RobotModelTest() {
    std::string urdf_path = franka_test_utils::getUrdfPath(__FILE__);
    auto urdf_string = franka_test_utils::readFileToString(urdf_path);
    model = std::make_unique<franka::RobotModel>(urdf_string);
  }
};

TEST_F(RobotModelTest, TestCoriolis) {
  std::array<double, 7> c_ne;

  model->coriolis(q, dq, i_total, m_total, f_x_ctotal, c_ne);

  for (double i : c_ne) {
    ASSERT_EQ(i, 0.0);
  }
}

TEST_F(RobotModelTest, TestCoriolisWithGravity) {
  std::array<double, 7> c_ne;

  model->coriolis(q, dq, i_total, m_total, f_x_ctotal, g_earth, c_ne);

  for (double i : c_ne) {
    ASSERT_EQ(i, 0.0);
  }
}

TEST_F(RobotModelTest, TestGravity) {
  std::array<double, 3> g_earth = {0, 0, -9.81};
  std::array<double, 7> g_ne;

  model->gravity(q, g_earth, m_total, f_x_ctotal, g_ne);

  Eigen::Matrix<double, 7, 1> expected_gravity;
  expected_gravity << 0, -3.52387, 0, -3.44254, 0, 1.63362, -2.47128e-17;

  for (int i = 0; i < expected_gravity.rows(); i++) {
    ASSERT_NEAR(g_ne[i], expected_gravity(i), franka_test_utils::kEps);
  }
}

TEST_F(RobotModelTest, TestMass) {
  std::array<double, 49> m_ne;

  model->mass(q, i_total, m_total, f_x_ctotal, m_ne);

  Eigen::Matrix<double, 7, 7> expected_mass_matrix;

  expected_mass_matrix << 0.132725, -0.0414226, 0.10001, 0.0120435, 0.0446215, -0.0022471,
      -0.000369021, -0.0414226, 2.73892, -0.0437247, -1.15444, -0.0372567, 0.0137625, -0.00396178,
      0.10001, -0.0437247, 0.10001, 0.0120435, 0.0446215, -0.0022471, -0.000369021, 0.0120435,
      -1.15444, 0.0120435, 0.644215, 0.0116149, -0.00714566, 0.00225227, 0.0446215, -0.0372567,
      0.0446215, 0.0116149, 0.0446215, -0.0022471, -0.000369021, -0.0022471, 0.0137625, -0.0022471,
      -0.00714566, -0.0022471, 0.0313282, 0.000174882, -0.000369021, -0.00396178, -0.000369021,
      0.00225227, -0.000369021, 0.000174882, 0.000119426;

  for (int i = 0; i < expected_mass_matrix.size(); i++) {
    ASSERT_NEAR(m_ne[i], expected_mass_matrix(i), franka_test_utils::kEps);
  }
}

TEST_F(RobotModelTest, TestCoriolisWithAddedFrankaEndEffectorInertiaToLastLink) {
  std::array<double, 7> c_ne;

  model->coriolis(q, unit_dq, ee_i_total, ee_m_total, ee_f_x_ctotal, c_ne);

  Eigen::Matrix<double, 7, 1> expected_coriolis;
  expected_coriolis << 0.25211, -1.97502, 0.254311, 0.987115, 0.122285, -0.256464, -0.000634562;

  for (int i = 0; i < expected_coriolis.rows(); i++) {
    ASSERT_NEAR(c_ne[i], expected_coriolis(i), franka_test_utils::kEps);
  }
}

TEST_F(RobotModelTest, TestGravityWithAddedFrankaEndEffectorInertiaToLastLink) {
  std::array<double, 7> g_ne;

  model->gravity(q, g_earth, ee_m_total, ee_f_x_ctotal, g_ne);

  Eigen::Matrix<double, 7, 1> expected_gravity;
  expected_gravity << 0, -4.22568, 0, -3.33154, 0, 2.33543, -8.83179e-17;

  for (int i = 0; i < expected_gravity.rows(); i++) {
    ASSERT_NEAR(g_ne[i], expected_gravity(i), franka_test_utils::kEps);
  }
}

TEST_F(RobotModelTest, TestMassWithAddedFrankaEndEffectorInertiaToLastLink) {
  std::array<double, 49> m_ne;

  model->mass(q, ee_i_total, ee_m_total, ee_f_x_ctotal, m_ne);

  Eigen::Matrix<double, 7, 7> expected_mass_matrix;
  expected_mass_matrix << 0.141436, -0.0414226, 0.108721, 0.0120435, 0.0533324, -0.0022471,
      -0.00278442, -0.0414226, 2.97982, -0.0437247, -1.25956, -0.0372567, 0.0605572, -0.00396178,
      0.108721, -0.0437247, 0.108721, 0.0120435, 0.0533324, -0.0022471, -0.00278442, 0.0120435,
      -1.25956, 0.0120435, 0.691427, 0.0116149, -0.0282393, 0.00225227, 0.0533324, -0.0372567,
      0.0533324, 0.0116149, 0.0533324, -0.0022471, -0.00278442, -0.0022471, 0.0605572, -0.0022471,
      -0.0282393, -0.0022471, 0.0545405, 0.000174882, -0.00278442, -0.00396178, -0.00278442,
      0.00225227, -0.00278442, 0.000174882, 0.00189243;

  for (int i = 0; i < expected_mass_matrix.size(); i++) {
    ASSERT_NEAR(m_ne[i], expected_mass_matrix(i), franka_test_utils::kEps);
  }
}

TEST_F(RobotModelTest, givenNoLoad_whenComputingGravity_thenCloseToBaseline) {
  std::array<double, 7> test_gravity;

  auto expected_gravity = std::array<double, 7>{0, -24.5858, 0, 17.6880, 0.5095, 1.6428, 0};

  model->gravity(default_joint_configuration, g_earth, m_total, f_x_ctotal, test_gravity);

  for (size_t i = 0; i < test_gravity.size(); ++i) {
    ASSERT_NEAR(test_gravity[i], expected_gravity[i], kDeltaGravity) << "Joint " << i;
  }
}

TEST_F(RobotModelTest, givenLoad_whenComputingGravity_thenCloseToBaseline) {
  std::array<double, 7> test_gravity;

  auto expected_gravity = std::array<double, 7>{0, -28.2417, 0, 20.7531, 0.5095, 2.3446, 0};
  model->gravity(moved_joint_configuration, g_earth, ee_m_total, ee_f_x_ctotal, test_gravity);

  for (size_t i = 0; i < test_gravity.size(); ++i) {
    ASSERT_NEAR(test_gravity[i], expected_gravity[i], kDeltaGravity) << "Joint " << i;
  }
}

TEST_F(RobotModelTest, givenNonZeroJointVelocity_whenComputingCoriolis_thenNonZeroCoriolis) {
  auto expected_coriolis =
      std::array<double, 7>{2.4562, -0.8199, 2.4532, -2.3450, -0.1492, -0.1801, 0.0164};

  std::array<double, 7> test_coriolis;
  model->coriolis(default_joint_configuration, unit_dq, i_total, m_total, f_x_ctotal,
                  test_coriolis);

  for (size_t i = 0; i < test_coriolis.size(); ++i) {
    ASSERT_NEAR(test_coriolis[i], expected_coriolis[i], kDeltaCoriolis);
  }
}

TEST_F(RobotModelTest,
       givenNonZeroJointVelocityWithLoad_whenComputingCoriolis_thenCloseToBaseline) {
  std::array<double, 7> test_coriolis;

  auto expected_coriolis =
      std::array<double, 7>{3.1468, -0.6225, 3.1439, -3.1063, -0.0658, -0.5416, 0.0089};

  model->coriolis(moved_joint_configuration, unit_dq, ee_i_total, ee_m_total, ee_f_x_ctotal,
                  test_coriolis);
  for (size_t i = 0; i < test_coriolis.size(); ++i) {
    ASSERT_NEAR(test_coriolis[i], expected_coriolis[i], kDeltaCoriolis);
  }
}

TEST_F(RobotModelTest, givenWithLoad_whenComputingInertia_thenCloseToBaseline) {
  std::array<double, 7 * 7> test_inertia;

  auto expected_inertia = std::array<double, 7 * 7>{
      1.1749,  -0.0085, 1.1660,  -0.0247, -0.0261, -0.0033, -0.0057, -0.0085, 1.7281,  0.0046,
      -0.8304, -0.0142, -0.1209, -0.0005, 1.1660,  0.0046,  1.1660,  -0.0247, -0.0261, -0.0033,
      -0.0057, -0.0247, -0.8304, -0.0247, 1.0399,  0.0263,  0.1526,  -0.0023, -0.0261, -0.0142,
      -0.0261, 0.0263,  0.0112,  0.0042,  0.0012,  -0.0033, -0.1209, -0.0033, 0.1526,  0.0042,
      0.0390,  -0.0004, -0.0057, -0.0005, -0.0057, -0.0023, 0.0012,  -0.0004, 0.0019};

  model->mass(moved_joint_configuration, ee_i_total, ee_m_total, ee_f_x_ctotal, test_inertia);

  for (size_t i = 0; i < test_inertia.size(); ++i) {
    ASSERT_NEAR(test_inertia[i], expected_inertia[i], kDeltaInertia);
  }
}

TEST_F(RobotModelTest, givenNoLoad_whenComputingInertia_thenCloseToBaseline) {
  std::array<double, 7 * 7> test_inertia;

  auto expected_inertia = std::array<double, 7 * 7>{
      0.9831,  -0.0078, 0.9742,  -0.0254, -0.0351, -0.0040, -0.0004, -0.0078, 1.5390,  0.0053,
      -0.6641, -0.0147, -0.0828, -0.0012, 0.9742,  0.0053,  0.9742,  -0.0254, -0.0351, -0.0040,
      -0.0004, -0.0254, -0.6641, -0.0254, 0.8185,  0.0268,  0.0888,  -0.0016, -0.0351, -0.0147,
      -0.0351, 0.0268,  0.0114,  0.0047,  0.0002,  -0.0040, -0.0828, -0.0040, 0.0888,  0.0047,
      0.0202,  0.0003,  -0.0004, -0.0012, -0.0004, -0.0016, 0.0002,  0.0003,  0.0002};

  model->mass(default_joint_configuration, i_total, m_total, f_x_ctotal, test_inertia);
  for (size_t i = 0; i < test_inertia.size(); ++i) {
    ASSERT_NEAR(test_inertia[i], expected_inertia[i], kDeltaInertia);
  }
}
