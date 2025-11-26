// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_model.h"
#include "gtest/gtest.h"

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

#include <franka/model.h>
#include "test_utils.h"

// Constants for test tolerance
constexpr double kJacobianTolerance = 1e-3;
constexpr double kDiffTol = 1e-5;

class RobotJacobianTest : public ::testing::Test {
 protected:
  std::unique_ptr<franka::RobotModel> model;
  std::array<double, 7> zero_joint_configuration{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> default_joint_configuration{0, 0, 0, -0.75 * M_PI, 0, 0.75 * M_PI, 0};
  std::array<double, 7> moved_joint_configuration{0.0010, 0.0010, 0.0010, -2.3552,
                                                  0.0010, 2.3572, 0.0010};
  std::array<double, 7> random_joint_configuration{0.5, -0.3, 0.2, -1.2, 0.4, 1.5, -0.2};
  std::array<double, 16> identity_ee{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 16> identity_stiffness{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 16> rotated_ee{0, -1, 0, 0.1, 1, 0, 0, 0.2, 0, 0, 1, 0.3, 0, 0, 0, 1};
  std::array<double, 16> rotated_stiffness{0,  0, 1, 0.05, 0, 1, 0, 0.15,
                                           -1, 0, 0, 0.25, 0, 0, 0, 1};

  // Expected zero Jacobian for default_joint_configuration {0, 0, 0, -0.75*M_PI, 0, 0.75*M_PI, 0}
  // This value is the same for Flange, EE (with identity transform), and Stiffness (with identity
  // transforms) frames, as well as with rotated transforms (the zero Jacobian is expressed in base
  // frame, so end effector transforms don't affect it)
  const std::array<double, 42> expected_default_zero_jacobian = {
      -3.300438565e-16,
      0.5003653134,
      0,
      0,
      0,
      1,  // Col 1
      -0.004192694528,
      -2.222068367e-16,
      -0.5003653134,
      0,
      1,
      -4.440892099e-16,  // Col 2
      -1.897116662e-16,
      0.5003653134,
      0,
      0,
      0,
      1,  // Col 3
      0.3201926945,
      -1.855694769e-16,
      0.4178653134,
      0,
      -1,
      -4.440892099e-16,  // Col 4
      -1.391756684e-17,
      0.01343502884,
      4.86479492e-19,
      0.7071067812,
      7.581077016e-16,
      -0.7071067812,  // Col 5
      0.107,
      -3.907985047e-17,
      0.088,
      0,
      -1,
      -4.440892099e-16,  // Col 6
      0,
      0,
      0,
      0,
      8.881784197e-16,
      -1  // Col 7
  };

  void SetUp() override {
    std::string urdf_path = franka_test_utils::getUrdfPath(__FILE__);
    auto urdf_string = franka_test_utils::readFileToString(urdf_path);

    model = std::make_unique<franka::RobotModel>(urdf_string);
  }

  // Helper function to compare Jacobian matrices
  void compareJacobians(const std::array<double, 42>& actual,
                        const std::array<double, 42>& expected,
                        double tolerance = kJacobianTolerance) {
    for (size_t i = 0; i < expected.size(); ++i) {
      ASSERT_NEAR(actual[i], expected[i], tolerance);
    }
  }

  // Helper function for bodyJacobian tests
  void testBodyJacobian(const std::array<double, 7>& config,
                        int joint_index,
                        const std::array<double, 42>& expected) {
    std::array<double, 42> actual = model->bodyJacobian(config, joint_index);
    compareJacobians(actual, expected);
  }

  // Helper function for bodyJacobianEe tests
  void testBodyJacobianEe(const std::array<double, 7>& config,
                          const std::array<double, 16>& ee_transform,
                          const std::array<double, 42>& expected) {
    std::array<double, 42> actual = model->bodyJacobianEe(config, ee_transform);
    compareJacobians(actual, expected);
  }

  // Helper function for bodyJacobianStiffness tests
  void testBodyJacobianStiffness(const std::array<double, 7>& config,
                                 const std::array<double, 16>& ee_transform,
                                 const std::array<double, 16>& stiffness_transform,
                                 const std::array<double, 42>& expected) {
    std::array<double, 42> actual =
        model->bodyJacobianStiffness(config, ee_transform, stiffness_transform);
    compareJacobians(actual, expected);
  }

  // Helper function for zeroJacobian tests
  void testZeroJacobian(const std::array<double, 7>& config,
                        int joint_index,
                        const std::array<double, 42>& expected) {
    std::array<double, 42> actual = model->zeroJacobian(config, joint_index);
    compareJacobians(actual, expected);
  }

  // Helper function for zeroJacobianEe tests
  void testZeroJacobianEe(const std::array<double, 7>& config,
                          const std::array<double, 16>& ee_transform,
                          const std::array<double, 42>& expected) {
    std::array<double, 42> actual = model->zeroJacobianEe(config, ee_transform);
    compareJacobians(actual, expected);
  }

  // Helper function for zeroJacobianStiffness tests
  void testZeroJacobianStiffness(const std::array<double, 7>& config,
                                 const std::array<double, 16>& ee_transform,
                                 const std::array<double, 16>& stiffness_transform,
                                 const std::array<double, 42>& expected) {
    std::array<double, 42> actual =
        model->zeroJacobianStiffness(config, ee_transform, stiffness_transform);
    compareJacobians(actual, expected);
  }
};

// Tests for bodyJacobian with different joint configurations and indices
TEST_F(RobotJacobianTest,
       GivenZeroJointConfiguration_WhenCalculatingBodyJacobianForJoint1_ThenResultMatchesExpected) {
  // Expected Body Jacobian for Joint1 at zero configuration (column-major format)
  std::array<double, 42> expected_jacobian = {
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000,  // Col 1
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 2
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 3
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 4
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 5
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000   // Col 6
  };

  testBodyJacobian(zero_joint_configuration, 1, expected_jacobian);
}

TEST_F(
    RobotJacobianTest,
    GivenDefaultJointConfiguration_WhenCalculatingBodyJacobianForJoint4_ThenResultMatchesExpected) {
  // Expected Body Jacobian for Joint4 at default configuration (column-major format)
  std::array<double, 42> expected_jacobian = {
      0.000000,  -0.000000, -0.082500, -0.707107, -0.707107, -0.000000, -0.165251,
      0.281923,  0.000000,  0.000000,  -0.000000, -1.000000, 0.000000,  -0.000000,
      -0.082500, -0.707107, -0.707107, 0.000000,  0.000000,  0.000000,  0.000000,
      0.000000,  0.000000,  1.000000,  0.000000,  0.000000,  0.000000,  0.000000,
      0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,
      0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000};

  testBodyJacobian(default_joint_configuration, 4, expected_jacobian);
}

// Tests for bodyJacobianEe with different joint configurations and EE transforms
TEST_F(
    RobotJacobianTest,
    GivenZeroJointConfiguration_WhenCalculatingBodyJacobianWithIdentityEE_ThenResultMatchesExpected) {
  std::array<double, 42> expected_jacobian = {
      0.000000,  -0.088000, 0.000000,  0.000000,  0.000000,  -1.000000, 0.593400,
      -0.000000, 0.088000,  0.000000,  -1.000000, 0.000000,  0.000000,  -0.088000,
      0.000000,  0.000000,  0.000000,  -1.000000, -0.277200, 0.000000,  -0.005500,
      0.000000,  1.000000,  0.000000,  0.000000,  -0.088000, 0.000000,  0.000000,
      0.000000,  -1.000000, 0.107000,  0.000000,  -0.088000, 0.000000,  1.000000,
      -0.000000, 0.000000,  -0.000000, 0.000000,  0.000000,  0.000000,  1.000000};

  testBodyJacobianEe(zero_joint_configuration, identity_ee, expected_jacobian);
}

TEST_F(
    RobotJacobianTest,
    GivenDefaultJointConfiguration_WhenCalculatingBodyJacobianWithRotatedEE_ThenResultMatchesExpected) {
  std::array<double, 42> expected_jacobian = {
      0.500507,  0.000000,  0.000000, 0.000000,  0.000000,  -1.000000, -0.000000,
      -0.004134, 0.500507,  1.000000, 0.000000,  0.000000,  0.500507,  0.000000,
      0.000000,  0.000000,  0.000000, -1.000000, 0.000000,  0.320334,  -0.418007,
      -1.000000, 0.000000,  0.000000, 0.013435,  0.000000,  0.000000,  -0.000000,
      0.707107,  0.707107,  0.000000, 0.107000,  -0.088000, -1.000000, -0.000000,
      0.000000,  -0.000000, 0.000000, 0.000000,  0.000000,  -0.000000, 1.000000};

  testBodyJacobianEe(default_joint_configuration, rotated_ee, expected_jacobian);
}

// Tests for bodyJacobianStiffness with different joint configurations and stiffness transforms
TEST_F(
    RobotJacobianTest,
    GivenMovedJointConfiguration_WhenCalculatingBodyJacobianStiffnessWithIdentityFrames_ThenResultMatchesExpected) {
  std::array<double, 42> expected_jacobian = {
      -0.000869, -0.500928, 0.000354,  0.000999,  -0.000709, -0.999999, -0.004631,
      0.000358,  0.500924,  -0.000708, -0.999999, 0.000708,  -0.000869, -0.500932,
      0.000354,  0.001999,  -0.000710, -0.999998, 0.320665,  -0.000843, -0.417792,
      0.001708,  0.999998,  -0.000706, -0.000013, -0.013296, 0.000000,  0.706395,
      -0.000706, 0.707817,  0.107000,  -0.000107, -0.088000, 0.001000,  1.000000,
      0.000000,  0.000000,  0.000000,  -0.000000, -0.000000, 0.000000,  1.000000};

  testBodyJacobianStiffness(moved_joint_configuration, identity_ee, identity_stiffness,
                            expected_jacobian);
}

TEST_F(
    RobotJacobianTest,
    GivenRandomJointConfiguration_WhenCalculatingBodyJacobianStiffnessWithRotatedFrames_ThenResultMatchesExpected) {
  std::array<double, 42> expected_jacobian = {
      0.085067,  0.029850,  -0.363308, -0.788034, 0.600610,  -0.135169, 0.530170,
      0.208236,  0.381096,  0.433742,  0.385847,  -0.814242, 0.191471,  0.069989,
      -0.483803, -0.881947, 0.366838,  -0.295973, -0.428203, 0.027333,  -0.178988,
      -0.388443, -0.209984, 0.897228,  0.000000,  0.022441,  -0.110705, -0.070737,
      0.977611,  0.198172,  -0.088000, 0.104867,  0.021258,  -0.000000, -0.198669,
      0.980067,  -0.000000, 0.000000,  0.000000,  1.000000,  0.000000,  -0.000000};

  testBodyJacobianStiffness(random_joint_configuration, rotated_ee, rotated_stiffness,
                            expected_jacobian);
}

// Tests for zeroJacobian with different joint configurations and indices
TEST_F(RobotJacobianTest,
       GivenZeroJointConfiguration_WhenCalculatingZeroJacobianForJoint1_ThenResultMatchesExpected) {
  std::array<double, 42> expected_jacobian = {
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000,  // Col 1
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 2
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 3
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 4
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  // Col 5
      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000   // Col 6
  };

  testZeroJacobian(zero_joint_configuration, 1, expected_jacobian);
}

TEST_F(
    RobotJacobianTest,
    GivenDefaultJointConfiguration_WhenCalculatingZeroJacobianForJoint4_ThenResultMatchesExpected) {
  // Expected Zero Jacobian for Joint4 at default configuration (column-major format)
  std::array<double, 42> expected_jacobian = {
      -0.000000, 0.082500, 0.000000, 0.000000,  0.000000,  1.000000,  0.316200, 0.000000, -0.082500,
      -0.000000, 1.000000, 0.000000, -0.000000, 0.082500,  0.000000,  0.000000, 0.000000, 1.000000,
      0.000000,  0.000000, 0.000000, 0.000000,  -1.000000, -0.000000, 0.000000, 0.000000, 0.000000,
      0.000000,  0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000, 0.000000, 0.000000,
      0.000000,  0.000000, 0.000000, 0.000000,  0.000000,  0.000000};

  testZeroJacobian(default_joint_configuration, 4, expected_jacobian);
}

// Tests for zeroJacobianEe with different joint configurations and EE transforms
TEST_F(
    RobotJacobianTest,
    GivenMovedJointConfiguration_WhenCalculatingZeroJacobianWithIdentityEE_ThenResultMatchesExpected) {
  // Expected Zero Jacobian for EE at moved configuration with identity EE transform (column-major
  // format)
  std::array<double, 42> expected_jacobian = {
      -0.001015, 0.500928,  0.000000,  0.000000,  0.000000, 1.000000,  -0.004131,
      -0.000004, -0.500928, -0.001000, 1.000000,  0.000000, -0.001015, 0.500932,
      0.000001,  0.001000,  0.000001,  1.000000,  0.320247, 0.000640,  0.418113,
      0.002000,  -0.999998, -0.000001, -0.000017, 0.013296, 0.000009,  0.707101,
      0.001415,  -0.707111, 0.106912,  0.000076,  0.088107, 0.001293,  -0.999999,
      -0.000708, 0.000000,  -0.000000, 0.000000,  0.000999, 0.000709,  -0.999999};

  testZeroJacobianEe(moved_joint_configuration, identity_ee, expected_jacobian);
}

TEST_F(
    RobotJacobianTest,
    GivenRandomJointConfiguration_WhenCalculatingZeroJacobianWithRotatedEE_ThenResultMatchesExpected) {
  std::array<double, 42> expected_jacobian = {
      -0.294080, 0.231598,  0.000000,  0.000000,  0.000000,  1.000000,  0.520058,
      0.284109,  -0.344236, -0.479426, 0.877583,  0.000000,  -0.364905, 0.374942,
      -0.043455, -0.259343, -0.141680, 0.955336,  -0.221942, -0.154799, 0.378048,
      0.636431,  -0.769096, 0.058711,  -0.088426, 0.064274,  0.028442,  0.583084,
      0.529537,  0.616120,  0.048438,  0.009352,  0.129458,  0.782826,  -0.569018,
      -0.251797, 0.000000,  0.000000,  0.000000,  0.175457,  0.590099,  -0.788034};

  testZeroJacobianEe(random_joint_configuration, rotated_ee, expected_jacobian);
}

// Tests for zeroJacobianStiffness with different joint configurations and stiffness transforms
TEST_F(
    RobotJacobianTest,
    GivenZeroJointConfiguration_WhenCalculatingZeroJacobianStiffnessWithIdentityFrames_ThenResultMatchesExpected) {
  // Expected Zero Jacobian for stiffness frame at zero config with identity transforms
  // (column-major format)
  std::array<double, 42> expected_jacobian = {
      0.000000,  0.088000,  0.000000,  0.000000, 0.000000,  1.000000,  0.593400,
      0.000000,  -0.088000, -0.000000, 1.000000, 0.000000,  0.000000,  0.088000,
      0.000000,  0.000000,  0.000000,  1.000000, -0.277200, -0.000000, 0.005500,
      0.000000,  -1.000000, -0.000000, 0.000000, 0.088000,  0.000000,  0.000000,
      -0.000000, 1.000000,  0.107000,  0.000000, 0.088000,  0.000000,  -1.000000,
      0.000000,  0.000000,  0.000000,  0.000000, 0.000000,  0.000000,  -1.000000};

  testZeroJacobianStiffness(zero_joint_configuration, identity_ee, identity_stiffness,
                            expected_jacobian);
}

TEST_F(
    RobotJacobianTest,
    GivenDefaultJointConfiguration_WhenCalculatingZeroJacobianStiffnessWithRotatedFrames_ThenResultMatchesExpected) {
  // Expected Zero Jacobian for stiffness frame at default config with rotated transforms
  // (column-major format)
  std::array<double, 42> expected_jacobian = {
      0.000000,  0.500507,  0.000000,  0.000000,  0.000000,  1.000000,  -0.004134,
      -0.000000, -0.500507, -0.000000, 1.000000,  0.000000,  -0.000000, 0.500507,
      0.000000,  0.000000,  0.000000,  1.000000,  0.320334,  0.000000,  0.418007,
      0.000000,  -1.000000, -0.000000, 0.000000,  0.013435,  0.000000,  0.707107,
      -0.000000, -0.707107, 0.107000,  -0.000000, 0.088000,  0.000000,  -1.000000,
      -0.000000, 0.000000,  0.000000,  0.000000,  -0.000000, 0.000000,  -1.000000};

  testZeroJacobianStiffness(default_joint_configuration, rotated_ee, rotated_stiffness,
                            expected_jacobian);
}

// BUG REPRODUCTION TESTS
// These tests verify that zeroJacobian functions return correct values for
// kFlange, kEndEffector, and kStiffness frames
TEST_F(RobotJacobianTest, ZeroJacobianFlangeReturnsCorrectValues) {
  std::array<double, 42> jacobian = model->zeroJacobianFlange(default_joint_configuration);
  compareJacobians(jacobian, expected_default_zero_jacobian);
}

TEST_F(RobotJacobianTest, ZeroJacobianEeReturnsCorrectValues) {
  std::array<double, 42> jacobian = model->zeroJacobianEe(default_joint_configuration, identity_ee);
  compareJacobians(jacobian, expected_default_zero_jacobian);
}

TEST_F(RobotJacobianTest, ZeroJacobianStiffnessReturnsCorrectValues) {
  std::array<double, 42> jacobian =
      model->zeroJacobianStiffness(default_joint_configuration, identity_ee, identity_stiffness);
  compareJacobians(jacobian, expected_default_zero_jacobian);
}

TEST_F(RobotJacobianTest, ZeroJacobianEeWithRotatedFrameReturnsCorrectValues) {
  // Zero Jacobian is expressed in base frame, so rotated EE transform doesn't affect it
  std::array<double, 42> jacobian = model->zeroJacobianEe(default_joint_configuration, rotated_ee);
  compareJacobians(jacobian, expected_default_zero_jacobian);
}

TEST_F(RobotJacobianTest, ZeroJacobianStiffnessWithRotatedFramesReturnsCorrectValues) {
  // Zero Jacobian is expressed in base frame, so rotated transforms don't affect it
  std::array<double, 42> jacobian =
      model->zeroJacobianStiffness(default_joint_configuration, rotated_ee, rotated_stiffness);
  compareJacobians(jacobian, expected_default_zero_jacobian);
}
