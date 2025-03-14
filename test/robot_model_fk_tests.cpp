// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_model.h"
#include "gtest/gtest.h"
#include "test_utils.h"

#include <Eigen/Dense>

// Helper function to create rotation matrix around X axis
std::array<double, 16> rotationX(double angle) {
  double c = std::cos(angle);
  double s = std::sin(angle);
  return {1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1};
}

// Helper function to create rotation matrix around Y axis
std::array<double, 16> rotationY(double angle) {
  double c = std::cos(angle);
  double s = std::sin(angle);
  return {c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1};
}

// Helper function to create rotation matrix around Z axis
std::array<double, 16> rotationZ(double angle) {
  double c = std::cos(angle);
  double s = std::sin(angle);
  return {c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

class RobotModelFKTest : public ::testing::Test {
 protected:
  std::unique_ptr<franka::RobotModel> model;
  std::array<double, 7> zero_position = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> default_joint_configuration{0, 0, 0, -0.75 * M_PI, 0, 0.75 * M_PI, 0};
  std::array<double, 7> moved_joint_configuration{0.0010, 0.0010, 0.0010, -2.3552,
                                                  0.0010, 2.3572, 0.0010};
  std::array<double, 16> identity_transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // Rotated end-effector and stiffness frames (90 degrees around different axes)
  std::array<double, 16> F_T_EE_rotX = rotationX(M_PI / 2);  // 90 degrees around X
  std::array<double, 16> F_T_EE_rotY = rotationY(M_PI / 2);  // 90 degrees around Y
  std::array<double, 16> F_T_EE_rotZ = rotationZ(M_PI / 2);  // 90 degrees around Z

  // Add some translation to the stiffness frame
  std::array<double, 16> EE_T_K_translated = {1, 0, 0, 0.05,  // 5cm in X
                                              0, 1, 0, 0.03,  // 3cm in Y
                                              0, 0, 1, 0.02,  // 2cm in Z
                                              0, 0, 0, 1};

  RobotModelFKTest() {
    std::string urdf_path = franka_test_utils::getUrdfPath(__FILE__);
    auto urdf_string = franka_test_utils::readFileToString(urdf_path);
    model = std::make_unique<franka::RobotModel>(urdf_string);
  }

  // Helper method to verify transformation matrices
  void verifyTransform(const std::array<double, 16>& actual,
                       const std::array<double, 16>& expected,
                       double delta = franka_test_utils::kForwardKinematicsEps) {
    for (int i = 0; i < 16; i++) {
      EXPECT_NEAR(actual[i], expected[i], delta) << "Matrix differs at index " << i;
    }
  }

  // Helper to check if two transforms are different
  bool transformsAreDifferent(const std::array<double, 16>& a,
                              const std::array<double, 16>& b,
                              double threshold = franka_test_utils::kForwardKinematicsEps) {
    for (int i = 0; i < 16; i++) {
      if (std::abs(a[i] - b[i]) > threshold) {
        return true;
      }
    }
    return false;
  }

  // Helper to test a joint pose against expected values
  void testJointPose(const std::array<double, 7>& joint_config,
                     int joint_index,
                     const std::array<double, 16>& expected,
                     double delta = franka_test_utils::kForwardKinematicsEps,
                     const std::string& message = "") {
    std::array<double, 16> actual = model->pose(joint_config, joint_index);
    verifyTransform(actual, expected, delta);
    if (!message.empty()) {
      SUCCEED() << message;
    }
  }

  // Helper to test a flange pose against expected values
  void testFlangePose(const std::array<double, 7>& joint_config,
                      const std::array<double, 16>& expected,
                      double delta = franka_test_utils::kForwardKinematicsEps,
                      const std::string& message = "") {
    std::array<double, 16> actual = model->poseFlange(joint_config);
    verifyTransform(actual, expected, delta);
    if (!message.empty()) {
      SUCCEED() << message;
    }
  }

  // Helper to test an end effector pose against expected values
  void testEndEffectorPose(const std::array<double, 7>& joint_config,
                           const std::array<double, 16>& f_t_ee,
                           const std::array<double, 16>& expected,
                           double delta = franka_test_utils::kForwardKinematicsEps,
                           const std::string& message = "") {
    std::array<double, 16> actual = model->poseEe(joint_config, f_t_ee);
    verifyTransform(actual, expected, delta);
    if (!message.empty()) {
      SUCCEED() << message;
    }
  }

  // Helper to test a stiffness frame pose against expected values
  void testStiffnessPose(const std::array<double, 7>& joint_config,
                         const std::array<double, 16>& f_t_ee,
                         const std::array<double, 16>& ee_t_k,
                         const std::array<double, 16>& expected,
                         double delta = franka_test_utils::kForwardKinematicsEps,
                         const std::string& message = "") {
    std::array<double, 16> actual = model->poseStiffness(joint_config, f_t_ee, ee_t_k);
    verifyTransform(actual, expected, delta);
    if (!message.empty()) {
      SUCCEED() << message;
    }
  }
};

TEST_F(RobotModelFKTest, givenZeroPosition_whenComputingAllJointPoses_thenMatchExpectedTransforms) {
  // Expected transforms for each joint at zero position

  // Joint 1 expected transform
  std::array<double, 16> expected_j1 = {1.000000, 0.000000, 0.000000, 0.000000, -0.000000, 1.000000,
                                        0.000000, 0.000000, 0.000000, 0.000000, 1.000000,  0.000000,
                                        0.000000, 0.000000, 0.333000, 1.000000};

  // Joint 2 expected transform
  std::array<double, 16> expected_j2 = {
      1.000000,  0.000000, -0.000000, 0.000000, -0.000000, -0.000000, -1.000000, 0.000000,
      -0.000000, 1.000000, 0.000000,  0.000000, 0.000000,  0.000000,  0.333000,  1.000000};

  // Joint 3 expected transform
  std::array<double, 16> expected_j3 = {
      1.000000, 0.000000, -0.000000, 0.000000, -0.000000, 1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 1.000000,  0.000000, 0.000000,  0.000000, 0.649200, 1.000000};

  // Joint 4 expected transform
  std::array<double, 16> expected_j4 = {
      1.000000, 0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 1.000000, 0.000000,
      0.000000, -1.000000, -0.000000, 0.000000, 0.082500, 0.000000, 0.649200, 1.000000};

  // Joint 5 expected transform
  std::array<double, 16> expected_j5 = {1.000000, 0.000000, 0.000000, 0.000000, -0.000000, 1.000000,
                                        0.000000, 0.000000, 0.000000, 0.000000, 1.000000,  0.000000,
                                        0.000000, 0.000000, 1.033400, 1.000000};

  // Joint 6 expected transform
  std::array<double, 16> expected_j6 = {
      1.000000, 0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 1.000000, 0.000000,
      0.000000, -1.000000, -0.000000, 0.000000, 0.000000, 0.000000, 1.033400, 1.000000};

  // Joint 7 expected transform
  std::array<double, 16> expected_j7 = {
      1.000000, 0.000000, 0.000000,  0.000000, 0.000000, -1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, -1.000000, 0.000000, 0.088000, 0.000000,  1.033400, 1.000000};

  // Flange expected transform
  std::array<double, 16> expected_flange = {
      1.000000, 0.000000, 0.000000,  0.000000, 0.000000, -1.000000, -0.000000, 0.000000,
      0.000000, 0.000000, -1.000000, 0.000000, 0.088000, 0.000000,  0.926400,  1.000000};

  // Test all joints and flange
  testJointPose(zero_position, 1, expected_j1, franka_test_utils::kForwardKinematicsEps,
                "Joint 1 pose at zero position");
  testJointPose(zero_position, 2, expected_j2, franka_test_utils::kForwardKinematicsEps,
                "Joint 2 pose at zero position");
  testJointPose(zero_position, 3, expected_j3, franka_test_utils::kForwardKinematicsEps,
                "Joint 3 pose at zero position");
  testJointPose(zero_position, 4, expected_j4, franka_test_utils::kForwardKinematicsEps,
                "Joint 4 pose at zero position");
  testJointPose(zero_position, 5, expected_j5, franka_test_utils::kForwardKinematicsEps,
                "Joint 5 pose at zero position");
  testJointPose(zero_position, 6, expected_j6, franka_test_utils::kForwardKinematicsEps,
                "Joint 6 pose at zero position");
  testJointPose(zero_position, 7, expected_j7, franka_test_utils::kForwardKinematicsEps,
                "Joint 7 pose at zero position");
  testFlangePose(zero_position, expected_flange, franka_test_utils::kForwardKinematicsEps,
                 "Flange pose at zero position");
}

TEST_F(RobotModelFKTest,
       givenDefaultConfiguration_whenComputingAllJointPoses_thenMatchExpectedTransforms) {
  // Expected transforms for each joint in default configuration

  // Joint 1 expected transform
  std::array<double, 16> expected_j1 = {1.000000, 0.000000, 0.000000, 0.000000, -0.000000, 1.000000,
                                        0.000000, 0.000000, 0.000000, 0.000000, 1.000000,  0.000000,
                                        0.000000, 0.000000, 0.333000, 1.000000};

  // Joint 2 expected transform
  std::array<double, 16> expected_j2 = {
      1.000000,  0.000000, -0.000000, 0.000000, -0.000000, -0.000000, -1.000000, 0.000000,
      -0.000000, 1.000000, 0.000000,  0.000000, 0.000000,  0.000000,  0.333000,  1.000000};

  // Joint 3 expected transform
  std::array<double, 16> expected_j3 = {
      1.000000, 0.000000, -0.000000, 0.000000, -0.000000, 1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 1.000000,  0.000000, 0.000000,  0.000000, 0.649200, 1.000000};

  // Joint 4 expected transform
  std::array<double, 16> expected_j4 = {
      -0.707107, -0.000000, -0.707107, 0.000000, 0.707107, 0.000000, -0.707107, 0.000000,
      0.000000,  -1.000000, -0.000000, 0.000000, 0.082500, 0.000000, 0.649200,  1.000000};

  // Joint 5 expected transform
  std::array<double, 16> expected_j5 = {
      -0.707107, 0.000000, -0.707107, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000,
      0.707107,  0.000000, -0.707107, 0.000000, 0.412507, 0.000000, 0.435866, 1.000000};

  // Joint 6 expected transform
  std::array<double, 16> expected_j6 = {
      1.000000, 0.000000,  -0.000000, 0.000000, -0.000000, -0.000000, 1.000000, 0.000000,
      0.000000, -1.000000, -0.000000, 0.000000, 0.412507,  0.000000,  0.435866, 1.000000};

  // Joint 7 expected transform
  std::array<double, 16> expected_j7 = {
      1.000000, -0.000000, -0.000000, 0.000000, -0.000000, -1.000000, 0.000000, 0.000000,
      0.000000, 0.000000,  -1.000000, 0.000000, 0.500507,  0.000000,  0.435866, 1.000000};

  // Flange expected transform
  std::array<double, 16> expected_flange = {
      1.000000, 0.000000, -0.000000, 0.000000, 0.000000, -1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, -1.000000, 0.000000, 0.500507, 0.000000,  0.328866, 1.000000};

  // Test all joints and flange
  testJointPose(default_joint_configuration, 1, expected_j1,
                franka_test_utils::kForwardKinematicsEps, "Joint 1 pose at default configuration");
  testJointPose(default_joint_configuration, 2, expected_j2,
                franka_test_utils::kForwardKinematicsEps, "Joint 2 pose at default configuration");
  testJointPose(default_joint_configuration, 3, expected_j3,
                franka_test_utils::kForwardKinematicsEps, "Joint 3 pose at default configuration");
  testJointPose(default_joint_configuration, 4, expected_j4,
                franka_test_utils::kForwardKinematicsEps, "Joint 4 pose at default configuration");
  testJointPose(default_joint_configuration, 5, expected_j5,
                franka_test_utils::kForwardKinematicsEps, "Joint 5 pose at default configuration");
  testJointPose(default_joint_configuration, 6, expected_j6,
                franka_test_utils::kForwardKinematicsEps, "Joint 6 pose at default configuration");
  testJointPose(default_joint_configuration, 7, expected_j7,
                franka_test_utils::kForwardKinematicsEps, "Joint 7 pose at default configuration");
  testFlangePose(default_joint_configuration, expected_flange,
                 franka_test_utils::kForwardKinematicsEps, "Flange pose at default configuration");
}

TEST_F(RobotModelFKTest,
       givenZeroPosition_whenApplyingCustomEndEffectorTransform_thenMatchesExpectedResult) {
  // Create a custom end effector transformation (column-major format)
  std::array<double, 16> f_t_ee = {
      0,   0, -1, 0,   // First column
      0,   1, 0,  0,   // Second column
      1,   0, 0,  0,   // Third column
      0.1, 0, 0,  1};  // Fourth column (translation + homogeneous part)

  std::array<double, 16> flange_transform = model->poseFlange(zero_position);

  Eigen::Matrix4d ee_transform = Eigen::Map<const Eigen::Matrix4d>(flange_transform.data()) *
                                 Eigen::Map<const Eigen::Matrix4d>(f_t_ee.data());

  // Convert Eigen matrix to array for comparison
  std::array<double, 16> expected_transform;
  std::copy(ee_transform.data(), ee_transform.data() + 16, expected_transform.begin());

  testEndEffectorPose(zero_position, f_t_ee, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenMovedJointConfiguration_whenComputingEndEffectorPose_thenMatchesExpectedTransform) {
  // identity end effector transformation
  std::array<double, 16> f_t_ee = identity_transform;

  std::array<double, 16> expected_transform = {
      0.999999, 0.000292, 0.000999,  0.0, 0.000293, -1.000000, -0.000709, 0.0,
      0.000999, 0.000709, -0.999999, 0.0, 0.500928, 0.001015,  0.328869,  1.0};

  testEndEffectorPose(moved_joint_configuration, f_t_ee, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenTwoDifferentJointConfigurations_whenComputingPoses_thenTransformsAreDifferent) {
  // This test verifies that the poses change consistently when we move from zero to another
  // position

  // Zero Position Joint 7 in column-major format
  std::array<double, 16> expected_zero = {
      1.000000, 0.000000, 0.000000,  0.000000, 0.000000, -1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, -1.000000, 0.000000, 0.088000, 0.000000,  1.033400, 1.000000};

  // Moved Joint Configuration Joint 7 in column-major format
  std::array<double, 16> expected_moved = {
      0.999999, 0.000292, 0.000999,  0.000000, 0.000293, -1.000000, -0.000709, 0.000000,
      0.000999, 0.000709, -0.999999, 0.000000, 0.500821, 0.000939,  0.435869,  1.000000};

  // Test each pose individually
  std::array<double, 16> joint7_zero = model->pose(zero_position, 7);
  std::array<double, 16> joint7_moved = model->pose(moved_joint_configuration, 7);

  // Check that joint7_zero matches expected_zero
  verifyTransform(joint7_zero, expected_zero, franka_test_utils::kForwardKinematicsEps);

  // Check that joint7_moved matches expected_moved
  verifyTransform(joint7_moved, expected_moved, franka_test_utils::kForwardKinematicsEps);

  // The moved position should be different from the zero position
  EXPECT_TRUE(transformsAreDifferent(joint7_zero, joint7_moved))
      << "Moving the robot should change the pose of joint 7";
}

TEST_F(RobotModelFKTest,
       givenMovedConfiguration_whenComputingJoint1Pose_thenMatchesExpectedTransform) {
  // Joint 1 in moved joint configuration
  std::array<double, 16> expected_transform = {
      1.000000, 0.001000, 0.000000, 0.000000, -0.001000, 1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 1.000000, 0.000000, 0.000000,  0.000000, 0.333000, 1.000000};

  testJointPose(moved_joint_configuration, 1, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenMovedConfiguration_whenComputingJoint2Pose_thenMatchesExpectedTransform) {
  // Joint 2 in moved joint configuration
  std::array<double, 16> expected_transform = {
      0.999999,  0.001000, -0.001000, 0.000000, -0.001000, -0.000001, -1.000000, 0.000000,
      -0.001000, 1.000000, 0.000000,  0.000000, 0.000000,  0.000000,  0.333000,  1.000000};

  testJointPose(moved_joint_configuration, 2, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenMovedConfiguration_whenComputingJoint3Pose_thenMatchesExpectedTransform) {
  // Joint 3 in moved joint configuration
  std::array<double, 16> expected_transform = {
      0.999998, 0.002000, -0.001000, 0.000000, -0.002000, 0.999998, 0.000001, 0.000000,
      0.001000, 0.000001, 1.000000,  0.000000, 0.000316,  0.000000, 0.649200, 1.000000};

  testJointPose(moved_joint_configuration, 3, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenMovedConfiguration_whenComputingJoint4Pose_thenMatchesExpectedTransform) {
  // Joint 4 in moved joint configuration
  std::array<double, 16> expected_transform = {
      -0.707109, -0.001414, -0.707103, 0.000000, 0.707101, 0.001415, -0.707111, 0.000000,
      0.002000,  -0.999998, -0.000001, 0.000000, 0.082816, 0.000165, 0.649117,  1.000000};

  testJointPose(moved_joint_configuration, 4, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenMovedConfiguration_whenComputingJoint5Pose_thenMatchesExpectedTransform) {
  // Joint 5 in moved joint configuration
  std::array<double, 16> expected_transform = {
      -0.707111, -0.000414, -0.707103, 0.000000, -0.001293, 0.999999, 0.000708, 0.000000,
      0.707101,  0.001415,  -0.707111, 0.000000, 0.412821,  0.000826, 0.435781, 1.000000};

  testJointPose(moved_joint_configuration, 5, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenMovedConfiguration_whenComputingJoint6Pose_thenMatchesExpectedTransform) {
  // Joint 6 in moved joint configuration
  std::array<double, 16> expected_transform = {
      0.999999, 0.001292,  0.001000,  0.000000, -0.000999, -0.000709, 0.999999, 0.000000,
      0.001293, -0.999999, -0.000708, 0.000000, 0.412821,  0.000826,  0.435781, 1.000000};

  testJointPose(moved_joint_configuration, 6, expected_transform);
}

TEST_F(RobotModelFKTest,
       givenMovedConfiguration_whenComputingJoint7Pose_thenMatchesExpectedTransform) {
  // Joint 7 in moved joint configuration
  std::array<double, 16> expected_transform = {
      0.999999, 0.000292, 0.000999,  0.000000, 0.000293, -1.000000, -0.000709, 0.000000,
      0.000999, 0.000709, -0.999999, 0.000000, 0.500821, 0.000939,  0.435869,  1.000000};

  testJointPose(moved_joint_configuration, 7, expected_transform);
}

// Test the pose of the stiffness frame
TEST_F(RobotModelFKTest, givenStiffnessFrame_whenComputingPose_thenMatchesExpectedTransform) {
  std::array<double, 16> expected_transform = {
      1.000000, 0.000000, 0.000000,  0.000000, 0.000000, -1.000000, -0.000000, 0.000000,
      0.000000, 0.000000, -1.000000, 0.000000, 0.088000, 0.000000,  0.926400,  1.000000};

  std::array<double, 16> actual_stiffness =
      model->poseStiffness(zero_position, identity_transform, identity_transform);
  verifyTransform(actual_stiffness, expected_transform, franka_test_utils::kForwardKinematicsEps);
}

// Test end effector rotated around X axis
TEST_F(RobotModelFKTest,
       givenDefaultConfiguration_whenEERotatedAroundX_thenMatchesExpectedTransform) {
  // Expected transform from CSV for default config with EE rotated around X
  std::array<double, 16> expected_transform = {
      1.000000, 0.000000,  -0.000000, 0.000000, -0.000000, -0.000000, 1.000000, 0.000000,
      0.000000, -1.000000, -0.000000, 0.000000, 0.500507,  0.000000,  0.328866, 1.000000};

  testEndEffectorPose(default_joint_configuration, F_T_EE_rotX, expected_transform,
                      franka_test_utils::kForwardKinematicsEps,
                      "End effector pose with 90° rotation around X axis");
}

// Test end effector rotated around Y axis
TEST_F(RobotModelFKTest,
       givenDefaultConfiguration_whenEERotatedAroundY_thenMatchesExpectedTransform) {
  // Expected transform from CSV for default config with EE rotated around Y
  std::array<double, 16> expected_transform = {
      0.000000,  0.000000, -1.000000, 0.000000, 0.000000, -1.000000, 0.000000, 0.000000,
      -1.000000, 0.000000, -0.000000, 0.000000, 0.500507, 0.000000,  0.328866, 1.000000};

  testEndEffectorPose(default_joint_configuration, F_T_EE_rotY, expected_transform,
                      franka_test_utils::kForwardKinematicsEps,
                      "End effector pose with 90° rotation around Y axis");
}

// Test end effector rotated around Z axis
TEST_F(RobotModelFKTest,
       givenDefaultConfiguration_whenEERotatedAroundZ_thenMatchesExpectedTransform) {
  // Expected transform from CSV for default config with EE rotated around Z
  std::array<double, 16> expected_transform = {
      0.000000, 1.000000, -0.000000, 0.000000, 1.000000, -0.000000, -0.000000, 0.000000,
      0.000000, 0.000000, -1.000000, 0.000000, 0.500507, 0.000000,  0.328866,  1.000000};

  testEndEffectorPose(default_joint_configuration, F_T_EE_rotZ, expected_transform,
                      franka_test_utils::kForwardKinematicsEps,
                      "End effector pose with 90° rotation around Z axis");
}

// Test stiffness frame with translation
TEST_F(RobotModelFKTest,
       givenDefaultConfiguration_whenStiffnessFrameTranslated_thenMatchesExpectedTransform) {
  // Expected transform from CSV for default config with stiffness frame translated
  std::array<double, 16> expected_transform = {
      1.025025, 0.000000, 0.016443,  0.050000, 0.015015, -1.000000, 0.009866, 0.030000,
      0.010010, 0.000000, -0.993423, 0.020000, 0.500507, 0.000000,  0.328866, 1.000000};

  testStiffnessPose(default_joint_configuration, identity_transform, EE_T_K_translated,
                    expected_transform, franka_test_utils::kForwardKinematicsEps,
                    "Stiffness frame pose with translation");
}

// Test combined EE rotation around X and stiffness frame translation
TEST_F(RobotModelFKTest,
       givenDefaultConfig_whenEERotatedXAndStiffnessTranslated_thenMatchesExpectedTransform) {
  // Expected transform from CSV for default config with EE rotation around X and stiffness frame
  // translated
  std::array<double, 16> expected_transform = {
      1.025025, 0.000000,  0.016443, 0.050000, 0.015015, -0.000000, 1.009866, 0.030000,
      0.010010, -1.000000, 0.006577, 0.020000, 0.500507, 0.000000,  0.328866, 1.000000};

  testStiffnessPose(default_joint_configuration, F_T_EE_rotX, EE_T_K_translated, expected_transform,
                    franka_test_utils::kForwardKinematicsEps,
                    "Stiffness frame pose with EE rotation around X and translation");
}

// Test moved configuration with EE rotation around Z and stiffness frame translation
TEST_F(RobotModelFKTest,
       givenMovedConfig_whenEERotatedZAndStiffnessTranslated_thenMatchesExpectedTransform) {
  // Expected transform from CSV for moved config with EE rotation around Z and stiffness frame
  // translated
  std::array<double, 16> expected_transform = {
      0.024753, 1.000050, 0.017153,  0.050000, 1.015027, 0.000323, 0.010865, 0.030000,
      0.011017, 0.000730, -0.993422, 0.020000, 0.500928, 0.001015, 0.328869, 1.000000};

  testStiffnessPose(
      moved_joint_configuration, F_T_EE_rotZ, EE_T_K_translated, expected_transform,
      franka_test_utils::kForwardKinematicsEps,
      "Stiffness frame pose with moved configuration, EE rotation around Z and translation");
}

// Test moved configuration with end effector rotated around Z axis
TEST_F(RobotModelFKTest, givenMovedConfig_whenEERotatedAroundZ_thenMatchesExpectedTransform) {
  // Expected transform from CSV for moved config with EE rotated around Z
  std::array<double, 16> expected_transform = {
      -0.000293, 1.000000, 0.000709,  0.000000, 0.999999, 0.000292, 0.000999, 0.000000,
      0.000999,  0.000709, -0.999999, 0.000000, 0.500928, 0.001015, 0.328869, 1.000000};

  testEndEffectorPose(
      moved_joint_configuration, F_T_EE_rotZ, expected_transform,
      franka_test_utils::kForwardKinematicsEps,
      "End effector pose with moved joint configuration and 90° rotation around Z axis");
}

// Test all joint positions in the default configuration with EE rotated around X
TEST_F(RobotModelFKTest,
       givenAllJoints_whenDefaultConfigWithEERotatedX_thenMatchesExpectedTransforms) {
  // The joints are not affected by the end-effector transformation, so we verify
  // they match the expected values from the default configuration

  // Joint 1
  std::array<double, 16> expected_j1 = {1.000000, 0.000000, 0.000000, 0.000000, -0.000000, 1.000000,
                                        0.000000, 0.000000, 0.000000, 0.000000, 1.000000,  0.000000,
                                        0.000000, 0.000000, 0.333000, 1.000000};

  // Joint 2
  std::array<double, 16> expected_j2 = {
      1.000000,  0.000000, -0.000000, 0.000000, -0.000000, -0.000000, -1.000000, 0.000000,
      -0.000000, 1.000000, 0.000000,  0.000000, 0.000000,  0.000000,  0.333000,  1.000000};

  // Joint 3
  std::array<double, 16> expected_j3 = {
      1.000000, 0.000000, -0.000000, 0.000000, -0.000000, 1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, 1.000000,  0.000000, 0.000000,  0.000000, 0.649200, 1.000000};

  // Joint 4
  std::array<double, 16> expected_j4 = {
      -0.707107, -0.000000, -0.707107, 0.000000, 0.707107, 0.000000, -0.707107, 0.000000,
      0.000000,  -1.000000, -0.000000, 0.000000, 0.082500, 0.000000, 0.649200,  1.000000};

  // Joint 5
  std::array<double, 16> expected_j5 = {
      -0.707107, 0.000000, -0.707107, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000,
      0.707107,  0.000000, -0.707107, 0.000000, 0.412507, 0.000000, 0.435866, 1.000000};

  // Joint 6
  std::array<double, 16> expected_j6 = {
      1.000000, 0.000000,  -0.000000, 0.000000, -0.000000, -0.000000, 1.000000, 0.000000,
      0.000000, -1.000000, -0.000000, 0.000000, 0.412507,  0.000000,  0.435866, 1.000000};

  // Joint 7
  std::array<double, 16> expected_j7 = {
      1.000000, -0.000000, -0.000000, 0.000000, -0.000000, -1.000000, 0.000000, 0.000000,
      0.000000, 0.000000,  -1.000000, 0.000000, 0.500507,  0.000000,  0.435866, 1.000000};

  // Flange
  std::array<double, 16> expected_flange = {
      1.000000, 0.000000, -0.000000, 0.000000, 0.000000, -1.000000, 0.000000, 0.000000,
      0.000000, 0.000000, -1.000000, 0.000000, 0.500507, 0.000000,  0.328866, 1.000000};

  // Verify each joint position in the default configuration
  testJointPose(default_joint_configuration, 1, expected_j1);
  testJointPose(default_joint_configuration, 2, expected_j2);
  testJointPose(default_joint_configuration, 3, expected_j3);
  testJointPose(default_joint_configuration, 4, expected_j4);
  testJointPose(default_joint_configuration, 5, expected_j5);
  testJointPose(default_joint_configuration, 6, expected_j6);
  testJointPose(default_joint_configuration, 7, expected_j7);
  testFlangePose(default_joint_configuration, expected_flange);

  // The end effector pose with X rotation
  std::array<double, 16> expected_ee = {
      1.000000, 0.000000,  -0.000000, 0.000000, -0.000000, -0.000000, 1.000000, 0.000000,
      0.000000, -1.000000, -0.000000, 0.000000, 0.500507,  0.000000,  0.328866, 1.000000};

  testEndEffectorPose(default_joint_configuration, F_T_EE_rotX, expected_ee,
                      franka_test_utils::kForwardKinematicsEps,
                      "End effector pose with rotated X axis verified");
}

// Test all joint positions in the moved configuration with EE rotated around Z
TEST_F(RobotModelFKTest,
       givenAllJoints_whenMovedConfigWithEERotatedZ_thenMatchesExpectedTransforms) {
  // The joints are not affected by the end-effector transformation, so we verify
  // they match the expected values from the moved configuration

  // Joint 1
  std::array<double, 16> expected_j1 = {1.000000, 0.001000, 0.000000, 0.000000, -0.001000, 1.000000,
                                        0.000000, 0.000000, 0.000000, 0.000000, 1.000000,  0.000000,
                                        0.000000, 0.000000, 0.333000, 1.000000};

  // Joint 2
  std::array<double, 16> expected_j2 = {
      0.999999,  0.001000, -0.001000, 0.000000, -0.001000, -0.000001, -1.000000, 0.000000,
      -0.001000, 1.000000, 0.000000,  0.000000, 0.000000,  0.000000,  0.333000,  1.000000};

  // Joint 3
  std::array<double, 16> expected_j3 = {
      0.999998, 0.002000, -0.001000, 0.000000, -0.002000, 0.999998, 0.000001, 0.000000,
      0.001000, 0.000001, 1.000000,  0.000000, 0.000316,  0.000000, 0.649200, 1.000000};

  // Joint 4
  std::array<double, 16> expected_j4 = {
      -0.707109, -0.001414, -0.707103, 0.000000, 0.707101, 0.001415, -0.707111, 0.000000,
      0.002000,  -0.999998, -0.000001, 0.000000, 0.082816, 0.000165, 0.649117,  1.000000};

  // Joint 5
  std::array<double, 16> expected_j5 = {
      -0.707111, -0.000414, -0.707103, 0.000000, -0.001293, 0.999999, 0.000708, 0.000000,
      0.707101,  0.001415,  -0.707111, 0.000000, 0.412821,  0.000826, 0.435781, 1.000000};

  // Joint 6
  std::array<double, 16> expected_j6 = {
      0.999999, 0.001292,  0.001000,  0.000000, -0.000999, -0.000709, 0.999999, 0.000000,
      0.001293, -0.999999, -0.000708, 0.000000, 0.412821,  0.000826,  0.435781, 1.000000};

  // Joint 7
  std::array<double, 16> expected_j7 = {
      0.999999, 0.000292, 0.000999,  0.000000, 0.000293, -1.000000, -0.000709, 0.000000,
      0.000999, 0.000709, -0.999999, 0.000000, 0.500821, 0.000939,  0.435869,  1.000000};

  // Flange
  std::array<double, 16> expected_flange = {
      0.999999, 0.000292, 0.000999,  0.000000, 0.000293, -1.000000, -0.000709, 0.000000,
      0.000999, 0.000709, -0.999999, 0.000000, 0.500928, 0.001015,  0.328869,  1.000000};

  // Verify each joint position in the moved configuration
  testJointPose(moved_joint_configuration, 1, expected_j1);
  testJointPose(moved_joint_configuration, 2, expected_j2);
  testJointPose(moved_joint_configuration, 3, expected_j3);
  testJointPose(moved_joint_configuration, 4, expected_j4);
  testJointPose(moved_joint_configuration, 5, expected_j5);
  testJointPose(moved_joint_configuration, 6, expected_j6);
  testJointPose(moved_joint_configuration, 7, expected_j7);
  testFlangePose(moved_joint_configuration, expected_flange);

  // The end effector pose with Z rotation
  std::array<double, 16> expected_ee = {
      -0.000293, 1.000000, 0.000709,  0.000000, 0.999999, 0.000292, 0.000999, 0.000000,
      0.000999,  0.000709, -0.999999, 0.000000, 0.500928, 0.001015, 0.328869, 1.000000};

  testEndEffectorPose(moved_joint_configuration, F_T_EE_rotZ, expected_ee,
                      franka_test_utils::kForwardKinematicsEps,
                      "End effector pose with rotated Z axis verified in moved configuration");
}
