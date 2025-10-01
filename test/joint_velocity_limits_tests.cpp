// Copyright (c) 2025 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gtest/gtest.h>

#include <franka/joint_velocity_limits.h>

#include "test_utils.h"

using namespace franka;

TEST(JointVelocityLimits, ParseConfigFromFR3URDF) {
  std::string urdf_path = franka_test_utils::getUrdfPath(__FILE__);
  auto urdf_string = franka_test_utils::readFileToString(urdf_path);
  ASSERT_FALSE(urdf_string.empty()) << "Failed to read URDF file: " << urdf_path;

  JointVelocityLimitsConfig config(urdf_string);

  const double tolerance = 1e-4;

  EXPECT_NEAR(config[0].max_velocity, 2.62, tolerance);
  EXPECT_NEAR(config[0].velocity_offset, 0.30, tolerance);
  EXPECT_NEAR(config[0].deceleration_limit, 6.0, tolerance);
  EXPECT_NEAR(config[0].upper_position_limit, 2.7501, tolerance);
  EXPECT_NEAR(config[0].lower_position_limit, -2.7501, tolerance);

  EXPECT_NEAR(config[1].max_velocity, 2.62, tolerance);
  EXPECT_NEAR(config[1].velocity_offset, 0.20, tolerance);
  EXPECT_NEAR(config[1].deceleration_limit, 2.585, tolerance);
  EXPECT_NEAR(config[1].upper_position_limit, 1.7918, tolerance);
  EXPECT_NEAR(config[1].lower_position_limit, -1.7918, tolerance);

  EXPECT_NEAR(config[4].max_velocity, 5.26, tolerance);
  EXPECT_NEAR(config[4].velocity_offset, 0.35, tolerance);
  EXPECT_NEAR(config[4].deceleration_limit, 17.0, tolerance);
  EXPECT_NEAR(config[4].upper_position_limit, 2.81009, tolerance);
  EXPECT_NEAR(config[4].lower_position_limit, -2.81009, tolerance);

  EXPECT_NEAR(config[6].max_velocity, 5.26, tolerance);
  EXPECT_NEAR(config[6].velocity_offset, 0.35, tolerance);
  EXPECT_NEAR(config[6].deceleration_limit, 17.0, tolerance);
  EXPECT_NEAR(config[6].upper_position_limit, 3.0196, tolerance);
  EXPECT_NEAR(config[6].lower_position_limit, -3.0196, tolerance);
}

TEST(JointVelocityLimits, ParseConfigFromURDFNewFormatIncomplete) {
  // This test now checks that incomplete joint definitions throw exceptions
  const std::string incomplete_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
      <joint name="fr3v2_joint1" type="revolute">
        <limit effort="87.0" lower="-2.9007400166666666" upper="2.9007400166666666" velocity="2.62"/>
        <position_based_velocity_limits velocity_offset="0.6520000381679385" deceleration_limit="6.0"/>
      </joint>
      <joint name="fr3v2_joint2" type="revolute">
        <limit effort="87.0" lower="-1.8360900166666667" upper="1.8360900166666667" velocity="2.62"/>
        <position_based_velocity_limits velocity_offset="0.2499685851463105" deceleration_limit="2.585"/>
      </joint>
      <!-- Missing joints 3-7 -->
    </robot>
  )";

  EXPECT_THROW(JointVelocityLimitsConfig config(incomplete_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, ParseConfigFromURDFMissingTags) {
  // Test missing position_based_velocity_limits element
  const std::string missing_tags_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
      <joint name="fr3v2_joint1" type="revolute">
        <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
        <!-- Missing position_based_velocity_limits -->
      </joint>
    </robot>
  )";

  EXPECT_THROW(JointVelocityLimitsConfig config(missing_tags_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, ParseConfigFromURDFMalformedXML) {
  const std::string malformed_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
      <joint name="joint1" type="revolute">
        <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
        <position_based_velocity_limits velocity_offset="0.65" deceleration_limit="6.0
        <!-- Missing closing tag for deceleration_limit and position_based_velocity_limits -->
      </joint>
    </robot>
  )";

  EXPECT_THROW(JointVelocityLimitsConfig config(malformed_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, ParseConfigFromURDFEmptyString) {
  const std::string empty_urdf = "";
  EXPECT_THROW(JointVelocityLimitsConfig config(empty_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, ParseConfigFromURDFInvalidXMLSyntax) {
  const std::string invalid_xml = "This is not valid XML at all!";
  EXPECT_THROW(JointVelocityLimitsConfig config(invalid_xml), std::runtime_error);
}

TEST(JointVelocityLimits, ParseConfigFromURDFMinimalValidXML) {
  const std::string minimal_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
    </robot>
  )";

  // Should throw exception because no joints are present
  EXPECT_THROW(JointVelocityLimitsConfig config(minimal_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, DefaultConstructor) {
  JointVelocityLimitsConfig config;

  // All parameters should be default (zero) values
  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(config[i].max_velocity, 0.0);
    EXPECT_DOUBLE_EQ(config[i].velocity_offset, 0.0);
    EXPECT_DOUBLE_EQ(config[i].deceleration_limit, 0.0);
    EXPECT_DOUBLE_EQ(config[i].upper_position_limit, 0.0);
    EXPECT_DOUBLE_EQ(config[i].lower_position_limit, 0.0);
  }
}

TEST(JointVelocityLimits, ParseFromURDFMethodIncomplete) {
  const std::string incomplete_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
      <joint name="fr3v2_joint1" type="revolute">
        <limit effort="87.0" lower="-2.5" upper="2.5" velocity="3.0"/>
        <position_based_velocity_limits velocity_offset="0.5" deceleration_limit="8.0"/>
      </joint>
      <!-- Missing joints 2-7 -->
    </robot>
  )";

  // Should throw exception because only 1 joint is present, need all 7
  EXPECT_THROW(JointVelocityLimitsConfig config(incomplete_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, GetJointParamsMethodIncomplete) {
  const std::string incomplete_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
      <joint name="fr3v2_joint1" type="revolute">
        <limit effort="87.0" lower="-1.5" upper="1.5" velocity="2.5"/>
        <position_based_velocity_limits velocity_offset="0.3" deceleration_limit="5.0"/>
      </joint>
      <!-- Missing joints 2-7 -->
    </robot>
  )";

  // Should throw exception because only 1 joint is present, need all 7
  EXPECT_THROW(JointVelocityLimitsConfig config(incomplete_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, ParseConfigFromURDFCompleteValidFormat) {
  // Test with all 7 joints properly defined
  const std::string complete_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
      <joint name="joint1" type="revolute">
        <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
        <position_based_velocity_limits velocity_offset="0.30" deceleration_limit="6.0"/>
      </joint>
      <joint name="joint2" type="revolute">
        <limit effort="87.0" lower="-1.8" upper="1.8" velocity="2.62"/>
        <position_based_velocity_limits velocity_offset="0.20" deceleration_limit="2.585"/>
      </joint>
      <joint name="joint3" type="revolute">
        <limit effort="87.0" lower="-2.9" upper="2.9" velocity="2.62"/>
        <position_based_velocity_limits velocity_offset="0.25" deceleration_limit="5.0"/>
      </joint>
      <joint name="joint4" type="revolute">
        <limit effort="87.0" lower="-3.1" upper="0.1" velocity="2.62"/>
        <position_based_velocity_limits velocity_offset="0.15" deceleration_limit="4.0"/>
      </joint>
      <joint name="joint5" type="revolute">
        <limit effort="12.0" lower="-2.8" upper="2.8" velocity="5.26"/>
        <position_based_velocity_limits velocity_offset="0.35" deceleration_limit="17.0"/>
      </joint>
      <joint name="joint6" type="revolute">
        <limit effort="12.0" lower="-3.8" upper="0.8" velocity="5.26"/>
        <position_based_velocity_limits velocity_offset="0.25" deceleration_limit="12.0"/>
      </joint>
      <joint name="joint7" type="revolute">
        <limit effort="12.0" lower="-3.0" upper="3.0" velocity="5.26"/>
        <position_based_velocity_limits velocity_offset="0.35" deceleration_limit="17.0"/>
      </joint>
    </robot>
  )";

  // Should succeed with all 7 joints
  JointVelocityLimitsConfig config(complete_urdf);

  // Test that getJointParams returns the same data as operator[]
  const auto& all_params = config.getJointParams();
  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(all_params[i].max_velocity, config[i].max_velocity);
    EXPECT_DOUBLE_EQ(all_params[i].velocity_offset, config[i].velocity_offset);
    EXPECT_DOUBLE_EQ(all_params[i].deceleration_limit, config[i].deceleration_limit);
    EXPECT_DOUBLE_EQ(all_params[i].upper_position_limit, config[i].upper_position_limit);
    EXPECT_DOUBLE_EQ(all_params[i].lower_position_limit, config[i].lower_position_limit);
  }

  // Verify some specific joint values
  EXPECT_DOUBLE_EQ(config[0].max_velocity, 2.62);
  EXPECT_DOUBLE_EQ(config[0].velocity_offset, 0.30);
  EXPECT_DOUBLE_EQ(config[0].deceleration_limit, 6.0);
  EXPECT_DOUBLE_EQ(config[6].max_velocity, 5.26);
  EXPECT_DOUBLE_EQ(config[6].velocity_offset, 0.35);
  EXPECT_DOUBLE_EQ(config[6].deceleration_limit, 17.0);
}

TEST(JointVelocityLimits, ParseConfigFromURDFMissingAttributes) {
  // Test missing required attributes
  const std::string missing_attr_urdf = R"(
    <?xml version="1.0" ?>
    <robot name="test_robot">
      <joint name="joint1" type="revolute">
        <limit effort="87.0" lower="-2.9" upper="2.9"/>
        <!-- Missing velocity attribute -->
        <position_based_velocity_limits velocity_offset="0.30" deceleration_limit="6.0"/>
      </joint>
    </robot>
  )";

  EXPECT_THROW(JointVelocityLimitsConfig config(missing_attr_urdf), std::runtime_error);
}

TEST(JointVelocityLimits, PositionBasedJointVelocityLimitConstantsConstructors) {
  // Test default constructor
  PositionBasedJointVelocityLimitConstants default_params;
  EXPECT_DOUBLE_EQ(default_params.max_velocity, 0.0);
  EXPECT_DOUBLE_EQ(default_params.velocity_offset, 0.0);
  EXPECT_DOUBLE_EQ(default_params.deceleration_limit, 0.0);
  EXPECT_DOUBLE_EQ(default_params.upper_position_limit, 0.0);
  EXPECT_DOUBLE_EQ(default_params.lower_position_limit, 0.0);

  // Test parameterized constructor
  PositionBasedJointVelocityLimitConstants params(2.5, 0.3, 5.0, 1.5, -1.5);
  EXPECT_DOUBLE_EQ(params.max_velocity, 2.5);
  EXPECT_DOUBLE_EQ(params.velocity_offset, 0.3);
  EXPECT_DOUBLE_EQ(params.deceleration_limit, 5.0);
  EXPECT_DOUBLE_EQ(params.upper_position_limit, 1.5);
  EXPECT_DOUBLE_EQ(params.lower_position_limit, -1.5);
}
