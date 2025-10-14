// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <franka/rate_limiting.h>

#include "helpers.h"

using namespace franka;

const double kNoLimit{std::numeric_limits<double>::max()};
const double kNoLowerLimit{std::numeric_limits<double>::lowest()};

std::array<double, 7> kJointsNoLimit{
    {kNoLimit, kNoLimit, kNoLimit, kNoLimit, kNoLimit, kNoLimit, kNoLimit}};

std::array<double, 7> kJointsNoLowerLimit{{kNoLowerLimit, kNoLowerLimit, kNoLowerLimit,
                                           kNoLowerLimit, kNoLowerLimit, kNoLowerLimit,
                                           kNoLowerLimit}};

template <int size>
std::array<double, size> integrateOneSample(std::array<double, size> last_value,
                                            std::array<double, size> derivative,
                                            double delta_t) {
  std::array<double, size> result{};
  for (size_t i = 0; i < size; i++) {
    result[i] = last_value[i] + derivative[i] * delta_t;
  }
  return result;
}

template <int size>
std::array<double, size> differentiateOneSample(std::array<double, size> value,
                                                std::array<double, size> last_value,
                                                double delta_t) {
  std::array<double, size> result{};
  for (size_t i = 0; i < size; i++) {
    result[i] = (value[i] - last_value[i]) / delta_t;
  }
  return result;
}

std::array<double, 16> integrateOneSample(std::array<double, 16> last_pose,
                                          std::array<double, 6> twist,
                                          double delta_t) {
  Eigen::Affine3d pose(Eigen::Matrix4d::Map(last_pose.data()));
  Eigen::Map<Eigen::Matrix<double, 6, 1>> dx(twist.data());
  Eigen::Matrix3d omega_skew;
  omega_skew << 0, -dx[5], dx[4], dx[5], 0, -dx[3], -dx[4], dx[3], 0;
  pose.linear() << pose.rotation() + omega_skew * pose.rotation() * delta_t;
  pose.translation() << pose.translation() + dx.head(3) * delta_t;

  std::array<double, 16> pose_after_integration{};
  Eigen::Map<Eigen::Matrix4d>(&pose_after_integration[0], 4, 4) = pose.matrix();
  return pose_after_integration;
}

bool violatesLimits(double desired_value, double max_value) {
  return std::abs(desired_value) > max_value;
}

bool violatesLimits(std::array<double, 7> values, std::array<double, 7> max_values) {
  bool violates_limits = false;
  for (size_t i = 0; i < 7 && !violates_limits; i++) {
    violates_limits = violates_limits || violatesLimits(values[i], max_values[i]);
  }
  return violates_limits;
}

bool violatesRateLimits(std::array<double, 7> max_derivatives,
                        std::array<double, 7> values,
                        std::array<double, 7> last_desired_values,
                        double delta_t) {
  return violatesLimits(differentiateOneSample<7>(values, last_desired_values, delta_t),
                        max_derivatives);
}

bool violatesRateLimits(std::array<double, 7> max_values,
                        std::array<double, 7> max_derivatives,
                        std::array<double, 7> max_dderivatives,
                        std::array<double, 7> values,
                        std::array<double, 7> last_values,
                        std::array<double, 7> last_dvalues,
                        double delta_t) {
  std::array<double, 7> desired_derivatives =
      differentiateOneSample<7>(values, last_values, delta_t);

  return violatesLimits(values, max_values) ||
         violatesRateLimits(max_derivatives, values, last_values, delta_t) ||
         violatesRateLimits(max_dderivatives, desired_derivatives, last_dvalues, delta_t);
}

bool violatesRateLimits(double max_translational_dx,
                        double max_translational_ddx,
                        double max_translational_dddx,
                        double max_rotational_dx,
                        double max_rotational_ddx,
                        double max_rotational_dddx,
                        std::array<double, 6> cmd_dx,
                        std::array<double, 6> O_dP_EE_c,
                        std::array<double, 6> O_ddP_EE_c,
                        double delta_t) {
  Eigen::Map<Eigen::Matrix<double, 6, 1>> dx(cmd_dx.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> last_dx(O_dP_EE_c.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> last_ddx(O_ddP_EE_c.data());
  Eigen::Matrix<double, 6, 1> ddx = (dx - last_dx) / delta_t;
  Eigen::Matrix<double, 6, 1> dddx = (ddx - last_ddx) / delta_t;
  return violatesLimits(dx.head(3).norm(), max_translational_dx) ||
         violatesLimits(ddx.head(3).norm(), max_translational_ddx) ||
         violatesLimits(dddx.head(3).norm(), max_translational_dddx) ||
         violatesLimits(dx.tail(3).norm(), max_rotational_dx) ||
         violatesLimits(ddx.tail(3).norm(), max_rotational_ddx) ||
         violatesLimits(dddx.tail(3).norm(), max_rotational_dddx);
}

std::array<double, 7> generateValuesIntoLimits(std::array<double, 7> last_cmd_values,
                                               std::array<double, 7> max_derivatives,
                                               double eps,
                                               double delta_t) {
  std::array<double, 7> cmd_value{};
  for (size_t i = 0; i < 7; i++) {
    // Make sure that the integration yields a value into limits
    cmd_value[i] = last_cmd_values[i] + (max_derivatives[i] - std::min(std::max(std::abs(eps), 0.0),
                                                                       2.0 * max_derivatives[i])) *
                                            delta_t;
  }
  return cmd_value;
}

std::array<double, 7> generateValuesOutsideLimits(std::array<double, 7> last_cmd_values,
                                                  std::array<double, 7> max_derivatives,
                                                  double eps,
                                                  double delta_t) {
  std::array<double, 7> cmd_value{};
  for (size_t i = 0; i < 7; i++) {
    // Make sure that diff yields a value outside limits
    cmd_value[i] =
        last_cmd_values[i] + (max_derivatives[i] + std::max(std::abs(eps), kLimitEps)) * delta_t;
  }
  return cmd_value;
}

std::array<double, 6> generateValuesIntoLimits(std::array<double, 6> last_cmd_values,
                                               double max_translational_derivative,
                                               double max_rotational_derivative,
                                               double eps,
                                               double delta_t) {
  Eigen::Map<Eigen::Matrix<double, 6, 1>> last_values(last_cmd_values.data());
  Eigen::Matrix<double, 6, 1> values;
  Eigen::Vector3d unit_vector(1.0, 0.0, 0.0);
  std::array<double, 6> result;
  values.head(3) << last_values.head(3) + unit_vector *
                                              (max_translational_derivative -
                                               std::min(std::max(std::abs(eps), 0.0),
                                                        2.0 * max_translational_derivative)) *
                                              delta_t;
  values.tail(3) << last_values.tail(3) + unit_vector *
                                              (max_rotational_derivative -
                                               std::min(std::max(std::abs(eps), 0.0),
                                                        2.0 * max_rotational_derivative)) *
                                              delta_t;

  Eigen::Matrix<double, 6, 1>::Map(&result[0], 6) = values;
  return result;
}

std::array<double, 6> generateValuesOutsideLimits(std::array<double, 6> last_cmd_values,
                                                  double max_translational_derivative,
                                                  double max_rotational_derivative,
                                                  double eps,
                                                  double delta_t) {
  Eigen::Map<Eigen::Matrix<double, 6, 1>> last_values(last_cmd_values.data());
  Eigen::Matrix<double, 6, 1> values;
  Eigen::Vector3d unit_vector(1.0, 0.0, 0.0);
  std::array<double, 6> result;
  values.head(3) << last_values.head(3) +
                        unit_vector *
                            (max_translational_derivative + std::max(std::abs(eps), kLimitEps)) *
                            delta_t;
  values.tail(3) << last_values.tail(3) +
                        unit_vector *
                            (max_rotational_derivative + std::max(std::abs(eps), kLimitEps)) *
                            delta_t;

  Eigen::Matrix<double, 6, 1>::Map(&result[0], 6) = values;
  return result;
}

TEST(RateLimiting, MaxDerivative) {
  std::array<double, 7> max_derivatives{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
  std::array<double, 7> last_cmd_values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate
  std::array<double, 7> values_into_limits =
      generateValuesIntoLimits(last_cmd_values, max_derivatives, eps, kDeltaT);
  ASSERT_FALSE(violatesRateLimits(max_derivatives, values_into_limits, last_cmd_values, kDeltaT));
  EXPECT_EQ(values_into_limits, limitRate(max_derivatives, values_into_limits, last_cmd_values));

  // Desired values are outside limits and limited after limitRate
  std::array<double, 7> values_outside_limits =
      generateValuesOutsideLimits(last_cmd_values, max_derivatives, eps, kDeltaT);
  std::array<double, 7> limited_values =
      limitRate(max_derivatives, values_outside_limits, last_cmd_values);
  ASSERT_TRUE(violatesRateLimits(max_derivatives, values_outside_limits, last_cmd_values, kDeltaT));
  EXPECT_NE(values_outside_limits, limited_values);
  EXPECT_FALSE(violatesRateLimits(max_derivatives, limited_values, last_cmd_values, kDeltaT));
}

TEST(RateLimiting, JointVelocity) {
  std::array<double, 7> last_cmd_velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> last_cmd_acceleration{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> max_acceleration{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  std::array<double, 7> max_jerk{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate (jerk)
  std::array<double, 7> joint_velocity_into_limits = integrateOneSample<7>(
      last_cmd_velocity, generateValuesIntoLimits(last_cmd_acceleration, max_jerk, eps, kDeltaT),
      kDeltaT);
  ASSERT_FALSE(violatesRateLimits(kJointsNoLimit, kJointsNoLimit, max_jerk,
                                  joint_velocity_into_limits, last_cmd_velocity,
                                  last_cmd_acceleration, kDeltaT));
  EXPECT_EQ(joint_velocity_into_limits,
            limitRate(kJointsNoLimit, kJointsNoLowerLimit, kJointsNoLimit, max_jerk,
                      joint_velocity_into_limits, last_cmd_velocity, last_cmd_acceleration));

  // Desired values are into limits and unchanged after limitRate (acceleration)
  joint_velocity_into_limits =
      generateValuesIntoLimits(last_cmd_velocity, max_acceleration, eps, kDeltaT);
  ASSERT_FALSE(violatesRateLimits(kJointsNoLimit, max_acceleration, kJointsNoLimit,
                                  joint_velocity_into_limits, last_cmd_velocity,
                                  last_cmd_acceleration, kDeltaT));
  EXPECT_EQ(joint_velocity_into_limits,
            limitRate(kJointsNoLimit, kJointsNoLowerLimit, max_acceleration, kJointsNoLimit,
                      joint_velocity_into_limits, last_cmd_velocity, last_cmd_acceleration));

  // Desired values are outside limits (jerk violation) and limited after limitRate
  std::array<double, 7> joint_velocity_outside_limits = integrateOneSample<7>(
      last_cmd_velocity, generateValuesOutsideLimits(last_cmd_acceleration, max_jerk, eps, kDeltaT),
      kDeltaT);
  std::array<double, 7> limited_joint_velocity =
      limitRate(kJointsNoLimit, kJointsNoLowerLimit, kJointsNoLimit, max_jerk,
                joint_velocity_outside_limits, last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(kJointsNoLimit, kJointsNoLimit, max_jerk,
                                 joint_velocity_outside_limits, last_cmd_velocity,
                                 last_cmd_acceleration, kDeltaT));
  EXPECT_NE(joint_velocity_outside_limits, limited_joint_velocity);
  EXPECT_FALSE(violatesRateLimits(kJointsNoLimit, kJointsNoLimit, max_jerk, limited_joint_velocity,
                                  last_cmd_velocity, last_cmd_acceleration, kDeltaT));

  // Desired values are outside limits (acceleration violation) and limited after limitRate
  joint_velocity_outside_limits =
      generateValuesOutsideLimits(last_cmd_velocity, max_acceleration, eps, kDeltaT);
  limited_joint_velocity =
      limitRate(kJointsNoLimit, kJointsNoLowerLimit, max_acceleration, kJointsNoLimit,
                joint_velocity_outside_limits, last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(kJointsNoLimit, max_acceleration, kJointsNoLimit,
                                 joint_velocity_outside_limits, last_cmd_velocity,
                                 last_cmd_acceleration, kDeltaT));
  EXPECT_NE(joint_velocity_outside_limits, limited_joint_velocity);
  EXPECT_FALSE(violatesRateLimits(kJointsNoLimit, max_acceleration, kJointsNoLimit,
                                  limited_joint_velocity, last_cmd_velocity, last_cmd_acceleration,
                                  kDeltaT));
}

TEST(RateLimiting, JointPosition) {
  std::array<double, 7> last_cmd_position{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> last_cmd_velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> last_cmd_acceleration{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> max_acceleration{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  std::array<double, 7> max_jerk{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate (jerk)
  std::array<double, 7> joint_position_into_limits = integrateOneSample<7>(
      last_cmd_position,
      integrateOneSample<7>(last_cmd_velocity,
                            generateValuesIntoLimits(last_cmd_acceleration, max_jerk, eps, kDeltaT),
                            kDeltaT),
      kDeltaT);
  ASSERT_FALSE(violatesRateLimits(
      kJointsNoLimit, kJointsNoLimit, max_jerk,
      differentiateOneSample<7>(joint_position_into_limits, last_cmd_position, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_EQ(joint_position_into_limits,
            limitRate(kJointsNoLimit, kJointsNoLowerLimit, kJointsNoLimit, max_jerk,
                      joint_position_into_limits, last_cmd_position, last_cmd_velocity,
                      last_cmd_acceleration));

  // Desired values are into limits and unchanged after limitRate (acceleration)
  joint_position_into_limits = integrateOneSample<7>(
      last_cmd_position,
      generateValuesIntoLimits(last_cmd_velocity, max_acceleration, eps, kDeltaT), kDeltaT);
  ASSERT_FALSE(violatesRateLimits(
      kJointsNoLimit, max_acceleration, kJointsNoLimit,
      differentiateOneSample<7>(joint_position_into_limits, last_cmd_position, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_EQ(joint_position_into_limits,
            limitRate(kJointsNoLimit, kJointsNoLowerLimit, max_acceleration, kJointsNoLimit,
                      joint_position_into_limits, last_cmd_position, last_cmd_velocity,
                      last_cmd_acceleration));

  // Desired values are outside limits (jerk violation) and limited after limitRate
  std::array<double, 7> joint_position_outside_limits = integrateOneSample<7>(
      last_cmd_position,
      integrateOneSample<7>(
          last_cmd_velocity,
          generateValuesOutsideLimits(last_cmd_acceleration, max_jerk, eps, kDeltaT), kDeltaT),
      kDeltaT);
  std::array<double, 7> limited_joint_position = limitRate(
      kJointsNoLimit, kJointsNoLowerLimit, kJointsNoLimit, max_jerk, joint_position_outside_limits,
      last_cmd_position, last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(
      kJointsNoLimit, kJointsNoLimit, max_jerk,
      differentiateOneSample<7>(joint_position_outside_limits, last_cmd_position, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_NE(joint_position_outside_limits, limited_joint_position);
  EXPECT_FALSE(violatesRateLimits(
      kJointsNoLimit, kJointsNoLimit, max_jerk,
      differentiateOneSample<7>(limited_joint_position, last_cmd_position, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));

  // Desired values outside limits (acceleration violation) and limited after limitRate
  joint_position_outside_limits = integrateOneSample<7>(
      last_cmd_position,
      generateValuesOutsideLimits(last_cmd_velocity, max_acceleration, eps, kDeltaT), kDeltaT);
  limited_joint_position = limitRate(kJointsNoLimit, kJointsNoLowerLimit, max_acceleration,
                                     kJointsNoLimit, joint_position_outside_limits,
                                     last_cmd_position, last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(
      kJointsNoLimit, max_acceleration, kJointsNoLimit,
      differentiateOneSample<7>(joint_position_outside_limits, last_cmd_position, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_NE(joint_position_outside_limits, limited_joint_position);
  EXPECT_FALSE(violatesRateLimits(
      kJointsNoLimit, max_acceleration, kJointsNoLimit,
      differentiateOneSample<7>(limited_joint_position, last_cmd_position, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
}

TEST(RateLimiting, CartesianVelocity) {
  std::array<double, 6> last_cmd_velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_cmd_acceleration{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double max_translational_acceleration{10.0};
  double max_translational_jerk{100.0};
  double max_rotational_acceleration{5.0};
  double max_rotational_jerk{50.0};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate (rotational and translational
  // jerk)
  std::array<double, 6> cartesian_velocity_into_limits =
      integrateOneSample<6>(last_cmd_velocity,
                            generateValuesIntoLimits(last_cmd_acceleration, max_translational_jerk,
                                                     max_rotational_jerk, eps, kDeltaT),
                            kDeltaT);
  ASSERT_FALSE(violatesRateLimits(kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit,
                                  max_rotational_jerk, cartesian_velocity_into_limits,
                                  last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_EQ(
      cartesian_velocity_into_limits,
      limitRate(kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit, max_rotational_jerk,
                cartesian_velocity_into_limits, last_cmd_velocity, last_cmd_acceleration));

  // Desired values are into limits and unchanged after limitRate (rotational and translational
  // acceleration)
  cartesian_velocity_into_limits = generateValuesIntoLimits(
      last_cmd_velocity, max_translational_acceleration, max_rotational_acceleration, eps, kDeltaT);
  ASSERT_FALSE(violatesRateLimits(
      kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit, max_rotational_acceleration,
      kNoLimit, cartesian_velocity_into_limits, last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_EQ(cartesian_velocity_into_limits,
            limitRate(kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit,
                      max_rotational_acceleration, kNoLimit, cartesian_velocity_into_limits,
                      last_cmd_velocity, last_cmd_acceleration));

  // Desired values are outside limits (rotational and translational jerk violation) and limited
  // after limitRate
  std::array<double, 6> cartesian_velocity_outside_limits = integrateOneSample<6>(
      last_cmd_velocity,
      generateValuesOutsideLimits(last_cmd_acceleration, max_translational_jerk,
                                  max_rotational_jerk, eps, kDeltaT),
      kDeltaT);
  std::array<double, 6> limited_cartesian_velocity =
      limitRate(kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit, max_rotational_jerk,
                cartesian_velocity_outside_limits, last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit,
                                 max_rotational_jerk, cartesian_velocity_outside_limits,
                                 last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_NE(cartesian_velocity_outside_limits, limited_cartesian_velocity);
  EXPECT_FALSE(violatesRateLimits(kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit,
                                  max_rotational_jerk, limited_cartesian_velocity,
                                  last_cmd_velocity, last_cmd_acceleration, kDeltaT));

  // Desired values are outside limits (rotational and translational acceleration violation) and
  // limited after limitRate
  cartesian_velocity_outside_limits = generateValuesOutsideLimits(
      last_cmd_velocity, max_translational_acceleration, max_rotational_acceleration, eps, kDeltaT);
  limited_cartesian_velocity = limitRate(
      kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit, max_rotational_acceleration,
      kNoLimit, cartesian_velocity_outside_limits, last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit,
                                 max_rotational_acceleration, kNoLimit,
                                 cartesian_velocity_outside_limits, last_cmd_velocity,
                                 last_cmd_acceleration, kDeltaT));
  EXPECT_NE(cartesian_velocity_outside_limits, limited_cartesian_velocity);
  EXPECT_FALSE(violatesRateLimits(kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit,
                                  max_rotational_acceleration, kNoLimit, limited_cartesian_velocity,
                                  last_cmd_velocity, last_cmd_acceleration, kDeltaT));
}

TEST(RateLimiting, CartesianPose) {
  std::array<double, 16> last_cmd_pose{
      {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  std::array<double, 6> last_cmd_velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_cmd_acceleration{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double max_translational_acceleration{10.0};
  double max_translational_jerk{100.0};
  double max_rotational_acceleration{5.0};
  double max_rotational_jerk{50.0};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate (rotational and translational
  // jerk)
  std::array<double, 16> cartesian_pose_into_limits = integrateOneSample(
      last_cmd_pose,
      integrateOneSample<6>(last_cmd_velocity,
                            generateValuesIntoLimits(last_cmd_acceleration, max_translational_jerk,
                                                     max_rotational_jerk, eps, kDeltaT),
                            kDeltaT),
      kDeltaT);
  ASSERT_FALSE(violatesRateLimits(
      kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit, max_rotational_jerk,
      differentiateOneSample(cartesian_pose_into_limits, last_cmd_pose, kDeltaT), last_cmd_velocity,
      last_cmd_acceleration, kDeltaT));

  std::array<double, 16> cartesian_pose_limited = limitRate(
      kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit, max_rotational_jerk,
      cartesian_pose_into_limits, last_cmd_pose, last_cmd_velocity, last_cmd_acceleration);

  for (size_t i = 0; i < cartesian_pose_into_limits.size(); i++) {
    EXPECT_NEAR(cartesian_pose_into_limits[i], cartesian_pose_limited[i], 1e-6);
  }

  // Desired values are into limits and unchanged after limitRate (rotational and translational
  // acceleration)
  cartesian_pose_into_limits =
      integrateOneSample(last_cmd_pose,
                         generateValuesIntoLimits(last_cmd_velocity, max_translational_acceleration,
                                                  max_rotational_acceleration, eps, kDeltaT),
                         kDeltaT);
  ASSERT_FALSE(violatesRateLimits(
      kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit, max_rotational_acceleration,
      kNoLimit, differentiateOneSample(cartesian_pose_into_limits, last_cmd_pose, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));

  cartesian_pose_limited =
      limitRate(kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit,
                max_rotational_acceleration, kNoLimit, cartesian_pose_into_limits, last_cmd_pose,
                last_cmd_velocity, last_cmd_acceleration);

  for (size_t i = 0; i < cartesian_pose_into_limits.size(); i++) {
    EXPECT_NEAR(cartesian_pose_into_limits[i], cartesian_pose_limited[i], 1e-6);
  }

  // Desired values are outside limits (rotational and translational jerk violation) and limited
  // after limitRate
  std::array<double, 16> cartesian_pose_outside_limits = integrateOneSample(
      last_cmd_pose,
      integrateOneSample<6>(
          last_cmd_velocity,
          generateValuesOutsideLimits(last_cmd_acceleration, max_translational_jerk,
                                      max_rotational_jerk, eps, kDeltaT),
          kDeltaT),
      kDeltaT);
  std::array<double, 16> limited_cartesian_pose = limitRate(
      kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit, max_rotational_jerk,
      cartesian_pose_outside_limits, last_cmd_pose, last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(
      kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit, max_rotational_jerk,
      differentiateOneSample(cartesian_pose_outside_limits, last_cmd_pose, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_NE(cartesian_pose_outside_limits, limited_cartesian_pose);
  EXPECT_FALSE(violatesRateLimits(
      kNoLimit, kNoLimit, max_translational_jerk, kNoLimit, kNoLimit, max_rotational_jerk,
      differentiateOneSample(limited_cartesian_pose, last_cmd_pose, kDeltaT), last_cmd_velocity,
      last_cmd_acceleration, kDeltaT));

  // Desired values are outside limits (rotational and translational acceleration violation) and
  // limited after limitRate
  cartesian_pose_outside_limits = integrateOneSample(
      last_cmd_pose,
      generateValuesOutsideLimits(last_cmd_velocity, max_translational_acceleration,
                                  max_rotational_acceleration, eps, kDeltaT),
      kDeltaT);
  limited_cartesian_pose =
      limitRate(kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit,
                max_rotational_acceleration, kNoLimit, cartesian_pose_outside_limits, last_cmd_pose,
                last_cmd_velocity, last_cmd_acceleration);
  ASSERT_TRUE(violatesRateLimits(
      kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit, max_rotational_acceleration,
      kNoLimit, differentiateOneSample(cartesian_pose_outside_limits, last_cmd_pose, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
  EXPECT_NE(cartesian_pose_outside_limits, limited_cartesian_pose);
  EXPECT_FALSE(violatesRateLimits(
      kNoLimit, max_translational_acceleration, kNoLimit, kNoLimit, max_rotational_acceleration,
      kNoLimit, differentiateOneSample(limited_cartesian_pose, last_cmd_pose, kDeltaT),
      last_cmd_velocity, last_cmd_acceleration, kDeltaT));
}

TEST(RateLimiting, CartesianPoseIntegrationAndDifferentation) {
  std::array<double, 16> last_cmd_pose{
      {0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  std::array<double, 6> last_cmd_velocity{{1.0, 2.0, 3.0, 0.4, 0.5, 0.3}};

  std::array<double, 16> cartesian_pose =
      integrateOneSample(last_cmd_pose, last_cmd_velocity, kDeltaT);
  auto twist = differentiateOneSample(cartesian_pose, last_cmd_pose, kDeltaT);
  for (int i = 0; i < 6; ++i) {
    ASSERT_NEAR(twist[i], last_cmd_velocity[i], 1e-6);
  }
}
