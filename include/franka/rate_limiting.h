// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

/**
 * @file rate_limiting.h
 * Contains functions for limiting the rate of torques, Cartesian pose, Cartesian velocity,
 * joint position and joint velocity.
 */

namespace franka {
/**
 * Sample time constant
 */
constexpr double kDeltaT = 1e-3;
/**
 * Epsilon value for checking limits
 */
constexpr double kLimitEps = 1e-3;
/**
 * Epsilon value for limiting Cartesian accelerations/jerks or not
 */
constexpr double kNormEps = std::numeric_limits<double>::epsilon();
/**
 * Number of packets lost considered for the definition of velocity limits.
 * When a packet is lost, FCI assumes a constant acceleration model.
 * For FR3 there are no expected package loses. Therefore this number is set to 0. If you
 * encounter package loses with your setup you can increase this number
 */
constexpr double kTolNumberPacketsLost = 0.0;
/**
 * Factor for the definition of rotational limits using the Cartesian Pose interface
 */
constexpr double kFactorCartesianRotationPoseInterface = 0.99;
/**
 * Maximum torque rate
 */
constexpr std::array<double, 7> kMaxTorqueRate{
    {1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps, 1000 - kLimitEps,
     1000 - kLimitEps, 1000 - kLimitEps}};
/**
 * Maximum joint jerk
 */
constexpr std::array<double, 7> kMaxJointJerk{
    {5000.0 - kLimitEps, 5000.0 - kLimitEps, 5000.0 - kLimitEps, 5000.0 - kLimitEps,
     5000.0 - kLimitEps, 5000.0 - kLimitEps, 5000.0 - kLimitEps}};
/**
 * Maximum joint acceleration
 */
constexpr std::array<double, 7> kMaxJointAcceleration{
    {10.0000 - kLimitEps, 10.0000 - kLimitEps, 10.0000 - kLimitEps, 10.0000 - kLimitEps,
     10.0000 - kLimitEps, 10.0000 - kLimitEps, 10.0000 - kLimitEps}};
/**
 * Tolerance value for joint velocity limits to deal with numerical errors and data losses.
 */
constexpr std::array<double, 7> kJointVelocityLimitsTolerance{
    kLimitEps + kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[0],
    kLimitEps + kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[1],
    kLimitEps + kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[2],
    kLimitEps + kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[3],
    kLimitEps + kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[4],
    kLimitEps + kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[5],
    kLimitEps + kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[6],
};
/**
 * Maximum translational jerk
 */
constexpr double kMaxTranslationalJerk = 4500.0 - kLimitEps;
/**
 * Maximum translational acceleration
 */
constexpr double kMaxTranslationalAcceleration = 9.0000 - kLimitEps;
/**
 * Maximum translational velocity
 */
constexpr double kMaxTranslationalVelocity =
    3.0000 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxTranslationalAcceleration;
/**
 * Maximum rotational jerk
 */
constexpr double kMaxRotationalJerk = 8500.0 - kLimitEps;
/**
 * Maximum rotational acceleration
 */
constexpr double kMaxRotationalAcceleration = 17.0000 - kLimitEps;
/**
 * Maximum rotational velocity
 */
constexpr double kMaxRotationalVelocity =
    2.5000 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxRotationalAcceleration;
/**
 * Maximum elbow jerk
 */
constexpr double kMaxElbowJerk = 5000 - kLimitEps;
/**
 * Maximum elbow acceleration
 */
constexpr double kMaxElbowAcceleration = 10.0000 - kLimitEps;

/**
 * Maximum elbow velocity
 */
constexpr double kMaxElbowVelocity =
    1.5000 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxElbowAcceleration;

/**
 * Minimum elbow velocity
 */
constexpr double kMinElbowVelocity = -kMaxElbowVelocity;

/**
 * Computes the maximum joint velocity based on joint position
 *
 * @note The implementation is based on
 * https://frankarobotics.github.io/docs/control_parameters.html#limits-for-franka-research-3.
 *
 * @param[in] q joint position.
 *
 * @return Upper limits of joint velocity at the given joint position.
 *
 * @deprecated This function is deprecated and will be removed in future versions.
 * Use `Robot::getUpperJointVelocityLimits(const std::array<double, 7UL> &joint_positions)` instead
 * (if applicable).
 */
[[deprecated(
    "Use Robot::getUpperJointVelocityLimits(const std::array<double, 7UL> &joint_positions) "
    "instead with system images >= 5.9.0.")]] inline std::array<double, 7>
computeUpperLimitsJointVelocity(
    const std::array<double, 7>& q) {  // NOLINT(readability-identifier-length)
  return std::array<double, 7>{
      std::min(2.62, std::max(0.0, -0.30 + std::sqrt(std::max(0.0, 12.0 * (2.75010 - q[0]))))) -
          kJointVelocityLimitsTolerance[0],
      std::min(2.62, std::max(0.0, -0.20 + std::sqrt(std::max(0.0, 5.17 * (1.79180 - q[1]))))) -
          kJointVelocityLimitsTolerance[1],
      std::min(2.62, std::max(0.0, -0.20 + std::sqrt(std::max(0.0, 7.00 * (2.90650 - q[2]))))) -
          kJointVelocityLimitsTolerance[2],
      std::min(2.62, std::max(0.0, -0.30 + std::sqrt(std::max(0.0, 8.00 * (-0.1458 - q[3]))))) -
          kJointVelocityLimitsTolerance[3],
      std::min(5.26, std::max(0.0, -0.35 + std::sqrt(std::max(0.0, 34.0 * (2.81010 - q[4]))))) -
          kJointVelocityLimitsTolerance[4],
      std::min(4.18, std::max(0.0, -0.35 + std::sqrt(std::max(0.0, 11.0 * (4.52050 - q[5]))))) -
          kJointVelocityLimitsTolerance[5],
      std::min(5.26, std::max(0.0, -0.35 + std::sqrt(std::max(0.0, 34.0 * (3.01960 - q[6]))))) -
          kJointVelocityLimitsTolerance[6],
  };
}

/**
 * Computes the minimum joint velocity based on joint position
 *
 * @note The implementation is based on
 * https://frankarobotics.github.io/docs/control_parameters.html#limits-for-franka-research-3.
 *
 * @param[in] q joint position.
 *
 * @return Lower limits of joint velocity at the given joint position.
 *
 * @deprecated This function is deprecated and will be removed in future versions.
 * Use `Robot::getlowerJointVelocityLimits(const std::array<double, 7UL> &joint_positions)` instead
 * (if applicable).
 */
[[deprecated(
    "Use Robot::getlowerJointVelocityLimits(const std::array<double, 7UL> &joint_positions) "
    "instead with system images >= 5.9.0.")]] inline std::array<double, 7>
computeLowerLimitsJointVelocity(
    const std::array<double, 7>& q) {  // NOLINT(readability-identifier-length)
  return std::array<double, 7>{
      std::max(-2.62, std::min(0.0, 0.30 - std::sqrt(std::max(0.0, 12.0 * (2.750100 + q[0]))))) +
          kJointVelocityLimitsTolerance[0],
      std::max(-2.62, std::min(0.0, 0.20 - std::sqrt(std::max(0.0, 5.17 * (1.791800 + q[1]))))) +
          kJointVelocityLimitsTolerance[1],
      std::max(-2.62, std::min(0.0, 0.20 - std::sqrt(std::max(0.0, 7.00 * (2.906500 + q[2]))))) +
          kJointVelocityLimitsTolerance[2],
      std::max(-2.62, std::min(0.0, 0.30 - std::sqrt(std::max(0.0, 8.00 * (3.048100 + q[3]))))) +
          kJointVelocityLimitsTolerance[3],
      std::max(-5.26, std::min(0.0, 0.35 - std::sqrt(std::max(0.0, 34.0 * (2.810100 + q[4]))))) +
          kJointVelocityLimitsTolerance[4],
      std::max(-4.18, std::min(0.0, 0.35 - std::sqrt(std::max(0.0, 11.0 * (-0.54092 + q[5]))))) +
          kJointVelocityLimitsTolerance[5],
      std::max(-5.26, std::min(0.0, 0.35 - std::sqrt(std::max(0.0, 34.0 * (3.019600 + q[6]))))) +
          kJointVelocityLimitsTolerance[6],
  };
}

/**
 * Limits the rate of an input vector of per-joint commands considering the maximum allowed
 * time derivatives.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_derivatives Per-joint maximum allowed time derivative.
 * @param[in] commanded_values Commanded values of the current time step.
 * @param[in] last_commanded_values Commanded values of the previous time step.
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 * @return Rate-limited vector of desired values.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& commanded_values,
                                const std::array<double, 7>& last_commanded_values);

/**
 * Limits the rate of a desired joint velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] upper_limits_velocity Upper limits of allowed velocity.
 * @param[in] lower_limits_velocity Lower limits of allowed velocity.
 * @param[in] max_acceleration Maximum allowed acceleration.
 * @param[in] max_jerk Maximum allowed jerk.
 * @param[in] commanded_velocity Commanded joint velocity of the current time step.
 * @param[in] last_commanded_velocity Commanded joint velocity of the previous time step.
 * @param[in] last_commanded_acceleration Commanded joint acceleration of the previous time step.
 *
 * @throw std::invalid_argument if commanded_velocity is infinite or NaN.
 *
 * @return Rate-limited desired joint velocity.
 */
double limitRate(double upper_limits_velocity,
                 double lower_limits_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double commanded_velocity,
                 double last_commanded_velocity,
                 double last_commanded_acceleration);

/**
 * Limits the rate of a desired joint position considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] upper_limits_velocity Upper limits of allowed velocity.
 * @param[in] lower_limits_velocity Lower limits of allowed velocity.
 * @param[in] max_acceleration Maximum allowed acceleration.
 * @param[in] max_jerk Maximum allowed jerk.
 * @param[in] commanded_position Commanded joint position of the current time step.
 * @param[in] last_commanded_position Commanded joint position of the previous time step.
 * @param[in] last_commanded_velocity Commanded joint velocity of the previous time step.
 * @param[in] last_commanded_acceleration Commanded joint acceleration of the previous time step.
 *
 * @throw std::invalid_argument if commanded_position is infinite or NaN.
 *
 * @return Rate-limited desired joint position.
 */
double limitRate(double upper_limits_velocity,
                 double lower_limits_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double commanded_position,
                 double last_commanded_position,
                 double last_commanded_velocity,
                 double last_commanded_acceleration);

/**
 * Limits the rate of a desired joint velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] upper_limits_velocity Per-joint upper limits of allowed velocity.
 * @param[in] lower_limits_velocity Per-joint lower limits of allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] max_jerk Per-joint maximum allowed jerk.
 * @param[in] commanded_velocities Commanded joint velocity of the current time step.
 * @param[in] last_commanded_velocities Commanded joint velocities of the previous time step.
 * @param[in] last_commanded_accelerations Commanded joint accelerations of the previous time step.
 *
 * @throw std::invalid_argument if commanded_velocities are infinite or NaN.
 *
 * @return Rate-limited vector of desired joint velocities.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& upper_limits_velocity,
                                const std::array<double, 7>& lower_limits_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_velocities,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations);

/**
 * Limits the rate of a desired joint position considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] upper_limits_velocity Per-joint upper limits of allowed velocity.
 * @param[in] lower_limits_velocity Per-joint lower limits of allowed velocity.
 * @param[in] max_acceleration Per-joint maximum allowed acceleration.
 * @param[in] max_jerk Per-joint maximum allowed jerk.
 * @param[in] commanded_positions Commanded joint positions of the current time step.
 * @param[in] last_commanded_positions Commanded joint positions of the previous time step.
 * @param[in] last_commanded_velocities Commanded joint velocities of the previous time step.
 * @param[in] last_commanded_accelerations Commanded joint accelerations of the previous time step.
 *
 * @throw std::invalid_argument if commanded_positions are infinite or NaN.
 *
 * @return Rate-limited vector of desired joint positions.
 */
std::array<double, 7> limitRate(const std::array<double, 7>& upper_limits_velocity,
                                const std::array<double, 7>& lower_limits_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_positions,
                                const std::array<double, 7>& last_commanded_positions,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations);

/**
 * Limits the rate of a desired Cartesian velocity considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_translational_velocity Maximum translational velocity.
 * @param[in] max_translational_acceleration Maximum translational acceleration.
 * @param[in] max_translational_jerk Maximum translational jerk.
 * @param[in] max_rotational_velocity Maximum rotational velocity.
 * @param[in] max_rotational_acceleration Maximum rotational acceleration.
 * @param[in] max_rotational_jerk Maximum rotational jerk.
 * @param[in] O_dP_EE_c Commanded end effector twist of the current time step.
 * @param[in] last_O_dP_EE_c Commanded end effector twist of the previous time step.
 * @param[in] last_O_ddP_EE_c Commanded end effector acceleration of the previous time step.
 *
 * @throw std::invalid_argument if an element of O_dP_EE_c is infinite or NaN.
 *
 * @return Rate-limited desired end effector twist.
 */
std::array<double, 6> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 6>& O_dP_EE_c,         // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_c,    // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_c);  // NOLINT(readability-identifier-naming)

/**
 * Limits the rate of a desired Cartesian pose considering the limits provided.
 *
 * @note
 * FCI filters must be deactivated to work properly.
 *
 * @param[in] max_translational_velocity Maximum translational velocity.
 * @param[in] max_translational_acceleration Maximum translational acceleration.
 * @param[in] max_translational_jerk Maximum translational jerk.
 * @param[in] max_rotational_velocity Maximum rotational velocity.
 * @param[in] max_rotational_acceleration Maximum rotational acceleration.
 * @param[in] max_rotational_jerk Maximum rotational jerk.
 * @param[in] O_T_EE_c Commanded pose of the current time step.
 * @param[in] last_O_T_EE_c Commanded pose of the previous time step.
 * @param[in] last_O_dP_EE_c Commanded end effector twist of the previous time step.
 * @param[in] last_O_ddP_EE_c Commanded end effector acceleration of the previous time step.
 *
 * @throw std::invalid_argument if an element of O_T_EE_c is infinite or NaN.
 *
 * @return Rate-limited desired pose.
 */
std::array<double, 16> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 16>& O_T_EE_c,         // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& last_O_T_EE_c,    // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_c,    // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_c);  // NOLINT(readability-identifier-naming)

}  // namespace franka
