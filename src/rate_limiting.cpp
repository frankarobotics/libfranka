// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/rate_limiting.h>

#include <Eigen/Dense>

#include <franka/control_tools.h>

namespace franka {

namespace {

Eigen::Vector3d limitRate(double max_velocity,
                          double max_acceleration,
                          double max_jerk,
                          const Eigen::Vector3d& commanded_velocity,
                          const Eigen::Vector3d& last_commanded_velocity,
                          const Eigen::Vector3d& last_commanded_acceleration) {
  // Differentiate to get jerk
  Eigen::Vector3d commanded_jerk =
      (((commanded_velocity - last_commanded_velocity) / kDeltaT) - last_commanded_acceleration) /
      kDeltaT;

  // Limit jerk and integrate to get desired acceleration
  Eigen::Vector3d commanded_acceleration = last_commanded_acceleration;
  if (commanded_jerk.norm() > kNormEps) {
    commanded_acceleration += (commanded_jerk / commanded_jerk.norm()) *
                              std::max(std::min(commanded_jerk.norm(), max_jerk), -max_jerk) *
                              kDeltaT;
  }

  // Compute Euclidean distance to the max velocity vector that would be reached starting from
  // last_commanded_velocity with the direction of the desired acceleration
  Eigen::Vector3d unit_commanded_acceleration =
      commanded_acceleration / commanded_acceleration.norm();
  double dot_product = unit_commanded_acceleration.transpose() * last_commanded_velocity;
  double distance_to_max_velocity =
      -dot_product + std::sqrt(pow(dot_product, 2.0) - last_commanded_velocity.squaredNorm() +
                               pow(max_velocity, 2.0));

  // Compute safe acceleration limits
  double safe_max_acceleration =
      std::min((max_jerk / max_acceleration) * distance_to_max_velocity, max_acceleration);

  // Limit acceleration and integrate to get desired velocities
  Eigen::Vector3d limited_commanded_velocity = last_commanded_velocity;

  if (commanded_acceleration.norm() > kNormEps) {
    limited_commanded_velocity += unit_commanded_acceleration *
                                  std::min(commanded_acceleration.norm(), safe_max_acceleration) *
                                  kDeltaT;
  }

  return limited_commanded_velocity;
}

}  // anonymous namespace

std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& commanded_values,
                                const std::array<double, 7>& last_commanded_values) {
  if (!std::all_of(commanded_values.begin(), commanded_values.end(),
                   [](double commanded_value) { return std::isfinite(commanded_value); })) {
    throw std::invalid_argument("Commanding value is infinite or NaN.");
  }
  std::array<double, 7> limited_values{};
  for (size_t i = 0; i < 7; i++) {
    double commanded_derivative = (commanded_values[i] - last_commanded_values[i]) / kDeltaT;
    limited_values[i] =
        last_commanded_values[i] +
        std::max(std::min(commanded_derivative, max_derivatives[i]), -max_derivatives[i]) * kDeltaT;
  }
  return limited_values;
}

double limitRate(double upper_limits_velocity,
                 double lower_limits_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double commanded_velocity,
                 double last_commanded_velocity,
                 double last_commanded_acceleration) {
  if (!std::isfinite(commanded_velocity)) {
    throw std::invalid_argument("commanded_velocity is infinite or NaN.");
  }
  // Differentiate to get jerk
  double commanded_jerk =
      (((commanded_velocity - last_commanded_velocity) / kDeltaT) - last_commanded_acceleration) /
      kDeltaT;

  // Limit jerk and integrate to get acceleration
  double commanded_acceleration = last_commanded_acceleration +
                                  std::max(std::min(commanded_jerk, max_jerk), -max_jerk) * kDeltaT;

  // Compute acceleration limits
  double safe_max_acceleration =
      std::min((max_jerk / max_acceleration) * (upper_limits_velocity - last_commanded_velocity),
               max_acceleration);
  double safe_min_acceleration =
      std::max((max_jerk / max_acceleration) * (lower_limits_velocity - last_commanded_velocity),
               -max_acceleration);

  // Limit acceleration and integrate to get desired velocities
  return last_commanded_velocity +
         std::max(std::min(commanded_acceleration, safe_max_acceleration), safe_min_acceleration) *
             kDeltaT;
}

double limitRate(double upper_limits_velocity,
                 double lower_limits_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double commanded_position,
                 double last_commanded_position,
                 double last_commanded_velocity,
                 double last_commanded_acceleration) {
  if (!std::isfinite(commanded_position)) {
    throw std::invalid_argument("commanded_position is infinite or NaN.");
  }
  return last_commanded_position +
         limitRate(upper_limits_velocity, lower_limits_velocity, max_acceleration, max_jerk,
                   (commanded_position - last_commanded_position) / kDeltaT,
                   last_commanded_velocity, last_commanded_acceleration) *
             kDeltaT;
}

std::array<double, 7> limitRate(const std::array<double, 7>& upper_limits_velocity,
                                const std::array<double, 7>& lower_limits_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_velocities,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations) {
  if (!std::all_of(commanded_velocities.begin(), commanded_velocities.end(),
                   [](double commanded_value) { return std::isfinite(commanded_value); })) {
    throw std::invalid_argument("commanded_velocities is infinite or NaN.");
  }
  std::array<double, 7> limited_commanded_velocities{};

  for (size_t i = 0; i < 7; i++) {
    limited_commanded_velocities[i] = limitRate(
        upper_limits_velocity[i], lower_limits_velocity[i], max_acceleration[i], max_jerk[i],
        commanded_velocities[i], last_commanded_velocities[i], last_commanded_accelerations[i]);
  }
  return limited_commanded_velocities;
}

std::array<double, 7> limitRate(const std::array<double, 7>& upper_limits_velocity,
                                const std::array<double, 7>& lower_limits_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_positions,
                                const std::array<double, 7>& last_commanded_positions,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations) {
  if (!std::all_of(commanded_positions.begin(), commanded_positions.end(),
                   [](double commanded_value) { return std::isfinite(commanded_value); })) {
    throw std::invalid_argument("commanded_positions is infinite or NaN.");
  }
  std::array<double, 7> limited_commanded_positions{};
  for (size_t i = 0; i < 7; i++) {
    limited_commanded_positions[i] =
        limitRate(upper_limits_velocity[i], lower_limits_velocity[i], max_acceleration[i],
                  max_jerk[i], commanded_positions[i], last_commanded_positions[i],
                  last_commanded_velocities[i], last_commanded_accelerations[i]);
  }
  return limited_commanded_positions;
}

std::array<double, 6> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 6>& O_dP_EE_c,          // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_c,     // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_c) {  // NOLINT(readability-identifier-naming)
  if (!std::all_of(O_dP_EE_c.begin(), O_dP_EE_c.end(),
                   [](double commanded_value) { return std::isfinite(commanded_value); })) {
    throw std::invalid_argument("O_dP_EE_c is infinite or NaN.");
  }
  Eigen::Matrix<double, 6, 1> dx(O_dP_EE_c.data());  // NOLINT(readability-identifier-length)
  Eigen::Matrix<double, 6, 1> last_dx(last_O_dP_EE_c.data());
  Eigen::Matrix<double, 6, 1> last_ddx(last_O_ddP_EE_c.data());

  dx.head(3) << limitRate(max_translational_velocity, max_translational_acceleration,
                          max_translational_jerk, dx.head(3), last_dx.head(3), last_ddx.head(3));
  dx.tail(3) << limitRate(max_rotational_velocity, max_rotational_acceleration, max_rotational_jerk,
                          dx.tail(3), last_dx.tail(3), last_ddx.tail(3));

  std::array<double, 6> limited_values{};
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(limited_values.data(), 6, 1) = dx;
  return limited_values;
}

std::array<double, 16> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 16>& O_T_EE_c,          // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& last_O_T_EE_c,     // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_c,     // NOLINT(readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_c) {  // NOLINT(readability-identifier-naming)
  if (!std::all_of(O_T_EE_c.begin(), O_T_EE_c.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("O_T_EE_c is infinite or NaN.");
  }
  if (!isHomogeneousTransformation(O_T_EE_c)) {
    throw std::invalid_argument(
        "O_T_EE_c is invalid transformation matrix. Has to be column major!");
  }
  Eigen::Matrix<double, 6, 1> dx;  // NOLINT(readability-identifier-length)
  Eigen::Affine3d commanded_pose(Eigen::Matrix4d::Map(O_T_EE_c.data()));
  Eigen::Affine3d limited_commanded_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_commanded_pose(Eigen::Matrix4d::Map(last_O_T_EE_c.data()));

  // Compute translational velocity
  dx.head(3) << (commanded_pose.translation() - last_commanded_pose.translation()) / kDeltaT;

  // Compute rotational velocity
  Eigen::AngleAxisd rot_difference(commanded_pose.rotation() *
                                   last_commanded_pose.rotation().transpose());
  dx.tail(3) << rot_difference.axis() * rot_difference.angle() / kDeltaT;

  // Limit the rate of the twist
  std::array<double, 6> commanded_O_dP_EE_c{};  // NOLINT(readability-identifier-naming)
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(commanded_O_dP_EE_c.data(), 6, 1) = dx;
  commanded_O_dP_EE_c =
      limitRate(max_translational_velocity, max_translational_acceleration, max_translational_jerk,
                kFactorCartesianRotationPoseInterface * max_rotational_velocity,
                kFactorCartesianRotationPoseInterface * max_rotational_acceleration,
                kFactorCartesianRotationPoseInterface * max_rotational_jerk, commanded_O_dP_EE_c,
                last_O_dP_EE_c, last_O_ddP_EE_c);
  dx = Eigen::Matrix<double, 6, 1>(commanded_O_dP_EE_c.data());

  // Integrate limited twist
  limited_commanded_pose.translation() << last_commanded_pose.translation() + dx.head(3) * kDeltaT;
  limited_commanded_pose.linear() << last_commanded_pose.rotation();
  if (dx.tail(3).norm() > kNormEps) {
    Eigen::Matrix3d omega_skew;
    Eigen::Vector3d w_norm(dx.tail(3) / dx.tail(3).norm());
    double theta = kDeltaT * dx.tail(3).norm();
    omega_skew << 0, -w_norm(2), w_norm(1), w_norm(2), 0, -w_norm(0), -w_norm(1), w_norm(0), 0;
    // NOLINTNEXTLINE(readability-identifier-naming)
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() + sin(theta) * omega_skew +
                               (1.0 - cos(theta)) * (omega_skew * omega_skew);
    limited_commanded_pose.linear() << rotation * last_commanded_pose.rotation();
  }

  std::array<double, 16> limited_values{};
  Eigen::Map<Eigen::Matrix4d>(limited_values.data(), 4, 4) = limited_commanded_pose.matrix();
  return limited_values;
}

}  // namespace franka
