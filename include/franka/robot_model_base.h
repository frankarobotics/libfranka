// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

/**
 * @file robot_model_base.h
 * Abstract interface class for robot dynamic parameters computed from the URDF model with
 * Pinocchio.
 */

/**
 * @brief Robot dynamic parameters computed from the URDF model with Pinocchio.
 */
class RobotModelBase {
 public:
  virtual ~RobotModelBase() = default;
  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] i_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] f_x_ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[out] c_ne Coriolis force vector. Unit: \f$[Nm]\f$.
   */
  virtual void coriolis(const std::array<double, 7>& q,   // NOLINT(readability-identifier-length)
                        const std::array<double, 7>& dq,  // NOLINT(readability-identifier-length)
                        const std::array<double, 9>& i_total,
                        double m_total,
                        const std::array<double, 3>& f_x_ctotal,
                        std::array<double, 7>& c_ne) = 0;

  /**
   * Calculates the Coriolis force vector with configurable gravity (recommended implementation).
   * Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] i_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] f_x_ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[in] g_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * @param[out] c_ne Coriolis force vector. Unit: \f$[Nm]\f$.
   */
  virtual void coriolis(const std::array<double, 7>& q,   // NOLINT(readability-identifier-length)
                        const std::array<double, 7>& dq,  // NOLINT(readability-identifier-length)
                        const std::array<double, 9>& i_total,
                        double m_total,
                        const std::array<double, 3>& f_x_ctotal,
                        const std::array<double, 3>& g_earth,
                        std::array<double, 7>& c_ne) = 0;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   *                    Unit: \f$[kg]\f$.
   * @param[in] f_x_Ctotal Translation from flange to center of mass of the attached total load.
   * @param[out] g_ne Gravity vector. Unit: \f$[Nm]\f$.
   */
  virtual void gravity(const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
                       const std::array<double, 3>& g_earth,
                       double m_total,
                       const std::array<double, 3>& f_x_ctotal,
                       std::array<double, 7>& g_ne) = 0;

  /**
   * Calculates the 7x7 inertia matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] i_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] f_x_ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[out] m_ne Vectorized 7x7 inertia matrix, column-major.
   */
  virtual void mass(const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
                    const std::array<double, 9>& i_total,
                    double m_total,
                    const std::array<double, 3>& f_x_ctotal,
                    std::array<double, 49>& m_ne) = 0;

  /**
   * Calculates the homogeneous transformation matrix for the specified joint or link.
   *
   * @param[in] q Joint position.
   * @param[in] joint_index The index of the joint/link for which to calculate the pose.
   * @return Homogeneous transformation matrix (4x4) in column-major order.
   */
  virtual std::array<double, 16> pose(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      int joint_index) = 0;

  /**
   * Calculates the homogeneous transformation matrix for the end effector, applying
   * the end effector transformation.
   *
   * @param[in] q Joint position.
   * @param[in] f_t_ee The transformation from flange to end effector (4x4 matrix, column-major).
   * @return Homogeneous transformation matrix (4x4) in column-major order.
   */
  virtual std::array<double, 16> poseEe(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee) = 0;

  /**
   * Calculates the homogeneous transformation matrix for the flange.
   *
   * @param[in] q Joint position.
   * @return Homogeneous transformation matrix (4x4) in column-major order.
   */
  virtual std::array<double, 16> poseFlange(
      const std::array<double, 7>& q) = 0;  // NOLINT(readability-identifier-length)

  /**
   * Calculates the homogeneous transformation matrix for the stiffness frame.
   *
   * @param[in] q Joint position.
   * @param[in] f_t_ee The transformation from flange to end effector (4x4 matrix, column-major).
   * @param[in] ee_t_k The transformation from end effector to stiffness frame (4x4 matrix,
   * column-major).
   * @return Homogeneous transformation matrix (4x4) in column-major order.
   */
  virtual std::array<double, 16> poseStiffness(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      const std::array<double, 16>& ee_t_k) = 0;
  /**
   * Calculates the 6x7 body Jacobian for the given joint, relative to that joint's frame.
   *
   * @param[in] q Joint position.
   * @param[in] joint_index The index of the joint/link for which to calculate the Jacobian.
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> bodyJacobian(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      int joint_index) = 0;

  /**
   * Calculates the 6x7 body Jacobian for the flange frame, relative to that frame.
   *
   * @param[in] q Joint position.
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> bodyJacobianFlange(
      const std::array<double, 7>& q) = 0;  // NOLINT(readability-identifier-length)

  /**
   * Calculates the 6x7 body Jacobian for the end effector frame, relative to that frame.
   *
   * @param[in] q Joint position.
   * @param[in] f_t_ee The transformation from flange to end effector (4x4 matrix, column-major).
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> bodyJacobianEe(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee) = 0;

  /**
   * Calculates the 6x7 body Jacobian for the stiffness frame, relative to that frame.
   *
   * @param[in] q Joint position.
   * @param[in] f_t_ee The transformation from flange to end effector (4x4 matrix, column-major).
   * @param[in] ee_t_k The transformation from end effector to stiffness frame (4x4 matrix,
   * column-major).
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> bodyJacobianStiffness(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      const std::array<double, 16>& ee_t_k) = 0;

  /**
   * Calculates the 6x7 zero Jacobian for the given joint, relative to the base frame.
   *
   * @param[in] q Joint position.
   * @param[in] joint_index The index of the joint/link for which to calculate the Jacobian.
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> zeroJacobian(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      int joint_index) = 0;

  /**
   * Calculates the 6x7 zero Jacobian for the flange frame, relative to the base frame.
   *
   * @param[in] q Joint position.
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> zeroJacobianFlange(
      const std::array<double, 7>& q) = 0;  // NOLINT(readability-identifier-length)

  /**
   * Calculates the 6x7 zero Jacobian for the end effector frame, relative to the base frame.
   *
   * @param[in] q Joint position.
   * @param[in] f_t_ee The transformation from flange to end effector (4x4 matrix, column-major).
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> zeroJacobianEe(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee) = 0;

  /**
   * Calculates the 6x7 zero Jacobian for the stiffness frame, relative to the base frame.
   *
   * @param[in] q Joint position.
   * @param[in] f_t_ee The transformation from flange to end effector (4x4 matrix, column-major).
   * @param[in] ee_t_k The transformation from end effector to stiffness frame (4x4 matrix,
   * column-major).
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> zeroJacobianStiffness(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      const std::array<double, 16>& f_t_ee,
      const std::array<double, 16>& ee_t_k) = 0;
};
