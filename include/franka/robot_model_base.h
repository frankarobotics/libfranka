// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

/**
 *  @brief Robot dynamic parameters computed from the URDF model with Pinocchio.
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
  virtual void coriolis(const std::array<double, 7>& q,
                        const std::array<double, 7>& dq,
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
  virtual void coriolis(const std::array<double, 7>& q,
                        const std::array<double, 7>& dq,
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
  virtual void gravity(const std::array<double, 7>& q,
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
  virtual void mass(const std::array<double, 7>& q,
                    const std::array<double, 9>& i_total,
                    double m_total,
                    const std::array<double, 3>& f_x_ctotal,
                    std::array<double, 49>& m_ne) = 0;
};