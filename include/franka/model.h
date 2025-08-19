// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>

#include <franka/robot.h>
#include <franka/robot_model_base.h>
#include <franka/robot_state.h>

/**
 * @file model.h
 * Contains model library types.
 */

namespace franka {

/**
 * Enumerates the seven joints, the flange, and the end effector of a robot.
 */
enum class Frame {
  kJoint1,
  kJoint2,
  kJoint3,
  kJoint4,
  kJoint5,
  kJoint6,
  kJoint7,
  kFlange,
  kEndEffector,
  kStiffness
};

/**
 * Post-increments the given Frame by one.
 *
 * For example, Frame::kJoint2++ results in Frame::kJoint3.
 *
 * @param[in] frame Frame to increment.
 *
 * @return Original Frame.
 */
Frame operator++(Frame& frame, int /* dummy */) noexcept;

/**
 * Calculates poses of joints and dynamic properties of the robot.
 */
class Model {
 public:
  /**
   * Creates a new Model instance.
   *
   * This constructor is for internal use only.
   *
   * @see Robot::loadModel
   *
   * @param[in] urdf_model The URDF model string from the robot.
   *
   * @throw ModelException if the model cannot be initialized.
   */
  explicit Model(const std::string& urdf_model);

  /**
   * Creates a new Model instance only for the tests.
   *
   * This constructor is for the unittests for enabling mocks.
   *
   * @param[in] robot_model unique pointer to the mocked robot_model
   *
   */
  explicit Model(std::unique_ptr<RobotModelBase> robot_model);

  /**
   * Move-constructs a new Model instance.
   *
   * @param[in] model Other Model instance.
   */
  Model(Model&& model) noexcept;

  /**
   * Move-assigns this Model from another Model instance.
   *
   * @param[in] model Other Model instance.
   *
   * @return Model instance.
   */
  Model& operator=(Model&& model) noexcept;

  /**
   * Unloads the model library.
   */
  ~Model() noexcept;

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  std::array<double, 16> pose(Frame frame, const franka::RobotState& robot_state) const;

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  std::array<double, 16> pose(
      Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const;

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> bodyJacobian(Frame frame, const franka::RobotState& robot_state) const;

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> bodyJacobian(
      Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const;

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> zeroJacobian(Frame frame, const franka::RobotState& robot_state) const;

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> zeroJacobian(
      Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const;

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] robot_state State from which the mass matrix should be calculated.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  std::array<double, 49> mass(const franka::RobotState& robot_state) const noexcept;

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  std::array<double, 49> mass(
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept;

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the Coriolis force vector should be calculated.
   *
   * @return Coriolis force vector.
   */
  std::array<double, 7> coriolis(const franka::RobotState& robot_state) const noexcept;

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @deprecated This overload is deprecated. Use coriolis(q, dq, i_total, m_total, f_x_ctotal,
   * g_earth) for better performance.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Coriolis force vector.
   */
  [[deprecated(
      "Use coriolis(q, dq, i_total, m_total, f_x_ctotal, g_earth) instead")]] std::array<double, 7>
  coriolis(const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
           const std::array<double, 7>& dq,       // NOLINT(readability-identifier-length)
           const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
           double m_total,
           const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept;

  /**
   * Calculates the Coriolis force vector with gravity (faster implementation).
   * Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total
   load.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Coriolis force vector.
   */
  std::array<double, 7> coriolis(
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 7>& dq,       // NOLINT(readability-identifier-length)
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth) const noexcept;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * Default to {0.0, 0.0, -9.81}.
   *
   * @return Gravity vector.
   */
  std::array<double, 7> gravity(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const noexcept;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Gravity vector.
   */
  std::array<double, 7> gravity(const franka::RobotState& robot_state,
                                const std::array<double, 3>& gravity_earth) const noexcept;

  /**
   * Calculates the gravity vector using the robot state. Unit: \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
   *
   * @return Gravity vector.
   */
  std::array<double, 7> gravity(const franka::RobotState& robot_state) const noexcept;

  /// @cond DO_NOT_DOCUMENT
  Model(const Model&) = delete;
  Model& operator=(const Model&) = delete;
  /// @endcond

 private:
  std::unique_ptr<RobotModelBase> robot_model_;
};

}  // namespace franka
